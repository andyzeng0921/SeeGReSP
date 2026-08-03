"""Navigation model interfaces and adapters.

Primary adapter:  Qwen-RobotNav (Qwen3-VL backbone, 2B/4B/8B).
Secondary:        RoboStral Navigate (Mistral, 8B).

Qwen-RobotNav treats navigation as context modelling: five task modes share
one perception-planning backbone but differ in how they consume visual history.
The model exposes a *controllable observation protocol* with four axes (token
budget, temporal decay, camera weights, frame sample mode) that an upper-level
planner can tune per call.

Backends:
  - ``GPUStackBackend`` — real inference via GPUStack API (qwen3-vl-8b-instruct
    + qwen3.5-35b-a3b), used when ``api_url`` is configured.
  - Placeholder — deterministic straight-ahead trajectory for integration tests.

Reference:
  - Qwen-RobotNav: https://qwen.ai/blog?id=qwen-robotnav
  - RoboStral Navigate: https://mistral.ai/news/robostral-navigate/
"""

from __future__ import annotations

import base64
import io
import json
import logging
import math
import os
import threading
import time
import urllib.error
import urllib.request
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

_logger = logging.getLogger(__name__)


# ============================================================================
# Shared data types
# ============================================================================

@dataclass(frozen=True)
class NavigationCommand:
    """Unified output from any navigation model.

    - *image_space*: robot should move toward pixel (u, v) in the current
      camera frame.
    - *metric*: body-frame displacement (dx_m, dy_m, yaw_rad) when the
      target is outside the field of view.
    """

    image_space: bool
    u: float = 0.0
    v: float = 0.0
    dx_m: float = 0.0
    dy_m: float = 0.0
    yaw_rad: float = 0.0
    done: bool = False


@dataclass(frozen=True)
class Waypoint:
    """Single waypoint in robot body frame — Qwen-RobotNav output atom."""

    x_m: float   # forward  displacement (body-frame X)
    y_m: float   # lateral  displacement (body-frame Y)
    theta_rad: float  # desired heading at waypoint


# ============================================================================
# Qwen-RobotNav – task modes
# ============================================================================

class TaskMode(str, Enum):
    VLN = "vln"            # Vision-Language Navigation (instruction following)
    POINTNAV = "pointnav"  # Navigate to a 2D point goal
    OBJNAV = "objnav"      # Object-goal navigation (e.g. "find the red ball")
    TRACKING = "tracking"  # Active visual target tracking
    DRIVING = "driving"    # Autonomous driving (NAVSIM-style)


# ============================================================================
# Controllable observation protocol (Qwen-RobotNav)
# ============================================================================

@dataclass
class ObservationConfig:
    """Inference-time controls for how Qwen-RobotNav consumes visual history.

    These four axes are randomised during training so the model generalises to
    any configuration at inference time without retraining.

    Attributes:
        token_budget: Total visual tokens shared across all cameras and
            timesteps.  Larger budgets improve long-horizon recall but cost
            compute.  Recommended range: 2048 – 4608.
        temporal_decay: How strongly recent frames are favoured over older
            ones (higher = more recency bias).  Set gamma >= 2.0 for
            instruction following, gamma < 1.0 for tracking.  Range 0.5–3.5.
        camera_weights: Per-camera importance.  Keys are viewpoint labels
            used in the natural-language observation tags (e.g. "Front View",
            "Front Right View").  Higher weight → more tokens allocated.
        frame_sample_mode: "random" for broad history coverage (VLN,
            object search) or "latest" for tight recency (tracking).
    """

    token_budget: int = 3072
    temporal_decay: float = 2.0
    camera_weights: dict[str, float] = field(default_factory=lambda: {
        "Front View": 1.0,
    })
    frame_sample_mode: str = "random"  # "random" | "latest"

    def __post_init__(self):
        if self.frame_sample_mode not in ("random", "latest"):
            raise ValueError(
                f"frame_sample_mode must be 'random' or 'latest', "
                f"got {self.frame_sample_mode!r}"
            )
        if self.token_budget < 128:
            raise ValueError(f"token_budget must be >= 128, got {self.token_budget}")


# Task-mode → recommended ObservationConfig presets
TASK_OBSERVATION_PRESETS: dict[TaskMode, ObservationConfig] = {
    TaskMode.VLN: ObservationConfig(
        token_budget=3584, temporal_decay=2.0,
        frame_sample_mode="random",
    ),
    TaskMode.POINTNAV: ObservationConfig(
        token_budget=2560, temporal_decay=1.5,
        frame_sample_mode="random",
    ),
    TaskMode.OBJNAV: ObservationConfig(
        token_budget=3072, temporal_decay=1.0,
        frame_sample_mode="random",
    ),
    TaskMode.TRACKING: ObservationConfig(
        token_budget=2048, temporal_decay=0.5,
        frame_sample_mode="latest",
    ),
    TaskMode.DRIVING: ObservationConfig(
        token_budget=4096, temporal_decay=2.5,
        frame_sample_mode="latest",
    ),
}


# ============================================================================
# GPUStack API backend — real inference via OpenAI-compatible API
# ============================================================================

class GPUStackBackend:
    """OpenAI-compatible API client for GPUStack-hosted VL and LLM models.

    Used by ``QwenRobotNavAdapter`` when ``api_url`` is configured, replacing
    the deterministic placeholder with real VL+LLM inference.

    Attributes:
        api_url: Base URL of the GPUStack API (e.g. ``http://host:8088/v1``).
        api_key: Bearer token for authentication.
        vl_model: Model ID for vision-language tasks (default ``qwen3-vl-8b-instruct``).
        llm_model: Model ID for text-only reasoning (default ``qwen3.5-35b-a3b``).
        vl_max_tokens: Max completion tokens for VL calls (default 1024).
        llm_max_tokens: Max completion tokens for LLM calls (default 4096,
            higher because this is a reasoning model that thinks before answering).
        timeout: HTTP timeout in seconds.
        max_retries: Number of retries on transient errors.
    """

    def __init__(
        self,
        api_url: str,
        api_key: str,
        vl_model: str = "qwen3-vl-8b-instruct",
        llm_model: str = "qwen3.5-35b-a3b",
        vl_max_tokens: int = 1024,
        llm_max_tokens: int = 4096,
        timeout: float = 120.0,
        max_retries: int = 2,
    ):
        if not api_url:
            raise ValueError("api_url is required for GPUStackBackend")
        self._api_url = api_url.rstrip("/")
        self._api_key = api_key
        self._vl_model = vl_model
        self._llm_model = llm_model
        self._vl_max_tokens = vl_max_tokens
        self._llm_max_tokens = llm_max_tokens
        self._timeout = timeout
        self._max_retries = max_retries
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def vl_chat(
        self,
        image: np.ndarray,
        prompt: str,
        max_tokens: Optional[int] = None,
        temperature: float = 0.3,
    ) -> str:
        """Send an image + text prompt to the VL model, return text response.

        Args:
            image: RGB image as (H, W, 3) uint8 numpy array.
            prompt: Natural-language instruction.
            max_tokens: Override default VL token budget.
            temperature: Sampling temperature (0.0 = deterministic).

        Returns:
            Model text response, stripped of leading/trailing whitespace.

        Raises:
            RuntimeError: On API failure after all retries.
        """
        b64 = self._encode_image(image)
        messages = [{
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {
                    "url": f"data:image/jpeg;base64,{b64}"
                }},
            ],
        }]
        return self._call_api(
            self._vl_model, messages,
            max_tokens or self._vl_max_tokens, temperature,
        )

    def llm_chat(
        self,
        system_prompt: str,
        user_prompt: str,
        max_tokens: Optional[int] = None,
        temperature: float = 0.3,
    ) -> str:
        """Send a text-only prompt to the LLM, return text response.

        The ``qwen3.5-35b-a3b`` model is a reasoning model — its output
        first goes through an internal thinking phase, then produces the
        final answer.  Set ``max_tokens`` generously (>= 4096) to avoid
        truncation during the thinking phase.

        Args:
            system_prompt: System-level instruction.
            user_prompt: User message content.
            max_tokens: Override default LLM token budget.
            temperature: Sampling temperature.
        """
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_prompt},
        ]
        return self._call_api(
            self._llm_model, messages,
            max_tokens or self._llm_max_tokens, temperature,
        )

    def validate(self) -> bool:
        """Quick connectivity check: list models and verify auth."""
        try:
            req = urllib.request.Request(
                f"{self._api_url}/models",
                headers={"Authorization": f"Bearer {self._api_key}"},
            )
            with urllib.request.urlopen(req, timeout=15) as resp:
                data = json.loads(resp.read())
                models = [m.get("id", "") for m in data.get("data", [])]
                return self._vl_model in models and self._llm_model in models
        except Exception as exc:
            _logger.warning("GPUStackBackend.validate failed: %s", exc)
            return False

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    @staticmethod
    def _encode_image(image: np.ndarray) -> str:
        """Encode a numpy RGB image to base64 JPEG."""
        try:
            from PIL import Image
        except ImportError:
            import traceback
            raise RuntimeError(
                "Pillow is required for GPUStack VL inference.  "
                "Install it with: pip install Pillow"
            ) from None
        if image.ndim == 2:
            mode = "L"
        elif image.shape[2] == 3:
            mode = "RGB"
        else:
            mode = "RGBA"
        pil_img = Image.fromarray(image, mode)
        buf = io.BytesIO()
        pil_img.save(buf, format="JPEG", quality=85)
        return base64.b64encode(buf.getvalue()).decode("utf-8")

    def _call_api(
        self,
        model: str,
        messages: list[dict],
        max_tokens: int,
        temperature: float,
    ) -> str:
        payload = json.dumps({
            "model": model,
            "messages": messages,
            "max_tokens": max_tokens,
            "temperature": temperature,
        }).encode("utf-8")
        last_error = None
        for attempt in range(self._max_retries + 1):
            try:
                req = urllib.request.Request(
                    f"{self._api_url}/chat/completions",
                    data=payload,
                    headers={
                        "Authorization": f"Bearer {self._api_key}",
                        "Content-Type": "application/json",
                    },
                )
                with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                    result = json.loads(resp.read())
                content = (
                    result["choices"][0]["message"].get("content") or ""
                ).strip()
                if not content:
                    finish = result["choices"][0].get("finish_reason", "?")
                    if finish == "length":
                        raise RuntimeError(
                            f"Model output truncated (token limit {max_tokens}). "
                            "Increase max_tokens for reasoning models."
                        )
                    # Some reasoning models return empty content on first call
                    # if the thinking phase wasn't complete.  That's a hard error.
                    raise RuntimeError(
                        f"Empty model response (finish_reason={finish}). "
                        "The model may need more max_tokens to complete thinking."
                    )
                return content
            except urllib.error.HTTPError as exc:
                body = ""
                try:
                    body = exc.read().decode()[:500]
                except Exception:
                    pass
                last_error = RuntimeError(
                    f"GPUStack HTTP {exc.code}: {body}"
                )
            except urllib.error.URLError as exc:
                last_error = RuntimeError(
                    f"GPUStack connection failed: {exc.reason}"
                )
            except (json.JSONDecodeError, KeyError, IndexError) as exc:
                last_error = RuntimeError(
                    f"GPUStack response parse error: {exc}"
                )
            except RuntimeError:
                raise  # re-raise structured errors (empty content, truncation)
            if attempt < self._max_retries:
                delay = 2.0 ** attempt
                _logger.warning(
                    "GPUStack call attempt %d/%d failed, retrying in %.1fs: %s",
                    attempt + 1, self._max_retries, delay, last_error,
                )
                time.sleep(delay)
        raise last_error  # type: ignore[misc]


# ============================================================================
# Abstract interface
# ============================================================================

class NavigationModel:
    """Abstract interface for embodied navigation models."""

    def predict(
        self,
        image: np.ndarray,
        instruction: str,
        history: Optional[list[np.ndarray]] = None,
    ) -> NavigationCommand:
        raise NotImplementedError

    def reset(self, instruction: str = "") -> None:
        raise NotImplementedError


# ============================================================================
# Qwen-RobotNav adapter  (primary)
# ============================================================================

class QwenRobotNavAdapter:
    """Adapter for Qwen-RobotNav (Qwen3-VL backbone, 2B/4B/8B).

    Architecture (from the Qwen-RobotNav technical report, 2026-06):
      - Qwen3-VL backbone, frozen or LoRA fine-tuned.
      - 4-layer MLP action head that predicts **8 waypoints**.
      - Each waypoint: (x, y, theta) in the current robot body frame.
      - Observation format: natural-language tags interleaved with visual
        tokens — "Time step 0 Front View <image> Front Right View <image> ..."

    **IMPORTANT**: As of 2026-07 the model weights have NOT been publicly
    released (Qwen team states "no current plan to release").  This adapter
    documents the full API contract and provides a placeholder inference
    path for integration testing.  Replace ``_placeholder_infer`` with the
    real backend when weights become available.

    Real-backend integration sketch (vLLM):

        from vllm import LLM, SamplingParams
        llm = LLM(model="Qwen/Qwen-RobotNav-8B")
        outputs = llm.generate({"prompt": prompt, "multi_modal_data": {...}})
        return _parse_waypoints(outputs[0].outputs[0].text)

    or via transformers:

        from transformers import Qwen3VLForConditionalGeneration, AutoProcessor
        model = Qwen3VLForConditionalGeneration.from_pretrained(...)
    """

    # Number of waypoints the model outputs per inference call.
    NUM_WAYPOINTS = 8

    def __init__(
        self,
        model_path: str = "",
        model_size: str = "4B",
        device: str = "cuda:0",
        image_size: tuple[int, int] = (640, 480),
        default_task_mode: TaskMode = TaskMode.VLN,
        api_url: str = "",
        api_key: str = "",
    ):
        self._model_path = model_path
        self._model_size = model_size
        self._device = device
        self._image_size = image_size
        self._default_task_mode = default_task_mode
        self._lock = threading.Lock()
        self._loaded = False
        self._history: list[np.ndarray] = []
        # When api_url is set, use the real GPUStack backend instead of placeholder.
        self._backend: Optional[GPUStackBackend] = None
        if api_url:
            self._backend = GPUStackBackend(
                api_url=api_url,
                api_key=api_key,
                timeout=120.0,
                max_retries=2,
            )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def predict(
        self,
        image: np.ndarray,
        instruction: str,
        task_mode: Optional[TaskMode] = None,
        obs_config: Optional[ObservationConfig] = None,
    ) -> list[Waypoint]:
        """Run one inference step and return up to 8 waypoints.

        Args:
            image: Current RGB frame (H, W, 3), uint8, BGR.
            instruction: Natural-language sub-goal instruction.
            task_mode: Which navigation behaviour to use.  Defaults to
                ``self._default_task_mode`` (set in constructor).
            obs_config: Observation context controls.  If None, the
                recommended preset for ``task_mode`` is used.

        Returns:
            List of Waypoint objects (1–8, ordered from nearest to farthest).
            Returns a single waypoint at origin when the model signals DONE.
        """
        mode = task_mode or self._default_task_mode
        config = obs_config or TASK_OBSERVATION_PRESETS.get(
            mode, ObservationConfig()
        )
        with self._lock:
            if not self._loaded:
                self._load()
            self._history.append(image)
            return self._infer(image, instruction, mode, config)

    def reset(self) -> None:
        """Clear history for a new navigation task."""
        with self._lock:
            self._history.clear()

    # ------------------------------------------------------------------
    # Prompt building
    # ------------------------------------------------------------------

    @staticmethod
    def build_observation_tags(
        history: list[np.ndarray],
        obs_config: ObservationConfig,
    ) -> list[str]:
        """Build natural-language observation tags for the prompt.

        Format::

            Time step 0 Front View <image> ...
            Time step 1 Front View <image> ...

        Camera identity and temporal order are communicated entirely through
        text tags interleaved with visual tokens — no custom embeddings needed.
        """
        tags: list[str] = []
        num_steps = min(len(history), 8)  # cap history at 8 steps
        cameras = list(obs_config.camera_weights.keys()) or ["Front View"]

        for t in range(num_steps):
            for camera in cameras:
                tags.append(f"Time step {t} {camera}")
        return tags

    @staticmethod
    def build_prompt(
        instruction: str,
        task_mode: TaskMode,
        obs_config: ObservationConfig,
        history_tags: list[str],
    ) -> str:
        """Assemble the full text prompt sent to Qwen-RobotNav.

        The prompt includes task-mode instructions, the sub-goal, and
        observation context so the model knows how to weight visual tokens.
        """
        mode_prompts = {
            TaskMode.VLN: "Follow the instruction to navigate through the environment.",
            TaskMode.POINTNAV: "Navigate to the specified point goal.",
            TaskMode.OBJNAV: f"Find and navigate to: {instruction}",
            TaskMode.TRACKING: f"Track the target: {instruction}",
            TaskMode.DRIVING: "Follow the route safely.",
        }

        mode_instruction = mode_prompts.get(task_mode, mode_prompts[TaskMode.VLN])
        num_history = history_tags.count("Time step 0")

        return (
            f"{mode_instruction}\n"
            f"Instruction: {instruction}\n"
            f"Observation context: {num_history} frames, "
            f"token_budget={obs_config.token_budget}, "
            f"temporal_decay={obs_config.temporal_decay:.1f}, "
            f"frame_sample={obs_config.frame_sample_mode}\n"
            f"Output waypoints as: (x, y, theta) one per line."
        )

    # ------------------------------------------------------------------
    # Output parsing
    # ------------------------------------------------------------------

    @staticmethod
    def parse_waypoints(raw: str) -> list[Waypoint]:
        """Parse the model's text output into a list of Waypoints.

        Expected format (one per line)::

            (1.23, -0.45, 0.10)
            (2.50, 0.30, -0.05)
            ...
            DONE

        The special string ``DONE`` (case-insensitive) signals the
        end of navigation.  It appears as the only line in the output
        when the model determines the instruction is complete.
        """
        results: list[Waypoint] = []
        for line in raw.strip().splitlines():
            text = line.strip().upper()
            if not text:
                continue
            if text == "DONE":
                if not results:
                    results.append(Waypoint(0.0, 0.0, 0.0))
                return results
            # Strip parentheses and parse comma-separated floats
            cleaned = text.strip("()[]{}")
            try:
                parts = [float(v.strip()) for v in cleaned.split(",")]
            except ValueError:
                continue
            if len(parts) >= 2:
                x, y = parts[0], parts[1]
                theta = parts[2] if len(parts) >= 3 else 0.0
                results.append(Waypoint(x, y, theta))
        return results if results else [Waypoint(0.0, 0.0, 0.0)]

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _load(self) -> None:
        """Lazy-load or validate the model backend.

        - If ``api_url`` is configured: validate GPUStack connectivity.
        - If ``model_path`` points to a local checkpoint: load weights.
        - Otherwise: enter placeholder mode (safe for integration tests).
        """
        self._loaded = True
        if self._backend is not None:
            if not self._backend.validate():
                _logger.warning(
                    "GPUStack backend validation failed. "
                    "Falling back to placeholder mode."
                )
                self._backend = None  # fall through to placeholder
            else:
                _logger.info("GPUStack backend validated successfully.")
                return
        if not self._model_path:
            return
        if not os.path.exists(self._model_path):
            raise FileNotFoundError(
                f"Qwen-RobotNav checkpoint not found: {self._model_path}"
            )

    def _infer(
        self,
        image: np.ndarray,
        instruction: str,
        task_mode: TaskMode,
        obs_config: ObservationConfig,
    ) -> list[Waypoint]:
        """Run inference.

        When ``self._backend`` is alive, this sends the latest image +
        navigation prompt to the VL model via GPUStack and parses the
        response into waypoints.  Otherwise, returns a deterministic
        straight-ahead placeholder trajectory.
        """
        if self._backend is not None:
            return self._infer_via_backend(image, instruction, task_mode, obs_config)
        return self._infer_placeholder(image, instruction, task_mode, obs_config)

    def _infer_via_backend(
        self,
        image: np.ndarray,
        instruction: str,
        task_mode: TaskMode,
        obs_config: ObservationConfig,
    ) -> list[Waypoint]:
        """Real inference: VL model sees the scene, LLM decides waypoints.

        Two-step pipeline:
          1. VL model (qwen3-vl-8b-instruct): describe scene, obstacles, free space.
          2. LLM (qwen3.5-35b-a3b): based on VL description + instruction,
             output waypoints in the ``(x, y, theta)`` format.
        """
        # Step 1: VL scene understanding
        scene_prompt = (
            "You are a robot navigation assistant. Analyze this front camera image "
            "and describe: (1) what obstacles are visible, (2) where the traversable "
            "free space is, (3) the approximate distance to the nearest obstacle in "
            "each direction (forward, left, right). Be concise and quantitative."
        )
        try:
            scene_desc = self._backend.vl_chat(image, scene_prompt, max_tokens=512)
        except Exception as exc:
            _logger.error("VL scene analysis failed: %s", exc)
            return self._infer_placeholder(image, instruction, task_mode, obs_config)

        # Step 2: LLM navigation decision
        mode_names = {
            TaskMode.VLN: "follow the instruction to navigate",
            TaskMode.OBJNAV: "find and navigate to the object",
            TaskMode.POINTNAV: "navigate to the point goal",
            TaskMode.TRACKING: "track the target",
            TaskMode.DRIVING: "follow the route",
        }
        system = (
            "You are a robot navigation planner. Based on a scene description and "
            "a navigation instruction, output exactly 1-8 waypoints in body-frame "
            "coordinates. Each waypoint is one line: (x, y, theta) where x is forward "
            "meters, y is lateral meters (positive = left), and theta is heading in "
            "radians at that waypoint. The last line should be DONE if the goal is "
            "reached. Keep waypoints within 3 meters forward. Output ONLY the waypoints, "
            "no other text."
        )
        user = (
            f"Task: {mode_names.get(task_mode, mode_names[TaskMode.VLN])}.\n"
            f"Instruction: {instruction}\n\n"
            f"Scene analysis from front camera:\n{scene_desc}\n\n"
            f"Output waypoints (one per line, format: (x, y, theta)):"
        )
        try:
            raw = self._backend.llm_chat(system, user, max_tokens=2048, temperature=0.0)
        except Exception as exc:
            _logger.error("LLM navigation decision failed: %s", exc)
            return self._infer_placeholder(image, instruction, task_mode, obs_config)

        return self.parse_waypoints(raw)

    def _infer_placeholder(
        self,
        image: np.ndarray,
        instruction: str,
        task_mode: TaskMode,
        obs_config: ObservationConfig,
    ) -> list[Waypoint]:
        """Placeholder: deterministic straight-ahead trajectory."""
        step = 0.3  # metres per waypoint step
        return [
            Waypoint(step * (i + 1), 0.0, 0.0)
            for i in range(self.NUM_WAYPOINTS)
        ]


# ============================================================================
# NavigationCommand helpers  (convert waypoints → unified command)
# ============================================================================

def waypoints_to_command(waypoints: list[Waypoint]) -> NavigationCommand:
    """Convert Qwen-RobotNav waypoints to a unified NavigationCommand.

    Takes the first waypoint as the immediate action.  If the only waypoint
    is at origin, the episode is considered DONE.
    """
    if not waypoints:
        return NavigationCommand(False, done=True)
    first = waypoints[0]
    done = (
        len(waypoints) == 1
        and abs(first.x_m) < 1e-6
        and abs(first.y_m) < 1e-6
    )
    return NavigationCommand(
        image_space=False,
        dx_m=first.x_m,
        dy_m=first.y_m,
        yaw_rad=first.theta_rad,
        done=done,
    )


# ============================================================================
# RoboStral Navigate adapter  (secondary)
# ============================================================================

class RobostralNavigateAdapter:
    """Adapter for Mistral RoboStral Navigate (8B embodied navigation model).

    This adapter is retained for reference.  The model takes a single RGB
    frame + instruction and outputs image-space pointing (u, v, yaw) when
    the goal is visible, or metric body-frame displacement otherwise.

    As of 2026-07, weights have NOT been publicly released (API/enterprise
    only).  This adapter runs in documented placeholder mode.
    """

    def __init__(
        self,
        model_path: str = "",
        device: str = "cuda:0",
        image_size: tuple[int, int] = (640, 480),
        confidence_threshold: float = 0.5,
    ):
        self._model_path = model_path
        self._device = device
        self._image_size = image_size
        self._confidence_threshold = confidence_threshold
        self._lock = threading.Lock()
        self._loaded = False
        self._instruction = ""

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def predict(
        self,
        image: np.ndarray,
        instruction: str,
        history: Optional[list[np.ndarray]] = None,
    ) -> NavigationCommand:
        with self._lock:
            if not self._loaded:
                self._load()
            if instruction and instruction != self._instruction:
                self._instruction = instruction
            return self._infer(image)

    def reset(self, instruction: str = "") -> None:
        with self._lock:
            self._instruction = instruction

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _load(self) -> None:
        self._loaded = True
        if not self._model_path:
            return
        if not os.path.exists(self._model_path):
            raise FileNotFoundError(
                f"RoboStral Navigate checkpoint not found: {self._model_path}"
            )

    def _infer(self, image: np.ndarray) -> NavigationCommand:
        h, w = image.shape[:2]
        return NavigationCommand(
            image_space=True,
            u=float(w) / 2.0,
            v=float(h) / 2.0,
            done=True,
        )


# ============================================================================
# RoboStral output parser
# ============================================================================

def parse_robostral_output(raw: str) -> NavigationCommand:
    """Parse RoboStral Navigate text output into a NavigationCommand.

    Format:
      - "VIEW <u> <v> <yaw_rad>"       → image-space pointing
      - "MOVE <dx_m> <dy_m> <yaw_rad>" → metric displacement
      - "DONE"                          → task complete
    """
    text = raw.strip().upper()
    if text.startswith("DONE"):
        return NavigationCommand(True, done=True)
    parts = text.split()
    if len(parts) < 4:
        raise ValueError(f"unexpected RoboStral output: {raw!r}")
    mode, a, b, c = parts[0], float(parts[1]), float(parts[2]), float(parts[3])
    if mode == "VIEW":
        return NavigationCommand(True, a, b, yaw_rad=c)
    if mode == "MOVE":
        return NavigationCommand(False, dx_m=a, dy_m=b, yaw_rad=c)
    raise ValueError(f"unknown RoboStral mode: {mode!r}")


# ============================================================================
# Coordinate helpers – project image-space pointing to world
# ============================================================================

def pixel_to_world_displacement(
    u: float,
    v: float,
    intrinsics_fx: float,
    intrinsics_fy: float,
    intrinsics_cx: float,
    intrinsics_cy: float,
    ground_z: float,
    camera_to_base_rotation: np.ndarray,
) -> tuple[float, float]:
    """Convert an image-space goal pixel to a body-frame displacement.

    Assumes the floor plane is horizontal at ``ground_z`` below the camera
    origin.  Returns (dx, dy) in the robot base frame.
    """
    dx_cam = (float(u) - float(intrinsics_cx)) / float(intrinsics_fx)
    dy_cam = (float(v) - float(intrinsics_cy)) / float(intrinsics_fy)
    direction_cam = np.array([dx_cam, dy_cam, 1.0], dtype=np.float64)
    scale = float(ground_z) / direction_cam[2]
    point_cam = direction_cam * scale
    point_base = np.asarray(camera_to_base_rotation, dtype=np.float64) @ point_cam
    return float(point_base[0]), float(point_base[1])


def displacement_to_pose_stamped(
    dx_m: float,
    dy_m: float,
    yaw_rad: float,
    base_frame: str,
    current_pose: Optional[tuple] = None,
):
    """Convert a body-frame displacement to a geometry_msgs/PoseStamped."""
    from geometry_msgs.msg import PoseStamped

    if current_pose is None:
        cx, cy, cz = 0.0, 0.0, 0.0
    else:
        cx, cy, cz = (
            float(current_pose[0]),
            float(current_pose[1]),
            float(current_pose[2]),
        )
    msg = PoseStamped()
    msg.header.frame_id = base_frame
    msg.pose.position.x = cx + float(dx_m)
    msg.pose.position.y = cy + float(dy_m)
    msg.pose.position.z = cz
    half = float(yaw_rad) / 2.0
    msg.pose.orientation.z = math.sin(half)
    msg.pose.orientation.w = math.cos(half)
    return msg
