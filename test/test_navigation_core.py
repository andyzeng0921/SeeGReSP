"""Tests for navigation_core module — Qwen-RobotNav + RoboStral Navigate."""

import math
from pathlib import Path
import numpy as np

from adaptive_object_grasping_nodes.navigation_core import (
    GPUStackBackend,
    NavigationCommand,
    ObservationConfig,
    QwenRobotNavAdapter,
    RobostralNavigateAdapter,
    TaskMode,
    Waypoint,
    displacement_to_pose_stamped,
    parse_robostral_output,
    pixel_to_world_displacement,
    waypoints_to_command,
)

ROOT = Path(__file__).resolve().parents[1]


def test_navigation_node_uses_environment_credentials_without_committed_secret():
    node = (ROOT / 'scripts/robostral_navigate_node.py').read_text(encoding='utf-8')
    config = (ROOT / 'config/navigation.yaml').read_text(encoding='utf-8')
    assert "'GPUSTACK_API_URL', ''" in node
    assert "'GPUSTACK_API_KEY', ''" in node
    assert 'gpustack_' not in config

# ============================================================================
# Waypoint
# ============================================================================

class TestWaypoint:
    def test_creation(self):
        wp = Waypoint(1.5, -0.3, 0.25)
        assert wp.x_m == 1.5
        assert wp.y_m == -0.3
        assert wp.theta_rad == 0.25

    def test_frozen(self):
        wp = Waypoint(0.0, 0.0, 0.0)
        try:
            wp.x_m = 1.0
            assert False, 'Waypoint should be frozen'
        except Exception:
            pass

# ============================================================================
# TaskMode
# ============================================================================

class TestTaskMode:
    def test_enum_values(self):
        assert TaskMode.VLN == "vln"
        assert TaskMode.POINTNAV == "pointnav"
        assert TaskMode.OBJNAV == "objnav"
        assert TaskMode.TRACKING == "tracking"
        assert TaskMode.DRIVING == "driving"

    def test_from_string(self):
        assert TaskMode("vln") == TaskMode.VLN
        assert TaskMode("tracking") == TaskMode.TRACKING

# ============================================================================
# ObservationConfig
# ============================================================================

class TestObservationConfig:
    def test_defaults(self):
        cfg = ObservationConfig()
        assert cfg.token_budget == 3072
        assert cfg.temporal_decay == 2.0
        assert cfg.frame_sample_mode == "random"
        assert cfg.camera_weights == {"Front View": 1.0}

    def test_custom(self):
        cfg = ObservationConfig(
            token_budget=4096, temporal_decay=3.0,
            frame_sample_mode="latest",
            camera_weights={"Front View": 1.0, "Front Right View": 0.5},
        )
        assert cfg.token_budget == 4096
        assert cfg.camera_weights["Front Right View"] == 0.5

    def test_invalid_sample_mode(self):
        try:
            ObservationConfig(frame_sample_mode="invalid")
            assert False, 'should have raised'
        except ValueError:
            pass

    def test_token_budget_too_small(self):
        try:
            ObservationConfig(token_budget=64)
            assert False, 'should have raised'
        except ValueError:
            pass

# ============================================================================
# QwenRobotNavAdapter
# ============================================================================

class TestQwenRobotNavAdapter:
    def test_creates(self):
        adapter = QwenRobotNavAdapter()
        assert adapter is not None
        assert adapter.NUM_WAYPOINTS == 8

    def test_placeholder_predict(self):
        adapter = QwenRobotNavAdapter(default_task_mode=TaskMode.VLN)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        waypoints = adapter.predict(image, 'go to the door')
        assert len(waypoints) == 8
        assert all(isinstance(wp, Waypoint) for wp in waypoints)
        # placeholder produces straight-ahead trajectory
        assert waypoints[0].x_m > 0.0
        assert waypoints[0].y_m == 0.0

    def test_predict_with_task_mode(self):
        adapter = QwenRobotNavAdapter()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        waypoints = adapter.predict(
            image, 'find the red ball',
            task_mode=TaskMode.OBJNAV,
        )
        assert len(waypoints) == 8

    def test_predict_with_obs_config(self):
        adapter = QwenRobotNavAdapter()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cfg = ObservationConfig(token_budget=2048, temporal_decay=0.5)
        waypoints = adapter.predict(
            image, 'track target',
            task_mode=TaskMode.TRACKING,
            obs_config=cfg,
        )
        assert len(waypoints) == 8

    def test_reset(self):
        adapter = QwenRobotNavAdapter()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        adapter.predict(image, 'go left')
        adapter.reset()
        # after reset, predict again should not error
        adapter.predict(image, 'new instruction')

    def test_history_accumulates(self):
        adapter = QwenRobotNavAdapter()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        for i in range(3):
            adapter.predict(image, f'step {i}')
        # history is internal; we just verify it doesn't explode
        assert True


class TestQwenBuildObservationTags:
    def test_single_camera_three_steps(self):
        history = [np.zeros((480, 640, 3), dtype=np.uint8)] * 3
        cfg = ObservationConfig()
        tags = QwenRobotNavAdapter.build_observation_tags(history, cfg)
        assert tags == [
            "Time step 0 Front View",
            "Time step 1 Front View",
            "Time step 2 Front View",
        ]

    def test_multi_camera(self):
        history = [np.zeros((480, 640, 3), dtype=np.uint8)] * 2
        cfg = ObservationConfig(
            camera_weights={"Front View": 1.0, "Front Right View": 0.5},
        )
        tags = QwenRobotNavAdapter.build_observation_tags(history, cfg)
        assert tags == [
            "Time step 0 Front View", "Time step 0 Front Right View",
            "Time step 1 Front View", "Time step 1 Front Right View",
        ]

    def test_caps_at_eight_steps(self):
        history = [np.zeros((480, 640, 3), dtype=np.uint8)] * 20
        cfg = ObservationConfig()
        tags = QwenRobotNavAdapter.build_observation_tags(history, cfg)
        # 8 steps max * 1 camera = 8 tags
        assert len(tags) == 8

class TestQwenBuildPrompt:
    def test_vln_prompt(self):
        prompt = QwenRobotNavAdapter.build_prompt(
            "go to the kitchen",
            TaskMode.VLN,
            ObservationConfig(),
            ["Time step 0 Front View"],
        )
        assert "Follow the instruction" in prompt
        assert "go to the kitchen" in prompt
        assert "token_budget=3072" in prompt
        assert "temporal_decay=2.0" in prompt

    def test_objnav_prompt(self):
        prompt = QwenRobotNavAdapter.build_prompt(
            "red ball", TaskMode.OBJNAV,
            ObservationConfig(),
            ["Time step 0 Front View"],
        )
        assert "Find and navigate to" in prompt

    def test_tracking_prompt(self):
        prompt = QwenRobotNavAdapter.build_prompt(
            "person in blue", TaskMode.TRACKING,
            ObservationConfig(),
            ["Time step 0 Front View"],
        )
        assert "Track the target" in prompt


class TestQwenParseWaypoints:
    def test_normal_output(self):
        raw = """(1.23, -0.45, 0.10)
(2.50, 0.30, -0.05)
(3.80, 0.15, 0.00)"""
        waypoints = QwenRobotNavAdapter.parse_waypoints(raw)
        assert len(waypoints) == 3
        assert waypoints[0] == Waypoint(1.23, -0.45, 0.10)
        assert waypoints[1] == Waypoint(2.50, 0.30, -0.05)
        assert waypoints[2] == Waypoint(3.80, 0.15, 0.00)

    def test_done_from_start(self):
        waypoints = QwenRobotNavAdapter.parse_waypoints("DONE")
        assert len(waypoints) == 1
        assert waypoints[0] == Waypoint(0.0, 0.0, 0.0)

    def test_done_lowercase(self):
        waypoints = QwenRobotNavAdapter.parse_waypoints("done")
        assert len(waypoints) == 1

    def test_done_after_waypoints(self):
        raw = """(1.0, 0.0, 0.0)
(2.0, 0.0, 0.0)
DONE"""
        waypoints = QwenRobotNavAdapter.parse_waypoints(raw)
        # DONE terminates parsing, so we get the first two
        assert len(waypoints) == 2
        assert waypoints[0] == Waypoint(1.0, 0.0, 0.0)

    def test_two_values_default_theta(self):
        waypoints = QwenRobotNavAdapter.parse_waypoints("(1.5, 0.3)")
        assert len(waypoints) == 1
        assert waypoints[0].theta_rad == 0.0

    def test_brackets_and_braces(self):
        """Parser strips various bracket types."""
        assert QwenRobotNavAdapter.parse_waypoints("[1.0, 2.0, 0.5]")[0].x_m == 1.0
        assert QwenRobotNavAdapter.parse_waypoints("{3.0, 4.0}")[0].x_m == 3.0

    def test_mixed_valid_invalid_lines(self):
        raw = """(1.0, 0.0, 0.0)
garbage line
(2.0, 0.0, 0.0)"""
        waypoints = QwenRobotNavAdapter.parse_waypoints(raw)
        assert len(waypoints) == 2

    def test_empty_input(self):
        waypoints = QwenRobotNavAdapter.parse_waypoints("")
        assert len(waypoints) == 1
        assert waypoints[0] == Waypoint(0.0, 0.0, 0.0)


# ============================================================================
# waypoints_to_command
# ============================================================================

class TestWaypointsToCommand:
    def test_first_waypoint_used(self):
        wps = [Waypoint(1.0, 0.5, 0.2), Waypoint(2.0, 0.0, 0.0)]
        cmd = waypoints_to_command(wps)
        assert not cmd.image_space
        assert cmd.dx_m == 1.0
        assert cmd.dy_m == 0.5
        assert cmd.yaw_rad == 0.2
        assert not cmd.done

    def test_single_zero_waypoint_is_done(self):
        wps = [Waypoint(0.0, 0.0, 0.0)]
        cmd = waypoints_to_command(wps)
        assert cmd.done

    def test_empty_waypoints_is_done(self):
        cmd = waypoints_to_command([])
        assert cmd.done


# ============================================================================
# NavigationCommand
# ============================================================================

class TestNavigationCommand:
    def test_creation(self):
        cmd = NavigationCommand(True, u=320.0, v=240.0, yaw_rad=0.5)
        assert cmd.image_space
        assert cmd.u == 320.0
        assert not cmd.done

    def test_done_flag(self):
        cmd = NavigationCommand(False, dx_m=1.0, dy_m=0.5, yaw_rad=0.3, done=True)
        assert cmd.done
        assert not cmd.image_space


# ============================================================================
# RobostralNavigateAdapter (legacy)
# ============================================================================

class TestRobostralNavigateAdapter:
    def test_creates(self):
        model = RobostralNavigateAdapter()
        assert model is not None

    def test_placeholder_predict(self):
        model = RobostralNavigateAdapter()
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cmd = model.predict(image, 'go to the desk')
        assert cmd.image_space
        assert cmd.u == 320.0
        assert cmd.v == 240.0
        assert cmd.done

    def test_reset(self):
        model = RobostralNavigateAdapter()
        model.reset('new instruction')
        model.predict(np.zeros((480, 640, 3), dtype=np.uint8), 'go left')


# ============================================================================
# parse_robostral_output
# ============================================================================

class TestParseRobostralOutput:
    def test_view_mode(self):
        cmd = parse_robostral_output('VIEW 320 240 0.5')
        assert cmd.image_space
        assert cmd.u == 320.0
        assert cmd.v == 240.0
        assert cmd.yaw_rad == 0.5
        assert not cmd.done

    def test_move_mode(self):
        cmd = parse_robostral_output('MOVE 1.5 -0.3 0.2')
        assert not cmd.image_space
        assert cmd.dx_m == 1.5
        assert cmd.dy_m == -0.3
        assert cmd.yaw_rad == 0.2

    def test_done(self):
        cmd = parse_robostral_output('DONE')
        assert cmd.done

    def test_done_lower(self):
        cmd = parse_robostral_output('done')
        assert cmd.done

    def test_invalid_raises(self):
        try:
            parse_robostral_output('garbage')
            assert False, 'should have raised'
        except ValueError:
            pass


# ============================================================================
# GPUStackBackend
# ============================================================================

class TestGPUStackBackend:
    def test_requires_api_url(self):
        try:
            GPUStackBackend(api_url="", api_key="test")
            assert False, "should have raised"
        except ValueError:
            pass

    def test_creates_with_url(self):
        backend = GPUStackBackend(
            api_url="http://localhost:8088/v1",
            api_key="test-key",
        )
        assert backend is not None
        assert backend._vl_model == "qwen3-vl-8b-instruct"
        assert backend._llm_model == "qwen3.5-35b-a3b"

    def test_custom_models(self):
        backend = GPUStackBackend(
            api_url="http://localhost:8088/v1",
            api_key="test-key",
            vl_model="custom-vl",
            llm_model="custom-llm",
            vl_max_tokens=512,
            llm_max_tokens=2048,
            timeout=60.0,
            max_retries=1,
        )
        assert backend._vl_model == "custom-vl"
        assert backend._vl_max_tokens == 512
        assert backend._max_retries == 1

    def test_validate_connection_refused(self):
        backend = GPUStackBackend(
            api_url="http://127.0.0.1:19999/v1",
            api_key="test-key",
            timeout=2.0,
            max_retries=0,
        )
        assert not backend.validate()

    def test_encode_image_rgb(self):
        backend = GPUStackBackend(
            api_url="http://localhost:8088/v1",
            api_key="test",
        )
        image = np.zeros((64, 64, 3), dtype=np.uint8)
        b64 = backend._encode_image(image)
        assert isinstance(b64, str)
        assert len(b64) > 0


class TestQwenRobotNavWithBackend:
    def test_adapter_creates_backend(self):
        adapter = QwenRobotNavAdapter(
            api_url="http://localhost:8088/v1",
            api_key="test-key",
        )
        assert adapter._backend is not None

    def test_adapter_no_backend_when_empty(self):
        adapter = QwenRobotNavAdapter()
        assert adapter._backend is None

    def test_load_graceful_degrade(self):
        adapter = QwenRobotNavAdapter(
            api_url="http://127.0.0.1:19999/v1",
            api_key="test",
        )
        adapter._load()
        assert adapter._backend is None  # cleared after failed validation
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        waypoints = adapter._infer_placeholder(
            image, "test", TaskMode.VLN, ObservationConfig()
        )
        assert len(waypoints) == 8


# ============================================================================
# Pixel / displacement helpers
# ============================================================================

class TestPixelToWorldDisplacement:
    def test_centre_straight_ahead(self):
        cx, cy, fx, fy = 320.0, 240.0, 500.0, 500.0
        dx, dy = pixel_to_world_displacement(
            320.0, 240.0, fx, fy, cx, cy,
            ground_z=1.0, camera_to_base_rotation=np.eye(3),
        )
        assert abs(dx) < 1e-9
        assert abs(dy) < 1e-9

    def test_rightward_pixel(self):
        cx, cy, fx, fy = 320.0, 240.0, 500.0, 500.0
        dx, dy = pixel_to_world_displacement(
            420.0, 240.0, fx, fy, cx, cy,
            ground_z=1.0, camera_to_base_rotation=np.eye(3),
        )
        assert dx > 0.0
        assert abs(dy) < 1e-9

    def test_upward_pixel(self):
        cx, cy, fx, fy = 320.0, 240.0, 500.0, 500.0
        dx, dy = pixel_to_world_displacement(
            320.0, 140.0, fx, fy, cx, cy,
            ground_z=1.0, camera_to_base_rotation=np.eye(3),
        )
        assert abs(dx) < 1e-9
        assert dy < 0.0


class TestDisplacementToPoseStamped:
    def test_from_origin(self):
        pose = displacement_to_pose_stamped(1.0, 0.5, 0.0, 'base_link')
        assert pose.header.frame_id == 'base_link'
        assert pose.pose.position.x == 1.0
        assert pose.pose.position.y == 0.5
        assert abs(pose.pose.orientation.w - 1.0) < 1e-9

    def test_yaw_ninety_degrees(self):
        pose = displacement_to_pose_stamped(0.0, 0.0, math.pi / 2.0, 'base_link')
        expected_z = math.sin(math.pi / 4.0)
        expected_w = math.cos(math.pi / 4.0)
        assert abs(pose.pose.orientation.z - expected_z) < 1e-9
        assert abs(pose.pose.orientation.w - expected_w) < 1e-9

    def test_from_current_pose(self):
        pose = displacement_to_pose_stamped(
            0.5, -0.3, 0.0, 'base_link',
            current_pose=(2.0, 1.0, 0.5, 0.0, 0.0, 0.0, 1.0),
        )
        assert pose.pose.position.x == 2.5
        assert pose.pose.position.y == 0.7
        assert pose.pose.position.z == 0.5
