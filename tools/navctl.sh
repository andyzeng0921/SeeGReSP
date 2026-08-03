#!/usr/bin/env bash
set -euo pipefail
# ============================================================================
# navctl.sh  –  Robot 306 视觉导航 & 底盘控制
# ============================================================================

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
RUNTIME="$ROOT/runtime"
LOG="$RUNTIME/nav-stack.log"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROBOT_ID="${ROBOT_ID:-306}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"

API_URL="${GPUSTACK_API_URL:-}"
API_KEY="${GPUSTACK_API_KEY:-}"
VL_MODEL="${VL_MODEL:-qwen3-vl-8b-instruct}"
TEXT_MODEL="${TEXT_MODEL:-qwen3.5-35b-a3b}"

HEAD_CAM_SHM="${HEAD_CAM_SHM:-/dev/shm/camera_image_buffer_head_left_jpeg}"

set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
set -u
mkdir -p "$RUNTIME"

# ============================================================================
_ros2_ok() { ros2 topic list &>/dev/null; }

_require_vl_credentials() {
  [[ -n "$API_URL" && -n "$API_KEY" ]] || {
    echo "VL credentials are not configured. Export GPUSTACK_API_URL and GPUSTACK_API_KEY outside the repository." >&2
    return 1
  }
}

_capture_camera() {
  local output="${1:-/tmp/nav_camera_current.jpg}"
  if [[ ! -f "$HEAD_CAM_SHM" ]]; then
    echo "HEAD CAMERA SHM NOT FOUND: $HEAD_CAM_SHM" >&2
    return 1
  fi
  python3 <<PYEOF
data = open('$HEAD_CAM_SHM', 'rb').read()
eoi = data.find(b'\xff\xd9')
jpeg = data[:eoi+2] if eoi > 0 else data
open('$output', 'wb').write(jpeg)
PYEOF
  echo "$output"
}

_send_image_to_vl() {
  local img="$1" prompt="$2" max_tokens="${3:-512}"
  _require_vl_credentials || return 1
  cat <<PYEOF | python3
import json, urllib.request, base64, sys
with open('$img', 'rb') as f:
    b64 = base64.b64encode(f.read()).decode('utf-8')
prompt = '''${prompt}'''
payload = {
    'model': '$VL_MODEL',
    'messages': [{'role': 'user', 'content': [
        {'type': 'text', 'text': prompt},
        {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64}'}}
    ]}],
    'max_tokens': $max_tokens, 'temperature': 0.3
}
req = urllib.request.Request(
    '$API_URL/chat/completions',
    data=json.dumps(payload).encode('utf-8'),
    headers={'Authorization': 'Bearer $API_KEY', 'Content-Type': 'application/json'}
)
resp = urllib.request.urlopen(req, timeout=180)
result = json.loads(resp.read())
print(result['choices'][0]['message']['content'].strip())
print('---')
print('tokens:', result['usage']['completion_tokens'], '/', result['usage']['total_tokens'])
PYEOF
}

_api_chat() {
  local system="$1" user="$2" max_tokens="${3:-1024}" temperature="${4:-0.7}"
  _require_vl_credentials || return 1
  cat <<PYEOF | python3
import json, urllib.request, sys
payload = {
    'model': '$TEXT_MODEL',
    'messages': [
        {'role': 'system', 'content': '''${system}'''},
        {'role': 'user', 'content': '''${user}'''}
    ],
    'max_tokens': $max_tokens, 'temperature': $temperature
}
req = urllib.request.Request(
    '$API_URL/chat/completions',
    data=json.dumps(payload).encode('utf-8'),
    headers={'Authorization': 'Bearer $API_KEY', 'Content-Type': 'application/json'}
)
resp = urllib.request.urlopen(req, timeout=180)
result = json.loads(resp.read())
content = (result['choices'][0]['message'].get('content') or '').strip()
if not content:
    print('(THINKING NOT FINISHED, increase max_tokens)', file=sys.stderr)
    sys.exit(1)
print(content)
PYEOF
}

# ============================================================================
case "${1:-help}" in
  status)
    _ros2_ok && echo "ROS2_OK domain=$ROS_DOMAIN_ID" || echo "ROS2_UNAVAILABLE"
    ros2 topic info /manual_cmd_vel 2>/dev/null && echo "CMD_VEL_OK" || echo "CMD_VEL_UNAVAILABLE"
    [[ -f "$HEAD_CAM_SHM" ]] && echo "HEAD_CAM_LEFT_OK" || echo "HEAD_CAM_LEFT_UNAVAILABLE"
    ;;

  camera)
    _capture_camera "${2:-/tmp/nav_camera.jpg}"
    ;;

  see)
    img=$(_capture_camera) || exit 1
    _send_image_to_vl "$img" "${2:-Describe what you see in this image: objects, obstacles, traversable space.}" 512
    ;;

  decide)
    img=$(_capture_camera) || exit 1
    _send_image_to_vl "$img" "${2:-Based on this front camera image, determine if the path ahead is clear. Output exactly: DIRECTION: [forward|left|right|backward|stop] REASON: [brief reason]}" 512
    ;;

  go)
    x="${2:-}" y="${3:-}" yaw="${4:-0.0}"
    [[ -n "$x" && -n "$y" ]] || { echo "Usage: navctl.sh go <x> <y> [yaw]"; exit 2; }
    echo "PLAN: navigate_to_pose x=$x y=$y yaw=$yaw"
    echo "Hardware execution is intentionally unavailable from OpenClaw."
    ;;

  move)
    dx="${2:-0.0}" dy="${3:-0.0}" da="${4:-0.0}" dur="${5:-1.0}"
    echo "PLAN: cmd_vel dx=$dx dy=$dy da=$da duration=${dur}s"
    echo "Hardware execution is intentionally unavailable from OpenClaw."
    ;;

  stop)
    echo "No software stop is issued by OpenClaw. Use the verified physical E-stop." >&2
    exit 3
    ;;

  execute)
    echo "REFUSED: OpenClaw navigation hardware execution is disabled." >&2
    echo "Use a separately reviewed, supervised robot control procedure." >&2
    exit 3
    ;;

  help|*)
    cat <<'NAVHELP'
navctl.sh  –  Robot 306 Visual Navigation & Base Control

Usage:
  navctl.sh status                  Check ROS 2 / cmd_vel / camera
  navctl.sh camera [output.jpg]     Capture head-left camera frame
  navctl.sh see [VL prompt]         Send camera to VL model, get description
  navctl.sh decide [VL prompt]      Ask VL model to decide movement direction
  navctl.sh go <x> <y> [yaw]        Plan Nav2 map goal (DRY-RUN)
  navctl.sh move <dx> <dy> <da> <dur> Plan velocity command (DRY-RUN)
  navctl.sh stop                    Explain physical E-stop requirement
  navctl.sh execute ...             Always refused

Example visual navigation workflow:
  1. navctl.sh see                  # Look at the scene
  2. navctl.sh decide               # Let AI decide direction
  3. navctl.sh move 0.15 0 0 2.0    # Plan: forward 0.15m/s, 2s
  4. A human reviews the plan in the supervised robot control system

SAFETY: This OpenClaw tool is perception and dry-run only. It cannot move the base.
The verified physical E-stop always has highest priority.
NAVHELP
    ;;
esac
