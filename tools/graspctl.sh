#!/usr/bin/env bash
set -euo pipefail

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
RUNTIME="$ROOT/runtime"
LOG="$RUNTIME/grasp-stack.log"
PIDFILE="$RUNTIME/grasp-stack.pid"
CONFIRMATION="I_HAVE_CHECKED_ESTOP_AND_WORKSPACE"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROBOT_ID="${ROBOT_ID:-306}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\"/></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"
export ADAPTIVE_GRASP_PACKAGE_ROOT="$ROOT"

set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$ROOT/install/setup.bash" ]]; then
  echo "Not built. Run: $ROOT/tools/build_project.sh" >&2
  exit 2
fi
source "$ROOT/install/setup.bash"
set -u
mkdir -p "$RUNTIME"

valid_label() {
  [[ "$1" =~ ^[A-Za-z0-9_.-]{1,64}$ ]] || {
    echo "Invalid label. Use a model class such as bottle, cup, apple." >&2
    exit 2
  }
}

stack_running() {
  [[ -f "$PIDFILE" ]] || return 1
  local pgid
  pgid="$(cat "$PIDFILE")"
  [[ "$pgid" =~ ^[1-9][0-9]*$ ]] || return 1
  # The setsid leader can exit before ros2 launch children. Check the whole
  # process group so a later start cannot create duplicate ROS action servers.
  kill -0 -- "-$pgid" 2>/dev/null
}

require_installed_config_sync() {
  local installed_share rel
  installed_share="$(ros2 pkg prefix --share adaptive_object_grasping 2>/dev/null)" || {
    echo "Cannot resolve the installed adaptive_object_grasping share directory." >&2
    exit 3
  }
  for rel in \
    config/motion.yaml \
    config/site_acceptance.yaml \
    config/environment_scene.yaml \
    config/visible_reset_pose.yaml; do
    [[ -f "$installed_share/$rel" ]] || {
      echo "Installed configuration is missing: $installed_share/$rel" >&2
      exit 3
    }
    cmp -s "$ROOT/$rel" "$installed_share/$rel" || {
      echo "Refusing hardware motion: source and installed $rel differ." >&2
      echo "Run $ROOT/tools/build_project.sh, then restart the stack." >&2
      exit 3
    }
  done
}

case "${1:-help}" in
  start)
    if stack_running; then
      echo "Grasp stack already running (PID $(cat "$PIDFILE"))."
      exit 0
    fi
    nohup setsid ros2 launch adaptive_object_grasping bringup.launch.py rviz:=false \
      >"$LOG" 2>&1 &
    echo "$!" >"$PIDFILE"
    echo "Started PID $!; log: $LOG"
    echo "Before planning, wait for 'GraspNet backend preload complete' in the log."
    ;;
  stop)
    if stack_running; then
      pgid="$(cat "$PIDFILE")"
      kill -- "-$pgid"
      echo "Stopped grasp stack process group $pgid."
      echo "This stops ROS processes only; it is not a robot motion stop. Use the physical E-stop if any hardware command may still be active." >&2
    else
      echo "Grasp stack is not running."
    fi
    rm -f "$PIDFILE"
    ;;
  logs)
    tail -n "${2:-120}" "$LOG"
    ;;
  status)
    stack_running && echo "stack=running pid=$(cat "$PIDFILE")" || echo "stack=stopped"
    if ! timeout 15 ros2 service call /check_grasp_hardware std_srvs/srv/Trigger '{}'; then
      echo "Hardware probe unavailable; hardware is not authorized." >&2
    else
      echo "Probe readiness only reports live inputs; it never unlocks hardware." >&2
    fi
    ;;
  list)
    timeout 15 ros2 service call /list_grasp_objects \
      adaptive_object_grasping/srv/ListObjects '{}'
    ;;
  scene|sample)
    label="${2:-bottle}"
    track_id="${3:--1}"
    valid_label "$label"
    [[ "$track_id" =~ ^-?[0-9]+$ ]] || {
      echo "Track ID must be an integer." >&2
      exit 2
    }
    stack_running || {
      echo "Grasp stack is stopped. Run: $0 start" >&2
      exit 2
    }
    timeout 30 "$ROOT/third_party/venv/bin/python" \
      "$ROOT/tools/observe_scene.py" \
      --label "$label" \
      --track-id "$track_id" \
      --output-directory "$RUNTIME/scene_samples"
    ;;
  plan)
    label="${2:-bottle}"
    arm="${3:-auto}"
    valid_label "$label"
    [[ "$arm" =~ ^(auto|left|right)$ ]] || { echo "Arm must be auto, left, or right." >&2; exit 2; }
    if [[ -f "$LOG" ]] && ! grep -q "GraspNet backend preload complete" "$LOG"; then
      echo "GraspNet preload is not confirmed yet; this first plan may fail safely." >&2
    fi
    timeout 120 ros2 action send_goal /pick_object \
      adaptive_object_grasping/action/PickObject \
      "{track_id: -1, label: '$label', preferred_arm: '$arm', execute: false, execution_confirmation: '', maximum_tracking_time: 15.0, required_stable_duration: 0.3}" \
      --feedback
    ;;
  reset-plan)
    stack_running || {
      echo "Grasp stack is stopped. Run: $0 start" >&2
      exit 2
    }
    timeout 120 ros2 service call /plan_visible_reset std_srvs/srv/Trigger '{}'
    ;;
  execute)
    label="${2:-}"
    arm="${3:-auto}"
    confirmation="${4:-}"
    valid_label "$label"
    [[ "$arm" =~ ^(auto|left|right)$ ]] || { echo "Arm must be auto, left, or right." >&2; exit 2; }
    [[ "$confirmation" == "$CONFIRMATION" ]] || {
      echo "Refusing hardware motion. Pass the exact operator confirmation: $CONFIRMATION" >&2
      exit 3
    }
    require_installed_config_sync
    "$ROOT/third_party/venv/bin/python" \
      "$ROOT/tools/check_execution_readiness.py" \
      "$ROOT/config/site_acceptance.yaml" || exit 3
    grep -Eq '^[[:space:]]*dry_run:[[:space:]]*false' "$ROOT/config/motion.yaml" &&
      grep -Eq '^[[:space:]]*allow_hardware_execution:[[:space:]]*true' "$ROOT/config/motion.yaml" || {
        echo "Hardware remains locked in config/motion.yaml. Follow HANDOFF.md calibration and unlock checklist." >&2
        exit 3
      }
    # Do not wrap a hardware action in a shell timeout: losing the CLI waiter
    # must never be mistaken for stopping robot motion.
    echo "WARNING: client exit, Ctrl-C, or graspctl stop cannot stop an accepted robot trajectory." >&2
    echo "Use the verified physical E-stop if motion is unsafe or its state is unknown." >&2
    ros2 action send_goal /pick_object \
      adaptive_object_grasping/action/PickObject \
      "{track_id: -1, label: '$label', preferred_arm: '$arm', execute: true, execution_confirmation: '$confirmation', maximum_tracking_time: 15.0, required_stable_duration: 0.5}" \
      --feedback
    ;;
  *)
    cat <<EOF
Usage:
  $0 start | stop | status | list | logs [lines] | reset-plan
  $0 scene <label> [track_id]       Exact RGB-D + bottle/arm 3D sample (read-only)
  $0 sample <label> [track_id]      Alias for scene
  $0 plan <label> [auto|left|right]
  $0 execute <label> [auto|left|right] $CONFIRMATION
EOF
    ;;
esac
