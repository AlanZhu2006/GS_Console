#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
WORKSPACE_ROOT="$(cd "$ROOT_DIR/.." && pwd -P)"
CVPR_ROOT="${CVPR_ROOT:-$WORKSPACE_ROOT/CVPR}"
NUC_ROOT="${NUC_ROOT:-$CVPR_ROOT/HMR3D/nuc}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WEB_HOST="${WEB_HOST:-0.0.0.0}"
WEB_PORT="${WEB_PORT:-5173}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
WORLD_NAV_PORT="${WORLD_NAV_PORT:-8892}"

START_LINGBOT="${START_LINGBOT:-1}"
START_ROSBRIDGE="${START_ROSBRIDGE:-1}"
START_WEBUI="${START_WEBUI:-1}"
START_MONITOR_SYNC="${START_MONITOR_SYNC:-1}"
START_NAV2_STYLE="${START_NAV2_STYLE:-1}"
START_GAUSSIAN_MONITOR_RENDER="${START_GAUSSIAN_MONITOR_RENDER:-0}"
START_FEEDFORWARD_GS_JOBS="${START_FEEDFORWARD_GS_JOBS:-0}"

LINGBOT_OUTPUT_DIR="${LINGBOT_OUTPUT_DIR:-$CVPR_ROOT/nuc_output/hikrobot_lingbot_ros2_current_cloud_live}"
GS_MONITOR_DIR="${GS_MONITOR_DIR:-$CVPR_ROOT/nuc_output/real2sim_hikrobot_lingbot_live_baseline}"
LINGBOT_LAUNCH="${LINGBOT_LAUNCH:-$NUC_ROOT/scripts/launch_cuvslam_lingbot_live_reconstruction.sh}"
LIVE_CONTRACT="${LIVE_CONTRACT:-/contracts/lingbot-map-ros2-live.live-contract.json}"

if [[ "$START_LINGBOT" == "1" && ! -x "$LINGBOT_LAUNCH" ]]; then
  echo "LingBot launch script is missing or not executable: $LINGBOT_LAUNCH" >&2
  exit 1
fi

if [[ ("$START_LINGBOT" == "1" || "$START_ROSBRIDGE" == "1" || "$START_NAV2_STYLE" == "1") && ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup not found: $ROS_SETUP" >&2
  echo "Set ROS_SETUP=/path/to/setup.bash or disable ROS pieces with START_ROSBRIDGE=0 START_NAV2_STYLE=0." >&2
  exit 1
fi

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
  wait >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

mkdir -p "$LINGBOT_OUTPUT_DIR" "$GS_MONITOR_DIR"
python3 "$CVPR_ROOT/HMR3D/nuc/scripts/generate_gs_console_monitor_viewer.py" \
  --baseline-dir "$GS_MONITOR_DIR" >/dev/null 2>&1 || true

if [[ "$START_ROSBRIDGE" == "1" ]]; then
  if bash -lc "source '$ROS_SETUP' && ros2 pkg prefix rosbridge_server >/dev/null 2>&1"; then
    echo "Starting ROS2 rosbridge on ws://localhost:$ROSBRIDGE_PORT"
    bash -lc "source '$ROS_SETUP' && ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=$ROSBRIDGE_PORT" &
    pids+=("$!")
  elif [[ -n "${ROSBRIDGE_COMMAND:-}" ]]; then
    echo "Starting custom rosbridge command on ws://localhost:$ROSBRIDGE_PORT"
    bash -lc "$ROSBRIDGE_COMMAND" &
    pids+=("$!")
  else
    cat >&2 <<EOF
rosbridge_server is not installed in this ROS environment: $ROS_SETUP
Install it with:
  sudo apt update
  sudo apt install ros-humble-rosbridge-server

Then rerun this script, or set START_ROSBRIDGE=0 if another rosbridge is already
running on ws://localhost:$ROSBRIDGE_PORT.
EOF
    START_ROSBRIDGE=0
  fi
fi

if [[ "$START_LINGBOT" == "1" ]]; then
  echo "Starting LingBot live reconstruction with ROS2 publish enabled"
  (
    cd "$CVPR_ROOT"
    export ROS2_PUBLISH=1
    export OUTPUT_DIR="$LINGBOT_OUTPUT_DIR"
    export ROS2_IMAGE_TOPIC="${ROS2_IMAGE_TOPIC:-/hikrobot/image_raw}"
    export ROS2_CAMERA_INFO_TOPIC="${ROS2_CAMERA_INFO_TOPIC:-/neural_mapping/camera_info}"
    export ROS2_POSE_TOPIC="${ROS2_POSE_TOPIC:-/neural_mapping/pose}"
    export ROS2_PATH_TOPIC="${ROS2_PATH_TOPIC:-/lingbot_nav/trajectory}"
    export ROS2_CLOUD_TOPIC="${ROS2_CLOUD_TOPIC:-/lingbot/cloud_rgb}"
    export ROS2_CLOUD_MIN_INTERVAL_SEC="${ROS2_CLOUD_MIN_INTERVAL_SEC:-0.25}"
    export ROS2_IMAGE_MAX_WIDTH="${ROS2_IMAGE_MAX_WIDTH:-640}"
    export ROS2_IMAGE_MAX_HEIGHT="${ROS2_IMAGE_MAX_HEIGHT:-512}"
    export ROS2_MAX_CLOUD_POINTS="${ROS2_MAX_CLOUD_POINTS:-300000}"
    export ROS2_MAX_CURRENT_CLOUD_POINTS="${ROS2_MAX_CURRENT_CLOUD_POINTS:-60000}"
    export PYTHON_BIN="${PYTHON_BIN:-$(command -v python3)}"
    export MODEL_PATH="${MODEL_PATH:-$CVPR_ROOT/third_party_research/lingbot_cache/lingbot-map.pt}"
    export LINGBOT_ROOT="${LINGBOT_ROOT:-$CVPR_ROOT/third_party_research/lingbot-map}"
    export TRACKING_BACKEND="${TRACKING_BACKEND:-hikrobot_mono_rgb}"
    export HIKROBOT_FPS="${HIKROBOT_FPS:-5}"
    export HIKROBOT_WIDTH="${HIKROBOT_WIDTH:-640}"
    export HIKROBOT_HEIGHT="${HIKROBOT_HEIGHT:-512}"
    export HIKROBOT_EXPOSURE_US="${HIKROBOT_EXPOSURE_US:-15000}"
    export HIKROBOT_GAIN="${HIKROBOT_GAIN:-12}"
    export IMAGE_SIZE="${IMAGE_SIZE:-224}"
    export MODEL_IMAGE_SIZE="${MODEL_IMAGE_SIZE:-518}"
    export WINDOW_SIZE="${WINDOW_SIZE:-2}"
    export WINDOW_STRIDE="${WINDOW_STRIDE:-1}"
    export FRAME_STEP="${FRAME_STEP:-1}"
    export MAX_FRAMES="${MAX_FRAMES:-0}"
    export SAMPLE_STRIDE="${SAMPLE_STRIDE:-1}"
    export SAMPLING_PATTERN="${SAMPLING_PATTERN:-random}"
    export MAX_POINTS_PER_FRAME="${MAX_POINTS_PER_FRAME:-15000}"
    export MAX_ACTIVE_FRAMES="${MAX_ACTIVE_FRAMES:-16}"
    export DROP_WHEN_BUSY="${DROP_WHEN_BUSY:-1}"
    export FLASHINFER_CUDA_ARCH_LIST="${FLASHINFER_CUDA_ARCH_LIST:-8.7}"
    export LINGBOT_CPU_CAST_SCOPE="${LINGBOT_CPU_CAST_SCOPE:-model}"
    set +u
    source "$ROS_SETUP"
    source "$CVPR_ROOT/HMR3D/nuc/configs/hikrobot_mvs_env.sh" 2>/dev/null || true
    source "$CVPR_ROOT/HMR3D/nuc/scripts/use_jetson_gpu_backend.sh" 2>/dev/null || true
    set -u
    exec "$LINGBOT_LAUNCH"
  ) &
  pids+=("$!")
fi

if [[ "$START_MONITOR_SYNC" == "1" ]]; then
  echo "Syncing LingBot live assets into GS monitor directory"
  python3 "$CVPR_ROOT/HMR3D/nuc/scripts/sync_gs_console_monitor_assets.py" \
    --live-dir "$LINGBOT_OUTPUT_DIR" \
    --output-dir "$GS_MONITOR_DIR" \
    --interval-sec "${MONITOR_SYNC_INTERVAL_SEC:-0.75}" \
    --max-points "${MONITOR_MAX_POINTS:-28000}" &
  pids+=("$!")
fi

if [[ "$START_NAV2_STYLE" == "1" ]]; then
  echo "Starting lightweight Nav2-style map bridge from GS monitor JSON"
  bash -lc "source '$ROS_SETUP' && python3 '$CVPR_ROOT/HMR3D/nuc/scripts/publish_gs_console_nav2_style_ros2.py' --monitor-json '$GS_MONITOR_DIR/monitor/live_monitor.json' --interval-sec '${NAV2_STYLE_INTERVAL_SEC:-0.75}' --resolution '${NAV2_STYLE_RESOLUTION:-0.12}'" &
  pids+=("$!")
fi

if [[ "$START_FEEDFORWARD_GS_JOBS" == "1" ]]; then
  echo "Preparing feed-forward GS jobs for GenWildSplat/AnySplat sidecar"
  python3 "$CVPR_ROOT/HMR3D/nuc/scripts/prepare_feedforward_gs_live_jobs.py" \
    --worker-dir "$LINGBOT_OUTPUT_DIR/worker" \
    --output-dir "$GS_MONITOR_DIR" \
    --interval-sec "${FEEDFORWARD_GS_INTERVAL_SEC:-3.0}" \
    --context-views "${FEEDFORWARD_GS_CONTEXT_VIEWS:-6}" \
    --window-results "${FEEDFORWARD_GS_WINDOW_RESULTS:-12}" \
    --image-size "${FEEDFORWARD_GS_IMAGE_SIZE:-448}" \
    --min-new-windows "${FEEDFORWARD_GS_MIN_NEW_WINDOWS:-2}" \
    --sidecar-url "${FEEDFORWARD_GS_SIDECAR_URL:-}" &
  pids+=("$!")
fi

if [[ "$START_GAUSSIAN_MONITOR_RENDER" == "1" ]]; then
  echo "Starting GS monitor renderer at ${GAUSSIAN_RENDER_INTERVAL_SEC:-3.0}s cadence"
  python3 "$CVPR_ROOT/HMR3D/nuc/scripts/render_gs_console_monitor_frame.py" \
    --baseline-dir "$GS_MONITOR_DIR" \
    --worker-dir "$LINGBOT_OUTPUT_DIR/worker" \
    --source "${GAUSSIAN_RENDER_SOURCE:-baseline_seed}" \
    --live-map "${GAUSSIAN_RENDER_LIVE_MAP:-$LINGBOT_OUTPUT_DIR/live_map.npz}" \
    --interval-sec "${GAUSSIAN_RENDER_INTERVAL_SEC:-3.0}" \
    --width "${GAUSSIAN_RENDER_WIDTH:-480}" \
    --render-view-budget-points "${GAUSSIAN_RENDER_BUDGET_POINTS:-24000}" \
    --live-map-max-points "${GAUSSIAN_RENDER_LIVE_MAX_POINTS:-24000}" \
    --live-gaussian-scale "${GAUSSIAN_RENDER_LIVE_SCALE:-0.055}" \
    --live-gaussian-opacity "${GAUSSIAN_RENDER_LIVE_OPACITY:-0.72}" &
  pids+=("$!")
fi

if [[ "$START_WEBUI" == "1" ]]; then
echo "Starting GS Console WebUI on http://localhost:$WEB_PORT/?scene=/scenes/lingbot-live/manifest.json&mode=live&liveContract=$LIVE_CONTRACT"
  (
    export WEB_ADAPTER=lingbot-live
    export WEB_MODE=live
    export WEB_LIVE_CONTRACT="$LIVE_CONTRACT"
    export WEB_LINGBOT_LIVE_ASSET_ROOT="$LINGBOT_OUTPUT_DIR"
    export WEB_LINGBOT_REAL2SIM_ASSET_ROOT="$GS_MONITOR_DIR"
    export ROSBRIDGE_PORT="$ROSBRIDGE_PORT"
    export WORLD_NAV_PORT="$WORLD_NAV_PORT"
    exec "$ROOT_DIR/scripts/launch_web_ui_dev.sh" "$WEB_HOST" "$WEB_PORT"
  ) &
  pids+=("$!")
fi

echo "LingBot realtime stack is up."
echo "WebUI:      http://localhost:$WEB_PORT/?scene=/scenes/lingbot-live/manifest.json&mode=live&liveContract=$LIVE_CONTRACT"
echo "rosbridge:  ws://localhost:$ROSBRIDGE_PORT"
echo "live dir:   $LINGBOT_OUTPUT_DIR"
echo "monitor:    $GS_MONITOR_DIR/real2sim_gs_console_viewer.html"

wait
