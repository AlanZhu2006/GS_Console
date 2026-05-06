#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
WORKSPACE_ROOT="$(cd "$ROOT_DIR/.." && pwd -P)"
CVPR_ROOT="${CVPR_ROOT:-$WORKSPACE_ROOT/CVPR}"

VIDEO_PATH="${VIDEO_PATH:-}"
VIDEO_DIR="${VIDEO_DIR:-$WORKSPACE_ROOT/videos}"
VIDEO_GLOB="${VIDEO_GLOB:-*.mp4}"
FRAMES_DIR="${FRAMES_DIR:-}"
OUTPUT_DIR="${OUTPUT_DIR:-$CVPR_ROOT/nuc_output/video_real2sim_playback/live}"
REAL2SIM_DIR="${REAL2SIM_DIR:-$CVPR_ROOT/nuc_output/video_real2sim_playback/real2sim}"
WEB_HOST="${WEB_HOST:-0.0.0.0}"
WEB_PORT="${WEB_PORT:-5173}"
WEB_CONTRACT="${WEB_CONTRACT:-/contracts/lingbot-map-video-playback.live-contract.json}"
BINARY_CLOUD_WS_PORT="${BINARY_CLOUD_WS_PORT:-19093}"
GLOBAL_BINARY_CLOUD_WS_PORT="${GLOBAL_BINARY_CLOUD_WS_PORT:-19094}"

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

mkdir -p "$OUTPUT_DIR" "$REAL2SIM_DIR"

for port in "$BINARY_CLOUD_WS_PORT" "$GLOBAL_BINARY_CLOUD_WS_PORT" "$WEB_PORT"; do
  pids_on_port="$(lsof -tiTCP:"$port" -sTCP:LISTEN 2>/dev/null || true)"
  if [[ -n "$pids_on_port" ]]; then
    echo "Stopping existing listener on port $port: $pids_on_port"
    kill $pids_on_port >/dev/null 2>&1 || true
    sleep 0.5
  fi
done

playback_args=(
  "$CVPR_ROOT/HMR3D/nuc/scripts/run_video_real2sim_playback_webui.py"
  --output-dir "$OUTPUT_DIR"
  --real2sim-dir "$REAL2SIM_DIR"
  --extract-fps "${EXTRACT_FPS:-2}"
  --playback-fps "${PLAYBACK_FPS:-2}"
  --max-frames "${MAX_FRAMES:-80}"
  --points-per-frame "${POINTS_PER_FRAME:-8000}"
  --max-global-points "${MAX_GLOBAL_POINTS:-180000}"
  --binary-cloud-ws-port "$BINARY_CLOUD_WS_PORT"
  --global-binary-cloud-ws-port "$GLOBAL_BINARY_CLOUD_WS_PORT"
  --binary-cloud-ws-host "${BINARY_CLOUD_WS_HOST:-0.0.0.0}"
  --write-ply-every "${WRITE_PLY_EVERY:-4}"
)

if [[ -n "$FRAMES_DIR" ]]; then
  playback_args+=(--frames-dir "$FRAMES_DIR")
elif [[ -n "$VIDEO_PATH" ]]; then
  playback_args+=(--video "$VIDEO_PATH")
else
  playback_args+=(--video-dir "$VIDEO_DIR" --video-glob "$VIDEO_GLOB")
fi
if [[ -n "${LINGBOT_PREDICTIONS_NPZ:-}" ]]; then
  playback_args+=(--lingbot-predictions-npz "$LINGBOT_PREDICTIONS_NPZ")
  if [[ -n "${LINGBOT_SUMMARY_JSON:-}" ]]; then
    playback_args+=(--lingbot-summary-json "$LINGBOT_SUMMARY_JSON")
  fi
  playback_args+=(--lingbot-conf-percentile "${LINGBOT_CONF_PERCENTILE:-45}")
  if [[ "${NORMALIZE_LINGBOT_WORLD:-1}" == "0" ]]; then
    playback_args+=(--no-normalize-lingbot-world)
  else
    playback_args+=(--normalize-lingbot-world)
  fi
fi

echo "Starting video real-to-sim playback sidecar"
python3 "${playback_args[@]}" &
pids+=("$!")

echo "Starting GS Console WebUI"
(
  export WEB_ADAPTER=lingbot-live
  export WEB_MODE=live
  export WEB_LIVE_CONTRACT="$WEB_CONTRACT"
  export WEB_LINGBOT_LIVE_ASSET_ROOT="$OUTPUT_DIR"
  export WEB_LINGBOT_REAL2SIM_ASSET_ROOT="$REAL2SIM_DIR"
  export WEB_ROSBRIDGE_PORT="${WEB_ROSBRIDGE_PORT:-9090}"
  export WEB_WORLD_NAV_PORT="${WEB_WORLD_NAV_PORT:-8892}"
  exec "$ROOT_DIR/scripts/launch_web_ui_dev.sh" "$WEB_HOST" "$WEB_PORT"
) &
pids+=("$!")

echo "Video real-to-sim playback stack is up."
echo "Open: http://localhost:$WEB_PORT/?scene=/scenes/lingbot-live/manifest.json&mode=live&liveContract=$WEB_CONTRACT"
echo "LAN:  http://$(hostname -I | awk '{print $1}'):$WEB_PORT/?scene=/scenes/lingbot-live/manifest.json\\&mode=live\\&liveContract=$WEB_CONTRACT"
echo "live root: $OUTPUT_DIR"
echo "real2sim:  $REAL2SIM_DIR"

wait
