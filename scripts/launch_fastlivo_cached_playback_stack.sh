#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CACHE_DIR="${1:-$ROOT_DIR/runtime/playback-cache/fast_livo2_compressed}"
PLAYBACK_CONTAINER="${2:-fastlivo-cached-playback}"
ROSBRIDGE_PORT="${3:-9090}"
WEB_PORT="${4:-5173}"
CONTROL_PORT="${5:-8765}"

CONTROL_PORT="$CONTROL_PORT" bash "$ROOT_DIR/scripts/launch_fastlivo_cached_playback.sh" "$CACHE_DIR" "$PLAYBACK_CONTAINER"
bash "$ROOT_DIR/scripts/launch_rosbridge_sidecar.sh" "$PLAYBACK_CONTAINER" "$ROSBRIDGE_PORT"
bash "$ROOT_DIR/scripts/launch_web_ui_dev.sh" 0.0.0.0 "$WEB_PORT"
