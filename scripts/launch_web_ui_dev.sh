#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
WEB_UI_DIR="$ROOT_DIR/examples/web-ui"
HOST="${1:-0.0.0.0}"
PORT="${2:-5173}"

EXISTING_PORT_PIDS="$(lsof -tiTCP:$PORT -sTCP:LISTEN 2>/dev/null || true)"
if [[ -n "$EXISTING_PORT_PIDS" ]]; then
  echo "Stopping existing server on port $PORT: $EXISTING_PORT_PIDS"
  kill $EXISTING_PORT_PIDS >/dev/null 2>&1 || true
  sleep 1
fi

cd "$WEB_UI_DIR"
npm run dev -- --host "$HOST" --port "$PORT"
