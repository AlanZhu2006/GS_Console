#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${GS_SDF_IMAGE:-gs_sdf_img:latest}"
OUTPUT_DIR="${1:-$ROOT_DIR/runtime/output/2026-04-02-23-01-07_fast_livo2_compressed.bag_fastlivo_cbd_host.yaml}"
CONTAINER_NAME="${2:-gssdf-view}"

if [[ ! -d "$OUTPUT_DIR" ]]; then
  echo "Output dir does not exist: $OUTPUT_DIR" >&2
  exit 1
fi

if [[ ! -f "$OUTPUT_DIR/model/local_map_checkpoint.pt" ]]; then
  echo "Checkpoint missing under $OUTPUT_DIR/model/local_map_checkpoint.pt" >&2
  exit 1
fi

OUTPUT_ABS="$(realpath "$OUTPUT_DIR")"
OUTPUT_PARENT="$(dirname "$OUTPUT_ABS")"
CONFIG_PATH="$OUTPUT_ABS/model/config/scene/config.yaml"
DATA_PATH=""
DATA_MOUNT_ARGS=()
MODEL_DIR="$OUTPUT_ABS/model"
COMPAT_MODEL_DIR="$MODEL_DIR/model"

if [[ -f "$CONFIG_PATH" ]]; then
  DATA_PATH="$(sed -n 's/^data_path: "\(.*\)"/\1/p' "$CONFIG_PATH" | head -n 1)"
fi

if [[ -n "$DATA_PATH" ]]; then
  if [[ ! -e "$DATA_PATH" ]]; then
    echo "Saved view config refers to missing data_path: $DATA_PATH" >&2
    exit 1
  fi
  DATA_PARENT="$(dirname "$(realpath "$DATA_PATH")")"
  DATA_MOUNT_ARGS=(-v "$DATA_PARENT:$DATA_PARENT")
fi

# Some GS-SDF view-mode builds resolve occupancy assets under model/model/.
# Prepare those compatibility symlinks inside a root container because the
# training outputs are commonly owned by root on the host.
docker run --rm \
  -v "$OUTPUT_PARENT:$OUTPUT_PARENT" \
  "$IMAGE" \
  bash -lc "set -euo pipefail; \
    mkdir -p \"$COMPAT_MODEL_DIR\"; \
    for asset in as_occ_prior.ply gs.ply local_map_checkpoint.pt; do \
      if [[ -f \"$MODEL_DIR/\$asset\" && ! -e \"$COMPAT_MODEL_DIR/\$asset\" ]]; then \
        ln -s \"../\$asset\" \"$COMPAT_MODEL_DIR/\$asset\"; \
      fi; \
    done"

docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true

echo "Image:      $IMAGE"
echo "Output dir: $OUTPUT_ABS"
echo "Container:  $CONTAINER_NAME"
if [[ -n "$DATA_PATH" ]]; then
  echo "Data path:   $DATA_PATH"
fi

docker run -d --rm \
  --name "$CONTAINER_NAME" \
  --gpus all \
  --shm-size=16g \
  -v "$ROOT_DIR:$ROOT_DIR" \
  -v "$OUTPUT_PARENT:$OUTPUT_PARENT" \
  "${DATA_MOUNT_ARGS[@]}" \
  "$IMAGE" \
  bash -lc "set -eo pipefail; \
    source /opt/ros/noetic/setup.bash; \
    source /root/gs_sdf_ws/devel/setup.bash; \
    roscore >/tmp/roscore.log 2>&1 & \
    ROSCORE_PID=\$!; \
    trap 'kill \$ROSCORE_PID >/dev/null 2>&1 || true' EXIT; \
    sleep 3; \
    /root/gs_sdf_ws/devel/lib/neural_mapping/neural_mapping_node view \"$OUTPUT_ABS\""

echo "Started $CONTAINER_NAME"
echo "Tail logs with: docker logs -f $CONTAINER_NAME"
