# Playback 1x Re-export Status

Date: 2026-04-14

## Current status

- Current playback video sidecar is still the old 8x entry:
  - [fast_livo2_compressed-ff0ecd06dda4.json](/home/chatsign/gs-sdf/runtime/playback-video-index/fast_livo2_compressed-ff0ecd06dda4.json)
  - `path=/media/chatsign/data-002/gs-sdf/runtime/videos/playback_rgb_vs_gssdf_quality_v2_synced.mp4`
  - `fps=12.0`
  - `speed=8.0`
- This sidecar was intentionally not changed to `1.0` yet.
- Reason: the current registered video is still the old 8x video. Changing only the sidecar speed would make playback timing incorrect.

## What was already fixed

- [export_playback_rgb_vs_gssdf_video.py](/home/chatsign/gs-sdf/scripts/export_playback_rgb_vs_gssdf_video.py) was updated so that:
  - default `--speed` is now `1.0`
  - duplicate source frames are preserved during 1x export
- The duplicate-frame fix matters because 1x export can hit the same cache frame multiple times. Dropping duplicates shortens the encoded video and breaks sidecar timing.

## Current blocker

- The GS-SDF HiFi render path can work, but it is currently unstable because GPU memory is almost fully occupied by an unrelated training job.
- Observed blocker process:
  - PID `2933300`
  - command:

```bash
/media/chatsign/data-0011/condaEnvs/ip_env/bin/python \
  /media/chatsign/data-002/zhewen/robo/instant_policy/ip/recurrent/train_aggregator.py \
  --model_path /media/chatsign/data-002/zhewen/robo/instant_policy/runs/v5_rlbench \
  --model_name best.pt \
  --data_dir /media/chatsign/data-002/zhewen/robo/instant_policy/data/rlbench_train_50_shards \
  --val_dir /media/chatsign/data-002/zhewen/robo/instant_policy/data/rlbench_val_50_shards \
  --save_dir /media/chatsign/data-002/zhewen/robo/instant_policy/runs/aggregator_v3 \
  --num_iters 27000 \
  --batch_size 64 \
  --lr 3e-4 \
  --v3 \
  --lambda_distill 1.0 \
  --lambda_anneal_steps 5000 \
  --save_interval 5000 \
  --val_interval 5000 \
  --device cuda
```

- At the time of debugging, this process was using about `45.5 GiB` of GPU memory.
- When the GS-SDF view container receives camera poses and starts rendering `/neural_mapping/rgb`, `neural_mapping_node view` can hit CUDA OOM and exit.
- That is why the 1x export could start, but could not finish.

## Do not do this

- Do not manually edit the sidecar speed from `8.0` to `1.0` while it still points to the old `playback_rgb_vs_gssdf_quality_v2_synced.mp4`.
- That would only relabel an 8x video as 1x and would reintroduce playback desync.

## Re-run checklist

### 1. Free enough GPU memory

Check current GPU usage:

```bash
nvidia-smi
```

If the unrelated training process is still occupying the GPU, stop it first:

```bash
kill -TERM 2933300
```

Confirm it is gone:

```bash
nvidia-smi --query-compute-apps=pid,process_name,used_gpu_memory --format=csv,noheader,nounits
```

### 2. Start the GS-SDF HiFi stack

Run:

```bash
VIEW_WIDTH=1920 VIEW_HEIGHT=1536 MJPEG_MAX_FPS=45 KEEP_CONTAINER_ON_EXIT=1 \
bash /home/chatsign/gs-sdf/scripts/launch_gssdf_hifi_stack.sh \
  /home/chatsign/gs-sdf/runtime/output/2026-04-03-17-16-16_fast_livo2_compressed.bag_fastlivo_cbd_quality_v2.yaml \
  gssdf-view-quality \
  8876 \
  gssdf-hifi-bridge
```

Wait for the bridge to become healthy:

```bash
curl -fsS http://127.0.0.1:8876/status
```

Healthy state should eventually show:

- `"ready": true`
- `"framesReceived"` increasing

If the bridge is pointing at the wrong source container, rebind it explicitly:

```bash
bash /home/chatsign/gs-sdf/scripts/launch_gssdf_hifi_bridge.sh \
  gssdf-view-quality \
  8876 \
  gssdf-hifi-bridge
```

### 3. Re-export the 1x compare video

Run:

```bash
python3 /home/chatsign/gs-sdf/scripts/export_playback_rgb_vs_gssdf_video.py \
  --cache-dir /home/chatsign/gs-sdf/runtime/playback-cache/fast_livo2_compressed \
  --scene-config /home/chatsign/gs-sdf/runtime/output/2026-04-03-17-16-16_fast_livo2_compressed.bag_fastlivo_cbd_quality_v2.yaml/model/config/scene/config.yaml \
  --bridge-url http://127.0.0.1:8876 \
  --output-video /home/chatsign/gs-sdf/runtime/videos/playback_rgb_vs_gssdf_quality_v2_1x.mp4 \
  --fps 12 \
  --speed 1.0 \
  --start-offset 0 \
  --end-offset -1
```

Notes:

- This export is long because it covers the full bag at 1x.
- The script will write the new sidecar automatically after a successful export.

### 4. Verify the new video and sidecar

Check the exported video:

```bash
ffprobe -v error \
  -show_entries format=duration:stream=avg_frame_rate,r_frame_rate,nb_frames,width,height \
  -of default=noprint_wrappers=1 \
  /home/chatsign/gs-sdf/runtime/videos/playback_rgb_vs_gssdf_quality_v2_1x.mp4
```

Check the sidecar:

```bash
cat /home/chatsign/gs-sdf/runtime/playback-video-index/fast_livo2_compressed-ff0ecd06dda4.json
```

Expected sidecar fields after success:

```json
{
  "path": "/home/chatsign/gs-sdf/runtime/videos/playback_rgb_vs_gssdf_quality_v2_1x.mp4",
  "fps": 12.0,
  "speed": 1.0,
  "startOffsetSec": 0.0,
  "endOffsetSec": -1.0
}
```

The actual path may resolve to `/media/chatsign/data-002/gs-sdf/...`; that is fine.

### 5. Restart cached playback so it reloads the new sidecar

Run:

```bash
VIEW_WIDTH=1920 VIEW_HEIGHT=1536 MJPEG_MAX_FPS=90 START_PAUSED=1 \
bash /home/chatsign/gs-sdf/scripts/launch_fastlivo_cached_playback_stack.sh \
  /home/chatsign/gs-sdf/runtime/playback-cache/fast_livo2_compressed \
  fastlivo-cached-playback \
  9090 \
  5173 \
  8765
```

Then verify:

```bash
curl -fsS http://127.0.0.1:8765/status
```

Expected:

- `"video"` is not `null`
- `"video.speed": 1.0`
- `"video.url": "video.mp4"`

## Quick summary

- Export script default is already fixed for 1x.
- Sidecar is intentionally still old because no valid 1x video has been produced yet.
- The remaining blocker is GPU memory pressure from an unrelated training process.
- Once GPU memory is available, run the HiFi stack, export the 1x video, verify the sidecar, then restart cached playback.
