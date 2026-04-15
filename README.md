# GS-SDF Web UI

This repository wraps the existing `GS-SDF` and `FAST-LIVO2` runtime into a browser-first console for:

- cached playback driven by a single master timeline
- 2D map editing and replay inspection
- side-by-side RGB vs GS-SDF compare video
- scene-pack based `GS` / `HiFi` / `Live` views
- remote access through `rosbridge` and a Vite web UI

This README is the current operational guide for this workspace. It is not a generic upstream GS-SDF manual.

## Current status

As of `2026-04-15`, the mainline that is working best in this repo is:

- playback-first workflow
- cached FAST-LIVO2 playback
- 1x compare video as the visual master timeline
- `kf16` playback cache for denser restore and lower seek lag

Active defaults in this workspace:

- active cache:
  `/home/chatsign/gs-sdf/runtime/playback-cache/fast_livo2_compressed_rgbclean_kf16`
- active 1x compare video:
  `/home/chatsign/gs-sdf/runtime/videos/playback_rgb_vs_gssdf_quality_v2_1x.mp4`
- active stack command:
  see `/home/chatsign/gs-sdf/command.txt`

What is implemented and verified:

- `Playback` mode with timeline, pause, seek, restart, speed control
- playback cache build from rosbag
- split overlay fetches for scan and accumulated map
- 2D map editing:
  `Goal`, `Initial Pose`, `Obstacle`, `Erase`, `Save PGM`
- `Workspace` and `Showcase` playback presentations
- compare video side-by-side:
  `Playback RGB` vs `GS-SDF Gaussian`
- 1x compare video export with sidecar metadata
- denser `kf16` playback cache
- browser-side `Live`, `Gs`, `HiFi` shell integration

Current boundaries:

- `Playback` is the best-supported path
- `Live` and browser `Gs` require WebGL in the browser session
- native `HiFi` depends on the GS-SDF native renderer and free GPU capacity
- high playback rates can still show some lag in accumulated map generation because the backend still reconstructs map snapshots on demand

## Recommended workflow

If your goal is to inspect data or demo results, use this path:

1. Build or reuse a cached playback pack.
2. Optionally export a 1x compare video.
3. Launch the cached playback stack.
4. Open `http://localhost:5173/?mode=playback`.

If your goal is to inspect Gaussian assets or native rendering, only then move to `Gs` or `HiFi`.

## Prerequisites

The current scripts assume the following are already available on the machine:

- Docker
- a working GPU Docker runtime
- image `gs_sdf_img:latest`
- ROS Noetic tooling inside the image
- `node` and `npm` on the host for the web UI
- `python3` on the host

For playback from a new bag, the bag should contain these topics:

- `/aft_mapped_to_init`
- `/cloud_registered_body`
- `/origin_img`

You can verify that with:

```bash
rosbag info /path/to/your_data.bag
```

For browser `Live` / `Gs` modes:

- the browser must support WebGL
- hardware acceleration should be enabled

For native `HiFi`:

- the GS-SDF view container must be able to publish `/neural_mapping/rgb`
- GPU memory pressure must be low enough for the renderer to stay alive

## Quick start for the current dataset

The current verified command is:

```bash
VIEW_WIDTH=1920 VIEW_HEIGHT=1536 MJPEG_MAX_FPS=90 START_PAUSED=1 \
bash /home/chatsign/gs-sdf/scripts/launch_fastlivo_cached_playback_stack.sh \
  /home/chatsign/gs-sdf/runtime/playback-cache/fast_livo2_compressed_rgbclean_kf16 \
  fastlivo-cached-playback \
  9090 \
  5173 \
  8765
```

Open:

```text
http://localhost:5173/?mode=playback
```

This starts:

- cached playback control API on `8765`
- rosbridge sidecar on `9090`
- web UI dev server on `5173`
- native HiFi stack if it can be resolved from the current scene manifest and `DISABLE_HIFI` is not set

## What the main scripts do

Core playback scripts:

- `/home/chatsign/gs-sdf/scripts/build_fastlivo_playback_cache.sh`
  builds a playback cache from a rosbag
- `/home/chatsign/gs-sdf/scripts/launch_fastlivo_cached_playback.sh`
  launches only the cached playback control server
- `/home/chatsign/gs-sdf/scripts/launch_fastlivo_cached_playback_stack.sh`
  launches cached playback, rosbridge, optional HiFi, and the web UI
- `/home/chatsign/gs-sdf/scripts/fastlivo_cached_playback_server.py`
  serves `/status`, `/overlay.json`, `/video_sync`, `/video.mp4`, `/frame.jpg`
- `/home/chatsign/gs-sdf/scripts/export_playback_rgb_vs_gssdf_video.py`
  exports the side-by-side compare video and writes `playback_video.json`

Scene / GS-SDF scripts:

- `/home/chatsign/gs-sdf/scripts/train_gssdf.sh`
  runs GS-SDF training or processing
- `/home/chatsign/gs-sdf/scripts/preprocess_gaussian_stream.mjs`
  prepares streamed Gaussian chunks
- `/home/chatsign/gs-sdf/scripts/sync_scene_pack.py`
  syncs a GS-SDF output directory into the web scene pack format
- `/home/chatsign/gs-sdf/scripts/launch_gssdf_hifi_stack.sh`
  starts native GS-SDF view plus MJPEG bridge

## Minimal onboarding for new data

This is the shortest path if you have new data and want it in the UI quickly.

### Path A: new rosbag, playback only

Use this if you want:

- timeline replay
- 2D map and robot trajectory
- scan and accumulated map
- optional compare video later

You do not need a full GS-SDF scene pack for this path.

#### Step 1: verify the bag

Make sure your bag contains the expected FAST-LIVO2 topics:

```bash
rosbag info /path/to/your_data.bag
```

Expected topics:

- `/aft_mapped_to_init`
- `/cloud_registered_body`
- `/origin_img`

If your topics are different, you should not force this pipeline unchanged. Adjust the playback extraction path first.

#### Step 2: build a playback cache

Recommended current build:

```bash
KEYFRAME_EVERY=16 \
bash /home/chatsign/gs-sdf/scripts/build_fastlivo_playback_cache.sh \
  /path/to/your_data.bag \
  /home/chatsign/gs-sdf/runtime/playback-cache/your_data_rgbclean_kf16
```

Why this is the recommended baseline:

- `KEYFRAME_EVERY=16` improves restore density compared with the older `64`
- current cache builder already writes cleaned scan colors under the current code path
- the `_rgbclean_kf16` suffix is a naming convention, not a special required flag

Useful cache build environment variables:

- `KEYFRAME_EVERY`
- `SCAN_MAX_POINTS`
- `SCAN_VOXEL_SIZE`
- `MAP_MAX_POINTS`
- `MAP_VOXEL_SIZE`
- `MAX_FRAMES`
- `CONFIG_PATH`

Example with explicit overrides:

```bash
CONFIG_PATH=/home/chatsign/gs-sdf/config/fastlivo_cbd_host.yaml \
KEYFRAME_EVERY=16 \
SCAN_MAX_POINTS=3600 \
MAP_MAX_POINTS=90000 \
bash /home/chatsign/gs-sdf/scripts/build_fastlivo_playback_cache.sh \
  /path/to/your_data.bag \
  /home/chatsign/gs-sdf/runtime/playback-cache/your_data_rgbclean_kf16
```

#### Step 3: launch the cached playback stack

```bash
VIEW_WIDTH=1920 VIEW_HEIGHT=1536 MJPEG_MAX_FPS=90 START_PAUSED=1 \
bash /home/chatsign/gs-sdf/scripts/launch_fastlivo_cached_playback_stack.sh \
  /home/chatsign/gs-sdf/runtime/playback-cache/your_data_rgbclean_kf16 \
  fastlivo-cached-playback \
  9090 \
  5173 \
  8765
```

Open:

```text
http://localhost:5173/?mode=playback
```

At this point you already have the minimum useful integration.

### Path B: add a 1x compare video for that playback cache

Use this if you want the side-by-side `Playback RGB` vs `GS-SDF Gaussian` compare video to be the master timeline.

This path needs:

- a playback cache
- a GS-SDF output directory for the same scene
- a working HiFi bridge

#### Step 1: start the HiFi stack

```bash
VIEW_WIDTH=1920 VIEW_HEIGHT=1536 MJPEG_MAX_FPS=45 KEEP_CONTAINER_ON_EXIT=1 \
bash /home/chatsign/gs-sdf/scripts/launch_gssdf_hifi_stack.sh \
  /path/to/your_gssdf_output_dir \
  gssdf-view-quality \
  8876 \
  gssdf-hifi-bridge
```

Health check:

```bash
curl -fsS http://127.0.0.1:8876/status
```

You want:

- `"ready": true`
- `"framesReceived"` increasing

#### Step 2: export the 1x compare video

```bash
python3 /home/chatsign/gs-sdf/scripts/export_playback_rgb_vs_gssdf_video.py \
  --cache-dir /home/chatsign/gs-sdf/runtime/playback-cache/your_data_rgbclean_kf16 \
  --scene-config /path/to/your_gssdf_output_dir/model/config/scene/config.yaml \
  --bridge-url http://127.0.0.1:8876 \
  --output-video /home/chatsign/gs-sdf/runtime/videos/your_data_rgb_vs_gssdf_1x.mp4 \
  --fps 12 \
  --speed 1.0 \
  --start-offset 0 \
  --end-offset -1
```

What this script also does:

- writes `/home/chatsign/gs-sdf/runtime/playback-cache/your_data_rgbclean_kf16/playback_video.json`
- writes a registry fallback under `/home/chatsign/gs-sdf/runtime/playback-video-index`

That means the cached playback server can pick up the video automatically on the next restart.

#### Step 3: restart the cached playback stack

```bash
VIEW_WIDTH=1920 VIEW_HEIGHT=1536 MJPEG_MAX_FPS=90 START_PAUSED=1 \
bash /home/chatsign/gs-sdf/scripts/launch_fastlivo_cached_playback_stack.sh \
  /home/chatsign/gs-sdf/runtime/playback-cache/your_data_rgbclean_kf16 \
  fastlivo-cached-playback \
  9090 \
  5173 \
  8765
```

Verify:

```bash
curl -fsS http://127.0.0.1:8765/status
```

You want:

- `"video"` not `null`
- `"video.speed": 1.0`
- `"video.url": "video.mp4"`

### Path C: full GS / HiFi scene integration

Use this if you want the new data to appear in `Gs`, `HiFi`, or scene-pack-backed `Live` workflows.

#### Step 1: run GS-SDF

```bash
bash /home/chatsign/gs-sdf/scripts/train_gssdf.sh \
  /path/to/your_data.bag \
  /home/chatsign/gs-sdf/config/fastlivo_cbd_host.yaml \
  your-run-name
```

Check status:

```bash
bash /home/chatsign/gs-sdf/scripts/gssdf_status.sh
```

Expected output directory pattern:

```text
/home/chatsign/gs-sdf/runtime/output/<timestamp>_<bag>_<config>
```

#### Step 2: preprocess the Gaussian asset

```bash
node /home/chatsign/gs-sdf/scripts/preprocess_gaussian_stream.mjs \
  --output-dir /path/to/your_gssdf_output_dir \
  --variant quality \
  --grid 6,6 \
  --max-sh 2 \
  --opacity-threshold 0.005 \
  --force
```

This produces streamed chunk metadata and `spz` chunks for the browser.

#### Step 3: sync the scene pack

```bash
python3 /home/chatsign/gs-sdf/scripts/sync_scene_pack.py \
  --output-dir /path/to/your_gssdf_output_dir \
  --scene-config /home/chatsign/gs-sdf/config/fastlivo_cbd_host.yaml \
  --scene-root /home/chatsign/gs-sdf/examples/web-ui/public/scenes \
  --scene-id your-scene-id \
  --status completed
```

After that, the web UI can load the new scene pack in `Gs` / `HiFi`.

## What files matter in a playback cache

A playback cache directory should contain at least:

- `meta.json`
- `scans/`
- `images/`
- `keyframes/`

Optional but recommended:

- `playback_video.json`

Key `meta.json` fields you will care about:

- `durationSec`
- `frameCount`
- `keyframeEvery`
- `scanMaxPoints`
- `mapMaxPoints`
- `configPath`
- `frames`

## Current recommended defaults

For new playback data in this repo, the current recommended defaults are:

- cache naming:
  `your_data_rgbclean_kf16`
- `KEYFRAME_EVERY=16`
- compare video:
  `1x`
- compare video fps:
  `12`
- stack start:
  `START_PAUSED=1`

Why:

- `kf16` makes random restore and dense playback more stable than `64`
- `1x` avoids timeline mismatch between video and bag
- `START_PAUSED=1` makes initial verification easier

## Verification checklist

After launching a new cache, verify these in order:

1. `curl -fsS http://127.0.0.1:8765/status`
2. `curl -fsS http://127.0.0.1:8765/topics`
3. open `http://localhost:5173/?mode=playback`
4. confirm timeline moves, 2D map appears, robot/trajectory update

If compare video is expected:

1. `curl -fsS http://127.0.0.1:8765/status`
2. confirm `"video"` is present
3. confirm UI no longer shows offline-video-missing state

## Troubleshooting

### `localhost:5173` does not open

The web UI is a Vite dev server. Common causes:

- the stack was not started
- port `5173` is already occupied locally
- when using SSH forwarding on Windows, `5173` may be in an excluded port range

If local forwarding is blocked, use a different local port such as `55173`.

### `Live` or `Gs` is black

Browser `Live` / `Gs` needs WebGL.

If the page says WebGL is unavailable:

- enable browser hardware acceleration
- use a browser/session with WebGL support
- if on remote desktop, use a mode that preserves GPU/WebGL

### `HiFi` is black

Native HiFi depends on the backend renderer, not just the web UI.

Check:

```bash
curl -fsS http://127.0.0.1:8876/status
```

If it is not ready, the native GS-SDF renderer is not producing frames.

### Playback is globally slow at high speed

This is usually backend throughput, not a pure front-end bug.

The current repo already uses:

- split scan/map overlay requests
- denser keyframes
- exact per-frame overlay map restore

If that is still not enough, the next step is precomputing denser playback map snapshots rather than tuning the browser further.

## Related docs

Additional docs in this repo:

- `/home/chatsign/gs-sdf/docs/new-data-onboarding.md`
- `/home/chatsign/gs-sdf/docs/playback_1x_reexport_status.md`
- `/home/chatsign/gs-sdf/docs/gs-sdf-ui-architecture.md`
- `/home/chatsign/gs-sdf/docs/web-integration.md`

Those are still useful, but this README is the current entry point for actually running the system.
