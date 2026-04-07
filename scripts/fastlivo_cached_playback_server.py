#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import functools
import json
import math
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from fastlivo_direct_playback_server import (
    CURRENT_SCAN_WORLD_TOPIC,
    GLOBAL_MAP_TOPIC,
    MapVoxel,
    create_rgb_cloud,
    trim_map_voxels,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Cached FAST-LIVO playback server.")
    parser.add_argument("--cache-dir", required=True)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--default-rate", type=float, default=5.0)
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--paused", action="store_true")
    parser.add_argument("--container-name", default="fastlivo-cached-playback")
    parser.add_argument("--image-name", default="gs_sdf_img:latest")
    parser.add_argument("--mjpeg-max-fps", type=float, default=60.0)
    return parser.parse_args()


@functools.lru_cache(maxsize=192)
def load_cloud_npz(path_str: str) -> tuple[np.ndarray, np.ndarray]:
    with np.load(path_str) as data:
        positions = data["positions"].astype(np.float32, copy=False)
        colors = data["colors"].astype(np.uint8, copy=False)
    return positions, colors


@functools.lru_cache(maxsize=192)
def load_binary_file(path_str: str) -> bytes:
    return Path(path_str).read_bytes()


class CachedPlaybackController:
    def __init__(self, cache_dir: Path, args: argparse.Namespace):
        self.cache_dir = cache_dir
        self.args = args
        self.meta = json.loads((cache_dir / "meta.json").read_text())
        self.frames: list[dict[str, Any]] = list(self.meta.get("frames", []))
        self.keyframes: list[dict[str, Any]] = list(self.meta.get("keyframes", []))
        self.frame_offsets = [float(frame["offsetSec"]) for frame in self.frames]
        self.keyframe_indices = [int(item["frameIndex"]) for item in self.keyframes]
        self.keyframe_files = [str((cache_dir / item["file"]).resolve()) for item in self.keyframes]
        self.duration_sec = float(self.meta.get("durationSec", 0.0))
        self.start_sec = float(self.meta.get("startSec", 0.0))
        self.end_sec = float(self.meta.get("endSec", self.start_sec + self.duration_sec))
        self.topics = {
            "/aft_mapped_to_init": {
                "type": "nav_msgs/Odometry",
                "count": len(self.frames),
            },
            CURRENT_SCAN_WORLD_TOPIC: {
                "type": "sensor_msgs/PointCloud2",
                "count": len(self.frames),
            },
            GLOBAL_MAP_TOPIC: {
                "type": "sensor_msgs/PointCloud2",
                "count": len(self.keyframes) if self.keyframes else len(self.frames),
            },
        }

        self.odom_pub = rospy.Publisher("/aft_mapped_to_init", Odometry, queue_size=8)
        self.current_scan_world_pub = rospy.Publisher(
            CURRENT_SCAN_WORLD_TOPIC, PointCloud2, queue_size=2
        )
        self.global_map_pub = rospy.Publisher(
            GLOBAL_MAP_TOPIC, PointCloud2, queue_size=1, latch=True
        )
        self.clock_pub = rospy.Publisher("/clock", Clock, queue_size=8)

        self.lock = threading.RLock()
        self.rate = float(args.default_rate)
        self.paused = bool(args.paused)
        self.loop = bool(args.loop)
        self.container_name = str(args.container_name)
        self.image_name = str(args.image_name)
        self.requested_offset_sec = max(0.0, min(float(args.start_offset), self.duration_sec))
        self.current_offset_sec = self.requested_offset_sec
        self.playback_alive = True
        self.control_version = 0
        self.session_start_wall = time.monotonic()
        self.session_start_offset = self.requested_offset_sec
        self.latest_image_jpeg: bytes | None = None
        self.latest_image_stamp_sec: float | None = None
        self.latest_pose_stamp_sec: float | None = None
        self.global_map_voxels: dict[tuple[int, int, int], MapVoxel] = {}
        self.global_map_point_count = 0
        self.current_scan_point_count = 0
        self.current_frame_index = -1
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def estimate_current_offset_sec(self) -> float:
        with self.lock:
            offset = self.current_offset_sec
            duration_sec = self.duration_sec
            if self.paused or not self.playback_alive:
                return max(0.0, min(offset, duration_sec))

            elapsed = max(0.0, time.monotonic() - self.session_start_wall)
            estimated = self.session_start_offset + elapsed * max(self.rate, 0.1)
            if self.loop and duration_sec > 0:
                estimated %= duration_sec
            else:
                estimated = min(estimated, duration_sec)
            return max(offset, estimated)

    def status(self) -> dict[str, Any]:
        with self.lock:
            return {
                "bagPath": str(self.meta.get("bagPath", self.cache_dir)),
                "containerName": self.container_name,
                "image": self.image_name,
                "rate": self.rate,
                "paused": self.paused,
                "loop": self.loop,
                "requestedOffsetSec": self.requested_offset_sec,
                "currentOffsetSec": self.current_offset_sec,
                "estimatedOffsetSec": self.estimate_current_offset_sec(),
                "durationSec": self.duration_sec,
                "bagStartSec": self.start_sec,
                "bagEndSec": self.end_sec,
                "playbackAlive": self.playback_alive,
                "latestImageStampSec": self.latest_image_stamp_sec,
                "latestPoseStampSec": self.latest_pose_stamp_sec,
                "globalMapPoints": self.global_map_point_count,
                "currentScanPoints": self.current_scan_point_count,
                "topics": dict(self.topics),
                "mode": "cache",
                "cacheDir": str(self.cache_dir),
                "frameIndex": self.current_frame_index,
                "frameCount": len(self.frames),
                "keyframeCount": len(self.keyframes),
            }

    def apply_control(self, payload: dict[str, Any]) -> dict[str, Any]:
        with self.lock:
            if "rate" in payload and payload["rate"] is not None:
                self.rate = max(0.1, float(payload["rate"]))
            if "paused" in payload and payload["paused"] is not None:
                self.paused = bool(payload["paused"])
            if "offsetSec" in payload and payload["offsetSec"] is not None:
                self.requested_offset_sec = max(0.0, min(float(payload["offsetSec"]), self.duration_sec))
                self.current_offset_sec = self.requested_offset_sec
            else:
                self.requested_offset_sec = self.current_offset_sec
            self.control_version += 1
            self.session_start_offset = self.requested_offset_sec
            self.session_start_wall = time.monotonic()
            return self.status()

    def close(self) -> None:
        self.stop_event.set()
        self.worker.join(timeout=2.0)

    def get_latest_jpeg_frame(self) -> tuple[float | None, bytes | None]:
        with self.lock:
            return self.latest_image_stamp_sec, self.latest_image_jpeg

    def offset_to_frame_index(self, offset_sec: float) -> int:
        if not self.frame_offsets:
            return -1
        clamped = max(0.0, min(offset_sec, self.duration_sec))
        index = bisect.bisect_right(self.frame_offsets, clamped) - 1
        return max(0, min(index, len(self.frame_offsets) - 1))

    def restore_map_from_keyframe(self, frame_index: int) -> int:
        self.global_map_voxels = {}
        if not self.keyframe_indices:
            return -1

        keyframe_slot = bisect.bisect_right(self.keyframe_indices, frame_index) - 1
        if keyframe_slot < 0:
            return -1

        keyframe_frame_index = self.keyframe_indices[keyframe_slot]
        positions, colors = load_cloud_npz(self.keyframe_files[keyframe_slot])
        stamp_sec = float(self.frames[keyframe_frame_index]["stampSec"])

        for index in range(positions.shape[0]):
            x = float(positions[index, 0])
            y = float(positions[index, 1])
            z = float(positions[index, 2])
            key = (
                int(round(x / max(float(self.meta["mapVoxelSize"]), 1e-3))),
                int(round(y / max(float(self.meta["mapVoxelSize"]), 1e-3))),
                int(round(z / max(float(self.meta["mapVoxelSize"]), 1e-3))),
            )
            self.global_map_voxels[key] = MapVoxel(
                x=x,
                y=y,
                z=z,
                position_samples=1,
                color=colors[index].astype(np.float64),
                color_samples=1,
                last_seen_stamp_sec=stamp_sec,
            )

        self.global_map_point_count = len(self.global_map_voxels)
        return keyframe_frame_index

    def update_global_map_from_scan(
        self,
        positions: np.ndarray,
        colors: np.ndarray,
        stamp_sec: float,
    ) -> None:
        voxel_size = max(float(self.meta.get("mapVoxelSize", 0.2)), 1e-3)
        max_points = int(self.meta.get("mapMaxPoints", 90_000))
        voxels = self.global_map_voxels
        for index in range(positions.shape[0]):
            x = float(positions[index, 0])
            y = float(positions[index, 1])
            z = float(positions[index, 2])
            key = (
                int(round(x / voxel_size)),
                int(round(y / voxel_size)),
                int(round(z / voxel_size)),
            )
            existing = voxels.get(key)
            if existing is None:
                voxels[key] = MapVoxel(
                    x=x,
                    y=y,
                    z=z,
                    position_samples=1,
                    color=colors[index].astype(np.float64),
                    color_samples=1,
                    last_seen_stamp_sec=stamp_sec,
                )
                continue

            existing.position_samples += 1
            mix = 1.0 / float(existing.position_samples)
            existing.x += (x - existing.x) * mix
            existing.y += (y - existing.y) * mix
            existing.z += (z - existing.z) * mix
            existing.last_seen_stamp_sec = stamp_sec
            if existing.color is None:
                existing.color = colors[index].astype(np.float64)
                existing.color_samples = 1
            else:
                existing.color_samples += 1
                color_mix = 1.0 / float(existing.color_samples)
                existing.color += (colors[index].astype(np.float64) - existing.color) * color_mix

        trim_map_voxels(voxels, max_points)
        self.global_map_point_count = len(voxels)

    def publish_global_map(self, stamp_sec: float) -> None:
        voxels = list(self.global_map_voxels.values())
        if not voxels:
            return

        positions = np.empty((len(voxels), 3), dtype=np.float32)
        colors = np.empty((len(voxels), 3), dtype=np.uint8)
        for index, voxel in enumerate(voxels):
            positions[index, 0] = voxel.x
            positions[index, 1] = voxel.y
            positions[index, 2] = voxel.z
            if voxel.color is None:
                colors[index] = np.array([216, 236, 255], dtype=np.uint8)
            else:
                colors[index] = np.clip(np.round(voxel.color), 0, 255).astype(np.uint8)

        header = Header(frame_id="world", stamp=rospy.Time.from_sec(stamp_sec))
        self.global_map_pub.publish(create_rgb_cloud(header, positions, colors))

    def publish_pose(self, frame: dict[str, Any]) -> None:
        pose = frame["pose"]
        stamp_sec = float(frame["stampSec"])
        message = Odometry()
        message.header.stamp = rospy.Time.from_sec(stamp_sec)
        message.header.frame_id = str(pose.get("frameId") or "world")
        message.child_frame_id = str(pose.get("childFrameId") or "base_link")
        position = pose.get("position") or [0.0, 0.0, 0.0]
        orientation = pose.get("orientation") or [0.0, 0.0, 0.0, 1.0]
        message.pose.pose.position.x = float(position[0])
        message.pose.pose.position.y = float(position[1])
        message.pose.pose.position.z = float(position[2])
        message.pose.pose.orientation.x = float(orientation[0])
        message.pose.pose.orientation.y = float(orientation[1])
        message.pose.pose.orientation.z = float(orientation[2])
        message.pose.pose.orientation.w = float(orientation[3])
        self.odom_pub.publish(message)
        self.latest_pose_stamp_sec = float(pose.get("stampSec") or stamp_sec)

    def publish_scan(self, stamp_sec: float, positions: np.ndarray, colors: np.ndarray) -> None:
        header = Header(frame_id="world", stamp=rospy.Time.from_sec(stamp_sec))
        self.current_scan_world_pub.publish(create_rgb_cloud(header, positions, colors))
        self.current_scan_point_count = int(positions.shape[0])

    def set_latest_image(self, frame: dict[str, Any]) -> None:
        image_rel = frame.get("imageFile")
        if image_rel:
            self.latest_image_jpeg = load_binary_file(str((self.cache_dir / str(image_rel)).resolve()))
            self.latest_image_stamp_sec = (
                float(frame.get("imageStampSec")) if frame.get("imageStampSec") is not None else None
            )
        else:
            self.latest_image_jpeg = None
            self.latest_image_stamp_sec = None

    def restore_to_frame(self, frame_index: int) -> None:
        if not self.frames:
            return

        frame_index = max(0, min(frame_index, len(self.frames) - 1))
        restored_keyframe_index = self.restore_map_from_keyframe(frame_index)
        start_index = restored_keyframe_index + 1
        current_positions: np.ndarray | None = None
        current_colors: np.ndarray | None = None
        current_frame = self.frames[frame_index]

        for index in range(start_index, frame_index + 1):
            frame = self.frames[index]
            positions, colors = load_cloud_npz(str((self.cache_dir / frame["scanFile"]).resolve()))
            self.update_global_map_from_scan(positions, colors, float(frame["stampSec"]))
            if index == frame_index:
                current_positions = positions
                current_colors = colors

        if current_positions is None or current_colors is None:
            current_positions, current_colors = load_cloud_npz(
                str((self.cache_dir / current_frame["scanFile"]).resolve())
            )

        stamp_sec = float(current_frame["stampSec"])
        self.clock_pub.publish(Clock(clock=rospy.Time.from_sec(stamp_sec)))
        self.publish_pose(current_frame)
        self.publish_scan(stamp_sec, current_positions, current_colors)
        self.publish_global_map(stamp_sec)
        self.set_latest_image(current_frame)
        self.current_frame_index = frame_index
        self.current_offset_sec = float(current_frame["offsetSec"])
        self.requested_offset_sec = self.current_offset_sec
        self.playback_alive = True

    def run(self) -> None:
        if not self.frames:
            return

        current_version = -1
        session_start_wall = 0.0
        session_start_offset = 0.0
        needs_restore = True

        while not self.stop_event.is_set() and not rospy.is_shutdown():
            with self.lock:
                paused = self.paused
                version = self.control_version
                requested_offset = self.requested_offset_sec
                rate = self.rate
                loop = self.loop

            if needs_restore or version != current_version:
                target_index = self.offset_to_frame_index(requested_offset)
                with self.lock:
                    self.restore_to_frame(target_index)
                    current_version = version
                    session_start_offset = self.current_offset_sec
                    session_start_wall = time.monotonic()
                    self.session_start_offset = session_start_offset
                    self.session_start_wall = session_start_wall
                needs_restore = False

            if paused:
                time.sleep(0.05)
                continue

            next_index = self.current_frame_index + 1
            if next_index >= len(self.frames):
                with self.lock:
                    if loop:
                        self.requested_offset_sec = 0.0
                        self.current_offset_sec = 0.0
                        self.control_version += 1
                    else:
                        self.paused = True
                        self.playback_alive = False
                needs_restore = True
                continue

            next_frame = self.frames[next_index]
            target_wall = session_start_wall + max(
                0.0,
                (float(next_frame["offsetSec"]) - session_start_offset) / max(rate, 0.1),
            )
            while time.monotonic() < target_wall:
                if self.stop_event.wait(0.002):
                    return
                with self.lock:
                    if self.control_version != current_version or self.paused:
                        needs_restore = True
                        break
            if needs_restore:
                continue

            positions, colors = load_cloud_npz(str((self.cache_dir / next_frame["scanFile"]).resolve()))
            stamp_sec = float(next_frame["stampSec"])
            with self.lock:
                self.clock_pub.publish(Clock(clock=rospy.Time.from_sec(stamp_sec)))
                self.publish_pose(next_frame)
                self.publish_scan(stamp_sec, positions, colors)
                self.update_global_map_from_scan(positions, colors, stamp_sec)
                map_publish_every = max(int(self.meta.get("keyframeEvery", 64) // 8), 1)
                if next_index <= 1 or next_index % map_publish_every == 0:
                    self.publish_global_map(stamp_sec)
                self.set_latest_image(next_frame)
                self.current_frame_index = next_index
                self.current_offset_sec = float(next_frame["offsetSec"])
                self.requested_offset_sec = self.current_offset_sec
                self.playback_alive = True


class Handler(BaseHTTPRequestHandler):
    controller: CachedPlaybackController

    def do_OPTIONS(self) -> None:
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_common_headers()
        self.end_headers()

    def do_GET(self) -> None:
        if self.path == "/status":
            self.respond_json(self.controller.status())
            return
        if self.path == "/topics":
            self.respond_json({"topics": self.controller.topics})
            return
        if self.path.startswith("/frame.jpg"):
            self.respond_jpeg_frame()
            return
        if self.path.startswith("/frame.mjpeg"):
            self.respond_mjpeg_stream()
            return
        self.respond_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        if self.path != "/control":
            self.respond_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)
            return
        content_length = int(self.headers.get("Content-Length", "0"))
        payload = json.loads(self.rfile.read(content_length) or b"{}")
        self.respond_json(self.controller.apply_control(payload))

    def respond_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_common_headers()
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def respond_jpeg_frame(self) -> None:
        stamp_sec, frame = self.controller.get_latest_jpeg_frame()
        if frame is None:
            self.respond_json({"error": "No image frame available yet."}, HTTPStatus.SERVICE_UNAVAILABLE)
            return

        self.send_response(HTTPStatus.OK)
        self.send_common_headers()
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("X-Frame-Stamp", str(stamp_sec or 0.0))
        self.send_header("Content-Length", str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def respond_mjpeg_stream(self) -> None:
        boundary = "frame"
        self.send_response(HTTPStatus.OK)
        self.send_common_headers()
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.end_headers()

        last_stamp_sec: float | None = None
        last_frame: bytes | None = None

        try:
            while True:
                stamp_sec, frame = self.controller.get_latest_jpeg_frame()
                if frame is None:
                    if self.controller.stop_event.wait(0.05):
                        return
                    continue

                if last_frame is None or stamp_sec != last_stamp_sec:
                    last_stamp_sec = stamp_sec
                    last_frame = frame

                self.wfile.write(f"--{boundary}\r\n".encode("ascii"))
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(last_frame)}\r\n".encode("ascii"))
                self.wfile.write(f"X-Frame-Stamp: {last_stamp_sec or 0.0}\r\n\r\n".encode("ascii"))
                self.wfile.write(last_frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()

                mjpeg_max_fps = max(0.0, float(getattr(self.controller.args, "mjpeg_max_fps", 60.0)))
                if mjpeg_max_fps > 0.0:
                    if self.controller.stop_event.wait(1.0 / mjpeg_max_fps):
                        return
                elif self.controller.stop_event.wait(0.005):
                    return
        except (BrokenPipeError, ConnectionResetError):
            return

    def send_common_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def log_message(self, format: str, *args: Any) -> None:
        return


def main() -> None:
    args = parse_args()
    cache_dir = Path(args.cache_dir).resolve()
    if not (cache_dir / "meta.json").exists():
        raise SystemExit(f"Cache metadata missing: {cache_dir / 'meta.json'}")

    rospy.init_node("fastlivo_cached_playback", anonymous=False, disable_signals=True)
    controller = CachedPlaybackController(cache_dir, args)
    Handler.controller = controller
    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    try:
        rospy.loginfo("cached playback ready on http://0.0.0.0:%s", args.port)
        server.serve_forever()
    finally:
        controller.close()


if __name__ == "__main__":
    main()
