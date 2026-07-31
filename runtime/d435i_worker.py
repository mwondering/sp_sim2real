#!/usr/bin/env python3
from __future__ import annotations

import argparse
from io import BytesIO
import json
import math
import struct
import sys

import numpy as np


def _send_payload(stdout, payload: bytes) -> None:
    stdout.write(struct.pack("<I", len(payload)))
    stdout.write(payload)
    stdout.flush()


def _send_metadata(stdout, metadata: dict[str, object]) -> None:
    payload = json.dumps(metadata, separators=(",", ":")).encode("utf-8")
    _send_payload(stdout, payload)


def _send(stdout, depth_raw: np.ndarray | None) -> None:
    if depth_raw is None:
        stdout.write(struct.pack("<I", 0))
        stdout.flush()
        return
    buffer = BytesIO()
    np.save(buffer, np.asarray(depth_raw, dtype=np.uint16), allow_pickle=False)
    _send_payload(stdout, buffer.getvalue())


def main() -> None:
    parser = argparse.ArgumentParser(description="D435i raw-depth worker")
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--fps", type=int, required=True)
    parser.add_argument("--timeout-s", type=float, required=True)
    parser.add_argument("--serial-number", default="")
    args = parser.parse_args()

    try:
        import pyrealsense2 as rs
    except (ImportError, ModuleNotFoundError) as exc:
        raise RuntimeError(
            f"worker Python cannot import pyrealsense2: {exc}"
        ) from exc

    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial_number:
        config.enable_device(args.serial_number)
    config.enable_stream(
        rs.stream.depth,
        args.width,
        args.height,
        rs.format.z16,
        args.fps,
    )
    stdin = sys.stdin.buffer
    stdout = sys.stdout.buffer
    profile = pipeline.start(config)
    device = profile.get_device()
    scale = float(device.first_depth_sensor().get_depth_scale())
    video = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    intrinsics = video.get_intrinsics()
    fov_x = math.degrees(
        2.0 * math.atan(intrinsics.width / (2.0 * intrinsics.fx))
    )
    fov_y = math.degrees(
        2.0 * math.atan(intrinsics.height / (2.0 * intrinsics.fy))
    )
    _send_metadata(
        stdout,
        {
            "protocol_version": 2,
            "pixel_format": "z16",
            "device_name": str(device.get_info(rs.camera_info.name)),
            "serial_number": str(
                device.get_info(rs.camera_info.serial_number)
            ),
            "depth_scale": scale,
            "width": int(intrinsics.width),
            "height": int(intrinsics.height),
            "fps": int(video.fps()),
            "fov_x_deg": fov_x,
            "fov_y_deg": fov_y,
        },
    )
    try:
        while True:
            command = stdin.readline()
            if not command or command.strip() == b"Q":
                break
            if command.strip() != b"R":
                continue
            try:
                frames = pipeline.wait_for_frames(
                    max(1, int(args.timeout_s * 1000.0))
                )
                frame = frames.get_depth_frame()
                depth_raw = (
                    np.asarray(frame.get_data(), dtype=np.uint16).copy()
                    if frame
                    else None
                )
            except Exception:
                depth_raw = None
            _send(stdout, depth_raw)
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
