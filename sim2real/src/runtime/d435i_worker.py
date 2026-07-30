#!/usr/bin/env python3
from __future__ import annotations

import argparse
from io import BytesIO
import struct
import sys

import numpy as np


def _send(stdout, depth_m: np.ndarray | None) -> None:
    if depth_m is None:
        stdout.write(struct.pack("<I", 0))
        stdout.flush()
        return
    buffer = BytesIO()
    np.save(buffer, np.asarray(depth_m, dtype=np.float32), allow_pickle=False)
    payload = buffer.getvalue()
    stdout.write(struct.pack("<I", len(payload)))
    stdout.write(payload)
    stdout.flush()


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
    profile = pipeline.start(config)
    scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
    stdin = sys.stdin.buffer
    stdout = sys.stdout.buffer
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
                depth_m = (
                    np.asanyarray(frame.get_data()).astype(np.float32) * scale
                    if frame
                    else None
                )
            except Exception:
                depth_m = None
            _send(stdout, depth_m)
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
