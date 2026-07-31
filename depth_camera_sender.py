"""Standalone G1-side D435i raw Z16 publisher."""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from runtime.d435i_source import D435iSource
from runtime.zmq_stream import ArrayPublisher


def _load(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
    if not isinstance(config, dict):
        raise ValueError(f"Camera config must be a mapping: {path}")
    return config


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="G1-side D435i raw Z16 depth publisher"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "camera.yaml",
    )
    parser.add_argument("--serial-number", default=None)
    parser.add_argument("--worker-python", default=None)
    parser.add_argument(
        "--depth-bind",
        default=None,
        help="Override camera_process.depth_bind",
    )
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args(argv)

    config = _load(args.config.expanduser().resolve())
    camera_cfg = config["camera_process"]
    if args.depth_bind is not None:
        camera_cfg["depth_bind"] = str(args.depth_bind)
    hardware = camera_cfg["hardware"]
    source = D435iSource(
        width=int(hardware.get("width", 640)),
        height=int(hardware.get("height", 360)),
        fps=int(hardware.get("fps", 30)),
        serial_number=(
            args.serial_number
            if args.serial_number is not None
            else str(hardware.get("serial_number", ""))
        ),
        timeout_s=float(hardware.get("frame_timeout_s", 0.5)),
        worker_python=(
            args.worker_python
            if args.worker_python is not None
            else str(hardware.get("worker_python", ""))
        ),
        expected_fov_x_deg=float(hardware.get("expected_fov_x_deg", 89.04)),
        expected_fov_y_deg=float(hardware.get("expected_fov_y_deg", 57.9)),
        fov_tolerance_deg=float(hardware.get("fov_tolerance_deg", 6.0)),
    )
    publisher = ArrayPublisher(
        str(camera_cfg["depth_bind"]), topic="depth", dtype="uint16"
    )
    stopped = False

    def _stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    captured = 0
    last_report = time.monotonic()
    last_timeout_warning = float("-inf")
    print(
        f"[G1Depth] D435i({source.mode}) -> {camera_cfg['depth_bind']} "
        f"raw-z16 shape=({source.height},{source.width}); "
        f"scale={source.depth_scale:.6g}m/unit"
    )
    try:
        while not stopped:
            frame = source.capture()
            if frame is None:
                now = time.monotonic()
                if now - last_timeout_warning >= 1.0:
                    print("[G1Depth] warning: camera frame timeout", flush=True)
                    last_timeout_warning = now
                continue
            publisher.send(
                frame.depth_raw,
                seq=captured,
                sim_time=frame.capture_monotonic,
                metadata={
                    "protocol": "d435i-raw-z16-v1",
                    "source": "d435i",
                    "capture_monotonic": frame.capture_monotonic,
                    "serial_number": source.device_serial,
                    "pixel_format": "z16",
                    "depth_scale": source.depth_scale,
                    "width": source.width,
                    "height": source.height,
                    "fps": source.fps,
                    "fov_x_deg": source.fov_deg[0],
                    "fov_y_deg": source.fov_deg[1],
                },
            )
            captured += 1
            now = time.monotonic()
            if now - last_report >= 1.0:
                print(f"[G1Depth] raw frames={captured}")
                last_report = now
            if args.max_frames > 0 and captured >= args.max_frames:
                break
    finally:
        publisher.close()
        source.close()
        print(f"[G1Depth] stopped after {captured} frames")


if __name__ == "__main__":
    main()
