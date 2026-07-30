"""Independent D435i producer for teleop-upper-lower-locomani sim2real."""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import numpy as np
import yaml

SRC_ROOT = Path(__file__).resolve().parent
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from runtime.d435i_source import D435iSource
from runtime.depth_pipeline import RealDepthProcessor
from runtime.zmq_stream import ArrayPublisher


def _load(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
    if not isinstance(config, dict):
        raise ValueError(f"Task config must be a mapping: {path}")
    return config


class _Viewer:
    def __init__(self, enabled: bool) -> None:
        self.enabled = bool(enabled)
        self.cv2 = None
        if self.enabled:
            try:
                import cv2
            except (ImportError, ModuleNotFoundError) as exc:
                raise RuntimeError(
                    "--show-depth 需要 opencv-python；相机发布本身不依赖它"
                ) from exc
            self.cv2 = cv2

    def show(self, depth: np.ndarray) -> bool:
        if not self.enabled:
            return True
        image = np.asarray(depth[0], dtype=np.float32)
        display = np.clip(image, 0.0, 1.0)
        display = (255.0 * (1.0 - display)).astype(np.uint8)
        invalid = (image < 0.0).astype(np.uint8)
        display = self.cv2.resize(
            display, (640, 360), interpolation=self.cv2.INTER_NEAREST
        )
        invalid = self.cv2.resize(
            invalid, (640, 360), interpolation=self.cv2.INTER_NEAREST
        )
        display = self.cv2.applyColorMap(display, self.cv2.COLORMAP_TURBO)
        display[invalid != 0] = 0
        self.cv2.imshow("D435i policy depth (near=red, invalid=black)", display)
        return (self.cv2.waitKey(1) & 0xFF) not in (27, ord("q"))

    def close(self) -> None:
        if self.cv2 is not None:
            self.cv2.destroyAllWindows()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="D435i depth publisher for teleop-upper-lower-locomani"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=SRC_ROOT.parent
        / "config/g1/teleop-upper-lower-locomani-real.yaml",
    )
    parser.add_argument("--serial-number", default=None)
    parser.add_argument("--worker-python", default=None)
    parser.add_argument("--show-depth", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args(argv)

    config_path = args.config.expanduser().resolve()
    config = _load(config_path)
    if str(config.get("target", "real")) != "real":
        raise ValueError("depth_camera_real requires a task config with target: real")
    camera_cfg = config["camera_process"]
    hardware = camera_cfg["hardware"]
    preprocess = camera_cfg["preprocess"]
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
    processor = RealDepthProcessor(
        min_distance=float(preprocess.get("min_distance_m", 0.05)),
        normalization_max_distance=float(
            preprocess.get("normalization_max_distance_m", 2.0)
        ),
        invalid_value=float(preprocess.get("invalid_value", -1.0)),
        valid_threshold=float(preprocess.get("valid_threshold", 0.5)),
        crop_top=int(preprocess.get("crop_top", 10)),
        crop_bottom=int(preprocess.get("crop_bottom", 0)),
        crop_left=int(preprocess.get("crop_left", 10)),
        crop_right=int(preprocess.get("crop_right", 10)),
        edge_fill_left_columns=int(
            preprocess.get("edge_fill_left_columns", 24)
        ),
    )
    publisher = ArrayPublisher(str(camera_cfg["depth_bind"]), topic="depth")
    viewer = _Viewer(args.show_depth)
    stopped = False

    def _stop(_signum, _frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    published = 0
    last_report = time.monotonic()
    last_timeout_warning = float("-inf")
    print(
        f"[DepthReal] D435i({source.mode}) -> {camera_cfg['depth_bind']} "
        "shape=(1,36,64); mount=pelvis, pitch_down=60deg"
    )
    try:
        while not stopped:
            frame = source.capture()
            if frame is None:
                now = time.monotonic()
                if now - last_timeout_warning >= 1.0:
                    print(
                        "[DepthReal] warning: camera frame timeout", flush=True
                    )
                    last_timeout_warning = now
                continue
            depth, stats = processor.process(frame.depth_m)
            publisher.send(
                depth,
                seq=published,
                sim_time=frame.capture_monotonic,
                metadata={
                    "source": "d435i",
                    "capture_monotonic": frame.capture_monotonic,
                    "serial_number": source.device_serial,
                    "invalid_value": processor.invalid_value,
                    **stats,
                },
            )
            published += 1
            if not viewer.show(depth):
                break
            now = time.monotonic()
            if now - last_report >= 1.0:
                print(
                    "[DepthReal] "
                    f"frames={published} invalid={stats['invalid_fraction']:.1%} "
                    f"range=[{stats['normalized_min']:.3f},"
                    f"{stats['normalized_max']:.3f}]"
                )
                last_report = now
            if args.max_frames > 0 and published >= args.max_frames:
                break
    finally:
        viewer.close()
        publisher.close()
        source.close()
        print(f"[DepthReal] stopped after {published} frames")


if __name__ == "__main__":
    main()
