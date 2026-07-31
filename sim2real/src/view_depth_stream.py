"""Attachable viewer for the policy depth ZMQ stream."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml

SRC_ROOT = Path(__file__).resolve().parent
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from runtime.depth_pipeline import RealDepthProcessor
from runtime.zmq_stream import ArraySubscriber


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Process and view the D435i raw Z16 depth stream"
    )
    parser.add_argument("--connect", default="tcp://127.0.0.1:28811")
    parser.add_argument(
        "--config",
        type=Path,
        default=SRC_ROOT.parent
        / "config/g1/teleop-upper-lower-locomani-real.yaml",
    )
    parser.add_argument("--timeout-s", type=float, default=0.5)
    args = parser.parse_args(argv)
    with args.config.expanduser().resolve().open(
        "r", encoding="utf-8"
    ) as file:
        config = yaml.safe_load(file)
    camera_cfg = config["camera_process"]
    processor = RealDepthProcessor.from_config(camera_cfg["preprocess"])
    hardware_cfg = camera_cfg["hardware"]
    raw_shape = (
        int(hardware_cfg.get("height", 360)),
        int(hardware_cfg.get("width", 640)),
    )
    try:
        import cv2
    except (ImportError, ModuleNotFoundError) as exc:
        raise RuntimeError(
            "view_depth_stream.py requires opencv-python"
        ) from exc

    subscriber = ArraySubscriber(args.connect, topic="depth")
    last_seq = None
    last_frame_time = time.monotonic()
    last_report = last_frame_time
    print(
        f"[DepthViewer] raw depth<-{args.connect}; processing on this host; "
        "q/Esc=quit, "
        "near=red, far=blue, invalid=black"
    )
    try:
        while True:
            packet = subscriber.read_latest()
            if packet is None or packet.seq == last_seq:
                if time.monotonic() - last_frame_time > args.timeout_s:
                    print("[DepthViewer] warning: depth stream is stale", flush=True)
                    last_frame_time = time.monotonic()
                time.sleep(0.005)
                continue
            raw = np.asarray(packet.values)
            if packet.metadata.get("protocol") != "d435i-raw-z16-v1":
                raise ValueError(
                    "Depth stream is not d435i-raw-z16-v1"
                )
            if raw.dtype != np.uint16 or raw.shape != raw_shape:
                raise ValueError(
                    f"Raw depth {raw.dtype} {raw.shape} "
                    f"!= uint16 {raw_shape}"
                )
            depth, stats = processor.process_raw(
                raw,
                depth_scale=float(packet.metadata["depth_scale"]),
            )
            image = depth[0]
            display = (255.0 * (1.0 - np.clip(image, 0.0, 1.0))).astype(
                np.uint8
            )
            invalid = (image < 0.0).astype(np.uint8)
            display = cv2.resize(
                display, (640, 360), interpolation=cv2.INTER_NEAREST
            )
            invalid = cv2.resize(
                invalid, (640, 360), interpolation=cv2.INTER_NEAREST
            )
            display = cv2.applyColorMap(display, cv2.COLORMAP_TURBO)
            display[invalid != 0] = 0
            cv2.imshow("policy depth (1x36x64)", display)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            last_seq = packet.seq
            last_frame_time = time.monotonic()
            if last_frame_time - last_report >= 1.0:
                print(
                    "[DepthViewer] "
                    f"seq={packet.seq} "
                    f"invalid={stats['invalid_fraction']:.1%} "
                    f"range=[{stats['normalized_min']:.3f},"
                    f"{stats['normalized_max']:.3f}]"
                )
                last_report = last_frame_time
    finally:
        subscriber.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
