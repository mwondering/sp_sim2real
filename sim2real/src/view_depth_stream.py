"""Attachable viewer for the policy depth ZMQ stream."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

SRC_ROOT = Path(__file__).resolve().parent
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from runtime.zmq_stream import ArraySubscriber


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="View the (1,36,64) policy depth stream"
    )
    parser.add_argument("--connect", default="tcp://127.0.0.1:28811")
    parser.add_argument("--timeout-s", type=float, default=0.5)
    args = parser.parse_args(argv)
    try:
        import cv2
    except (ImportError, ModuleNotFoundError) as exc:
        raise RuntimeError(
            "view_depth_stream.py requires opencv-python"
        ) from exc

    subscriber = ArraySubscriber(args.connect, topic="depth")
    last_seq = None
    last_frame_time = time.monotonic()
    print(
        f"[DepthViewer] depth<-{args.connect}; q/Esc=quit, "
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
            depth = np.asarray(packet.values, dtype=np.float32)
            if depth.shape != (1, 36, 64):
                raise ValueError(
                    f"Depth stream shape {depth.shape} != (1,36,64)"
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
    finally:
        subscriber.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
