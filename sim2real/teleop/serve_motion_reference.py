#!/usr/bin/env python3
"""Serve a robot motion NPZ through the live G1 reference protocol."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import threading
import time

import numpy as np

from retarget.viser_viewer import MJViserViewer
from utils.helper import default_controller_buttons
from utils.robot_config import load_teleop_robot_config


PROTOCOL = "g1-reference-v1"


def _decode_names(values: np.ndarray) -> list[str]:
    return [
        value.decode("utf-8") if isinstance(value, (bytes, np.bytes_)) else str(value)
        for value in np.asarray(values).reshape(-1).tolist()
    ]


def load_motion(path: Path, target_joint_names: tuple[str, ...]) -> tuple[np.ndarray, float]:
    with np.load(path, allow_pickle=False) as data:
        required = {"root_pos", "root_rot", "dof_pos", "joint_names"}
        missing = sorted(required.difference(data.files))
        if missing:
            raise ValueError(f"motion is missing fields {missing}: {path}")

        root_pos = np.asarray(data["root_pos"], dtype=np.float32)
        root_xyzw = np.asarray(data["root_rot"], dtype=np.float32)
        joint_pos = np.asarray(data["dof_pos"], dtype=np.float32)
        source_names = _decode_names(data["joint_names"])
        fps = float(np.asarray(data["fps"]).reshape(())) if "fps" in data.files else 50.0

    if root_pos.ndim != 2 or root_pos.shape[1] != 3:
        raise ValueError(f"root_pos must be [T,3], got {root_pos.shape}")
    if root_xyzw.ndim != 2 or root_xyzw.shape[1] != 4:
        raise ValueError(f"root_rot must be [T,4] xyzw, got {root_xyzw.shape}")
    if joint_pos.ndim != 2 or joint_pos.shape[0] != root_pos.shape[0]:
        raise ValueError(f"dof_pos must be [T,J], got {joint_pos.shape}")
    if len(source_names) != joint_pos.shape[1]:
        raise ValueError("joint_names and dof_pos dimensions differ")

    by_name = {name: index for index, name in enumerate(source_names)}
    missing_names = [name for name in target_joint_names if name not in by_name]
    if missing_names:
        raise ValueError(f"motion is missing G1 joints: {missing_names}")
    joint_pos = joint_pos[:, [by_name[name] for name in target_joint_names]]

    root_wxyz = np.concatenate([root_xyzw[:, 3:4], root_xyzw[:, :3]], axis=1)
    norms = np.linalg.norm(root_wxyz, axis=1, keepdims=True)
    if np.any(norms < 1.0e-6):
        raise ValueError("motion contains an invalid root quaternion")
    root_wxyz = root_wxyz / norms
    qpos = np.concatenate([root_pos, root_wxyz, joint_pos], axis=1).astype(np.float32)
    if not np.isfinite(qpos).all() or qpos.shape[1] != 7 + len(target_joint_names):
        raise ValueError(f"invalid qpos array {qpos.shape}")
    if fps <= 0.0:
        raise ValueError(f"fps must be positive, got {fps}")
    return np.ascontiguousarray(qpos), fps


class MotionReferenceServer:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        config = load_teleop_robot_config("g1", args.config)
        self.config = replace(
            config,
            req_bind_addr=args.req_bind_addr or config.req_bind_addr,
            rep_bind_addr=args.rep_bind_addr or config.rep_bind_addr,
            ctrl_bind_addr=args.ctrl_bind_addr or config.ctrl_bind_addr,
        )
        self.qpos, self.fps = load_motion(args.motion.expanduser().resolve(), self.config.dof_names)
        self.index = 0
        self.sequence = 0
        self.loop = bool(args.loop)
        self.stop_event = threading.Event()
        self.viewer = None if args.no_viewer else MJViserViewer(
            "g1", host=args.viewer_host, port=args.viewer_port
        )
        self.zmq_context = None
        self.req_sock = None
        self.rep_sock = None
        self.ctrl_sock = None

    def _control_loop(self) -> None:
        import zmq

        buttons = default_controller_buttons()
        # A sustained right-primary signal makes every new deploy policy session
        # produce one clean rising edge after VRMotionSource resets its state.
        buttons["right_key_one"] = True
        payload = {
            "protocol": PROTOCOL,
            "source": "motion",
            "controller_buttons": buttons,
        }
        period = 1.0 / float(self.config.ctrl_fps)
        while not self.stop_event.is_set():
            payload["t_ms"] = int(time.time() * 1000)
            try:
                self.ctrl_sock.send_string(json.dumps(payload), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            self.stop_event.wait(period)

    def _next_frame(self) -> np.ndarray:
        frame = self.qpos[self.index]
        if self.index + 1 < self.qpos.shape[0]:
            self.index += 1
        elif self.loop:
            self.index = 0
        return frame

    def run(self) -> None:
        import zmq

        self.zmq_context = zmq.Context.instance()
        self.req_sock = self.zmq_context.socket(zmq.PULL)
        self.rep_sock = self.zmq_context.socket(zmq.PUSH)
        self.ctrl_sock = self.zmq_context.socket(zmq.PUSH)
        for sock in (self.req_sock, self.rep_sock, self.ctrl_sock):
            sock.setsockopt(zmq.LINGER, 0)
        self.req_sock.bind(self.config.req_bind_addr)
        self.rep_sock.bind(self.config.rep_bind_addr)
        self.ctrl_sock.bind(self.config.ctrl_bind_addr)

        print("Motion reference server initialized")
        print(f"  motion: {self.args.motion}")
        print(f"  frames/fps: {self.qpos.shape[0]}/{self.fps:.3f}")
        print(f"  req/rep/ctrl: {self.config.req_bind_addr} {self.config.rep_bind_addr} {self.config.ctrl_bind_addr}")
        if self.viewer is not None:
            print(f"  viewer_url: http://localhost:{self.args.viewer_port}")

        control_thread = threading.Thread(target=self._control_loop, daemon=True)
        control_thread.start()
        try:
            while True:
                req = json.loads(self.req_sock.recv_string())
                frame = self._next_frame()
                qpos = np.ascontiguousarray(frame.reshape(1, -1), dtype=np.float32)
                if self.viewer is not None:
                    self.viewer.update_qpos(frame)
                header = json.dumps(
                    {
                        "protocol": PROTOCOL,
                        "source": "motion",
                        "seq": self.sequence,
                        "start": bool(req.get("start", False)),
                        "num_frames": 1,
                        "qpos_size": int(qpos.shape[1]),
                    }
                ).encode("utf-8")
                self.rep_sock.send_multipart([header, qpos], copy=False)
                self.sequence += 1
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            control_thread.join(timeout=1.0)
            for sock in (self.req_sock, self.rep_sock, self.ctrl_sock):
                if sock is not None:
                    sock.close(0)
            if self.viewer is not None:
                self.viewer.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion", type=Path)
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--req-bind-addr", default=None)
    parser.add_argument("--rep-bind-addr", default=None)
    parser.add_argument("--ctrl-bind-addr", default=None)
    parser.add_argument("--viewer-host", default="0.0.0.0")
    parser.add_argument("--viewer-port", type=int, default=8080)
    parser.add_argument("--no-viewer", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    MotionReferenceServer(parse_args()).run()
