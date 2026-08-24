#!/usr/bin/env python3
"""Serve a robot motion NPZ through the live G1 reference protocol."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np

from retarget.viser_viewer import MJViserViewer
from utils.helper import default_controller_buttons
from utils.robot_config import load_teleop_robot_config


PROTOCOL = "g1-reference-v1"
MOTION_SELECT_PROTOCOL = "g1-motion-select-v1"


def discover_motion_files(root: Path) -> list[Path]:
    root = root.expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"motion root is not a directory: {root}")
    return sorted(path.resolve() for path in root.rglob("*.npz") if path.is_file())


def motion_display_name(path: Path, root: Path) -> str:
    path = path.expanduser().resolve()
    root = root.expanduser().resolve()
    try:
        relative = path.relative_to(root)
    except ValueError:
        return path.name
    return relative.with_suffix("").as_posix()


def resolve_motion_choice(choice: str, root: Path) -> Path:
    root = root.expanduser().resolve()
    files = discover_motion_files(root)
    requested = str(choice).strip()
    if not requested:
        raise ValueError("motion choice is empty")

    direct = Path(requested).expanduser()
    if direct.is_absolute():
        resolved = direct.resolve()
        try:
            resolved.relative_to(root)
        except ValueError as exc:
            raise ValueError(f"motion must be inside {root}: {resolved}") from exc
        if resolved in files:
            return resolved

    normalized = requested.removesuffix(".npz").replace("\\", "/")
    exact = [path for path in files if motion_display_name(path, root) == normalized]
    if len(exact) == 1:
        return exact[0]

    basename = [path for path in files if path.stem == normalized]
    if len(basename) == 1:
        return basename[0]
    if len(basename) > 1:
        names = [motion_display_name(path, root) for path in basename]
        raise ValueError(f"ambiguous motion {requested!r}: {names}")
    raise ValueError(f"unknown motion {requested!r}")


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
        self.motion_root = args.motion_root.expanduser().resolve()
        self.motion_files = discover_motion_files(self.motion_root)
        if not self.motion_files:
            raise RuntimeError(f"no .npz motions found under {self.motion_root}")
        self.motion_path: Path | None = None
        self.motion_name = ""
        self.qpos: np.ndarray | None = None
        self.fps = 0.0
        self.index = 0
        self.sequence = 0
        self.motion_command_seq = 0
        self.loop = bool(args.loop)
        self.queued = False
        self.playing = False
        self.finished = False
        if args.motion is not None:
            self._load_selected_motion(str(args.motion))
            self.motion_command_seq = 1
            self.queued = True
        self.stop_event = threading.Event()
        self.viewer = None if args.no_viewer else MJViserViewer(
            "g1", host=args.viewer_host, port=args.viewer_port
        )
        self.zmq_context = None
        self.req_sock = None
        self.rep_sock = None
        self.ctrl_sock = None
        self.select_sock = None

    @property
    def state(self) -> str:
        if self.playing:
            return "playing"
        if self.queued:
            return "queued"
        if self.finished:
            return "finished"
        return "waiting"

    def _status_payload(self) -> dict[str, Any]:
        return {
            "protocol": MOTION_SELECT_PROTOCOL,
            "ok": True,
            "state": self.state,
            "motion": self.motion_name,
            "motion_command_seq": int(self.motion_command_seq),
            "frames": 0 if self.qpos is None else int(self.qpos.shape[0]),
            "fps": float(self.fps),
        }

    def _load_selected_motion(self, choice: str) -> None:
        path = resolve_motion_choice(choice, self.motion_root)
        qpos, fps = load_motion(path, self.config.dof_names)
        self.motion_path = path
        self.motion_name = motion_display_name(path, self.motion_root)
        self.qpos = qpos
        self.fps = fps
        self.index = 0

    def _select_motion(self, choice: str) -> dict[str, Any]:
        if self.playing:
            return {
                **self._status_payload(),
                "ok": False,
                "detail": "motion switch rejected while the current motion is playing",
            }
        try:
            self._load_selected_motion(choice)
        except (FileNotFoundError, OSError, ValueError) as exc:
            return {
                **self._status_payload(),
                "ok": False,
                "detail": str(exc),
            }

        self.motion_command_seq += 1
        self.queued = True
        self.playing = False
        self.finished = False
        print(
            f"[MotionReference] queued '{self.motion_name}' "
            f"frames={self.qpos.shape[0]} fps={self.fps:.3f}"
        )
        return {
            **self._status_payload(),
            "detail": "queued; it will start automatically when the onboard policy is ready",
        }

    def handle_selection_request(self, payload: Any) -> dict[str, Any]:
        if not isinstance(payload, dict):
            return {
                **self._status_payload(),
                "ok": False,
                "detail": "selection request must be a JSON object",
            }
        protocol = payload.get("protocol")
        if protocol not in (None, MOTION_SELECT_PROTOCOL):
            return {
                **self._status_payload(),
                "ok": False,
                "detail": f"unsupported protocol: {protocol!r}",
            }
        command = str(payload.get("command", "status")).strip().lower()
        if command == "status":
            return self._status_payload()
        if command == "list":
            return {
                **self._status_payload(),
                "motions": [
                    motion_display_name(path, self.motion_root)
                    for path in discover_motion_files(self.motion_root)
                ],
            }
        if command == "select":
            return self._select_motion(str(payload.get("motion", "")))
        return {
            **self._status_payload(),
            "ok": False,
            "detail": f"unknown selection command: {command!r}",
        }

    def _control_loop(self) -> None:
        import zmq

        buttons = default_controller_buttons()
        payload = {
            "protocol": PROTOCOL,
            "source": "motion",
            "controller_buttons": buttons,
        }
        period = 1.0 / float(self.config.ctrl_fps)
        while not self.stop_event.is_set():
            payload["t_ms"] = int(time.time() * 1000)
            payload["motion"] = self.motion_name
            payload["state"] = self.state
            payload["motion_command_seq"] = int(self.motion_command_seq)
            try:
                self.ctrl_sock.send_string(json.dumps(payload), flags=zmq.NOBLOCK)
            except zmq.Again:
                pass
            self.stop_event.wait(period)

    def _next_frame(self, *, start: bool) -> tuple[np.ndarray, bool]:
        if start:
            self.index = 0
            self.queued = False
            self.playing = True
            self.finished = False
            print(f"[MotionReference] playing '{self.motion_name}'")

        if self.qpos is None:
            raise RuntimeError("no motion is selected")
        frame = self.qpos[self.index]
        is_last = self.index + 1 >= self.qpos.shape[0]
        if not self.playing:
            return frame, self.finished
        if not is_last:
            self.index += 1
        elif self.loop:
            self.index = 0
        else:
            self.playing = False
            self.finished = True
            print(
                f"[MotionReference] finished '{self.motion_name}'; "
                "onboard policy will return to default and wait for the next selection"
            )
        return frame, self.finished

    def _handle_reference_request(self, req: Any) -> tuple[np.ndarray, dict[str, Any]] | None:
        if not isinstance(req, dict):
            return None
        if self.qpos is None:
            return None

        start = bool(req.get("start", False))
        if start and not self.queued:
            return None
        if not start and not self.playing:
            return None
        frame, finished = self._next_frame(start=start)
        qpos = np.ascontiguousarray(frame.reshape(1, -1), dtype=np.float32)
        header = {
            "protocol": PROTOCOL,
            "source": "motion",
            "motion": self.motion_name,
            "state": self.state,
            "finished": bool(finished),
            "seq": self.sequence,
            "start": start,
            "num_frames": 1,
            "qpos_size": int(qpos.shape[1]),
        }
        self.sequence += 1
        return qpos, header

    def run(self) -> None:
        import zmq

        self.zmq_context = zmq.Context.instance()
        self.req_sock = self.zmq_context.socket(zmq.PULL)
        self.rep_sock = self.zmq_context.socket(zmq.PUSH)
        self.ctrl_sock = self.zmq_context.socket(zmq.PUSH)
        self.select_sock = self.zmq_context.socket(zmq.REP)
        for sock in (self.req_sock, self.rep_sock, self.ctrl_sock, self.select_sock):
            sock.setsockopt(zmq.LINGER, 0)
        self.req_sock.bind(self.config.req_bind_addr)
        self.rep_sock.bind(self.config.rep_bind_addr)
        self.ctrl_sock.bind(self.config.ctrl_bind_addr)
        self.select_sock.bind(self.args.select_bind_addr)

        poller = zmq.Poller()
        poller.register(self.req_sock, zmq.POLLIN)
        poller.register(self.select_sock, zmq.POLLIN)

        print("Motion reference server initialized")
        print(f"  motion_root: {self.motion_root} ({len(self.motion_files)} files)")
        if self.motion_path is None:
            print("  motion: none selected; use the motion-select window")
        else:
            print(f"  motion: {self.motion_name} ({self.motion_path})")
            print(f"  frames/fps: {self.qpos.shape[0]}/{self.fps:.3f}")
        print(f"  req/rep/ctrl: {self.config.req_bind_addr} {self.config.rep_bind_addr} {self.config.ctrl_bind_addr}")
        print(f"  selector: {self.args.select_bind_addr}")
        if self.viewer is not None:
            print(f"  viewer_url: http://localhost:{self.args.viewer_port}")

        control_thread = threading.Thread(target=self._control_loop, daemon=True)
        control_thread.start()
        try:
            while True:
                events = dict(poller.poll(timeout=100))
                if self.select_sock in events:
                    try:
                        request = self.select_sock.recv_json()
                        reply = self.handle_selection_request(request)
                    except Exception as exc:
                        reply = {
                            **self._status_payload(),
                            "ok": False,
                            "detail": str(exc),
                        }
                    self.select_sock.send_json(reply)

                if self.req_sock in events:
                    try:
                        req = json.loads(self.req_sock.recv_string())
                    except Exception:
                        continue
                    result = self._handle_reference_request(req)
                    if result is None:
                        continue
                    qpos, header = result
                    if self.viewer is not None:
                        self.viewer.update_qpos(qpos[0])
                    self.rep_sock.send_multipart(
                        [json.dumps(header).encode("utf-8"), qpos], copy=False
                    )
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            control_thread.join(timeout=1.0)
            for sock in (self.req_sock, self.rep_sock, self.ctrl_sock, self.select_sock):
                if sock is not None:
                    sock.close(0)
            if self.viewer is not None:
                self.viewer.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion", type=Path, nargs="?", default=None)
    parser.add_argument(
        "--motion-root",
        type=Path,
        default=Path("config/g1/motions"),
        help="Root containing selectable G1 motion NPZ files.",
    )
    parser.add_argument(
        "--select-bind-addr",
        default="tcp://127.0.0.1:28704",
        help="Local ZMQ REP endpoint used by motion_select.py.",
    )
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
