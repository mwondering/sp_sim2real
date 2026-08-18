"""TAP—terrain task: thin I/O adapter around the complete terrain ONNX."""

from __future__ import annotations

import argparse
import time
import traceback
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import yaml

from deploy import Controller, get_config
from paths import SIM2REAL_ROOT, controller_config_path
from runtime.depth_pipeline import TAPTerrainDepthProcessor
from runtime.policy import TAPTerrainPolicy
from runtime.zmq_stream import ArraySubscriber


TASK_NAME = "TAP—terrain"


def _load_task_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
    if not isinstance(config, dict):
        raise ValueError(f"Task config must be a mapping: {path}")
    if config.get("task_name") != TASK_NAME:
        raise ValueError(
            f"Task config {path} has task_name={config.get('task_name')!r}, "
            f"expected {TASK_NAME!r}"
        )
    return config


class TAPTerrainInputAdapter:
    """Convert one depth stream and one velocity source to ONNX-ready inputs."""

    DEPTH_SHAPE = (1, 18, 32)

    def __init__(
        self,
        *,
        target: str,
        camera_config: Mapping[str, object],
        velocity_config: Mapping[str, object],
        depth_connect: str | None = None,
        velocity_override: Sequence[float] | None = None,
        subscriber=None,
    ) -> None:
        self.target = str(target)
        if self.target not in ("sim", "real"):
            raise ValueError(f"TAP—terrain target must be sim or real, got {target!r}")
        expected_source = "mujoco" if self.target == "sim" else "d435i"
        source = str(camera_config.get("source", ""))
        if source != expected_source:
            raise ValueError(
                f"TAP—terrain target={self.target!r} requires camera source "
                f"{expected_source!r}, got {source!r}"
            )

        self.depth_timeout_s = float(camera_config.get("timeout_s", 0.25))
        self.max_invalid_fraction = float(
            camera_config.get("max_invalid_fraction", 0.6)
        )
        if self.depth_timeout_s <= 0.0:
            raise ValueError("camera_process.timeout_s must be positive")
        if not 0.0 <= self.max_invalid_fraction <= 1.0:
            raise ValueError("camera_process.max_invalid_fraction must be in [0,1]")
        self.depth_connect = str(
            camera_config["depth_connect"]
            if depth_connect is None
            else depth_connect
        )
        self.depth_sub = (
            ArraySubscriber(self.depth_connect, topic="depth")
            if subscriber is None
            else subscriber
        )

        preprocess = camera_config.get("preprocess")
        if not isinstance(preprocess, Mapping):
            raise ValueError("TAP—terrain camera_process.preprocess must be a mapping")
        self.depth_processor = TAPTerrainDepthProcessor.from_config(preprocess)
        hardware = camera_config.get("hardware", {})
        if not isinstance(hardware, Mapping):
            raise ValueError("camera_process.hardware must be a mapping")
        self.raw_depth_shape = (
            int(hardware.get("height", 360)),
            int(hardware.get("width", 640)),
        )

        configured_velocity_mode = str(velocity_config.get("source", "sticks"))
        self.velocity_mode = (
            "fixed" if velocity_override is not None else configured_velocity_mode
        )
        if self.velocity_mode not in ("fixed", "sticks"):
            raise ValueError("velocity_command.source must be fixed or sticks")
        fixed = (
            velocity_config.get("fixed", (0.0, 0.0, 0.0))
            if velocity_override is None
            else velocity_override
        )
        self.fixed_velocity = np.asarray(fixed, dtype=np.float32).reshape(3)
        self.deadband = float(velocity_config.get("deadband", 0.1))
        self.max_vx = float(velocity_config.get("max_vx", 1.0))
        self.max_vy = float(velocity_config.get("max_vy", 0.0))
        self.max_wz = float(velocity_config.get("max_wz", 0.5))
        self.forward_only = bool(velocity_config.get("forward_only", True))
        self.min_forward_speed = float(
            velocity_config.get("min_forward_speed", 0.0)
        )
        if not 0.0 <= self.deadband < 1.0:
            raise ValueError("velocity_command.deadband must be in [0,1)")
        if min(self.max_vx, self.max_vy, self.max_wz, self.min_forward_speed) < 0.0:
            raise ValueError("velocity command limits must be non-negative")
        if self.min_forward_speed > self.max_vx:
            raise ValueError("min_forward_speed cannot exceed max_vx")
        if not np.isfinite(self.fixed_velocity).all():
            raise ValueError("velocity_command.fixed contains non-finite values")

    def _axis(self, sticks: Mapping[str, float], name: str) -> float:
        value = float(np.clip(float(sticks.get(name, 0.0)), -1.0, 1.0))
        return 0.0 if abs(value) < self.deadband else value

    def velocity_command(self, sticks: Mapping[str, float]) -> np.ndarray:
        if self.velocity_mode == "fixed":
            command = self.fixed_velocity.copy()
        else:
            command = np.asarray(
                [
                    self.max_vx * self._axis(sticks, "ly"),
                    self.max_vy * self._axis(sticks, "lx"),
                    self.max_wz * self._axis(sticks, "rx"),
                ],
                dtype=np.float32,
            )
        command[0] = np.clip(command[0], -self.max_vx, self.max_vx)
        command[1] = np.clip(command[1], -self.max_vy, self.max_vy)
        command[2] = np.clip(command[2], -self.max_wz, self.max_wz)
        if self.forward_only:
            if command[0] <= 0.0:
                command[0] = 0.0
            elif command[0] < self.min_forward_speed:
                command[0] = self.min_forward_speed
            command[1] = 0.0
        return command.astype(np.float32, copy=False)

    def read_depth(self) -> tuple[np.ndarray | None, str]:
        packet = self.depth_sub.read_latest()
        if packet is None:
            return None, "depth_missing"
        age = time.monotonic() - float(packet.recv_time)
        if age < -0.1 or age > self.depth_timeout_s:
            return None, f"depth_stale(age={age:.3f}s)"

        if self.target == "real":
            if packet.metadata.get("protocol") != "d435i-raw-z16-v1":
                raise ValueError(
                    "Real TAP—terrain depth stream must use protocol "
                    "d435i-raw-z16-v1"
                )
            raw = np.asarray(packet.values)
            if raw.dtype != np.uint16 or raw.shape != self.raw_depth_shape:
                raise ValueError(
                    f"Raw depth {raw.dtype} {raw.shape} != uint16 "
                    f"{self.raw_depth_shape}"
                )
            depth, stats = self.depth_processor.process_raw(
                raw, depth_scale=float(packet.metadata["depth_scale"])
            )
            invalid_fraction = float(stats["invalid_fraction"])
        else:
            depth = np.asarray(packet.values, dtype=np.float32)
            if depth.shape != self.DEPTH_SHAPE:
                raise ValueError(
                    f"Sim TAP—terrain depth {depth.shape} != {self.DEPTH_SHAPE}; "
                    "run depth_camera.py with the TAP—terrain task config"
                )
            invalid_fraction = float(packet.metadata.get("invalid_fraction", 0.0))

        if not np.isfinite(depth).all():
            return None, "depth_nonfinite"
        if invalid_fraction > self.max_invalid_fraction:
            return None, f"depth_invalid_fraction={invalid_fraction:.3f}"
        return np.ascontiguousarray(depth, dtype=np.float32), "ready"

    def close(self) -> None:
        self.depth_sub.close()


class TAPTerrainController(Controller):
    def __init__(
        self,
        args,
        ctrl_cfg,
        task_config: dict,
        task_config_path: Path,
    ) -> None:
        self.task_config = task_config
        self.task_config_path = task_config_path
        super().__init__(args, ctrl_cfg)
        self.tap_inputs = TAPTerrainInputAdapter(
            target=str(task_config["target"]),
            camera_config=task_config["camera_process"],
            velocity_config=task_config["velocity_command"],
            depth_connect=args.depth_connect,
            velocity_override=args.velocity,
        )
        self.history_warmup_steps = int(task_config.get("history_warmup_steps", 0))
        if self.history_warmup_steps < 0:
            raise ValueError("history_warmup_steps must be non-negative")
        self._depth_ready_once = False
        self._last_wait_report = 0.0
        print(
            f"[{TASK_NAME}] target={task_config['target']} "
            f"onnx={self.policies['tracking'].policy_path} "
            f"depth<-{self.tap_inputs.depth_connect} "
            f"velocity_source={self.tap_inputs.velocity_mode} "
            f"history_warmup_steps={self.history_warmup_steps}"
        )

    def _create_tracking_policy(self, name, tracking_cfg):
        del name
        return TAPTerrainPolicy("tap_terrain", tracking_cfg, self)

    def _hold_default(self, *, enable: bool) -> None:
        self.cmd_q[:] = self.default_qpos
        self.cmd_qd[:] = 0.0
        self.cmd_kp[:] = self.kps
        self.cmd_kd[:] = self.kds
        self.cmd_enable = int(enable)

    def run_tap_terrain(self) -> None:
        policy = self.policies["tracking"]
        if not isinstance(policy, TAPTerrainPolicy):
            raise TypeError(f"Expected TAPTerrainPolicy, got {type(policy).__name__}")
        self._start_policy_recorder()
        warmed_steps = 0
        print(
            f"[{TASK_NAME}] running; fresh depth is mandatory after the first frame"
        )
        while True:
            if not self.process_state(wait_next=True, timeout_s=1.0):
                raise TimeoutError("No bridge state for 1s")
            if self.btn_rise["stop"]:
                break

            depth, reason = self.tap_inputs.read_depth()
            if depth is None:
                if self._depth_ready_once:
                    raise TimeoutError(f"TAP—terrain lost required depth: {reason}")
                now = time.monotonic()
                if now - self._last_wait_report >= 1.0:
                    print(f"[{TASK_NAME}] waiting for depth: {reason}")
                    self._last_wait_report = now
                # A selects TAP but does not release sim2sim's temporary base
                # stabilizer.  Handoff occurs atomically with the first policy
                # target after a valid depth frame arrives.
                self._hold_default(enable=False)
                self.send_cmd()
                continue
            if not self._depth_ready_once:
                print(f"[{TASK_NAME}] depth ready")
                self._depth_ready_once = True

            velocity = self.tap_inputs.velocity_command(self.sticks)
            policy.set_external_inputs(velocity, depth)
            policy.update_obs()
            policy_observation = policy.policy_observation_copy()
            if warmed_steps < self.history_warmup_steps:
                action = np.zeros(self.dof_size, dtype=np.float32)
                warmed_steps += 1
                self._hold_default(enable=False)
                if warmed_steps == self.history_warmup_steps:
                    print(f"[{TASK_NAME}] proprio history warmup complete")
            else:
                action = policy.compute_action()
                self._apply_action(action)
            self.send_cmd()

            if self.recorder is not None:
                self.recorder.record_step(
                    self,
                    action,
                    policy_observation=policy_observation,
                )
            policy.post_step()
            self.policy_step += 1

    def close_tap_terrain(self) -> None:
        self.is_alive = False
        self._save_policy_recorder()
        if hasattr(self, "tap_inputs"):
            self.tap_inputs.close()
        self.transport.close()


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="TAP—terrain deployment task")
    parser.add_argument("--robot", choices=("g1",), default="g1")
    parser.add_argument("--target", choices=("sim", "real"), default="sim")
    parser.add_argument("--task-config", type=Path, default=None)
    parser.add_argument("--controller-config", type=Path, default=None)
    parser.add_argument("--depth-connect", default=None)
    parser.add_argument(
        "--velocity",
        type=float,
        nargs=3,
        metavar=("VX", "VY", "WZ"),
        default=None,
        help="Override task velocity command in m/s, m/s, rad/s",
    )
    parser.add_argument("--record-dir", type=Path, default=None)
    parser.add_argument("--no-record", action="store_true")
    return parser


def main(argv=None) -> None:
    args = _argument_parser().parse_args(argv)
    if args.task_config is None:
        config_name = (
            "tap-terrain-real.yaml" if args.target == "real" else "tap-terrain.yaml"
        )
        args.task_config = SIM2REAL_ROOT / "config/g1" / config_name
    task_config_path = args.task_config.expanduser().resolve()
    task_config = _load_task_config(task_config_path)
    configured_target = str(task_config.get("target", ""))
    if configured_target != args.target:
        raise ValueError(
            f"--target {args.target!r} does not match task config target "
            f"{configured_target!r}: {task_config_path}"
        )
    tracking_config = Path(
        str(task_config.get("tracking_config", "tap-terrain-policy.yaml"))
    )
    if not tracking_config.is_absolute():
        tracking_config = (task_config_path.parent / tracking_config).resolve()
    args.tracking_config = str(tracking_config)
    args.pico_store = None
    args.force_vr_motion_source = False

    controller = None
    try:
        config_path = (
            args.controller_config
            if args.controller_config is not None
            else controller_config_path(args.robot)
        )
        controller = TAPTerrainController(
            args,
            get_config(config_path),
            task_config,
            task_config_path,
        )
        controller.zero_torque_state()
        controller.move_to_default_qpos()
        # Keep cmd_enable=0 after policy selection until run_tap_terrain() has
        # prepared the first valid observation and policy target. In sim2sim
        # this keeps the startup base stabilizer active up to the handoff step.
        activation_label = (
            "P in the sim2sim terminal"
            if args.target == "sim"
            else "A on the G1 remote"
        )
        controller.default_qpos_state(
            enable_immediately=False,
            activation_label=activation_label,
        )
        controller.run_tap_terrain()
    except KeyboardInterrupt:
        print(f"[{TASK_NAME}] interrupted")
    except Exception as exc:
        print(f"[{TASK_NAME}] error: {exc}")
        traceback.print_exc()
    finally:
        if controller is not None:
            try:
                controller.set_damping_cmd()
                controller.send_cmd()
            finally:
                controller.close_tap_terrain()


if __name__ == "__main__":
    main()


__all__ = [
    "TASK_NAME",
    "TAPTerrainController",
    "TAPTerrainInputAdapter",
    "_load_task_config",
]
