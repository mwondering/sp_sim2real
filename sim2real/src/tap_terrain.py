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
from runtime.policy import TAPTerrainPolicy, TrackingPolicyRaw
from runtime.shared_pico import PicoFrameStore
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
        self.pico_store = PicoFrameStore()
        args.pico_store = self.pico_store
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
        self._terrain_warmed_steps = 0
        self._depth_ready_once = False
        self._last_wait_report = 0.0

        switch_cfg = task_config.get("policy_switch", {})
        if not isinstance(switch_cfg, Mapping):
            raise ValueError("policy_switch must be a mapping")
        self.switch_enabled = bool(switch_cfg.get("enabled", False))
        combo_buttons = tuple(
            str(name).strip()
            for name in switch_cfg.get(
                "combo_buttons", ("left_key_one", "right_key_one")
            )
        )
        if len(combo_buttons) != 2 or any(not name for name in combo_buttons):
            raise ValueError("policy_switch.combo_buttons must contain two names")
        if combo_buttons[0] == combo_buttons[1]:
            raise ValueError("policy_switch.combo_buttons must be distinct")
        self.switch_combo_buttons = combo_buttons
        self.switch_freshness_timeout_s = float(
            switch_cfg.get("freshness_timeout_s", 0.25)
        )
        if self.switch_freshness_timeout_s <= 0.0:
            raise ValueError("policy_switch.freshness_timeout_s must be positive")
        blend_duration_s = float(switch_cfg.get("blend_duration_s", 1.0))
        if blend_duration_s < 0.0:
            raise ValueError("policy_switch.blend_duration_s must be non-negative")
        self.switch_blend_steps = int(round(blend_duration_s / self.control_dt))
        self._switch_blend_step = self.switch_blend_steps
        self._switch_blend_q = self.cmd_q.copy()
        self._switch_blend_kp = self.cmd_kp.copy()
        self._switch_blend_kd = self.cmd_kd.copy()
        self._switch_combo_was_pressed = False
        self.active_policy_name = "terrain"

        if self.switch_enabled:
            teleop_config = Path(
                str(
                    switch_cfg.get(
                        "teleop_tracking_config", "tap-teleop-policy.yaml"
                    )
                )
            )
            if not teleop_config.is_absolute():
                teleop_config = (
                    task_config_path.parent / teleop_config
                ).resolve()
            teleop_cfg = get_config(teleop_config)
            teleop_cfg._pico_store = self.pico_store
            if str(teleop_cfg.motion_source.get("type", "")) != "vr":
                raise ValueError(
                    "TAP teleop switch policy requires motion_source.type='vr'"
                )
            teleop_policy = TrackingPolicyRaw("tap_teleop", teleop_cfg, self)
            if teleop_policy.actor_profile != "tap_teleop":
                raise ValueError(
                    "TAP teleop switch policy requires actor_profile='tap_teleop'"
                )
            self.policies["teleop"] = teleop_policy

        teleop_status = (
            f"teleop_onnx={self.policies['teleop'].policy_path} "
            if self.switch_enabled
            else "teleop_switch=disabled "
        )
        print(
            f"[{TASK_NAME}] target={task_config['target']} "
            f"terrain_onnx={self.policies['tracking'].policy_path} "
            f"{teleop_status}"
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

    def _switch_combo_rising(self, snapshot, *, now: float | None = None) -> bool:
        pressed = all(
            bool(snapshot.buttons.get(name, False))
            for name in self.switch_combo_buttons
        )
        rising = pressed and not self._switch_combo_was_pressed
        self._switch_combo_was_pressed = pressed
        if not rising:
            return False
        timestamp = float(snapshot.control_timestamp)
        age = (time.monotonic() if now is None else float(now)) - timestamp
        if age < -0.1 or age > self.switch_freshness_timeout_s:
            print(
                f"[{TASK_NAME}] policy switch ignored: PICO control age "
                f"{age:.3f}s exceeds {self.switch_freshness_timeout_s:.3f}s"
            )
            return False
        return True

    @staticmethod
    def _policy_pd(policy, fallback_q, fallback_kp, fallback_kd):
        default_q = (
            fallback_q
            if policy.controller_default_qpos is None
            else policy.controller_default_qpos
        )
        kp = fallback_kp if policy.controller_kps is None else policy.controller_kps
        kd = fallback_kd if policy.controller_kds is None else policy.controller_kds
        return (
            np.asarray(default_q, dtype=np.float32),
            np.asarray(kp, dtype=np.float32),
            np.asarray(kd, dtype=np.float32),
        )

    def _apply_policy_action(self, policy, action_delta: np.ndarray) -> None:
        if action_delta is None or not np.all(np.isfinite(action_delta)):
            print(f"[{TASK_NAME}] action invalid; stopping task")
            raise KeyboardInterrupt
        default_q, kp, kd = self._policy_pd(
            policy, self.default_qpos, self.kps, self.kds
        )
        target_q = default_q + np.asarray(action_delta, dtype=np.float32)
        if self._switch_blend_step < self.switch_blend_steps:
            self._switch_blend_step += 1
            alpha = self._switch_blend_step / self.switch_blend_steps
            target_q = self._switch_blend_q * (1.0 - alpha) + target_q * alpha
            kp = self._switch_blend_kp * (1.0 - alpha) + kp * alpha
            kd = self._switch_blend_kd * (1.0 - alpha) + kd * alpha
        self.cmd_q[:] = target_q
        self.cmd_qd[:] = 0.0
        self.cmd_kp[:] = kp
        self.cmd_kd[:] = kd
        self.cmd_enable = 1

    def _commit_applied_policy_target(self, policy) -> None:
        default_q, _, _ = self._policy_pd(
            policy, self.default_qpos, self.kps, self.kds
        )
        delta_controller = self.cmd_q - default_q
        scaled_policy = policy.mapper_action.map_state_to_from(
            delta_controller
        ).astype(np.float32)
        scale = np.asarray(policy.action_scale, dtype=np.float32)
        raw = np.divide(
            scaled_policy,
            scale,
            out=np.zeros_like(scaled_policy),
            where=np.abs(scale) > 1.0e-8,
        )
        raw = np.clip(raw, -policy.action_clip, policy.action_clip)
        policy.last_action[:] = raw
        policy.applied_action[:] = raw * scale

    def _rotate_policy_recorder(self) -> None:
        if self.recorder is None:
            return
        self.recorder.save()
        self.recorder = None
        self._start_policy_recorder()

    def _switch_policy(self) -> bool:
        if not self.switch_enabled:
            return False
        destination = (
            "teleop" if self.active_policy_name == "terrain" else "terrain"
        )
        if destination == "terrain":
            _, reason = self.tap_inputs.read_depth()
            if reason != "ready":
                print(
                    f"[{TASK_NAME}] switch to terrain ignored: fresh depth "
                    f"required ({reason})"
                )
                return False

        old_policy = self.current_policy
        new_policy = self.policies[
            "teleop" if destination == "teleop" else "tracking"
        ]
        if old_policy is not None:
            old_policy.fade_out()
        new_policy.fade_in()
        self.current_policy = new_policy
        self.active_policy_name = destination
        self._terrain_warmed_steps = 0
        self._switch_blend_q = self.cmd_q.copy()
        self._switch_blend_kp = self.cmd_kp.copy()
        self._switch_blend_kd = self.cmd_kd.copy()
        self._switch_blend_step = 0
        if destination == "teleop":
            request_start = getattr(new_policy.source, "request_start", None)
            if not callable(request_start):
                raise TypeError("TAP teleop policy source cannot start VR")
            request_start()
        self._rotate_policy_recorder()
        print(
            f"[{TASK_NAME}] switched -> {destination}; "
            f"blend_steps={self.switch_blend_steps}"
        )
        return True

    def _handle_policy_switch(self) -> None:
        if not self.switch_enabled:
            return
        teleop_policy = self.policies["teleop"]
        teleop_policy.source.poll_control()
        snapshot = self.pico_store.snapshot()
        if self._switch_combo_rising(snapshot):
            self._switch_policy()

    def run_tap_terrain(self) -> None:
        terrain_policy = self.policies["tracking"]
        if not isinstance(terrain_policy, TAPTerrainPolicy):
            raise TypeError(
                f"Expected TAPTerrainPolicy, got {type(terrain_policy).__name__}"
            )
        self._start_policy_recorder()
        if self.switch_enabled:
            # Discard startup-time button history. A deliberate switch must be
            # released and pressed again after the policy loop is running.
            teleop_policy = self.policies["teleop"]
            teleop_policy.source.poll_control()
            snapshot = self.pico_store.snapshot()
            self._switch_combo_was_pressed = all(
                bool(snapshot.buttons.get(name, False))
                for name in self.switch_combo_buttons
            )
        print(
            f"[{TASK_NAME}] running; terrain mode requires fresh depth; "
            "press both PICO primary buttons to toggle terrain/teleop"
        )
        while True:
            if not self.process_state(wait_next=True, timeout_s=1.0):
                raise TimeoutError("No bridge state for 1s")
            if self.btn_rise["stop"]:
                break

            self._handle_policy_switch()
            policy = self.current_policy
            if policy is None:
                raise RuntimeError("TAP task has no active policy")

            if self.active_policy_name == "teleop":
                policy.update_obs()
                policy_observation = policy.policy_observation_copy()
                action = policy.compute_action()
                self._apply_policy_action(policy, action)
                self._commit_applied_policy_target(policy)
                self.send_cmd()
                if self.recorder is not None:
                    self.recorder.record_step(
                        self,
                        action,
                        policy_observation=policy_observation,
                    )
                policy.post_step()
                self.policy_step += 1
                continue

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
            if self._terrain_warmed_steps < self.history_warmup_steps:
                action = np.zeros(self.dof_size, dtype=np.float32)
                self._terrain_warmed_steps += 1
                self._hold_default(enable=False)
                if self._terrain_warmed_steps == self.history_warmup_steps:
                    print(f"[{TASK_NAME}] proprio history warmup complete")
            else:
                action = policy.compute_action()
                self._apply_policy_action(policy, action)
                self._commit_applied_policy_target(policy)
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
        for policy in getattr(self, "policies", {}).values():
            try:
                policy.deactivate()
            except Exception as exc:
                print(f"[{TASK_NAME}] policy close failed: {exc}")
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
        "--terrain-only",
        action="store_true",
        help="Disable the optional PICO teleop policy and run terrain only",
    )
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
    if args.terrain_only:
        switch_config = task_config.get("policy_switch", {})
        if not isinstance(switch_config, Mapping):
            raise ValueError("policy_switch must be a mapping")
        task_config["policy_switch"] = dict(switch_config)
        task_config["policy_switch"]["enabled"] = False
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
