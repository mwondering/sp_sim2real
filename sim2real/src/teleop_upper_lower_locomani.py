"""Dual-mode PICO task: whole-body teleop or upper/lower locomani."""

from __future__ import annotations

import argparse
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

SRC_ROOT = Path(__file__).resolve().parent
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from deploy import Controller, get_config
from paths import controller_config_path
from runtime.dual_locomani import ControlCommand, DualLocomaniRuntime
from runtime.shared_pico import PicoFrameStore
from runtime.zmq_stream import ArraySubscriber


def _resolve(config_path: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (config_path.parent / path).resolve()


def _load_task_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
    if not isinstance(config, dict):
        raise ValueError(f"Task config must be a mapping: {path}")
    return config


class TeleopUpperLowerLocomaniController(Controller):
    MODE_WHOLE_BODY = "whole-body"
    MODE_DUAL = "upper-lower-locomani"
    MODE_SAFE_HOLD = "safe-hold"

    def __init__(self, args, ctrl_cfg, task_config: dict, task_config_path: Path):
        self.pico_store = PicoFrameStore()
        args.pico_store = self.pico_store
        args.force_vr_motion_source = True
        super().__init__(args, ctrl_cfg)
        self.task_config = task_config
        self.target = str(task_config.get("target", "sim"))
        self.mode = self.MODE_WHOLE_BODY
        self._previous_mode_button = False
        self._last_camera_warning = 0.0
        self._neutral_since = time.monotonic()
        self._safe_hold_q = self.cmd_q.copy()
        self._pico_session_started = False

        lower_path = _resolve(task_config_path, task_config["lower_onnx"])
        upper_path = _resolve(task_config_path, task_config["upper_onnx"])
        kinematics_xml = _resolve(task_config_path, task_config["kinematics_xml"])
        self.dual = DualLocomaniRuntime(
            lower_onnx=lower_path,
            upper_onnx=upper_path,
            kinematics_xml=kinematics_xml,
            controller_joint_names=self.policy_joint_names,
            terrain_class=int(args.terrain_class),
        )
        camera_cfg = task_config["camera_process"]
        self.depth_sub = ArraySubscriber(
            str(camera_cfg["depth_connect"]), topic="depth"
        )
        self.depth_timeout_s = float(camera_cfg.get("timeout_s", 0.25))
        self.max_invalid_fraction = float(
            camera_cfg.get("max_invalid_fraction", 0.6)
        )
        switch_cfg = task_config.get("switch", {})
        self.switch_button = str(switch_cfg.get("button", "right_key_two"))
        self.neutral_hold_s = float(switch_cfg.get("neutral_hold_s", 0.25))
        self.stick_deadband = float(switch_cfg.get("stick_deadband", 0.1))
        safety_cfg = task_config.get("safety", {})
        self.state_timeout_is_fatal = bool(
            safety_cfg.get("state_timeout_is_fatal", self.target == "real")
        )
        self.safe_hold_kp_scale = float(
            safety_cfg.get("safe_hold_kp_scale", 0.5)
        )
        self.safe_hold_min_kd = float(
            safety_cfg.get("safe_hold_min_kd", 2.0)
        )
        self.max_abs_gyro = float(
            safety_cfg.get("max_abs_gyro_rad_s", 4.0)
        )
        self.max_tilt_rad = float(safety_cfg.get("max_tilt_rad", 0.7))
        self.blend_steps = max(
            1,
            int(
                round(
                    float(switch_cfg.get("blend_duration_s", 0.4))
                    / self.control_dt
                )
            ),
        )
        self._blend_step = self.blend_steps
        self._blend_start = ControlCommand(
            q_des=self.cmd_q.copy(),
            kp=self.cmd_kp.copy(),
            kd=self.cmd_kd.copy(),
        )
        print(
            "[TeleopLocomani] "
            f"target={self.target}, whole_body={args.tracking_config}, "
            f"terrain_class={args.terrain_class}, "
            f"blend_steps={self.blend_steps}, depth<-{camera_cfg['depth_connect']}"
        )

    def _whole_body_command(self, action: np.ndarray) -> ControlCommand:
        policy = self.policies["tracking"]
        default = (
            self.default_qpos
            if policy.controller_default_qpos is None
            else policy.controller_default_qpos
        )
        kp = self.kps if policy.controller_kps is None else policy.controller_kps
        kd = self.kds if policy.controller_kds is None else policy.controller_kds
        return ControlCommand(
            q_des=np.asarray(default + action, dtype=np.float32),
            kp=np.asarray(kp, dtype=np.float32),
            kd=np.asarray(kd, dtype=np.float32),
        )

    def _depth(self):
        packet = self.depth_sub.read_latest()
        if packet is None:
            return None
        capture_time = packet.metadata.get("capture_monotonic")
        if capture_time is None:
            age = time.monotonic() - packet.recv_time
        else:
            age = time.monotonic() - float(capture_time)
        if age < -0.1 or age > self.depth_timeout_s:
            return None
        if packet.values.shape != (1, 36, 64):
            raise ValueError(
                f"Depth stream shape {packet.values.shape} != (1, 36, 64)"
            )
        depth = np.asarray(packet.values, dtype=np.float32)
        if not np.all(np.isfinite(depth)):
            raise FloatingPointError("Depth stream contains NaN/Inf")
        invalid_value = float(packet.metadata.get("invalid_value", -1.0))
        invalid_fraction = float(
            packet.metadata.get(
                "invalid_fraction", np.mean(depth == invalid_value)
            )
        )
        if invalid_fraction > self.max_invalid_fraction:
            return None
        return depth

    def _set_mode(self, mode: str) -> None:
        if mode == self.mode:
            return
        self.mode = mode
        self._blend_start = ControlCommand(
            q_des=self.cmd_q.copy(),
            kp=self.cmd_kp.copy(),
            kd=self.cmd_kd.copy(),
        )
        self._blend_step = 0
        print(f"[TeleopLocomani] switched -> {mode}")

    def _sticks_neutral(self, snapshot) -> bool:
        return all(
            abs(float(snapshot.sticks.get(name, 0.0))) <= self.stick_deadband
            for name in ("lx", "ly", "rx", "ry")
        )

    def _update_neutral_hold(self, snapshot) -> bool:
        now = time.monotonic()
        if self._sticks_neutral(snapshot):
            if self._neutral_since is None:
                self._neutral_since = now
        else:
            self._neutral_since = None
        return (
            self._neutral_since is not None
            and now - self._neutral_since >= self.neutral_hold_s
        )

    def _body_stable(self) -> bool:
        gyro_ok = (
            np.all(np.isfinite(self.gyro))
            and float(np.max(np.abs(self.gyro))) <= self.max_abs_gyro
        )
        quat = np.asarray(self.quat, dtype=np.float64)
        norm = float(np.linalg.norm(quat))
        if not np.isfinite(norm) or norm < 1.0e-6:
            return False
        gravity = R.from_quat(
            quat / norm, scalar_first=True
        ).inv().apply(np.array([0.0, 0.0, -1.0]))
        tilt = float(np.arccos(np.clip(-gravity[2], -1.0, 1.0)))
        return gyro_ok and tilt <= self.max_tilt_rad

    def _handle_mode_button(
        self,
        snapshot,
        *,
        pico_ready: bool,
        dual_ready: bool,
        neutral_ready: bool,
        stable: bool,
    ) -> None:
        pressed = bool(snapshot.buttons.get(self.switch_button, False))
        rising = pressed and not self._previous_mode_button
        self._previous_mode_button = pressed
        if not rising:
            return
        if not neutral_ready or not stable:
            print(
                "[TeleopLocomani] switch ignored: keep all sticks neutral "
                "and robot upright for the configured hold time"
            )
            return
        if self.mode == self.MODE_DUAL:
            self._set_mode(self.MODE_WHOLE_BODY)
        elif self.mode == self.MODE_SAFE_HOLD and pico_ready:
            self._set_mode(self.MODE_WHOLE_BODY)
        elif dual_ready:
            self._set_mode(self.MODE_DUAL)
        else:
            print(
                "[TeleopLocomani] right B ignored: wait for active PICO pose "
                "and a fresh camera frame"
            )

    def _enter_safe_hold(self, reason: str) -> None:
        if self.mode == self.MODE_SAFE_HOLD:
            return
        self._safe_hold_q = np.asarray(self.qj, dtype=np.float32).copy()
        self.mode = self.MODE_SAFE_HOLD
        self._blend_step = self.blend_steps
        print(f"[TeleopLocomani] SAFE HOLD: {reason}")

    def _safe_hold_command(self) -> ControlCommand:
        return ControlCommand(
            q_des=self._safe_hold_q,
            kp=np.asarray(self.kps * self.safe_hold_kp_scale, dtype=np.float32),
            kd=np.asarray(
                np.maximum(self.kds, self.safe_hold_min_kd),
                dtype=np.float32,
            ),
        )

    def _apply_command(self, command: ControlCommand) -> ControlCommand:
        if not (
            np.all(np.isfinite(command.q_des))
            and np.all(np.isfinite(command.kp))
            and np.all(np.isfinite(command.kd))
        ):
            raise FloatingPointError("Control command contains NaN/Inf")
        if self._blend_step < self.blend_steps:
            self._blend_step += 1
            alpha = self._blend_step / self.blend_steps
            q_des = self._blend_start.q_des * (1.0 - alpha) + command.q_des * alpha
            kp = self._blend_start.kp * (1.0 - alpha) + command.kp * alpha
            kd = self._blend_start.kd * (1.0 - alpha) + command.kd * alpha
        else:
            q_des, kp, kd = command.q_des, command.kp, command.kd
        self.cmd_q[:] = q_des
        self.cmd_qd[:] = 0.0
        self.cmd_kp[:] = kp
        self.cmd_kd[:] = kd
        self.cmd_enable = 1
        return ControlCommand(
            q_des=np.asarray(q_des, dtype=np.float32).copy(),
            kp=np.asarray(kp, dtype=np.float32).copy(),
            kd=np.asarray(kd, dtype=np.float32).copy(),
        )

    def _commit_applied_command(
        self, command: ControlCommand, fullbody
    ) -> None:
        self.dual.commit_applied_target(command.q_des)
        default = (
            self.default_qpos
            if fullbody.controller_default_qpos is None
            else fullbody.controller_default_qpos
        )
        delta_controller = np.asarray(command.q_des) - np.asarray(default)
        scaled_policy = fullbody.mapper_action.map_state_to_from(
            delta_controller
        ).astype(np.float32)
        scale = np.asarray(fullbody.action_scale, dtype=np.float32)
        raw = np.divide(
            scaled_policy,
            scale,
            out=np.zeros_like(scaled_policy),
            where=np.abs(scale) > 1.0e-8,
        )
        raw = np.clip(raw, -fullbody.action_clip, fullbody.action_clip)
        fullbody.last_action[:] = raw
        fullbody.applied_action[:] = raw * scale

    def run_locomani(self) -> None:
        fullbody = self.policies["tracking"]
        # default_qpos_state() may update cmd_q after this task object was
        # constructed (especially for metadata-driven SPV5-2). Start the first
        # policy handoff from the command actually holding the robot.
        self._blend_start = ControlCommand(
            q_des=self.cmd_q.copy(),
            kp=self.cmd_kp.copy(),
            kd=self.cmd_kd.copy(),
        )
        self._blend_step = 0
        print(
            "[TeleopLocomani] running: right A=start PICO, right B=switch policy, "
            "left X=stop PICO"
        )
        while True:
            if not self.process_state(wait_next=True, timeout_s=1.0):
                if self.state_timeout_is_fatal:
                    raise TimeoutError(
                        "No bridge state for 1s; real task is stopping so the "
                        "bridge watchdog can enter damping"
                    )
                print("[TeleopLocomani] no bridge state for 1s")
                continue
            if self.btn_rise["stop"]:
                break

            fullbody.update_obs()
            whole_action = fullbody.compute_action()
            whole_command = self._whole_body_command(whole_action)
            snapshot = self.pico_store.snapshot()
            depth = self._depth()
            now = time.monotonic()
            pico_ready = bool(
                snapshot.active
                and snapshot.has_pose
                and now - snapshot.pose_timestamp <= self.depth_timeout_s
                and now - snapshot.control_timestamp <= self.depth_timeout_s
            )
            if pico_ready:
                self._pico_session_started = True
            dual_ready = bool(pico_ready and depth is not None)
            dual_command = None
            if dual_ready:
                dual_command, reference = self.dual.step(
                    controller=self,
                    snapshot=snapshot,
                    depth=depth,
                )
                dual_ready = reference.pico_valid

            neutral_ready = self._update_neutral_hold(snapshot)
            stable = self._body_stable()
            self._handle_mode_button(
                snapshot,
                pico_ready=pico_ready,
                dual_ready=dual_ready,
                neutral_ready=neutral_ready,
                stable=stable,
            )
            if self.mode == self.MODE_DUAL and not dual_ready:
                now = time.monotonic()
                if now - self._last_camera_warning > 1.0:
                    print(
                        "[TeleopLocomani] dual input stale; entering safe hold"
                    )
                    self._last_camera_warning = now
                self._enter_safe_hold("PICO pose/control or depth stream is stale")
            elif (
                self.mode != self.MODE_SAFE_HOLD
                and self._pico_session_started
                and not pico_ready
            ):
                self._enter_safe_hold("PICO pose/control stream is stale")
            elif self.mode != self.MODE_SAFE_HOLD and not stable:
                self._enter_safe_hold("IMU tilt/angular velocity safety limit")

            if self.mode == self.MODE_SAFE_HOLD:
                command = self._safe_hold_command()
            elif self.mode == self.MODE_DUAL and dual_command is not None:
                command = dual_command
            else:
                command = whole_command
            applied = self._apply_command(command)
            self._commit_applied_command(applied, fullbody)
            self.send_cmd()
            fullbody.post_step()
            self.policy_step += 1

    def close_locomani(self) -> None:
        try:
            self.policies["tracking"].deactivate()
        finally:
            self.depth_sub.close()
            self.transport.close()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="teleop-upper-lower-locomani sim2sim/sim2real task"
    )
    parser.add_argument("--robot", choices=["g1"], default="g1")
    parser.add_argument(
        "--whole-body-policy",
        choices=("heft", "spv5_2"),
        default="heft",
        help="Whole-body policy used by mode 1",
    )
    parser.add_argument(
        "--tracking-config",
        default=None,
        help="Optional explicit whole-body tracking YAML",
    )
    parser.add_argument(
        "--task-config",
        type=Path,
        default=None,
    )
    parser.add_argument("--target", choices=("sim", "real"), default="sim")
    parser.add_argument(
        "--terrain-class",
        type=int,
        choices=(0, 1, 2),
        default=2,
        help="0=flat/rough, 1=slope, 2=stairs forward-only",
    )
    args = parser.parse_args(argv)
    # The dual task does not yet use PolicyRunRecorder's single-policy schema.
    args.no_record = True
    args.record_dir = None
    if args.tracking_config is None:
        args.tracking_config = (
            "tracking.yaml"
            if args.whole_body_policy == "heft"
            else "tracking_spv5_2.yaml"
        )

    if args.task_config is None:
        config_name = (
            "teleop-upper-lower-locomani-real.yaml"
            if args.target == "real"
            else "teleop-upper-lower-locomani.yaml"
        )
        args.task_config = SRC_ROOT.parent / "config/g1" / config_name
    task_config_path = args.task_config.expanduser().resolve()
    task_config = _load_task_config(task_config_path)
    configured_target = str(task_config.get("target", args.target))
    if configured_target != args.target:
        raise ValueError(
            f"--target {args.target} does not match task config target "
            f"{configured_target!r}: {task_config_path}"
        )
    controller = None
    try:
        controller = TeleopUpperLowerLocomaniController(
            args,
            get_config(controller_config_path(args.robot)),
            task_config,
            task_config_path,
        )
        controller.zero_torque_state()
        controller.move_to_default_qpos()
        controller.default_qpos_state()
        controller.run_locomani()
    except KeyboardInterrupt:
        print("[TeleopLocomani] interrupted")
    except Exception as exc:
        print(f"[TeleopLocomani] error: {exc}")
        traceback.print_exc()
    finally:
        if controller is not None:
            try:
                controller.set_damping_cmd()
                controller.send_cmd()
            finally:
                controller.close_locomani()


if __name__ == "__main__":
    main()
