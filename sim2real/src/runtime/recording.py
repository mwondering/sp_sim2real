from __future__ import annotations

import re
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np

from paths import SIM2REAL_ROOT


def _sanitize_filename_part(value: str) -> str:
    clean = re.sub(r"[^A-Za-z0-9._-]+", "_", str(value).strip())
    clean = clean.strip("._-")
    return clean or "unknown"


def _ckpt_name_from_policy_path(policy_path: str | Path) -> str:
    path = Path(policy_path)
    if path.parent.name:
        return path.parent.name
    return path.stem


class PolicyRunRecorder:
    def __init__(
        self,
        *,
        robot: str,
        policy,
        joint_names: list[str],
        control_dt: float,
        output_root: Optional[str | Path] = None,
    ):
        self.robot = _sanitize_filename_part(robot)
        self.policy = policy
        self.joint_names = list(joint_names)
        self.control_dt = float(control_dt)
        if output_root is None:
            self.output_root = SIM2REAL_ROOT / "assets" / "policy_logs"
        else:
            root = Path(output_root)
            self.output_root = root if root.is_absolute() else SIM2REAL_ROOT / root

        self.ckpt_name = _sanitize_filename_part(_ckpt_name_from_policy_path(policy.policy_path))
        self.policy_observation_key = str(policy.input_key)
        self.policy_observation_dim = int(policy.num_obs)
        policy_config = getattr(policy, "config", None)
        self.policy_config_path = str(getattr(policy_config, "_config_path", ""))
        self.output_dir = self.output_root / self.robot
        self.output_path = self._next_output_path()

        self._saved = False
        self._frames: list[dict[str, np.ndarray | np.integer | np.floating | str]] = []
        self._start_wall_ns = time.time_ns()
        self._start_monotonic = time.monotonic()

    def _next_output_path(self) -> Path:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"{timestamp}_{self.ckpt_name}"
        path = self.output_dir / f"{base}.npz"
        if not path.exists():
            return path
        for idx in range(1, 1000):
            candidate = self.output_dir / f"{base}_{idx:03d}.npz"
            if not candidate.exists():
                return candidate
        raise RuntimeError(f"Cannot find free recorder output path under {self.output_dir}")

    @property
    def frame_count(self) -> int:
        return len(self._frames)

    def record_step(
        self,
        controller,
        action_delta: np.ndarray,
        *,
        policy_observation: np.ndarray,
    ) -> None:
        policy = self.policy
        observation = np.asarray(policy_observation, dtype=np.float32)
        expected_shape = (self.policy_observation_dim,)
        if observation.shape != expected_shape:
            raise ValueError(
                f"Policy observation has shape {observation.shape}, expected {expected_shape}"
            )

        state_seq = getattr(controller, "last_state_seq", None)
        state_receive_time_ns = getattr(controller, "last_state_receive_time_ns", None)
        dof_size = int(np.asarray(controller.qj).size)
        missing_motor_float = np.full(dof_size, np.nan, dtype=np.float32)
        missing_motor_u32 = np.zeros(dof_size, dtype=np.uint32)

        def motor_array(name: str, dtype, fallback: np.ndarray) -> np.ndarray:
            value = np.asarray(getattr(controller, name, fallback), dtype=dtype)
            if value.shape != (dof_size,):
                raise ValueError(
                    f"Controller {name} has shape {value.shape}, expected {(dof_size,)}"
                )
            return value.copy()

        self._frames.append(
            {
                "wall_time_ns": np.int64(time.time_ns()),
                "monotonic_time_s": np.float64(time.monotonic()),
                "policy_step": np.int64(controller.policy_step),
                "state_seq": np.int64(-1 if state_seq is None else state_seq),
                "state_receive_time_ns": np.int64(
                    -1 if state_receive_time_ns is None else state_receive_time_ns
                ),
                "policy_observation": observation.copy(),
                "joint_pos": controller.qj.astype(np.float32, copy=True),
                "joint_vel": controller.dqj.astype(np.float32, copy=True),
                "joint_torque": controller.tau.astype(np.float32, copy=True),
                "joint_torque_latest": controller.tau_latest.astype(
                    np.float32, copy=True
                ),
                "motor_acceleration": motor_array(
                    "motor_ddq", np.float32, missing_motor_float
                ),
                "motor_temperature_casing": motor_array(
                    "motor_temperature_casing", np.float32, missing_motor_float
                ),
                "motor_temperature_winding": motor_array(
                    "motor_temperature_winding", np.float32, missing_motor_float
                ),
                "motor_voltage": motor_array(
                    "motor_voltage", np.float32, missing_motor_float
                ),
                "motor_mode": motor_array("motor_mode", np.uint32, missing_motor_u32),
                "motor_sensor_0": motor_array(
                    "motor_sensor_0", np.uint32, missing_motor_u32
                ),
                "motor_sensor_1": motor_array(
                    "motor_sensor_1", np.uint32, missing_motor_u32
                ),
                "motor_state": motor_array("motor_state", np.uint32, missing_motor_u32),
                "motor_reserve_0": motor_array(
                    "motor_reserve_0", np.uint32, missing_motor_u32
                ),
                "motor_reserve_1": motor_array(
                    "motor_reserve_1", np.uint32, missing_motor_u32
                ),
                "motor_reserve_2": motor_array(
                    "motor_reserve_2", np.uint32, missing_motor_u32
                ),
                "motor_reserve_3": motor_array(
                    "motor_reserve_3", np.uint32, missing_motor_u32
                ),
                "motor_diagnostic_flags": motor_array(
                    "motor_diagnostic_flags", np.uint32, missing_motor_u32
                ),
                "motor_diagnostic_flag_schema": np.uint32(
                    getattr(controller, "motor_diagnostic_flag_schema", 0)
                ),
                "diagnostic_warning_count": np.uint32(
                    getattr(controller, "diagnostic_warning_count", 0)
                ),
                "diagnostic_critical_count": np.uint32(
                    getattr(controller, "diagnostic_critical_count", 0)
                ),
                "diagnostic_imu_flags": np.uint32(
                    getattr(controller, "diagnostic_imu_flags", 0)
                ),
                "mode_machine": np.int32(getattr(controller, "mode_machine", -1)),
                "action_delta": np.asarray(action_delta, dtype=np.float32).copy(),
                "cmd_q": controller.cmd_q.astype(np.float32, copy=True),
                "cmd_qd": controller.cmd_qd.astype(np.float32, copy=True),
                "cmd_kp": controller.cmd_kp.astype(np.float32, copy=True),
                "cmd_kd": controller.cmd_kd.astype(np.float32, copy=True),
                "cmd_enable": np.int8(controller.cmd_enable),
                "imu_angvel": controller.gyro.astype(np.float32, copy=True),
                "imu_linacc": controller.linacc.astype(np.float32, copy=True),
                "imu_quat_wxyz": controller.quat.astype(np.float32, copy=True),
                "policy_action_raw": policy.last_action.astype(np.float32, copy=True),
                "policy_action_scaled": policy.applied_action.astype(np.float32, copy=True),
                "ref_idx": np.int64(getattr(policy, "ref_idx", -1)),
                "ref_len": np.int64(getattr(policy, "ref_len", -1)),
                "motion_name": str(getattr(policy, "current_name", "")),
            }
        )

    def save(self) -> Optional[Path]:
        if self._saved:
            return self.output_path
        self._saved = True

        if len(self._frames) == 0:
            print("[PolicyRunRecorder] no policy frames captured; skip save")
            return None

        keys = self._frames[0].keys()
        payload = {}
        for key in keys:
            payload[key] = np.asarray([frame[key] for frame in self._frames])

        payload.update(
            {
                "robot": np.asarray(self.robot),
                "ckpt_name": np.asarray(self.ckpt_name),
                "policy_name": np.asarray(str(getattr(self.policy, "name", ""))),
                "policy_path": np.asarray(str(getattr(self.policy, "policy_path", ""))),
                "policy_config_path": np.asarray(self.policy_config_path),
                "actor_profile": np.asarray(
                    str(getattr(self.policy, "actor_profile", "legacy"))
                ),
                "policy_observation_key": np.asarray(self.policy_observation_key),
                "policy_observation_dim": np.int64(self.policy_observation_dim),
                "joint_names": np.asarray(self.joint_names),
                "action_joint_names": np.asarray(list(getattr(self.policy, "action_joint_names", []))),
                "control_dt": np.float32(self.control_dt),
                "control_freq": np.float32(1.0 / self.control_dt if self.control_dt > 0.0 else 0.0),
                "start_wall_time_ns": np.int64(self._start_wall_ns),
                "start_monotonic_time_s": np.float64(self._start_monotonic),
            }
        )

        np.savez_compressed(self.output_path, **payload)
        print(f"[PolicyRunRecorder] saved={self.output_path} frames={len(self._frames)}")
        return self.output_path
