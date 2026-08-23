from __future__ import annotations

from collections.abc import Sequence

import numpy as np


MOTOR_DIAGNOSTIC_FLAG_SCHEMA = 1

MOTOR_STATE_FAULT = 1 << 0
CASING_TEMPERATURE_WARNING = 1 << 1
CASING_TEMPERATURE_LIMIT = 1 << 2
WINDING_TEMPERATURE_WARNING = 1 << 3
WINDING_TEMPERATURE_LIMIT = 1 << 4
JOINT_POSITION_WARNING = 1 << 5
JOINT_POSITION_LIMIT = 1 << 6
JOINT_VELOCITY_WARNING = 1 << 7
JOINT_VELOCITY_LIMIT = 1 << 8
JOINT_TORQUE_WARNING = 1 << 9
JOINT_TORQUE_LIMIT = 1 << 10
MOTOR_STATE_NON_FINITE = 1 << 11

IMU_ANGULAR_VELOCITY_WARNING = 1 << 0
IMU_ANGULAR_VELOCITY_LIMIT = 1 << 1
IMU_STATE_NON_FINITE = 1 << 2

MOTOR_WARNING_MASK = (
    CASING_TEMPERATURE_WARNING
    | WINDING_TEMPERATURE_WARNING
    | JOINT_POSITION_WARNING
    | JOINT_VELOCITY_WARNING
    | JOINT_TORQUE_WARNING
)
MOTOR_CRITICAL_MASK = (
    MOTOR_STATE_FAULT
    | CASING_TEMPERATURE_LIMIT
    | WINDING_TEMPERATURE_LIMIT
    | JOINT_POSITION_LIMIT
    | JOINT_VELOCITY_LIMIT
    | JOINT_TORQUE_LIMIT
    | MOTOR_STATE_NON_FINITE
)
IMU_CRITICAL_MASK = IMU_ANGULAR_VELOCITY_LIMIT | IMU_STATE_NON_FINITE

_MOTOR_FLAG_NAMES = (
    (MOTOR_STATE_FAULT, "motor_state_fault"),
    (CASING_TEMPERATURE_WARNING, "casing_temperature_warning"),
    (CASING_TEMPERATURE_LIMIT, "casing_temperature_limit"),
    (WINDING_TEMPERATURE_WARNING, "winding_temperature_warning"),
    (WINDING_TEMPERATURE_LIMIT, "winding_temperature_limit"),
    (JOINT_POSITION_WARNING, "joint_position_warning"),
    (JOINT_POSITION_LIMIT, "joint_position_limit"),
    (JOINT_VELOCITY_WARNING, "joint_velocity_warning"),
    (JOINT_VELOCITY_LIMIT, "joint_velocity_limit"),
    (JOINT_TORQUE_WARNING, "joint_torque_warning"),
    (JOINT_TORQUE_LIMIT, "joint_torque_limit"),
    (MOTOR_STATE_NON_FINITE, "non_finite_motor_feedback"),
)

_IMU_FLAG_NAMES = (
    (IMU_ANGULAR_VELOCITY_WARNING, "imu_angular_velocity_warning"),
    (IMU_ANGULAR_VELOCITY_LIMIT, "imu_angular_velocity_limit"),
    (IMU_STATE_NON_FINITE, "non_finite_imu_feedback"),
)


def _active_names(flags: int, definitions: Sequence[tuple[int, str]]) -> str:
    return ",".join(name for bit, name in definitions if int(flags) & bit)


class MotorDiagnosticReporter:
    """Emit concise deploy-side messages when bridge diagnostic state changes."""

    def __init__(self, joint_names: Sequence[str]):
        self._joint_names = tuple(str(name) for name in joint_names)
        self._last_signature: tuple[bytes, bytes, int] | None = None

    def update(
        self,
        *,
        flags: np.ndarray,
        motor_state: np.ndarray,
        casing_temperature: np.ndarray,
        winding_temperature: np.ndarray,
        q: np.ndarray,
        dq: np.ndarray,
        tau: np.ndarray,
        imu_flags: int,
    ) -> list[str]:
        flags = np.asarray(flags, dtype=np.uint32)
        motor_state = np.asarray(motor_state, dtype=np.uint32)
        expected_shape = (len(self._joint_names),)
        if flags.shape != expected_shape or motor_state.shape != expected_shape:
            raise ValueError(
                "Motor diagnostic arrays must match policy_joint_names: "
                f"flags={flags.shape}, motor_state={motor_state.shape}, expected={expected_shape}"
            )

        signature = (flags.tobytes(), motor_state.tobytes(), int(imu_flags))
        if signature == self._last_signature:
            return []
        first_update = self._last_signature is None
        previously_active = (
            False
            if first_update
            else any(self._last_signature[0]) or int(self._last_signature[2]) != 0
        )
        self._last_signature = signature

        active = bool(np.any(flags)) or int(imu_flags) != 0
        if not active:
            if first_update:
                return [
                    f"[Deploy][MotorDiag] telemetry connected schema={MOTOR_DIAGNOSTIC_FLAG_SCHEMA} status=OK"
                ]
            if previously_active:
                return ["[Deploy][MotorDiag][RECOVERED] all reported conditions cleared"]
            return []

        casing_temperature = np.asarray(casing_temperature, dtype=np.float32)
        winding_temperature = np.asarray(winding_temperature, dtype=np.float32)
        q = np.asarray(q, dtype=np.float32)
        dq = np.asarray(dq, dtype=np.float32)
        tau = np.asarray(tau, dtype=np.float32)
        for name, values in (
            ("motor_temperature_casing", casing_temperature),
            ("motor_temperature_winding", winding_temperature),
            ("q", q),
            ("dq", dq),
            ("tau", tau),
        ):
            if values.shape != expected_shape:
                raise ValueError(f"{name} has shape {values.shape}, expected {expected_shape}")

        lines: list[str] = []
        for index in np.flatnonzero(flags):
            joint_flags = int(flags[index])
            level = "CRITICAL" if joint_flags & MOTOR_CRITICAL_MASK else "WARNING"
            lines.append(
                f"[Deploy][MotorDiag][{level}] joint={self._joint_names[index]} "
                f"flags={_active_names(joint_flags, _MOTOR_FLAG_NAMES)} "
                f"motorstate=0x{int(motor_state[index]):x} q={float(q[index]):.3f}rad "
                f"dq={float(dq[index]):.3f}rad/s tau={float(tau[index]):.2f}Nm "
                f"casing={float(casing_temperature[index]):.1f}C "
                f"winding={float(winding_temperature[index]):.1f}C"
            )
        if int(imu_flags) != 0:
            level = "CRITICAL" if int(imu_flags) & IMU_CRITICAL_MASK else "WARNING"
            lines.append(
                f"[Deploy][MotorDiag][{level}] flags="
                f"{_active_names(int(imu_flags), _IMU_FLAG_NAMES)}"
            )
        return lines
