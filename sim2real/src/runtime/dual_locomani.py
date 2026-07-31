from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import onnxruntime as ort
from scipy.spatial.transform import Rotation as R

from runtime.kinematics import RobotKinematics
from runtime.shared_pico import PicoSnapshot


LOWER_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
)
UPPER_JOINT_NAMES = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
ALL_JOINT_NAMES = LOWER_JOINT_NAMES + UPPER_JOINT_NAMES

DEFAULT_JOINT_POS = np.array(
    [
        -0.312,
        0.0,
        0.0,
        0.669,
        -0.363,
        0.0,
        -0.312,
        0.0,
        0.0,
        0.669,
        -0.363,
        0.0,
        0.0,
        0.0,
        0.0,
        0.2,
        0.2,
        0.0,
        0.6,
        0.0,
        0.0,
        0.0,
        0.2,
        -0.2,
        0.0,
        0.6,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)


def _joint_contract(names: tuple[str, ...]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    scales = []
    stiffness = []
    damping = []
    for name in names:
        if "wrist_pitch" in name or "wrist_yaw" in name:
            scale, kp, kd = 0.07450087032950714, 16.77832748089279, 1.06814150219
        elif (
            "hip_pitch" in name
            or "hip_yaw" in name
            or name == "waist_yaw_joint"
        ):
            scale, kp, kd = (
                0.5475464629911068,
                40.17923863450712,
                2.557889775413375,
            )
        elif "hip_roll" in name or "knee" in name:
            scale, kp, kd = (
                0.35066146637882434,
                99.09842777666111,
                6.308801853496639,
            )
        elif (
            "ankle" in name
            or name in {"waist_roll_joint", "waist_pitch_joint"}
        ):
            scale, kp, kd = (
                0.43857731392336724,
                28.50124619574858,
                1.814445686584846,
            )
        else:
            scale, kp, kd = (
                0.43857731392336724,
                14.25062309787429,
                0.907222843292423,
            )
        scales.append(scale)
        stiffness.append(kp)
        damping.append(kd)
    return (
        np.asarray(scales, dtype=np.float32),
        np.asarray(stiffness, dtype=np.float32),
        np.asarray(damping, dtype=np.float32),
    )


ACTION_SCALE, JOINT_KP, JOINT_KD = _joint_contract(ALL_JOINT_NAMES)


@dataclass(frozen=True)
class ControlCommand:
    q_des: np.ndarray
    kp: np.ndarray
    kd: np.ndarray


@dataclass(frozen=True)
class DualReference:
    twist: np.ndarray
    waist_rpy: np.ndarray
    upper_joint_pos: np.ndarray
    ee_pos: np.ndarray
    ee_quat: np.ndarray
    pico_valid: bool


class OnnxDualPolicy:
    def __init__(self, lower_path: str | Path, upper_path: str | Path) -> None:
        options = ort.SessionOptions()
        options.intra_op_num_threads = 4
        options.inter_op_num_threads = 1
        self.lower = ort.InferenceSession(
            str(Path(lower_path).resolve()),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        self.upper = ort.InferenceSession(
            str(Path(upper_path).resolve()),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        lower_inputs = {item.name: item.shape for item in self.lower.get_inputs()}
        expected_lower = {
            "obs": [1, 472],
            "depth": [1, 1, 36, 64],
            "lower_actor_history": [1, 5, 100],
        }
        for name, shape in expected_lower.items():
            if lower_inputs.get(name) != shape:
                raise ValueError(
                    f"Lower ONNX input {name!r}: {lower_inputs.get(name)} != {shape}"
                )
        upper_inputs = self.upper.get_inputs()
        if len(upper_inputs) != 1 or upper_inputs[0].shape != [1, 135]:
            raise ValueError(
                f"Upper ONNX must expose one [1,135] input, got "
                f"{[(item.name, item.shape) for item in upper_inputs]}"
            )
        self.upper_input_name = upper_inputs[0].name

    def act(
        self,
        lower_obs: np.ndarray,
        upper_obs: np.ndarray,
        depth: np.ndarray,
        lower_history: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        lower = self.lower.run(
            None,
            {
                "obs": lower_obs.reshape(1, 472).astype(np.float32),
                "depth": depth.reshape(1, 1, 36, 64).astype(np.float32),
                "lower_actor_history": lower_history.reshape(1, 5, 100).astype(
                    np.float32
                ),
            },
        )[0][0]
        upper = self.upper.run(
            None,
            {
                self.upper_input_name: upper_obs.reshape(1, 135).astype(
                    np.float32
                )
            },
        )[0][0]
        return (
            np.asarray(lower, dtype=np.float32),
            np.asarray(upper, dtype=np.float32),
        )


def _append_with_backfill(history: deque[np.ndarray], value: np.ndarray) -> None:
    value = np.asarray(value, dtype=np.float32).copy()
    if not history:
        history.extend(value.copy() for _ in range(history.maxlen or 1))
    else:
        history.append(value)


class ObservationBuilder:
    """Exact observation layout from mjlab-loco Dual-Depth training."""

    def __init__(self) -> None:
        self.last_action = np.zeros(29, dtype=np.float32)
        self._term_history = {
            name: deque(maxlen=5)
            for name in ("ang_vel", "gravity", "joint_pos", "joint_vel", "action")
        }
        self._proprio_history: deque[np.ndarray] = deque(maxlen=5)

    def build(
        self,
        *,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        base_ang_vel: np.ndarray,
        projected_gravity: np.ndarray,
        reference: DualReference,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        q_rel = joint_pos - DEFAULT_JOINT_POS
        terms = {
            "ang_vel": base_ang_vel,
            "gravity": projected_gravity,
            "joint_pos": q_rel,
            "joint_vel": joint_vel,
            "action": self.last_action,
        }
        for name, value in terms.items():
            _append_with_backfill(self._term_history[name], value)

        current_lower = np.concatenate(
            (
                base_ang_vel,
                projected_gravity,
                q_rel,
                joint_vel,
                self.last_action,
                reference.twist,
                reference.waist_rpy,
            )
        ).astype(np.float32)
        if current_lower.shape != (100,):
            raise RuntimeError(f"Lower current observation is {current_lower.shape}")
        _append_with_backfill(self._proprio_history, current_lower)
        lower_obs = np.concatenate(
            tuple(
                np.stack(self._term_history[name], axis=0).reshape(-1)
                for name in ("ang_vel", "gravity", "joint_pos", "joint_vel", "action")
            )
            + (reference.twist, reference.waist_rpy)
        ).astype(np.float32)
        current_upper = joint_pos[len(LOWER_JOINT_NAMES) :]
        upper_obs = np.concatenate(
            (
                base_ang_vel,
                projected_gravity,
                q_rel,
                joint_vel,
                reference.upper_joint_pos,
                reference.upper_joint_pos - current_upper,
                self.last_action,
                reference.ee_pos,
                reference.ee_quat,
            )
        ).astype(np.float32)
        history = np.stack(self._proprio_history, axis=0).astype(np.float32)
        if lower_obs.shape != (472,) or upper_obs.shape != (135,):
            raise RuntimeError(
                f"Observation shapes lower={lower_obs.shape}, upper={upper_obs.shape}"
            )
        return lower_obs, upper_obs, history

    def update_action(self, lower: np.ndarray, upper: np.ndarray) -> None:
        self.set_applied_action(np.concatenate((lower, upper)))

    def set_applied_action(self, action: np.ndarray) -> None:
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (29,):
            raise ValueError(f"Applied dual action must be (29,), got {action.shape}")
        self.last_action = action.copy()

    def reset(self, applied_action: np.ndarray | None = None) -> None:
        for history in self._term_history.values():
            history.clear()
        self._proprio_history.clear()
        if applied_action is None:
            self.last_action.fill(0.0)
        else:
            self.set_applied_action(applied_action)


class PicoDualReferenceBuilder:
    def __init__(
        self,
        *,
        kinematics_xml: str | Path,
        controller_joint_names: tuple[str, ...] | list[str],
        base_height: float = 0.65,
        pose_timeout_s: float = 0.25,
        deadband: float = 0.1,
        max_vx: float = 1.0,
        max_vy: float = 0.5,
        max_wz: float = 0.5,
        terrain_class: int = 2,
    ) -> None:
        self.controller_joint_names = tuple(controller_joint_names)
        self.kinematics = RobotKinematics(kinematics_xml, ALL_JOINT_NAMES)
        self.base_height = float(base_height)
        self.pose_timeout_s = float(pose_timeout_s)
        self.deadband = float(deadband)
        self.max_vx = float(max_vx)
        self.max_vy = float(max_vy)
        self.max_wz = float(max_wz)
        self.terrain_class = int(terrain_class)

    def _axis(self, snapshot: PicoSnapshot, name: str) -> float:
        value = float(np.clip(snapshot.sticks.get(name, 0.0), -1.0, 1.0))
        return 0.0 if abs(value) < self.deadband else value

    @staticmethod
    def _map_by_name(
        values: np.ndarray,
        source_names: tuple[str, ...] | list[str],
        target_names: tuple[str, ...] | list[str],
    ) -> np.ndarray:
        indexes = {name: index for index, name in enumerate(source_names)}
        missing = [name for name in target_names if name not in indexes]
        if missing:
            raise ValueError(f"PICO reference is missing joints: {missing}")
        return np.asarray(
            [values[indexes[name]] for name in target_names], dtype=np.float32
        )

    def build(
        self,
        snapshot: PicoSnapshot,
        *,
        current_joint_pos: np.ndarray,
    ) -> DualReference:
        current_upper = current_joint_pos[len(LOWER_JOINT_NAMES) :]
        pose_valid = (
            snapshot.active
            and snapshot.has_pose
            and time.monotonic() - snapshot.pose_timestamp <= self.pose_timeout_s
        )
        control_valid = (
            snapshot.active
            and time.monotonic() - snapshot.control_timestamp <= self.pose_timeout_s
        )
        if pose_valid:
            pico_q = self._map_by_name(
                snapshot.joint_pos, snapshot.joint_names, ALL_JOINT_NAMES
            )
            upper = pico_q[len(LOWER_JOINT_NAMES) :]
            positions, quaternions = self.kinematics.body_pose(
                pico_q,
                ("left_wrist_yaw_link", "right_wrist_yaw_link"),
                reference_body_name="torso_link",
            )
            waist = np.array(
                [
                    pico_q[ALL_JOINT_NAMES.index("waist_roll_joint")],
                    pico_q[ALL_JOINT_NAMES.index("waist_pitch_joint")],
                    pico_q[ALL_JOINT_NAMES.index("waist_yaw_joint")],
                ],
                dtype=np.float32,
            )
            waist = np.clip(
                waist,
                np.array([-0.3, -0.3, -0.5], dtype=np.float32),
                np.array([0.3, 0.3, 0.5], dtype=np.float32),
            )
        else:
            upper = current_upper.copy()
            positions, quaternions = self.kinematics.body_pose(
                current_joint_pos,
                ("left_wrist_yaw_link", "right_wrist_yaw_link"),
                reference_body_name="torso_link",
            )
            waist = np.zeros(3, dtype=np.float32)

        vx = self.max_vx * self._axis(snapshot, "ly")
        vy = self.max_vy * self._axis(snapshot, "lx")
        wz = self.max_wz * self._axis(snapshot, "rx")
        if not pose_valid or not control_valid:
            vx = vy = wz = 0.0
        elif self.terrain_class == 2:
            if vx <= 0.0:
                vx = 0.0
            else:
                vx = float(np.clip(vx, 0.3, self.max_vx))
            vy = 0.0
            wz = 0.0
        twist = np.array([vx, vy, wz, self.base_height], dtype=np.float32)
        return DualReference(
            twist=twist,
            waist_rpy=waist,
            upper_joint_pos=np.asarray(upper, dtype=np.float32),
            ee_pos=np.asarray(positions, dtype=np.float32).reshape(6),
            ee_quat=np.asarray(quaternions, dtype=np.float32).reshape(8),
            pico_valid=bool(pose_valid),
        )


class DualLocomaniRuntime:
    def __init__(
        self,
        *,
        lower_onnx: str | Path,
        upper_onnx: str | Path,
        kinematics_xml: str | Path,
        controller_joint_names: list[str],
        terrain_class: int,
    ) -> None:
        self.controller_joint_names = tuple(controller_joint_names)
        self.policy = OnnxDualPolicy(lower_onnx, upper_onnx)
        self.observation = ObservationBuilder()
        self.reference = PicoDualReferenceBuilder(
            kinematics_xml=kinematics_xml,
            controller_joint_names=controller_joint_names,
            terrain_class=terrain_class,
        )
        source_index = {name: i for i, name in enumerate(ALL_JOINT_NAMES)}
        controller_index = {
            name: i for i, name in enumerate(self.controller_joint_names)
        }
        missing = [name for name in ALL_JOINT_NAMES if name not in controller_index]
        if missing:
            raise ValueError(f"Controller is missing G1 joints: {missing}")
        self._controller_from_canonical = np.array(
            [source_index[name] for name in self.controller_joint_names],
            dtype=np.int32,
        )
        self._canonical_from_controller = np.array(
            [controller_index[name] for name in ALL_JOINT_NAMES], dtype=np.int32
        )

    def step(
        self,
        *,
        controller,
        snapshot: PicoSnapshot,
        depth: np.ndarray,
    ) -> tuple[ControlCommand, DualReference]:
        joint_pos = controller.qj[self._canonical_from_controller].astype(np.float32)
        joint_vel = controller.dqj[self._canonical_from_controller].astype(np.float32)
        quat = np.asarray(controller.quat, dtype=np.float32)
        quat /= max(float(np.linalg.norm(quat)), 1.0e-6)
        projected_gravity = R.from_quat(
            quat, scalar_first=True
        ).inv().apply(np.array([0.0, 0.0, -1.0])).astype(np.float32)
        reference = self.reference.build(
            snapshot, current_joint_pos=joint_pos
        )
        lower_obs, upper_obs, history = self.observation.build(
            joint_pos=joint_pos,
            joint_vel=joint_vel,
            base_ang_vel=np.asarray(controller.gyro, dtype=np.float32),
            projected_gravity=projected_gravity,
            reference=reference,
        )
        lower, upper = self.policy.act(
            lower_obs, upper_obs, depth, history
        )
        if lower.shape != (15,) or upper.shape != (14,):
            raise ValueError(
                f"Dual action shapes lower={lower.shape}, upper={upper.shape}"
            )
        action = np.concatenate((lower, upper))
        target = DEFAULT_JOINT_POS + action * ACTION_SCALE
        indexes = self._controller_from_canonical
        return (
            ControlCommand(
                q_des=target[indexes].astype(np.float32),
                kp=JOINT_KP[indexes].astype(np.float32),
                kd=JOINT_KD[indexes].astype(np.float32),
            ),
            reference,
        )

    def commit_applied_target(self, q_des_controller: np.ndarray) -> None:
        """Commit the command that was actually sent after blending/safety."""
        q_des = np.asarray(q_des_controller, dtype=np.float32).reshape(-1)
        if q_des.shape != (len(self.controller_joint_names),):
            raise ValueError(
                f"Controller target is {q_des.shape}, expected "
                f"{(len(self.controller_joint_names),)}"
            )
        canonical_target = q_des[self._canonical_from_controller]
        action = (canonical_target - DEFAULT_JOINT_POS) / ACTION_SCALE
        self.observation.set_applied_action(action)

    def reset_history(self, q_des_controller: np.ndarray | None = None) -> None:
        if q_des_controller is None:
            self.observation.reset()
            return
        q_des = np.asarray(q_des_controller, dtype=np.float32).reshape(-1)
        canonical_target = q_des[self._canonical_from_controller]
        self.observation.reset(
            (canonical_target - DEFAULT_JOINT_POS) / ACTION_SCALE
        )


__all__ = [
    "ALL_JOINT_NAMES",
    "ControlCommand",
    "DualLocomaniRuntime",
    "LOWER_JOINT_NAMES",
    "UPPER_JOINT_NAMES",
]
