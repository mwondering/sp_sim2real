from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

from runtime.math_utils import _quat_apply_inv, _quat_mul_wxyz, _quat_normalize_wxyz


def quat_to_rot6d_wxyz(quat: np.ndarray) -> np.ndarray:
    """Encode the first two rotation-matrix columns, column-major.

    This matches ``sp_tracking.tasks.tracking.mdp.sp._rot6d`` exactly.
    """
    quat = _quat_normalize_wxyz(np.asarray(quat, dtype=np.float32))
    matrix = R.from_quat(quat, scalar_first=True).as_matrix()
    return matrix[..., :, :2].swapaxes(-2, -1).reshape(*quat.shape[:-1], 6).astype(np.float32)


def finite_difference(values: np.ndarray, fps: float) -> np.ndarray:
    values = np.asarray(values, dtype=np.float32)
    out = np.zeros_like(values)
    if values.shape[0] < 2 or fps <= 0.0:
        return out
    out[1:-1] = (values[2:] - values[:-2]) * (float(fps) / 2.0)
    out[0] = (values[1] - values[0]) * float(fps)
    out[-1] = (values[-1] - values[-2]) * float(fps)
    return out


def smooth_avg5(values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=np.float32)
    if values.shape[0] < 2:
        return values.copy()
    padded = np.pad(values, ((2, 2), *([(0, 0)] * (values.ndim - 1))), mode="edge")
    return sum(padded[offset : offset + values.shape[0]] for offset in range(5)) / 5.0


def joint_velocity_from_positions(joint_pos: np.ndarray, fps: float) -> np.ndarray:
    return smooth_avg5(finite_difference(joint_pos, fps))


def angular_velocity_from_quaternions_wxyz(quat: np.ndarray, fps: float) -> np.ndarray:
    """HEFT-compatible quaternion finite difference in the world frame."""
    quat = _quat_normalize_wxyz(np.asarray(quat, dtype=np.float32))
    if quat.shape[0] < 2 or fps <= 0.0:
        return np.zeros((*quat.shape[:-1], 3), dtype=np.float32)

    continuous = quat.copy()
    for index in range(1, continuous.shape[0]):
        if float(np.dot(continuous[index], continuous[index - 1])) < 0.0:
            continuous[index] *= -1.0
    qdot = finite_difference(continuous, fps)
    conjugate = continuous.copy()
    conjugate[..., 1:] *= -1.0
    return (2.0 * _quat_mul_wxyz(qdot, conjugate)[..., 1:]).astype(np.float32)


@dataclass(frozen=True)
class KeypointSpec:
    name: str
    body_name: str
    local_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    local_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    correction_body_name: str | None = None
    correction_local_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)


SPV5_KEYPOINT_SPECS = (
    KeypointSpec("left_hip", "left_hip_yaw_link"),
    KeypointSpec("left_knee", "left_knee_link"),
    KeypointSpec("left_foot", "left_ankle_roll_link"),
    KeypointSpec("right_hip", "right_hip_yaw_link"),
    KeypointSpec("right_knee", "right_knee_link"),
    KeypointSpec("right_foot", "right_ankle_roll_link"),
    KeypointSpec("head", "torso_link", local_pos=(0.01, 0.0, 0.41)),
    KeypointSpec("left_shoulder", "left_shoulder_yaw_link"),
    KeypointSpec("left_wrist", "left_wrist_roll_link"),
    KeypointSpec(
        "left_hand",
        "left_wrist_yaw_link",
        local_pos=(0.116, 0.0, 0.0),
        correction_body_name="left_wrist_pitch_link",
        correction_local_pos=(0.005, 0.0, 0.0),
    ),
    KeypointSpec("right_shoulder", "right_shoulder_yaw_link"),
    KeypointSpec("right_wrist", "right_wrist_roll_link"),
    KeypointSpec(
        "right_hand",
        "right_wrist_yaw_link",
        local_pos=(0.116, 0.0, 0.0),
        correction_body_name="right_wrist_pitch_link",
        correction_local_pos=(0.005, 0.0, 0.0),
    ),
)


class RobotKinematics:
    """Small MuJoCo FK wrapper used by deploy-time actor observations."""

    def __init__(self, xml_path: str | Path, joint_names: Sequence[str]):
        self.xml_path = Path(xml_path)
        self.model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.data = mujoco.MjData(self.model)
        self.joint_names = tuple(str(name) for name in joint_names)
        self._qpos_adrs: list[int] = []
        for name in self.joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise ValueError(f"Joint {name!r} is absent from {self.xml_path}")
            self._qpos_adrs.append(int(self.model.jnt_qposadr[joint_id]))

        self.root_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        if self.root_body_id < 0:
            raise ValueError(f"Body 'pelvis' is absent from {self.xml_path}")

    def _body_ids(self, body_names: Iterable[str]) -> np.ndarray:
        ids = []
        for name in body_names:
            body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, str(name))
            if body_id < 0:
                raise ValueError(f"Body {name!r} is absent from {self.xml_path}")
            ids.append(body_id)
        return np.asarray(ids, dtype=np.int32)

    def body_pose(
        self,
        joint_pos: np.ndarray,
        body_names: Sequence[str],
        *,
        reference_body_name: str = "pelvis",
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return body poses relative to ``reference_body_name``."""
        joints = np.asarray(joint_pos, dtype=np.float64)
        single = joints.ndim == 1
        if single:
            joints = joints[None, :]
        if joints.ndim != 2 or joints.shape[1] != len(self.joint_names):
            raise ValueError(
                f"joint_pos must have shape [T, {len(self.joint_names)}], got {joints.shape}"
            )
        body_ids = self._body_ids(body_names)
        reference_body_id = int(self._body_ids((reference_body_name,))[0])
        positions = np.empty((joints.shape[0], len(body_ids), 3), dtype=np.float32)
        quaternions = np.empty((joints.shape[0], len(body_ids), 4), dtype=np.float32)

        for frame_index, frame in enumerate(joints):
            self.data.qpos[:] = 0.0
            self.data.qpos[3] = 1.0
            self.data.qpos[self._qpos_adrs] = frame
            self.data.qvel[:] = 0.0
            mujoco.mj_forward(self.model, self.data)

            root_pos = self.data.xpos[reference_body_id]
            root_quat = self.data.xquat[reference_body_id]
            positions[frame_index] = _quat_apply_inv(
                root_quat,
                self.data.xpos[body_ids] - root_pos,
            )
            root_rotation = R.from_quat(root_quat, scalar_first=True)
            body_rotation = R.from_quat(self.data.xquat[body_ids], scalar_first=True)
            quaternions[frame_index] = (root_rotation.inv() * body_rotation).as_quat(
                scalar_first=True
            )

        if single:
            return positions[0], quaternions[0]
        return positions, quaternions

    def body_state(
        self,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        root_ang_vel_b: np.ndarray,
        body_names: Sequence[str],
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return root-frame pose and velocity for the current robot state."""
        joint_pos = np.asarray(joint_pos, dtype=np.float32).reshape(-1)
        joint_vel = np.asarray(joint_vel, dtype=np.float32).reshape(-1)
        if joint_pos.size != len(self.joint_names) or joint_vel.size != len(self.joint_names):
            raise ValueError("Current joint position/velocity dimensions do not match FK joint names")

        eps = 1.0e-4
        pos, quat = self.body_pose(joint_pos, body_names)
        pos_minus, quat_minus = self.body_pose(joint_pos - eps * joint_vel, body_names)
        pos_plus, quat_plus = self.body_pose(joint_pos + eps * joint_vel, body_names)
        relative_rotation = R.from_quat(quat_plus, scalar_first=True) * R.from_quat(
            quat_minus, scalar_first=True
        ).inv()
        relative_ang_vel = relative_rotation.as_rotvec().astype(np.float32) / (2.0 * eps)
        root_ang_vel_b = np.asarray(root_ang_vel_b, dtype=np.float32).reshape(3)
        lin_vel = (pos_plus - pos_minus) / (2.0 * eps)
        lin_vel += np.cross(np.broadcast_to(root_ang_vel_b, pos.shape), pos)
        return pos, quat, lin_vel.astype(np.float32), relative_ang_vel

    def semantic_keypoint_state(
        self,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        root_ang_vel_b: np.ndarray,
        specs: Sequence[KeypointSpec] = SPV5_KEYPOINT_SPECS,
    ) -> np.ndarray:
        physical_names: list[str] = []
        for spec in specs:
            for name in (spec.body_name, spec.correction_body_name or spec.body_name):
                if name not in physical_names:
                    physical_names.append(name)
        pos, quat, lin_vel, ang_vel = self.body_state(
            joint_pos,
            joint_vel,
            root_ang_vel_b,
            physical_names,
        )
        indexes = {name: index for index, name in enumerate(physical_names)}

        semantic_pos = []
        semantic_quat = []
        semantic_lin_vel = []
        semantic_ang_vel = []
        for spec in specs:
            parent = indexes[spec.body_name]
            correction = indexes[spec.correction_body_name or spec.body_name]
            parent_rotation = R.from_quat(quat[parent], scalar_first=True)
            correction_rotation = R.from_quat(quat[correction], scalar_first=True)
            offset = parent_rotation.apply(np.asarray(spec.local_pos, dtype=np.float32))
            correction_offset = correction_rotation.apply(
                np.asarray(spec.correction_local_pos, dtype=np.float32)
            )
            semantic_pos.append(pos[parent] + offset + correction_offset)
            semantic_quat.append(
                (
                    parent_rotation
                    * R.from_quat(np.asarray(spec.local_quat), scalar_first=True)
                ).as_quat(scalar_first=True)
            )
            semantic_lin_vel.append(
                lin_vel[parent]
                + np.cross(ang_vel[parent], offset)
                + np.cross(ang_vel[correction], correction_offset)
            )
            semantic_ang_vel.append(ang_vel[parent])

        semantic_pos_array = np.asarray(semantic_pos, dtype=np.float32)
        semantic_quat_array = np.asarray(semantic_quat, dtype=np.float32)
        return np.concatenate(
            (
                semantic_pos_array.reshape(-1),
                quat_to_rot6d_wxyz(semantic_quat_array).reshape(-1),
                np.asarray(semantic_lin_vel, dtype=np.float32).reshape(-1),
                np.asarray(semantic_ang_vel, dtype=np.float32).reshape(-1),
            )
        ).astype(np.float32)


__all__ = [
    "RobotKinematics",
    "SPV5_KEYPOINT_SPECS",
    "angular_velocity_from_quaternions_wxyz",
    "finite_difference",
    "joint_velocity_from_positions",
    "quat_to_rot6d_wxyz",
    "smooth_avg5",
]
