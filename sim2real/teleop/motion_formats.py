from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

from utils.math import quat_xyzw_to_wxyz_np


# IsaacLab stores G1 joints in this articulation order. Sonic/SP_Tracking
# exports use the same order but usually do not embed joint_names in the NPZ.
ISAACLAB_G1_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "waist_yaw_joint",
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "waist_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
)

ROBOT_REQUIRED_FIELDS = frozenset({"root_pos", "root_rot", "dof_pos", "joint_names"})
ISAACLAB_REQUIRED_FIELDS = frozenset({"joint_pos", "body_pos_w", "body_quat_w"})
ISAACLAB_EXPORT_FIELDS = (
    "fps",
    "joint_pos",
    "joint_vel",
    "body_pos_w",
    "body_quat_w",
    "body_lin_vel_w",
    "body_ang_vel_w",
)


def decode_names(values: np.ndarray) -> tuple[str, ...]:
    return tuple(
        value.decode("utf-8") if isinstance(value, (bytes, np.bytes_)) else str(value)
        for value in np.asarray(values).reshape(-1).tolist()
    )


def read_fps(data: np.lib.npyio.NpzFile, *, path: Path) -> float:
    fps = float(np.asarray(data["fps"]).reshape(())) if "fps" in data.files else 50.0
    if not np.isfinite(fps) or fps <= 0.0:
        raise ValueError(f"fps must be finite and positive, got {fps}: {path}")
    return fps


def normalize_quaternions_wxyz(values: np.ndarray, *, label: str) -> np.ndarray:
    quaternions = np.asarray(values, dtype=np.float32)
    if quaternions.shape[-1:] != (4,):
        raise ValueError(f"{label} must end in quaternion dimension 4, got {quaternions.shape}")
    if not np.isfinite(quaternions).all():
        raise ValueError(f"{label} contains non-finite values")
    norms = np.linalg.norm(quaternions, axis=-1, keepdims=True)
    if np.any(norms < 1.0e-6):
        raise ValueError(f"{label} contains a zero-length quaternion")
    return np.asarray(quaternions / norms, dtype=np.float32)


def reindex_joint_array(
    values: np.ndarray,
    *,
    source_names: Sequence[str],
    target_names: Sequence[str],
    label: str = "joint_pos",
) -> np.ndarray:
    joints = np.asarray(values, dtype=np.float32)
    if joints.ndim != 2:
        raise ValueError(f"{label} must have shape [T, J], got {joints.shape}")

    source = tuple(str(name) for name in source_names)
    target = tuple(str(name) for name in target_names)
    if joints.shape[1] != len(source):
        raise ValueError(
            f"{label} has {joints.shape[1]} columns but source_names has {len(source)} entries"
        )
    if len(set(source)) != len(source):
        raise ValueError("source joint names contain duplicates")
    if len(set(target)) != len(target):
        raise ValueError("target joint names contain duplicates")

    index_by_name = {name: index for index, name in enumerate(source)}
    missing = [name for name in target if name not in index_by_name]
    if missing:
        raise ValueError(f"motion is missing target joints: {missing}")
    indices = [index_by_name[name] for name in target]
    return np.ascontiguousarray(joints[:, indices], dtype=np.float32)


def _validate_reference_arrays(
    *,
    path: Path,
    joint_pos: np.ndarray,
    root_pos: np.ndarray,
    root_quat_wxyz: np.ndarray,
) -> None:
    if joint_pos.ndim != 2:
        raise ValueError(f"joint positions must be [T,J], got {joint_pos.shape}: {path}")
    if joint_pos.shape[0] == 0:
        raise ValueError(f"motion has no frames: {path}")
    if root_pos.ndim != 2 or root_pos.shape[1] != 3:
        raise ValueError(f"root position must be [T,3], got {root_pos.shape}: {path}")
    if root_quat_wxyz.ndim != 2 or root_quat_wxyz.shape[1] != 4:
        raise ValueError(
            f"root quaternion must be [T,4], got {root_quat_wxyz.shape}: {path}"
        )
    if root_pos.shape[0] != joint_pos.shape[0] or root_quat_wxyz.shape[0] != joint_pos.shape[0]:
        raise ValueError(
            "motion frame counts differ: "
            f"joint={joint_pos.shape}, root_pos={root_pos.shape}, "
            f"root_quat={root_quat_wxyz.shape}: {path}"
        )
    if not np.isfinite(joint_pos).all() or not np.isfinite(root_pos).all():
        raise ValueError(f"motion contains non-finite joint or root positions: {path}")


def load_reference_motion(
    path: Path,
    target_joint_names: Sequence[str],
    *,
    root_body_index: int = 0,
) -> tuple[np.ndarray, float, str]:
    """Load either robot-motion or IsaacLab/Sonic NPZ as G1 qpos.

    Robot-motion files contain root_pos/root_rot(xyzw)/dof_pos/joint_names.
    IsaacLab/Sonic files contain joint_pos/body_pos_w/body_quat_w(wxyz). Files
    without embedded joint_names use the fixed IsaacLab G1 articulation order.
    """

    motion_path = Path(path).expanduser().resolve()
    with np.load(motion_path, allow_pickle=False) as data:
        fields = set(data.files)
        fps = read_fps(data, path=motion_path)

        if ROBOT_REQUIRED_FIELDS.issubset(fields):
            joint_pos = np.asarray(data["dof_pos"], dtype=np.float32)
            root_pos = np.asarray(data["root_pos"], dtype=np.float32)
            root_rot_xyzw = np.asarray(data["root_rot"], dtype=np.float32)
            if root_rot_xyzw.ndim != 2 or root_rot_xyzw.shape[1] != 4:
                raise ValueError(
                    f"root_rot must have shape [T,4] in xyzw order, got "
                    f"{root_rot_xyzw.shape}: {motion_path}"
                )
            root_quat_wxyz = quat_xyzw_to_wxyz_np(root_rot_xyzw)
            source_names = decode_names(data["joint_names"])
            schema = "robot"
        elif ISAACLAB_REQUIRED_FIELDS.issubset(fields):
            joint_pos = np.asarray(data["joint_pos"], dtype=np.float32)
            body_pos_w = np.asarray(data["body_pos_w"], dtype=np.float32)
            body_quat_w = np.asarray(data["body_quat_w"], dtype=np.float32)
            if body_pos_w.ndim != 3 or body_pos_w.shape[2] != 3:
                raise ValueError(
                    f"body_pos_w must have shape [T,B,3], got {body_pos_w.shape}: {motion_path}"
                )
            if body_quat_w.ndim != 3 or body_quat_w.shape[2] != 4:
                raise ValueError(
                    f"body_quat_w must have shape [T,B,4] in wxyz order, got "
                    f"{body_quat_w.shape}: {motion_path}"
                )
            if body_pos_w.shape[:2] != body_quat_w.shape[:2]:
                raise ValueError(
                    f"body pose shapes differ: {body_pos_w.shape} vs {body_quat_w.shape}: "
                    f"{motion_path}"
                )
            if root_body_index < 0 or root_body_index >= body_pos_w.shape[1]:
                raise ValueError(
                    f"root_body_index={root_body_index} is outside body dimension "
                    f"{body_pos_w.shape[1]}: {motion_path}"
                )
            root_pos = body_pos_w[:, root_body_index]
            root_quat_wxyz = body_quat_w[:, root_body_index]
            if "joint_names" in fields:
                source_names = decode_names(data["joint_names"])
                schema = "isaaclab/embedded"
            else:
                source_names = ISAACLAB_G1_JOINT_NAMES
                schema = "isaaclab/g1"
        else:
            raise ValueError(
                f"unsupported motion fields in {motion_path}; expected either "
                f"{sorted(ROBOT_REQUIRED_FIELDS)} or {sorted(ISAACLAB_REQUIRED_FIELDS)}, "
                f"got {sorted(fields)}"
            )

    root_pos = np.asarray(root_pos, dtype=np.float32)
    root_quat_wxyz = normalize_quaternions_wxyz(
        root_quat_wxyz, label=f"root quaternion in {motion_path}"
    )
    joint_pos = reindex_joint_array(
        joint_pos,
        source_names=source_names,
        target_names=target_joint_names,
    )
    _validate_reference_arrays(
        path=motion_path,
        joint_pos=joint_pos,
        root_pos=root_pos,
        root_quat_wxyz=root_quat_wxyz,
    )

    qpos = np.concatenate((root_pos, root_quat_wxyz, joint_pos), axis=1).astype(
        np.float32, copy=False
    )
    return np.ascontiguousarray(qpos), fps, schema
