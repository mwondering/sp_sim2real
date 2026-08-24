#!/usr/bin/env python3
"""Convert repository G1 robot-motion NPZ files to IsaacLab/Sonic format."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import tempfile
from typing import Any

import numpy as np

from motion_formats import (
    ISAACLAB_EXPORT_FIELDS,
    ISAACLAB_G1_JOINT_NAMES,
    ROBOT_REQUIRED_FIELDS,
    decode_names,
    normalize_quaternions_wxyz,
    read_fps,
    reindex_joint_array,
)
from retarget.fk import LocalKinematicsModel
from utils.math import (
    quat_apply_np,
    quat_mul_np,
    quat_wxyz_to_xyzw_np,
    quat_xyzw_to_wxyz_np,
)


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = SIM2REAL_ROOT.parent
DEFAULT_INPUT_ROOT = SIM2REAL_ROOT / "config/g1/motions"
DEFAULT_OUTPUT_ROOT = REPOSITORY_ROOT / "motion"


def finite_difference(values: np.ndarray, fps: float) -> np.ndarray:
    values_arr = np.asarray(values, dtype=np.float32)
    output = np.zeros_like(values_arr)
    if values_arr.shape[0] < 2:
        return output
    output[1:-1] = (values_arr[2:] - values_arr[:-2]) * (float(fps) / 2.0)
    output[0] = (values_arr[1] - values_arr[0]) * float(fps)
    output[-1] = (values_arr[-1] - values_arr[-2]) * float(fps)
    return output


def make_quaternions_continuous_wxyz(values: np.ndarray) -> np.ndarray:
    quaternions = np.array(values, dtype=np.float32, copy=True)
    for frame_index in range(1, quaternions.shape[0]):
        flip = np.sum(
            quaternions[frame_index] * quaternions[frame_index - 1], axis=-1
        ) < 0.0
        quaternions[frame_index] = np.where(
            np.asarray(flip)[..., None],
            -quaternions[frame_index],
            quaternions[frame_index],
        )
    return quaternions


def angular_velocity_from_quaternions_wxyz(values: np.ndarray, fps: float) -> np.ndarray:
    quaternions = normalize_quaternions_wxyz(values, label="body_quat_w")
    quaternions = make_quaternions_continuous_wxyz(quaternions)
    quaternion_derivative = finite_difference(quaternions, fps)
    conjugate = quaternions.copy()
    conjugate[..., 1:] *= -1.0
    angular_velocity = 2.0 * quat_mul_np(quaternion_derivative, conjugate)[..., 1:]
    return np.asarray(angular_velocity, dtype=np.float32)


def _validate_source_arrays(
    *,
    source_path: Path,
    root_pos: np.ndarray,
    root_rot_xyzw: np.ndarray,
    dof_pos: np.ndarray,
    joint_names: tuple[str, ...],
) -> None:
    if root_pos.ndim != 2 or root_pos.shape[1] != 3:
        raise ValueError(f"root_pos must have shape [T,3], got {root_pos.shape}: {source_path}")
    if root_rot_xyzw.ndim != 2 or root_rot_xyzw.shape[1] != 4:
        raise ValueError(
            f"root_rot must have shape [T,4] in xyzw order, got "
            f"{root_rot_xyzw.shape}: {source_path}"
        )
    if dof_pos.ndim != 2:
        raise ValueError(f"dof_pos must have shape [T,J], got {dof_pos.shape}: {source_path}")
    if dof_pos.shape[0] == 0:
        raise ValueError(f"motion has no frames: {source_path}")
    if root_pos.shape[0] != dof_pos.shape[0] or root_rot_xyzw.shape[0] != dof_pos.shape[0]:
        raise ValueError(
            f"motion frame counts differ: root_pos={root_pos.shape}, "
            f"root_rot={root_rot_xyzw.shape}, dof_pos={dof_pos.shape}: {source_path}"
        )
    if dof_pos.shape[1] != len(joint_names):
        raise ValueError(
            f"dof_pos has {dof_pos.shape[1]} columns but joint_names has "
            f"{len(joint_names)} entries: {source_path}"
        )
    if not all(np.isfinite(values).all() for values in (root_pos, root_rot_xyzw, dof_pos)):
        raise ValueError(f"motion contains non-finite values: {source_path}")


def _root_first_body_order(data: Any, body_count: int) -> np.ndarray:
    order = np.arange(body_count, dtype=np.int64)
    if "body_names" not in data.files:
        return order
    body_names = decode_names(data["body_names"])
    if len(body_names) != body_count:
        raise ValueError(
            f"body_names has {len(body_names)} entries but body arrays have {body_count} bodies"
        )
    root_name = "pelvis"
    if "root_link" in data.files:
        root_name = decode_names(np.asarray(data["root_link"]))[0]
    if root_name not in body_names:
        raise ValueError(f"root body {root_name!r} is absent from body_names")
    root_index = body_names.index(root_name)
    if root_index == 0:
        return order
    return np.concatenate(([root_index], order[order != root_index]))


def _world_bodies_from_local(
    *,
    root_pos: np.ndarray,
    root_quat_wxyz: np.ndarray,
    local_body_pos: np.ndarray,
    local_body_rot_xyzw: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    frame_count = root_pos.shape[0]
    if (
        local_body_pos.ndim != 3
        or local_body_pos.shape[0] != frame_count
        or local_body_pos.shape[2] != 3
    ):
        raise ValueError(
            f"local_body_pos must have shape [T,B,3], got {local_body_pos.shape}"
        )
    if (
        local_body_rot_xyzw.ndim != 3
        or local_body_rot_xyzw.shape[:2] != local_body_pos.shape[:2]
        or local_body_rot_xyzw.shape[2] != 4
    ):
        raise ValueError(
            f"local_body_rot must have shape [T,B,4] matching local_body_pos, got "
            f"{local_body_rot_xyzw.shape}"
        )

    local_body_quat_wxyz = normalize_quaternions_wxyz(
        quat_xyzw_to_wxyz_np(local_body_rot_xyzw), label="local_body_rot"
    )
    root_quat = root_quat_wxyz[:, None, :]
    body_pos_w = root_pos[:, None, :] + quat_apply_np(
        root_quat, np.asarray(local_body_pos, dtype=np.float32)
    )
    body_quat_w = quat_mul_np(root_quat, local_body_quat_wxyz)
    body_quat_w = normalize_quaternions_wxyz(body_quat_w, label="body_quat_w")
    return np.asarray(body_pos_w, dtype=np.float32), body_quat_w


def convert_motion_arrays(
    data: np.lib.npyio.NpzFile,
    *,
    source_path: Path,
) -> dict[str, np.ndarray]:
    fields = set(data.files)
    missing = sorted(ROBOT_REQUIRED_FIELDS.difference(fields))
    if missing:
        raise ValueError(
            f"source must use robot-motion fields {sorted(ROBOT_REQUIRED_FIELDS)}; "
            f"missing {missing}: {source_path}"
        )

    fps = read_fps(data, path=source_path)
    root_pos = np.asarray(data["root_pos"], dtype=np.float32)
    root_rot_xyzw = np.asarray(data["root_rot"], dtype=np.float32)
    dof_pos = np.asarray(data["dof_pos"], dtype=np.float32)
    source_joint_names = decode_names(data["joint_names"])
    _validate_source_arrays(
        source_path=source_path,
        root_pos=root_pos,
        root_rot_xyzw=root_rot_xyzw,
        dof_pos=dof_pos,
        joint_names=source_joint_names,
    )

    root_quat_wxyz = normalize_quaternions_wxyz(
        quat_xyzw_to_wxyz_np(root_rot_xyzw), label=f"root_rot in {source_path}"
    )
    root_quat_wxyz = make_quaternions_continuous_wxyz(root_quat_wxyz)
    joint_pos = reindex_joint_array(
        dof_pos,
        source_names=source_joint_names,
        target_names=ISAACLAB_G1_JOINT_NAMES,
        label="dof_pos",
    )

    has_local_pos = "local_body_pos" in fields
    has_local_rot = "local_body_rot" in fields
    if has_local_pos != has_local_rot:
        raise ValueError(
            f"local_body_pos and local_body_rot must either both be present or both be absent: "
            f"{source_path}"
        )
    if has_local_pos:
        local_body_pos = np.asarray(data["local_body_pos"], dtype=np.float32)
        local_body_rot = np.asarray(data["local_body_rot"], dtype=np.float32)
        body_order = _root_first_body_order(data, local_body_pos.shape[1])
        body_pos_w, body_quat_w = _world_bodies_from_local(
            root_pos=root_pos,
            root_quat_wxyz=root_quat_wxyz,
            local_body_pos=local_body_pos[:, body_order],
            local_body_rot_xyzw=local_body_rot[:, body_order],
        )
    else:
        kinematics = LocalKinematicsModel("g1")
        model_joint_pos = reindex_joint_array(
            dof_pos,
            source_names=source_joint_names,
            target_names=kinematics.joint_names,
            label="dof_pos",
        )
        body_pos_w, body_rot_xyzw = kinematics.forward_kinematics(
            root_pos,
            quat_wxyz_to_xyzw_np(root_quat_wxyz),
            model_joint_pos,
        )
        body_quat_w = normalize_quaternions_wxyz(
            quat_xyzw_to_wxyz_np(body_rot_xyzw), label="body_quat_w"
        )

    body_quat_w = make_quaternions_continuous_wxyz(body_quat_w)
    output = {
        "fps": np.asarray([fps], dtype=np.float32),
        "joint_pos": np.asarray(joint_pos, dtype=np.float32),
        "joint_vel": finite_difference(joint_pos, fps),
        "body_pos_w": np.asarray(body_pos_w, dtype=np.float32),
        "body_quat_w": np.asarray(body_quat_w, dtype=np.float32),
        "body_lin_vel_w": finite_difference(body_pos_w, fps),
        "body_ang_vel_w": angular_velocity_from_quaternions_wxyz(body_quat_w, fps),
    }
    if tuple(output) != ISAACLAB_EXPORT_FIELDS:
        raise AssertionError("converter output fields do not match the IsaacLab export contract")
    for name, values in output.items():
        if not np.isfinite(values).all():
            raise ValueError(f"converted {name} contains non-finite values: {source_path}")
    return output


def convert_motion_file(
    source_path: Path,
    destination_path: Path,
    *,
    overwrite: bool = False,
) -> tuple[int, int, float]:
    source = Path(source_path).expanduser().resolve()
    destination = Path(destination_path).expanduser().resolve()
    if source == destination:
        raise ValueError("source and destination paths must differ")
    if destination.exists() and not overwrite:
        raise FileExistsError(f"destination exists; pass --overwrite to replace it: {destination}")

    with np.load(source, allow_pickle=False) as data:
        output = convert_motion_arrays(data, source_path=source)

    destination.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.stem}.", suffix=".npz", dir=destination.parent
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(file_descriptor, "wb") as temporary_file:
            np.savez_compressed(temporary_file, **output)
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
        temporary_path.replace(destination)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise

    frame_count = int(output["joint_pos"].shape[0])
    body_count = int(output["body_pos_w"].shape[1])
    fps = float(output["fps"].reshape(()))
    return frame_count, body_count, fps


def discover_input_files(input_path: Path, output_root: Path) -> list[Path]:
    source = input_path.expanduser().resolve()
    destination_root = output_root.expanduser().resolve()
    if source.is_file():
        if source.suffix.lower() != ".npz":
            raise ValueError(f"input file must end in .npz: {source}")
        return [source]
    if not source.is_dir():
        raise FileNotFoundError(f"input path does not exist: {source}")

    files = []
    for path in sorted(source.rglob("*.npz")):
        resolved = path.resolve()
        try:
            resolved.relative_to(destination_root)
        except ValueError:
            files.append(resolved)
    if not files:
        raise RuntimeError(f"no .npz files found under {source}")
    return files


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input",
        type=Path,
        nargs="?",
        default=DEFAULT_INPUT_ROOT,
        help=f"Robot-motion NPZ file or directory (default: {DEFAULT_INPUT_ROOT})",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help=f"Converted output directory (default: {DEFAULT_OUTPUT_ROOT})",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace existing destination files. Existing files are preserved by default.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = args.input.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    input_files = discover_input_files(input_path, output_root)

    destinations: dict[Path, Path] = {}
    for source in input_files:
        destination = output_root / source.name
        previous = destinations.get(destination)
        if previous is not None:
            raise ValueError(
                f"flat output name collision: {previous} and {source} both map to {destination}"
            )
        destinations[destination] = source
    existing_destinations = [path for path in destinations if path.exists()]
    if existing_destinations and not args.overwrite:
        raise FileExistsError(
            f"{len(existing_destinations)} destination file(s) already exist; "
            f"pass --overwrite to replace them, first: {existing_destinations[0]}"
        )

    converted = 0
    for destination, source in destinations.items():
        frames, bodies, fps = convert_motion_file(
            source, destination, overwrite=bool(args.overwrite)
        )
        converted += 1
        print(
            f"[converted] {source} -> {destination} "
            f"(frames={frames}, bodies={bodies}, fps={fps:g})"
        )
    print(f"Converted {converted} motion file(s) into {output_root}")


if __name__ == "__main__":
    main()
