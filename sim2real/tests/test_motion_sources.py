from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = SIM2REAL_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from common.utils import DictToClass
from runtime.motion_sources import (
    ISAACLAB_G1_JOINT_NAMES,
    MotionSourceBase,
    VRMotionSource,
)
from runtime.policy import _load_policy_metadata


MUJOCO_G1_JOINT_NAMES = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)


class MotionSourceFormatTests(unittest.TestCase):
    def test_isaaclab_sonic_npz_is_reordered_and_keeps_last_frame(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "sonic.npz"
            frames = 3
            joint_pos = np.stack(
                [np.arange(29, dtype=np.float32) + 100.0 * frame for frame in range(frames)]
            )
            body_pos_w = np.zeros((frames, 2, 3), dtype=np.float32)
            body_pos_w[:, 0, 0] = np.arange(frames, dtype=np.float32)
            body_pos_w[:, 0, 2] = 0.76
            body_quat_w = np.zeros((frames, 2, 4), dtype=np.float32)
            body_quat_w[..., 0] = 2.0  # loader normalizes wxyz quaternions
            np.savez(
                path,
                joint_pos=joint_pos,
                joint_vel=np.zeros_like(joint_pos),
                body_pos_w=body_pos_w,
                body_quat_w=body_quat_w,
                body_lin_vel_w=np.zeros_like(body_pos_w),
                body_ang_vel_w=np.zeros_like(body_pos_w),
                fps=np.asarray([50], dtype=np.int32),
            )

            config = DictToClass(
                {
                    "_config_dir": tmp_dir,
                    "motion_type": "isaaclab",
                    "root_body_index": 0,
                    "motions": [
                        {"name": "sonic", "path": str(path), "start": 0, "end": -1}
                    ],
                    "motion_clips": [
                        {
                            "name": "default",
                            "joint_pos": [0.0] * 29,
                            "root_quat": [1.0, 0.0, 0.0, 0.0],
                            "root_pos": [0.0, 0.0, 0.76],
                        }
                    ],
                }
            )
            policy = SimpleNamespace(
                dataset_joint_names=list(MUJOCO_G1_JOINT_NAMES),
                obs_joint_names=list(MUJOCO_G1_JOINT_NAMES),
            )

            source = MotionSourceBase(policy, config)
            loaded = source.motions["sonic"]

            self.assertEqual(loaded["joint_pos"].shape, (frames, 29))
            expected_indices = [
                ISAACLAB_G1_JOINT_NAMES.index(name) for name in MUJOCO_G1_JOINT_NAMES
            ]
            np.testing.assert_array_equal(
                loaded["joint_pos"][2], joint_pos[2, expected_indices]
            )
            np.testing.assert_array_equal(loaded["root_pos"], body_pos_w[:, 0])
            np.testing.assert_array_equal(
                loaded["root_quat"],
                np.tile(np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32), (frames, 1)),
            )

    def test_legacy_npz_remains_supported(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            path = Path(tmp_dir) / "legacy.npz"
            joint_pos = np.arange(58, dtype=np.float32).reshape(2, 29)
            np.savez(
                path,
                dof_pos=joint_pos,
                root_pos=np.asarray([[0.0, 0.0, 0.76], [0.1, 0.0, 0.76]], dtype=np.float32),
                root_rot=np.asarray([[0.0, 0.0, 0.0, 1.0]] * 2, dtype=np.float32),
                joint_names=np.asarray(MUJOCO_G1_JOINT_NAMES),
            )
            config = DictToClass(
                {
                    "_config_dir": tmp_dir,
                    "motion_type": "isaaclab",
                    "motions": [
                        {"name": "legacy", "path": str(path), "start": 0, "end": -1}
                    ],
                    "motion_clips": [
                        {
                            "name": "default",
                            "joint_pos": [0.0] * 29,
                            "root_quat": [1.0, 0.0, 0.0, 0.0],
                            "root_pos": [0.0, 0.0, 0.76],
                        }
                    ],
                }
            )
            policy = SimpleNamespace(
                dataset_joint_names=list(MUJOCO_G1_JOINT_NAMES),
                obs_joint_names=list(MUJOCO_G1_JOINT_NAMES),
            )

            loaded = MotionSourceBase(policy, config).motions["legacy"]
            np.testing.assert_array_equal(loaded["joint_pos"], joint_pos)
            self.assertEqual(loaded["joint_pos"].shape[0], 2)
            np.testing.assert_array_equal(loaded["root_quat"][0], [1.0, 0.0, 0.0, 0.0])


class PolicyMetadataFallbackTests(unittest.TestCase):
    def test_fallback_fills_missing_fields_without_overwriting_sidecar(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            policy_path = Path(tmp_dir) / "policy.onnx"
            policy_path.with_suffix(".json").write_text(
                json.dumps({"joint_names": ["sidecar_joint"], "in_keys": ["obs"]})
            )
            config = DictToClass(
                {
                    "policy_path": str(policy_path),
                    "_config_dir": tmp_dir,
                    "policy_metadata_fallback": {
                        "joint_names": ["fallback_joint"],
                        "action_scale": [0.5],
                    },
                }
            )

            metadata = _load_policy_metadata(config)

            self.assertEqual(metadata["joint_names"], ["sidecar_joint"])
            self.assertEqual(metadata["action_scale"], [0.5])


class RemoteMotionLifecycleTests(unittest.TestCase):
    def test_return_to_default_requires_finished_motion_and_drained_reference(self):
        source = object.__new__(VRMotionSource)
        source.remote_reference_source = "motion"
        source.remote_motion_finished = True
        source.policy = SimpleNamespace(current_done=True)
        self.assertTrue(source.can_return_to_default())

        source.policy.current_done = False
        self.assertFalse(source.can_return_to_default())
        source.policy.current_done = True
        source.remote_motion_finished = False
        self.assertFalse(source.can_return_to_default())
        source.remote_motion_finished = True
        source.remote_reference_source = "pico"
        self.assertFalse(source.can_return_to_default())

    def test_default_pose_notification_uses_reference_request_channel(self):
        class FakeRequestSocket:
            def __init__(self):
                self.messages = []

            def send_string(self, value, flags):
                self.messages.append((value, flags))

        source = object.__new__(VRMotionSource)
        source._req_sock = FakeRequestSocket()
        source.remote_motion_finished = True

        self.assertTrue(source.notify_default_pose())
        self.assertEqual(json.loads(source._req_sock.messages[0][0]), {"command": "default"})
        self.assertFalse(source.remote_motion_finished)


if __name__ == "__main__":
    unittest.main()
