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
    UDPMotionSource,
    VRMotionSource,
    discover_motion_files,
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


class OnboardMotionSourceTests(unittest.TestCase):
    def test_local_motion_is_discovered_and_loaded_only_when_selected(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir) / "motion"
            path = root / "nested" / "local.npz"
            path.parent.mkdir(parents=True)
            frames = 2
            joint_pos = np.stack(
                [np.arange(29, dtype=np.float32) + 100.0 * frame for frame in range(frames)]
            )
            body_pos_w = np.zeros((frames, 1, 3), dtype=np.float32)
            body_pos_w[:, 0, 2] = 0.76
            body_quat_w = np.zeros((frames, 1, 4), dtype=np.float32)
            body_quat_w[..., 0] = 1.0
            np.savez(
                path,
                joint_pos=joint_pos,
                body_pos_w=body_pos_w,
                body_quat_w=body_quat_w,
                fps=np.asarray([50], dtype=np.int32),
            )

            self.assertEqual(discover_motion_files(root), {"nested/local": path.resolve()})

            config = DictToClass(
                {
                    "_config_dir": tmp_dir,
                    "motion_type": "isaaclab",
                    "root_body_index": 0,
                    "motion_source": {
                        "type": "udp",
                        "udp": {
                            "enable": False,
                            "host": "127.0.0.1",
                            "port": 28562,
                            "motion_root": str(root),
                        },
                    },
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
                current_name="default",
                current_done=True,
            )
            source = UDPMotionSource(policy, config)

            self.assertNotIn("nested/local", source.motions)
            captured = []

            def append_loaded(name):
                captured.append(source.motions[name]["joint_pos"].copy())
                return True

            source.append_motion_from_tail = append_loaded
            self.assertTrue(source.request_motion("nested/local"))
            self.assertNotIn("nested/local", source.motions)
            expected_indices = [
                ISAACLAB_G1_JOINT_NAMES.index(name) for name in MUJOCO_G1_JOINT_NAMES
            ]
            np.testing.assert_array_equal(captured[0][1], joint_pos[1, expected_indices])

    def test_queued_local_motion_waits_for_automatic_default_return(self):
        source = object.__new__(UDPMotionSource)
        source.motion_root = Path("/motion")
        source.policy = SimpleNamespace(current_name="first", current_done=True)
        source._pending_local_motion = "second"
        calls = []
        source.append_motion_from_tail = lambda name: calls.append(("append", name)) or True
        source.request_motion = lambda name: calls.append(("play", name)) or True

        source._advance_local_motion_queue()
        self.assertEqual(calls, [("append", "default")])
        self.assertEqual(source._pending_local_motion, "second")

        source.policy.current_name = "default"
        source.policy.current_done = True
        source._advance_local_motion_queue()
        self.assertEqual(calls[-1], ("play", "second"))
        self.assertIsNone(source._pending_local_motion)


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
    def test_new_host_motion_sequence_is_queued_once(self):
        source = object.__new__(VRMotionSource)
        source._last_motion_command_seq = -1
        source._pending_motion_command_seq = None
        source.remote_reference_source = ""
        source.remote_motion_name = ""
        source.remote_motion_finished = True

        payload = {
            "source": "motion",
            "state": "queued",
            "motion": "omni_extreme/omni_extreme_1",
            "motion_command_seq": 1,
        }
        self.assertTrue(source._record_motion_command(payload))
        self.assertEqual(source._pending_motion_command_seq, 1)
        self.assertEqual(source.remote_motion_name, payload["motion"])
        self.assertFalse(source.remote_motion_finished)
        self.assertFalse(source._record_motion_command(payload))

        next_payload = dict(payload, motion="omni_extreme/omni_extreme_2")
        next_payload["motion_command_seq"] = 2
        self.assertTrue(source._record_motion_command(next_payload))
        self.assertEqual(source._pending_motion_command_seq, 2)

    def test_queued_motion_starts_only_after_default_reference_finishes(self):
        source = object.__new__(VRMotionSource)
        source.policy = SimpleNamespace(current_done=False)
        source._pending_motion_command_seq = 7
        source._vr_active = False
        source._pending_start_request = False
        starts = []
        source.request_start = lambda: starts.append(True)

        self.assertFalse(source._start_queued_motion_if_ready())
        self.assertEqual(source._pending_motion_command_seq, 7)
        self.assertEqual(starts, [])

        source.policy.current_done = True
        self.assertTrue(source._start_queued_motion_if_ready())
        self.assertIsNone(source._pending_motion_command_seq)
        self.assertEqual(starts, [True])

    def test_finished_motion_appends_default_without_leaving_policy(self):
        source = object.__new__(VRMotionSource)
        source.remote_motion_name = "omni_extreme/omni_extreme_1"
        source.remote_motion_finished = False
        source._vr_user_enabled = True
        source._pending_start_request = False
        source._vr_active = True
        source._req_inflight = True
        source._req_inflight_steps_left = 1
        source._vr_in_transition = True
        source._vr_transition_count = 1
        source._shared_store = None
        source._pending_motion_command_seq = 8
        appended = []
        source.append_motion_from_tail = lambda name: appended.append(name) or True

        source._finish_remote_motion()

        self.assertTrue(source.remote_motion_finished)
        self.assertFalse(source._vr_user_enabled)
        self.assertFalse(source._vr_active)
        self.assertEqual(appended, ["default"])
        self.assertEqual(source._pending_motion_command_seq, 8)


if __name__ == "__main__":
    unittest.main()
