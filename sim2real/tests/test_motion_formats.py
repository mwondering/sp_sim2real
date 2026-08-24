from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = SIM2REAL_ROOT.parent
TELEOP_ROOT = SIM2REAL_ROOT / "teleop"
if str(TELEOP_ROOT) not in sys.path:
    sys.path.insert(0, str(TELEOP_ROOT))

from convert_motion_to_isaaclab import convert_motion_file, finite_difference
from motion_formats import ISAACLAB_EXPORT_FIELDS, ISAACLAB_G1_JOINT_NAMES
from serve_motion_reference import MotionReferenceServer, load_motion
from utils.robot_config import load_teleop_robot_config


class IsaacLabMotionFormatTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_teleop_robot_config("g1")

    def test_player_loads_repository_isaaclab_motion(self):
        path = REPOSITORY_ROOT / "motion/flip_360_001__A304.npz"
        with np.load(path, allow_pickle=False) as data:
            source_joint_pos = np.asarray(data["joint_pos"], dtype=np.float32)
            expected_root_pos = np.asarray(data["body_pos_w"][:, 0], dtype=np.float32)
            expected_root_quat = np.asarray(data["body_quat_w"][:, 0], dtype=np.float32)
            expected_root_quat /= np.linalg.norm(expected_root_quat, axis=1, keepdims=True)
            expected_joint_indices = [
                ISAACLAB_G1_JOINT_NAMES.index(name) for name in self.config.dof_names
            ]

        qpos, fps = load_motion(path, self.config.dof_names)

        self.assertEqual(qpos.shape, (358, 36))
        self.assertEqual(qpos.dtype, np.float32)
        self.assertEqual(fps, 50.0)
        np.testing.assert_allclose(qpos[:, :3], expected_root_pos, atol=1.0e-6)
        np.testing.assert_allclose(qpos[:, 3:7], expected_root_quat, atol=1.0e-6)
        np.testing.assert_array_equal(
            qpos[:, 7:], source_joint_pos[:, expected_joint_indices]
        )

    def test_motion_server_selects_repository_isaaclab_motion(self):
        server = MotionReferenceServer(
            SimpleNamespace(
                config=SIM2REAL_ROOT / "config/g1/retarget/teleop.yaml",
                req_bind_addr=None,
                rep_bind_addr=None,
                ctrl_bind_addr=None,
                motion_root=REPOSITORY_ROOT / "motion",
                root_body_index=0,
                motion=None,
                loop=False,
                no_viewer=True,
                viewer_host="127.0.0.1",
                viewer_port=18080,
                select_bind_addr="tcp://127.0.0.1:28704",
            )
        )

        reply = server.handle_selection_request(
            {"command": "select", "motion": "flip_360_001__A304"}
        )

        self.assertTrue(reply["ok"])
        self.assertEqual(reply["state"], "queued")
        self.assertEqual(reply["frames"], 358)
        self.assertEqual(server.qpos.shape, (358, 36))

    def test_converter_preserves_reference_and_writes_target_fields(self):
        source_joint_names = tuple(self.config.dof_names)
        frame_count = 3
        root_pos = np.asarray(
            [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [3.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        half_sqrt = np.float32(np.sqrt(0.5))
        root_rot_xyzw = np.asarray(
            [
                [0.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, half_sqrt, half_sqrt],
                [0.0, 0.0, 1.0, 0.0],
            ],
            dtype=np.float32,
        )
        dof_pos = np.stack(
            [
                np.arange(len(source_joint_names), dtype=np.float32) + frame * 100.0
                for frame in range(frame_count)
            ]
        )
        local_body_pos = np.zeros((frame_count, 2, 3), dtype=np.float32)
        local_body_pos[:, 1, 0] = 1.0
        local_body_rot = np.zeros((frame_count, 2, 4), dtype=np.float32)
        local_body_rot[..., 3] = 1.0

        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_root = Path(temporary_directory)
            source = temporary_root / "source.npz"
            destination = temporary_root / "converted.npz"
            np.savez_compressed(
                source,
                fps=np.float32(50.0),
                root_pos=root_pos,
                root_rot=root_rot_xyzw,
                dof_pos=dof_pos,
                joint_names=np.asarray(source_joint_names),
                local_body_pos=local_body_pos,
                local_body_rot=local_body_rot,
                body_names=np.asarray(["pelvis", "test_link"]),
                root_link=np.asarray("pelvis"),
            )

            frames, bodies, fps = convert_motion_file(source, destination)
            self.assertEqual((frames, bodies, fps), (3, 2, 50.0))

            with np.load(destination, allow_pickle=False) as converted:
                self.assertEqual(tuple(converted.files), ISAACLAB_EXPORT_FIELDS)
                expected_indices = [
                    source_joint_names.index(name) for name in ISAACLAB_G1_JOINT_NAMES
                ]
                np.testing.assert_array_equal(
                    converted["joint_pos"], dof_pos[:, expected_indices]
                )
                np.testing.assert_array_equal(
                    converted["joint_vel"],
                    finite_difference(dof_pos[:, expected_indices], 50.0),
                )
                np.testing.assert_allclose(
                    converted["body_pos_w"][:, 0], root_pos, atol=1.0e-6
                )
                np.testing.assert_allclose(
                    converted["body_pos_w"][:, 1],
                    np.asarray(
                        [[1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [2.0, 0.0, 1.0]],
                        dtype=np.float32,
                    ),
                    atol=1.0e-5,
                )
                expected_root_wxyz = root_rot_xyzw[:, [3, 0, 1, 2]]
                np.testing.assert_allclose(
                    converted["body_quat_w"][:, 0], expected_root_wxyz, atol=1.0e-6
                )

            qpos, loaded_fps = load_motion(destination, source_joint_names)
            self.assertEqual(loaded_fps, 50.0)
            np.testing.assert_allclose(qpos[:, :3], root_pos, atol=1.0e-6)
            np.testing.assert_allclose(
                qpos[:, 3:7], root_rot_xyzw[:, [3, 0, 1, 2]], atol=1.0e-6
            )
            np.testing.assert_array_equal(qpos[:, 7:], dof_pos)

            with self.assertRaises(FileExistsError):
                convert_motion_file(source, destination)


if __name__ == "__main__":
    unittest.main()
