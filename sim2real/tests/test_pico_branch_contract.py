from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = SIM2REAL_ROOT.parent
TELEOP_ROOT = SIM2REAL_ROOT / "teleop"
SRC_ROOT = SIM2REAL_ROOT / "src"
for path in (TELEOP_ROOT, SRC_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from paths import SUPPORTED_ROBOTS
from retarget.xrobot_retarget import XRobotRetargetWorkerRuntime
from serve_motion_reference import load_motion
from utils.robot_config import load_teleop_robot_config


def _tracked_and_new_files() -> list[str]:
    output = subprocess.check_output(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard"],
        cwd=REPO_ROOT,
        text=True,
    )
    return sorted(line for line in output.splitlines() if line)


class PicoBranchContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.files = _tracked_and_new_files()

    def test_contains_no_checkpoint_or_policy_artifact(self):
        model_suffixes = (".onnx", ".onnx.data", ".pt", ".pth", ".ckpt")
        offenders = []
        for value in self.files:
            path = Path(value)
            if "ckpt" in path.parts or "ckpts" in path.parts:
                offenders.append(value)
            elif value.endswith(model_suffixes):
                offenders.append(value)
        self.assertEqual(offenders, [])

    def test_contains_no_deploy_or_robot_control_runtime(self):
        self.assertFalse(any(path.startswith("g1_sim2real/") for path in self.files))
        source_files = [path for path in self.files if path.startswith("sim2real/src/")]
        self.assertEqual(source_files, ["sim2real/src/paths.py"])
        forbidden_names = {
            "deploy.py",
            "sim2sim.py",
            "motion_select.py",
            "tap_terrain.py",
            "teleop_upper_lower_locomani.py",
        }
        self.assertFalse(any(Path(path).name in forbidden_names for path in self.files))

    def test_preserves_all_motion_files(self):
        root_motion = [path for path in self.files if path.startswith("motion/")]
        g1_motion = [
            path for path in self.files if path.startswith("sim2real/config/g1/motions/")
        ]
        l7_motion = [
            path for path in self.files if path.startswith("sim2real/config/l7/motions/")
        ]
        self.assertEqual((len(root_motion), len(g1_motion), len(l7_motion)), (4, 17, 10))

    def test_only_g1_retarget_runtime_is_exposed(self):
        self.assertEqual(SUPPORTED_ROBOTS, ("g1",))
        config_files = [path for path in self.files if path.startswith("sim2real/config/")]
        non_motion = [path for path in config_files if "/motions/" not in path]
        allowed_prefixes = (
            "sim2real/config/g1/assets/",
            "sim2real/config/g1/retarget/",
        )
        self.assertTrue(non_motion)
        self.assertTrue(all(path.startswith(allowed_prefixes) for path in non_motion))

    def test_g1_motion_loads_as_36d_reference(self):
        config = load_teleop_robot_config("g1")
        motion_path = (
            SIM2REAL_ROOT
            / "config/g1/motions/omni_extreme/omni_extreme_1.npz"
        )
        qpos, fps = load_motion(motion_path, config.dof_names)
        self.assertEqual(config.dof_count, 29)
        self.assertEqual(qpos.shape, (588, 36))
        self.assertEqual(qpos.dtype, np.float32)
        self.assertTrue(np.isfinite(qpos).all())
        self.assertGreater(fps, 0.0)

    def test_g1_retarget_runtime_initializes_without_policy_runtime(self):
        config = load_teleop_robot_config("g1")
        runtime = XRobotRetargetWorkerRuntime(
            {
                "qpos_size": config.qpos_size,
                "target_robot": "g1",
                "actual_human_height": config.actual_human_height,
                "max_iter": config.max_iter,
                "send_human_motion": False,
                "enable_height_alignment": config.height_alignment_enabled,
                "height_alignment_xrobot_body_min_each_frame": (
                    config.height_alignment_xrobot_body_min_each_frame
                ),
                "height_alignment_foot_body_names": list(
                    config.height_alignment_foot_body_names
                ),
                "height_alignment_target_z": config.height_alignment_target_z,
                "height_bootstrap_frames": config.height_alignment_bootstrap_frames,
            }
        )
        self.assertEqual(runtime.qpos_size, 36)
        self.assertEqual(len(runtime.retarget.robot_joint_names), 29)


if __name__ == "__main__":
    unittest.main()
