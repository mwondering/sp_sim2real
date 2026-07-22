from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = SIM2REAL_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from sim2sim import (
    _compute_base_stabilization,
    _compute_pd_control,
    _normalize_default_pose_mode,
)


class Sim2SimStartupTests(unittest.TestCase):
    def test_default_pose_mode_aliases(self):
        self.assertEqual(_normalize_default_pose_mode("teleport"), "teleport")
        self.assertEqual(_normalize_default_pose_mode("sim2real-pd"), "sim2real_pd")
        self.assertEqual(_normalize_default_pose_mode("dynamic"), "sim2real_pd")
        with self.assertRaisesRegex(ValueError, "default_pose_mode"):
            _normalize_default_pose_mode("unknown")

    def test_pd_control_matches_real_bridge_command_semantics(self):
        q = np.asarray([0.2, -0.5])
        dq = np.asarray([0.1, -0.2])
        target = np.asarray([0.5, 0.5])
        kp = np.asarray([10.0, 20.0])
        kd = np.asarray([1.0, 2.0])
        control = _compute_pd_control(
            q,
            dq,
            target,
            kp,
            kd,
            lower=np.asarray([-2.0, -5.0]),
            upper=np.asarray([2.0, 5.0]),
        )
        np.testing.assert_allclose(control, [2.0, 5.0])

    def test_base_stabilizer_has_no_vertical_force_and_damps_rotation(self):
        wrench = _compute_base_stabilization(
            qpos=np.asarray([1.0, 2.0, 0.75, 1.0, 0.0, 0.0, 0.0]),
            qvel=np.asarray([0.5, -0.25, 3.0, 0.1, -0.2, 0.3]),
            target_qpos=np.asarray([2.0, 0.0, 0.80, 1.0, 0.0, 0.0, 0.0]),
            xy_kp=10.0,
            xy_kd=2.0,
            rotation_kp=20.0,
            rotation_kd=4.0,
        )
        np.testing.assert_allclose(wrench, [9.0, -19.5, 0.0, -0.4, 0.8, -1.2])


if __name__ == "__main__":
    unittest.main()
