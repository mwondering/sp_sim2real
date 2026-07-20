from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = SIM2REAL_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from common.udp_transport import _state_payload
from runtime.kinematics import quat_to_rot6d_wxyz
from runtime.observation import SPV51ActorObservation, WBTeleopActorObservation


class _FakePolicy:
    def __init__(self, config_name: str):
        config_path = SIM2REAL_ROOT / "config" / "g1" / config_name
        config = yaml.safe_load(config_path.read_text())
        config["_config_dir"] = str(config_path.parent)
        self.config = SimpleNamespace(**config)
        self.obs_joint_names = list(config["dataset_joint_names"])
        self.default_joint_pos_obs = np.asarray(
            config["motion_clips"][0]["joint_pos"], dtype=np.float32
        )
        self.last_action = np.linspace(-0.2, 0.2, 29, dtype=np.float32)
        self.controller = SimpleNamespace(
            quat=np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            gyro=np.asarray([0.1, -0.2, 0.3], dtype=np.float32),
        )
        self._joint_pos = self.default_joint_pos_obs + 0.01
        self._joint_vel = np.linspace(-0.5, 0.5, 29, dtype=np.float32)
        self._joint_torque = np.linspace(-2.0, 2.0, 29, dtype=np.float32)

        frame_count = 100
        phase = np.arange(frame_count, dtype=np.float32)[:, None]
        self.ref_joint_pos = self.default_joint_pos_obs[None, :] + phase * 1.0e-4
        self.ref_root_pos = np.zeros((frame_count, 3), dtype=np.float32)
        self.ref_root_pos[:, 2] = 0.76
        self.ref_root_pos[:, 0] = np.arange(frame_count, dtype=np.float32) * 0.002
        self.ref_root_quat = np.zeros((frame_count, 4), dtype=np.float32)
        self.ref_root_quat[:, 0] = 1.0
        self.ref_idx = 50
        self.ref_len = frame_count

    def current_joint_pos_obs(self):
        return self._joint_pos.copy()

    def current_joint_vel_obs(self):
        return self._joint_vel.copy()

    def current_joint_torque_obs(self):
        return self._joint_torque.copy()


class ActorProfileContractTests(unittest.TestCase):
    def test_rot6d_identity_layout(self):
        actual = quat_to_rot6d_wxyz(np.asarray([1.0, 0.0, 0.0, 0.0]))
        np.testing.assert_array_equal(actual, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0])

    def test_wbteleop_observation_contract(self):
        observation = WBTeleopActorObservation(_FakePolicy("tracking_wbteleop.yaml"))
        observation.update()
        value = observation.compute()
        self.assertEqual(value.shape, (886,))
        self.assertTrue(np.isfinite(value).all())

    def test_spv5_1_observation_contract(self):
        observation = SPV51ActorObservation(_FakePolicy("tracking_spv5_1.yaml"))
        observation.update()
        value = observation.compute()
        self.assertEqual(value.shape, (8199,))
        self.assertTrue(np.isfinite(value).all())
        # First history insertion is backfilled like MJLab's CircularBuffer.
        qpos_start = 4
        qpos_history = value[qpos_start : qpos_start + 50 * 29].reshape(50, 29)
        np.testing.assert_allclose(qpos_history[0], qpos_history[-1])

    def test_udp_state_payload_carries_torque(self):
        tau = np.arange(29, dtype=np.float32)
        payload = _state_payload(
            q=np.zeros(29),
            dq=np.zeros(29),
            quat_wxyz=np.asarray([1.0, 0.0, 0.0, 0.0]),
            gyro=np.zeros(3),
            linacc=np.zeros(3),
            tau=tau,
            buttons={},
            sticks={},
        )
        np.testing.assert_array_equal(payload["tau"], tau)


if __name__ == "__main__":
    unittest.main()
