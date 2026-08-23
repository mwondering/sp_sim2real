from __future__ import annotations

import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

import numpy as np


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = SIM2REAL_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from runtime.policy import Policy
from runtime.recording import PolicyRunRecorder


class PolicyObservationRecordingTests(unittest.TestCase):
    def test_policy_observation_copy_returns_exact_onnx_input(self):
        policy = object.__new__(Policy)
        policy.name = "tracking"
        policy.input_key = "spv5_2_observation"
        policy.num_obs = 4
        policy.policy_input = {
            policy.input_key: np.asarray([[1.0, 2.0, 3.0, 4.0]], dtype=np.float32)
        }

        observation = policy.policy_observation_copy()

        np.testing.assert_array_equal(observation, [1.0, 2.0, 3.0, 4.0])
        policy.policy_input[policy.input_key][0, 0] = 99.0
        self.assertEqual(observation[0], 1.0)

    def test_recorder_saves_every_policy_observation_with_yaml_profile_metadata(self):
        policy = SimpleNamespace(
            name="tracking",
            actor_profile="spv5_2",
            policy_path="/tmp/0722/policy.onnx",
            input_key="spv5_2_observation",
            num_obs=4,
            action_joint_names=["j0", "j1"],
            config=SimpleNamespace(_config_path="/tmp/tracking_spv5_2.yaml"),
            last_action=np.asarray([0.1, 0.2], dtype=np.float32),
            applied_action=np.asarray([0.01, 0.02], dtype=np.float32),
            ref_idx=7,
            ref_len=20,
            current_name="walk",
        )
        controller = SimpleNamespace(
            policy_step=0,
            last_state_seq=10,
            last_state_receive_time_ns=123456,
            qj=np.asarray([1.0, 2.0], dtype=np.float32),
            dqj=np.asarray([3.0, 4.0], dtype=np.float32),
            tau=np.asarray([5.0, 6.0], dtype=np.float32),
            tau_latest=np.asarray([7.0, 8.0], dtype=np.float32),
            motor_ddq=np.asarray([0.5, 0.6], dtype=np.float32),
            motor_temperature_casing=np.asarray([41.0, 42.0], dtype=np.float32),
            motor_temperature_winding=np.asarray([51.0, 52.0], dtype=np.float32),
            motor_voltage=np.asarray([47.8, 47.9], dtype=np.float32),
            motor_mode=np.asarray([1, 1], dtype=np.uint32),
            motor_sensor_0=np.asarray([100, 101], dtype=np.uint32),
            motor_sensor_1=np.asarray([200, 201], dtype=np.uint32),
            motor_state=np.asarray([0, 0x12], dtype=np.uint32),
            motor_reserve_0=np.asarray([1, 2], dtype=np.uint32),
            motor_reserve_1=np.asarray([3, 4], dtype=np.uint32),
            motor_reserve_2=np.asarray([5, 6], dtype=np.uint32),
            motor_reserve_3=np.asarray([7, 8], dtype=np.uint32),
            motor_diagnostic_flags=np.asarray([0, 1], dtype=np.uint32),
            motor_diagnostic_flag_schema=1,
            diagnostic_warning_count=0,
            diagnostic_critical_count=1,
            diagnostic_imu_flags=0,
            mode_machine=5,
            cmd_q=np.asarray([9.0, 10.0], dtype=np.float32),
            cmd_qd=np.zeros(2, dtype=np.float32),
            cmd_kp=np.ones(2, dtype=np.float32),
            cmd_kd=np.ones(2, dtype=np.float32) * 2.0,
            cmd_enable=1,
            gyro=np.asarray([0.1, 0.2, 0.3], dtype=np.float32),
            linacc=np.asarray([0.4, 0.5, 0.6], dtype=np.float32),
            quat=np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        )

        with TemporaryDirectory() as tmp_dir:
            recorder = PolicyRunRecorder(
                robot="g1",
                policy=policy,
                joint_names=["j0", "j1"],
                control_dt=0.02,
                output_root=tmp_dir,
            )
            first_observation = np.asarray([1.0, 2.0, 3.0, 4.0], dtype=np.float32)
            recorder.record_step(
                controller,
                np.asarray([0.01, 0.02], dtype=np.float32),
                policy_observation=first_observation,
            )
            first_observation[:] = -1.0

            controller.policy_step = 1
            controller.last_state_seq = 11
            controller.tau_latest[:] = [9.0, 10.0]
            recorder.record_step(
                controller,
                np.asarray([0.03, 0.04], dtype=np.float32),
                policy_observation=np.asarray([5.0, 6.0, 7.0, 8.0], dtype=np.float32),
            )

            output_path = recorder.save()
            self.assertIsNotNone(output_path)
            with np.load(output_path, allow_pickle=False) as log:
                self.assertEqual(str(log["actor_profile"]), "spv5_2")
                self.assertEqual(
                    str(log["policy_config_path"]), "/tmp/tracking_spv5_2.yaml"
                )
                self.assertEqual(
                    str(log["policy_observation_key"]), "spv5_2_observation"
                )
                self.assertEqual(int(log["policy_observation_dim"]), 4)
                np.testing.assert_array_equal(log["policy_step"], [0, 1])
                np.testing.assert_array_equal(log["state_seq"], [10, 11])
                np.testing.assert_array_equal(
                    log["policy_observation"],
                    [[1.0, 2.0, 3.0, 4.0], [5.0, 6.0, 7.0, 8.0]],
                )
                np.testing.assert_array_equal(
                    log["joint_torque_latest"], [[7.0, 8.0], [9.0, 10.0]]
                )
                np.testing.assert_array_equal(
                    log["motor_temperature_casing"], [[41.0, 42.0], [41.0, 42.0]]
                )
                np.testing.assert_array_equal(log["motor_state"], [[0, 0x12], [0, 0x12]])
                np.testing.assert_array_equal(log["motor_diagnostic_flags"], [[0, 1], [0, 1]])
                np.testing.assert_array_equal(log["diagnostic_critical_count"], [1, 1])


if __name__ == "__main__":
    unittest.main()
