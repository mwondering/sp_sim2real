import unittest

import numpy as np

from runtime.motor_diagnostics import (
    IMU_ANGULAR_VELOCITY_LIMIT,
    JOINT_VELOCITY_LIMIT,
    MOTOR_STATE_FAULT,
    MotorDiagnosticReporter,
)


class MotorDiagnosticReporterTests(unittest.TestCase):
    def setUp(self):
        self.reporter = MotorDiagnosticReporter(["joint_0", "joint_1"])
        self.zeros_f32 = np.zeros(2, dtype=np.float32)
        self.zeros_u32 = np.zeros(2, dtype=np.uint32)

    def update(self, *, flags=None, motor_state=None, imu_flags=0):
        return self.reporter.update(
            flags=self.zeros_u32 if flags is None else flags,
            motor_state=self.zeros_u32 if motor_state is None else motor_state,
            casing_temperature=np.asarray([40.0, 45.0], dtype=np.float32),
            winding_temperature=np.asarray([50.0, 55.0], dtype=np.float32),
            q=self.zeros_f32,
            dq=np.asarray([0.0, 10.5], dtype=np.float32),
            tau=self.zeros_f32,
            imu_flags=imu_flags,
        )

    def test_reports_connection_once_for_clean_state(self):
        lines = self.update()
        self.assertEqual(len(lines), 1)
        self.assertIn("telemetry connected", lines[0])
        self.assertEqual(self.update(), [])

    def test_reports_fault_details_and_recovery(self):
        self.update()
        flags = np.asarray([MOTOR_STATE_FAULT, JOINT_VELOCITY_LIMIT], dtype=np.uint32)
        motor_state = np.asarray([0x12, 0], dtype=np.uint32)
        lines = self.update(
            flags=flags,
            motor_state=motor_state,
            imu_flags=IMU_ANGULAR_VELOCITY_LIMIT,
        )
        self.assertEqual(len(lines), 3)
        self.assertIn("joint=joint_0", lines[0])
        self.assertIn("motorstate=0x12", lines[0])
        self.assertIn("joint_velocity_limit", lines[1])
        self.assertIn("imu_angular_velocity_limit", lines[2])
        self.assertTrue(all("[CRITICAL]" in line for line in lines))

        self.assertEqual(self.update(), [
            "[Deploy][MotorDiag][RECOVERED] all reported conditions cleared"
        ])

    def test_rejects_wrong_shape(self):
        with self.assertRaisesRegex(ValueError, "must match policy_joint_names"):
            self.update(flags=np.zeros(1, dtype=np.uint32))


if __name__ == "__main__":
    unittest.main()
