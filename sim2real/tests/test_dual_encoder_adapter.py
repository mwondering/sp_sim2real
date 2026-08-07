from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = SIM2REAL_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from dual_teacher_adapter import (
    ACTION_SCALE,
    CANONICAL_JOINT_NAMES,
    ControlCommand,
    DEFAULT_JOINT_POS,
    DualEncoderAdapter,
    DualTeacherController,
    JOINT_KD,
    JOINT_KP,
    reference_joint_velocity,
)
from runtime.shared_pico import PicoSnapshot


class _Node:
    def __init__(self, name, shape):
        self.name = name
        self.shape = shape
        self.type = "tensor(float)"


class _FakeSession:
    def __init__(self, action=None):
        self.action = np.zeros((1, 29), dtype=np.float32)
        if action is not None:
            self.action[:] = np.asarray(action, dtype=np.float32).reshape(1, 29)
        self.run_count = 0
        self.last_inputs = None

    def get_inputs(self):
        return [
            _Node("encoder_motion", [1, 523]),
            _Node("encoder_velocity", [1, 468]),
            _Node("encoder_mask", [1, 2]),
            _Node("motion_prior", [1, 465]),
            _Node("encoder_depth", [1, 1, 36, 64]),
        ]

    def get_outputs(self):
        return [_Node("actions", [1, 29])]

    def run(self, output_names, inputs):
        if output_names != ["actions"]:
            raise AssertionError(output_names)
        self.run_count += 1
        self.last_inputs = {name: value.copy() for name, value in inputs.items()}
        return [self.action.copy()]


class DualEncoderAdapterTests(unittest.TestCase):
    def setUp(self):
        self.controller_names = tuple(reversed(CANONICAL_JOINT_NAMES))
        self.controller = SimpleNamespace(
            qj=DEFAULT_JOINT_POS[::-1].copy(),
            dqj=np.zeros(29, dtype=np.float32),
            quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            gyro=np.zeros(3, dtype=np.float32),
        )
        frames = 11
        time_axis = np.arange(frames, dtype=np.float32)[:, None]
        weights = np.arange(1, 30, dtype=np.float32)[None, :] * 1.0e-4
        canonical_reference = DEFAULT_JOINT_POS + time_axis**2 * weights
        self.reference_canonical = canonical_reference.astype(np.float32)
        self.reference_policy = SimpleNamespace(
            ref_joint_pos=self.reference_canonical[:, ::-1].copy(),
            ref_len=frames,
            ref_idx=5,
            obs_joint_names=self.controller_names,
        )

    @staticmethod
    def _snapshot(*, sticks=None, active=True, age=0.0, has_pose=True):
        now = time.monotonic()
        return PicoSnapshot(
            seq=1,
            timestamp=now,
            pose_timestamp=now - age,
            control_timestamp=now - age,
            joint_names=CANONICAL_JOINT_NAMES,
            joint_pos=DEFAULT_JOINT_POS.copy() if has_pose else None,
            root_pos=(
                np.array([0.0, 0.0, 0.78], dtype=np.float32)
                if has_pose
                else None
            ),
            root_quat=(
                np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
                if has_pose
                else None
            ),
            buttons={},
            sticks={} if sticks is None else sticks,
            active=active,
        )

    def _adapter(self, session=None, **kwargs):
        return DualEncoderAdapter(
            onnx_path="unused.onnx",
            controller_joint_names=self.controller_names,
            session=_FakeSession() if session is None else session,
            **kwargs,
        )

    def test_reference_velocity_matches_length_aware_heft_definition(self):
        values = self.reference_canonical
        fps = 50.0
        raw = np.zeros_like(values)
        raw[1:-1] = (values[2:] - values[:-2]) * (fps * 0.5)
        raw[0] = (values[1] - values[0]) * fps
        raw[-1] = (values[-1] - values[-2]) * fps
        for index in range(len(values)):
            velocity_indices = np.clip(
                np.arange(index - 2, index + 3), 0, len(values) - 1
            )
            expected = raw[velocity_indices].mean(axis=0)
            np.testing.assert_allclose(
                reference_joint_velocity(values, index, fps),
                expected,
                atol=1.0e-6,
                rtol=1.0e-6,
            )

    def test_motion_mode_builds_exact_mask_reference_and_zero_depth(self):
        session = _FakeSession()
        adapter = self._adapter(session)
        result = adapter.step(
            controller=self.controller,
            reference_policy=self.reference_policy,
            snapshot=self._snapshot(),
            depth=None,
            input_mode="motion",
        )
        self.assertTrue(result.ready, result.reason)
        self.assertIsNotNone(result.command)
        inputs = session.last_inputs
        self.assertEqual(
            {name: value.shape for name, value in inputs.items()},
            {
                name: tuple(shape)
                for name, shape in DualEncoderAdapter.EXPECTED_INPUTS.items()
            },
        )
        np.testing.assert_array_equal(inputs["encoder_mask"], [[1.0, 0.0]])
        np.testing.assert_array_equal(inputs["encoder_depth"], 0.0)
        np.testing.assert_array_equal(inputs["encoder_velocity"][0, :3], 0.0)
        np.testing.assert_allclose(
            inputs["encoder_motion"][0, :29],
            self.reference_canonical[self.reference_policy.ref_idx],
        )
        np.testing.assert_allclose(
            inputs["encoder_motion"][0, 29:58],
            reference_joint_velocity(
                self.reference_canonical,
                self.reference_policy.ref_idx,
                50.0,
            ),
        )
        np.testing.assert_allclose(
            result.command.q_des, DEFAULT_JOINT_POS[::-1]
        )

    def test_velocity_mode_masks_reference_and_uses_depth_and_sticks(self):
        session = _FakeSession()
        adapter = self._adapter(session, max_vx=1.0, max_vy=1.0, max_wz=0.5)
        depth = np.full((1, 36, 64), 0.5, dtype=np.float32)
        result = adapter.step(
            controller=self.controller,
            reference_policy=self.reference_policy,
            snapshot=self._snapshot(
                sticks={"ly": 0.5, "lx": -0.25, "rx": 0.4}
            ),
            depth=depth,
            input_mode="velocity",
        )
        self.assertTrue(result.ready, result.reason)
        inputs = session.last_inputs
        np.testing.assert_array_equal(inputs["encoder_mask"], [[0.0, 1.0]])
        np.testing.assert_allclose(inputs["encoder_motion"][0, :29], DEFAULT_JOINT_POS)
        np.testing.assert_array_equal(inputs["encoder_motion"][0, 29:58], 0.0)
        np.testing.assert_allclose(
            inputs["encoder_velocity"][0, :3], [0.5, -0.25, 0.2]
        )
        np.testing.assert_allclose(inputs["encoder_depth"], 0.5)

    def test_velocity_mode_does_not_require_a_pico_pose(self):
        adapter = self._adapter()
        result = adapter.step(
            controller=self.controller,
            reference_policy=self.reference_policy,
            snapshot=self._snapshot(has_pose=False),
            depth=np.full((1, 36, 64), 0.5, dtype=np.float32),
            input_mode="velocity",
        )
        self.assertTrue(result.ready, result.reason)

    def test_actual_applied_target_enters_action_history_on_next_step(self):
        session = _FakeSession()
        adapter = self._adapter(session)
        first = adapter.step(
            controller=self.controller,
            reference_policy=self.reference_policy,
            snapshot=self._snapshot(),
            depth=None,
            input_mode="motion",
        )
        self.assertTrue(first.ready, first.reason)

        applied_canonical = DEFAULT_JOINT_POS + ACTION_SCALE
        adapter.commit_applied_target(applied_canonical[::-1])
        second = adapter.step(
            controller=self.controller,
            reference_policy=self.reference_policy,
            snapshot=self._snapshot(),
            depth=None,
            input_mode="motion",
        )
        self.assertTrue(second.ready, second.reason)
        action_history = session.last_inputs["motion_prior"][0, 320:].reshape(5, 29)
        np.testing.assert_allclose(action_history[:-1], 0.0, atol=1.0e-6)
        np.testing.assert_allclose(action_history[-1], 1.0, atol=1.0e-6)

    def test_motion_mode_waits_for_three_future_reference_frames(self):
        session = _FakeSession()
        adapter = self._adapter(session)
        self.reference_policy.ref_idx = self.reference_policy.ref_len - 3
        result = adapter.step(
            controller=self.controller,
            reference_policy=self.reference_policy,
            snapshot=self._snapshot(),
            depth=None,
            input_mode="motion",
        )
        self.assertFalse(result.ready)
        self.assertEqual(result.reason, "reference_future_horizon")
        self.assertEqual(session.run_count, 0)

    def test_controller_and_tracking_configs_match_checkpoint_contract(self):
        config_dir = SIM2REAL_ROOT / "config/g1"
        controller = yaml.safe_load((config_dir / "controller.yaml").read_text())
        names = tuple(controller["policy_joint_names"])
        canonical_from_controller = np.asarray(
            [names.index(name) for name in CANONICAL_JOINT_NAMES]
        )
        np.testing.assert_allclose(
            np.asarray(controller["default_qpos"])[canonical_from_controller],
            DEFAULT_JOINT_POS,
        )
        np.testing.assert_allclose(
            np.asarray(controller["kps"])[canonical_from_controller], JOINT_KP
        )
        np.testing.assert_allclose(
            np.asarray(controller["kds"])[canonical_from_controller], JOINT_KD
        )

        tracking = yaml.safe_load((config_dir / "tracking.yaml").read_text())
        action_names = tuple(tracking["action_joint_names"])
        canonical_from_action = np.asarray(
            [action_names.index(name) for name in CANONICAL_JOINT_NAMES]
        )
        np.testing.assert_allclose(
            np.asarray(tracking["action_scale"])[canonical_from_action],
            ACTION_SCALE,
        )

    def test_existing_upper_lower_task_configs_are_unchanged(self):
        config_dir = SIM2REAL_ROOT / "config/g1"
        for filename in (
            "teleop-upper-lower-locomani.yaml",
            "teleop-upper-lower-locomani-real.yaml",
        ):
            path = config_dir / filename
            task = yaml.safe_load(path.read_text())
            self.assertEqual(task["task_name"], "teleop-upper-lower-locomani")
            self.assertIn("lower_onnx", task)
            self.assertIn("upper_onnx", task)
            self.assertNotIn("dual_onnx", task)

    def test_dual_teacher_task_configs_resolve_checkpoint_and_contract(self):
        config_dir = SIM2REAL_ROOT / "config/g1"
        for filename, target in (
            ("dual-teacher.yaml", "sim"),
            ("dual-teacher-real.yaml", "real"),
        ):
            path = config_dir / filename
            task = yaml.safe_load(path.read_text())
            self.assertEqual(task["task_name"], "dual-teacher")
            self.assertEqual(task["target"], target)
            self.assertEqual(task["input_mode"], "motion")
            self.assertEqual(task["tracking_config"], "tracking.yaml")
            self.assertEqual(float(task["reference_fps"]), 50.0)
            self.assertEqual(float(task["handoff"]["blend_duration_s"]), 1.0)
            self.assertEqual(float(task["handoff"]["ready_hold_s"]), 0.1)
            model = (path.parent / task["dual_onnx"]).resolve()
            self.assertTrue(model.is_file(), model)

    def test_real_checkpoint_runs_with_adapter_contract_when_available(self):
        config_path = SIM2REAL_ROOT / "config/g1/dual-teacher.yaml"
        task = yaml.safe_load(config_path.read_text())
        model = (config_path.parent / task["dual_onnx"]).resolve()
        if not model.is_file():
            self.skipTest(f"External deployment checkpoint is absent: {model}")
        adapter = DualEncoderAdapter(
            onnx_path=model,
            controller_joint_names=self.controller_names,
        )
        result = adapter.step(
            controller=self.controller,
            reference_policy=self.reference_policy,
            snapshot=self._snapshot(),
            depth=None,
            input_mode="motion",
        )
        self.assertTrue(result.ready, result.reason)
        self.assertEqual(result.command.q_des.shape, (29,))
        self.assertTrue(np.isfinite(result.command.q_des).all())

        velocity_result = adapter.step(
            controller=self.controller,
            reference_policy=self.reference_policy,
            snapshot=self._snapshot(
                sticks={"ly": 0.25, "lx": 0.0, "rx": 0.0},
                has_pose=False,
            ),
            depth=np.full((1, 36, 64), 0.5, dtype=np.float32),
            input_mode="velocity",
        )
        self.assertTrue(velocity_result.ready, velocity_result.reason)
        self.assertEqual(velocity_result.command.q_des.shape, (29,))
        self.assertTrue(np.isfinite(velocity_result.command.q_des).all())


class DualTeacherHandoffTests(unittest.TestCase):
    @staticmethod
    def _controller() -> DualTeacherController:
        controller = DualTeacherController.__new__(DualTeacherController)
        controller.policy_joint_names = list(CANONICAL_JOINT_NAMES)
        controller.default_qpos = DEFAULT_JOINT_POS.copy()
        controller.kps = JOINT_KP.copy()
        controller.kds = JOINT_KD.copy()
        controller.cmd_q = DEFAULT_JOINT_POS.copy()
        controller.cmd_qd = np.zeros(29, dtype=np.float32)
        controller.cmd_kp = JOINT_KP.copy()
        controller.cmd_kd = JOINT_KD.copy()
        controller.cmd_enable = 0
        controller.blend_steps = 2
        controller.ready_steps = 3
        controller._dual_ready_steps = 0
        controller._control_source = "tracking"
        controller._blend_step = controller.blend_steps
        controller._blend_start = ControlCommand(
            q_des=controller.cmd_q.copy(),
            kp=controller.cmd_kp.copy(),
            kd=controller.cmd_kd.copy(),
        )
        return controller

    def test_tracking_fallback_builds_a_balancing_policy_command(self):
        controller = self._controller()
        delta = np.linspace(-0.2, 0.2, 29, dtype=np.float32)
        reference = SimpleNamespace(compute_action=lambda: delta.copy())

        command = controller._tracking_command(reference)

        np.testing.assert_allclose(command.q_des, DEFAULT_JOINT_POS + delta)
        np.testing.assert_array_equal(command.kp, JOINT_KP)
        np.testing.assert_array_equal(command.kd, JOINT_KD)

    def test_dual_requires_consecutive_ready_steps_and_falls_back_immediately(self):
        controller = self._controller()

        controller._update_control_source(dual_ready=True)
        controller._update_control_source(dual_ready=True)
        self.assertEqual(controller._control_source, "tracking")

        controller._update_control_source(dual_ready=True)
        self.assertEqual(controller._control_source, "dual")
        self.assertEqual(controller._blend_step, 0)

        controller._update_control_source(dual_ready=False)
        self.assertEqual(controller._control_source, "tracking")
        self.assertEqual(controller._dual_ready_steps, 0)
        self.assertEqual(controller._blend_step, 0)

    def test_source_switch_blends_from_the_command_currently_applied(self):
        controller = self._controller()
        controller._blend_step = 0
        target = ControlCommand(
            q_des=DEFAULT_JOINT_POS + 0.2,
            kp=JOINT_KP * 0.5,
            kd=JOINT_KD * 0.5,
        )

        first = controller._apply_command(target)
        np.testing.assert_allclose(first.q_des, DEFAULT_JOINT_POS + 0.1)
        np.testing.assert_allclose(first.kp, JOINT_KP * 0.75)
        self.assertEqual(controller.cmd_enable, 1)

        second = controller._apply_command(target)
        np.testing.assert_allclose(second.q_des, target.q_des)
        np.testing.assert_allclose(second.kp, target.kp)

    def test_actual_blended_command_is_committed_to_both_action_histories(self):
        controller = self._controller()

        class _Adapter:
            committed = None

            def commit_applied_target(self, q_des):
                self.committed = np.asarray(q_des).copy()

        class _IdentityMapper:
            @staticmethod
            def map_state_to_from(values):
                return np.asarray(values).copy()

        controller.adapter = _Adapter()
        reference = SimpleNamespace(
            controller_default_qpos=None,
            mapper_action=_IdentityMapper(),
            action_scale=ACTION_SCALE.copy(),
            action_clip=10.0,
            last_action=np.zeros(29, dtype=np.float32),
            applied_action=np.zeros(29, dtype=np.float32),
        )
        q_des = DEFAULT_JOINT_POS + ACTION_SCALE * 0.25
        command = ControlCommand(q_des=q_des, kp=JOINT_KP, kd=JOINT_KD)

        controller._commit_applied_command(command, reference)

        np.testing.assert_array_equal(controller.adapter.committed, q_des)
        np.testing.assert_allclose(reference.last_action, 0.25, atol=1.0e-6)
        np.testing.assert_allclose(
            reference.applied_action, ACTION_SCALE * 0.25, atol=1.0e-6
        )


if __name__ == "__main__":
    unittest.main()
