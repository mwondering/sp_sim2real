from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import yaml


SIM2REAL_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = SIM2REAL_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from depth_camera import DepthRayCamera, _resize_bilinear_align_corners_false
from runtime.depth_pipeline import RealDepthProcessor
from runtime.depth_overlay import DepthPointCloudOverlay
from runtime.dual_locomani import (
    ALL_JOINT_NAMES,
    DEFAULT_JOINT_POS,
    DualLocomaniRuntime,
    ObservationBuilder,
    PicoDualReferenceBuilder,
)
from runtime.shared_pico import PicoSnapshot


class TeleopUpperLowerLocomaniTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xml_path = (
            SIM2REAL_ROOT
            / "config/g1/assets/g1_teleop_upper_lower_locomani.xml"
        )

    def _snapshot(self, *, sticks=None, active=True, age=0.0):
        now = time.monotonic()
        return PicoSnapshot(
            seq=1,
            timestamp=now,
            pose_timestamp=now - age,
            control_timestamp=now - age,
            joint_names=ALL_JOINT_NAMES,
            joint_pos=DEFAULT_JOINT_POS.copy(),
            root_pos=np.array([0.0, 0.0, 0.76], dtype=np.float32),
            root_quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            buttons={},
            sticks={} if sticks is None else sticks,
            active=active,
        )

    def test_scene_contains_training_terrain_and_camera_mount(self):
        model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        site_id = model.site("depth_camera").id
        self.assertGreaterEqual(site_id, 0)
        np.testing.assert_allclose(
            model.site_pos[site_id],
            [0.12546158, 0.0197, -0.05952004],
            atol=1.0e-6,
        )
        np.testing.assert_allclose(
            model.site_quat[site_id],
            [0.8660254, 0.0, 0.5, 0.0],
            atol=1.0e-6,
        )
        self.assertGreaterEqual(model.geom("stairs_platform").id, 0)
        self.assertGreaterEqual(model.geom("slope_up").id, 0)

    def test_depth_camera_produces_policy_shape(self):
        model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        data = mujoco.MjData(model)
        data.qpos[:7] = [-1.0, 0.0, 0.76, 1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, data)
        depth = DepthRayCamera(model, data).capture()
        self.assertEqual(depth.shape, (1, 36, 64))
        self.assertEqual(depth.dtype, np.float32)
        self.assertTrue(np.isfinite(depth).all())

    def test_mujoco_depth_overlay_reconstructs_colored_point_samples(self):
        model = mujoco.MjModel.from_xml_path(str(self.xml_path))
        data = mujoco.MjData(model)
        data.qpos[:7] = [-1.0, 0.0, 0.76, 1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(model, data)
        overlay = object.__new__(DepthPointCloudOverlay)
        overlay.model = model
        overlay.data = data
        overlay.site_id = model.site("depth_camera").id
        overlay.stride = 4
        overlay.sample_rows = np.arange(0, 36, 4, dtype=np.int32)
        overlay.sample_cols = np.arange(0, 64, 4, dtype=np.int32)
        overlay.local_directions = (
            DepthPointCloudOverlay._output_pixel_directions()
        )
        points, values = overlay.world_points(
            np.full((1, 36, 64), 0.5, dtype=np.float32)
        )
        self.assertEqual(points.shape, (144, 3))
        np.testing.assert_allclose(values, 0.5)
        self.assertTrue(np.isfinite(points).all())

    def test_bilinear_resize_keeps_constant_image(self):
        image = np.full((26, 44), 0.37, dtype=np.float32)
        resized = _resize_bilinear_align_corners_false(image, (36, 64))
        np.testing.assert_allclose(resized, 0.37, atol=1.0e-6)

    def test_d435i_preprocess_matches_policy_shape_and_normalization(self):
        processor = RealDepthProcessor()
        processed, stats = processor.process(
            np.full((360, 640), 1.0, dtype=np.float32)
        )
        self.assertEqual(processed.shape, (1, 36, 64))
        self.assertEqual(processed.dtype, np.float32)
        np.testing.assert_allclose(processed, 0.5, atol=1.0e-6)
        self.assertEqual(stats["invalid_fraction"], 0.0)

    def test_d435i_preprocess_keeps_large_invalid_region_sentinel(self):
        processor = RealDepthProcessor(edge_fill_left_columns=0)
        raw = np.ones((360, 640), dtype=np.float32)
        raw[:, 200:440] = 0.0
        processed, stats = processor.process(raw)
        self.assertGreater(stats["invalid_fraction"], 0.2)
        self.assertTrue(np.any(processed == -1.0))

    def test_stairs_forces_forward_only_but_allows_neutral_stop(self):
        builder = PicoDualReferenceBuilder(
            kinematics_xml=self.xml_path,
            controller_joint_names=ALL_JOINT_NAMES,
            terrain_class=2,
        )
        moving = builder.build(
            self._snapshot(sticks={"ly": 0.2, "lx": 1.0, "rx": 1.0}),
            current_joint_pos=DEFAULT_JOINT_POS,
        )
        np.testing.assert_allclose(moving.twist, [0.3, 0.0, 0.0, 0.65])
        stopped = builder.build(
            self._snapshot(sticks={"ly": 0.0, "lx": 1.0, "rx": 1.0}),
            current_joint_pos=DEFAULT_JOINT_POS,
        )
        np.testing.assert_allclose(stopped.twist, [0.0, 0.0, 0.0, 0.65])
        reverse = builder.build(
            self._snapshot(sticks={"ly": -1.0}),
            current_joint_pos=DEFAULT_JOINT_POS,
        )
        np.testing.assert_allclose(reverse.twist, [0.0, 0.0, 0.0, 0.65])

    def test_stale_pico_zeroes_velocity_and_invalidates_reference(self):
        builder = PicoDualReferenceBuilder(
            kinematics_xml=self.xml_path,
            controller_joint_names=ALL_JOINT_NAMES,
            terrain_class=1,
            pose_timeout_s=0.25,
        )
        reference = builder.build(
            self._snapshot(sticks={"ly": 1.0}, age=1.0),
            current_joint_pos=DEFAULT_JOINT_POS,
        )
        self.assertFalse(reference.pico_valid)
        np.testing.assert_allclose(reference.twist[:3], 0.0)

    def test_copied_dual_onnx_pair_runs_with_exact_contract(self):
        controller_names = yaml.safe_load(
            (SIM2REAL_ROOT / "config/g1/controller.yaml").read_text()
        )["policy_joint_names"]
        runtime = DualLocomaniRuntime(
            lower_onnx=SIM2REAL_ROOT
            / "config/g1/ckpts/MJLab_Locomani/lower.onnx",
            upper_onnx=SIM2REAL_ROOT
            / "config/g1/ckpts/MJLab_Locomani/upper.onnx",
            kinematics_xml=self.xml_path,
            controller_joint_names=controller_names,
            terrain_class=1,
        )
        canonical_index = {name: i for i, name in enumerate(ALL_JOINT_NAMES)}
        controller = SimpleNamespace(
            qj=np.asarray(
                [DEFAULT_JOINT_POS[canonical_index[name]] for name in controller_names],
                dtype=np.float32,
            ),
            dqj=np.zeros(29, dtype=np.float32),
            quat=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
            gyro=np.zeros(3, dtype=np.float32),
        )
        command, reference = runtime.step(
            controller=controller,
            snapshot=self._snapshot(sticks={"ly": 0.5}),
            depth=np.full((1, 36, 64), 0.5, dtype=np.float32),
        )
        self.assertTrue(reference.pico_valid)
        self.assertEqual(command.q_des.shape, (29,))
        self.assertTrue(np.isfinite(command.q_des).all())
        # Inference alone must not claim that its candidate action was applied.
        np.testing.assert_allclose(runtime.observation.last_action, 0.0)
        runtime.commit_applied_target(command.q_des)
        self.assertFalse(np.allclose(runtime.observation.last_action, 0.0))

    def test_real_config_records_confirmed_camera_mount_and_separate_source(self):
        config = yaml.safe_load(
            (
                SIM2REAL_ROOT
                / "config/g1/teleop-upper-lower-locomani-real.yaml"
            ).read_text()
        )
        self.assertEqual(config["target"], "real")
        self.assertEqual(config["camera_process"]["source"], "d435i")
        hardware = config["camera_process"]["hardware"]
        self.assertEqual(hardware["mount_body"], "pelvis")
        self.assertAlmostEqual(float(hardware["pitch_down_deg"]), 60.0)

    def test_spv5_2_task_option_resolves_inside_current_repository(self):
        config_path = SIM2REAL_ROOT / "config/g1/tracking_spv5_2.yaml"
        config = yaml.safe_load(config_path.read_text())
        policy = (config_path.parent / config["policy_path"]).resolve()
        self.assertTrue(policy.is_file(), policy)
        self.assertTrue(policy.with_suffix(".json").is_file())

    def test_original_routes_remain_opt_in(self):
        original_bridge = yaml.safe_load(
            (SIM2REAL_ROOT / "config/g1/bridge.yaml").read_text()
        )
        original_tracking = yaml.safe_load(
            (SIM2REAL_ROOT / "config/g1/tracking.yaml").read_text()
        )
        self.assertNotIn("sim_state_stream", original_bridge)
        self.assertEqual(original_tracking["motion_source"]["type"], "udp")
        original_real_bridge = yaml.safe_load(
            (
                SIM2REAL_ROOT.parent / "g1_sim2real/config/g1_bridge.yaml"
            ).read_text()
        )
        self.assertNotIn("safety", original_real_bridge)


if __name__ == "__main__":
    unittest.main()
