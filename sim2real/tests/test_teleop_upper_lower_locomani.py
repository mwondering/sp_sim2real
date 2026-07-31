from __future__ import annotations

from dataclasses import replace
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
import runtime.zmq_stream as zmq_stream
from runtime.zmq_stream import ArrayPublisher, ArraySubscriber
from teleop_upper_lower_locomani import TeleopUpperLowerLocomaniController


class TeleopUpperLowerLocomaniTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xml_path = (
            SIM2REAL_ROOT
            / "config/g1/assets/g1_teleop_upper_lower_locomani.xml"
        )

    def _snapshot(self, *, buttons=None, sticks=None, active=True, age=0.0):
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
            buttons={} if buttons is None else buttons,
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
        processed, stats = processor.process_raw(
            np.full((360, 640), 1000, dtype=np.uint16),
            depth_scale=0.001,
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

    def test_zmq_array_protocol_preserves_raw_uint16_depth(self):
        class SendSocket:
            def __init__(self):
                self.parts = None

            def send_multipart(self, parts, **_kwargs):
                self.parts = [
                    bytes(parts[0]),
                    bytes(parts[1]),
                    bytes(parts[2]),
                ]

        raw = np.arange(12, dtype=np.uint16).reshape(3, 4)
        send_socket = SendSocket()
        publisher = object.__new__(ArrayPublisher)
        publisher.topic = b"depth"
        publisher.dtype = np.dtype(np.uint16)
        publisher.socket = send_socket
        self.assertTrue(
            publisher.send(raw, seq=7, sim_time=1.25)
        )

        class ReceiveSocket:
            def __init__(self, parts):
                self.parts = parts

            def recv_multipart(self, **_kwargs):
                if self.parts is None:
                    raise zmq_stream.zmq.Again()
                parts, self.parts = self.parts, None
                return [
                    parts[0],
                    parts[1],
                    SimpleNamespace(buffer=memoryview(parts[2])),
                ]

        subscriber = object.__new__(ArraySubscriber)
        subscriber.topic = b"depth"
        subscriber.socket = ReceiveSocket(send_socket.parts)
        subscriber._latest = None
        packet = subscriber.read_latest()
        self.assertEqual(packet.values.dtype, np.uint16)
        np.testing.assert_array_equal(packet.values, raw)

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

    def test_pico_left_x_requests_software_stop_only_when_control_is_fresh(self):
        controller = object.__new__(TeleopUpperLowerLocomaniController)
        controller.pico_stop_enabled = True
        controller.pico_stop_button = "left_key_one"
        controller.pico_stop_timeout_s = 0.25
        now = time.monotonic()
        fresh = self._snapshot(buttons={"left_key_one": True})
        self.assertTrue(
            controller._pico_software_stop_requested(fresh, now=now)
        )
        stale = replace(fresh, control_timestamp=now - 1.0)
        self.assertFalse(
            controller._pico_software_stop_requested(stale, now=now)
        )
        released = replace(fresh, buttons={"left_key_one": False})
        self.assertFalse(
            controller._pico_software_stop_requested(released, now=now)
        )

    def test_remote_depth_freshness_uses_server_receive_clock(self):
        controller = object.__new__(TeleopUpperLowerLocomaniController)
        controller.target = "real"
        controller.depth_timeout_s = 0.25
        controller.max_invalid_fraction = 0.6
        controller.depth_processor = RealDepthProcessor()
        controller.raw_depth_shape = (360, 640)
        now = time.monotonic()
        packet = SimpleNamespace(
            recv_time=now - 0.02,
            values=np.full((360, 640), 1000, dtype=np.uint16),
            metadata={
                # A remote monotonic timestamp has an unrelated epoch and must
                # not participate in the policy server's freshness decision.
                "capture_monotonic": now - 1_000_000.0,
                "protocol": "d435i-raw-z16-v1",
                "depth_scale": 0.001,
            },
        )
        controller.depth_sub = SimpleNamespace(read_latest=lambda: packet)
        np.testing.assert_allclose(controller._depth(), 0.5)

    def test_remote_depth_rejects_stale_or_future_receive_timestamp(self):
        controller = object.__new__(TeleopUpperLowerLocomaniController)
        controller.target = "real"
        controller.depth_timeout_s = 0.25
        controller.max_invalid_fraction = 0.6
        controller.depth_processor = RealDepthProcessor()
        controller.raw_depth_shape = (360, 640)
        now = time.monotonic()
        values = np.full((360, 640), 1000, dtype=np.uint16)
        stale = SimpleNamespace(
            recv_time=now - 1.0,
            values=values,
            metadata={
                "capture_monotonic": now,
                "protocol": "d435i-raw-z16-v1",
                "depth_scale": 0.001,
            },
        )
        controller.depth_sub = SimpleNamespace(read_latest=lambda: stale)
        self.assertIsNone(controller._depth())

        future = SimpleNamespace(
            recv_time=now + 1.0,
            values=values,
            metadata={
                "capture_monotonic": now,
                "protocol": "d435i-raw-z16-v1",
                "depth_scale": 0.001,
            },
        )
        controller.depth_sub = SimpleNamespace(read_latest=lambda: future)
        self.assertIsNone(controller._depth())

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
        self.assertEqual(config["switch"]["blend_duration_s"], 1.0)
        self.assertEqual(
            config["pico_software_stop"]["button"], "left_key_one"
        )
        hardware = config["camera_process"]["hardware"]
        self.assertEqual(hardware["mount_body"], "pelvis")
        self.assertAlmostEqual(float(hardware["pitch_down_deg"]), 60.0)

    def test_real_profiles_keep_safe_local_defaults(self):
        config_dir = SIM2REAL_ROOT / "config/g1"
        controller = yaml.safe_load(
            (config_dir / "controller.yaml").read_text()
        )
        self.assertEqual(controller["udp"]["state_bind_host"], "0.0.0.0")
        self.assertEqual(controller["udp"]["cmd_host"], "127.0.0.1")
        task = yaml.safe_load(
            (
                config_dir
                / "teleop-upper-lower-locomani-real.yaml"
            ).read_text()
        )
        self.assertEqual(task["target"], "real")
        self.assertEqual(task["camera_process"]["source"], "d435i")

        retarget = yaml.safe_load(
            (config_dir / "retarget/teleop-server.yaml").read_text()
        )
        for field in ("req_bind_addr", "rep_bind_addr", "ctrl_bind_addr"):
            self.assertTrue(
                retarget["server"][field].startswith("tcp://127.0.0.1:")
            )

        bridge = yaml.safe_load(
            (
                SIM2REAL_ROOT.parent
                / "g1_sim2real/config"
                / "g1_bridge_teleop_upper_lower_locomani.yaml"
            ).read_text()
        )
        self.assertTrue(bridge["safety"]["startup_damping"])
        self.assertEqual(bridge["safety"]["damping_publish_hz"], 50.0)

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
