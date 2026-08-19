"""Independent MuJoCo depth-camera process for the locomani task."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml

SRC_ROOT = Path(__file__).resolve().parent
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from runtime.depth_pipeline import (
    TAPTerrainDepthProcessor,
    resize_bilinear_align_corners_false as _resize_bilinear_align_corners_false,
)
from runtime.zmq_stream import ArrayPublisher, ArraySubscriber


class DepthRayCamera:
    """Training-compatible 64x36 grouped ray camera mounted by an XML site."""

    raw_height = 36
    raw_width = 64
    output_height = 36
    output_width = 64
    max_distance = 10.0
    min_distance = 0.05

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        site_name: str = "depth_camera",
        add_noise: bool = False,
        seed: int = 0,
        fov_y_deg: float = 57.9,
        depth_processor: TAPTerrainDepthProcessor | None = None,
    ) -> None:
        self.model = model
        self.data = data
        self.add_noise = bool(add_noise)
        self.rng = np.random.default_rng(seed)
        self.depth_processor = depth_processor
        self.site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, site_name
        )
        if self.site_id < 0:
            raise ValueError(f"Camera mount site {site_name!r} is absent from XML")
        self.camera_body_id = int(model.site_bodyid[self.site_id])

        self.fov_y_deg = float(fov_y_deg)
        if not 0.0 < self.fov_y_deg < 180.0:
            raise ValueError(f"Invalid vertical camera FOV: {self.fov_y_deg}")
        vertical_aperture = 2.0 * math.tan(
            math.radians(self.fov_y_deg) / 2.0
        )
        horizontal_aperture = vertical_aperture * self.raw_width / self.raw_height
        fx = self.raw_width / horizontal_aperture
        fy = self.raw_height / vertical_aperture
        cx = self.raw_width / 2.0
        cy = self.raw_height / 2.0
        pixel_u, pixel_v = np.meshgrid(
            np.arange(self.raw_width, dtype=np.float64),
            np.arange(self.raw_height, dtype=np.float64),
            indexing="xy",
        )
        image_x = (pixel_u + 0.5 - cx) / fx
        image_y = (pixel_v + 0.5 - cy) / fy
        local = np.stack(
            (np.ones_like(image_x), -image_x, -image_y), axis=-1
        ).reshape(-1, 3)
        self.local_directions = local / np.linalg.norm(
            local, axis=1, keepdims=True
        )
        self.geom_group = np.array([1, 0, 1, 0, 0, 0], dtype=np.uint8)
        self.geom_ids = np.empty(len(self.local_directions), dtype=np.int32)
        self.distances = np.empty(len(self.local_directions), dtype=np.float64)

    def capture(self) -> np.ndarray:
        origin = self.data.site_xpos[self.site_id].copy()
        world_rotation = self.data.site_xmat[self.site_id].reshape(3, 3)
        world_directions = np.ascontiguousarray(
            self.local_directions @ world_rotation.T, dtype=np.float64
        )
        # mj_multiRay's cutoff also culls geoms by their model-frame location.
        # A camera on a terrain lane far from the world origin can therefore
        # miss the floor even when the ray intersection itself is nearby.
        query_cutoff = self.max_distance + float(np.linalg.norm(origin))
        mujoco.mj_multiRay(
            self.model,
            self.data,
            origin,
            world_directions.reshape(-1),
            self.geom_group,
            True,
            self.camera_body_id,
            self.geom_ids,
            self.distances,
            None,
            len(world_directions),
            query_cutoff,
        )

        close_hits = np.flatnonzero(
            (self.distances >= 0.0) & (self.distances <= self.min_distance)
        )
        for index in close_hits:
            traveled = float(self.distances[index]) + 1.0e-4
            ray_origin = origin + world_directions[index] * traveled
            for _ in range(6):
                geom_id = np.empty(1, dtype=np.int32)
                extra = mujoco.mj_ray(
                    self.model,
                    self.data,
                    ray_origin,
                    world_directions[index],
                    self.geom_group,
                    True,
                    self.camera_body_id,
                    geom_id,
                )
                if extra < 0.0:
                    traveled = -1.0
                    break
                traveled += float(extra)
                if traveled > self.min_distance:
                    break
                ray_origin += world_directions[index] * (float(extra) + 1.0e-4)
                traveled += 1.0e-4
            self.distances[index] = traveled

        depth = self.distances * self.local_directions[:, 0]
        depth[self.distances < 0.0] = self.max_distance
        depth = np.clip(depth, 0.0, self.max_distance).reshape(
            self.raw_height, self.raw_width
        )
        if self.add_noise:
            sigma = 0.005 + 0.01 * depth
            depth += self.rng.normal(0.0, sigma, size=depth.shape)
        if self.depth_processor is not None:
            if self.add_noise:
                depth[self.rng.random(depth.shape) < 0.01] = 0.0
            return self.depth_processor.process(depth)[0]
        depth = np.clip(depth, 0.0, 2.0) / 2.0
        if self.add_noise:
            depth[self.rng.random(depth.shape) < 0.01] = -1.0
        cropped = depth[10:, 10:-10]
        return _resize_bilinear_align_corners_false(
            cropped, (self.output_height, self.output_width)
        )[None]


def _load_camera_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as file:
        config = yaml.safe_load(file)
    if not isinstance(config, dict):
        raise ValueError(f"Camera config must be a mapping: {path}")
    return config


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="Independent policy depth process")
    parser.add_argument(
        "--config",
        type=Path,
        default=SRC_ROOT.parent
        / "config/g1/teleop-upper-lower-locomani.yaml",
    )
    parser.add_argument(
        "--show-depth",
        action="store_true",
        help="Compatibility flag; depth is visualized in the main MuJoCo window",
    )
    parser.add_argument("--depth-noise", action="store_true")
    args = parser.parse_args(argv)

    config_path = args.config.expanduser().resolve()
    config = _load_camera_config(config_path)
    camera_cfg = config["camera_process"]
    xml_path = Path(config["scene_xml"])
    if not xml_path.is_absolute():
        xml_path = (config_path.parent / xml_path).resolve()

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    state_sub = ArraySubscriber(
        str(camera_cfg["sim_state_connect"]), topic="sim_state"
    )
    depth_pub = ArrayPublisher(str(camera_cfg["depth_bind"]), topic="depth")
    preprocess_cfg = camera_cfg.get("preprocess")
    depth_processor = None
    if isinstance(preprocess_cfg, dict) and preprocess_cfg.get("contract") == "tap_terrain_v1":
        depth_processor = TAPTerrainDepthProcessor.from_config(preprocess_cfg)
    hardware_cfg = camera_cfg.get("hardware", {})
    if not isinstance(hardware_cfg, dict):
        raise ValueError("camera_process.hardware must be a mapping")
    camera = DepthRayCamera(
        model,
        data,
        site_name=str(camera_cfg.get("site_name", "depth_camera")),
        add_noise=bool(args.depth_noise),
        seed=int(camera_cfg.get("seed", 0)),
        fov_y_deg=float(hardware_cfg.get("expected_fov_y_deg", 57.9)),
        depth_processor=depth_processor,
    )
    if args.show_depth:
        print(
            "[DepthCamera] --show-depth now uses the main MuJoCo window "
            "(depth_debug in the bridge config)"
        )
    period = 1.0 / float(camera_cfg.get("update_hz", 30.0))
    last_capture = float("-inf")
    last_seq = -1
    output_shape = (
        depth_processor.output_shape
        if depth_processor is not None
        else (1, camera.output_height, camera.output_width)
    )
    print(
        f"[DepthCamera] state<-{camera_cfg['sim_state_connect']} "
        f"depth->{camera_cfg['depth_bind']} xml={xml_path} "
        f"output_shape={output_shape}"
    )
    try:
        while True:
            packet = state_sub.read_latest()
            if packet is None or packet.seq == last_seq:
                time.sleep(0.001)
                continue
            if packet.sim_time < last_capture:
                last_capture = float("-inf")
            if packet.sim_time - last_capture + 1.0e-9 < period:
                time.sleep(0.001)
                continue
            nq = int(packet.metadata["nq"])
            nv = int(packet.metadata["nv"])
            if packet.values.shape != (nq + nv,):
                raise ValueError(
                    f"Sim-state shape {packet.values.shape} != {(nq + nv,)}"
                )
            if nq != model.nq or nv != model.nv:
                raise ValueError(
                    f"Camera XML model state is nq/nv={model.nq}/{model.nv}, "
                    f"sim publishes {nq}/{nv}"
                )
            data.qpos[:] = packet.values[:nq]
            data.qvel[:] = packet.values[nq:]
            data.time = packet.sim_time
            mujoco.mj_forward(model, data)
            depth = camera.capture()
            depth_pub.send(depth, seq=packet.seq, sim_time=packet.sim_time)
            last_seq = packet.seq
            last_capture = packet.sim_time
    except KeyboardInterrupt:
        pass
    finally:
        state_sub.close()
        depth_pub.close()


if __name__ == "__main__":
    main()
