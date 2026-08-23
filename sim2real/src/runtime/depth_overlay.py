from __future__ import annotations

import math

import mujoco
import numpy as np

from runtime.zmq_stream import ArraySubscriber


class DepthPointCloudOverlay:
    """Draw processed policy depth as a colored point cloud in MuJoCo."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        connect: str,
        site_name: str = "depth_camera",
        stride: int = 4,
        point_size: float = 0.012,
    ) -> None:
        self.model = model
        self.data = data
        self.subscriber = ArraySubscriber(connect, topic="depth")
        self.site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, str(site_name)
        )
        if self.site_id < 0:
            raise ValueError(f"Depth debug site {site_name!r} is absent from XML")
        self.stride = max(1, int(stride))
        self.point_size = float(point_size)
        self.local_directions = self._output_pixel_directions()
        self.sample_rows = np.arange(0, 36, self.stride, dtype=np.int32)
        self.sample_cols = np.arange(0, 64, self.stride, dtype=np.int32)
        print(
            f"[DepthOverlay] depth<-{connect}, points="
            f"{len(self.sample_rows) * len(self.sample_cols)}"
        )

    @staticmethod
    def _output_pixel_directions() -> np.ndarray:
        """Map resized policy pixels back into the raw camera ray grid."""
        out_y, out_x = np.meshgrid(
            np.arange(36, dtype=np.float64),
            np.arange(64, dtype=np.float64),
            indexing="ij",
        )
        # Inverse of crop [10:, 10:-10] followed by align_corners=False
        # resize from 26x44 to 36x64.
        raw_y = (out_y + 0.5) * 26.0 / 36.0 - 0.5 + 10.0
        raw_x = (out_x + 0.5) * 44.0 / 64.0 - 0.5 + 10.0
        vertical_aperture = 2.0 * math.tan(math.radians(57.9) / 2.0)
        horizontal_aperture = vertical_aperture * 64.0 / 36.0
        fx = 64.0 / horizontal_aperture
        fy = 36.0 / vertical_aperture
        image_x = (raw_x + 0.5 - 32.0) / fx
        image_y = (raw_y + 0.5 - 18.0) / fy
        directions = np.stack(
            (np.ones_like(image_x), -image_x, -image_y), axis=-1
        )
        return directions / np.linalg.norm(directions, axis=-1, keepdims=True)

    def world_points(self, depth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        image = np.asarray(depth, dtype=np.float32)
        if image.shape != (1, 36, 64):
            raise ValueError(f"Depth overlay expected (1,36,64), got {image.shape}")
        sampled_depth = image[0][np.ix_(self.sample_rows, self.sample_cols)]
        directions = self.local_directions[
            np.ix_(self.sample_rows, self.sample_cols)
        ]
        valid = np.isfinite(sampled_depth) & (sampled_depth >= 0.0)
        normalized = sampled_depth[valid]
        directions = directions[valid]
        forward_distance = normalized.astype(np.float64) * 2.0
        ray_distance = forward_distance / np.maximum(directions[:, 0], 1.0e-6)
        local_points = directions * ray_distance[:, None]
        origin = self.data.site_xpos[self.site_id]
        rotation = self.data.site_xmat[self.site_id].reshape(3, 3)
        world_points = origin + local_points @ rotation.T
        return world_points, normalized

    def update(self, viewer) -> None:
        packet = self.subscriber.read_latest()
        if packet is None:
            viewer.user_scn.ngeom = 0
            return
        points, normalized = self.world_points(packet.values)
        count = min(len(points), int(viewer.user_scn.maxgeom))
        identity = np.eye(3, dtype=np.float64).reshape(-1)
        size = np.full(3, self.point_size, dtype=np.float64)
        for index in range(count):
            value = float(np.clip(normalized[index], 0.0, 1.0))
            color = np.array(
                [value, 1.0 - abs(2.0 * value - 1.0), 1.0 - value, 0.9],
                dtype=np.float32,
            )
            mujoco.mjv_initGeom(
                viewer.user_scn.geoms[index],
                mujoco.mjtGeom.mjGEOM_SPHERE,
                size,
                points[index],
                identity,
                color,
            )
        viewer.user_scn.ngeom = count

    def close(self) -> None:
        self.subscriber.close()


__all__ = ["DepthPointCloudOverlay"]
