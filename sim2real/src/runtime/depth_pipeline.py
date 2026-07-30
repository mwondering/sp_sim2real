from __future__ import annotations

import numpy as np


def resize_bilinear_align_corners_false(
    image: np.ndarray, output_shape: tuple[int, int]
) -> np.ndarray:
    """NumPy equivalent of torch interpolate(..., align_corners=False)."""
    source = np.asarray(image, dtype=np.float32)
    if source.ndim != 2:
        raise ValueError(f"Expected a 2D image, got {source.shape}")
    in_h, in_w = source.shape
    out_h, out_w = (int(output_shape[0]), int(output_shape[1]))
    if out_h <= 0 or out_w <= 0:
        raise ValueError(f"Invalid output shape: {output_shape}")
    if (in_h, in_w) == (out_h, out_w):
        return source.copy()

    ys = (np.arange(out_h, dtype=np.float32) + 0.5) * in_h / out_h - 0.5
    xs = (np.arange(out_w, dtype=np.float32) + 0.5) * in_w / out_w - 0.5
    ys = np.clip(ys, 0.0, in_h - 1.0)
    xs = np.clip(xs, 0.0, in_w - 1.0)
    y0 = np.floor(ys).astype(np.int32)
    x0 = np.floor(xs).astype(np.int32)
    y1 = np.minimum(y0 + 1, in_h - 1)
    x1 = np.minimum(x0 + 1, in_w - 1)
    wy = (ys - y0)[:, None]
    wx = (xs - x0)[None, :]
    top = source[y0[:, None], x0[None, :]] * (1.0 - wx)
    top += source[y0[:, None], x1[None, :]] * wx
    bottom = source[y1[:, None], x0[None, :]] * (1.0 - wx)
    bottom += source[y1[:, None], x1[None, :]] * wx
    return (top * (1.0 - wy) + bottom * wy).astype(np.float32)


def _fill_left_edge_invalid(
    depth: np.ndarray, invalid: np.ndarray, max_columns: int
) -> tuple[np.ndarray, np.ndarray]:
    """Repair the short left-edge invalid runs commonly produced by D435i."""
    values = np.asarray(depth, dtype=np.float32).copy()
    mask = np.asarray(invalid, dtype=bool).copy()
    limit = min(max(int(max_columns), 0), values.shape[1])
    if limit == 0:
        return values, mask
    for row in range(values.shape[0]):
        valid_columns = np.flatnonzero(~mask[row, :limit])
        if valid_columns.size == 0:
            continue
        first_valid = int(valid_columns[0])
        if first_valid > 0:
            values[row, :first_valid] = values[row, first_valid]
            mask[row, :first_valid] = False
    return values, mask


class RealDepthProcessor:
    """Convert D435i metric depth to the MJLab lower-policy depth contract."""

    raw_policy_shape = (36, 64)
    output_shape = (1, 36, 64)

    def __init__(
        self,
        *,
        min_distance: float = 0.05,
        normalization_max_distance: float = 2.0,
        invalid_value: float = -1.0,
        valid_threshold: float = 0.5,
        crop_top: int = 10,
        crop_bottom: int = 0,
        crop_left: int = 10,
        crop_right: int = 10,
        edge_fill_left_columns: int = 24,
    ) -> None:
        self.min_distance = float(min_distance)
        self.normalization_max_distance = float(normalization_max_distance)
        self.invalid_value = float(invalid_value)
        self.valid_threshold = float(valid_threshold)
        self.crop = (
            int(crop_top),
            int(crop_bottom),
            int(crop_left),
            int(crop_right),
        )
        self.edge_fill_left_columns = int(edge_fill_left_columns)
        if self.min_distance < 0.0:
            raise ValueError("min_distance must be non-negative")
        if self.normalization_max_distance <= self.min_distance:
            raise ValueError(
                "normalization_max_distance must be greater than min_distance"
            )
        if not 0.0 <= self.valid_threshold <= 1.0:
            raise ValueError("valid_threshold must be in [0, 1]")
        top, bottom, left, right = self.crop
        if min(self.crop) < 0 or top + bottom >= 36 or left + right >= 64:
            raise ValueError(f"Invalid policy-frame crop: {self.crop}")

    def process(
        self, depth_m: np.ndarray
    ) -> tuple[np.ndarray, dict[str, float]]:
        depth = np.asarray(depth_m, dtype=np.float32)
        if depth.ndim != 2 or min(depth.shape) <= 1:
            raise ValueError(f"Expected raw depth [H,W], got {depth.shape}")

        invalid = (
            (~np.isfinite(depth))
            | (depth <= 0.0)
            | (depth < self.min_distance)
        )
        depth, invalid = _fill_left_edge_invalid(
            depth, invalid, self.edge_fill_left_columns
        )
        raw_invalid_fraction = float(np.mean(invalid))

        # Invalid samples are represented as far range while resizing, and the
        # separately resized validity mask restores the policy's -1 sentinel.
        safe = depth.copy()
        safe[invalid] = self.normalization_max_distance
        valid = (~invalid).astype(np.float32)
        safe = resize_bilinear_align_corners_false(
            safe, self.raw_policy_shape
        )
        valid = resize_bilinear_align_corners_false(
            valid, self.raw_policy_shape
        )
        normalized = (
            np.clip(safe, 0.0, self.normalization_max_distance)
            / self.normalization_max_distance
        ).astype(np.float32)
        normalized[valid < self.valid_threshold] = self.invalid_value

        top, bottom, left, right = self.crop
        row_end = normalized.shape[0] - bottom if bottom else normalized.shape[0]
        col_end = normalized.shape[1] - right if right else normalized.shape[1]
        cropped = normalized[top:row_end, left:col_end]
        cropped_invalid = (cropped == self.invalid_value).astype(np.float32)
        output = resize_bilinear_align_corners_false(
            cropped, self.raw_policy_shape
        )
        output_invalid = resize_bilinear_align_corners_false(
            cropped_invalid, self.raw_policy_shape
        )
        output[output_invalid >= self.valid_threshold] = self.invalid_value
        output = output[None].astype(np.float32)
        return output, {
            "raw_invalid_fraction": raw_invalid_fraction,
            "invalid_fraction": float(np.mean(output == self.invalid_value)),
            "normalized_min": float(np.min(output)),
            "normalized_max": float(np.max(output)),
        }


__all__ = [
    "RealDepthProcessor",
    "resize_bilinear_align_corners_false",
]
