from __future__ import annotations

from dataclasses import dataclass
from io import BytesIO
import json
import math
import os
from pathlib import Path
import select
import struct
import subprocess
import sys
import time

import numpy as np


@dataclass(frozen=True)
class D435iFrame:
    depth_m: np.ndarray
    capture_monotonic: float


class D435iSource:
    """D435i depth source with an optional pyrealsense2 worker environment."""

    def __init__(
        self,
        *,
        width: int,
        height: int,
        fps: int,
        serial_number: str = "",
        timeout_s: float = 0.5,
        worker_python: str = "",
        expected_fov_x_deg: float = 89.04,
        expected_fov_y_deg: float = 57.9,
        fov_tolerance_deg: float = 6.0,
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.serial_number = str(serial_number).strip()
        self.timeout_s = float(timeout_s)
        self.worker_python = str(worker_python).strip()
        self.expected_fov = (
            float(expected_fov_x_deg),
            float(expected_fov_y_deg),
        )
        self.fov_tolerance_deg = float(fov_tolerance_deg)
        self.pipeline = None
        self.profile = None
        self.worker = None
        self.depth_scale = 0.001
        self.fov_deg: tuple[float, float] | None = None
        self._worker_device_serial: str | None = None
        self._worker_device_name: str | None = None
        self._worker_buffer = bytearray()
        try:
            import pyrealsense2 as rs
        except (ImportError, ModuleNotFoundError) as exc:
            self.rs = None
            self.import_error = exc
            self.mode = "worker"
            self._start_worker()
        else:
            self.rs = rs
            self.import_error = None
            self.mode = "direct"
            self._start_direct()

    def _start_direct(self) -> None:
        self.pipeline = self.rs.pipeline()
        config = self.rs.config()
        if self.serial_number:
            config.enable_device(self.serial_number)
        config.enable_stream(
            self.rs.stream.depth,
            self.width,
            self.height,
            self.rs.format.z16,
            self.fps,
        )
        try:
            self.profile = self.pipeline.start(config)
        except Exception as exc:
            raise RuntimeError(
                "无法启动 D435i 深度流 "
                f"{self.width}x{self.height}@{self.fps}"
                + (
                    f"，serial={self.serial_number}"
                    if self.serial_number
                    else ""
                )
            ) from exc

        device = self.profile.get_device()
        device_name = str(device.get_info(self.rs.camera_info.name))
        self._validate_device_name(device_name)
        sensor = device.first_depth_sensor()
        self.depth_scale = float(sensor.get_depth_scale())
        video = self.profile.get_stream(
            self.rs.stream.depth
        ).as_video_stream_profile()
        intrinsics = video.get_intrinsics()
        fov_x = math.degrees(
            2.0 * math.atan(intrinsics.width / (2.0 * intrinsics.fx))
        )
        fov_y = math.degrees(
            2.0 * math.atan(intrinsics.height / (2.0 * intrinsics.fy))
        )
        self.fov_deg = (fov_x, fov_y)
        self._validate_fov()
        print(
            "[D435i] direct "
            f"serial={self.device_serial} stream={self.width}x{self.height}@{self.fps} "
            f"scale={self.depth_scale:.6g} fov={fov_x:.2f}x{fov_y:.2f}deg"
        )

    @property
    def device_serial(self) -> str:
        if self.profile is None:
            return (
                self._worker_device_serial
                or self.serial_number
                or "worker-unknown"
            )
        try:
            return str(
                self.profile.get_device().get_info(
                    self.rs.camera_info.serial_number
                )
            )
        except Exception:
            return self.serial_number or "unknown"

    def _validate_fov(self) -> None:
        if self.fov_deg is None:
            return
        errors = [
            abs(actual - expected)
            for actual, expected in zip(self.fov_deg, self.expected_fov)
        ]
        if max(errors) > self.fov_tolerance_deg:
            raise RuntimeError(
                "D435i 内参视场角与训练配置差异过大："
                f"actual={self.fov_deg[0]:.2f}x{self.fov_deg[1]:.2f}deg, "
                f"expected={self.expected_fov[0]:.2f}x{self.expected_fov[1]:.2f}deg"
            )

    @staticmethod
    def _validate_device_name(device_name: str) -> None:
        if "d435i" not in str(device_name).strip().lower():
            raise RuntimeError(
                f"Expected an Intel RealSense D435i, got {device_name!r}"
            )

    def _resolve_worker_python(self) -> str:
        candidates = [self.worker_python] if self.worker_python else []
        candidates.extend(
            [
                "/home/unitree/miniconda3/envs/realsense/bin/python",
                "/home/unitree/miniconda3/envs/locodist/bin/python",
            ]
        )
        failures = []
        for candidate in candidates:
            path = Path(candidate).expanduser()
            if not path.is_file() or not os.access(path, os.X_OK):
                continue
            try:
                probe = subprocess.run(
                    [str(path), "-c", "import pyrealsense2"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=10.0,
                    check=False,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                failures.append(f"{path}: {exc}")
                continue
            if probe.returncode == 0:
                return str(path)
            failures.append(f"{path}: import pyrealsense2 failed")
        detail = "; ".join(failures)
        raise RuntimeError(
            "当前 Python 不能导入 pyrealsense2，且未找到可用 worker_python。"
            "请安装 pyrealsense2，或在 real task 配置中设置 camera_process."
            f"hardware.worker_python。原始错误：{self.import_error}。{detail}"
        )

    def _start_worker(self) -> None:
        worker_python = self._resolve_worker_python()
        command = [
            worker_python,
            str(Path(__file__).with_name("d435i_worker.py")),
            "--width",
            str(self.width),
            "--height",
            str(self.height),
            "--fps",
            str(self.fps),
            "--timeout-s",
            str(self.timeout_s),
        ]
        if self.serial_number:
            command.extend(["--serial-number", self.serial_number])
        self.worker = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        try:
            payload = self._read_worker_payload(
                timeout_s=max(10.0, 2.0 * self.timeout_s),
                allow_header_timeout=False,
            )
            if not payload:
                raise RuntimeError("D435i worker did not return startup metadata")
            try:
                metadata = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise RuntimeError(
                    "D435i worker returned invalid startup metadata"
                ) from exc
            self._apply_worker_metadata(metadata)
        except Exception:
            if self.worker.poll() is None:
                self.worker.terminate()
                try:
                    self.worker.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.worker.kill()
                    self.worker.wait(timeout=2.0)
            self.worker = None
            raise
        print(
            f"[D435i] worker python={worker_python} "
            f"device={self._worker_device_name} serial={self.device_serial} "
            f"stream={self.width}x{self.height}@{self.fps} "
            f"scale={self.depth_scale:.6g} "
            f"fov={self.fov_deg[0]:.2f}x{self.fov_deg[1]:.2f}deg"
        )

    def _apply_worker_metadata(self, metadata: object) -> None:
        if not isinstance(metadata, dict):
            raise RuntimeError("D435i worker startup metadata must be an object")
        if int(metadata.get("protocol_version", -1)) != 1:
            raise RuntimeError("Unsupported D435i worker protocol version")
        actual_profile = (
            int(metadata.get("width", -1)),
            int(metadata.get("height", -1)),
            int(metadata.get("fps", -1)),
        )
        expected_profile = (self.width, self.height, self.fps)
        if actual_profile != expected_profile:
            raise RuntimeError(
                "D435i worker stream profile mismatch: "
                f"actual={actual_profile}, expected={expected_profile}"
            )
        depth_scale = float(metadata.get("depth_scale", float("nan")))
        fov = (
            float(metadata.get("fov_x_deg", float("nan"))),
            float(metadata.get("fov_y_deg", float("nan"))),
        )
        if (
            not math.isfinite(depth_scale)
            or depth_scale <= 0.0
            or not all(math.isfinite(value) and value > 0.0 for value in fov)
        ):
            raise RuntimeError("D435i worker returned invalid scale or FOV")
        actual_serial = str(metadata.get("serial_number", "")).strip()
        if not actual_serial:
            raise RuntimeError("D435i worker did not report a device serial")
        if self.serial_number and actual_serial != self.serial_number:
            raise RuntimeError(
                "D435i worker selected an unexpected device: "
                f"actual={actual_serial}, expected={self.serial_number}"
            )
        self.depth_scale = depth_scale
        self.fov_deg = fov
        self._worker_device_serial = actual_serial
        self._worker_device_name = str(
            metadata.get("device_name", "")
        ).strip()
        if not self._worker_device_name:
            raise RuntimeError("D435i worker did not report a device name")
        self._validate_device_name(self._worker_device_name)
        self._validate_fov()

    def capture(self) -> D435iFrame | None:
        if self.mode == "worker":
            return self._capture_worker()
        try:
            frames = self.pipeline.wait_for_frames(
                max(1, int(self.timeout_s * 1000.0))
            )
        except Exception:
            return None
        frame = frames.get_depth_frame()
        if not frame:
            return None
        capture_time = time.monotonic()
        depth_m = (
            np.asanyarray(frame.get_data()).astype(np.float32)
            * self.depth_scale
        )
        return D435iFrame(depth_m=depth_m, capture_monotonic=capture_time)

    def _capture_worker(self) -> D435iFrame | None:
        if (
            self.worker is None
            or self.worker.stdin is None
            or self.worker.stdout is None
        ):
            raise RuntimeError("D435i worker is not running")
        self.worker.stdin.write(b"R\n")
        self.worker.stdin.flush()

        payload = self._read_worker_payload(
            timeout_s=self.timeout_s,
            allow_header_timeout=True,
        )
        if not payload:
            return None
        depth_m = np.load(BytesIO(payload), allow_pickle=False).astype(
            np.float32
        )
        return D435iFrame(
            depth_m=depth_m, capture_monotonic=time.monotonic()
        )

    def _read_worker_payload(
        self, *, timeout_s: float, allow_header_timeout: bool
    ) -> bytes | None:
        if self.worker is None or self.worker.stdout is None:
            raise RuntimeError("D435i worker is not running")
        fd = self.worker.stdout.fileno()
        deadline = time.monotonic() + float(timeout_s)
        while len(self._worker_buffer) < 4:
            readable, _, _ = select.select(
                [fd], [], [], max(0.0, deadline - time.monotonic())
            )
            if not readable:
                if allow_header_timeout:
                    return None
                raise RuntimeError(
                    "Timed out waiting for D435i worker startup metadata"
                )
            chunk = os.read(fd, 65536)
            if not chunk:
                self._raise_worker_exit()
            self._worker_buffer.extend(chunk)
        payload_size = struct.unpack("<I", self._worker_buffer[:4])[0]
        del self._worker_buffer[:4]
        if payload_size == 0:
            return b""
        if payload_size > 128 * 1024 * 1024:
            raise RuntimeError(
                f"D435i worker payload is too large: {payload_size} bytes"
            )
        while len(self._worker_buffer) < payload_size:
            readable, _, _ = select.select(
                [fd], [], [], max(0.0, deadline - time.monotonic())
            )
            if not readable:
                raise RuntimeError("D435i worker 返回了不完整的帧")
            chunk = os.read(fd, 65536)
            if not chunk:
                self._raise_worker_exit()
            self._worker_buffer.extend(chunk)
        payload = bytes(self._worker_buffer[:payload_size])
        del self._worker_buffer[:payload_size]
        return payload

    def _raise_worker_exit(self) -> None:
        stderr = ""
        if self.worker is not None and self.worker.stderr is not None:
            stderr = self.worker.stderr.read().decode(
                errors="replace"
            ).strip()
        raise RuntimeError(f"D435i worker 已退出。{stderr}")

    def close(self) -> None:
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None
        if self.worker is not None:
            try:
                if self.worker.stdin is not None:
                    self.worker.stdin.write(b"Q\n")
                    self.worker.stdin.flush()
                self.worker.wait(timeout=2.0)
            except Exception:
                self.worker.terminate()
            self.worker = None


__all__ = ["D435iFrame", "D435iSource"]
