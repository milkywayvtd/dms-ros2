"""Capture OV5647 Bayer frames and publish configurable BGR ROS images."""

from __future__ import annotations

import subprocess
import threading
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .configuration import CAMERA_MODES


HARDWARE_PARAMETERS = {
    "width", "height", "vertical_blanking", "exposure", "analogue_gain",
    "horizontal_flip", "vertical_flip",
}


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_node")
        defaults = {
            "video_device": "/dev/video0",
            "sensor_device": "/dev/v4l-subdev0",
            "width": 640,
            "height": 480,
            "fps": 15.0,
            "vertical_blanking": 520,
            "exposure": 900,
            "analogue_gain": 256,
            "gamma": 0.55,
            "brightness": 0.0,
            "contrast": 1.0,
            "saturation": 1.0,
            "horizontal_flip": False,
            "vertical_flip": False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.config_lock = threading.Lock()
        self.capture_config = {name: self.get_parameter(name).value for name in defaults}
        self.bridge = CvBridge()
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.publisher = self.create_publisher(Image, "/camera/image_raw", image_qos)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/driver/system_status", 10
        )
        self.frame_lock = threading.Lock()
        self.latest_frame: np.ndarray | None = None
        self.running = True
        self.restart_event = threading.Event()
        self.process: subprocess.Popen[bytes] | None = None
        self.last_frame_time = 0.0
        self.capture_error = ""
        self.published_frames = 0
        self.fps_window_started = time.monotonic()
        self.measured_fps = 0.0
        self.add_on_set_parameters_callback(self._on_parameters)
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        # Run fast enough for every supported configured FPS; publication is gated below.
        self.last_publish_time = 0.0
        self.create_timer(1.0 / 30.0, self._publish_frame)
        self.create_timer(1.0, self._publish_diagnostic)

    @staticmethod
    def _read_exact(stream, size: int) -> bytes | None:
        data = bytearray()
        while len(data) < size:
            chunk = stream.read(size - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)

    @staticmethod
    def _validate(config: dict[str, Any]) -> str:
        if (int(config["width"]), int(config["height"])) not in CAMERA_MODES:
            return "unsupported OV5647 resolution"
        ranges = {
            "fps": (1.0, 30.0), "vertical_blanking": (24, 32287),
            "exposure": (4, 996), "analogue_gain": (16, 1023),
            "gamma": (0.1, 3.0), "brightness": (-100.0, 100.0),
            "contrast": (0.5, 2.0), "saturation": (0.0, 2.0),
        }
        for name, (minimum, maximum) in ranges.items():
            value = float(config[name])
            if not minimum <= value <= maximum:
                return f"{name} must be between {minimum} and {maximum}"
        return ""

    def _on_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        with self.config_lock:
            candidate = dict(self.capture_config)
            for parameter in parameters:
                if parameter.name in candidate:
                    candidate[parameter.name] = parameter.value
            error = self._validate(candidate)
            if error:
                return SetParametersResult(successful=False, reason=error)
            restart = any(
                p.name in HARDWARE_PARAMETERS
                and p.name in self.capture_config
                and p.value != self.capture_config[p.name]
                for p in parameters
            )
            self.capture_config = candidate
        if restart:
            self.restart_event.set()
            process = self.process
            if process and process.poll() is None:
                process.terminate()
        return SetParametersResult(successful=True)

    def _snapshot(self) -> dict[str, Any]:
        with self.config_lock:
            return dict(self.capture_config)

    @staticmethod
    def _bayer_conversion(horizontal: bool, vertical: bool) -> int:
        if horizontal and vertical:
            return cv2.COLOR_BayerGR2BGR
        if horizontal:
            return cv2.COLOR_BayerBG2BGR
        if vertical:
            return cv2.COLOR_BayerRG2BGR
        return cv2.COLOR_BayerGB2BGR

    @staticmethod
    def _pixel_format(horizontal: bool, vertical: bool) -> str:
        if horizontal and vertical:
            return "BA10"  # GRBG10 in the Raspberry Pi V4L2 driver.
        if horizontal:
            return "BG10"
        if vertical:
            return "RG10"
        return "GB10"

    @staticmethod
    def _adjust_image(frame: np.ndarray, config: dict[str, Any]) -> np.ndarray:
        gamma = float(config["gamma"])
        lut = np.array(
            [((value / 255.0) ** gamma) * 255.0 for value in range(256)],
            dtype=np.uint8,
        )
        result = cv2.LUT(frame, lut)
        result = cv2.convertScaleAbs(
            result,
            alpha=float(config["contrast"]),
            beta=float(config["brightness"]),
        )
        saturation = float(config["saturation"])
        if saturation != 1.0:
            hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
            hsv[:, :, 1] = np.clip(
                hsv[:, :, 1].astype(np.float32) * saturation, 0, 255
            ).astype(np.uint8)
            result = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        return result

    def _configure_sensor(self, config: dict[str, Any]) -> None:
        sensor = str(config["sensor_device"])
        width, height = int(config["width"]), int(config["height"])
        for name, value in (
            ("horizontal_flip", int(bool(config["horizontal_flip"]))),
            ("vertical_flip", int(bool(config["vertical_flip"]))),
        ):
            subprocess.run(
                ["v4l2-ctl", "-d", sensor, f"--set-ctrl={name}={value}"],
                check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            )
        subprocess.run(
            [
                "v4l2-ctl", "-d", sensor,
                f"--set-subdev-fmt=pad=0,width={width},height={height},code=0x300e",
            ],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
        )
        for name, value in (
            ("vertical_blanking", int(config["vertical_blanking"])),
            ("exposure", int(config["exposure"])),
            ("analogue_gain", int(config["analogue_gain"])),
        ):
            subprocess.run(
                ["v4l2-ctl", "-d", sensor, f"--set-ctrl={name}={value}"],
                check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            )

    def _capture_once(self, config: dict[str, Any]) -> None:
        self._configure_sensor(config)
        width, height = int(config["width"]), int(config["height"])
        horizontal = bool(config["horizontal_flip"])
        vertical = bool(config["vertical_flip"])
        pixel_format = self._pixel_format(horizontal, vertical)
        command = [
            "v4l2-ctl", "-d", str(config["video_device"]),
            f"--set-fmt-video=width={width},height={height},pixelformat={pixel_format}",
            "--stream-mmap=4", "--stream-count=0", "--stream-to=-",
        ]
        self.process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
        )
        assert self.process.stdout is not None
        frame_size = width * height * 2
        conversion = self._bayer_conversion(horizontal, vertical)
        while self.running and not self.restart_event.is_set():
            payload = self._read_exact(self.process.stdout, frame_size)
            if payload is None:
                if self.restart_event.is_set() or not self.running:
                    return
                raise RuntimeError("V4L2 stream ended unexpectedly")
            raw = np.frombuffer(payload, dtype="<u2").reshape(height, width)
            bgr16 = cv2.cvtColor(raw, conversion)
            bgr8 = np.right_shift(bgr16, 2).clip(0, 255).astype(np.uint8)
            frame = self._adjust_image(bgr8, self._snapshot())
            with self.frame_lock:
                self.latest_frame = frame
                self.last_frame_time = time.monotonic()

    def _capture_loop(self) -> None:
        while self.running:
            self.restart_event.clear()
            config = self._snapshot()
            try:
                self._capture_once(config)
                if self.running and not self.restart_event.is_set():
                    raise RuntimeError("V4L2 capture stopped")
                self.capture_error = ""
            except Exception as exc:
                if not self.restart_event.is_set() and self.running:
                    self.capture_error = str(exc)
                    self.get_logger().error(f"Camera capture failed: {exc}")
                    time.sleep(1.0)
            finally:
                process = self.process
                if process and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                self.process = None

    def _publish_frame(self) -> None:
        now = time.monotonic()
        fps = float(self._snapshot()["fps"])
        if now - self.last_publish_time < 1.0 / max(1.0, fps):
            return
        with self.frame_lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
        if frame is None:
            return
        message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "driver_camera"
        self.publisher.publish(message)
        self.last_publish_time = now
        self.published_frames += 1

    def _publish_diagnostic(self) -> None:
        now = time.monotonic()
        elapsed = now - self.fps_window_started
        self.measured_fps = self.published_frames / elapsed if elapsed > 0 else 0.0
        self.published_frames = 0
        self.fps_window_started = now
        stale = not self.last_frame_time or now - self.last_frame_time > 1.0
        status = DiagnosticStatus()
        status.name = "driver_monitor/camera"
        status.hardware_id = str(self._snapshot()["video_device"])
        status.level = DiagnosticStatus.ERROR if stale else DiagnosticStatus.OK
        status.message = self.capture_error or ("No recent frame" if stale else "OK")
        status.values = [KeyValue(key="fps", value=f"{self.measured_fps:.1f}")]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def destroy_node(self) -> bool:
        self.running = False
        self.restart_event.set()
        if self.process and self.process.poll() is None:
            self.process.terminate()
        self.capture_thread.join(timeout=3.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
