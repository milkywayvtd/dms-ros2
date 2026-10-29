"""Dashboard, MJPEG stream and live ROS configuration API."""

from __future__ import annotations

import copy
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from driver_monitor_interfaces.msg import DriverMetrics, WarningState
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .configuration import (
    CAMERA_MODES,
    SENSITIVITY_PRESETS,
    atomic_write_config,
    dashboard_parameters,
    load_config,
    validate_config,
)


def byte_value(value: Any) -> int:
    """Normalize ROS IDL byte/uint8 values across Humble Python generators."""
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else 0
    return int(value)


class FrameStore:
    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.frame: bytes | None = None
        self.sequence = 0
        self.running = True

    def update(self, frame: bytes) -> None:
        with self.condition:
            self.frame = frame
            self.sequence += 1
            self.condition.notify_all()

    def wait_next(self, previous: int) -> tuple[bytes | None, int]:
        with self.condition:
            self.condition.wait_for(
                lambda: self.sequence != previous or not self.running, timeout=2.0
            )
            return self.frame, self.sequence

    def stop(self) -> None:
        with self.condition:
            self.running = False
            self.condition.notify_all()


class DashboardState:
    LABELS = {
        WarningState.NORMAL: "NORMAL",
        WarningState.EYES_CLOSED: "DROWSY",
        WarningState.DISTRACTED: "DISTRACTED",
        WarningState.OUT_OF_FRAME: "OUT OF FRAME",
        WarningState.SYSTEM_FAULT: "SYSTEM FAULT",
    }

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.warning = WarningState()
        self.metrics = DriverMetrics()
        self.diagnostics: dict[str, dict[str, Any]] = {}

    def update_warning(self, message: WarningState) -> None:
        with self.lock:
            self.warning = message

    def update_metrics(self, message: DriverMetrics) -> None:
        with self.lock:
            self.metrics = message

    def update_diagnostics(self, message: DiagnosticArray) -> None:
        with self.lock:
            for status in message.status:
                self.diagnostics[status.name] = {
                    "level": byte_value(status.level),
                    "message": status.message,
                    "values": {item.key: item.value for item in status.values},
                }

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            warning, metrics = self.warning, self.metrics
            return {
                "warning": {
                    "state": byte_value(warning.state),
                    "label": self.LABELS.get(byte_value(warning.state), "UNKNOWN"),
                    "eyes_closed_alarm": bool(warning.eyes_closed_alarm),
                    "distraction_alarm": bool(warning.distraction_alarm),
                    "out_of_frame_alarm": bool(warning.out_of_frame_alarm),
                    "eyes_closed_duration": round(float(warning.eyes_closed_duration), 2),
                    "distraction_duration": round(float(warning.distraction_duration), 2),
                    "out_of_frame_duration": round(
                        float(warning.out_of_frame_duration), 2
                    ),
                    "camera_fault": bool(warning.camera_fault),
                },
                "metrics": {
                    "face_detected": bool(metrics.face_detected),
                    "eye_aspect_ratio": round(float(metrics.eye_aspect_ratio), 4),
                    "eyes_closed": bool(metrics.eyes_closed),
                    "yaw_deg": round(float(metrics.yaw_deg), 2),
                    "pitch_deg": round(float(metrics.pitch_deg), 2),
                    "looking_away": bool(metrics.looking_away),
                    "confidence": round(float(metrics.confidence), 3),
                },
                "diagnostics": dict(self.diagnostics),
            }


class ConfigurationController:
    def __init__(self, node: "WebStreamNode", config_path: Path, defaults_path: Path):
        self.node = node
        self.config_path = config_path
        self.defaults_path = defaults_path
        self.lock = threading.Lock()
        self.clients = {
            name: node.create_client(
                SetParametersAtomically, f"/{name}/set_parameters_atomically"
            )
            for name in (
                "camera_node", "driver_monitor_node", "warning_manager_node",
                "web_stream_node",
            )
        }

    def response(self) -> dict[str, Any]:
        return {
            "config": load_config(self.config_path),
            "presets": SENSITIVITY_PRESETS,
            "camera_modes": [
                {"width": width, "height": height} for width, height in CAMERA_MODES
            ],
        }

    @staticmethod
    def _wait(future, timeout: float = 5.0):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout):
            raise TimeoutError("ROS parameter service timed out")
        exception = future.exception()
        if exception:
            raise exception
        return future.result()

    def _set_parameters(self, node_name: str, values: dict[str, Any]) -> None:
        client = self.clients[node_name]
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"ROS node {node_name} is unavailable")
        parameters = [Parameter(name=name, value=value) for name, value in values.items()]
        request = SetParametersAtomically.Request()
        request.parameters = [parameter.to_parameter_msg() for parameter in parameters]
        response = self._wait(client.call_async(request))
        if not response.result.successful:
            raise ValueError(f"{node_name}: {response.result.reason}")

    def apply(self, document: dict[str, Any]) -> dict[str, Any]:
        candidate = validate_config(document)
        candidate_parameters = dashboard_parameters(candidate)
        with self.lock:
            previous = load_config(self.config_path)
            previous_parameters = dashboard_parameters(previous)
            applied: list[str] = []
            try:
                for node_name, values in candidate_parameters.items():
                    self._set_parameters(node_name, values)
                    applied.append(node_name)
                atomic_write_config(self.config_path, candidate)
            except Exception:
                for node_name in reversed(applied):
                    try:
                        self._set_parameters(node_name, previous_parameters[node_name])
                    except Exception as rollback_error:
                        self.node.get_logger().error(
                            f"Configuration rollback failed for {node_name}: {rollback_error}"
                        )
                raise
        return self.response()

    def apply_video(self, changes: dict[str, Any]) -> dict[str, Any]:
        """Apply only camera/video fields without touching pending warning values."""
        allowed = {
            "camera_node": {
                "width", "height", "fps", "exposure", "analogue_gain", "gamma",
                "brightness", "contrast", "saturation", "horizontal_flip",
                "vertical_flip",
            },
            "driver_monitor_node": {"draw_landmarks", "show_metrics_overlay"},
            "web_stream_node": {"jpeg_quality"},
        }
        if not isinstance(changes, dict):
            raise ValueError("video configuration must be an object")
        candidate = copy.deepcopy(load_config(self.config_path))
        for node_name, values in changes.items():
            if node_name not in allowed or not isinstance(values, dict):
                raise ValueError(f"unsupported video configuration section: {node_name}")
            unknown = set(values) - allowed[node_name]
            if unknown:
                raise ValueError(
                    f"unsupported video parameter: {node_name}.{sorted(unknown)[0]}"
                )
            candidate[node_name]["ros__parameters"].update(values)
        return self.apply(candidate)

    def apply_warning(self, changes: dict[str, Any]) -> dict[str, Any]:
        """Apply recognition/warning fields only after the dashboard Apply action."""
        allowed = {
            "driver_monitor_node": {
                "ear_threshold", "yaw_threshold_deg", "pitch_threshold_deg",
                "calibration_seconds",
            },
            "warning_manager_node": {
                "eye_trigger_seconds", "distraction_trigger_seconds",
                "out_of_frame_trigger_seconds", "eye_clear_seconds",
                "distraction_clear_seconds", "out_of_frame_clear_seconds",
                "metrics_timeout_seconds",
            },
        }
        if not isinstance(changes, dict):
            raise ValueError("warning configuration must be an object")
        candidate = copy.deepcopy(load_config(self.config_path))
        for node_name, values in changes.items():
            if node_name not in allowed or not isinstance(values, dict):
                raise ValueError(f"unsupported warning configuration section: {node_name}")
            unknown = set(values) - allowed[node_name]
            if unknown:
                raise ValueError(
                    f"unsupported warning parameter: {node_name}.{sorted(unknown)[0]}"
                )
            candidate[node_name]["ros__parameters"].update(values)
        return self.apply(candidate)

    def restore_defaults(self) -> dict[str, Any]:
        return self.apply(load_config(self.defaults_path))


def make_handler(node: "WebStreamNode", static_dir: Path):
    mime_types = {".html": "text/html", ".css": "text/css", ".js": "text/javascript"}

    class Handler(BaseHTTPRequestHandler):
        def _send_bytes(self, status: int, body: bytes, content_type: str) -> None:
            self.send_response(status)
            self.send_header("Content-Type", f"{content_type}; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, status: int, payload: Any) -> None:
            self._send_bytes(
                status,
                json.dumps(payload, ensure_ascii=False).encode("utf-8"),
                "application/json",
            )

        def _read_json(self) -> dict[str, Any]:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 1024 * 1024:
                raise ValueError("invalid request size")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("JSON body must be an object")
            return payload

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path == "/stream.mjpg":
                self._stream()
                return
            if path == "/api/status":
                self._json(200, node.state.snapshot())
                return
            if path == "/api/config":
                try:
                    self._json(200, node.configuration.response())
                except Exception as exc:
                    self._json(500, {"error": str(exc)})
                return
            filename = "index.html" if path in ("/", "/index.html") else path.lstrip("/")
            if filename not in ("index.html", "styles.css", "app.js"):
                self.send_error(404)
                return
            target = static_dir / filename
            if not target.is_file():
                self.send_error(404)
                return
            self._send_bytes(200, target.read_bytes(), mime_types.get(target.suffix, "application/octet-stream"))

        def do_PUT(self) -> None:
            path = urlparse(self.path).path
            if path not in ("/api/config", "/api/config/video", "/api/config/warning"):
                self.send_error(404)
                return
            try:
                payload = self._read_json()
                if path == "/api/config/video":
                    result = node.configuration.apply_video(payload)
                elif path == "/api/config/warning":
                    result = node.configuration.apply_warning(payload)
                else:
                    result = node.configuration.apply(payload)
                self._json(200, result)
            except (ValueError, TimeoutError, RuntimeError) as exc:
                self._json(400, {"error": str(exc)})
            except Exception as exc:
                node.get_logger().error(f"Apply configuration failed: {exc}")
                self._json(500, {"error": str(exc)})

        def do_POST(self) -> None:
            if urlparse(self.path).path != "/api/config/restore-defaults":
                self.send_error(404)
                return
            try:
                self._json(200, node.configuration.restore_defaults())
            except Exception as exc:
                node.get_logger().error(f"Restore defaults failed: {exc}")
                self._json(500, {"error": str(exc)})

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, no-store")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            sequence = -1
            try:
                while node.store.running:
                    frame, sequence = node.store.wait_next(sequence)
                    if frame is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, format: str, *args: object) -> None:
            return

    return Handler


class WebStreamNode(Node):
    def __init__(self) -> None:
        super().__init__("web_stream_node")
        source_config = Path.home() / "driver_monitor_ws/src/driver_monitor/config"
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("config_path", str(source_config / "driver_monitor.yaml"))
        self.declare_parameter(
            "defaults_path", str(source_config / "driver_monitor.defaults.yaml")
        )
        self.bridge = CvBridge()
        self.store = FrameStore()
        self.state = DashboardState()
        image_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Image, "/driver/image_annotated", self._on_image, image_qos)
        self.create_subscription(WarningState, "/driver/warning", self.state.update_warning, 10)
        self.create_subscription(DriverMetrics, "/driver/metrics", self.state.update_metrics, 10)
        self.create_subscription(DiagnosticArray, "/driver/system_status", self.state.update_diagnostics, 10)
        self.configuration = ConfigurationController(
            self,
            Path(str(self.get_parameter("config_path").value)),
            Path(str(self.get_parameter("defaults_path").value)),
        )
        address = (
            str(self.get_parameter("host").value),
            int(self.get_parameter("port").value),
        )
        static_dir = Path(get_package_share_directory("driver_monitor")) / "static"
        self.server = ThreadingHTTPServer(address, make_handler(self, static_dir))
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f"Dashboard listening on http://{address[0]}:{address[1]}")

    def _on_image(self, message: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        quality = int(self.get_parameter("jpeg_quality").value)
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if ok:
            self.store.update(encoded.tobytes())

    def destroy_node(self) -> bool:
        self.store.stop()
        self.server.shutdown()
        self.server.server_close()
        self.server_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebStreamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
