"""Detect eye closure and driver head direction with MediaPipe Face Mesh."""

from __future__ import annotations

import time

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from driver_monitor_interfaces.msg import DriverMetrics
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from .core import eye_aspect_ratio, is_looking_away


LEFT_EYE = (33, 160, 158, 133, 153, 144)
RIGHT_EYE = (362, 385, 387, 263, 373, 380)
POSE_INDICES = (1, 152, 33, 263, 61, 291)
MODEL_POINTS = np.array(
    [
        (0.0, 0.0, 0.0),
        (0.0, -63.6, -12.5),
        (-43.3, 32.7, -26.0),
        (43.3, 32.7, -26.0),
        (-28.9, -28.9, -24.1),
        (28.9, -28.9, -24.1),
    ],
    dtype=np.float64,
)


class DriverMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("driver_monitor_node")
        defaults = {
            "ear_threshold": 0.20,
            "yaw_threshold_deg": 30.0,
            "pitch_threshold_deg": 20.0,
            "calibration_seconds": 5.0,
            "draw_landmarks": True,
            "show_metrics_overlay": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.bridge = CvBridge()
        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            refine_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        image_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.metrics_pub = self.create_publisher(DriverMetrics, "/driver/metrics", 10)
        self.image_pub = self.create_publisher(
            Image, "/driver/image_annotated", image_qos
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/driver/system_status", 10
        )
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self._on_image, image_qos
        )
        self.calibration_started: float | None = None
        self.calibration_yaw: list[float] = []
        self.calibration_pitch: list[float] = []
        self.yaw_offset = 0.0
        self.pitch_offset = 0.0
        self.calibrated = False
        self.last_process_time = time.monotonic()
        self.fps = 0.0
        self.last_success_time = 0.0
        self.last_error = ""
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_timer(1.0, self._publish_diagnostic)

    def _on_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        ranges = {
            "ear_threshold": (0.05, 0.5),
            "yaw_threshold_deg": (5.0, 90.0),
            "pitch_threshold_deg": (5.0, 90.0),
            "calibration_seconds": (0.0, 30.0),
        }
        recalibrate = False
        for parameter in parameters:
            if parameter.name in ranges:
                minimum, maximum = ranges[parameter.name]
                if not minimum <= float(parameter.value) <= maximum:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be between {minimum} and {maximum}",
                    )
                recalibrate = recalibrate or parameter.name == "calibration_seconds"
            if parameter.name in ("draw_landmarks", "show_metrics_overlay") and not isinstance(
                parameter.value, bool
            ):
                return SetParametersResult(successful=False, reason="overlay values must be boolean")
        if recalibrate:
            self.calibration_started = None
            self.calibration_yaw.clear()
            self.calibration_pitch.clear()
            self.calibrated = False
        return SetParametersResult(successful=True)

    @staticmethod
    def _point(landmarks, index: int, width: int, height: int) -> tuple[float, float]:
        point = landmarks[index]
        return point.x * width, point.y * height

    def _estimate_pose(self, landmarks, width: int, height: int) -> tuple[float, float]:
        image_points = np.array(
            [self._point(landmarks, index, width, height) for index in POSE_INDICES],
            dtype=np.float64,
        )
        focal = float(width)
        camera_matrix = np.array(
            [[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        ok, rotation, _ = cv2.solvePnP(
            MODEL_POINTS,
            image_points,
            camera_matrix,
            np.zeros((4, 1)),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return 0.0, 0.0
        rotation_matrix, _ = cv2.Rodrigues(rotation)
        angles = cv2.RQDecomp3x3(rotation_matrix)[0]
        return float(angles[1]), float(angles[0])

    def _on_image(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            height, width = frame.shape[:2]
            result = self.face_mesh.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            metrics = DriverMetrics()
            metrics.stamp = self.get_clock().now().to_msg()
            metrics.looking_away = True
            if result.multi_face_landmarks:
                face = result.multi_face_landmarks[0]
                landmarks = face.landmark
                left = [self._point(landmarks, i, width, height) for i in LEFT_EYE]
                right = [self._point(landmarks, i, width, height) for i in RIGHT_EYE]
                ear = (eye_aspect_ratio(left) + eye_aspect_ratio(right)) / 2.0
                raw_yaw, raw_pitch = self._estimate_pose(landmarks, width, height)
                calibration_seconds = float(
                    self.get_parameter("calibration_seconds").value
                )
                if not self.calibrated:
                    if self.calibration_started is None:
                        self.calibration_started = time.monotonic()
                    self.calibration_yaw.append(raw_yaw)
                    self.calibration_pitch.append(raw_pitch)
                    if time.monotonic() - self.calibration_started >= calibration_seconds:
                        self.yaw_offset = float(np.median(self.calibration_yaw))
                        self.pitch_offset = float(np.median(self.calibration_pitch))
                        self.calibrated = True
                        self.get_logger().info(
                            f"Calibration complete: yaw={self.yaw_offset:.1f}, "
                            f"pitch={self.pitch_offset:.1f}"
                        )
                yaw = raw_yaw - self.yaw_offset
                pitch = raw_pitch - self.pitch_offset
                metrics.face_detected = True
                metrics.eye_aspect_ratio = ear
                metrics.eyes_closed = ear < float(
                    self.get_parameter("ear_threshold").value
                )
                metrics.yaw_deg = yaw
                metrics.pitch_deg = pitch
                metrics.looking_away = is_looking_away(
                    yaw,
                    pitch,
                    float(self.get_parameter("yaw_threshold_deg").value),
                    float(self.get_parameter("pitch_threshold_deg").value),
                )
                metrics.confidence = 1.0
                if self.get_parameter("draw_landmarks").value:
                    mp.solutions.drawing_utils.draw_landmarks(
                        frame,
                        face,
                        mp.solutions.face_mesh.FACEMESH_CONTOURS,
                        landmark_drawing_spec=None,
                        connection_drawing_spec=mp.solutions.drawing_styles.get_default_face_mesh_contours_style(),
                    )
            self.metrics_pub.publish(metrics)
            now = time.monotonic()
            self.last_success_time = now
            self.last_error = ""
            interval = now - self.last_process_time
            if interval > 0:
                current_fps = 1.0 / interval
                self.fps = current_fps if self.fps == 0 else 0.9 * self.fps + 0.1 * current_fps
            self.last_process_time = now
            status = "CALIBRATING" if not self.calibrated else (
                "NO FACE" if not metrics.face_detected else
                "EYES CLOSED" if metrics.eyes_closed else
                "LOOKING AWAY" if metrics.looking_away else "NORMAL"
            )
            color = (0, 255, 0) if status == "NORMAL" else (0, 0, 255)
            if bool(self.get_parameter("show_metrics_overlay").value):
                lines = [
                    f"State: {status}",
                    f"EAR: {metrics.eye_aspect_ratio:.3f}",
                    f"Yaw/Pitch: {metrics.yaw_deg:.1f}/{metrics.pitch_deg:.1f}",
                    f"FPS: {self.fps:.1f}",
                ]
                for row, text in enumerate(lines):
                    cv2.putText(frame, text, (12, 28 + row * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
            annotated = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            annotated.header = message.header
            self.image_pub.publish(annotated)
        except Exception as exc:
            self.last_error = str(exc)
            self.get_logger().error(f"Frame processing failed: {exc}")

    def _publish_diagnostic(self) -> None:
        stale = (
            not self.last_success_time
            or time.monotonic() - self.last_success_time > 1.0
        )
        status = DiagnosticStatus()
        status.name = "driver_monitor/detector"
        status.hardware_id = "mediapipe_face_mesh"
        status.level = DiagnosticStatus.ERROR if stale else DiagnosticStatus.OK
        status.message = self.last_error or ("No recent result" if stale else "OK")
        status.values = [
            KeyValue(key="fps", value=f"{self.fps:.1f}"),
            KeyValue(key="calibrated", value=str(self.calibrated).lower()),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def destroy_node(self) -> bool:
        self.face_mesh.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriverMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
