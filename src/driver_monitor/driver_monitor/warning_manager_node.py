"""Apply temporal thresholds and hysteresis to instantaneous driver metrics."""

from __future__ import annotations

import time

import rclpy
from driver_monitor_interfaces.msg import DriverMetrics, WarningState
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from .core import DetectionSample, WarningEngine


class WarningManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("warning_manager_node")
        defaults = {
            "eye_trigger_seconds": 2.0,
            "distraction_trigger_seconds": 3.0,
            "out_of_frame_trigger_seconds": 1.5,
            "eye_clear_seconds": 0.5,
            "distraction_clear_seconds": 1.0,
            "out_of_frame_clear_seconds": 0.5,
            "metrics_timeout_seconds": 1.0,
            "publish_rate_hz": 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.engine = WarningEngine(
            eye_trigger_seconds=float(self.get_parameter("eye_trigger_seconds").value),
            distraction_trigger_seconds=float(
                self.get_parameter("distraction_trigger_seconds").value
            ),
            out_of_frame_trigger_seconds=float(
                self.get_parameter("out_of_frame_trigger_seconds").value
            ),
            eye_clear_seconds=float(self.get_parameter("eye_clear_seconds").value),
            distraction_clear_seconds=float(
                self.get_parameter("distraction_clear_seconds").value
            ),
            out_of_frame_clear_seconds=float(
                self.get_parameter("out_of_frame_clear_seconds").value
            ),
        )
        self.latest = DetectionSample()
        self.last_metrics_time: float | None = None
        self.publisher = self.create_publisher(WarningState, "/driver/warning", 10)
        self.subscription = self.create_subscription(
            DriverMetrics, "/driver/metrics", self._on_metrics, 10
        )
        self.add_on_set_parameters_callback(self._on_parameters)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(1.0, rate), self._publish)

    def _on_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        ranges = {
            "eye_trigger_seconds": (0.1, 30.0),
            "distraction_trigger_seconds": (0.1, 30.0),
            "out_of_frame_trigger_seconds": (0.1, 30.0),
            "eye_clear_seconds": (0.0, 10.0),
            "distraction_clear_seconds": (0.0, 10.0),
            "out_of_frame_clear_seconds": (0.0, 10.0),
            "metrics_timeout_seconds": (0.1, 10.0),
            "publish_rate_hz": (1.0, 50.0),
        }
        candidate = {name: float(self.get_parameter(name).value) for name in ranges}
        for parameter in parameters:
            if parameter.name in ranges:
                minimum, maximum = ranges[parameter.name]
                value = float(parameter.value)
                if not minimum <= value <= maximum:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be between {minimum} and {maximum}",
                    )
                candidate[parameter.name] = value
        timer_fields = {
            "eye_trigger_seconds", "distraction_trigger_seconds",
            "out_of_frame_trigger_seconds",
            "eye_clear_seconds", "distraction_clear_seconds",
            "out_of_frame_clear_seconds",
        }
        if any(parameter.name in timer_fields for parameter in parameters):
            self.engine = WarningEngine(
                eye_trigger_seconds=candidate["eye_trigger_seconds"],
                distraction_trigger_seconds=candidate["distraction_trigger_seconds"],
                out_of_frame_trigger_seconds=candidate[
                    "out_of_frame_trigger_seconds"
                ],
                eye_clear_seconds=candidate["eye_clear_seconds"],
                distraction_clear_seconds=candidate["distraction_clear_seconds"],
                out_of_frame_clear_seconds=candidate["out_of_frame_clear_seconds"],
            )
            self.last_metrics_time = None
        return SetParametersResult(successful=True)

    def _on_metrics(self, message: DriverMetrics) -> None:
        self.latest = DetectionSample(
            face_detected=message.face_detected,
            eyes_closed=message.eyes_closed,
            looking_away=message.looking_away,
        )
        self.last_metrics_time = time.monotonic()

    def _publish(self) -> None:
        now = time.monotonic()
        timeout = float(self.get_parameter("metrics_timeout_seconds").value)
        camera_fault = self.last_metrics_time is None or now - self.last_metrics_time > timeout
        message = WarningState()
        message.stamp = self.get_clock().now().to_msg()
        message.camera_fault = camera_fault
        if camera_fault:
            self.engine.reset()
            message.state = WarningState.SYSTEM_FAULT
        else:
            snapshot = self.engine.update(self.latest, now)
            message.state = snapshot.state
            message.eyes_closed_alarm = snapshot.eyes_closed_alarm
            message.distraction_alarm = snapshot.distraction_alarm
            message.out_of_frame_alarm = snapshot.out_of_frame_alarm
            message.eyes_closed_duration = snapshot.eyes_closed_duration
            message.distraction_duration = snapshot.distraction_duration
            message.out_of_frame_duration = snapshot.out_of_frame_duration
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WarningManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
