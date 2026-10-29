"""Transmit driver status and heartbeat frames through Linux SocketCAN."""

from __future__ import annotations

import socket
import struct
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from driver_monitor_interfaces.msg import DriverMetrics, WarningState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

from .core import pack_driver_status, pack_heartbeat, unpack_arduino_status


CAN_FRAME_FORMAT = "=IB3x8s"


class SocketCANBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("socketcan_bridge_node")
        defaults = {
            "interface": "can0",
            "status_can_id": 0x180,
            "heartbeat_can_id": 0x181,
            "feedback_can_id": 0x280,
            "status_rate_hz": 10.0,
            "heartbeat_rate_hz": 1.0,
            "feedback_timeout_seconds": 3.0,
            "protocol_version": 1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.warning = WarningState()
        self.warning.camera_fault = True
        self.metrics = DriverMetrics()
        self.last_metric_arrival: float | None = None
        self.fps = 0.0
        self.status_counter = 0
        self.heartbeat_counter = 0
        self.can_socket: socket.socket | None = None
        self.last_error = ""
        self.last_feedback_arrival: float | None = None
        self.feedback = None
        self.debug_pub = self.create_publisher(UInt8MultiArray, "/driver/can_tx", 10)
        self.arduino_status_pub = self.create_publisher(
            UInt8MultiArray, "/arduino/status", 10
        )
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, "/driver/system_status", 10
        )
        self.create_subscription(WarningState, "/driver/warning", self._on_warning, 10)
        self.create_subscription(DriverMetrics, "/driver/metrics", self._on_metrics, 10)
        self._open_socket()
        self.create_timer(
            1.0 / max(1.0, float(self.get_parameter("status_rate_hz").value)),
            self._send_status,
        )
        self.create_timer(
            1.0 / max(0.1, float(self.get_parameter("heartbeat_rate_hz").value)),
            self._send_heartbeat,
        )
        self.create_timer(0.02, self._receive_feedback)

    def _open_socket(self) -> bool:
        if self.can_socket is not None:
            return True
        try:
            can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            can_socket.bind((str(self.get_parameter("interface").value),))
            can_socket.setblocking(False)
            self.can_socket = can_socket
            self.last_error = ""
            self.get_logger().info(
                f"SocketCAN connected to {self.get_parameter('interface').value}"
            )
            return True
        except OSError as exc:
            self.last_error = str(exc)
            self.get_logger().warning(f"SocketCAN unavailable: {exc}")
            return False

    def _receive_feedback(self) -> None:
        if not self._open_socket() or self.can_socket is None:
            return
        try:
            while True:
                frame = self.can_socket.recv(16)
                can_id, dlc, payload = struct.unpack(CAN_FRAME_FORMAT, frame)
                if (can_id & 0x1FFFFFFF) != int(
                    self.get_parameter("feedback_can_id").value
                ):
                    continue
                if dlc != 8:
                    self.get_logger().warning("Ignored CAN 0x280 with DLC != 8")
                    continue
                try:
                    status = unpack_arduino_status(payload)
                except ValueError as exc:
                    self.get_logger().warning(f"Ignored invalid CAN 0x280: {exc}")
                    continue
                self.feedback = status
                self.last_feedback_arrival = time.monotonic()
                message = UInt8MultiArray()
                message.data = [
                    int(status.hardware_ok),
                    status.rolling_counter,
                ]
                self.arduino_status_pub.publish(message)
        except BlockingIOError:
            pass
        except OSError as exc:
            self.last_error = str(exc)

    def _on_warning(self, message: WarningState) -> None:
        self.warning = message

    def _on_metrics(self, message: DriverMetrics) -> None:
        now = time.monotonic()
        if self.last_metric_arrival is not None:
            interval = now - self.last_metric_arrival
            if interval > 0:
                current = 1.0 / interval
                self.fps = current if self.fps == 0 else 0.9 * self.fps + 0.1 * current
        self.last_metric_arrival = now
        self.metrics = message

    def _send(self, can_id: int, payload: bytes) -> bool:
        if not self._open_socket():
            return False
        try:
            assert self.can_socket is not None
            frame = struct.pack(CAN_FRAME_FORMAT, can_id, len(payload), payload)
            self.can_socket.send(frame)
            debug = UInt8MultiArray()
            debug.data = [can_id & 0xFF, (can_id >> 8) & 0xFF, *payload]
            self.debug_pub.publish(debug)
            self.last_error = ""
            return True
        except OSError as exc:
            self.last_error = str(exc)
            self.get_logger().error(f"CAN transmit failed: {exc}")
            if self.can_socket is not None:
                self.can_socket.close()
                self.can_socket = None
            return False

    def _send_status(self) -> None:
        payload = pack_driver_status(
            state=self.warning.state,
            counter=self.status_counter,
        )
        self._send(int(self.get_parameter("status_can_id").value), payload)
        self.status_counter = (self.status_counter + 1) & 0xFF

    def _send_heartbeat(self) -> None:
        health = 1 if self.warning.camera_fault else 0
        if self.last_error:
            health = 3
        payload = pack_heartbeat(
            protocol_version=int(self.get_parameter("protocol_version").value),
            health=health,
            counter=self.heartbeat_counter,
        )
        self._send(int(self.get_parameter("heartbeat_can_id").value), payload)
        self.heartbeat_counter = (self.heartbeat_counter + 1) & 0xFF
        status = DiagnosticStatus()
        status.name = "driver_monitor/socketcan"
        status.hardware_id = str(self.get_parameter("interface").value)
        status.level = DiagnosticStatus.ERROR if self.last_error else DiagnosticStatus.OK
        status.message = self.last_error or "OK"
        status.values = [KeyValue(key="fps", value=f"{self.fps:.1f}")]
        feedback_online = (
            self.last_feedback_arrival is not None
            and time.monotonic() - self.last_feedback_arrival
            <= float(self.get_parameter("feedback_timeout_seconds").value)
        )
        status.values.extend(
            [
                KeyValue(key="arduino_online", value=str(feedback_online).lower()),
                KeyValue(
                    key="arduino_hardware_ok",
                    value=str(bool(self.feedback and self.feedback.hardware_ok)).lower(),
                ),
            ]
        )
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def destroy_node(self) -> bool:
        if self.can_socket is not None:
            self.can_socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SocketCANBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
