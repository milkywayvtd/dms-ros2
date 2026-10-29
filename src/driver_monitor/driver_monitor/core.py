"""Pure-Python algorithms shared by ROS nodes and unit tests."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Sequence


def eye_aspect_ratio(points: Sequence[Sequence[float]]) -> float:
    """Return EAR for points ordered [outer, upper1, upper2, inner, lower2, lower1]."""
    if len(points) != 6:
        raise ValueError("EAR requires exactly six eye landmarks")
    horizontal = hypot(points[0][0] - points[3][0], points[0][1] - points[3][1])
    if horizontal <= 1e-9:
        return 0.0
    vertical_a = hypot(points[1][0] - points[5][0], points[1][1] - points[5][1])
    vertical_b = hypot(points[2][0] - points[4][0], points[2][1] - points[4][1])
    return (vertical_a + vertical_b) / (2.0 * horizontal)


def is_looking_away(
    yaw_deg: float, pitch_deg: float, yaw_threshold: float, pitch_threshold: float
) -> bool:
    return abs(yaw_deg) > yaw_threshold or abs(pitch_deg) > pitch_threshold


@dataclass
class DetectionSample:
    face_detected: bool = False
    eyes_closed: bool = False
    looking_away: bool = True


@dataclass
class WarningSnapshot:
    state: int
    eyes_closed_alarm: bool
    distraction_alarm: bool
    out_of_frame_alarm: bool
    eyes_closed_duration: float
    distraction_duration: float
    out_of_frame_duration: float


class _TimedAlarm:
    def __init__(self, trigger_seconds: float, clear_seconds: float):
        self.trigger_seconds = trigger_seconds
        self.clear_seconds = clear_seconds
        self.active_since: float | None = None
        self.clear_since: float | None = None
        self.alarm = False
        self.duration = 0.0

    def update(self, condition: bool, now: float) -> None:
        if condition:
            self.clear_since = None
            if self.active_since is None:
                self.active_since = now
            self.duration = max(0.0, now - self.active_since)
            if self.duration >= self.trigger_seconds:
                self.alarm = True
            return

        self.duration = 0.0
        if not self.alarm:
            self.active_since = None
            self.clear_since = None
            return
        if self.clear_since is None:
            self.clear_since = now
        if now - self.clear_since >= self.clear_seconds:
            self.alarm = False
            self.active_since = None
            self.clear_since = None


class WarningEngine:
    NORMAL = 0
    EYES_CLOSED = 1
    DISTRACTED = 2
    OUT_OF_FRAME = 3

    def __init__(
        self,
        eye_trigger_seconds: float = 2.0,
        distraction_trigger_seconds: float = 3.0,
        out_of_frame_trigger_seconds: float = 1.5,
        eye_clear_seconds: float = 0.5,
        distraction_clear_seconds: float = 1.0,
        out_of_frame_clear_seconds: float = 0.5,
    ):
        self.eye = _TimedAlarm(eye_trigger_seconds, eye_clear_seconds)
        self.distraction = _TimedAlarm(
            distraction_trigger_seconds, distraction_clear_seconds
        )
        self.out_of_frame = _TimedAlarm(
            out_of_frame_trigger_seconds, out_of_frame_clear_seconds
        )

    def reset(self) -> None:
        """Clear all timers and latched alarms after a sensor fault."""
        self.eye.active_since = None
        self.eye.clear_since = None
        self.eye.alarm = False
        self.eye.duration = 0.0
        self.distraction.active_since = None
        self.distraction.clear_since = None
        self.distraction.alarm = False
        self.distraction.duration = 0.0
        self.out_of_frame.active_since = None
        self.out_of_frame.clear_since = None
        self.out_of_frame.alarm = False
        self.out_of_frame.duration = 0.0

    def update(self, sample: DetectionSample, now: float) -> WarningSnapshot:
        self.eye.update(sample.face_detected and sample.eyes_closed, now)
        self.distraction.update(sample.face_detected and sample.looking_away, now)
        self.out_of_frame.update(not sample.face_detected, now)
        state = self.NORMAL
        if self.out_of_frame.alarm:
            state = self.OUT_OF_FRAME
        elif self.eye.alarm:
            state = self.EYES_CLOSED
        elif self.distraction.alarm:
            state = self.DISTRACTED
        return WarningSnapshot(
            state=state,
            eyes_closed_alarm=self.eye.alarm,
            distraction_alarm=self.distraction.alarm,
            out_of_frame_alarm=self.out_of_frame.alarm,
            eyes_closed_duration=self.eye.duration,
            distraction_duration=self.distraction.duration,
            out_of_frame_duration=self.out_of_frame.duration,
        )


def _u8(value: float | int) -> int:
    return max(0, min(255, int(round(value))))


def pack_driver_status(
    *,
    state: int,
    counter: int,
) -> bytes:
    return bytes([_u8(state), counter & 0xFF, 0, 0, 0, 0, 0, 0])


def pack_heartbeat(
    *, protocol_version: int, health: int, counter: int
) -> bytes:
    return bytes([_u8(protocol_version), _u8(health), counter & 0xFF, 0, 0, 0, 0, 0])


@dataclass(frozen=True)
class ArduinoStatus:
    hardware_ok: bool
    rolling_counter: int


def unpack_arduino_status(payload: bytes) -> ArduinoStatus:
    """Validate and decode the minimal Arduino feedback payload (CAN 0x280)."""
    if len(payload) != 8:
        raise ValueError("Arduino status payload must contain 8 bytes")
    if payload[0] not in (0, 1):
        raise ValueError("Arduino hardware status must be 0 or 1")
    if any(payload[2:8]):
        raise ValueError("Arduino status reserved bytes must be zero")
    return ArduinoStatus(bool(payload[0]), payload[1])
