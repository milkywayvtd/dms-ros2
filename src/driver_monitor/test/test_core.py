import pytest

from driver_monitor.core import (
    DetectionSample,
    WarningEngine,
    eye_aspect_ratio,
    is_looking_away,
    pack_driver_status,
    pack_heartbeat,
    unpack_arduino_status,
)


def test_eye_aspect_ratio() -> None:
    points = [(0, 0), (2, -1), (8, -1), (10, 0), (8, 1), (2, 1)]
    assert eye_aspect_ratio(points) == pytest.approx(0.2)


def test_head_direction_thresholds() -> None:
    assert not is_looking_away(30.0, -20.0, 30.0, 20.0)
    assert is_looking_away(30.1, 0.0, 30.0, 20.0)
    assert is_looking_away(0.0, -20.1, 30.0, 20.0)


def test_warning_trigger_and_hysteresis() -> None:
    engine = WarningEngine(
        eye_trigger_seconds=2.0,
        distraction_trigger_seconds=3.0,
        eye_clear_seconds=0.5,
        distraction_clear_seconds=1.0,
    )
    closed = DetectionSample(True, True, False)
    assert engine.update(closed, 10.0).state == WarningEngine.NORMAL
    snapshot = engine.update(closed, 12.0)
    assert snapshot.state == WarningEngine.EYES_CLOSED
    opened = DetectionSample(True, False, False)
    assert engine.update(opened, 12.1).eyes_closed_alarm
    assert not engine.update(opened, 12.7).eyes_closed_alarm


def test_looking_away_becomes_distraction() -> None:
    engine = WarningEngine(distraction_trigger_seconds=3.0)
    looking_away = DetectionSample(True, False, True)
    engine.update(looking_away, 0.0)
    assert engine.update(looking_away, 2.9).state == WarningEngine.NORMAL
    assert engine.update(looking_away, 3.0).state == WarningEngine.DISTRACTED


def test_missing_face_becomes_out_of_frame_after_1_5_seconds() -> None:
    engine = WarningEngine(out_of_frame_trigger_seconds=1.5)
    missing = DetectionSample(False, False, True)
    engine.update(missing, 0.0)
    assert engine.update(missing, 1.4).state == WarningEngine.NORMAL
    snapshot = engine.update(missing, 1.5)
    assert snapshot.state == WarningEngine.OUT_OF_FRAME
    assert snapshot.out_of_frame_alarm


def test_reset_clears_latched_alarms() -> None:
    engine = WarningEngine(eye_trigger_seconds=1.0)
    closed = DetectionSample(True, True, False)
    engine.update(closed, 0.0)
    assert engine.update(closed, 1.0).eyes_closed_alarm
    engine.reset()
    assert engine.update(DetectionSample(True, False, False), 20.0).state == 0


def test_driver_status_minimal_layout() -> None:
    payload = pack_driver_status(state=1, counter=7)
    assert payload == bytes([1, 7, 0, 0, 0, 0, 0, 0])


def test_driver_status_counter_wraps_to_uint8() -> None:
    payload = pack_driver_status(state=2, counter=300)
    assert payload == bytes([2, 44, 0, 0, 0, 0, 0, 0])


def test_heartbeat_minimal_layout() -> None:
    payload = pack_heartbeat(protocol_version=1, health=0, counter=9)
    assert payload == bytes([1, 0, 9, 0, 0, 0, 0, 0])


def test_unpack_minimal_arduino_status() -> None:
    status = unpack_arduino_status(bytes([1, 9, 0, 0, 0, 0, 0, 0]))
    assert status.hardware_ok
    assert status.rolling_counter == 9


def test_rejects_invalid_arduino_hardware_status() -> None:
    with pytest.raises(ValueError, match="0 or 1"):
        unpack_arduino_status(bytes([2, 9, 0, 0, 0, 0, 0, 0]))
