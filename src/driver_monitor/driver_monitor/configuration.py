"""Validation, presets and atomic persistence for dashboard configuration."""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any

import yaml


CAMERA_MODES = ((640, 480), (1296, 972), (1920, 1080), (2592, 1944))

SENSITIVITY_PRESETS = {
    "low": {
        "ear_threshold": 0.17,
        "yaw_threshold_deg": 40.0,
        "pitch_threshold_deg": 30.0,
        "eye_trigger_seconds": 3.0,
        "distraction_trigger_seconds": 5.0,
        "out_of_frame_trigger_seconds": 1.5,
    },
    "normal": {
        "ear_threshold": 0.20,
        "yaw_threshold_deg": 30.0,
        "pitch_threshold_deg": 20.0,
        "eye_trigger_seconds": 2.0,
        "distraction_trigger_seconds": 3.0,
        "out_of_frame_trigger_seconds": 1.5,
    },
    "high": {
        "ear_threshold": 0.23,
        "yaw_threshold_deg": 25.0,
        "pitch_threshold_deg": 15.0,
        "eye_trigger_seconds": 1.5,
        "distraction_trigger_seconds": 2.0,
        "out_of_frame_trigger_seconds": 1.5,
    },
}

# type, minimum, maximum. None means no numeric range check.
PARAMETER_SCHEMA: dict[str, dict[str, tuple[type, float | None, float | None]]] = {
    "camera_node": {
        "width": (int, 640, 2592),
        "height": (int, 480, 1944),
        "fps": (float, 1.0, 30.0),
        "vertical_blanking": (int, 24, 32287),
        "exposure": (int, 4, 996),
        "analogue_gain": (int, 16, 1023),
        "gamma": (float, 0.1, 3.0),
        "brightness": (float, -100.0, 100.0),
        "contrast": (float, 0.5, 2.0),
        "saturation": (float, 0.0, 2.0),
        "horizontal_flip": (bool, None, None),
        "vertical_flip": (bool, None, None),
    },
    "driver_monitor_node": {
        "ear_threshold": (float, 0.05, 0.5),
        "yaw_threshold_deg": (float, 5.0, 90.0),
        "pitch_threshold_deg": (float, 5.0, 90.0),
        "calibration_seconds": (float, 0.0, 30.0),
        "draw_landmarks": (bool, None, None),
        "show_metrics_overlay": (bool, None, None),
    },
    "warning_manager_node": {
        "eye_trigger_seconds": (float, 0.1, 30.0),
        "distraction_trigger_seconds": (float, 0.1, 30.0),
        "out_of_frame_trigger_seconds": (float, 0.1, 30.0),
        "eye_clear_seconds": (float, 0.0, 10.0),
        "distraction_clear_seconds": (float, 0.0, 10.0),
        "out_of_frame_clear_seconds": (float, 0.0, 10.0),
        "metrics_timeout_seconds": (float, 0.1, 10.0),
        "publish_rate_hz": (float, 1.0, 50.0),
    },
    "web_stream_node": {
        "jpeg_quality": (int, 30, 100),
    },
}


def _coerce(value: Any, expected: type, label: str) -> Any:
    if expected is bool:
        if not isinstance(value, bool):
            raise ValueError(f"{label} must be true or false")
        return value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be numeric")
    if expected is int:
        if int(value) != value:
            raise ValueError(f"{label} must be an integer")
        return int(value)
    return float(value)


def validate_config(document: dict[str, Any]) -> dict[str, Any]:
    """Return a normalized copy containing all dashboard-managed parameters."""
    if not isinstance(document, dict):
        raise ValueError("configuration must be an object")
    normalized = copy.deepcopy(document)
    for node, fields in PARAMETER_SCHEMA.items():
        section = normalized.get(node)
        if not isinstance(section, dict) or not isinstance(
            section.get("ros__parameters"), dict
        ):
            raise ValueError(f"missing {node}.ros__parameters")
        parameters = section["ros__parameters"]
        for name, (expected, minimum, maximum) in fields.items():
            if name not in parameters:
                raise ValueError(f"missing {node}.{name}")
            value = _coerce(parameters[name], expected, f"{node}.{name}")
            if minimum is not None and value < minimum:
                raise ValueError(f"{node}.{name} must be >= {minimum}")
            if maximum is not None and value > maximum:
                raise ValueError(f"{node}.{name} must be <= {maximum}")
            parameters[name] = value
    camera = normalized["camera_node"]["ros__parameters"]
    if (camera["width"], camera["height"]) not in CAMERA_MODES:
        raise ValueError("unsupported OV5647 resolution")
    # Preserve ROS parameter types for sections not managed by the dashboard.
    # YAML may load an integral-looking rate as int after an older config write.
    socketcan = normalized.get("socketcan_bridge_node", {}).get("ros__parameters", {})
    for name in ("status_rate_hz", "heartbeat_rate_hz"):
        if name in socketcan:
            socketcan[name] = float(socketcan[name])
    return normalized


def dashboard_parameters(document: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Extract only parameters managed by the dashboard, grouped by ROS node."""
    validated = validate_config(document)
    return {
        node: {
            name: validated[node]["ros__parameters"][name]
            for name in fields
        }
        for node, fields in PARAMETER_SCHEMA.items()
    }


def load_config(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    return validate_config(document)


def atomic_write_config(path: str | Path, document: dict[str, Any]) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    validated = validate_config(document)
    temporary = target.with_name(f".{target.name}.tmp")
    try:
        with temporary.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(validated, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        if temporary.exists():
            temporary.unlink()
