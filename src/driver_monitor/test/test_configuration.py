from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from driver_monitor.configuration import (
    SENSITIVITY_PRESETS,
    atomic_write_config,
    load_config,
    validate_config,
)
from driver_monitor.camera_node import CameraNode


DEFAULTS = (
    Path(__file__).parents[1] / "config" / "driver_monitor.defaults.yaml"
)


def default_document():
    return yaml.safe_load(DEFAULTS.read_text(encoding="utf-8"))


def test_default_configuration_is_valid():
    validated = validate_config(default_document())
    assert validated["camera_node"]["ros__parameters"]["width"] == 640


def test_socketcan_rates_keep_ros_double_type():
    document = default_document()
    can = document["socketcan_bridge_node"]["ros__parameters"]
    can["status_rate_hz"] = 10
    can["heartbeat_rate_hz"] = 1
    validated = validate_config(document)
    assert validated["socketcan_bridge_node"]["ros__parameters"]["status_rate_hz"] == 10.0
    assert isinstance(
        validated["socketcan_bridge_node"]["ros__parameters"]["heartbeat_rate_hz"],
        float,
    )


@pytest.mark.parametrize("width,height", [(800, 600), (1920, 972)])
def test_rejects_unsupported_camera_mode(width, height):
    document = default_document()
    camera = document["camera_node"]["ros__parameters"]
    camera["width"], camera["height"] = width, height
    with pytest.raises(ValueError, match="unsupported OV5647 resolution"):
        validate_config(document)


@pytest.mark.parametrize(
    "name,ear,eye_seconds",
    [("low", 0.17, 3.0), ("normal", 0.20, 2.0), ("high", 0.23, 1.5)],
)
def test_sensitivity_presets(name, ear, eye_seconds):
    assert SENSITIVITY_PRESETS[name]["ear_threshold"] == ear
    assert SENSITIVITY_PRESETS[name]["eye_trigger_seconds"] == eye_seconds


def test_atomic_write_does_not_change_defaults(tmp_path):
    defaults_copy = tmp_path / "driver_monitor.defaults.yaml"
    defaults_copy.write_bytes(DEFAULTS.read_bytes())
    before = defaults_copy.read_bytes()
    user_path = tmp_path / "driver_monitor.yaml"
    document = deepcopy(default_document())
    document["camera_node"]["ros__parameters"]["brightness"] = 25.0
    atomic_write_config(user_path, document)
    assert load_config(user_path)["camera_node"]["ros__parameters"]["brightness"] == 25.0
    assert defaults_copy.read_bytes() == before
    assert not (tmp_path / ".driver_monitor.yaml.tmp").exists()


def test_rejects_out_of_range_exposure():
    document = default_document()
    document["camera_node"]["ros__parameters"]["exposure"] = 1000
    with pytest.raises(ValueError, match="exposure"):
        validate_config(document)


@pytest.mark.parametrize(
    "horizontal,vertical,pixel_format",
    [
        (False, False, "GB10"),
        (True, False, "BG10"),
        (False, True, "RG10"),
        (True, True, "BA10"),
    ],
)
def test_flip_selects_matching_bayer_format(horizontal, vertical, pixel_format):
    assert CameraNode._pixel_format(horizontal, vertical) == pixel_format
