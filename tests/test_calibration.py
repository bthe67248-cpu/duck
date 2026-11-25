import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from calibration import load_calibration


def write(tmp_path, **kwargs):
    path = tmp_path / "measured.json"
    path.write_text(json.dumps({"schema_version": 1, **kwargs}))
    return path


def test_unknown_values_do_not_become_measurements():
    data = load_calibration(Path(__file__).resolve().parents[1] / "config/robot_measurements.example.json")
    assert data["measured_mass_kg"] == data["print_mass_g"] == data["servo"] == {}
    assert "head_shift_m" not in data


@pytest.mark.parametrize("field,value", [
    ("measured_mass_kg", {"trunk_base": 0}),
    ("measured_mass_kg", {"trunk_base": -1}),
    ("measured_mass_kg", {"trunk_base": True}),
    ("measured_mass_kg", {"typo": 1}),
    ("print_mass_g", {"head_upper_v4": float("nan")}),
    ("print_mass_g", {"hat_rear_shield_v3": 30}),          # the old shield is not a V4 part
    ("servo", {"kp": 0}), ("servo", {"stall": float("inf")}),
    ("shield_pos_m", [-0.05, 0, 0.02]), ("head_shift_m", [0, True, 0]),
    ("whole_robot_mass_kg", -1), ("unknown", 42),
])
def test_bad_inputs_fail_before_regeneration(tmp_path, field, value):
    with pytest.raises(ValueError):
        load_calibration(write(tmp_path, **{field: value}))


def test_valid_units_and_provenance(tmp_path):
    path = write(tmp_path, measured_mass_kg={"jaw_soft": 0.125},
                 servo={"kp": 0.8, "kv": 0, "delay_s": 0.03},
                 head_shift_m=[0.0, 0.0, 0.001])
    data = load_calibration(path)
    assert data["measured_mass_kg"]["jaw_soft"] == 0.125
    assert data["servo"]["kv"] == 0
    assert len(data["source_sha256"]) == 64
