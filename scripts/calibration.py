"""Validate measured model inputs without importing MuJoCo or modifying source code.

Null means unknown, never zero. Link weights include everything rigidly attached
to that link; trunk_base includes the V4 body shell and everything inside it; jaw_soft includes
both V4 head shells and the mouth servo, but not the jaw. Weights alone do not measure a centre of mass or inertia tensor.
"""
import hashlib
import json
import math
from pathlib import Path

LINKS = (
    "trunk_base", "yaw2roll", "hip_l", "upper_leg_left", "leg", "ankle_left",
    "neck", "neck_pitch", "yaw_roll_motion", "jaw_soft", "bearing_roll",
    "hip_l_2", "upper_leg_right", "leg_2", "ankle_right",
)
PRINTS = ("head_upper_v4", "head_lower_v4", "jaw_v4", "body_shell_v4", "body_lid_v4", "keel_cover_v4")


def number(value, name, *, zero=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}: expected a finite number")
    if not math.isfinite(value) or (value < 0 if zero else value <= 0):
        raise ValueError(f"{name}: expected a finite {'nonnegative' if zero else 'positive'} number")
    return float(value)


def load_calibration(path):
    path = Path(path)
    raw = path.read_bytes()
    data = json.loads(raw)
    if not isinstance(data, dict):
        raise ValueError("calibration must be a JSON object")
    allowed = {"schema_version", "notes", "measured_mass_kg", "print_mass_g",
               "head_shift_m", "servo", "whole_robot_mass_kg"}
    extra = set(data) - allowed
    if extra:
        raise ValueError(f"unknown calibration fields: {sorted(extra)}")
    if type(data.get("schema_version")) is not int or data["schema_version"] != 1:
        raise ValueError("schema_version must be 1")
    out = {}
    for section, names in (("measured_mass_kg", LINKS), ("print_mass_g", PRINTS),
                           ("servo", ("kp", "kv", "stall", "delay_s"))):
        values = data.get(section, {})
        if not isinstance(values, dict) or set(values) - set(names):
            raise ValueError(f"{section}: expected an object containing only {names}")
        out[section] = {key: number(value, f"{section}.{key}", zero=key in ("kv", "delay_s"))
                        for key, value in values.items() if value is not None}
    for section in ("head_shift_m",):
        value = data.get(section)
        if value is None:
            continue
        if (not isinstance(value, list) or len(value) != 3 or any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value)):
            raise ValueError(f"{section}: expected three finite coordinates in metres")
        out[section] = value
    mass = data.get("whole_robot_mass_kg")
    out["whole_robot_mass_kg"] = None if mass is None else number(mass, "whole_robot_mass_kg")
    out["source_sha256"] = hashlib.sha256(raw).hexdigest()
    return out
