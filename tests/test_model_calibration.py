"""Build real MJCF from upstream assets; skip only when those assets are absent."""
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = Path(os.environ.get("DUCKLING_UPSTREAM", ROOT / "microduck_rl/src/mjlab_microduck/robot/microduck"))


@pytest.fixture(scope="module")
def built(tmp_path_factory):
    mujoco = pytest.importorskip("mujoco")
    if not (UPSTREAM / "robot_walk.xml").exists():
        pytest.skip("clone the pinned microduck_rl model to run model integration tests")
    root = tmp_path_factory.mktemp("model")
    # Copy rather than symlink: Windows symlinks can require elevated rights.
    shutil.copytree(ROOT / "design/v4", root / "design/v4", ignore=shutil.ignore_patterns("preview", "sim_runs", "*.py"))
    (root / "scripts").mkdir()
    (root / "scripts/collision_proxies.json").write_bytes((ROOT / "scripts/collision_proxies.json").read_bytes())
    env = {**os.environ, "DUCKLING_ROOT": str(root), "DUCKLING_UPSTREAM": str(UPSTREAM)}

    def run(script, *args, check=True):
        return subprocess.run([sys.executable, str(ROOT / "scripts" / script), *map(str, args)],
                              env=env, text=True, capture_output=True, check=check)

    run("prep_meshes.py")
    run("make_duck.py")
    baseline = mujoco.MjModel.from_xml_path(str(root / "model/duck_v4_scene.xml"))
    weights = root / "weights.json"
    weights.write_text(json.dumps({"schema_version": 1,
        "measured_mass_kg": {"jaw_soft": .125, "trunk_base": .250, "neck": .025},
        "print_mass_g": {"head_upper_v4": 50, "head_lower_v4": 40, "jaw_v4": 4, "body_shell_v4": 70},
        "servo": {"kp": .8, "kv": .02, "stall": .6, "delay_s": .04},
        "whole_robot_mass_kg": .8}))
    run("make_duck.py", "--tag", "measured", "--calibration", weights)
    measured = mujoco.MjModel.from_xml_path(str(root / "model/duck_v4_measured_scene.xml"))
    return root, run, baseline, measured


def test_weighed_links_replace_mass_and_shells_are_not_added_twice(built):
    root, _, base, measured = built
    assert measured.body("jaw_soft").mass[0] == pytest.approx(.125)
    assert measured.body("trunk_base").mass[0] == pytest.approx(.250)  # weighed torso includes the body and its contents
    assert measured.body("neck").mass[0] == pytest.approx(.025)
    changed = {"jaw_soft", "trunk_base", "neck", "jaw"}      # jaw: its print mass is in the weights file
    for i in range(base.nbody):
        if base.body(i).name not in changed:
            assert measured.body_mass[i] == base.body_mass[i]
    np.testing.assert_allclose(measured.actuator_gainprm[:14, 0], .8)
    np.testing.assert_allclose(measured.actuator_forcerange[:14], [[-.6, .6]] * 14)
    ledger = json.loads((root / "model/mass_ledger_measured.json").read_text())
    assert ledger["parts"]["head_upper_v4"]["included_in_measured_head"]
    assert ledger["parts"]["body_shell_v4"]["included_in_measured_torso"]
    assert ledger["servo_delay_s"] == .04


def test_stand_numbers_uses_tagged_ledger_and_reports_scale_disagreement(built):
    root, run, _, _ = built
    output = root / "numbers.json"
    result = run("stand_numbers.py", root / "model/duck_v4_measured_scene.xml",
                 "--output", output, "--strict-mass", check=False)
    numbers = json.loads(output.read_text())
    assert numbers["servo_delay_s"] == .04
    assert numbers["printed_part_mass_g"]["head_upper_v4"] == 50
    assert numbers["mass_error_fraction"] == pytest.approx(abs(numbers["mass"] - .8) / .8)
    assert result.returncode == (0 if numbers["mass_within_5_percent"] else 1)
    assert len(numbers["calibration_sha256"]) == 64


def test_stale_ledger_is_rejected(built):
    root, run, _, _ = built
    path = root / "model/duck_v4_measured.xml"
    original = path.read_bytes()
    try:
        path.write_bytes(original + b"\n")
        result = run("stand_numbers.py", root / "model/duck_v4_measured_scene.xml", check=False)
        assert result.returncode == 1
        assert "does not match" in result.stderr
    finally:
        path.write_bytes(original)


def test_contact_gate_rejects_feet_only_model(built):
    root, run, _, _ = built
    result = run("check_contacts.py", "--scene", root / "model/duck_v4_scene.xml",
                 "--samples", 1, check=False)
    assert result.returncode == 2
    assert "needs the ground-contact model" in result.stderr
