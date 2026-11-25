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


def test_lipo_replaces_the_upstream_battery_at_its_v4_location(built):
    root, _, base, _ = built
    ledger = json.loads((root / "model/mass_ledger.json").read_text())
    torso, lipo = ledger["parts"]["_torso_link"], ledger["parts"]["battery_v4"]
    assert torso["removed_upstream_parts_g_ESTIMATE"]["np_f970"] == ledger["upstream_battery_removed_g_ESTIMATE"]
    params = json.loads((root / "design/v4/v4_params.json").read_text())["battery"]
    assert lipo["mass_g"] == params["mass_g"]
    np.testing.assert_allclose(lipo["com_mm"], (np.array(params["lo"]) + np.array(params["hi"])) / 2, atol=0.6)
    added = sum(ledger["parts"][n]["mass_g"] for n in ("body_shell_v4", "body_lid_v4", "keel_cover_v4", "fasteners_body_v4",
                                                     "pcb_stack_v4", "a010_v4", "xt60_v4", "battery_v4"))
    assert torso["new_mass_g"] == pytest.approx(torso["remainder_g"] + added, abs=0.2)
    assert base.body("trunk_base").mass[0] * 1000 == pytest.approx(torso["new_mass_g"], abs=0.1)
    # lighter (55 g pack for a ~100 g upstream battery) and the torso COM moves back towards the pack
    assert torso["new_mass_g"] < torso["upstream_mass_g"] + added - 100
    assert torso["com_mm"][0] < torso["upstream_com_mm"][0]


def test_servo_delay_is_carried_by_the_model(built):
    root, _, base, measured = built
    sys.path.insert(0, str(ROOT / "ros2/duck_nav"))
    from duck_nav.actuation import model_servo_delay
    assert model_servo_delay(base) == pytest.approx(.02)        # ESTIMATE: middle of the upstream 15-30 ms lag
    assert model_servo_delay(measured) == pytest.approx(.04)    # the calibration's measured delay
    ledger = json.loads((root / "model/mass_ledger.json").read_text())
    assert ledger["servo_delay_sim_s"] == .02 and ledger["servo_delay_s"] is None
    assert json.loads((root / "model/mass_ledger_measured.json").read_text())["servo_delay_source"].startswith("calibration")


def test_delayed_targets_reach_the_joints_later(built):
    mujoco = pytest.importorskip("mujoco")
    root, _, base, _ = built
    sys.path.insert(0, str(ROOT / "ros2/duck_nav"))
    from duck_nav.actuation import ServoDelay, model_servo_delay
    m = mujoco.MjModel.from_xml_path(str(root / "model/duck_v4_scene.xml")); m.opt.timestep = 0.005
    key = m.key("sim_start").id
    knee = m.jnt_qposadr[m.joint("left_knee").id]
    paths = {}
    for name, delay, step in (("hold", 0.0, 0.0), ("delayed", model_servo_delay(m), 0.3)):
        d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, key)
        target = m.key_ctrl[key][:14].copy()
        s = ServoDelay(delay, m.opt.timestep, target)
        target[3] += step                                       # left knee target steps by 0.3 rad at t = 0
        path = []
        for _ in range(12):
            d.ctrl[:14] = s.step(target); mujoco.mj_step(m, d); path.append(d.qpos[knee])
        paths[name] = np.array(path)
    # with a 20 ms (4 physics steps) delay the knee follows the unchanged target for 4 steps, then departs from it
    np.testing.assert_array_equal(paths["delayed"][:4], paths["hold"][:4])
    assert abs(paths["delayed"][-1] - paths["hold"][-1]) > 1e-3


def test_print_settings_come_from_v4_params(built):
    root, run, _, _ = built
    path = root / "design/v4/v4_params.json"
    original = path.read_text()
    try:
        params = json.loads(original)
        params["print"] = {"default": {"wall_mm": 1.2, "infill": .3}, "head_upper_v4": {"wall_mm": .8, "infill": .15}}
        path.write_text(json.dumps(params))
        run("make_duck.py", "--tag", "thin")
    finally:
        path.write_text(original)
    thin = json.loads((root / "model/mass_ledger_thin.json").read_text())["parts"]
    base = json.loads((root / "model/mass_ledger.json").read_text())["parts"]
    u = thin["head_upper_v4"]
    vw = min(u["volume_mm3"], u["area_mm2"] * .8)
    assert u["mass_g"] == pytest.approx((vw + .15 * (u["volume_mm3"] - vw)) * .00124, abs=.01)
    assert (u["wall_mm"], u["infill"]) == (.8, .15)
    assert u["mass_g"] < base["head_upper_v4"]["mass_g"] - 10
    assert thin["head_lower_v4"]["mass_g"] == base["head_lower_v4"]["mass_g"]


# The one static contact left (design/v4/README.md): the upstream motor support seated on the lower shell's ledge.
EXPLAINED_STATIC = {"jaw_soft/head_lower_v4 vs jaw_soft/motor_support"}


def test_fit_check_static_fit_is_clean_and_the_jaw_clears_over_its_range(built):
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("trimesh")
    root, _, _, _ = built
    sys.path.insert(0, str(ROOT / "design/v4"))
    import check_v4_fit as fit
    m = mujoco.MjModel.from_xml_path(str(root / "model/duck_v4_scene.xml")); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("sim_start").id); mujoco.mj_forward(m, d)
    params = json.loads((root / "design/v4/v4_params.json").read_text())
    parts = fit.build_parts(m, params, root / "model/duck_v4.xml")
    static = fit.static_fit(m, d, parts, .8)
    # the pairs the review reported (lens 6.7, mouth servo / lens holder 6.1, head-roll servo 5.1, neck bearing 3.7 mm)
    # are still measured, now clear
    for pair in ("jaw_soft/head_upper_v4 vs jaw_soft/lens", "jaw_soft/head_lower_v4 vs yaw_roll_motion/xl330",
                 "jaw_soft/head_lower_v4 vs neck_pitch/seeed_bearing__configuration__22x16x4"):
        assert static.get(pair, {"new_mm": 0})["new_mm"] <= .8, pair
    assert {k for k, v in static.items() if v["clash"]} <= EXPLAINED_STATIC
    # every connector / cable envelope and the new body parts are in the check
    names = {p.name.split("#")[0] for p in parts}
    for env in params["envelopes"]:
        assert "envelope/" + env["name"] in names
    assert {"trunk_base/body_lid_v4", "trunk_base/keel_cover_v4"} <= names
    # jaw sweep: no jaw / head contact deeper than 0.5 mm anywhere in 0..open_deg
    jaw = [p for p in parts if p.name.startswith("jaw/")]
    head = [p for p in parts if p.name.startswith("jaw_soft/head_")]
    qa = m.jnt_qposadr[m.joint("mouth").id]
    for deg in np.linspace(0, params["head"]["jaw"]["open_deg"], 7):
        d.qpos[qa] = np.radians(deg); mujoco.mj_kinematics(m, d)
        T = fit.transforms(d)
        worst = max(fit.pair_depth(a, b, T[a.body], T[b.body]) for a in jaw for b in head)
        assert worst * 1000 < .5, (deg, worst)
