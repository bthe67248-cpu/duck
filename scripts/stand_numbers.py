"""stand_numbers.py -- settle the duck in its STAND pose and print the numbers MATLAB needs.

Usage:  .venv\\Scripts\\python.exe scripts\\stand_numbers.py model\\duck_v4_scene.xml
Writes model/stand_numbers.json (read by matlab/duck_numbers.m).
"""
import argparse, hashlib, json, sys
from pathlib import Path
import mujoco
import numpy as np

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("scene", type=Path)
ap.add_argument("--output", type=Path, help="default: model/stand_numbers.json alongside the input scene")
ap.add_argument("--strict-mass", action="store_true", help="fail unless a supplied whole-robot weighing agrees within 5%%")
args = ap.parse_args()
path = args.scene
m = mujoco.MjModel.from_xml_path(str(path))
d = mujoco.MjData(m)
kid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "STAND")
if kid < 0:
    raise SystemExit("scene has no STAND keyframe")
mujoco.mj_resetDataKeyframe(m, d, kid)
home = m.key_ctrl[kid].copy()
mujoco.mj_forward(m, d)                   # nominal STAND pose, before anything sags

torso = m.body("trunk_base").id
com = d.subtree_com[torso].copy()
mass = float(m.body_subtreemass[torso])
pitch = 0.0

# ankle pivots (world) and the sole outline: lowest vertices of the two foot collision meshes
ank = {s: d.xanchor[m.joint(f"{s}_ankle").id].copy() for s in ("left", "right")}
pts = []
for s in ("left", "right"):
    g = m.geom(f"{s}_foot_collision").id
    mid = m.geom_dataid[g]
    v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
    w = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
    pts.append(w[w[:, 2] < w[:, 2].min() + 0.001])
pts = np.vstack(pts)
floor_z = float(pts[:, 2].min())
ankle_x = float(np.mean([ank["left"][0], ank["right"][0]]))
ankle_z = float(np.mean([ank["left"][2], ank["right"][2]])) - floor_z
com[2] -= floor_z
out = dict(
    timestep=m.opt.timestep, mass=mass, com=com.tolist(), settled_pitch_rad=pitch,
    torso_z=float(d.xpos[torso][2]),
    ankle_x=ankle_x, ankle_z=ankle_z, ankle_y=[float(ank["left"][1]), float(ank["right"][1])],
    h=float(com[2] - ankle_z),                       # CoM height above the ankle pivot
    com_height_floor=float(com[2]),
    toe_x=float(pts[:, 0].max()), heel_x=float(pts[:, 0].min()),
    a_toe=float(pts[:, 0].max() - com[0]),           # CoM -> front edge of support
    a_heel=float(com[0] - pts[:, 0].min()),          # CoM -> rear edge of support
    half_width=float((pts[:, 1].max() - pts[:, 1].min()) / 2),
    knee_x=float(d.xanchor[m.joint("left_knee").id][0]),
    kp=float(m.actuator_gainprm[0, 0]), stall=float(m.actuator_forcerange[0, 1]),
    home=home[:14].tolist(),
    actuators=[m.actuator(i).name for i in range(m.nu)],
)
# provenance (S9): which model revision these numbers describe and how provisional the printed masses are
tag = path.stem.removeprefix("duck_v4").removesuffix("_scene")
ledger_file = path.parent / f"mass_ledger{tag}.json"
out["model_scene"] = path.name
out["model_scene_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()[:16]
robot_xml = path.parent / path.name.replace("_scene", "")
if robot_xml.exists():
    out["model_robot_sha256"] = hashlib.sha256(robot_xml.read_bytes()).hexdigest()[:16]
if ledger_file.exists():
    led = json.loads(ledger_file.read_text())
    ledger_hash = led.get("model_robot_sha256")
    if ledger_hash and (not robot_xml.exists() or hashlib.sha256(robot_xml.read_bytes()).hexdigest() != ledger_hash):
        raise SystemExit(f"{ledger_file.name} does not match {robot_xml.name}; regenerate the model and ledger together")
    out["mass_status"] = led["status"]
    out["printed_part_mass_g"] = {k: v["mass_g"] for k, v in led["parts"].items() if not k.startswith("_")}
    out["servo_delay_s"] = led.get("servo_delay_s")
    out["calibration_sha256"] = led.get("calibration_sha256")
    out["mass_ledger_sha256"] = hashlib.sha256(ledger_file.read_bytes()).hexdigest()
    measured = led.get("whole_robot_mass_kg")
    if measured is not None:
        out["measured_robot_mass_kg"] = measured
        out["mass_error_fraction"] = abs(mass - measured) / measured
        out["mass_within_5_percent"] = out["mass_error_fraction"] <= 0.05
out["d_knee"] = abs(out["knee_x"] - out["com"][0])
for k, v in out.items():
    print(f"{k:18s} {v}")
destination = args.output or path.parent / "stand_numbers.json"
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_text(json.dumps(out, indent=1))
if args.strict_mass and not out.get("mass_within_5_percent", False):
    raise SystemExit("mass gate FAIL: supply a whole_robot_mass_kg measurement and get model error <= 5%")
