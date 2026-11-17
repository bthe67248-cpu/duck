r"""make_duck_v3.py -- build model/duck_v3.xml (+ scene) from the upstream Microduck model.

Usage:  .venv\\Scripts\\python.exe scripts\\make_duck_v3.py

Edit ONLY the block below with your own scale/ruler measurements, then re-run.
Run scripts\prep_v3_meshes.py once first (it prepares the V3 head and shield meshes).
Everything marked ESTIMATE is a placeholder, not a measurement.
"""
import copy, json, shutil, sys
import mujoco
import numpy as np
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "microduck_rl" / "src" / "mjlab_microduck" / "robot" / "microduck"
DST = ROOT / "model"

# ----------------------------------------------------------------- EDIT THIS BLOCK
# Weighed link masses in kg (servos + screws included). None = keep upstream value.
MEASURED_MASS = {
    "trunk_base": None, "jaw_soft": None,           # torso, head
    "upper_leg_left": None, "upper_leg_right": None,
    "leg": None, "leg_2": None,                      # shanks
    "ankle_left": None, "ankle_right": None,         # feet
}
# V3 printed parts (meshes prepared by scripts/prep_v3_meshes.py -> model/v3_meshes.json).
# Mass of each part: weigh it and type the value in kg. None = ESTIMATE from mesh volume,
# volume x PLA_DENSITY x PRINT_FILL (the README warns this is only a rough guess).
PLA_DENSITY = 1240.0      # kg/m3
PRINT_FILL = 0.9          # thin shells print almost solid
V3_HEAD_UPPER_MASS = None  # kg, weighed duck_head_upper_v3
V3_HEAD_LOWER_MASS = None  # kg, weighed duck_head_lower_v3
V3_SHIELD_MASS = None      # kg, weighed rear shield
SHIELD_POS = "-0.050 0 0.020"   # shield centre in the torso frame, m (x back, z up) -- measure it
HEAD_SHIFT = (0.0, 0.0, 0.0)    # nudge the V3 head (forward, left, up) in m if it sits wrong
SERVO = dict(kp=None, kv=None, stall=None)   # from your bench test (Part 5); None = upstream
# ----------------------------------------------------------------- END OF EDIT BLOCK

# Optional robustness variants:  make_duck_v3.py --torso-scale 1.1 --tag mplus
import argparse
_ap = argparse.ArgumentParser()
_ap.add_argument("--torso-scale", type=float, default=1.0)
_ap.add_argument("--tag", default="")
_ap.add_argument("--base", default="robot_walk.xml",
                 help="upstream robot file: robot_walk.xml (feet-only contact) or robot_groundcontact.xml (body can lie on the floor)")
_args = _ap.parse_args()
OUT_ROBOT = f"duck_v3{'_' + _args.tag if _args.tag else ''}.xml"
OUT_SCENE = f"duck_v3{'_' + _args.tag if _args.tag else ''}_scene.xml"

JOINTS = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
          "neck_pitch", "head_pitch", "head_yaw", "head_roll",
          "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"]


def main():
    if not DST.exists():
        shutil.copytree(SRC, DST)
    tree = ET.parse(SRC / _args.base)
    root = tree.getroot()
    root.set("model", "duck_v3")
    bodies = {b.get("name"): b for b in root.iter("body")}

    # 1. weighed masses, inertia scaled by the same ratio
    for name, mass in MEASURED_MASS.items():
        if mass is None:
            continue
        inert = bodies[name].find("inertial")
        ratio = mass / float(inert.get("mass"))
        inert.set("mass", f"{mass:.6f}")
        key = "fullinertia" if inert.get("fullinertia") else "diaginertia"
        inert.set(key, " ".join(f"{float(v) * ratio:.6g}" for v in inert.get(key).split()))

    if _args.torso_scale != 1.0:                       # mass-error test (Part 8, test 6)
        inert = bodies["trunk_base"].find("inertial")
        inert.set("mass", f"{float(inert.get('mass')) * _args.torso_scale:.6f}")
        inert.set("fullinertia", " ".join(f"{float(v) * _args.torso_scale:.6g}"
                                          for v in inert.get("fullinertia").split()))

    # 2. V3 printed parts: redesigned head shells replace the upstream ones, rear shield added.
    #    Mass goes in welded child bodies (a bare geom mass is ignored when the parent has <inertial>).
    v3 = json.loads((DST / "v3_meshes.json").read_text())
    asset = root.find("asset")
    for n in v3:
        ET.SubElement(asset, "mesh", name=n, file=f"{n}.stl")

    def est(name, weighed, minus_original=False):
        if weighed is not None:
            return weighed, "weighed"
        vol = v3[name]["volume_mm3"] - (v3[name].get("orig_volume_mm3", 0.0) if minus_original else 0.0)
        return vol * 1e-9 * PLA_DENSITY * PRINT_FILL, "ESTIMATE from mesh volume"

    def add_mass(parent, name, pos, mass, half=0.015):
        body = ET.SubElement(bodies[parent], "body", name=name, pos=" ".join(f"{v:.5f}" for v in pos))
        i = mass * 2 / 3 * half * half
        ET.SubElement(body, "inertial", pos="0 0 0", mass=f"{mass:.5f}", diaginertia=f"{i:.4g} {i:.4g} {i:.4g}")

    # head: where the upstream shells sit (world frame, STAND pose, head level, +x forward)
    um = mujoco.MjModel.from_xml_path(str(SRC / "scene_walk.xml")); ud = mujoco.MjData(um)
    mujoco.mj_resetDataKeyframe(um, ud, mujoco.mj_name2id(um, mujoco.mjtObj.mjOBJ_KEY, "STAND"))
    mujoco.mj_forward(um, ud)
    hb = um.body("jaw_soft").id
    Rb, pb = ud.xmat[hb].reshape(3, 3), ud.xpos[hb]

    def world_bounds(mesh_name):
        for g in range(um.ngeom):
            if um.geom_bodyid[g] == hb and um.geom_dataid[g] >= 0 and um.mesh(um.geom_dataid[g]).name == mesh_name:
                mid = um.geom_dataid[g]
                v = um.mesh_vert[um.mesh_vertadr[mid]: um.mesh_vertadr[mid] + um.mesh_vertnum[mid]]
                w = v @ ud.geom_xmat[g].reshape(3, 3).T + ud.geom_xpos[g]
                return w.min(0), w.max(0)
    top_lo, top_hi = world_bounds("top_head_shell")
    bot_lo, _ = world_bounds("bottom_head_shell")
    # V3 mesh frame (mm): x = width, -y = front, z = up.  world = Rz(+90 deg) * mesh + t
    Rw = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    up, lo = v3["duck_head_upper_v3"], v3["duck_head_lower_v3"]
    mid_wid = (up["min_mm"][0] + up["max_mm"][0]) / 2000.0         # shell centre along world y
    front = -up["min_mm"][1] / 1000.0                             # upper shell origin -> its front edge
    t_lower = np.array([top_hi[0] - front, -mid_wid, bot_lo[2]]) + np.array(HEAD_SHIFT)   # fronts flush
    t_upper = t_lower + np.array([0, 0, lo["max_mm"][2] / 1000.0])  # upper sits on the lower's seam
    head = bodies["jaw_soft"]
    for g in list(head.findall("geom")):                            # remove the upstream shells
        if g.get("mesh") in ("top_head_shell", "bottom_head_shell") and g.get("class") == "visual":
            head.remove(g)
    quat = np.zeros(4); mujoco.mju_mat2Quat(quat, (Rb.T @ Rw).flatten())
    for n, t in (("duck_head_upper_v3", t_upper), ("duck_head_lower_v3", t_lower)):
        ET.SubElement(head, "geom", type="mesh", mesh=n, **{"class": "visual"}, rgba="0.22 0.23 0.25 1",
                      pos=" ".join(f"{v:.5f}" for v in Rb.T @ (t - pb)), quat=" ".join(f"{v:.6f}" for v in quat))
    # head mass: upstream head already includes its own shells, so add only the EXTRA plastic
    for n, t, weighed in (("duck_head_upper_v3", t_upper, V3_HEAD_UPPER_MASS), ("duck_head_lower_v3", t_lower, V3_HEAD_LOWER_MASS)):
        if weighed is not None:      # weighed V3 shell minus the estimated original shell
            extra = weighed - v3[n]["orig_volume_mm3"] * 1e-9 * PLA_DENSITY * PRINT_FILL; how = "weighed - original (est.)"
        else:
            extra, how = est(n, None, minus_original=True)
        c_world = Rw @ (np.array(v3[n]["centroid_mm"]) / 1000.0) + t
        add_mass("jaw_soft", "v3_extra_" + n, Rb.T @ (c_world - pb), max(extra, 1e-4))
        print(f"  {n}: +{extra*1000:.1f} g on the head ({how})")

    # head camera (RGB + depth come from this one MuJoCo camera): at the upstream camera site,
    # looking forward and level in the STAND pose. MuJoCo cameras look along their -z, +y is up.
    cam_w = ud.site("head_camera").xpos + np.array([0.012, 0.0, 0.0])      # just in front of the lens
    Rc_w = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # columns: cam x,y,z in world
    quat = np.zeros(4); mujoco.mju_mat2Quat(quat, (Rb.T @ Rc_w).flatten())
    if "groundcontact" in _args.base:      # only in the motions/SLAM model: a camera in the XML makes the
        # Simulink plant block grow RGB/depth output ports, which duck_sim.slx does not use
        ET.SubElement(head, "camera", name="head_cam", fovy="60", pos=" ".join(f"{v:.5f}" for v in Rb.T @ (cam_w - pb)),
                      quat=" ".join(f"{v:.6f}" for v in quat))

    # rear shield on the torso: mesh x -> -y, mesh y -> up, mesh z -> backwards (solid face at the rear)
    m_sh, how = est("hat_rear_shield_v3", V3_SHIELD_MASS)
    sp = np.array([float(v) for v in SHIELD_POS.split()])
    Rs = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    sh = v3["hat_rear_shield_v3"]
    c_mesh = (np.array(sh["min_mm"]) + np.array(sh["max_mm"])) / 2000.0
    quat = np.zeros(4); mujoco.mju_mat2Quat(quat, Rs.flatten())
    ET.SubElement(bodies["trunk_base"], "geom", type="mesh", mesh="hat_rear_shield_v3", **{"class": "visual"},
                  rgba="0.22 0.23 0.25 1", pos=" ".join(f"{v:.5f}" for v in sp - Rs @ c_mesh),
                  quat=" ".join(f"{v:.6f}" for v in quat))
    add_mass("trunk_base", "v3_shield", sp - Rs @ c_mesh + Rs @ (np.array(sh["centroid_mm"]) / 1000.0), m_sh, half=0.03)
    print(f"  rear shield: {m_sh*1000:.1f} g on the torso ({how})")

    # 3. push point on the torso + force motors (placed LAST in the actuator list)
    ET.SubElement(bodies["trunk_base"], "site", name="push_point", pos="0 0 0")
    act = root.find("actuator")
    if any(v is not None for v in SERVO.values()):
        for p in act.findall("position"):
            if SERVO["kp"] is not None: p.set("kp", str(SERVO["kp"]))
            if SERVO["kv"] is not None: p.set("kv", str(SERVO["kv"]))
            if SERVO["stall"] is not None: p.set("forcerange", f"-{SERVO['stall']} {SERVO['stall']}")
    ET.SubElement(act, "motor", name="push_x", site="push_point", gear="1 0 0 0 0 0")
    ET.SubElement(act, "motor", name="push_y", site="push_point", gear="0 1 0 0 0 0")

    # 4. sensors with Simulink-safe names (no hyphens), fixed order
    for s in root.findall("sensor"):
        root.remove(s)
    sen = ET.SubElement(root, "sensor")
    ET.SubElement(sen, "gyro", name="gyro", site="imu")
    ET.SubElement(sen, "accelerometer", name="accel", site="imu")
    ET.SubElement(sen, "framequat", name="quat", objtype="site", objname="imu")
    ET.SubElement(sen, "framepos", name="torso_pos", objtype="site", objname="imu")
    ET.SubElement(sen, "framelinvel", name="torso_vel", objtype="site", objname="imu")
    for j in JOINTS:
        ET.SubElement(sen, "jointpos", name="q_" + j, joint=j)

    ET.indent(tree, "  ")
    tree.write(DST / OUT_ROBOT)

    # 5. scene: floor + light + keyframes (ctrl padded with 2 zeros for the push motors)
    scene = ET.parse(SRC / "scene_walk.xml")
    sroot = scene.getroot()
    sroot.find("include").set("file", OUT_ROBOT)
    for k in sroot.iter("key"):
        k.set("qpos", " ".join(k.get("qpos").split()))
        k.set("ctrl", " ".join(k.get("ctrl").split()) + " 0 0")
    # start pose for Simulink (patched blockset starts from the key named "sim_start"):
    # STAND with the torso lowered 2.8 mm so the soles just touch the floor
    stand = next(k for k in sroot.iter("key") if k.get("name") == "STAND")
    q = stand.get("qpos").split(); q[2] = "0.1172"
    ET.SubElement(sroot.find("keyframe"), "key", name="sim_start", qpos=" ".join(q),
                  ctrl=stand.get("ctrl"))
    ET.indent(scene, "  ")
    scene.write(DST / OUT_SCENE)
    print("wrote", DST / OUT_ROBOT, "and", DST / OUT_SCENE)


if __name__ == "__main__":
    sys.exit(main())
