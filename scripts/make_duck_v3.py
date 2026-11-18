r"""make_duck_v3.py -- build model/duck_v3*.xml (+ scene) from the upstream Microduck model.

Usage:  .venv\Scripts\python.exe scripts\make_duck_v3.py [--base robot_groundcontact.xml --tag gc]
Run scripts\prep_v3_meshes.py once first (it prepares the V3 head and shield meshes from design/).

Roots are derived from this file's location. Overrides: $DUCKLING_ROOT (repo root), $DUCKLING_UPSTREAM
(folder holding robot_walk.xml, scene_walk.xml, assets/ ...; default microduck_rl/.../robot/microduck).

Edit ONLY the block below with your own scale/ruler measurements, then re-run.
Everything marked ESTIMATE / PROVISIONAL is a placeholder, not a measurement.
"""
import argparse, json, os, shutil, sys
import mujoco
import numpy as np
import trimesh
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
SRC = Path(os.environ.get("DUCKLING_UPSTREAM",
                          ROOT / "microduck_rl" / "src" / "mjlab_microduck" / "robot" / "microduck"))
DST = ROOT / "model"

# ----------------------------------------------------------------- EDIT THIS BLOCK
# Weighed COMPLETE-LINK masses in kg (servos + screws included). None = keep upstream value.
# Convention: a weighed link replaces the whole link. For "jaw_soft" (the head link) the weighed value means the
# head WITH the V3 shells installed, so the V3 composite step below is skipped for it (never counted twice).
# For "trunk_base" it means the torso WITHOUT the V3 rear shield (the shield is added on top).
MEASURED_MASS = {
    "trunk_base": None, "jaw_soft": None,           # torso, head
    "upper_leg_left": None, "upper_leg_right": None,
    "leg": None, "leg_2": None,                      # shanks
    "ankle_left": None, "ankle_right": None,         # feet
}
# ---- S9: per-part PRINTED masses (grams), replacing the old global PRINT_FILL=0.9 volume heuristic.
# None -> use PROVISIONAL_30PCT_G below.  A number here (from the slicer, then the scale) overrides it.
V3_PART_MASS_G = {"duck_head_upper_v3": None, "duck_head_lower_v3": None, "hat_rear_shield_v3": None}
# PROVISIONAL: 30 % gyroid, 3 perimeters, 1.24 g/cm3 black PLA, no supports -- handoff section 2 proxy:
#   V_wall = min(V, A*1.2 mm); V_print = V_wall + 0.30 (V - V_wall); m = V_print * 0.00124 g/mm3.
# Upper head and shield use the handoff's table; lower head is recomputed from the NEW bill mesh (design/mesh_checks.json).
PROVISIONAL_30PCT_G = {"duck_head_upper_v3": 51.5, "duck_head_lower_v3": "formula", "hat_rear_shield_v3": 32.3}
PLA_DENSITY_G_MM3 = 0.00124
INFILL, WALL_MM = 0.30, 1.2
# Upstream head parts that the V3 head REPLACES (their mass/inertia is subtracted from the upstream head link).
# ESTIMATE: Onshape part mass is not in the repo, so mesh volume x 1240 kg/m3 (solid) is assumed.
UPSTREAM_PART_DENSITY = 1240.0
SHIELD_POS = "-0.050 0 0.020"   # shield centre in the torso frame, m (x back, z up) -- measure it
HEAD_SHIFT = (0.0, 0.0, 0.0)    # nudge the V3 head (forward, left, up) in m if it sits wrong
SERVO = dict(kp=None, kv=None, stall=None)   # from your bench test (Part 5); None = upstream
# ----------------------------------------------------------------- END OF EDIT BLOCK

# A1: black PLA for visible printed rigid parts. Servos, bearings, battery, PCBs, speaker, lens/holder and the TPU
# soles keep their own look so optics / metal / electronics stay recognisable.
BLACK_PLA = dict(name="black_pla", rgba="0.025 0.025 0.025 1", specular="0.35", shininess="0.30")
LENS_GLASS = dict(name="lens_glass", rgba="0.04 0.07 0.12 1", specular="0.9", shininess="0.95")
PRINTED_RIGID_MESHES = {
    "trunk_base", "right_shell", "left_shell", "power_support", "banana_pcb_locker", "motor_support",
    "neck", "neck_pitch", "yaw2roll", "yaw_roll_motion", "bearing_roll", "hip_l",
    "upper_leg_left", "upper_leg_right", "upper_leg_rigidity_plate", "leg",
    "ankle_left", "ankle_right", "foot_left", "foot_right", "face_part", "noenoeil",
    "duck_head_upper_v3", "duck_head_lower_v3", "hat_rear_shield_v3"}
LENS_MESHES = {"lens"}
# A2/S1: upstream mouth visuals/collisions that would form a second mouth over the V3 bill.
REMOVED_VISUAL_MESHES = {"top_head_shell", "bottom_head_shell", "jaw", "jaw_soft", "soft_mouth_top"}
REMOVED_COLLISION_MESHES = {"top_head_shell", "bottom_head_shell", "jaw"}   # obsolete head-shell collision

_ap = argparse.ArgumentParser()
_ap.add_argument("--torso-scale", type=float, default=1.0)
_ap.add_argument("--tag", default="")
_ap.add_argument("--base", default="robot_walk.xml",
                 help="upstream robot file: robot_walk.xml (feet-only contact) or robot_groundcontact.xml (body can lie on the floor)")
_args = _ap.parse_args()
OUT_ROBOT = f"duck_v3{'_' + _args.tag if _args.tag else ''}.xml"
OUT_SCENE = f"duck_v3{'_' + _args.tag if _args.tag else ''}_scene.xml"
GC = "groundcontact" in _args.base

JOINTS = ["left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
          "neck_pitch", "head_pitch", "head_yaw", "head_roll",
          "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle"]


def sync_upstream():
    """Copy upstream files that are missing from model/ (never overwrite what is there, e.g. generated XML)."""
    if not SRC.exists():
        raise SystemExit(f"upstream model folder not found: {SRC}  (clone microduck_rl or set DUCKLING_UPSTREAM)")
    DST.mkdir(parents=True, exist_ok=True)
    (DST / "assets").mkdir(exist_ok=True)
    n = 0
    for f in SRC.rglob("*"):
        if f.is_file():
            t = DST / f.relative_to(SRC)
            if not t.exists():
                t.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(f, t); n += 1
    return n


# ----------------------------------------------------------------- rigid-body composite helpers
def quat_of(R):
    q = np.zeros(4); mujoco.mju_mat2Quat(q, np.asarray(R, float).flatten()); return q


def mat_of(q):
    R = np.zeros(9); mujoco.mju_quat2Mat(R, np.asarray(q, float)); return R.reshape(3, 3)


def part_props(mesh_path, R, p, mass=None, density=None):
    """(mass, COM in parent frame, inertia about COM in parent axes) of a uniform-density closed mesh placed with R, p."""
    m = trimesh.load(mesh_path)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p
    m.apply_transform(T)
    vol = float(m.volume)
    if not m.is_watertight or vol <= 0:
        print(f"  WARNING {Path(mesh_path).name}: not a closed mesh, mass properties approximate")
    mass = vol * density if mass is None else mass
    return mass, np.asarray(m.center_mass), np.asarray(m.moment_inertia) * (mass / vol)


def combine(parts):
    """Composite of (m, c, I_about_c) parts. Negative m removes a part. NO clamping: caller checks validity."""
    M = sum(p[0] for p in parts)
    C = sum(p[0] * p[1] for p in parts) / M
    I = np.zeros((3, 3))
    for m, c, Ic in parts:
        d = c - C
        I += Ic + m * (d @ d * np.eye(3) - np.outer(d, d))
    return M, C, I


def check_physical(name, M, I):
    w = np.linalg.eigvalsh(I)
    ok = M > 0 and w.min() > 0 and (w[0] + w[1] >= w[2] * (1 - 1e-9))
    if not ok:
        raise SystemExit(f"{name}: composite is not physical (mass {M}, principal moments {w}); "
                         f"check the part masses instead of clamping")
    return w


def body_inertial(body):
    i = body.find("inertial")
    if i.get("fullinertia"):
        xx, yy, zz, xy, xz, yz = (float(v) for v in i.get("fullinertia").split())
    else:
        xx, yy, zz = (float(v) for v in i.get("diaginertia").split()); xy = xz = yz = 0.0
    if i.get("quat") or i.get("euler"):
        raise SystemExit("inertial with orientation not supported")
    I = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    return float(i.get("mass")), np.array([float(v) for v in i.get("pos", "0 0 0").split()]), I


def set_inertial(body, M, C, I):
    i = body.find("inertial")
    for k in ("diaginertia", "fullinertia", "quat"):
        i.attrib.pop(k, None)
    i.set("pos", " ".join(f"{v:.6g}" for v in C)); i.set("mass", f"{M:.6f}")
    i.set("fullinertia", " ".join(f"{v:.6g}" for v in (I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2])))


def formula_mass_g(vol_mm3, area_mm2):
    vw = min(vol_mm3, area_mm2 * WALL_MM)
    return (vw + INFILL * (vol_mm3 - vw)) * PLA_DENSITY_G_MM3


def part_mass_kg(name, v3, design_checks, ledger):
    wt = V3_PART_MASS_G[name]
    if wt is not None:
        g, how = wt, "WEIGHED/SLICED (user supplied)"
    else:
        prov = PROVISIONAL_30PCT_G[name]
        if prov == "formula":
            chk = design_checks[name + ".stl"]
            g, how = formula_mass_g(chk["volume_mm3"], chk["surface_area_mm2"]), \
                "PROVISIONAL 30% infill wall-proxy formula, recomputed for the current mesh (not a slicer value)"
        else:
            g, how = prov, "PROVISIONAL 30% infill wall-proxy (handoff table; not a slicer value)"
    ledger[name] = dict(mass_g=round(g, 2), status=how, volume_mm3=round(v3[name]["volume_mm3"], 1),
                        area_mm2=round(v3[name].get("area_mm2", float("nan")), 1))
    return g / 1000.0


def main():
    n = sync_upstream()
    if n:
        print(f"  copied {n} missing upstream files into {DST}")
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

    if _args.torso_scale != 1.0:                       # mass-error test (Part 8, test 6): upstream torso only
        inert = bodies["trunk_base"].find("inertial")
        inert.set("mass", f"{float(inert.get('mass')) * _args.torso_scale:.6f}")
        inert.set("fullinertia", " ".join(f"{float(v) * _args.torso_scale:.6g}"
                                          for v in inert.get("fullinertia").split()))

    # 2. V3 printed parts
    if not (DST / "v3_meshes.json").exists():
        raise SystemExit("model/v3_meshes.json missing: run scripts/prep_v3_meshes.py first")
    v3 = json.loads((DST / "v3_meshes.json").read_text())
    design_checks = json.loads((ROOT / "design" / "mesh_checks.json").read_text())
    PROXIES = json.loads((ROOT / "scripts" / "collision_proxies.json").read_text())
    asset = root.find("asset")
    for n_ in v3:
        ET.SubElement(asset, "mesh", name=n_, file=f"{n_}.stl")
    ledger = {}

    # head pose of the upstream shells (world frame, STAND pose, head level, +x forward)
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
    mid_wid = (up["min_mm"][0] + up["max_mm"][0]) / 2000.0
    front = -up["min_mm"][1] / 1000.0
    t_lower = np.array([top_hi[0] - front, -mid_wid, bot_lo[2]]) + np.array(HEAD_SHIFT)   # fronts flush
    t_upper = t_lower + np.array([0, 0, lo["max_mm"][2] / 1000.0])
    head = bodies["jaw_soft"]
    R_mb = Rb.T @ Rw                                               # V3 mesh axes -> head body axes
    quat_mb = quat_of(R_mb)
    place = {"duck_head_upper_v3": Rb.T @ (t_upper - pb), "duck_head_lower_v3": Rb.T @ (t_lower - pb)}

    # A2: drop the upstream shells, jaw, jaw_soft, soft_mouth_top visuals and the obsolete head collision meshes.
    removed_parts = []
    for g in list(head.findall("geom")):
        mname = g.get("mesh")
        if mname in REMOVED_VISUAL_MESHES and g.get("class") == "visual":
            removed_parts.append((mname, np.array([float(v) for v in g.get("pos", "0 0 0").split()]),
                                  np.array([float(v) for v in g.get("quat", "1 0 0 0").split()])))
            head.remove(g)
        elif mname in REMOVED_COLLISION_MESHES and g.get("class") == "collision":
            head.remove(g)
    for g in head.findall("geom"):                                 # nothing obsolete may remain
        assert g.get("mesh") not in REMOVED_COLLISION_MESHES
    for n_ in ("duck_head_upper_v3", "duck_head_lower_v3"):
        ET.SubElement(head, "geom", type="mesh", mesh=n_, **{"class": "visual"}, material="black_pla",
                      pos=" ".join(f"{v:.5f}" for v in place[n_]), quat=" ".join(f"{v:.6f}" for v in quat_mb))

    # ---- head mass: composite recomputed = upstream head - replaced upstream parts + V3 shells (no clamping)
    if MEASURED_MASS["jaw_soft"] is None:
        M0, C0, I0 = body_inertial(head)
        parts = [(M0, C0, I0)]
        removed_g = {}
        for mname, pos, q in removed_parts:
            mm, cc, ii = part_props(DST / "assets" / f"{mname}.stl", mat_of(q), pos, density=UPSTREAM_PART_DENSITY)
            parts.append((-mm, cc, ii)); removed_g[mname] = round(mm * 1000, 1)
        Mr, Cr, Ir = combine(parts)
        check_physical("head without replaced upstream parts", Mr, Ir)
        parts = [(Mr, Cr, Ir)]
        for n_ in ("duck_head_upper_v3", "duck_head_lower_v3"):
            kg = part_mass_kg(n_, v3, design_checks, ledger)
            parts.append(part_props(DST / "assets" / f"{n_}.stl", R_mb, place[n_], mass=kg))
        M1, C1, I1 = combine(parts)
        check_physical("new head link", M1, I1)
        set_inertial(head, M1, C1, I1)
        ledger["_head_link"] = dict(upstream_mass_g=round(M0 * 1000, 1), removed_upstream_parts_g_ESTIMATE=removed_g,
                                    remainder_g=round(Mr * 1000, 1), new_mass_g=round(M1 * 1000, 1),
                                    note="replaced, not added: upstream shells/jaw/soft mouth subtracted, V3 shells added")
        print(f"  head link: {M0*1000:.1f} g upstream - {sum(removed_g.values()):.1f} g replaced parts (est.) "
              f"+ V3 shells -> {M1*1000:.1f} g")
    else:
        print("  head link mass is the WEIGHED value; V3 composite step skipped (weighed value includes the shells)")
        ledger["_head_link"] = dict(new_mass_g=MEASURED_MASS["jaw_soft"] * 1000, note="weighed complete link")

    # mouth_tip: lowest-front point of the new bill (vertex maximising -y -z near the centre line)
    lm = trimesh.load(DST / "assets" / "duck_head_lower_v3.stl").vertices
    cen = lm[np.abs(lm[:, 0] - (lo["min_mm"][0] + lo["max_mm"][0]) / 2000.0) < 0.008]
    tip_mesh = cen[np.argmax(-cen[:, 1] - cen[:, 2])]
    tip_body = R_mb @ tip_mesh + place["duck_head_lower_v3"]
    for s_ in head.findall("site"):
        if s_.get("name") == "mouth_tip":
            s_.set("pos", " ".join(f"{v:.6f}" for v in tip_body))
    print(f"  mouth_tip (bill tip) in head frame: {np.round(tip_body, 4)} m")

    # head camera (RGB + depth come from this one MuJoCo camera): at the upstream camera site
    cam_w = ud.site("head_camera").xpos + np.array([0.012, 0.0, 0.0])
    Rc_w = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    quat = quat_of(Rb.T @ Rc_w)
    if GC:      # only in the motions/SLAM model: a camera in the XML makes the Simulink plant block grow RGB/depth ports
        ET.SubElement(head, "camera", name="head_cam", fovy="60", pos=" ".join(f"{v:.5f}" for v in Rb.T @ (cam_w - pb)),
                      quat=" ".join(f"{v:.6f}" for v in quat))

    # S1: simplified collision proxies (ground-contact model only; the walk model collides at the feet only).
    # Primitives are defined in V3 mesh axes (mm) in scripts/collision_proxies.json.
    def proxy(parent, name, kind, size_mm, centre_mm, origin, R):
        c = origin + R @ (np.array(centre_mm) / 1000.0)
        ET.SubElement(parent, "geom", name=name, type=kind, **{"class": "collision"},
                      size=" ".join(f"{s / 1000.0:.5f}" for s in size_mm),
                      pos=" ".join(f"{v:.5f}" for v in c), quat=" ".join(f"{v:.6f}" for v in quat_of(R)))
    if GC:
        for p_ in PROXIES["head"]:
            proxy(head, p_["name"], p_["type"], p_["size_mm"], p_["centre_mm"], place[p_["frame"]], R_mb)

    # rear shield on the torso: mesh x -> -y, mesh y -> up, mesh z -> backwards (solid face at the rear)
    sp = np.array([float(v) for v in SHIELD_POS.split()])
    Rs = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    sh = v3["hat_rear_shield_v3"]
    c_mesh = (np.array(sh["min_mm"]) + np.array(sh["max_mm"])) / 2000.0
    sh_origin = sp - Rs @ c_mesh
    trunk = bodies["trunk_base"]
    ET.SubElement(trunk, "geom", type="mesh", mesh="hat_rear_shield_v3", **{"class": "visual"}, material="black_pla",
                  pos=" ".join(f"{v:.5f}" for v in sh_origin), quat=" ".join(f"{v:.6f}" for v in quat_of(Rs)))
    Mt, Ct, It = body_inertial(trunk)          # torso link WITHOUT the V3 shield; the shield is always an addition
    kg = part_mass_kg("hat_rear_shield_v3", v3, design_checks, ledger)
    Msh = part_props(DST / "assets" / "hat_rear_shield_v3.stl", Rs, sh_origin, mass=kg)
    M2, C2, I2 = combine([(Mt, Ct, It), Msh])
    check_physical("torso with shield", M2, I2)
    set_inertial(trunk, M2, C2, I2)
    print(f"  torso link: {Mt*1000:.1f} g + shield {kg*1000:.1f} g -> {M2*1000:.1f} g")
    if GC:
        for p_ in PROXIES["shield"]:
            proxy(trunk, p_["name"], p_["type"], p_["size_mm"], p_["centre_mm"], sh_origin, Rs)
        if PROXIES.get("exclude"):
            contact = root.find("contact")
            if contact is None:
                contact = ET.SubElement(root, "contact")
            for a_, b_ in PROXIES["exclude"]:
                ET.SubElement(contact, "exclude", body1=a_, body2=b_)

    # A1: materials. black PLA for printed rigid parts, glass for the lens, everything else untouched.
    for spec in (BLACK_PLA, LENS_GLASS):
        ET.SubElement(asset, "material", **spec)
    for body in root.iter("body"):
        for g in body.findall("geom"):
            if g.get("class") != "visual":
                continue
            if g.get("mesh") in PRINTED_RIGID_MESHES:
                g.set("material", "black_pla")
            elif g.get("mesh") in LENS_MESHES:
                g.set("material", "lens_glass")
    used_mats = {g.get("material") for g in root.iter("geom")}
    used_meshes = {g.get("mesh") for g in root.iter("geom")}
    for mt in list(asset.findall("material")):                     # drop materials/meshes no geom references any more
        if mt.get("name") not in used_mats:
            asset.remove(mt)
    for ms in list(asset.findall("mesh")):
        nm = ms.get("name") or Path(ms.get("file")).stem
        if nm not in used_meshes:
            asset.remove(ms)

    # 3. push point on the torso + force motors (placed LAST in the actuator list)
    ET.SubElement(trunk, "site", name="push_point", pos="0 0 0")
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
    (DST / f"mass_ledger{'_' + _args.tag if _args.tag else ''}.json").write_text(json.dumps(
        dict(status="PROVISIONAL: 30% infill wall-proxy masses, not slicer/scale values",
             pla_density_g_mm3=PLA_DENSITY_G_MM3, parts=ledger), indent=1))

    # 5. scene: floor + light + keyframes (ctrl padded with 2 zeros for the push motors)
    scene = ET.parse(SRC / "scene_walk.xml")
    sroot = scene.getroot()
    sroot.find("include").set("file", OUT_ROBOT)
    for k in sroot.iter("key"):
        k.set("qpos", " ".join(k.get("qpos").split()))
        k.set("ctrl", " ".join(k.get("ctrl").split()) + " 0 0")
    stand = next(k for k in sroot.iter("key") if k.get("name") == "STAND")
    q = stand.get("qpos").split(); q[2] = "0.1172"
    ET.SubElement(sroot.find("keyframe"), "key", name="sim_start", qpos=" ".join(q),
                  ctrl=stand.get("ctrl"))
    ET.indent(scene, "  ")
    scene.write(DST / OUT_SCENE)
    print("wrote", DST / OUT_ROBOT, "and", DST / OUT_SCENE)


if __name__ == "__main__":
    sys.exit(main())
