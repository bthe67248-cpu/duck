r"""make_duck.py -- build model/duck_v4*.xml (+ scene) from the upstream Microduck model and the V4 parts in design/v4.

Usage:  .venv\Scripts\python.exe scripts\make_duck.py [--base robot_groundcontact.xml --tag gc] [--calibration F]
Run scripts\prep_meshes.py once first (it prepares the V4 meshes from design/v4).

Roots are derived from this file's location. Overrides: $DUCKLING_ROOT (repo root), $DUCKLING_UPSTREAM
(folder holding robot_walk.xml, scene_walk.xml, assets/ ...; default microduck_rl/.../robot/microduck).

V4 against the upstream Microduck (design/v4/README.md):
  torso  upstream left/right shells, banana PCB locker, battery holder (power_support) and battery -> body_shell_v4
         (+ body_lid_v4, keel_cover_v4 and their fasteners) enclosing the UNO Q + 70 mm HAT stack, a 2S LiPo + XT60
         and a Sipeed A010 (site `tof_body` on its front face). The upstream battery's
         mass, COM and inertia are subtracted (UPSTREAM_BATTERY_G) and the LiPo is added as its own box at its V4
         location (mass from v4_params.json battery.mass_g)
  head   upstream shells, mouth, face plate/ring (face_part, noenoeil) and Pi Zero + HAT -> head_upper_v4 /
         head_lower_v4 with an XL330 mouth servo in the dome; the lower half of the beak is a `jaw` body on the `mouth`
         hinge with a position actuator (placed after the push motors)
  limits neck pitch capped at NECK_PITCH_MAX (head/body clearance, design/v4/check_v4_fit.py)
  servos command delay SERVO_DELAY_S (or the calibration's servo.delay_s), stored in the model as
         <custom><numeric name="servo_delay_s">; scripts/motions.py and world_sim apply it (duck_nav/actuation.py)

Edit ONLY the block below with your own scale/ruler measurements, then re-run.
Everything marked ESTIMATE / PROVISIONAL is a placeholder, not a measurement.
"""
import argparse, hashlib, json, os, shutil, sys
import mujoco
import numpy as np
import trimesh
from pathlib import Path
import xml.etree.ElementTree as ET
from calibration import load_calibration

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
SRC = Path(os.environ.get("DUCKLING_UPSTREAM",
                          ROOT / "microduck_rl" / "src" / "mjlab_microduck" / "robot" / "microduck"))
DST = ROOT / "model"
V4 = ROOT / "design" / "v4"

# ----------------------------------------------------------------- EDIT THIS BLOCK
# Weighed COMPLETE-LINK masses in kg (servos + screws included). None = keep the composite estimate below.
# "jaw_soft" (head link) = the head WITH its V4 shells and mouth servo, WITHOUT the jaw; "trunk_base" = the torso
# with the body shell, boards, battery, XT60 and A010 installed.
MEASURED_MASS = {
    "trunk_base": None, "jaw_soft": None,           # torso, head
    "upper_leg_left": None, "upper_leg_right": None,
    "leg": None, "leg_2": None,                      # shanks
    "ankle_left": None, "ankle_right": None,         # feet
}
# Printed V4 parts (grams). None -> PROVISIONAL wall-proxy formula with the print settings in design/v4/v4_params.json
# ("print": wall_mm and infill, a per-part entry over "default"):
#   V_wall = min(V, A*wall); V_print = V_wall + infill (V - V_wall); m = V_print * 0.00124 g/mm3.
PART_MASS_G = {"head_upper_v4": None, "head_lower_v4": None, "jaw_v4": None, "body_shell_v4": None,
               "body_lid_v4": None, "keel_cover_v4": None}
# Bought parts (grams). ESTIMATES until weighed.
BOUGHT_G = {"pcb_stack_v4": 45.0, "a010_v4": 10.0, "xt60_v4": 8.0, "mouth_servo_v4": 18.0,
            "fasteners_body_v4": 4.0}   # 12 x M2 screws + heat-set inserts (lid, cover, trunk and servo mounts)
PLA_DENSITY_G_MM3 = 0.00124
# The upstream torso's battery (np_f970 part) is inside its inertial; V4 replaces it with the 2S LiPo. The part is a
# Sony NP-F550/F570-size 2S Li-ion camcorder pack (mesh 38.6 x 20.6 x 70.8 mm; the F970 is 51 mm thick); retail
# listings for that size give about 100 g. The mass fit (torso remainder after the other parts at the masses implied
# by the upstream neck, shank and hip links: 123 g; torso COM fit: 103-125 g) agrees. Weigh the real pack to replace it.
UPSTREAM_BATTERY_G = 100.0
# Servo command delay (s): the servos act on the target sent this long ago. ESTIMATE: the upstream policies are trained
# with 3-6 physics steps (15-30 ms) of actuator lag (microduck_rl microduck_constants.py). A measured servo.delay_s in
# the calibration file replaces it.
SERVO_DELAY_S = 0.020
# Upstream parts that V4 REPLACES (their mass/inertia is subtracted). ESTIMATE: mesh volume x 1240 kg/m3 (solid).
UPSTREAM_PART_DENSITY = 1240.0
HEAD_SHIFT = (0.0, 0.0, 0.0)    # nudge the head (forward, left, up) in m if it sits wrong
SERVO = dict(kp=None, kv=None, stall=None)   # from your bench test (Part 5); None = upstream
NECK_PITCH_MAX = 0.70           # rad (upstream 1.047): neck tipped back beyond this lets the head reach the board bay top
                                # (design/v4/check_v4_fit.py); set the same limit in the neck servo
MOUTH_KP, MOUTH_KV, MOUTH_FORCE = 0.5, 0.02, 0.6
# ----------------------------------------------------------------- END OF EDIT BLOCK

BLACK_PLA = dict(name="black_pla", rgba="0.025 0.025 0.025 1", specular="0.35", shininess="0.30")
LENS_GLASS = dict(name="lens_glass", rgba="0.04 0.07 0.12 1", specular="0.9", shininess="0.95")
V4_MATERIALS = [dict(name="v4_pcb", rgba="0.05 0.35 0.15 1"), dict(name="v4_lipo", rgba="0.15 0.3 0.75 1"),
                dict(name="v4_xt60", rgba="0.85 0.65 0.1 1"), dict(name="v4_sensor", rgba="0.12 0.12 0.14 1"),
                dict(name="v4_servo", rgba="0.3 0.3 0.3 1")]
PRINTED_RIGID_MESHES = {
    "trunk_base", "power_support", "motor_support",
    "neck", "neck_pitch", "yaw2roll", "yaw_roll_motion", "bearing_roll", "hip_l",
    "upper_leg_left", "upper_leg_right", "upper_leg_rigidity_plate", "leg",
    "ankle_left", "ankle_right", "foot_left", "foot_right",
    "head_upper_v4", "head_lower_v4", "jaw_v4", "body_shell_v4", "body_lid_v4", "keel_cover_v4"}
LENS_MESHES = {"lens"}
# upstream head visuals replaced by the V4 head (shells, mouth, face plate/ring, old head electronics)
REMOVED_HEAD_VISUAL = {"top_head_shell", "bottom_head_shell", "jaw", "jaw_soft", "soft_mouth_top",
                       "face_part", "noenoeil", "elec_rpi_robot_hat_pcb", "pcb__raspberry_pi_zero_2_w"}
REMOVED_HEAD_COLLISION = {"top_head_shell", "bottom_head_shell", "jaw"}
# upstream torso parts replaced by the V4 body and the 2S LiPo (np_f970: the battery, UPSTREAM_BATTERY_G; power_support:
# the old battery holder, replaced by the body's battery compartment)
REMOVED_TORSO = {"left_shell", "right_shell", "banana_pcb_locker", "np_f970", "power_support"}
V4_PARTS = ["head_upper_v4", "head_lower_v4", "jaw_v4", "mouth_servo_v4", "body_shell_v4", "body_lid_v4", "keel_cover_v4",
            "pcb_stack_v4", "a010_v4", "battery_v4", "xt60_v4"]

_ap = argparse.ArgumentParser()
_ap.add_argument("--torso-scale", type=float, default=1.0)
_ap.add_argument("--tag", default="")
_ap.add_argument("--calibration", type=Path, help="measured JSON inputs (see config/robot_measurements.example.json)")
_ap.add_argument("--base", default="robot_walk.xml",
                 help="upstream robot file: robot_walk.xml (feet-only contact) or robot_groundcontact.xml (body can lie on the floor)")
_args = _ap.parse_args()
if any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in _args.tag):
    _ap.error("--tag may contain only letters, numbers, underscore and hyphen")
CALIBRATION = None
if _args.calibration:
    try:
        CALIBRATION = load_calibration(_args.calibration)
    except (ValueError, OSError) as exc:
        _ap.error(str(exc))
    MEASURED_MASS.update(CALIBRATION["measured_mass_kg"])
    PART_MASS_G.update(CALIBRATION["print_mass_g"])
    SERVO.update({k: v for k, v in CALIBRATION["servo"].items() if k != "delay_s"})
    if "head_shift_m" in CALIBRATION:
        HEAD_SHIFT = CALIBRATION["head_shift_m"]
    if "delay_s" in CALIBRATION["servo"]:
        SERVO_DELAY_S = CALIBRATION["servo"]["delay_s"]
OUT_ROBOT = f"duck_v4{'_' + _args.tag if _args.tag else ''}.xml"
OUT_SCENE = f"duck_v4{'_' + _args.tag if _args.tag else ''}_scene.xml"
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


def fmt(v, p=5):
    return " ".join(f"{x:.{p}f}" for x in v)


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
    if i is None:
        i = ET.Element("inertial"); body.insert(0, i)
    for k in ("diaginertia", "fullinertia", "quat"):
        i.attrib.pop(k, None)
    i.set("pos", " ".join(f"{v:.6g}" for v in C)); i.set("mass", f"{M:.6f}")
    i.set("fullinertia", " ".join(f"{v:.6g}" for v in (I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2])))


def print_settings(params, name):
    """(wall_mm, infill) of a printed part: v4_params.json "print" (a per-part entry over "default")."""
    pr = params.get("print", {})
    s = {"wall_mm": 1.2, "infill": 0.30, **pr.get("default", {}), **pr.get(name, {})}
    return float(s["wall_mm"]), float(s["infill"])


def formula_mass_g(vol_mm3, area_mm2, wall_mm=1.2, infill=0.30):
    vw = min(vol_mm3, area_mm2 * wall_mm)
    return (vw + infill * (vol_mm3 - vw)) * PLA_DENSITY_G_MM3


def printed_kg(name, meshes, ledger, params):
    wall, infill = print_settings(params, name)
    if PART_MASS_G[name] is not None:
        g, how = PART_MASS_G[name], "WEIGHED/SLICED (user supplied)"
    else:
        g = formula_mass_g(meshes[name]["volume_mm3"], meshes[name]["area_mm2"], wall, infill)
        how = f"PROVISIONAL {infill:.0%} infill, {wall:g} mm wall-proxy formula (not a slicer value)"
    ledger[name] = dict(mass_g=round(g, 2), status=how, volume_mm3=round(meshes[name]["volume_mm3"], 1),
                        area_mm2=round(meshes[name]["area_mm2"], 1), wall_mm=wall, infill=infill)
    return g / 1000.0


def bought_kg(name, ledger):
    ledger[name] = dict(mass_g=BOUGHT_G[name], status="ESTIMATE (bought part, not weighed)")
    return BOUGHT_G[name] / 1000.0


def body_proxies(params):
    """Torso collision boxes = the outer blocks of the V4 body shell (bevels ignored), torso frame, mm (half sizes)."""
    return [dict(name=f"body_{b['name']}_collision", type="box",
                 size_mm=[(h - l) / 2 for l, h in zip(b["lo"], b["hi"])],
                 centre_mm=[(h + l) / 2 for l, h in zip(b["lo"], b["hi"])]) for b in params["body"]["outer"]]


def remove_parts(body, visual_names, collision_names=()):
    """Remove geoms of the named upstream meshes; return (mesh, pos, quat) of the removed visuals."""
    out = []
    for g in list(body.findall("geom")):
        mname = g.get("mesh")
        if mname in visual_names and g.get("class") == "visual":
            out.append((mname, np.array([float(v) for v in g.get("pos", "0 0 0").split()]),
                        np.array([float(v) for v in g.get("quat", "1 0 0 0").split()])))
            body.remove(g)
        elif mname in visual_names or mname in collision_names:
            body.remove(g)
    return out


def main():
    n = sync_upstream()
    if n:
        print(f"  copied {n} missing upstream files into {DST}")
    tree = ET.parse(SRC / _args.base)
    root = tree.getroot()
    root.set("model", "duck_v4")
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

    # 2. V4 parts
    if not (DST / "v4_meshes.json").exists():
        raise SystemExit("model/v4_meshes.json missing: run scripts/prep_meshes.py first")
    meshes = json.loads((DST / "v4_meshes.json").read_text())
    params = json.loads((V4 / "v4_params.json").read_text())
    PROXIES = json.loads((ROOT / "scripts" / "collision_proxies.json").read_text())
    BOUGHT_G.setdefault("battery_v4", float(params["battery"]["mass_g"]))
    asset = root.find("asset")
    for n_ in V4_PARTS:
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
    # head mesh frame (mm): x = width, -y = front, z = up.  world = Rz(+90 deg) * mesh + t; fronts flush with upstream
    Rw = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    up = meshes.get("head_upper_v4_whole", meshes["head_upper_v4"])   # uncut source: the lens opening is not the front
    lo = meshes["head_lower_v4"]
    mid_wid = (up["min_mm"][0] + up["max_mm"][0]) / 2000.0
    front = -up["min_mm"][1] / 1000.0
    seam = params["head"]["upper_zshift"] / 1000.0                 # upper part's own frame starts at the seam
    t_lower = np.array([top_hi[0] - front, -mid_wid, bot_lo[2]]) + np.array(HEAD_SHIFT)
    t_upper = t_lower + np.array([0, 0, seam])
    head = bodies["jaw_soft"]
    R_mb = Rb.T @ Rw                                               # head mesh axes -> head body axes
    quat_mb = quat_of(R_mb)
    place = {"head_upper_v4": Rb.T @ (t_upper - pb), "head_lower_v4": Rb.T @ (t_lower - pb)}
    place["mouth_servo_v4"] = place["head_lower_v4"]               # placeholders are drawn in the lower-head frame

    removed_head = remove_parts(head, REMOVED_HEAD_VISUAL, REMOVED_HEAD_COLLISION)
    for n_ in ("head_upper_v4", "head_lower_v4"):
        ET.SubElement(head, "geom", type="mesh", mesh=n_, **{"class": "visual"}, material="black_pla",
                      pos=fmt(place[n_]), quat=fmt(quat_mb, 6))
    ET.SubElement(head, "geom", type="mesh", mesh="mouth_servo_v4", **{"class": "visual"}, material="v4_servo",
                  pos=fmt(place["mouth_servo_v4"]), quat=fmt(quat_mb, 6))

    # ---- head mass: upstream head - replaced upstream parts + V4 shells + mouth servo (the jaw is its own body)
    if MEASURED_MASS["jaw_soft"] is None:
        M0, C0, I0 = body_inertial(head)
        parts = [(M0, C0, I0)]; removed_g = {}
        for mname, pos, q in removed_head:
            mm, cc, ii = part_props(DST / "assets" / f"{mname}.stl", mat_of(q), pos, density=UPSTREAM_PART_DENSITY)
            parts.append((-mm, cc, ii)); removed_g[mname] = removed_g.get(mname, 0) + round(mm * 1000, 1)
        Mr, Cr, Ir = combine(parts)
        check_physical("head without replaced upstream parts", Mr, Ir)
        parts = [(Mr, Cr, Ir)]
        for n_ in ("head_upper_v4", "head_lower_v4"):
            parts.append(part_props(DST / "assets" / f"{n_}.stl", R_mb, place[n_], mass=printed_kg(n_, meshes, ledger, params)))
        parts.append(part_props(DST / "assets" / "mouth_servo_v4.stl", R_mb, place["mouth_servo_v4"],
                                mass=bought_kg("mouth_servo_v4", ledger)))
        M1, C1, I1 = combine(parts)
        check_physical("new head link", M1, I1)
        set_inertial(head, M1, C1, I1)
        ledger["_head_link"] = dict(upstream_mass_g=round(M0 * 1000, 1), removed_upstream_parts_g_ESTIMATE=removed_g,
                                    remainder_g=round(Mr * 1000, 1), new_mass_g=round(M1 * 1000, 1),
                                    note="replaced, not added: upstream shells/mouth/face plate/head boards subtracted, "
                                         "V4 shells + mouth servo added; the jaw is a separate body")
        print(f"  head link: {M0*1000:.1f} g upstream - {sum(removed_g.values()):.1f} g replaced parts (est.) "
              f"+ V4 shells and mouth servo -> {M1*1000:.1f} g")
    else:
        print("  head link mass is the WEIGHED value (shells + mouth servo included, jaw excluded)")
        ledger["_head_link"] = dict(new_mass_g=MEASURED_MASS["jaw_soft"] * 1000, note="weighed complete link")
        for n_ in ("head_upper_v4", "head_lower_v4"):
            printed_kg(n_, meshes, ledger, params); ledger[n_]["included_in_measured_head"] = True
        bought_kg("mouth_servo_v4", ledger); ledger["mouth_servo_v4"]["included_in_measured_head"] = True

    # ---- jaw: child body at the hinge (rear-bottom edge of the beak cut), axis = head-mesh x, opens 0..open_deg
    J = params["head"]["jaw"]
    hinge_mesh = np.array([0.0, J["hinge_y"], J["hinge_z"]]) / 1000.0
    hinge_body = R_mb @ hinge_mesh + place["head_lower_v4"]
    jaw_rel = place["head_lower_v4"] - hinge_body
    jaw = ET.SubElement(head, "body", name="jaw", pos=fmt(hinge_body, 6))
    jaw_kg = printed_kg("jaw_v4", meshes, ledger, params)
    Mj, Cj, Ij = part_props(DST / "assets" / "jaw_v4.stl", R_mb, jaw_rel, mass=jaw_kg)
    ET.SubElement(jaw, "inertial", pos=" ".join(f"{v:.6g}" for v in Cj), mass=f"{Mj:.6f}",
                  fullinertia=" ".join(f"{v:.6g}" for v in (Ij[0, 0], Ij[1, 1], Ij[2, 2], Ij[0, 1], Ij[0, 2], Ij[1, 2])))
    ET.SubElement(jaw, "joint", name="mouth", type="hinge", axis=fmt(R_mb @ np.array([1.0, 0, 0]), 6),
                  range=f"0 {np.radians(J['open_deg']):.6f}", damping="0.01", armature="0.0005")
    ET.SubElement(jaw, "geom", type="mesh", mesh="jaw_v4", **{"class": "visual"}, material="black_pla",
                  pos=fmt(jaw_rel), quat=fmt(quat_mb, 6))
    # mouth_tip: lowest-front point of the jaw (bill tip), now on the moving jaw
    jm = trimesh.load(DST / "assets" / "jaw_v4.stl").vertices
    cx_mesh = (lo["min_mm"][0] + lo["max_mm"][0]) / 2000.0
    cen = jm[np.abs(jm[:, 0] - cx_mesh) < 0.008]
    tip = R_mb @ cen[np.argmax(-cen[:, 1] - cen[:, 2])] + jaw_rel
    for s_ in list(head.findall("site")):
        if s_.get("name") == "mouth_tip":
            head.remove(s_)
    ET.SubElement(jaw, "site", group="3", name="mouth_tip", pos=fmt(tip, 6))
    print(f"  mouth hinge in head frame {np.round(hinge_body, 4)} m; bill tip on the jaw {np.round(tip, 4)}")

    # head camera (RGB + depth come from this one MuJoCo camera): at the upstream camera site
    cam_w = ud.site("head_camera").xpos + np.array([0.012, 0.0, 0.0])
    Rc_w = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    if GC:      # only in the motions/SLAM model: a camera in the XML makes the Simulink plant block grow RGB/depth ports
        ET.SubElement(head, "camera", name="head_cam", fovy="60", pos=fmt(Rb.T @ (cam_w - pb)),
                      quat=fmt(quat_of(Rb.T @ Rc_w), 6))

    # ---- torso: V4 body shell with boards, LiPo, XT60 and A010 inside
    trunk = bodies["trunk_base"]
    removed_torso = remove_parts(trunk, REMOVED_TORSO)
    for n_, mat_ in (("body_shell_v4", "black_pla"), ("body_lid_v4", "black_pla"), ("keel_cover_v4", "black_pla"),
                     ("pcb_stack_v4", "v4_pcb"), ("a010_v4", "v4_sensor"), ("battery_v4", "v4_lipo"), ("xt60_v4", "v4_xt60")):
        ET.SubElement(trunk, "geom", type="mesh", mesh=n_, **{"class": "visual"}, material=mat_)
    if MEASURED_MASS["trunk_base"] is None:
        Mt, Ct, It = body_inertial(trunk)
        parts = [(Mt, Ct, It)]; removed_g = {}
        for mname, pos, q in removed_torso:
            if mname == "np_f970":                                # upstream battery: ESTIMATED mass, uniform density
                mm, cc, ii = part_props(DST / "assets" / f"{mname}.stl", mat_of(q), pos, mass=UPSTREAM_BATTERY_G / 1000)
            else:
                mm, cc, ii = part_props(DST / "assets" / f"{mname}.stl", mat_of(q), pos, density=UPSTREAM_PART_DENSITY)
            parts.append((-mm, cc, ii)); removed_g[mname] = round(mm * 1000, 1)
        Mr, Cr, Ir = combine(parts)
        check_physical("torso without replaced upstream parts", Mr, Ir)
        for n_ in ("body_shell_v4", "body_lid_v4", "keel_cover_v4"):
            parts.append(part_props(DST / "assets" / f"{n_}.stl", np.eye(3), np.zeros(3),
                                    mass=printed_kg(n_, meshes, ledger, params)))
        # fasteners: a point mass at the body shell's centroid (spread over the lid, cover and mounts)
        fc = np.array(meshes["body_shell_v4"]["centroid_mm"]) / 1000.0
        parts.append((bought_kg("fasteners_body_v4", ledger), fc, np.zeros((3, 3))))
        for n_ in ("pcb_stack_v4", "a010_v4", "xt60_v4", "battery_v4"):
            parts.append(part_props(DST / "assets" / f"{n_}.stl", np.eye(3), np.zeros(3), mass=bought_kg(n_, ledger)))
        lipo = parts[-1]
        ledger["battery_v4"].update(com_mm=[round(v * 1000, 1) for v in lipo[1]],
                                    principal_inertia_kg_m2=[float(f"{v:.4g}") for v in np.linalg.eigvalsh(lipo[2])],
                                    note="uniform box at its V4 location (design/v4/v4_params.json battery)")
        M2, C2, I2 = combine(parts)
        check_physical("V4 torso", M2, I2)
        set_inertial(trunk, M2, C2, I2)
        ledger["_torso_link"] = dict(upstream_mass_g=round(Mt * 1000, 1), removed_upstream_parts_g_ESTIMATE=removed_g,
                                     remainder_g=round(Mr * 1000, 1), new_mass_g=round(M2 * 1000, 1),
                                     upstream_com_mm=[round(v * 1000, 1) for v in Ct], com_mm=[round(v * 1000, 1) for v in C2],
                                     note="replaced, not added: upstream shells, PCB locker, battery holder (power_support) "
                                          "and battery (np_f970, UPSTREAM_BATTERY_G ESTIMATE) subtracted; V4 body shell, "
                                          "lid, keel cover, fasteners, boards, A010, XT60 and the 2S LiPo added at their V4 "
                                          "locations")
        print(f"  torso link: {Mt*1000:.1f} g upstream -> {M2*1000:.1f} g with the V4 body and its contents")
    else:
        print("  torso link mass is the WEIGHED value (body shell, boards, battery, XT60 and A010 included)")
        ledger["_torso_link"] = dict(new_mass_g=MEASURED_MASS["trunk_base"] * 1000, note="weighed complete link")
        for n_ in ("body_shell_v4", "body_lid_v4", "keel_cover_v4"):
            printed_kg(n_, meshes, ledger, params); ledger[n_]["included_in_measured_torso"] = True
        bought_kg("fasteners_body_v4", ledger); ledger["fasteners_body_v4"]["included_in_measured_torso"] = True
        for n_ in ("pcb_stack_v4", "a010_v4", "xt60_v4", "battery_v4"):
            bought_kg(n_, ledger); ledger[n_]["included_in_measured_torso"] = True
    a = params["a010"]; alo, ahi = np.array(a["lo"]) / 1000, np.array(a["hi"]) / 1000
    ET.SubElement(trunk, "site", group="3", name="tof_body",
                  pos=fmt([ahi[0] + 0.0002, (alo[1] + ahi[1]) / 2, (alo[2] + ahi[2]) / 2], 6))

    # S1: simplified collision proxies (ground-contact model only; the walk model collides at the feet only)
    def proxy(parent, name, kind, size_mm, centre_mm, origin, R):
        c = origin + R @ (np.array(centre_mm) / 1000.0)
        ET.SubElement(parent, "geom", name=name, type=kind, **{"class": "collision"},
                      size=" ".join(f"{s / 1000.0:.5f}" for s in size_mm), pos=fmt(c), quat=fmt(quat_of(R), 6))
    if GC:
        for p_ in PROXIES["head"]:
            proxy(head, p_["name"], p_["type"], p_["size_mm"], p_["centre_mm"], place[p_["frame"]], R_mb)
        for p_ in PROXIES["jaw"]:
            proxy(jaw, p_["name"], p_["type"], p_["size_mm"], p_["centre_mm"], jaw_rel, R_mb)
        for b_ in body_proxies(params):
            proxy(trunk, b_["name"], b_["type"], b_["size_mm"], b_["centre_mm"], np.zeros(3), np.eye(3))
        if PROXIES.get("exclude"):
            contact = root.find("contact")
            if contact is None:
                contact = ET.SubElement(root, "contact")
            for a_, b_ in PROXIES["exclude"]:
                ET.SubElement(contact, "exclude", body1=a_, body2=b_)

    # neck pitch cap (head/body clearance)
    for j in root.iter("joint"):
        if j.get("name") == "neck_pitch":
            lo_, hi_ = (float(v) for v in j.get("range").split())
            j.set("range", f"{lo_:.6f} {min(hi_, NECK_PITCH_MAX):.6f}")

    # A1: materials. black PLA for printed rigid parts, glass for the lens, everything else untouched.
    for spec in [BLACK_PLA, LENS_GLASS] + V4_MATERIALS:
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

    # 3. push point on the torso + force motors, then the mouth (actuators 0-13 policy, 14-15 push, 16 mouth)
    ET.SubElement(trunk, "site", name="push_point", pos="0 0 0")
    act = root.find("actuator")
    if any(v is not None for v in SERVO.values()):
        for p in act.findall("position"):
            if SERVO["kp"] is not None: p.set("kp", str(SERVO["kp"]))
            if SERVO["kv"] is not None: p.set("kv", str(SERVO["kv"]))
            if SERVO["stall"] is not None: p.set("forcerange", f"-{SERVO['stall']} {SERVO['stall']}")
    ET.SubElement(act, "motor", name="push_x", site="push_point", gear="1 0 0 0 0 0")
    ET.SubElement(act, "motor", name="push_y", site="push_point", gear="0 1 0 0 0 0")
    ET.SubElement(act, "position", name="mouth", joint="mouth", kp=str(MOUTH_KP), kv=str(MOUTH_KV),
                  ctrlrange=f"0 {np.radians(J['open_deg']):.6f}", forcerange=f"-{MOUTH_FORCE} {MOUTH_FORCE}")

    # servo command delay: a model constant read by scripts/motions.py and duck_nav.world_sim (custom numerics load in
    # every MuJoCo version and the Simulink plant ignores them; it has its own delay block)
    custom = root.find("custom")
    if custom is None:
        custom = ET.SubElement(root, "custom")
    for nm_ in list(custom.findall("numeric")):
        if nm_.get("name") == "servo_delay_s":
            custom.remove(nm_)
    ET.SubElement(custom, "numeric", name="servo_delay_s", data=f"{SERVO_DELAY_S:.6g}")

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
    m_ = mujoco.MjModel.from_xml_path(str(DST / OUT_ROBOT))
    mouth_adr = int(m_.jnt_qposadr[m_.joint("mouth").id])
    record = dict(status="PROVISIONAL: unmeasured components and upstream COM/inertia remain estimates",
                  model_robot_sha256=hashlib.sha256((DST / OUT_ROBOT).read_bytes()).hexdigest(),
                  pla_density_g_mm3=PLA_DENSITY_G_MM3, parts=ledger,
                  measured_link_mass_kg={k: v for k, v in MEASURED_MASS.items() if v is not None},
                  head_shift_m=list(HEAD_SHIFT), neck_pitch_max_rad=NECK_PITCH_MAX,
                  servo_overrides=SERVO, servo_delay_s=None, servo_delay_sim_s=SERVO_DELAY_S,
                  servo_delay_source=("calibration servo.delay_s" if CALIBRATION and "delay_s" in CALIBRATION["servo"]
                                      else "ESTIMATE: upstream training lag 15-30 ms"),
                  upstream_battery_removed_g_ESTIMATE=UPSTREAM_BATTERY_G,
                  robot_mass_kg=round(float(m_.body_subtreemass[1]), 4),
                  note="Link masses rescale existing inertia at unchanged COM; weights alone do not calibrate COM/inertia.")
    if CALIBRATION:
        record["calibration_sha256"] = CALIBRATION["source_sha256"]
        record["servo_delay_s"] = CALIBRATION["servo"].get("delay_s")
        record["whole_robot_mass_kg"] = CALIBRATION["whole_robot_mass_kg"]
    (DST / f"mass_ledger{'_' + _args.tag if _args.tag else ''}.json").write_text(json.dumps(record, indent=1))

    # 5. scene: floor + light + keyframes. qpos gets the mouth (closed) at its own address -- the jaw sits between
    #    the head and the right leg in the tree; ctrl gets the two push motors and the mouth.
    scene = ET.parse(SRC / "scene_walk.xml")
    sroot = scene.getroot()
    sroot.find("include").set("file", OUT_ROBOT)
    for k in sroot.iter("key"):
        q = k.get("qpos").split(); q.insert(mouth_adr, "0")
        k.set("qpos", " ".join(q))
        k.set("ctrl", " ".join(k.get("ctrl").split()) + " 0 0 0")
    stand = next(k for k in sroot.iter("key") if k.get("name") == "STAND")
    q = stand.get("qpos").split(); q[2] = "0.1172"
    ET.SubElement(sroot.find("keyframe"), "key", name="sim_start", qpos=" ".join(q), ctrl=stand.get("ctrl"))
    ET.indent(scene, "  ")
    scene.write(DST / OUT_SCENE)
    print(f"wrote {DST / OUT_ROBOT} and {DST / OUT_SCENE} (robot {record['robot_mass_kg']} kg, mouth qpos {mouth_adr})")


if __name__ == "__main__":
    sys.exit(main())
