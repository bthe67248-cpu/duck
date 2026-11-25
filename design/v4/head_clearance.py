"""head_clearance.py -- clearance cutters for the V4 head shells, from the moving upstream parts around the neck.

    python design/v4/head_clearance.py [--write]      (reads model/duck_v4_gc.xml; --write updates v4_params.json)

The head shells (head_upper_v4 / head_lower_v4) were reshaped from the V3 head, and their neck opening no longer
clears the upstream neck/head mechanism (fit check: head-roll servo 5.1 mm, neck-pitch bearing 3.7 mm, bracket 2.5 mm,
neck servo 2.7 mm when the head pitches). build_v4.py subtracts clearance volumes from the shells; this script
measures them, in the lower-head mesh frame (mm, x = width, -y = front, z = up), so build_v4.py itself needs only
v4_params.json (no upstream meshes):

  revolve   parts that turn about the head-yaw axis (vertical, through yaw_xy): everything on the neck_pitch body, and
            the neck body's parts swept over the head-pitch range first. Yaw covers +-170 deg, so their envelope is a
            solid of revolution: profile [z, r] = largest distance from the yaw axis per 1 mm of height + clearance.
  boxes     parts on yaw_roll_motion (the head-roll servo, the roll bearings) and the XL330s fixed in the head:
            axis-aligned boxes (the XL330s are boxes; the bearings' bounding boxes) + clearance.
build_v4.py sweeps the revolve cutter and the yaw_roll_motion boxes over the head-roll range (roll axis along y
through roll_xz) before subtracting them; the fixed boxes are subtracted as they are.
"""
import argparse, json, sys
from pathlib import Path
import mujoco, numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
import check_v4_fit as C  # noqa: E402
from solid import Solid  # noqa: E402

CLEAR = 1.2           # mm radial clearance (fit check counts > 0.8 mm; the simulation meshes are decimated)
PITCH_RANGE = (-0.25, 0.95)   # head_pitch (rad): the fit check's pose envelope (standing 0.35 +- 0.6)


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("--write", action="store_true")
    ap.add_argument("--model", default=str(ROOT / "model/duck_v4_gc.xml"))
    a = ap.parse_args(argv)
    m = mujoco.MjModel.from_xml_path(str(Path(a.model).with_name("duck_v4_gc_world.xml"))); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("sim_start").id)
    for j in ("head_yaw", "head_roll"):
        d.qpos[m.jnt_qposadr[m.joint(j).id]] = 0.0
    mujoco.mj_forward(m, d)
    R_h, p_h = C.head_mesh_frame(a.model)
    hd = m.body("jaw_soft").id
    Rhd, phd = d.xmat[hd].reshape(3, 3), d.xpos[hd]

    def to_head(Xw):
        return ((Xw - phd) @ Rhd - p_h) @ R_h * 1000.0

    def axis_head(j):
        return to_head(d.xanchor[j][None])[0], (d.xaxis[j] @ Rhd) @ R_h

    yaw_p, yaw_a = axis_head(m.joint("head_yaw").id)
    roll_p, roll_a = axis_head(m.joint("head_roll").id)
    pit_p, pit_a = axis_head(m.joint("head_pitch").id)
    assert abs(abs(yaw_a[2]) - 1) < 1e-3 and abs(abs(roll_a[1]) - 1) < 1e-3, (yaw_a, roll_a)
    q_pitch = d.qpos[m.jnt_qposadr[m.joint("head_pitch").id]]

    def samples(body):
        out = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != m.body(body).id or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or m.geom_group[g] != 2:
                continue
            mid = m.geom_dataid[g]
            V = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(float)
            F = m.mesh_face[m.mesh_faceadr[mid]: m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
            W = V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
            X = to_head(W)
            out.append((m.mesh(mid).name, X, Solid(X / 1000.0, F, n_samples=20000).P * 1000.0))
        return out

    def rot(X, p, ax, ang):
        ax = ax / np.linalg.norm(ax); K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        R = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
        return (X - p) @ R.T + p

    # revolve: neck_pitch parts as they are, neck parts swept over head pitch (relative to the standing pose)
    pts = [P for _, _, P in samples("neck_pitch")]
    neck = [P for _, _, P in samples("neck")]
    for q in np.linspace(*PITCH_RANGE, 41):
        # the head (and the yaw/roll axes) turn by +(q - q0) about the pitch axis relative to the neck, so in the head
        # frame the neck turns by -(q - q0)
        pts += [rot(P, pit_p, pit_a, -(q - q_pitch)) for P in neck]
    X = np.concatenate(pts)
    r = np.hypot(X[:, 0] - yaw_p[0], X[:, 1] - yaw_p[1])
    # keep the points that can reach the shell (z >= -1) once yawed to the side and rolled: highest point over the
    # roll range of a point at distance r from the axis (the yaw and roll axes cross at x = yaw_xy[0])
    phi = np.linspace(*m.jnt_range[m.joint("head_roll").id], 41)
    dz = X[:, 2] - roll_p[2]
    zmax = roll_p[2] + (r[:, None] * np.abs(np.sin(phi))[None] + dz[:, None] * np.cos(phi)[None]).max(1)
    X, r = X[zmax >= -1.0], r[zmax >= -1.0]
    zb = np.floor(X[:, 2]).astype(int)
    lo_z, hi_z = int(zb.min()), 34       # build_v4.py extends the lowest ring 15 mm further down
    prof = []
    for z in range(lo_z, hi_z + 1):
        sel = (zb >= z - 1) & (zb <= z + 1)      # max over the neighbouring 1 mm bins: conservative between bins
        if sel.any():
            prof.append([z, round(float(r[sel].max()) + CLEAR, 1)])
    boxes = []
    for body, sweep in (("yaw_roll_motion", True), ("jaw_soft", False)):
        for name, V, _ in samples(body):
            if name not in ("xl330", "seeed_bearing__configuration__22x16x4"):
                continue
            lo, hi = V.min(0) - CLEAR, V.max(0) + CLEAR
            boxes.append(dict(part=f"{body}/{name}", roll_sweep=sweep, lo=lo.round(1).tolist(), hi=hi.round(1).tolist()))
    out = dict(_note="written by head_clearance.py from the upstream neck/head parts (see its docstring); lower-head "
                     "mesh frame, mm. clear_mm = radial clearance added", clear_mm=CLEAR,
               yaw_xy=yaw_p[:2].round(2).tolist(), roll_xz=[round(float(roll_p[0]), 2), round(float(roll_p[2]), 2)],
               roll_range_rad=[round(float(v), 3) for v in m.jnt_range[m.joint("head_roll").id]],
               head_pitch_range_rad=list(PITCH_RANGE), revolve_profile=prof, boxes=boxes)
    print(json.dumps({k: v for k, v in out.items() if k != "revolve_profile"}, indent=1))
    print("profile z: r =", " ".join(f"{z}:{r_}" for z, r_ in prof))
    if a.write:
        pf = HERE / "v4_params.json"
        params = json.loads(pf.read_text())
        params["head"]["clearance"] = out
        pf.write_text(json.dumps(params, indent=1) + "\n")
        print("wrote", pf)
    return out


if __name__ == "__main__":
    main()
