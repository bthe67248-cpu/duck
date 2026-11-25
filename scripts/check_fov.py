r"""check_fov.py -- does the new bill (or any robot part) block the head camera / ToF fields of view?

Usage:  .venv\Scripts\python.exe scripts\check_fov.py
Ray-casts (mj_ray, visual meshes = geom group 2 only, floor/world excluded) from the head camera (`head_cam`, 60 deg
vertical, 4:3) and the ToF site (`tof`, 8x8 zones over 45 x 45 deg) through a sweep of head poses (neck_pitch,
head_pitch, head_yaw, head_roll). A ray that hits the robot itself within 0.5 m counts as blocked. The ToF boresight is
the site axis that points forward (+x) at the STAND pose. The camera origin in the model is already 12 mm in front
of the upstream camera site; the ToF site is not, so the ToF is evaluated both at its site and 12 mm forward of it. Writes results/post_fix/fov_check.txt.
Limits: centre rays only (a real ToF zone integrates an angular area), no lens/bracket geometry (none exists in the model).
"""
import itertools, os
from pathlib import Path
import mujoco, numpy as np

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
m = mujoco.MjModel.from_xml_path(str(ROOT / "model" / "duck_v4_gc_scene.xml")); d = mujoco.MjData(m)
key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "STAND")
mujoco.mj_resetDataKeyframe(m, d, key); mujoco.mj_forward(m, d)
home = d.qpos.copy()
cam, tof = m.camera("head_cam").id, m.site("tof").id
R0 = d.site_xmat[tof].reshape(3, 3)
axis = int(np.argmax(np.abs(R0[0])))                      # local axis most aligned with world +x at STAND
sgn = float(np.sign(R0[0, axis]))
GROUP = np.zeros(6, np.uint8); GROUP[2] = 1
lines = []


def log(s=""):
    print(s); lines.append(s)


def rays(origin, R, fwd_local, right_local, up_local, hfov, vfov, n):
    u = np.tan(np.radians(np.linspace(-hfov / 2, hfov / 2, n) * (1 - 1 / n)))
    v = np.tan(np.radians(np.linspace(-vfov / 2, vfov / 2, n) * (1 - 1 / n)))
    out = []
    for a in u:
        for b in v:
            dirl = fwd_local + a * right_local + b * up_local
            out.append(R @ (dirl / np.linalg.norm(dirl)))
    return np.array(out)


def blocked(origin, dirs):
    n = 0; hits = {}; dmin = 9.0
    gid = np.zeros(1, np.int32)
    for dv in dirs:
        dist = mujoco.mj_ray(m, d, origin, dv, GROUP, 1, -1, gid)
        if 0 <= dist < 0.5:
            n += 1; dmin = min(dmin, dist)
            nm = m.geom(gid[0]).name or m.body(m.geom_bodyid[gid[0]]).name
            hits[nm] = hits.get(nm, 0) + 1
    return n, dmin, hits


def sensors():
    Rc = d.cam_xmat[cam].reshape(3, 3)
    cam_dirs = rays(d.cam_xpos[cam], Rc, np.array([0, 0, -1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), 60 * 4 / 3, 60, 16)
    Rt = d.site_xmat[tof].reshape(3, 3)
    f = np.zeros(3); f[axis] = sgn
    # other two local axes: keep the one most aligned with world y as "right", the remaining as "up"
    others = [i for i in range(3) if i != axis]
    ri = max(others, key=lambda i: abs(Rt[1, i])); ui = [i for i in others if i != ri][0]
    r_l = np.zeros(3); r_l[ri] = 1; u_l = np.zeros(3); u_l[ui] = 1
    tof_dirs = rays(d.site_xpos[tof], Rt, f, r_l, u_l, 45, 45, 8)
    fwd_w = Rt @ f
    return (d.cam_xpos[cam], cam_dirs), (d.site_xpos[tof], tof_dirs), (d.site_xpos[tof] + 0.012 * fwd_w, tof_dirs)


names = ("neck_pitch", "head_pitch", "head_yaw", "head_roll")
adr = {n: m.jnt_qposadr[m.joint(n).id] for n in names}
log(f"ToF boresight = site axis {'xyz'[axis]} (sign {sgn:+.0f}); STAND-pose boresight in world: {np.round(R0[:, axis] * sgn, 2)}")
grid = dict(neck_pitch=[home[adr['neck_pitch']] + o for o in (-0.4, 0.0, 0.4)],
            head_pitch=[home[adr['head_pitch']] + o for o in (-0.6, -0.3, 0.0, 0.3, 0.6)],
            head_yaw=[-1.2, 0.0, 1.2], head_roll=[-0.43, 0.0, 0.43])
LABS = ("camera", "tof @ site", "tof +12 mm fwd")
worst = {l: (0, 9.0, {}, None) for l in LABS}
tot = {l: 0 for l in LABS}; npose = 0
for vals in itertools.product(*(grid[n] for n in names)):
    d.qpos[:] = home
    for n, v in zip(names, vals):
        lo, hi = m.jnt_range[m.joint(n).id]; d.qpos[adr[n]] = float(np.clip(v, lo, hi))
    mujoco.mj_forward(m, d)
    npose += 1
    for lab, (o, dirs) in zip(LABS, sensors()):
        n, dmin, hits = blocked(o, dirs)
        tot[lab] += n
        if n > worst[lab][0] or (n == worst[lab][0] and dmin < worst[lab][1]):
            worst[lab] = (n, dmin, hits, vals)
        if vals == tuple(home[adr[x]] for x in names):
            log(f"{lab}: level head (STAND): {n}/{len(dirs)} rays hit the robot within 0.5 m" + (f", nearest {dmin*1000:.0f} mm {hits}" if n else ""))
for lab in LABS:
    n, dmin, hits, vals = worst[lab]
    nr = 256 if lab == "camera" else 64
    log(f"{lab}: over {npose} head poses: mean {100*tot[lab]/(npose*nr):.2f}% of rays blocked; worst pose {n}/{nr} "
        f"(nearest hit {dmin*1000:.0f} mm; geoms {hits}; joints {dict(zip(names, np.round(vals, 2))) if vals else None})")
(ROOT / "results" / "post_fix").mkdir(parents=True, exist_ok=True)
(ROOT / "results" / "post_fix" / "fov_check.txt").write_text("\n".join(lines) + "\n")
