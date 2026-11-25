"""check_v4_fit.py -- mechanical fit check of the V4 duck: static fit, a sampled pose envelope and recorded motions.

    python design/v4/check_v4_fit.py [--samples 3000] [--clear-mm 0.8] [--traj results/v4_fit/traj/*.npz]
                                     (reads model/duck_v4_gc_world.xml and design/v4/v4_params.json)

Every visual mesh of the robot becomes a `Solid` (design/v4/solid.py): containment from signed ray crossings and
depth as the distance to the surface, so a point deep inside a part reports its real depth. Two parts interpenetrate
by the deeper of "samples of A inside B" and "samples of B inside A" (mesh-mesh, both directions). The previous
check only looked within 2.5 mm of B's surface samples, one direction; its numbers (`old`) are kept for comparison.

Static (standing pose, mouth closed): every pair of parts on the torso, and on the head, where at least one is a V4
part or a keep-out envelope (`envelopes` in v4_params.json: connector plugs and cable bends, ESTIMATES). Upstream
CAD pairs (servo in its bracket, bearing in its housing) are interference fits by design and not listed.
Pose envelope: random poses (head joints neck_pitch +-0.4, head_pitch +-0.6, head_yaw +-1.2, head_roll +-0.44 rad;
legs +-0.3 rad around standing; mouth over its range). Pairs checked: each moving part against the torso's V4 parts,
power support and torso envelopes; the jaw against the head; the V4 head parts against the neck and head-servo
bodies; head chain against both legs; left leg against right.
Motions: qpos recordings from `scripts/motions.py --record DIR` (walk, sit/stand, fall + get-up, ...), the same pairs
on every 50 Hz frame, plus the MuJoCo collision proxies' robot-robot contacts deeper than `clear-mm`.
A pose or frame counts when a pair penetrates more than `clear-mm`. Writes design/v4/fit_check.json.
"""
import argparse, collections, glob, json, sys, time
import xml.etree.ElementTree as ET
from pathlib import Path
import mujoco, numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from solid import Solid, box_mesh  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
MJ = ROOT / "model"
V4_TORSO = ["body_shell_v4", "battery_v4", "pcb_stack_v4", "xt60_v4", "a010_v4"]
TORSO_TARGETS = V4_TORSO + ["power_support"]
HEAD_SHELLS = ["head_upper_v4", "head_lower_v4"]
V4_HEAD = HEAD_SHELLS + ["mouth_servo_v4", "jaw_v4"]
NSUB = 4000                         # surface samples per part used as query points in the pose loops


def parse_args(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=3000)
    ap.add_argument("--clear-mm", type=float, default=0.8)   # penetration depth (mm) that counts as a clash
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--traj", nargs="*", default=[], help="qpos recordings (.npz) from scripts/motions.py --record")
    ap.add_argument("--stride", type=int, default=1, help="check every n-th recorded frame")
    ap.add_argument("--model", default=str(MJ / "duck_v4_gc_world.xml"))
    ap.add_argument("--out", default=str(ROOT / "design/v4/fit_check.json"))
    return ap.parse_args(argv)


def mat(q):
    R = np.zeros(9); mujoco.mju_quat2Mat(R, np.asarray(q, float)); return R.reshape(3, 3)


class Part:
    """A mesh (or envelope box) rigidly attached to a body, as a Solid in the BODY frame."""
    def __init__(self, name, body, V, F, chain):
        self.name, self.body, self.chain = name, body, chain
        self.S = Solid(V, F)
        step = max(1, len(self.S.P) // NSUB)
        self.sub = self.S.P[::step]


def chain_of(m, b):
    """Kinematic chain of a body: 'torso', 'head', 'left', 'right' (first body below the trunk)."""
    tb = m.body("trunk_base").id
    while m.body_parentid[b] not in (tb, 0):
        b = m.body_parentid[b]
    if b == tb:
        return "torso"
    return {"neck": "head", "yaw2roll": "left", "bearing_roll": "right"}.get(m.body(b).name, m.body(b).name)


def head_mesh_frame(model_xml):
    """(R, p) of the lower-head mesh frame (metres) in the head body, from the generated robot XML."""
    geo = next(e for e in ET.parse(model_xml).getroot().iter("geom") if e.get("mesh") == "head_lower_v4")
    return mat([float(v) for v in geo.get("quat").split()]), np.array([float(v) for v in geo.get("pos").split()])


def build_parts(m, params, model_xml):
    parts = []
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or m.geom_group[g] != 2:
            continue
        mid = m.geom_dataid[g]
        V = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(float)
        F = m.mesh_face[m.mesh_faceadr[mid]: m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
        V = V @ mat(m.geom_quat[g]).T + m.geom_pos[g]
        b = int(m.geom_bodyid[g])
        parts.append(Part(f"{m.body(b).name}/{m.mesh(mid).name}#{g}", b, V, F, chain_of(m, b)))
    # keep-out envelopes: torso frame = trunk_base body frame; head frame = the lower-head mesh frame (mm)
    tb, hd = m.body("trunk_base").id, m.body("jaw_soft").id
    R_h, p_h = head_mesh_frame(model_xml)
    for env in params.get("envelopes", []):
        V, F = box_mesh(np.array(env["lo"]) / 1000, np.array(env["hi"]) / 1000)
        if env["frame"] == "torso":
            parts.append(Part("envelope/" + env["name"], tb, V, F, "torso"))
        else:
            parts.append(Part("envelope/" + env["name"], hd, V @ R_h.T + p_h, F, "head"))
    return parts


def short(p):
    return p.name.split("/")[1].split("#")[0]


def is_v4(p):
    return p.name.startswith("envelope/") or short(p) in V4_TORSO + V4_HEAD


def pair_depth(A, B, TA, TB, old=False, dense=False):
    """Deepest mutual penetration (m) of parts A, B with body transforms TA, TB = (R, p); with old=True also the
    previous one-directional band estimate (A's samples near B's surface). dense: all surface samples as queries."""
    (Ra, pa), (Rb, pb) = TA, TB
    ca, cb = Ra @ A.S.centre + pa, Rb @ B.S.centre + pb
    if np.linalg.norm(ca - cb) > A.S.radius + B.S.radius:
        return (0.0, 0.0) if old else 0.0
    Rab, pab = Rb.T @ Ra, Rb.T @ (pa - pb)              # A frame -> B frame
    XA = (A.S.P if dense else A.sub) @ Rab.T + pab
    XB = ((B.S.P if dense else B.sub) - pab) @ Rab                           # B frame -> A frame
    dep = max(float(B.S.depth(XA).max(initial=0)), float(A.S.depth(XB).max(initial=0)))
    return (dep, float(B.S.band_depth(XA).max(initial=0))) if old else dep


def pairs_for_motion(parts, m):
    """(i, j) index pairs checked in moving poses (see module docstring)."""
    jaw = m.body("jaw").id
    out = []
    for i, A in enumerate(parts):
        for j in range(i + 1, len(parts)):
            B = parts[j]
            ca, cb = A.chain, B.chain
            if A.body == B.body or ca == cb == "torso":
                continue
            if "torso" in (ca, cb):
                T, M = (A, B) if ca == "torso" else (B, A)
                if short(T) in TORSO_TARGETS or T.name.startswith("envelope/"):
                    out.append((j, i) if T is A else (i, j))   # (moving, torso): the old method's direction
            elif ca == cb == "head":
                J, H = (A, B) if A.body == jaw else (B, A) if B.body == jaw else (None, None)
                if J is not None and (short(H) in HEAD_SHELLS + ["mouth_servo_v4"] or H.name.startswith("envelope/")):
                    out.append((parts.index(J), parts.index(H)))
                elif J is None and (is_v4(A) or is_v4(B)):
                    out.append((i, j))                   # V4 head parts vs the neck / head-servo bodies they move around
            elif ca != cb:
                out.append((i, j))                       # head vs a leg, left leg vs right leg
    return out


def transforms(d):
    return [(d.xmat[b].reshape(3, 3).copy(), d.xpos[b].copy()) for b in range(len(d.xpos))]


def label(parts, i, j):
    return " vs ".join(sorted((parts[i].name.split("#")[0], parts[j].name.split("#")[0])))


def static_fit(m, d, parts, clear):
    T = transforms(d)
    rows = {}
    for i, A in enumerate(parts):
        for j in range(i + 1, len(parts)):
            B = parts[j]
            if not (is_v4(A) or is_v4(B)) or A.chain != B.chain or A.chain not in ("torso", "head"):
                continue
            new, old = pair_depth(A, B, T[A.body], T[B.body], old=True, dense=True)
            if new > 0 or old > 0:
                k = label(parts, i, j)
                prev = rows.get(k, (0, 0))
                rows[k] = (max(prev[0], round(new * 1000, 1)), max(prev[1], round(old * 1000, 1)))
    return {k: dict(new_mm=v[0], old_mm=v[1], clash=v[0] > clear) for k, v in sorted(rows.items(), key=lambda kv: -kv[1][0])}


def run_poses(m, d, parts, pairs, qposes, clear, rest=frozenset()):
    """Over a sequence of qpos vectors: poses with any clash (new / old method), poses with a clash in a pair that is
    clear at rest (`rest` = pairs already clashing standing; motion-caused clashes), per pair counts, worst depth (mm)."""
    per, per_old, worst = collections.Counter(), collections.Counter(), collections.defaultdict(float)
    bad = bad_old = bad_moving = 0
    for q in qposes:
        d.qpos[:] = q; mujoco.mj_kinematics(m, d)
        T = transforms(d)
        hit = hit_old = hit_moving = False
        for i, j in pairs:
            dn, do = pair_depth(parts[i], parts[j], T[parts[i].body], T[parts[j].body], old=True)
            k = label(parts, i, j)
            worst[k] = max(worst[k], dn * 1000)
            if dn * 1000 > clear:
                per[k] += 1; hit = True; hit_moving |= k not in rest
            if do * 1000 > clear:
                per_old[k] += 1; hit_old = True
        bad += hit; bad_old += hit_old; bad_moving += hit_moving
    return dict(bad=bad, bad_old=bad_old, bad_moving=bad_moving, per=per, per_old=per_old, worst=worst)


def proxy_contacts(m, d, qposes, clear):
    """MuJoCo collision proxies: frames with a robot-robot contact deeper than `clear` mm, per geom pair."""
    per = collections.Counter(); n = 0
    for q in qposes:
        d.qpos[:] = q; mujoco.mj_forward(m, d)
        hit = False
        for c in d.contact[:d.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if b1 == 0 or b2 == 0 or -c.dist * 1000 <= clear:
                continue
            n1 = m.geom(c.geom1).name or f"{m.body(b1).name}/geom"; n2 = m.geom(c.geom2).name or f"{m.body(b2).name}/geom"
            per[" vs ".join(sorted((n1, n2)))] += 1; hit = True
        n += hit
    return n, dict(per.most_common(10))


def sample_poses(m, home, n, seed):
    head = {"neck_pitch": .4, "head_pitch": .6, "head_yaw": 1.2, "head_roll": .44}
    joints = [m.actuator(i).name for i in range(14)]
    rng = np.random.default_rng(seed)
    jm = m.joint("mouth").id
    for _ in range(n):
        q = home.copy()
        for nme in list(head) + [x for x in joints if x not in head]:
            j = m.joint(nme).id; a = head.get(nme, .3)
            q[m.jnt_qposadr[j]] = np.clip(home[m.jnt_qposadr[j]] + rng.uniform(-a, a), *m.jnt_range[j])
        q[m.jnt_qposadr[jm]] = rng.uniform(*m.jnt_range[jm])
        yield q


def top(counter, n):
    return dict(collections.Counter(counter).most_common(n))


def main(argv=None):
    args = parse_args(argv)
    params = json.loads((ROOT / "design/v4/v4_params.json").read_text())
    m = mujoco.MjModel.from_xml_path(args.model); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("sim_start").id); mujoco.mj_forward(m, d)
    home = d.qpos.copy()
    t0 = time.time()
    parts = build_parts(m, params, Path(args.model).with_name("duck_v4_gc.xml"))
    print(f"{len(parts)} parts as solids in {time.time() - t0:.0f} s", flush=True)
    out = dict(samples=args.samples, clear_mm=args.clear_mm,
               method="mesh-mesh containment (signed ray crossings) + surface distance, both directions; "
                      "old = previous 2.5 mm surface band, one direction")
    out["static"] = static_fit(m, d, parts, args.clear_mm)
    pairs = pairs_for_motion(parts, m)
    rest = frozenset(k for k, v in run_poses(m, d, parts, pairs, [home], args.clear_mm)["per"].items() if v)
    out["clash_at_rest"] = sorted(rest)
    r = run_poses(m, d, parts, pairs, sample_poses(m, home, args.samples, args.seed), args.clear_mm, rest)
    out["pose_envelope"] = dict(pairs_checked=len(pairs), poses_with_clash=r["bad"], poses_with_clash_old_method=r["bad_old"],
                                poses_with_clash_not_at_rest=r["bad_moving"],
                                poses_per_pair=top(r["per"], 25), poses_per_pair_old_method=top(r["per_old"], 25),
                                worst_mm={k: round(v, 1) for k, v in sorted(r["worst"].items(), key=lambda kv: -kv[1])[:15] if v > 0})
    print(f"pose envelope: {r['bad']}/{args.samples} poses with a clash > {args.clear_mm} mm (old method {r['bad_old']}), "
          f"{r['bad_moving']} in pairs that are clear at rest "
          f"[{time.time() - t0:.0f} s]", flush=True)
    motions = {}
    for f in sorted({p for pat in args.traj for p in glob.glob(pat)}):
        qs = np.load(f)["qpos"][::args.stride]
        if qs.shape[1] != m.nq:
            raise SystemExit(f"{f}: qpos has {qs.shape[1]} values, model has {m.nq}")
        r = run_poses(m, d, parts, pairs, qs, args.clear_mm, rest)
        nprox, prox = proxy_contacts(m, d, qs, args.clear_mm)
        motions[Path(f).stem] = dict(frames=len(qs), seconds=round(len(qs) * 0.02 * args.stride, 1), frames_with_clash=r["bad"],
                                     frames_with_clash_not_at_rest=r["bad_moving"],
                                     frames_with_clash_old_method=r["bad_old"], frames_per_pair=top(r["per"], 15),
                                     worst_mm={k: round(v, 1) for k, v in sorted(r["worst"].items(), key=lambda kv: -kv[1])[:8] if v > 0},
                                     proxy_contact_frames=nprox, proxy_contacts=prox)
        print(f"motion {Path(f).stem}: {r['bad']}/{len(qs)} frames with a mesh clash (old method {r['bad_old']}), "
              f"{r['bad_moving']} in pairs clear at rest, "
              f"{nprox} with a proxy contact > {args.clear_mm} mm; worst {max(r['worst'].values(), default=0):.1f} mm "
              f"[{time.time() - t0:.0f} s]", flush=True)
    out["motions"] = motions
    Path(args.out).write_text(json.dumps(out, indent=1))
    print("static fit (mm, new / old method):")
    for k, v in out["static"].items():
        print(f"  {v['new_mm']:5.1f} / {v['old_mm']:4.1f}  {'CLASH ' if v['clash'] else ''}{k}")
    return out


if __name__ == "__main__":
    main()
