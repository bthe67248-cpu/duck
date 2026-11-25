"""check_v4_fit.py -- clearance check of the V4 concept shells against V3 (static fit + sampled operating envelope).

    python design/v4/check_v4_fit.py [--samples 600] [--clear-mm 0.8]   (reads model/duck_v4_gc_world.xml)

Static: parts rigidly inside the torso / head must not sit more than `clear-mm` inside their own body's shells.
Envelope: random poses (head joints as scripts/check_pcb_target.py: neck_pitch +-0.4, head_pitch +-0.6, head_yaw +-1.2,
head_roll +-0.44 rad; legs +-0.3 rad around the standing pose; mouth over its range). For every torso part listed
below, the number of poses in which any moving body is more than `clear-mm` inside it (penetration). The envelope is a
coarse kinematic sample, not the gait. Reference (V3, now deleted, same sampling): 129/600 poses, all the head hitting the
old rear shield. Distances use dense surface samples (about 0.3 mm).
Writes design/v4/fit_check.json.
"""
import argparse, collections, json
from pathlib import Path
import mujoco, numpy as np, trimesh
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser(); ap.add_argument("--samples", type=int, default=600)
ap.add_argument("--clear-mm", type=float, default=0.8)   # penetration depth (mm) that counts as a clash
ap.add_argument("--seed", type=int, default=1)
args = ap.parse_args()
MJ = ROOT / "model"
TORSO_PARTS = {"v3": ["left_shell", "right_shell", "hat_rear_shield_v3", "np_f970", "power_support"],
               "v4": ["body_shell_v4", "battery_v4", "pcb_stack_v4", "xt60_v4", "power_support"]}
HEAD_SHELLS = {"v3": ["duck_head_upper_v3", "duck_head_lower_v3"], "v4": ["head_upper_v4", "head_lower_v4"]}


def load(tag):
    m = mujoco.MjModel.from_xml_path(str(MJ / f"duck_{tag}_gc_world.xml")); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("sim_start").id); mujoco.mj_forward(m, d)
    return m, d


def geom_points(m, g, n, normals=False):
    mid = m.geom_dataid[g]
    V = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
    F = m.mesh_face[m.mesh_faceadr[mid]: m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
    tm = trimesh.Trimesh(V, F, process=False)
    pts, fi = trimesh.sample.sample_surface_even(tm, n, seed=0)
    R = np.zeros(9); mujoco.mju_quat2Mat(R, m.geom_quat[g]); R = R.reshape(3, 3)
    P = pts @ R.T + m.geom_pos[g]
    return (P, tm.face_normals[fi] @ R.T) if normals else P


class Solid:
    """Approximate penetration depth into a closed mesh: nearest dense surface sample + its outward normal."""
    def __init__(self, P, N):
        self.P, self.N, self.tree = P, N, cKDTree(P)

    def depth(self, X):
        dist, i = self.tree.query(X, distance_upper_bound=0.0025)      # clashes start at the surface: 2.5 mm band
        ok = np.isfinite(dist); out = np.zeros(len(X))
        out[ok] = np.maximum(0.0, -np.einsum("ij,ij->i", X[ok] - self.P[i[ok]], self.N[i[ok]]))
        return out


def analyse(tag, poses_rng):
    m, d = load(tag); home = d.qpos.copy()
    mesh_geoms = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g] == 2]
    nm = lambda g: m.mesh(m.geom_dataid[g]).name
    tb, hd = m.body("trunk_base").id, m.body("jaw_soft").id
    jaw = m.body("jaw").id if tag == "v4" else -1
    trees = {nm(g): (g, Solid(*geom_points(m, g, 100000 if "shell" in nm(g) or "head" in nm(g) else 20000, normals=True)))
             for g in mesh_geoms if nm(g) in TORSO_PARTS[tag] + HEAD_SHELLS[tag]}
    # static fit in the head (and torso, V4) against the own body's shells
    static = {}
    for g in mesh_geoms:
        b = m.geom_bodyid[g]; own = [s for s in (HEAD_SHELLS[tag] if b == hd else ["body_shell_v4"] if tag == "v4" and b == tb else [])
                                     if s in trees and nm(g) != s]
        if own and nm(g) not in HEAD_SHELLS[tag] + ["body_shell_v4"]:
            p = geom_points(m, g, 2500)
            static[f"{m.body(b).name}/{nm(g)}_{g}"] = round(max(float(trees[s][1].depth(p).max()) for s in own) * 1000, 1)
    head = {"neck_pitch": .4, "head_pitch": .6, "head_yaw": 1.2, "head_roll": .44}
    joints = [m.actuator(i).name for i in range(14)]
    moving = [g for g in mesh_geoms if m.geom_bodyid[g] != tb]
    pts = {g: geom_points(m, g, 400) for g in moving}
    rng = np.random.default_rng(args.seed)
    per_part = collections.Counter(); per_pair = collections.Counter(); bad = 0
    for _ in range(args.samples):
        q = home.copy()
        for n in list(head) + [n for n in joints if n not in head]:
            j = m.joint(n).id; a = head.get(n, .3)
            q[m.jnt_qposadr[j]] = np.clip(home[m.jnt_qposadr[j]] + rng.uniform(-a, a), *m.jnt_range[j])
        if jaw >= 0:
            jm = m.joint("mouth").id; q[m.jnt_qposadr[jm]] = rng.uniform(*m.jnt_range[jm])
        d.qpos[:] = q; mujoco.mj_kinematics(m, d)
        hit_parts = set()
        for g in moving:
            b = m.geom_bodyid[g]; W = pts[g] @ d.xmat[b].reshape(3, 3).T + d.xpos[b]
            L = (W - d.xpos[tb]) @ d.xmat[tb].reshape(3, 3)
            for s in TORSO_PARTS[tag]:
                if s in trees and b not in (hd,) or (s in trees and b == hd):
                    if float(trees[s][1].depth(L).max()) * 1000 > args.clear_mm:
                        hit_parts.add(s); per_pair[f"{m.body(b).name} vs {s}"] += 1
            if b == jaw:
                Lh = (W - d.xpos[hd]) @ d.xmat[hd].reshape(3, 3)
                for s in HEAD_SHELLS[tag]:
                    if float(trees[s][1].depth(Lh).max()) * 1000 > args.clear_mm:
                        hit_parts.add("jaw vs " + s); per_pair[f"jaw vs {s}"] += 1
        for s in hit_parts:
            per_part[s] += 1
        bad += bool(hit_parts)
    return dict(static_max_penetration_mm=static, poses_with_contact=bad,
                poses_per_torso_part=dict(per_part.most_common()), poses_per_pair=dict(per_pair.most_common(25)))


out = dict(samples=args.samples, clear_mm=args.clear_mm, v4=analyse("v4", None),
           v3_reference="V3 (deleted): 129/600 poses, all head-into-rear-shield (same sampling, 24 Nov 2026)")
(ROOT / "design/v4/fit_check.json").write_text(json.dumps(out, indent=1))
for tag in ("v4",):
    r = out[tag]
    print(f"== {tag}: {r['poses_with_contact']}/{args.samples} poses with a moving part more than {args.clear_mm} mm inside a torso part")
    print("   per torso part:", r["poses_per_torso_part"])
    print("   parts inside their own body's shell walls at rest (max mm):",
          {k.split('/')[1]: v for k, v in r["static_max_penetration_mm"].items() if v > args.clear_mm} or "none")
