r"""check_collision_proxies.py -- how well do the S1 collision primitives follow the print meshes?

Usage:  .venv\Scripts\python.exe scripts\check_collision_proxies.py
Uses dense surface samples of the print meshes. Head: upper+lower+bill together (lower-mesh frame, upper shifted +22.35 mm).
Reports, per group: % of mesh samples more than 2 mm outside every proxy (under-coverage), the worst such distance, and
the 95th/max distance of proxy-surface samples to the nearest mesh sample (over-reach; for a hollow shell the proxy
surface is compared with the outer+inner wall, so interior-lying proxy faces are not penalised heavily).
Writes results/post_fix/collision_proxy_check.json.
"""
import json, os
from pathlib import Path
import numpy as np, trimesh
from scipy.spatial import cKDTree

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
P = json.loads((ROOT / "scripts" / "collision_proxies.json").read_text())
ASSET = ROOT / "model" / "assets"
SEAM_MM = 22.35
SHIFT = {"duck_head_lower_v3": 0.0, "duck_head_upper_v3": SEAM_MM, "hat_rear_shield_v3": 0.0}


def sdist(pr, pts):
    q = pts - np.array(pr["centre_mm"]); s = np.array(pr["size_mm"])
    if pr["type"] == "box":
        d = np.abs(q) - s
        return np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(d.max(1), 0)
    r = np.linalg.norm(q / s, axis=1)
    return (r - 1) * s.min()


def surface(pr, n=6000):
    rng = np.random.default_rng(0); s = np.array(pr["size_mm"]); c = np.array(pr["centre_mm"])
    if pr["type"] == "ellipsoid":
        u = rng.normal(size=(n, 3)); u /= np.linalg.norm(u, axis=1)[:, None]; return c + u * s
    pts = rng.uniform(-1, 1, (n, 3)) * s
    ax = rng.integers(0, 3, n); sg = rng.choice([-1, 1], n); pts[np.arange(n), ax] = sg * s[ax]; return c + pts


def mesh_pts(name, n=120000):
    m = trimesh.load(ASSET / f"{name}.stl"); m.apply_scale(1000.0)
    pts, _ = trimesh.sample.sample_surface(m, n, seed=0)
    return pts + [0, 0, SHIFT[name]]


groups = {"head": (["duck_head_upper_v3", "duck_head_lower_v3"], P["head"]), "shield": (["hat_rear_shield_v3"], P["shield"])}
rep = {}
for g, (meshes, prs) in groups.items():
    pts = np.vstack([mesh_pts(m) for m in meshes])
    shifted = []
    for p in prs:
        q = dict(p); q["centre_mm"] = list(np.array(p["centre_mm"]) + [0, 0, SHIFT.get(p.get("frame", "hat_rear_shield_v3"), 0.0)])
        shifted.append(q)
    d = np.min([sdist(p, pts) for p in shifted], axis=0)
    tree = cKDTree(pts)
    over = {}
    for p in shifted:
        dd, _ = tree.query(surface(p)); over[p["name"]] = dict(p95=round(float(np.percentile(dd, 95)), 1), max=round(float(dd.max()), 1))
    rep[g] = dict(samples_outside_gt2mm_pct=round(float((d > 2).mean() * 100), 2), worst_outside_mm=round(float(d.max()), 1),
                  proxy_surface_dist_to_mesh_mm=over)
    print(g, json.dumps(rep[g]))
out = ROOT / "results" / "post_fix"; out.mkdir(parents=True, exist_ok=True)
(out / "collision_proxy_check.json").write_text(json.dumps(rep, indent=1))
