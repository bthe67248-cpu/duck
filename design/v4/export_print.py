"""export_print.py -- print-ready STLs and a print report for the Duckling V4 printed parts.

    python design/v4/export_print.py [--bed 220 220 250] [--min-wall 0.8] [--out design/v4/print]

For each printed part (head shells, jaw, body shell, lid, keel cover):
  * closed solid check (every edge shared by exactly two faces; a slicer needs it, and the masses rely on it);
  * wall thickness: from 8 000 surface samples a ray goes inwards along the face normal to the next surface; the
    distribution (1st/5th percentile, median) and the share thinner than --min-wall (0.8 mm = two 0.4 mm perimeters)
    are reported. Samples on tiny features (screw bores, chamfers) produce some thin readings by construction, so
    the gate is on the 5th percentile, not the minimum;
  * no protrusion: no added feature of the head shells (lower + jaw) may stick out through their source shells' skin;
  * print orientation: fixed per part where holes or mating faces decide it (PREFERRED), otherwise the axis-aligned
    orientation with the least overhang area (faces steeper than 45 deg below horizontal, not on the bed);
  * writes <out>/<part>.stl (mm, oriented, on the bed at z = 0, centred) and <out>/print_report.json.
Exit status 1 if a part is not closed, does not fit the bed, or its 5th-percentile wall is below --hard-wall (0.6 mm);
a 5th percentile between --hard-wall and --min-wall (0.8 mm) is a warning.
"""
import argparse, json, sys
from pathlib import Path

import numpy as np
import trimesh

HERE = Path(__file__).resolve().parent
PARTS = {
    "head_upper_v4": "upper head shell (dome, bill, lens opening, mouth-servo cradle, 4 pilot holes)",
    "head_lower_v4": "lower head shell (neck clearance, jaw hinge ears, 4 clearance holes)",
    "jaw_v4": "jaw / lower bill (hinge bosses, pushrod lever)",
    "body_shell_v4": "body shell (board bay, battery compartment, connector well, A010 pocket, trunk mounts)",
    "body_lid_v4": "rear lid of the board bay",
    "keel_cover_v4": "keel cover of the connector well (LiPo rails)",
}
# Per-part orientation: chosen for the holes and mating faces, not only overhang. Shells print seam-down so the seam
# is flat and the screw bosses (axis z) are vertical, which prints the pilot holes round; the jaw lies on its side
# so the hinge bosses (axis x) are vertical and the pin holes come out round. None = least overhang.
PREFERRED = {"head_upper_v4": "as modelled", "head_lower_v4": "upside down", "jaw_v4": "on -x side",
             "body_shell_v4": None, "body_lid_v4": None, "keel_cover_v4": None}
# Added features must stay inside the original outer skin (the convex-hull trim once let the mouth-servo cradle poke
# 1.4 mm through the forehead): parts built from a committed source are compared with it.
SOURCES = {"head_upper_v4": ["head_upper_v4_whole"], "head_lower_v4": ["head_lower_v4_whole"]}
ORIENT = {"as modelled": np.eye(3), "upside down": np.diag([1.0, -1.0, -1.0]),
          "on +x side": np.array([[0, 0, -1.0], [0, 1, 0], [1, 0, 0]]), "on -x side": np.array([[0, 0, 1.0], [0, 1, 0], [-1, 0, 0]]),
          "on +y side": np.array([[1.0, 0, 0], [0, 0, -1], [0, 1, 0]]), "on -y side": np.array([[1.0, 0, 0], [0, 0, 1], [0, -1, 0]])}


def wall_thickness(m, n=8000, seed=0):
    pts, fi = trimesh.sample.sample_surface(m, n, seed=seed)
    nrm = m.face_normals[fi]
    origins = pts - nrm * 1e-3
    t = np.full(n, np.nan)
    for i0 in range(0, n, 500):                       # chunks: the candidate arrays grow with rays x triangles
        o, d = origins[i0:i0 + 500], -nrm[i0:i0 + 500]
        loc, ray, _ = m.ray.intersects_location(o, d, multiple_hits=False)
        t[i0 + ray] = np.linalg.norm(loc - o[ray], axis=1)
    return t[np.isfinite(t)]


def protrusion(parts, source, n=8000, seed=0, escape_frac=0.45, chunk=200):
    """Features added to `source` that stick out through its outer skin. Surface samples of `parts` that are new
    (more than 0.1 mm from the source's surface and not inside its wall) are tested with 26 rays each: a point in the
    shell's cavity sees the shell in most directions (rays escape only through the open underside or the lens hole),
    a point outside the skin sees open air in about half of them. Returns (number of new samples outside, the share
    of new samples outside, one example point)."""
    pts, _ = trimesh.sample.sample_surface(parts, n, seed=seed)
    dist = np.concatenate([source.nearest.on_surface(pts[i:i + chunk])[1] for i in range(0, len(pts), chunk)])
    new = pts[(dist > 0.1)]
    if len(new):
        new = new[~np.concatenate([source.contains(new[i:i + chunk]) for i in range(0, len(new), chunk)])]
    if not len(new):
        return 0, 0.0, None
    dirs = np.array([(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1) if (i, j, k) != (0, 0, 0)], float)
    dirs /= np.linalg.norm(dirs, axis=1)[:, None]
    escaped = np.zeros(len(new))
    for dv in dirs:
        for i0 in range(0, len(new), chunk):
            o = new[i0:i0 + chunk] + dv * 0.05
            hit = source.ray.intersects_any(o, np.tile(dv, (len(o), 1)))
            escaped[i0:i0 + chunk] += ~hit
    out = escaped / len(dirs) > escape_frac
    ex = new[np.argmax(escaped)].round(1).tolist() if out.any() else None
    return int(out.sum()), float(out.mean()), ex


def overhang_area(m, R):
    nz = (m.face_normals @ R.T)[:, 2]
    z = (m.triangles_center @ R.T)[:, 2]
    zmin = (m.vertices @ R.T)[:, 2].min()
    over = (nz < -np.cos(np.radians(45))) & (z > zmin + 0.3)        # steeper than 45 deg and not on the bed
    return float(m.area_faces[over].sum())


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--bed", type=float, nargs=3, default=[220.0, 220.0, 250.0])
    ap.add_argument("--min-wall", type=float, default=0.8, help="walls thinner than this are counted and warned about")
    ap.add_argument("--hard-wall", type=float, default=0.6,
                    help="fail if the 5th-percentile wall is below this (0.6 mm prints with a 0.4 mm nozzle and a "
                         "variable-width wall generator: Arachne in PrusaSlicer >= 2.5, Cura >= 5, OrcaSlicer)")
    ap.add_argument("--out", default=str(HERE / "print"))
    a = ap.parse_args(argv)
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    report, bad = {}, []
    for name, what in PARTS.items():
        m = trimesh.load(HERE / f"{name}.stl")
        closed = bool(m.is_watertight and m.is_winding_consistent)
        t = wall_thickness(m)
        best = None
        for oname, R in ORIENT.items():
            if PREFERRED.get(name) and oname != PREFERRED[name]:
                continue
            V = m.vertices @ R.T
            size = V.max(0) - V.min(0)
            fits = bool(np.all(size[:2] <= np.array(a.bed[:2]) - 10) and size[2] <= a.bed[2])
            key = (not fits, round(overhang_area(m, R) / 100.0), size[2])    # overhang in cm2 steps, then height
            if best is None or key < best[0]:
                best = (key, oname, R, size, fits)
        _, oname, R, size, fits = best
        o = m.copy(); o.vertices = m.vertices @ R.T
        o.apply_translation([-(o.bounds[0][0] + o.bounds[1][0]) / 2, -(o.bounds[0][1] + o.bounds[1][1]) / 2, -o.bounds[0][2]])
        o.export(out / f"{name}.stl")
        r = dict(description=what, closed=closed, volume_cm3=round(float(m.volume) / 1000, 2),
                 bbox_mm=[round(float(v), 1) for v in size], orientation=oname, fits_bed=fits,
                 overhang_cm2=round(overhang_area(m, R) / 100, 1),
                 wall_mm=dict(p1=round(float(np.percentile(t, 1)), 2), p5=round(float(np.percentile(t, 5)), 2),
                              median=round(float(np.median(t)), 2),
                              share_below_min_pct=round(float((t < a.min_wall).mean() * 100), 1)))
        report[name] = r
        if name in SOURCES:
            src = trimesh.util.concatenate([trimesh.load(HERE / f"{n}.stl") for n in SOURCES[name]])
            parts = m if name != "head_lower_v4" else trimesh.util.concatenate([m, trimesh.load(HERE / "jaw_v4.stl")])
            n_out, share, at = protrusion(parts, src)
            r["protruding_samples"] = n_out; r["protrusion_example_mm"] = at
        ok = closed and fits and r["wall_mm"]["p5"] >= a.hard_wall and r.get("protruding_samples", 0) == 0
        r["warning"] = (f"5th-percentile wall {r['wall_mm']['p5']} mm < {a.min_wall} mm: enable variable-width walls "
                        "(Arachne) and check the thin areas in the slicer preview") if ok and r["wall_mm"]["p5"] < a.min_wall else ""
        if not ok:
            bad.append(name)
        print(f"{'FAIL' if not ok else 'warn' if r['warning'] else 'ok  '} {name:15s} {oname:12s} bbox {r['bbox_mm']} overhang {r['overhang_cm2']} cm2 "
              f"wall p5 {r['wall_mm']['p5']} mm median {r['wall_mm']['median']} mm (<{a.min_wall}: "
              f"{r['wall_mm']['share_below_min_pct']} %) closed={closed}"
              + (f" protruding samples {r['protruding_samples']}" if "protruding_samples" in r else ""))
    report["_settings"] = dict(bed_mm=a.bed, min_wall_mm=a.min_wall, hard_wall_mm=a.hard_wall,
                               note="STLs in mm, oriented and placed on the bed; slice with the settings in PRINT_GUIDE.md")
    (out / "print_report.json").write_text(json.dumps(report, indent=1))
    if bad:
        print("not print-ready:", ", ".join(bad)); return 1
    print(f"all {len(PARTS)} parts print-ready -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
