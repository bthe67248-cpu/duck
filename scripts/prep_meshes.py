r"""prep_meshes.py -- decimate the V4 print meshes for simulation and measure them.

Usage:  .venv\Scripts\python.exe scripts\prep_meshes.py
Reads   design/v4/*_v4.stl (millimetres; versioned in the repo).
Writes  model/assets/<part>.stl (metres, at most ~20k triangles) and model/v4_meshes.json (bounds, volumes, areas of the
        full-resolution print meshes, used for the provisional print masses). No absolute user paths are written.
        Each entry has a "role": "asset" (a simulation mesh written to model/assets) or "reference" (bounds of an uncut
        source the head placement is measured from; no asset is written for it).
Fails if a print mesh or its decimated asset is not a closed solid (masses and inertias come from these volumes).
"""
import json, os
from pathlib import Path
import fast_simplification, numpy as np, trimesh

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
V4 = ROOT / "design" / "v4"
OUT = ROOT / "model" / "assets"
OUT.mkdir(parents=True, exist_ok=True)
JOBS = ["head_upper_v4", "head_lower_v4", "jaw_v4", "mouth_servo_v4", "body_shell_v4", "body_lid_v4", "keel_cover_v4",
        "pcb_stack_v4", "a010_v4", "battery_v4", "xt60_v4"]
info, bad = {}, []
for name in JOBS:
    mesh = trimesh.load(V4 / f"{name}.stl")
    entry = dict(source=f"design/v4/{name}.stl", faces_in=len(mesh.faces), volume_mm3=float(mesh.volume),
                 area_mm2=float(mesh.area), min_mm=mesh.bounds[0].tolist(), max_mm=mesh.bounds[1].tolist(),
                 centroid_mm=mesh.center_mass.tolist(), watertight=bool(mesh.is_watertight), role="asset")
    if not mesh.is_watertight:
        bad.append(f"design/v4/{name}.stl")
    # decimate to ~20k triangles; if that opens the mesh (checked on the file as written, since reloading merges
    # vertices that decimation brought together), keep more triangles, down to none removed
    for target in (20000, 40000, 80000, None):
        if target is not None and len(mesh.faces) <= target:
            continue
        out = mesh.copy()
        if target is not None:
            v, f = fast_simplification.simplify(out.vertices, out.faces, target_count=target)
            out = trimesh.Trimesh(v, f)
        out.apply_scale(0.001)                                # mm -> m
        out.export(OUT / f"{name}.stl")
        written = trimesh.load(OUT / f"{name}.stl")
        if written.is_watertight:
            break
    entry["faces_out"] = len(written.faces)
    entry["asset_watertight"] = bool(written.is_watertight)
    if not written.is_watertight:
        bad.append(f"model/assets/{name}.stl")
    info[name] = entry
    print(f"{name}: {entry['faces_in']} -> {entry['faces_out']} faces, volume {entry['volume_mm3']/1000:.1f} cm3")
# sources the head placement is measured from (make_duck.py): openings cut into the shells must not move the head
for name in ["head_upper_v4_whole"]:
    mesh = trimesh.load(V4 / f"{name}.stl")
    info[name] = dict(source=f"design/v4/{name}.stl", min_mm=mesh.bounds[0].tolist(), max_mm=mesh.bounds[1].tolist(),
                      role="reference")
if bad:
    raise SystemExit("not a closed solid (rebuild with design/v4/build_v4.py): " + ", ".join(bad))
(ROOT / "model").mkdir(exist_ok=True)
(ROOT / "model" / "v4_meshes.json").write_text(json.dumps(info, indent=1))
