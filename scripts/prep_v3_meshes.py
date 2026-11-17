"""prep_v3_meshes.py -- decimate the V3 print meshes for simulation and measure them.

Usage:  .venv\\Scripts\\python.exe scripts\\prep_v3_meshes.py [path to duckling_v3 folder]
Reads   <duckling_v3>/modified/*.stl and original_parts/duck_head_*.stl   (millimetres)
Writes  model/assets/duck_head_upper_v3.stl, duck_head_lower_v3.stl, hat_rear_shield_v3.stl
        (metres, ~20k triangles) and model/v3_meshes.json (bounds, volumes).
"""
import json, sys
from pathlib import Path
import fast_simplification, numpy as np, trimesh

ROOT = Path(__file__).resolve().parents[1]
V3 = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "Downloads" / "Duckling_V3_Design_and_MuJoCo" / "duckling_v3"
OUT = ROOT / "model" / "assets"
JOBS = {  # output name: (new mesh, original mesh it replaces or None)
    "duck_head_upper_v3": ("modified/duck_head_upper_v3.stl", "original_parts/duck_head_upper.stl"),
    "duck_head_lower_v3": ("modified/duck_head_lower_v3.stl", "original_parts/duck_head_lower.stl"),
    "hat_rear_shield_v3": ("modified/hat_rear_shield_v3_open_vertical_ends.stl", None),
}
info = {}
for name, (new, old) in JOBS.items():
    mesh = trimesh.load(V3 / new)
    entry = dict(source=str(V3 / new), faces_in=len(mesh.faces), volume_mm3=float(mesh.volume),
                 min_mm=mesh.bounds[0].tolist(), max_mm=mesh.bounds[1].tolist(),
                 centroid_mm=mesh.center_mass.tolist())
    if old:
        entry["orig_volume_mm3"] = float(trimesh.load(V3 / old).volume)
    target = 20000
    if len(mesh.faces) > target:
        v, f = fast_simplification.simplify(mesh.vertices, mesh.faces, target_count=target)
        mesh = trimesh.Trimesh(v, f)
    mesh.apply_scale(0.001)                                   # mm -> m
    mesh.export(OUT / f"{name}.stl")
    entry["faces_out"] = len(mesh.faces)
    info[name] = entry
    print(f"{name}: {entry['faces_in']} -> {entry['faces_out']} faces, volume {entry['volume_mm3']/1000:.1f} cm3"
          + (f" (original {entry['orig_volume_mm3']/1000:.1f} cm3)" if old else ""))
(ROOT / "model" / "v3_meshes.json").write_text(json.dumps(info, indent=1))
