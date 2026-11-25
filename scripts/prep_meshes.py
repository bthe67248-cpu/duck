r"""prep_meshes.py -- decimate the V4 print meshes for simulation and measure them.

Usage:  .venv\Scripts\python.exe scripts\prep_meshes.py
Reads   design/v4/*_v4.stl (millimetres; versioned in the repo).
Writes  model/assets/<part>.stl (metres, at most ~20k triangles) and model/v4_meshes.json (bounds, volumes, areas of the
        full-resolution print meshes, used for the provisional print masses). No absolute user paths are written.
"""
import json, os
from pathlib import Path
import fast_simplification, numpy as np, trimesh

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
V4 = ROOT / "design" / "v4"
OUT = ROOT / "model" / "assets"
OUT.mkdir(parents=True, exist_ok=True)
JOBS = ["head_upper_v4", "head_lower_v4", "jaw_v4", "mouth_servo_v4", "body_shell_v4",
        "pcb_stack_v4", "a010_v4", "battery_v4", "xt60_v4"]
info = {}
for name in JOBS:
    mesh = trimesh.load(V4 / f"{name}.stl")
    entry = dict(source=f"design/v4/{name}.stl", faces_in=len(mesh.faces), volume_mm3=float(mesh.volume),
                 area_mm2=float(mesh.area), min_mm=mesh.bounds[0].tolist(), max_mm=mesh.bounds[1].tolist(),
                 centroid_mm=mesh.center_mass.tolist(), watertight=bool(mesh.is_watertight))
    if len(mesh.faces) > 20000:
        v, f = fast_simplification.simplify(mesh.vertices, mesh.faces, target_count=20000)
        mesh = trimesh.Trimesh(v, f)
    mesh.apply_scale(0.001)                                   # mm -> m
    mesh.export(OUT / f"{name}.stl")
    entry["faces_out"] = len(mesh.faces)
    info[name] = entry
    print(f"{name}: {entry['faces_in']} -> {entry['faces_out']} faces, volume {entry['volume_mm3']/1000:.1f} cm3")
(ROOT / "model").mkdir(exist_ok=True)
(ROOT / "model" / "v4_meshes.json").write_text(json.dumps(info, indent=1))
