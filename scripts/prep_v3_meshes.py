r"""prep_v3_meshes.py -- decimate the V3 print meshes for simulation and measure them.

Usage:  .venv\Scripts\python.exe scripts\prep_v3_meshes.py [design folder]
Design folder (first that exists): argv[1], $DUCKLING_DESIGN, <repo>/design.
Reads   <design>/modified/*.stl (millimetres; versioned in the repo) and, only for the "original volume"
        bookkeeping, <design>/original_parts/duck_head_*.stl if present (else the recorded constants below).
Writes  model/assets/duck_head_upper_v3.stl, duck_head_lower_v3.stl, hat_rear_shield_v3.stl
        (metres, ~20k triangles) and model/v3_meshes.json (bounds, volumes, areas).
Output directories are created if missing. No absolute user paths are written.
"""
import json, os, sys
from pathlib import Path
import fast_simplification, numpy as np, trimesh

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
_c = ([Path(sys.argv[1])] if len(sys.argv) > 1 else []) + \
     ([Path(os.environ["DUCKLING_DESIGN"])] if os.environ.get("DUCKLING_DESIGN") else []) + [ROOT / "design"]
V3 = next((c for c in _c if (c / "modified").is_dir()), None)
if V3 is None:
    raise SystemExit("design folder with modified/ not found; tried " + ", ".join(map(str, _c)))
OUT = ROOT / "model" / "assets"
OUT.mkdir(parents=True, exist_ok=True)
# volumes (mm3) of the upstream original shells, measured from design/original_parts (used if that folder is absent)
ORIG_VOL = {"duck_head_upper": 28681.334, "duck_head_lower": 39772.674}
JOBS = {  # output name: (new mesh, original part stem or None)
    "duck_head_upper_v3": ("modified/duck_head_upper_v3.stl", "duck_head_upper"),
    "duck_head_lower_v3": ("modified/duck_head_lower_v3.stl", "duck_head_lower"),
    "hat_rear_shield_v3": ("modified/hat_rear_shield_v3_open_vertical_ends.stl", None),
}
info = {}
for name, (new, old) in JOBS.items():
    mesh = trimesh.load(V3 / new)
    entry = dict(source=new, faces_in=len(mesh.faces), volume_mm3=float(mesh.volume), area_mm2=float(mesh.area),
                 min_mm=mesh.bounds[0].tolist(), max_mm=mesh.bounds[1].tolist(),
                 centroid_mm=mesh.center_mass.tolist())
    if old:
        p = V3 / "original_parts" / f"{old}.stl"
        entry["orig_volume_mm3"] = float(trimesh.load(p).volume) if p.exists() else ORIG_VOL[old]
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
(ROOT / "model").mkdir(exist_ok=True)
(ROOT / "model" / "v3_meshes.json").write_text(json.dumps(info, indent=1))
