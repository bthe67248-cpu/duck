r"""render_geometry.py -- assembled-robot renders (visual meshes) for the acceptance evidence.

Usage:  .venv\Scripts\python.exe scripts\render_geometry.py [model scene xml] [out dir]
Default: model/duck_v4_gc_scene.xml -> results/geometry_v4/{front,side,rear,three_quarter,mouth_closeup}.png
plus collision_proxies_*.png showing the S1 primitives (geom group 3) over the visual meshes.
"""
import os, sys
from pathlib import Path
import mujoco, numpy as np
from PIL import Image

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
xml = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "model" / "duck_v4_gc_scene.xml"
out = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "results" / "geometry_v4"
out.mkdir(parents=True, exist_ok=True)
m = mujoco.MjModel.from_xml_path(str(xml)); d = mujoco.MjData(m)
m.vis.global_.offwidth, m.vis.global_.offheight = 1600, 1200
m.vis.headlight.ambient[:] = 0.45; m.vis.headlight.diffuse[:] = 0.75; m.vis.headlight.specular[:] = 0.15
mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")); mujoco.mj_forward(m, d)
r = mujoco.Renderer(m, 960, 1280)
tip = d.site("mouth_tip").xpos.copy()


def shot(name, az, el, look, dist, groups=(2,), size=None):
    opt = mujoco.MjvOption(); opt.geomgroup[:] = 0
    for g in groups: opt.geomgroup[g] = 1
    cam = mujoco.MjvCamera(); cam.lookat[:] = look; cam.distance = dist; cam.azimuth = az; cam.elevation = el
    r.update_scene(d, cam, opt); Image.fromarray(r.render()).save(out / f"{name}.png")


whole = [-0.01, 0.0, 0.15]
for n, az, el in (("front", 180, 5), ("side", 90, 3), ("rear", 0, 8), ("three_quarter", 140, -12)):
    shot(n, az, el, whole, 0.62)
head = d.body("jaw_soft").xpos.copy() + [0.02, 0, -0.005]
shot("mouth_closeup", 125, -8, tip + [0.0, 0, 0.012], 0.17)
shot("mouth_side_closeup", 90, 0, head, 0.20)
shot("collision_proxies_side", 90, 3, whole, 0.62, groups=(2, 3))
shot("collision_proxies_rear_threequarter", 35, -15, whole, 0.62, groups=(2, 3))
print("saved to", out)
