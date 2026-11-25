"""render_png.py -- save a picture of the standing duck to results/duck_v4_stand.png"""
from pathlib import Path
import mujoco
from PIL import Image
ROOT = Path(__file__).resolve().parents[1]
m = mujoco.MjModel.from_xml_path(str(ROOT / "model" / "duck_v4_scene.xml")); d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")); mujoco.mj_forward(m, d)
cam = mujoco.MjvCamera(); cam.lookat[:] = [0, 0, 0.09]; cam.distance = 0.55; cam.azimuth = 150; cam.elevation = -15
r = mujoco.Renderer(m, 480, 640); r.update_scene(d, cam)
(ROOT / "results").mkdir(exist_ok=True)
Image.fromarray(r.render()).save(ROOT / "results" / "duck_v4_stand.png"); print("saved")
