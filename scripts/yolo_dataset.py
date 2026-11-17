r"""yolo_dataset.py -- render labelled training images for YOLO from the duck's head camera.

Usage:  .venv\Scripts\python.exe scripts\yolo_dataset.py [--scenes 45] [--per-scene 16]
Writes yolo\dataset\{images,labels}\{train,val}\ and yolo\dataset\data.yaml.
Labels come for free: MuJoCo's segmentation render says which geom every pixel belongs to.
Classes: 0 = beacon (the magenta goal cylinder), 1 = box (things on the floor).
The duck's own body is hidden in these renders, as it is in the ROS camera node.
"""
import argparse, math, random
from pathlib import Path
import mujoco, numpy as np
from PIL import Image
import worldgen

ROOT = Path(__file__).resolve().parents[1]
DS = ROOT / "yolo" / "dataset"
W, H = 320, 240

ap = argparse.ArgumentParser()
ap.add_argument("--scenes", type=int, default=45); ap.add_argument("--per-scene", type=int, default=16)
a = ap.parse_args()
rng = random.Random(1)
for split in ("train", "val"):
    (DS / "images" / split).mkdir(parents=True, exist_ok=True); (DS / "labels" / split).mkdir(parents=True, exist_ok=True)
tmp = ROOT / "model" / "_yolo_tmp.xml"
count = {"images": 0, "beacon": 0, "box": 0, "empty": 0}
for s in range(a.scenes):
    xml, (hx, hy) = worldgen.random_world(rng)
    tmp.write_text(xml)
    m = mujoco.MjModel.from_xml_path(str(tmp)); d = mujoco.MjData(m)
    key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")
    cam = m.camera("head_cam").id
    yaw_adr = m.jnt_qposadr[m.joint("head_yaw").id]; pit_adr = m.jnt_qposadr[m.joint("head_pitch").id]
    cls = {}
    for g in range(m.ngeom):
        n = m.geom(g).name
        if n == "beacon": cls[g] = 0
        elif n.startswith("box"): cls[g] = 1
    opt = mujoco.MjvOption(); opt.geomgroup[2] = 0                       # hide the duck's own meshes
    m.vis.headlight.diffuse[:] = rng.uniform(0.4, 0.8); m.vis.headlight.ambient[:] = rng.uniform(0.2, 0.45)
    rgb_r = mujoco.Renderer(m, H, W); seg_r = mujoco.Renderer(m, H, W); seg_r.enable_segmentation_rendering()
    split = "val" if s % 8 == 7 else "train"
    for k in range(a.per_scene):
        mujoco.mj_resetDataKeyframe(m, d, key)
        yaw = rng.uniform(-math.pi, math.pi); tilt = rng.uniform(-0.12, 0.12)
        d.qpos[0], d.qpos[1] = rng.uniform(-hx, hx) * 0.8, rng.uniform(-hy, hy) * 0.8
        bg = [g for g, c in cls.items() if c == 0]
        if bg and rng.random() < 0.5:                                      # half the views look roughly at the beacon
            bx, by = m.geom_pos[bg[0]][:2]
            yaw = math.atan2(by - d.qpos[1], bx - d.qpos[0]) + rng.uniform(-0.5, 0.5)
        d.qpos[3:7] = [math.cos(yaw / 2) * math.cos(tilt / 2), -math.sin(yaw / 2) * math.sin(tilt / 2),
                       math.cos(yaw / 2) * math.sin(tilt / 2), math.sin(yaw / 2) * math.cos(tilt / 2)]
        d.qpos[yaw_adr] = rng.uniform(-0.9, 0.9); d.qpos[pit_adr] += rng.uniform(-0.15, 0.15)
        mujoco.mj_forward(m, d)
        rgb_r.update_scene(d, camera=cam, scene_option=opt); img = rgb_r.render()
        seg_r.update_scene(d, camera=cam, scene_option=opt); seg = seg_r.render()[..., 0]
        lines = []
        for g, c in cls.items():
            ys, xs = np.nonzero(seg == g)
            if len(xs) < 40: continue
            x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
            if x1 - x0 < 5 or y1 - y0 < 5: continue
            lines.append(f"{c} {(x0 + x1) / 2 / W:.5f} {(y0 + y1) / 2 / H:.5f} {(x1 - x0) / W:.5f} {(y1 - y0) / H:.5f}")
            count["beacon" if c == 0 else "box"] += 1
        if not lines:
            count["empty"] += 1
            if rng.random() < 0.6: continue                                # keep some empty images as negatives
        name = f"s{s:03d}_{k:02d}"
        Image.fromarray(img).save(DS / "images" / split / f"{name}.jpg", quality=90)
        (DS / "labels" / split / f"{name}.txt").write_text("\n".join(lines))
        count["images"] += 1
    rgb_r.close(); seg_r.close()
tmp.unlink()
(DS / "data.yaml").write_text(f"path: {DS.as_posix()}\ntrain: images/train\nval: images/val\nnames:\n  0: beacon\n  1: box\n")
print(count)
