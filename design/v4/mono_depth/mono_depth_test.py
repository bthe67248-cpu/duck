"""mono_depth_test.py -- can a small monocular depth network stand in for (or add to) the A010?

    python design/v4/mono_depth/mono_depth_test.py [--n 40] [--threads 4]

Renders the V4 head camera (320 x 240, 60 deg, as world_sim) at random standing poses in the simulated room with
MuJoCo's exact depth, runs Depth Anything V2 Metric Indoor Small (Hugging Face transformers, CPU) on the RGB image,
and compares metric depth with the truth over the pixels both see within 0.2-3 m. Also times the network at the
model's default input and at a reduced input, with torch limited to `--threads` CPU threads (the UNO Q has four
Cortex-A53 cores; this PC's cores are several times faster, so the board figure is an extrapolation).

Caveat: these are flat-shaded simulator images, not camera photos; the network was trained on real indoor photos, so
the accuracy here is indicative only (it can be better or worse on the real robot).
Writes design/v4/mono_depth/results.json and example.png.
"""
import argparse, json, math, time
from pathlib import Path
import numpy as np, mujoco

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
ap = argparse.ArgumentParser(); ap.add_argument("--n", type=int, default=40); ap.add_argument("--threads", type=int, default=4)
ap.add_argument("--model", default="depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf")
args = ap.parse_args()

import torch
from transformers import AutoImageProcessor, AutoModelForDepthEstimation
torch.set_num_threads(args.threads)
proc = AutoImageProcessor.from_pretrained(args.model)
net = AutoModelForDepthEstimation.from_pretrained(args.model).eval()
n_params = sum(p.numel() for p in net.parameters())

m = mujoco.MjModel.from_xml_path(str(ROOT / "model/duck_v4_gc_world.xml")); d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, m.key("sim_start").id)
cam = m.camera("head_cam").id
W, H = 320, 240
rgb_r = mujoco.Renderer(m, H, W); dep_r = mujoco.Renderer(m, H, W); dep_r.enable_depth_rendering()
opt = mujoco.MjvOption(); opt.geomgroup[2] = 1
rng = np.random.default_rng(0)
room = json.loads((ROOT / "model/world.json").read_text()) if (ROOT / "model/world.json").exists() else {}


def free_pose():
    """Random standing pose inside the room (x 0..4 m, y -1.6..1.6 m) whose camera sees something within 3 m."""
    for _ in range(200):
        x, y, yaw = rng.uniform(-0.3, 4.0), rng.uniform(-1.6, 1.6), rng.uniform(-math.pi, math.pi)
        mujoco.mj_resetDataKeyframe(m, d, m.key("sim_start").id)
        d.qpos[0], d.qpos[1] = x, y
        d.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        mujoco.mj_forward(m, d)
        if d.ncon and any(m.geom_bodyid[c.geom1] == 0 and m.geom(c.geom1).name not in ("floor",) for c in d.contact[:d.ncon]):
            continue                                       # inside an obstacle
        dep_r.update_scene(d, camera=cam, scene_option=opt); z = dep_r.render()
        if np.mean((z > 0.2) & (z < 3.0)) > 0.5:
            return z
    raise RuntimeError("no free pose found")


def run(img, size=None):
    kw = {} if size is None else dict(size={"height": size, "width": size})
    inputs = proc(images=img, return_tensors="pt", **kw)
    with torch.no_grad():
        t0 = time.perf_counter(); out = net(**inputs); dt = time.perf_counter() - t0
    pred = torch.nn.functional.interpolate(out.predicted_depth.unsqueeze(1), size=(H, W), mode="bicubic",
                                           align_corners=False).squeeze().numpy()
    return pred, dt, tuple(inputs["pixel_values"].shape[-2:])


rows = []; example = None
for i in range(args.n):
    z = free_pose()
    rgb_r.update_scene(d, camera=cam, scene_option=opt); img = rgb_r.render()
    img = np.clip(img.astype(np.float32) + rng.normal(0, 4.0, img.shape), 0, 255).astype(np.uint8)   # cam_noise 4
    for size in (None, 266):
        pred, dt, insize = run(img, size)
        ok = (z > 0.2) & (z < 3.0) & np.isfinite(pred) & (pred > 0)
        e = pred[ok] - z[ok]
        scale = float(np.median(z[ok] / pred[ok]))
        rows.append(dict(i=i, variant="reduced" if size else "default", input=f"{insize[0]}x{insize[1]}", ms=dt * 1000, absrel=float(np.mean(np.abs(e) / z[ok])),
                         rmse=float(np.sqrt(np.mean(e ** 2))), median_scale=scale,
                         absrel_scaled=float(np.mean(np.abs(pred[ok] * scale - z[ok]) / z[ok])),
                         within_10pct=float(np.mean(np.abs(e) / z[ok] < 0.10))))
        if example is None and size is None:
            example = (img, z, pred)


def summary(sel):
    a = {k: np.array([r[k] for r in sel]) for k in ("ms", "absrel", "rmse", "median_scale", "absrel_scaled", "within_10pct")}
    return dict(frames=len(sel), input=sel[0]["input"], ms_median=round(float(np.median(a["ms"][1:])), 1),
                absrel_mean=round(float(a["absrel"].mean()), 3), rmse_m=round(float(a["rmse"].mean()), 3),
                median_scale_spread=[round(float(np.percentile(a["median_scale"], 10)), 2), round(float(np.percentile(a["median_scale"], 90)), 2)],
                absrel_after_per_frame_scale=round(float(a["absrel_scaled"].mean()), 3),
                pixels_within_10pct=round(float(a["within_10pct"].mean()), 3))


res = dict(model=args.model, parameters_millions=round(n_params / 1e6, 1), torch_threads=args.threads,
           default_input=summary([r for r in rows if r["variant"] == "default"]),
           reduced_input=summary([r for r in rows if r["variant"] == "reduced"]),
           note="simulator images (flat shading), 320x240 camera, truth depth 0.2-3 m; ms = network forward pass on this PC")
(HERE / "results.json").write_text(json.dumps(res, indent=1)); print(json.dumps(res, indent=1))
try:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    img, z, pred = example
    f, ax = plt.subplots(1, 3, figsize=(12, 3.2))
    ax[0].imshow(img); ax[0].set_title("simulated head camera")
    for a_, im, t in ((ax[1], z, "true depth (m)"), (ax[2], pred, "Depth Anything V2 S (m)")):
        h = a_.imshow(np.clip(im, 0, 3), vmin=0, vmax=3, cmap="viridis"); a_.set_title(t); plt.colorbar(h, ax=a_, fraction=0.035)
    for a_ in ax: a_.axis("off")
    plt.tight_layout(); plt.savefig(HERE / "example.png", dpi=90)
except Exception as exc:
    print("no example image:", exc)
