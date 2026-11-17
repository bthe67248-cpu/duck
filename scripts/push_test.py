"""push_test.py -- MuJoCo-only cross-check of the Simulink balance loop (no MATLAB needed).

Usage:
  .venv\\Scripts\\python.exe scripts\\push_test.py model\\duck_v3_scene.xml <push N> [--off] [--view]
      [--kth K] [--kd K] [--khip K] [--delay N] [--T seconds]

Same structure as the Simulink Controller: 100 Hz, complementary tilt filter from gyro+accel,
PD on pitch -> ankle offset (opposite sign on the mirrored right ankle).
"""
import argparse, mujoco, numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("xml"); ap.add_argument("force", type=float, nargs="?", default=0.0)
ap.add_argument("--off", action="store_true"); ap.add_argument("--view", action="store_true")
ap.add_argument("--kth", type=float, default=None); ap.add_argument("--kd", type=float, default=None)
ap.add_argument("--khip", type=float, default=0.0)
ap.add_argument("--delay", type=int, default=0); ap.add_argument("--T", type=float, default=6.0)
ap.add_argument("--tpush", type=float, default=0.05); ap.add_argument("--quiet", action="store_true")
a = ap.parse_args()

m = mujoco.MjModel.from_xml_path(a.xml); d = mujoco.MjData(m)
kid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "STAND")
mujoco.mj_resetDataKeyframe(m, d, kid)
d.qpos[2] -= 0.0028                                   # feet just touching the floor
home = m.key_ctrl[kid][:14].copy()
LA, RA, LH, RH = 4, 13, 2, 11                         # ankle / hip-pitch actuator indices
gyro = m.sensor_adr[m.sensor("gyro").id]; acc = m.sensor_adr[m.sensor("accel").id]
push = m.actuator("push_x").id
KTH = 2.5 if a.kth is None else a.kth
KD = 0.20 if a.kd is None else a.kd
dt_c, alpha, th = 0.01, 0.980, 0.0
n_sub = round(dt_c / m.opt.timestep)
buf = [home.copy()] * (a.delay + 1)
log = []
viewer = None
if a.view:
    import mujoco.viewer, time
    viewer = mujoco.viewer.launch_passive(m, d)
mujoco.mj_forward(m, d)
step = 0
while d.time < a.T:
    if step % n_sub == 0:                             # ---- 100 Hz controller
        gy, ax, az = d.sensordata[gyro + 1], d.sensordata[acc], d.sensordata[acc + 2]
        th = alpha * (th + gy * dt_c) + (1 - alpha) * np.arctan2(-ax, az)
        u = home.copy()
        if not a.off:
            delta = float(np.clip(KTH * th + KD * gy, -0.25, 0.25))
            hip = float(np.clip(a.khip * th, -0.25, 0.25))
            u[LA] += delta; u[RA] -= delta
            u[LH] += hip;   u[RH] -= hip
        buf = buf[1:] + [u]
        log.append((d.time, th, d.qpos[2]))
    d.ctrl[:14] = buf[0]
    d.ctrl[push] = a.force if 1.0 <= d.time < 1.0 + a.tpush else 0.0
    mujoco.mj_step(m, d); step += 1
    if viewer:
        viewer.sync(); time.sleep(m.opt.timestep)
log = np.array(log)
fell = d.qpos[2] < 0.08
late = log[log[:, 0] > a.T - 1.0, 1]
if not a.quiet:
    print(f"push {a.force:.2f} N x {a.tpush}s = {a.force*a.tpush:.3f} N.s | balance {'OFF' if a.off else 'ON'} "
          f"kth={KTH} kd={KD} khip={a.khip} delay={a.delay} -> {'FELL' if fell else 'standing'} "
          f"| torso z {d.qpos[2]:.3f} m | pitch last 1 s: mean {np.degrees(late.mean()):+.2f} deg, "
          f"max |.| over run {np.degrees(np.abs(log[:,1]).max()):.2f} deg")
raise SystemExit(1 if fell else 0)
