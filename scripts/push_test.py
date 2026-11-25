r"""push_test.py -- MuJoCo-only cross-check of the Simulink balance loop (no MATLAB needed).

Usage:
  .venv\Scripts\python.exe scripts\push_test.py model\duck_v4_scene.xml <push N> [--off] [--view]
      [--kth K] [--kd K] [--khip K] [--delay N] [--T seconds] [--search [--cap N]]

Same structure as the Simulink Controller: 100 Hz, complementary tilt filter from gyro+accel,
PD on pitch -> ankle offset (opposite sign on the mirrored right ankle).

FELL is judged CONTINUOUSLY during the run (torso height < 0.08 m or torso tilt > 45 deg at ANY sample), not from the
final state. --search steps the push by 0.5 N up to --cap (default 12 N) in +x and -x and reports "survives A N, falls
at B N", or, if the cap is reached without a fall, "no fall up to the CAP of N N: limit is only BOUNDED BELOW (>= N N)".
Exit status 1 if the single push (or, with --search, the 2 N reference push) fell; 0 otherwise.
"""
import argparse, math, os, sys
from pathlib import Path
import mujoco, numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("xml"); ap.add_argument("force", type=float, nargs="?", default=0.0)
ap.add_argument("--off", action="store_true"); ap.add_argument("--view", action="store_true")
ap.add_argument("--kth", type=float, default=None); ap.add_argument("--kd", type=float, default=None)
ap.add_argument("--khip", type=float, default=0.0)
ap.add_argument("--delay", type=int, default=0); ap.add_argument("--T", type=float, default=6.0)
ap.add_argument("--tpush", type=float, default=0.05); ap.add_argument("--quiet", action="store_true")
ap.add_argument("--search", action="store_true", help="search the push limit in 0.5 N steps (+x and -x)")
ap.add_argument("--cap", type=float, default=12.0, help="upper end of the --search range, N (a bound, not a failure level)")
a = ap.parse_args()

LA, RA, LH, RH = 4, 13, 2, 11                         # ankle / hip-pitch actuator indices
KTH = 2.5 if a.kth is None else a.kth
KD = 0.20 if a.kd is None else a.kd
TILT_FALL_DEG, Z_FALL = 45.0, 0.08


def run(force, view=False):
    m = mujoco.MjModel.from_xml_path(a.xml); d = mujoco.MjData(m)
    kid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "STAND")
    mujoco.mj_resetDataKeyframe(m, d, kid)
    d.qpos[2] -= 0.0028                               # feet just touching the floor
    home = m.key_ctrl[kid][:14].copy()
    gyro = m.sensor_adr[m.sensor("gyro").id]; acc = m.sensor_adr[m.sensor("accel").id]
    push = m.actuator("push_x").id; trunk = m.body("trunk_base").id
    dt_c, alpha, th = 0.01, 0.980, 0.0
    n_sub = round(dt_c / m.opt.timestep)
    buf = [home.copy()] * (a.delay + 1)
    log, fell_at = [], None
    viewer = None
    if view:
        import time; from mujoco import viewer as mj_viewer
        viewer = mj_viewer.launch_passive(m, d)
    mujoco.mj_forward(m, d)
    step = 0
    while d.time < a.T:
        if step % n_sub == 0:                         # ---- 100 Hz controller
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
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, d.xmat[trunk].reshape(3, 3)[2, 2]))))
            if fell_at is None and (d.qpos[2] < Z_FALL or tilt > TILT_FALL_DEG):
                fell_at = d.time
        d.ctrl[:14] = buf[0]
        d.ctrl[push] = force if 1.0 <= d.time < 1.0 + a.tpush else 0.0
        mujoco.mj_step(m, d); step += 1
        if viewer:
            viewer.sync(); time.sleep(m.opt.timestep)
    log = np.array(log)
    late = log[log[:, 0] > a.T - 1.0, 1]
    return fell_at, d.qpos[2], late, log


if a.search:
    ref = run(2.0)[0]
    print(f"reference push 2 N x {a.tpush} s: {'FELL at %.2f s' % ref if ref is not None else 'standing'}")
    for sign, name in ((1, "forward (+x)"), (-1, "backward (-x)")):
        F, last, found = 0.0, 0.0, None
        while F < a.cap:
            F += 0.5
            fell_at, *_ = run(sign * F)
            if fell_at is not None:
                found = F; break
            last = F
        if found is None:
            print(f"push limit {name}: NO FALL up to the search CAP of {a.cap:g} N -> limit only bounded below, >= {last*a.tpush:.3f} N.s (not a failure level)")
        else:
            print(f"push limit {name}: survives {last:.1f} N, falls at {found:.1f} N -> {last*a.tpush:.3f}..{found*a.tpush:.3f} N.s")
    raise SystemExit(1 if ref is not None else 0)

fell_at, z, late, log = run(a.force, a.view)
fell = fell_at is not None
if not a.quiet:
    print(f"push {a.force:.2f} N x {a.tpush}s = {a.force*a.tpush:.3f} N.s | balance {'OFF' if a.off else 'ON'} "
          f"kth={KTH} kd={KD} khip={a.khip} delay={a.delay} -> {'FELL at %.2f s' % fell_at if fell else 'standing'} "
          f"| torso z {z:.3f} m | pitch last 1 s: mean {np.degrees(late.mean()):+.2f} deg, "
          f"max |.| over run {np.degrees(np.abs(log[:,1]).max()):.2f} deg")
raise SystemExit(1 if fell else 0)
