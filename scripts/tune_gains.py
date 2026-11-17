"""tune_gains.py -- sweep the ankle PD gains against extra delay and pushes (MuJoCo only, fast).

Usage:  .venv\\Scripts\\python.exe scripts\\tune_gains.py [model\\duck_v3_scene.xml]
For every (K_TH, K_D) pair it runs: extra delay 0..4 controller samples with a +2 N and a
-1.5 N push (0.05 s), 10 s each, and counts how many of those runs stay standing.
Python delay N corresponds to Simulink "Extra delay" N-1 (Simulink's rate transitions add one).
"""
import sys
from pathlib import Path
import mujoco, numpy as np

ROOT = Path(__file__).resolve().parents[1]
xml = sys.argv[1] if len(sys.argv) > 1 else str(ROOT / "model" / "duck_v3_scene.xml")
m = mujoco.MjModel.from_xml_path(xml)
kid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")
home = m.key_ctrl[kid][:14].copy()
gyro = m.sensor_adr[m.sensor("gyro").id]; acc = m.sensor_adr[m.sensor("accel").id]
push = m.actuator("push_x").id
n_sub = round(0.01 / m.opt.timestep)


def run(kth, kd, delay, force, T=10.0):
    d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, kid)
    th, buf, peak = 0.0, [home.copy()] * (delay + 1), 0.0
    for step in range(round(T / m.opt.timestep)):
        if step % n_sub == 0:
            gy = d.sensordata[gyro + 1]
            th = 0.98 * (th + gy * 0.01) + 0.02 * np.arctan2(-d.sensordata[acc], d.sensordata[acc + 2])
            u = home.copy(); delta = float(np.clip(kth * th + kd * gy, -0.25, 0.25))
            u[4] += delta; u[13] -= delta
            buf = buf[1:] + [u]
            if d.time > 1.0: peak = max(peak, abs(th))
        d.ctrl[:14] = buf[0]
        d.ctrl[push] = force if 1.0 <= d.time < 1.05 else 0.0
        mujoco.mj_step(m, d)
        if d.qpos[2] < 0.08:
            return False, peak
    # still standing, and not in a growing oscillation at the end
    return True, peak


def limit(kth, kd, sign):
    f = 0.0
    while f < 12 and run(kth, kd, 1, sign * (f + 0.5), T=5.0)[0]:
        f += 0.5
    return f


if __name__ == "__main__":
    print(" K_TH   K_D | stands at delay 0..4 (+2 N / -1.5 N)   | score")
    best = []
    for kth in (1.5, 2.0, 2.5, 3.0, 3.5, 4.21):
        for kd in (0.05, 0.10, 0.15, 0.20, 0.26):
            row, score = [], 0
            for delay in range(5):
                a = run(kth, kd, delay, 2.0)[0]; b = run(kth, kd, delay, -1.5)[0]
                row.append(("Y" if a else "-") + ("Y" if b else "-")); score += a + b
            print(f"{kth:5.2f} {kd:5.2f} | {'  '.join(row)}                 | {score}")
            best.append((score, kth, kd))
    best.sort(reverse=True)
    print("\nTop candidates, with push limits at Python delay 1 (= Simulink nominal):")
    for score, kth, kd in best[:5]:
        print(f"  K_TH={kth} K_D={kd}: score {score}/10, limit forward {limit(kth, kd, 1):.1f} N, backward {limit(kth, kd, -1):.1f} N")
