"""calibrate_odom.py -- measure gait-command -> SI velocity gains for slam_node in MuJoCo (no ROS).
Runs the same 50 Hz policy loop as world_sim for forward, reverse and lateral commands and reports the mean
steady-state body-frame velocity per unit command. These are SIMULATION gains for the upstream velstand policy;
they say nothing about the real Feetech robot.
  python ros2/duck_nav/tools/calibrate_odom.py [--mjcf ...] [--policy ...] [--out results/post_fix/odom_calibration.txt]"""
import argparse
import math
import os

import mujoco
import numpy as np
import onnxruntime as ort

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def run(m, sess, inp, cmd, seconds=12.0, settle=3.0):
    d = mujoco.MjData(m)
    key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")
    mujoco.mj_resetDataKeyframe(m, d, key); mujoco.mj_forward(m, d)
    default = m.key_ctrl[key][:14].astype(np.float32)
    jid = [m.actuator_trnid[i, 0] for i in range(14)]
    qadr = np.array([m.jnt_qposadr[j] for j in jid]); vadr = np.array([m.jnt_dofadr[j] for j in jid])
    gyro = m.sensor_adr[m.sensor("gyro").id]; trunk = m.body("trunk_base").id
    last = np.zeros(14, np.float32); c = np.zeros(13, np.float32); c[0:3] = cmd
    vel = []; fell = False
    for k in range(int(seconds / 0.02)):
        q = d.xquat[trunk]; g = np.array([0.0, 0.0, -1.0]); t = np.cross(q[1:], g) * 2
        grav = g - q[0] * t + np.cross(q[1:], t)
        obs = np.concatenate([d.sensordata[gyro:gyro + 3], grav, d.qpos[qadr] - default, d.qvel[vadr], last, c]).astype(np.float32)[None]
        last = sess.run(None, {inp: obs})[0][0].astype(np.float32)
        d.ctrl[:14] = default + last
        for _ in range(4):
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        if d.qpos[2] < 0.085:
            fell = True; break
        if k * 0.02 >= settle:
            yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
            v = d.qvel[:2]
            vel.append((math.cos(yaw) * v[0] + math.sin(yaw) * v[1], -math.sin(yaw) * v[0] + math.cos(yaw) * v[1]))
    v = np.array(vel) if vel else np.zeros((1, 2))
    return v.mean(0), v.std(0), fell


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", default=os.path.join(ROOT, "ros2", "duck_nav", "mjcf", "duck_v4_gc_world.xml"))
    ap.add_argument("--policy", default=os.path.join(ROOT, "policies", "velstand.onnx"))
    ap.add_argument("--out", default=os.path.join(ROOT, "results", "post_fix", "odom_calibration.txt"))
    a = ap.parse_args()
    m = mujoco.MjModel.from_xml_path(a.mjcf); m.opt.timestep = 0.005
    sess = ort.InferenceSession(a.policy, providers=["CPUExecutionProvider"]); inp = sess.get_inputs()[0].name
    lines = [f"model {os.path.relpath(a.mjcf, ROOT)}, policy {os.path.relpath(a.policy, ROOT)}",
             "command (vx, vy, wz) -> mean body velocity (vx, vy) m/s [std], gain = v / command"]
    for cmd in ((0.5, 0, 0), (0.3, 0, 0), (-0.3, 0, 0), (-0.5, 0, 0), (0, 0.3, 0), (0, -0.3, 0), (0, 0.5, 0)):
        mean, std, fell = run(m, sess, inp, np.array(cmd, np.float32))
        axis = 0 if cmd[0] else 1
        gain = mean[axis] / cmd[axis]
        lines.append(f"  {cmd} -> ({mean[0]:+.3f}, {mean[1]:+.3f}) [{std[0]:.3f}, {std[1]:.3f}]  gain {gain:.3f}"
                     + ("  FELL" if fell else ""))
    txt = "\n".join(lines); print(txt)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        f.write(txt + "\n")


if __name__ == "__main__":
    main()
