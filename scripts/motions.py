r"""motions.py -- run the official Microduck ONNX policies on the Duckling V3 model in MuJoCo.

Usage:
  .venv\Scripts\python.exe scripts\motions.py <motion> [--view] [--gif] [--kp-scale 1.0]
Motions:
  walk        walk forward 8 s on flat ground
  obstacles   walk over planks of rising height (--heights in mm)
  fall_getup  shove it over (front, back), let the policy recover
  sitstand    sit down, hold, stand up
  roulade     forward roll over the head
  pick        crouch and touch the ground with the beak
  dance       choreography: sway, bob, head moves, spin (velstand policy commands)
  all         every motion above, summary table in results\motions\summary.txt

Policy contract (from microduck_rl/scripts/infer_policy.py): 50 Hz control, 200 Hz physics,
61-D observation = gyro(3) + projected gravity(3) + joint pos rel. STAND(14) + joint vel(14)
+ last action(14) + command(13: twist 3, head 4, body pose 6); target = STAND + action.
Servos here are the model's plain PD actuators (upstream trains with the BAM XL330 model).
"""
import argparse, math, sys
from pathlib import Path
import mujoco, numpy as np, onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]
POL = ROOT / "policies"
OUT = ROOT / "results" / "motions"
SCENE = ROOT / "model" / "duck_v3_gc_scene.xml"


def make_obstacle_scene(heights_mm, spacing=0.30, first=0.35):
    """Scene with planks across the path (full width, 30 mm deep) at rising heights."""
    txt = SCENE.read_text()
    geoms = ""
    for i, h in enumerate(heights_mm):
        x = first + i * spacing
        geoms += (f'<geom name="plank{i}" type="box" size="0.015 0.4 {h/2000:.5f}" pos="{x:.3f} 0 {h/2000:.5f}" '
                  f'rgba="0.85 0.55 0.2 1"/>\n')
    txt = txt.replace("</worldbody>", geoms + "</worldbody>", 1)
    path = SCENE.parent / "duck_v3_gc_obstacles.xml"
    path.write_text(txt)
    return path


class Duck:
    def __init__(self, xml=SCENE, kp_scale=1.0, record=False, view=False):
        self.m = mujoco.MjModel.from_xml_path(str(xml))
        self.m.opt.timestep = 0.005
        self.m.actuator_gainprm[:14, 0] *= kp_scale
        self.m.actuator_biasprm[:14, 1] *= kp_scale
        self.d = mujoco.MjData(self.m)
        key = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")
        mujoco.mj_resetDataKeyframe(self.m, self.d, key)
        self.default = self.m.key_ctrl[key][:14].copy().astype(np.float32)
        jid = [self.m.actuator_trnid[i, 0] for i in range(14)]
        self.qadr = np.array([self.m.jnt_qposadr[j] for j in jid])
        self.vadr = np.array([self.m.jnt_dofadr[j] for j in jid])
        self.gyro = self.m.sensor_adr[self.m.sensor("gyro").id]
        self.trunk = self.m.body("trunk_base").id
        self.push_x = self.m.actuator("push_x").id
        self.last = np.zeros(14, np.float32)
        self.cmd = np.zeros(13, np.float32)
        self.sessions = {}
        self.policy = None
        self.frames, self.renderer, self.viewer = [], None, None
        if record:
            self.renderer = mujoco.Renderer(self.m, 240, 320)
            self.cam = mujoco.MjvCamera(); self.cam.distance = 0.75; self.cam.azimuth = 120; self.cam.elevation = -15
        if view:
            from mujoco import viewer as mj_viewer
            self.viewer = mj_viewer.launch_passive(self.m, self.d)
        self.steps = 0
        mujoco.mj_forward(self.m, self.d)

    def use(self, name):
        if name not in self.sessions:
            self.sessions[name] = ort.InferenceSession(str(POL / f"{name}.onnx"), providers=["CPUExecutionProvider"])
        self.policy = self.sessions[name]

    def up_z(self):
        return float(self.d.xmat[self.trunk].reshape(3, 3)[2, 2])

    def upright(self):
        return self.up_z() > 0.8 and self.d.qpos[2] > 0.085

    def obs(self):
        q = self.d.xquat[self.trunk]
        g = np.array([0.0, 0.0, -1.0]); t = np.cross(q[1:], g) * 2
        grav = g - q[0] * t + np.cross(q[1:], t)
        return np.concatenate([self.d.sensordata[self.gyro:self.gyro + 3], grav,
                               self.d.qpos[self.qadr] - self.default, self.d.qvel[self.vadr],
                               self.last, self.cmd]).astype(np.float32)[None]

    def step(self, push=0.0):
        """One 50 Hz control step (4 physics steps)."""
        inp = self.policy.get_inputs()[0].name
        self.last = self.policy.run(None, {inp: self.obs()})[0][0].astype(np.float32)
        self.d.ctrl[:14] = self.default + self.last
        self.d.ctrl[self.push_x] = push
        for _ in range(4):
            mujoco.mj_step(self.m, self.d)
        self.steps += 1
        if self.renderer and self.steps % 2 == 0:                  # 25 fps
            self.cam.lookat[:] = [self.d.qpos[0], self.d.qpos[1], 0.10]
            self.renderer.update_scene(self.d, self.cam); self.frames.append(self.renderer.render().copy())
        if self.viewer:
            import time
            self.viewer.sync(); time.sleep(0.02)

    def run(self, seconds, push=0.0, each=None):
        for i in range(round(seconds * 50)):
            if each: each(i / 50.0)
            self.step(push)

    def save(self, name):
        if not self.frames: return
        from PIL import Image
        OUT.mkdir(parents=True, exist_ok=True)
        imgs = [Image.fromarray(f) for f in self.frames]
        imgs[0].save(OUT / f"{name}.gif", save_all=True, append_images=imgs[1:], duration=40, loop=0)
        n = 12; idx = np.linspace(0, len(imgs) - 1, n).astype(int)
        sheet = Image.new("RGB", (320 * 6, 240 * 2))
        for k, i in enumerate(idx):
            sheet.paste(imgs[i], (320 * (k % 6), 240 * (k // 6)))
        sheet.save(OUT / f"{name}_sheet.png")


# ----------------------------------------------------------------------------- motions
def m_walk(a):
    d = Duck(kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("velstand")
    d.run(1.0); d.cmd[0] = a.speed; d.run(8.0); d.cmd[0] = 0; d.run(1.0)
    d.save("walk")
    dist = d.d.qpos[0]
    return d.upright(), f"walked {dist:.2f} m in 8 s at command {a.speed} m/s ({dist/8:.3f} m/s), drift y {d.d.qpos[1]:+.2f} m"


def m_obstacles(a):
    hs = a.heights
    d = Duck(make_obstacle_scene(hs), kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("velstand")
    d.run(1.0); d.cmd[0] = a.obstacle_speed
    t, cleared = 0.0, 0
    while t < 30 and d.upright() and d.d.qpos[0] < 0.35 + 0.30 * len(hs):
        d.cmd[2] = float(np.clip(-2.0 * math.atan2(2 * (d.d.qpos[3] * d.d.qpos[6]), 1 - 2 * d.d.qpos[6] ** 2)
                                 - 2.0 * d.d.qpos[1], -1, 1))     # hold heading and the centre line
        d.step(); t += 0.02
        cleared = sum(d.d.qpos[0] > 0.35 + 0.30 * i + 0.06 for i in range(len(hs)))
    d.cmd[:] = 0; d.run(1.0); d.save("obstacles")
    done = [f"{h:g}" for h in hs[:cleared]]
    return d.upright() and cleared == len(hs), (f"cleared {cleared}/{len(hs)} planks (heights mm: {', '.join(done) or 'none'}) "
                                                 f"in {t:.1f} s, reached x = {d.d.qpos[0]:.2f} m, {'upright' if d.upright() else 'fell'}")


def m_fall_getup(a):
    d = Duck(kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("velstand")
    notes, ok = [], True
    for label, f in (("pushed forward", 12.0), ("pushed backward", -12.0)):
        d.run(1.0); d.run(0.2, push=f)
        t, fell, t_up = 0.0, False, None
        while t < 12:
            d.step(); t += 0.02
            if not fell and (d.up_z() < 0.5 or d.d.qpos[2] < 0.07): fell = True
            if fell and d.upright():
                t_up = t; break
        if not fell:
            notes.append(f"{label}: stayed up (push absorbed)")
        elif t_up:
            d.run(2.0)
            still = d.upright(); ok &= still
            notes.append(f"{label}: fell, back on its feet after {t_up:.1f} s" + ("" if still else " but fell again"))
        else:
            ok = False; notes.append(f"{label}: fell and did NOT get up in 12 s")
    d.save("fall_getup")
    return ok, "; ".join(notes)


def m_sitstand(a):
    d = Duck(kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("alpha_sitstand")
    d.run(1.5); z0 = d.d.qpos[2]
    d.cmd[0] = 1.0; d.run(4.0); z_sit = d.d.qpos[2]
    d.cmd[0] = 0.0; d.run(4.0); z_up = d.d.qpos[2]
    d.save("sitstand")
    return d.upright() and z_sit < z0 - 0.02 and z_up > z0 - 0.01, f"torso height stand {z0:.3f} -> sit {z_sit:.3f} -> stand {z_up:.3f} m"


def m_roulade(a):
    d = Duck(kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("velstand"); d.run(1.5)
    x0 = d.d.qpos[0]; d.use("roulade"); d.cmd[:] = 0
    min_up, t = 1.0, 0.0
    while t < 2.0:
        d.step(); t += 0.02; min_up = min(min_up, d.up_z())
    d.use("velstand"); d.run(4.0); d.save("roulade")
    return d.upright() and min_up < -0.5, f"went {'fully over' if min_up < -0.5 else 'NOT over'} (min up-axis {min_up:+.2f}), moved {d.d.qpos[0]-x0:+.2f} m, ends {'upright' if d.upright() else 'down'}"


def m_pick(a):
    d = Duck(kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("velstand"); d.run(1.5)
    d.use("alpha_ground_pick"); tip = d.m.site("mouth_tip").id
    lowest, phase = 1.0, 0.0
    while phase < 0.7:
        d.cmd[0], d.cmd[1] = math.cos(2 * math.pi * phase), math.sin(2 * math.pi * phase)
        d.step(); phase += 0.02 / 4.0; lowest = min(lowest, d.d.site_xpos[tip][2])
    d.cmd[:] = 0; d.use("velstand"); d.run(2.0); d.save("pick")
    return d.upright() and lowest < 0.03, f"beak tip came down to {lowest*1000:.0f} mm above the floor, ends {'upright' if d.upright() else 'down'}"


def m_dance(a):
    """Choreography on the velstand policy: body roll/pitch sway, bob, head moves, then a spin."""
    d = Duck(kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("velstand")
    beat = 0.5                                                     # s per beat, 120 bpm
    rng = dict(roll=0.0, pitch=0.0, yaw=0.0)

    def choreo(t):
        c = d.cmd; c[:] = 0
        ph = 2 * math.pi * t / (2 * beat)
        if t < 4:                      # side sway + head counter-roll
            c[10] = 0.15 * math.sin(ph); c[6] = -0.3 * math.sin(ph)
        elif t < 8:                    # nod: body pitch + head pitch on the beat
            c[11] = 0.12 * math.sin(ph); c[4] = 0.35 * math.sin(2 * ph)
        elif t < 12:                   # head shake + bob down/up
            c[5] = 0.8 * math.sin(ph); c[9] = -0.015 * (1 - math.cos(2 * ph)) / 2
        elif t < 16:                   # spin on the spot, head looking into the turn
            c[2] = 1.5; c[5] = 0.5
        else:                          # finish: sway while turning back
            c[2] = -1.5 if t < 19 else 0.0; c[10] = 0.12 * math.sin(ph)
        r = d.d.xmat[d.trunk].reshape(3, 3)
        rng["roll"] = max(rng["roll"], abs(math.degrees(math.atan2(r[2, 1], r[2, 2]))))
        rng["pitch"] = max(rng["pitch"], abs(math.degrees(math.asin(max(-1, min(1, -r[2, 0]))))))
    yaw0 = 0.0; total = [0.0, 0.0]

    def each(t):
        choreo(t)
        q = d.d.qpos[3:7]; yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
        dy = (yaw - total[1] + math.pi) % (2 * math.pi) - math.pi; total[0] += abs(dy); total[1] = yaw
    d.run(1.0); d.run(20.0, each=each); d.cmd[:] = 0; d.run(1.0); d.save("dance")
    return d.upright(), f"20 s routine: body roll up to {rng['roll']:.0f} deg, pitch up to {rng['pitch']:.0f} deg, turned {math.degrees(total[0]):.0f} deg in total, ends {'upright' if d.upright() else 'down'}"


MOTIONS = dict(walk=m_walk, obstacles=m_obstacles, fall_getup=m_fall_getup, sitstand=m_sitstand,
               roulade=m_roulade, pick=m_pick, dance=m_dance)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("motion", choices=list(MOTIONS) + ["all"])
    ap.add_argument("--view", action="store_true"); ap.add_argument("--gif", action="store_true")
    ap.add_argument("--kp-scale", type=float, default=1.0, help="multiply servo stiffness (tuning knob)")
    ap.add_argument("--speed", type=float, default=0.3, help="forward command, m/s (the policy stands still below about 0.2)")
    ap.add_argument("--obstacle-speed", type=float, default=0.5,
                    help="forward command on the plank course (tune_obstacles.py: 0.3 stalls at 5 mm, 0.5 clears 12 mm)")
    ap.add_argument("--heights", type=float, nargs="+", default=[3, 5, 8, 10, 12], help="plank heights, mm")
    a = ap.parse_args()
    names = list(MOTIONS) if a.motion == "all" else [a.motion]
    lines = []
    for n in names:
        ok, note = MOTIONS[n](a)
        lines.append(f"{n:11s} {'PASS' if ok else 'FAIL'}  {note}"); print(lines[-1], flush=True)
    if a.motion == "all":
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / "summary.txt").write_text("\n".join(lines) + "\n")
