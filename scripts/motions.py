r"""motions.py -- run the official Microduck ONNX policies on the Duckling V4 model in MuJoCo, with pass/fail criteria.

Usage:
  .venv\Scripts\python.exe scripts\motions.py <motion> [--view] [--gif] [--kp-scale 1.0] [--servo-delay S] [--record DIR]
                                                     [--walk-policy policies/duck_v4_walk.onnx] [--out DIR]
Motions:
  balance     stand 10 s at zero command: tilt, height, drift, settling, joint-limit margin, actuator saturation
  walk        walk forward 8 s: minimum travel, drift, heading, continuous fall monitoring
  obstacles   walk over planks of rising height (--heights in mm)
  fall_getup  shove it over (front, back) with escalating pushes; "got up" only after a CONFIRMED fall
  sitstand    sit down, hold, stand up
  roulade     forward roll over the head
  pick        crouch and touch the ground with the bill (scored from the new `mouth_tip` site + bill contact)
  dance       choreography: sway, bob, head moves, spin (velstand policy commands)
  all         every motion above, summary table in results\post_fix\motions\summary.txt

Exit status: 0 all PASS; 1 any FAIL; 3 no FAIL but at least one INCONCLUSIVE (a test that did not exercise what it
claims to test, e.g. a push that never made the robot fall). Results of the earlier, pre-fix model are in results\pre_fix.

Policy contract (from microduck_rl/scripts/infer_policy.py): 50 Hz control, 200 Hz physics,
61-D observation = gyro(3) + projected gravity(3) + joint pos rel. STAND(14) + joint vel(14)
+ last action(14) + command(13: twist 3, head 4, body pose 6); target = STAND + action.
Servos here are the model's plain PD actuators (upstream trains with the BAM XL330 model). They act on each target
`servo_delay_s` after the policy issued it (the model's <custom> value: 20 ms ESTIMATE or the measured calibration
value; --servo-delay overrides it; ros2/duck_nav/duck_nav/actuation.py). --record DIR saves each motion's qpos at
50 Hz to DIR/<motion>.npz for design/v4/check_v4_fit.py --traj.

Fall monitor (Duck.mon, sampled every 50 Hz control step, latched): FALL when torso up-axis z < 0.5 (tilt > 60 deg),
or torso height < 0.07 m, or a head-upper/head-lower/body collision proxy touches the floor for >= 3 consecutive
steps (training/duck_env.py uses the same set and persistence). The head_upper hull includes the upper bill, so a
very deep pick could register as a fall; the jaw (lower bill) proxy is deliberately not a fall indicator (picking uses it). Final uprightness alone is never used.
"""
import argparse, math, os, sys
from collections import Counter
from pathlib import Path
import mujoco, numpy as np, onnxruntime as ort

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2" / "duck_nav"))
from duck_nav.actuation import ServoDelay, model_servo_delay  # noqa: E402
POL = ROOT / "policies"
OUT = ROOT / "results" / "post_fix" / "motions"
SCENE = ROOT / "model" / "duck_v4_gc_scene.xml"
DISABLE = None          # None | "body" | "head" | "all": switch off those S1 collision proxies (A/B diagnosis only)
SERVO_DELAY = None      # s; None = the model's servo_delay_s
RECORD = None           # directory for the qpos recordings (--record)
GIF_W, GIF_H = 320, 240 # --gif-size
WALK_POLICY = None      # path of an ONNX that replaces velstand (--walk-policy), e.g. training/runs/<run>/policy.onnx
BODY_FALL_GEOMS = {"head_upper_collision", "head_lower_collision",
                   "body_front_upper_collision", "body_rear_upper_collision", "body_board_bay_collision"}


def make_obstacle_scene(heights_mm, spacing=0.30, first=0.35):
    """Scene with planks across the path (full width, 30 mm deep) at rising heights."""
    txt = SCENE.read_text()
    geoms = ""
    for i, h in enumerate(heights_mm):
        x = first + i * spacing
        geoms += (f'<geom name="plank{i}" type="box" size="0.015 0.4 {h/2000:.5f}" pos="{x:.3f} 0 {h/2000:.5f}" '
                  f'rgba="0.85 0.55 0.2 1"/>\n')
    txt = txt.replace("</worldbody>", geoms + "</worldbody>", 1)
    path = SCENE.parent / "duck_v4_gc_obstacles.xml"
    path.write_text(txt)
    return path


class Monitor:
    """Continuous per-control-step record of falls, saturation, joint margins and contacts."""
    def __init__(self, duck):
        self.dk = duck; self.reset()

    def reset(self):
        self.fell = False; self.t_fell = None; self.why = ""
        self.min_up, self.min_z, self.max_z = 1.0, 9.0, 0.0
        self.sat_run = self.max_sat_run = 0
        self.min_margin = 9.0
        self.floor_touch = Counter()            # geom name -> control steps touching the floor (non-foot only)
        self.self_pairs = Counter()             # robot-robot contacts involving proxies
        self._body_run = 0

    def update(self):
        dk, m, d = self.dk, self.dk.m, self.dk.d
        up, z = dk.up_z(), float(d.qpos[2])
        self.min_up = min(self.min_up, up); self.min_z = min(self.min_z, z); self.max_z = max(self.max_z, z)
        why = ""
        if up < 0.5: why = f"tilt>{math.degrees(math.acos(0.5)):.0f}deg"
        elif z < 0.07: why = "torso below 0.07 m"
        f = np.abs(d.actuator_force[:14]); lim = m.actuator_forcerange[:14, 1]
        if np.any(f >= 0.98 * lim): self.sat_run += 1
        else: self.sat_run = 0
        self.max_sat_run = max(self.max_sat_run, self.sat_run)
        q = d.qpos[dk.qadr]; lo, hi = dk.jrange[:, 0], dk.jrange[:, 1]
        self.min_margin = min(self.min_margin, float(np.min(np.minimum(q - lo, hi - q))))
        body_now = False
        for c in d.contact[:d.ncon]:
            g1, g2 = c.geom1, c.geom2
            n1, n2 = dk.gname[g1], dk.gname[g2]
            if g1 == dk.floor or g2 == dk.floor:
                other = n2 if g1 == dk.floor else n1
                if other in ("left_foot_collision", "right_foot_collision"): continue
                self.floor_touch[other] += 1
                if other in BODY_FALL_GEOMS: body_now = True
            elif (n1 in dk.proxy_names) or (n2 in dk.proxy_names):
                self.self_pairs[tuple(sorted((n1, n2)))] += 1
        self._body_run = self._body_run + 1 if body_now else 0
        if self._body_run >= 3 and not why: why = "head/body proxy on the floor"
        if why and not self.fell:
            self.fell, self.t_fell, self.why = True, float(d.time), why


class Duck:
    def __init__(self, xml=None, kp_scale=1.0, record=False, view=False):
        self.m = mujoco.MjModel.from_xml_path(str(xml or SCENE))
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
        self.jrange = np.array([self.m.jnt_range[j] for j in jid])
        self.gyro = self.m.sensor_adr[self.m.sensor("gyro").id]
        self.trunk = self.m.body("trunk_base").id
        self.push_x = self.m.actuator("push_x").id
        self.floor = self.m.geom("floor").id
        self.gname = [self.m.geom(g).name or f"{self.m.body(self.m.geom_bodyid[g]).name}/geom{g}" for g in range(self.m.ngeom)]
        self.proxy_names = {n for n in self.gname if n.split("_")[0] in ("head", "bill", "jaw", "body") and n.endswith("_collision")}
        for n in self.proxy_names:                                 # A/B switch, never on by default
            if DISABLE == "all" or (DISABLE == "body" and n.startswith("body")) or (DISABLE == "head" and not n.startswith("body")):
                self.m.geom_contype[self.m.geom(n).id] = 0; self.m.geom_conaffinity[self.m.geom(n).id] = 0
        self.last = np.zeros(14, np.float32)
        self.cmd = np.zeros(13, np.float32)
        self.servo = ServoDelay(model_servo_delay(self.m) if SERVO_DELAY is None else SERVO_DELAY,
                                self.m.opt.timestep, self.default)
        self.traj = []
        self.sessions = {}
        self.policy = None
        self.mon = Monitor(self)
        self.frames, self.renderer, self.viewer = [], None, None
        if record:
            self.m.vis.global_.offwidth = max(self.m.vis.global_.offwidth, GIF_W)
            self.m.vis.global_.offheight = max(self.m.vis.global_.offheight, GIF_H)
            self.renderer = mujoco.Renderer(self.m, GIF_H, GIF_W)
            self.cam = mujoco.MjvCamera(); self.cam.distance = 0.75; self.cam.azimuth = 120; self.cam.elevation = -15
        if view:
            from mujoco import viewer as mj_viewer
            self.viewer = mj_viewer.launch_passive(self.m, self.d)
        self.steps = 0
        mujoco.mj_forward(self.m, self.d)

    def use(self, name):
        if name not in self.sessions:
            path = WALK_POLICY if (name == "velstand" and WALK_POLICY) else POL / f"{name}.onnx"
            self.sessions[name] = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])
        self.policy = self.sessions[name]

    def up_z(self):
        return float(self.d.xmat[self.trunk].reshape(3, 3)[2, 2])

    def tilt_deg(self):
        return math.degrees(math.acos(max(-1.0, min(1.0, self.up_z()))))

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
        target = self.default + self.last
        self.d.ctrl[self.push_x] = push
        for _ in range(4):
            self.d.ctrl[:14] = self.servo.step(target)          # servos act on the target issued servo_delay ago
            mujoco.mj_step(self.m, self.d)
        self.steps += 1
        if RECORD:
            self.traj.append(self.d.qpos.copy())
        self.mon.update()
        if self.renderer and self.steps % 2 == 0:                  # 25 fps
            self.cam.lookat[:] = [self.d.qpos[0], self.d.qpos[1], 0.10]
            self.renderer.update_scene(self.d, self.cam)
            # no shadows/reflections: software (headless) GL draws them as streaks
            self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
            self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
            self.frames.append(self.renderer.render().copy())
        if self.viewer:
            import time
            self.viewer.sync(); time.sleep(0.02)

    def run(self, seconds, push=0.0, each=None):
        for i in range(round(seconds * 50)):
            if each: each(i / 50.0)
            self.step(push)

    def save(self, name):
        if RECORD and self.traj:
            Path(RECORD).mkdir(parents=True, exist_ok=True)
            np.savez_compressed(Path(RECORD) / f"{name}.npz", qpos=np.array(self.traj), dt=0.02,
                                servo_delay_s=self.servo.delay_s, robot_mass_kg=float(self.m.body_subtreemass[1]))
        if not self.frames: return
        from PIL import Image
        OUT.mkdir(parents=True, exist_ok=True)
        imgs = [Image.fromarray(f) for f in self.frames]
        imgs[0].save(OUT / f"{name}.gif", save_all=True, append_images=imgs[1:], duration=40, loop=0)
        n = 12; idx = np.linspace(0, len(imgs) - 1, n).astype(int)
        sheet = Image.new("RGB", (GIF_W * 6, GIF_H * 2))
        for k, i in enumerate(idx):
            sheet.paste(imgs[i], (GIF_W * (k % 6), GIF_H * (k // 6)))
        sheet.save(OUT / f"{name}_sheet.png")


def yaw_of(d):
    q = d.d.qpos[3:7]
    return math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))


def self_note(d):
    """robot-robot proxy contacts seen during the monitored phase (head vs body etc.)"""
    if not d.mon.self_pairs:
        return ""
    top = ", ".join(f"{a}~{b}x{n}" for (a, b), n in d.mon.self_pairs.most_common(3))
    return f"; self-contact steps: {top}"


# ----------------------------------------------------------------------------- motions
# Each returns (status, note); status is True/False or "PASS"/"FAIL"/"INCONCLUSIVE".
BALANCE = dict(max_tilt_deg=10.0, min_height_frac=0.87, max_drift_m=0.10, settle_deg=3.0, settle_by_s=4.0,
               min_joint_margin_rad=0.02, max_sat_run_s=1.0)
WALK = dict(min_travel_m=0.8, max_lateral_m=0.30, max_heading_deg=45.0)


def m_balance(a):
    d = Duck(kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("velstand")
    d.run(1.0)                                              # let the start pose settle; criteria use t >= 1 s
    d.mon.reset(); z0 = float(d.d.qpos[2]); p0 = d.d.qpos[:2].copy(); tilts = []
    d.run(10.0, each=lambda t: tilts.append(d.tilt_deg()))
    tilts = np.array(tilts); t = np.arange(len(tilts)) / 50.0
    c = BALANCE
    drift = float(np.linalg.norm(d.d.qpos[:2] - p0))
    final = float(np.mean(tilts[-100:]))
    bad = np.where(np.abs(tilts - final) > c["settle_deg"])[0]
    t_settle = float(t[bad[-1]] + 0.02) if len(bad) else 0.0
    checks = {
        f"tilt max {tilts.max():.1f} <= {c['max_tilt_deg']} deg": tilts.max() <= c["max_tilt_deg"],
        f"min height {d.mon.min_z*1000:.0f} mm >= {c['min_height_frac']*z0*1000:.0f} mm": d.mon.min_z >= c["min_height_frac"] * z0,
        f"drift {drift:.3f} <= {c['max_drift_m']} m": drift <= c["max_drift_m"],
        f"settled within {c['settle_deg']} deg by {t_settle:.1f} s (<= {c['settle_by_s']})": t_settle <= c["settle_by_s"],
        f"joint margin {d.mon.min_margin:.3f} >= {c['min_joint_margin_rad']} rad": d.mon.min_margin >= c["min_joint_margin_rad"],
        f"longest actuator saturation {d.mon.max_sat_run*0.02:.2f} <= {c['max_sat_run_s']} s": d.mon.max_sat_run * 0.02 <= c["max_sat_run_s"],
        "no fall": not d.mon.fell,
    }
    d.save("balance")
    failed = [k for k, v in checks.items() if not v]
    return (not failed), ("; ".join(("FAILED: " + k) if k in failed else k for k in checks) + self_note(d))


def m_walk(a):
    d = Duck(kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("velstand")
    d.run(1.0); p0 = d.d.qpos[:2].copy(); yaw0 = yaw_of(d); d.mon.reset()
    d.cmd[0] = a.speed; d.run(8.0); d.cmd[0] = 0; d.run(1.0)
    d.save("walk")
    dp = d.d.qpos[:2] - p0
    c, th = WALK, np.array([math.cos(yaw0), math.sin(yaw0)])
    travel = float(dp @ th); lateral = abs(float(-dp[0] * th[1] + dp[1] * th[0]))
    dyaw = abs((math.degrees(yaw_of(d) - yaw0) + 180) % 360 - 180)
    checks = {
        f"travel {travel:.2f} m >= {c['min_travel_m']}": travel >= c["min_travel_m"],
        f"lateral drift {lateral:.2f} <= {c['max_lateral_m']} m": lateral <= c["max_lateral_m"],
        f"heading change {dyaw:.0f} <= {c['max_heading_deg']:.0f} deg": dyaw <= c["max_heading_deg"],
        f"no fall during run (min up-axis {d.mon.min_up:.2f}, min height {d.mon.min_z*1000:.0f} mm" +
        (f", FELL at {d.mon.t_fell:.1f} s: {d.mon.why}" if d.mon.fell else "") + ")": not d.mon.fell,
        "ends upright": d.upright(),
    }
    failed = [k for k, v in checks.items() if not v]
    return (not failed), (f"command {a.speed} m/s for 8 s = {a.speed*8:.1f} m commanded, {travel/8:.3f} m/s achieved; " +
                          "; ".join(("FAILED: " + k) if k in failed else k for k in checks) + self_note(d))


def m_obstacles(a):
    hs = a.heights
    d = Duck(make_obstacle_scene(hs), kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("velstand")
    d.run(1.0); d.mon.reset(); d.cmd[0] = a.obstacle_speed
    t, cleared = 0.0, 0
    while t < 30 and not d.mon.fell and d.d.qpos[0] < 0.35 + 0.30 * len(hs):
        d.cmd[2] = float(np.clip(-2.0 * math.atan2(2 * (d.d.qpos[3] * d.d.qpos[6]), 1 - 2 * d.d.qpos[6] ** 2)
                                 - 2.0 * d.d.qpos[1], -1, 1))     # hold heading and the centre line
        d.step(); t += 0.02
        cleared = sum(d.d.qpos[0] > 0.35 + 0.30 * i + 0.06 for i in range(len(hs)))
    d.cmd[:] = 0; d.run(1.0); d.save("obstacles")
    done = [f"{h:g}" for h in hs[:cleared]]
    ok = d.upright() and cleared == len(hs) and not d.mon.fell
    return ok, (f"cleared {cleared}/{len(hs)} planks (heights mm: {', '.join(done) or 'none'}) "
                f"in {t:.1f} s, reached x = {d.d.qpos[0]:.2f} m, {'upright' if d.upright() else 'down'}"
                + (f", FELL at {d.mon.t_fell:.1f} s ({d.mon.why})" if d.mon.fell else "") + self_note(d))


PUSHES_N = (12.0, 20.0, 30.0, 45.0)      # 0.2 s each; the last value is a SEARCH CAP, not a failure threshold


def m_fall_getup(a):
    d = Duck(kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("velstand")
    notes, results = [], []
    for label, sign in (("forward", 1.0), ("backward", -1.0)):
        outcome = None
        for F in PUSHES_N:
            d.run(1.5); d.mon.reset(); d.run(0.2, push=sign * F)
            t, t_up = 0.0, None
            while t < 12:
                d.step(); t += 0.02
                if d.mon.fell and d.upright() and t_up is None:     # confirmed fall FIRST, then back on its feet
                    t_up = t
                if t_up is not None:
                    break
            if not d.mon.fell:
                continue                                              # push absorbed: escalate
            if t_up:
                d.run(2.0)
                if d.upright() and d.up_z() > 0.9:
                    outcome = ("PASS", f"{label}: {F:g} N x 0.2 s caused a CONFIRMED fall ({d.mon.why} at t={d.mon.t_fell:.2f} s); "
                                       f"got up after {t_up:.1f} s")
                else:
                    outcome = ("FAIL", f"{label}: fell at {F:g} N, up after {t_up:.1f} s but not stably upright 2 s later")
            else:
                outcome = ("FAIL", f"{label}: fell at {F:g} N ({d.mon.why}) and did NOT get up in 12 s")
            break
        if outcome is None:
            outcome = ("INCONCLUSIVE", f"{label}: push absorbed at every level up to {PUSHES_N[-1]:g} N x 0.2 s "
                                        f"(search cap, not a limit) -- recovery NOT exercised")
        results.append(outcome[0]); notes.append(outcome[1])
    d.save("fall_getup")
    status = "FAIL" if "FAIL" in results else ("INCONCLUSIVE" if "INCONCLUSIVE" in results else "PASS")
    return status, "; ".join(notes)


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


PICK_TIP_MAX_M = 0.03


def m_pick(a):
    """Scored from the physical bill tip: the NEW `mouth_tip` site (lowest-front bill point), plus whether the bill's
    collision proxy touched the floor. The old upstream tip position is not used."""
    d = Duck(kp_scale=a.kp_scale, record=a.gif, view=a.view); d.use("velstand"); d.run(1.5)
    d.use("alpha_ground_pick"); tip = d.m.site("mouth_tip").id; d.mon.reset()
    lowest, phase = 1.0, 0.0
    while phase < 0.7:
        d.cmd[0], d.cmd[1] = math.cos(2 * math.pi * phase), math.sin(2 * math.pi * phase)
        d.step(); phase += 0.02 / 4.0; lowest = min(lowest, d.d.site_xpos[tip][2])
    d.cmd[:] = 0; d.use("velstand"); d.run(2.0); d.save("pick")
    bill_steps = d.mon.floor_touch.get("jaw_collision", 0)
    # a "fall" here is only a body-proxy-on-floor / tilt event; crouching is expected to tilt the torso a lot
    ok = d.upright() and lowest < PICK_TIP_MAX_M and not d.mon.fell
    return ok, (f"bill tip (mouth_tip site) came down to {lowest*1000:.0f} mm above the floor (need < {PICK_TIP_MAX_M*1000:.0f}); "
                f"bill proxy touched the floor for {bill_steps} control steps; "
                f"{'no fall' if not d.mon.fell else 'FELL at %.1f s (%s)' % (d.mon.t_fell, d.mon.why)}; "
                f"ends {'upright' if d.upright() else 'down'}" + self_note(d))


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
    total = [0.0, 0.0]

    def each(t):
        choreo(t)
        yaw = yaw_of(d)
        dy = (yaw - total[1] + math.pi) % (2 * math.pi) - math.pi; total[0] += abs(dy); total[1] = yaw
    d.run(1.0); d.mon.reset(); d.run(20.0, each=each); d.cmd[:] = 0; d.run(1.0); d.save("dance")
    ok = d.upright() and not d.mon.fell
    return ok, (f"20 s routine: body roll up to {rng['roll']:.0f} deg, pitch up to {rng['pitch']:.0f} deg, turned {math.degrees(total[0]):.0f} deg in total, "
                f"{'no fall' if not d.mon.fell else 'FELL at %.1f s (%s)' % (d.mon.t_fell, d.mon.why)}, ends {'upright' if d.upright() else 'down'}" + self_note(d))


MOTIONS = dict(balance=m_balance, walk=m_walk, obstacles=m_obstacles, fall_getup=m_fall_getup, sitstand=m_sitstand,
               roulade=m_roulade, pick=m_pick, dance=m_dance)


def label(status):
    return status if isinstance(status, str) else ("PASS" if status else "FAIL")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("motion", choices=list(MOTIONS) + ["all"])
    ap.add_argument("--view", action="store_true"); ap.add_argument("--gif", action="store_true")
    ap.add_argument("--kp-scale", type=float, default=1.0, help="multiply servo stiffness (tuning knob)")
    ap.add_argument("--speed", type=float, default=0.3, help="forward command, m/s (the policy stands still below about 0.2)")
    ap.add_argument("--obstacle-speed", type=float, default=0.5,
                    help="forward command on the plank course (tune_obstacles.py: 0.3 stalls at 5 mm, 0.5 clears 12 mm)")
    ap.add_argument("--heights", type=float, nargs="+", default=[3, 5, 8, 10, 12], help="plank heights, mm")
    ap.add_argument("--xml", help="scene xml to use instead of model/duck_v4_gc_scene.xml (e.g. the pre-fix model for A/B)")
    ap.add_argument("--disable-proxies", choices=["body", "head", "all"], help="diagnosis: switch off these collision proxies")
    ap.add_argument("--out", help="output directory for gifs/summary")
    ap.add_argument("--servo-delay", type=float, help="servo command delay, s (default: the model's servo_delay_s)")
    ap.add_argument("--record", help="save each motion's qpos (50 Hz) to RECORD/<motion>.npz for check_v4_fit.py")
    ap.add_argument("--walk-policy", help="ONNX policy used wherever velstand is (walk, balance, obstacles, get-up, "
                                          "dance, and the stand phases of the others); e.g. policies/duck_v4_walk.onnx")
    ap.add_argument("--gif-size", type=int, nargs=2, default=[320, 240], metavar=("W", "H"), help="GIF frame size")
    a = ap.parse_args()
    GIF_W, GIF_H = a.gif_size
    SERVO_DELAY, RECORD = a.servo_delay, a.record
    WALK_POLICY = Path(a.walk_policy).resolve() if a.walk_policy else None
    if a.xml: SCENE = Path(a.xml).resolve()
    if a.out: OUT = Path(a.out).resolve()
    DISABLE = a.disable_proxies
    names = list(MOTIONS) if a.motion == "all" else [a.motion]
    lines, labels = [], []
    for n in names:
        st, note = MOTIONS[n](a)
        labels.append(label(st))
        lines.append(f"{n:11s} {labels[-1]:12s} {note}"); print(lines[-1], flush=True)
    if a.motion == "all":
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / "summary.txt").write_text("\n".join(lines) + "\n")
    sys.exit(1 if "FAIL" in labels else (3 if "INCONCLUSIVE" in labels else 0))
