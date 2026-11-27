"""duck_env.py -- CPU MuJoCo walking environment for the Duckling V4, with the Microduck policy contract.

The policy contract is the one scripts/motions.py and ros2/duck_nav world_sim already run (from upstream
microduck_rl/scripts/infer_policy.py), so a policy trained here drops into both unchanged:
  * 50 Hz control, 200 Hz physics (timestep 0.005 s, 4 physics steps per action);
  * observation (61) = gyro(3) + projected gravity(3) + joint pos rel. STAND(14) + joint vel(14) + last action(14)
    + command(13: twist vx vy wz, head pose 4, body pose 6);
  * action (14) -> servo target = STAND + action (unfiltered), applied after the servo delay.
Joint order is the model's first 14 actuators: left leg 0-4, neck/head 5-8, right leg 9-13.

Robustness (domain randomization, re-applied from stored defaults at every reset, so it never accumulates):
  servo stiffness kp, damping kv, torque limit, command delay, joint damping/friction/armature, every body's mass,
  trunk and head centre of mass, foot friction, IMU mounting tilt, joint encoder bias, observation noise, pushes.
The servo ranges are deliberately wide: the duck's Feetech servos have not been bench-tested yet (TODO.md step 3),
so the policy must work across the plausible range around the model's (XL330) numbers. When scripts/make_duck.py is
re-run with a measured calibration file, the model's servo numbers change and the ranges re-centre on them.

Rewards follow upstream microduck_rl's velocity recipe (velocity tracking, upright, leg pose, foot air time and
clearance, slip, smoothness), plus a self-collision penalty for the ground-contact model, whose leg/body collision
proxies are what made the pretrained walk fail on V4 (README Part D).
"""
import copy
import math
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OBS_DIM, ACT_DIM, CRIT_EXTRA = 61, 14, 9
CTRL_DT, PHYS_DT, SUBSTEPS = 0.02, 0.005, 4
LEG = np.r_[0:5, 9:14]
HEAD = np.r_[5:9]
FALL_GEOMS = {"head_upper_collision", "head_lower_collision", "body_front_upper_collision", "body_rear_upper_collision", "body_board_bay_collision",
              "body_connector_well_collision"}

DEFAULT_CFG = dict(
    episode_s=20.0,
    # commands
    resample_s=(4.0, 8.0), zero_cmd_prob=0.2, turn_in_place_prob=0.1,
    vx=(-0.2, 0.4), vy=(-0.15, 0.15), wz=(-1.0, 1.0), head=0.3, body=0.02,
    # domain randomization (scales are multiplicative around the model's values)
    kp=(0.75, 1.25), kv=(0.7, 1.3), torque=(0.8, 1.2), delay_s=(0.010, 0.040),
    joint_damping=(0.5, 2.0), joint_friction=(0.5, 2.0), armature=(0.8, 1.2),
    mass=(0.9, 1.1), com_m=0.008, foot_friction=(0.5, 1.2), imu_tilt_deg=5.0, encoder_bias=0.015,
    push_interval_s=(3.0, 6.0), push_dv=0.3, init_joint_noise=0.05,
    # fall recovery: this share of episodes starts lying (dropped in a random attitude and settled); in them a fall
    # does not end the episode until the duck has stood upright for `recovered_s`; uprighting pays by potential
    # (progress in up-axis and height, so lying still earns nothing and holding upright earns the normal terms)
    recover_prob=0.15, recovered_s=1.0, w_rise=20.0, max_recover_s=10.0,
    # observation noise (uniform half-widths, as upstream)
    n_gyro=0.03, n_grav=0.01, n_qpos=0.002, n_qvel=0.25,
    # rewards
    w_lin=2.0, s_lin=0.05, w_ang=2.0, s_ang=0.25, w_up=2.0, s_up=0.05, w_pose=1.0, w_head=1.0,
    w_air=3.0, air_min=0.125, air_max=0.30, w_clear=-30.0, clear_h=0.02, w_slip=-0.1, w_rate=-0.1,
    w_angvel=-0.05, w_limits=-1.0, w_self=-1.0, w_torque=-0.002, w_term=-10.0,
)
STD_STAND = np.array([0.1, 0.05, 0.15, 0.15, 0.1] * 2)
STD_WALK = np.array([0.3, 0.05, 0.4, 0.4, 0.25] * 2)


_QC, _OUT = np.zeros(4), np.zeros(3)


def quat_rotate_inv(q, v):
    """Rotate world vector v into the frame of unit quaternion q (w, x, y, z) (MuJoCo's C routine: fast)."""
    mujoco.mju_negQuat(_QC, np.asarray(q, float))
    mujoco.mju_rotVecQuat(_OUT, np.asarray(v, float), _QC)
    return _OUT.copy()


_MODELS = {}


def physics_model(xml):
    """The scene without its visual-only mesh geoms (contype = conaffinity = 0) and their meshes: every body carries an
    explicit <inertial>, so the dynamics are identical, but each environment needs a few MB instead of hundreds.
    Compiled once per process and copied for each environment (each copy is randomized on its own)."""
    if xml not in _MODELS:
        spec = mujoco.MjSpec.from_file(xml)
        for g in [g for g in spec.geoms if g.contype == 0 and g.conaffinity == 0 and g.type == mujoco.mjtGeom.mjGEOM_MESH]:
            spec.delete(g)
        used = {g.meshname for g in spec.geoms if g.meshname}
        for msh in [m for m in spec.meshes if (m.name or Path(m.file).stem) not in used]:
            spec.delete(msh)
        _MODELS[xml] = spec.compile()
    return copy.deepcopy(_MODELS[xml])


class DuckEnv:
    def __init__(self, xml=None, seed=0, cfg=None):
        self.cfg = dict(DEFAULT_CFG, **(cfg or {}))
        self.rng = np.random.default_rng(seed)
        self.m = physics_model(str(xml or ROOT / "model" / "duck_v4_gc_scene.xml"))
        self.m.opt.timestep = PHYS_DT
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")
        self.default = m.key_ctrl[self.key][:ACT_DIM].copy()
        jid = [m.actuator_trnid[i, 0] for i in range(ACT_DIM)]
        self.qadr = np.array([m.jnt_qposadr[j] for j in jid])
        self.vadr = np.array([m.jnt_dofadr[j] for j in jid])
        self.jrange = np.array([m.jnt_range[j] for j in jid])
        self.gyro = m.sensor_adr[m.sensor("gyro").id]
        self.trunk = m.body("trunk_base").id
        self.floor = m.geom("floor").id
        self.feet = [m.geom("left_foot_collision").id, m.geom("right_foot_collision").id]
        self.foot_sites = [m.site("left_foot").id, m.site("right_foot").id]
        names = [m.geom(g).name for g in range(m.ngeom)]
        self.fall_geoms = {g for g, n in enumerate(names) if n in FALL_GEOMS}
        self.proxy_geoms = {g for g, n in enumerate(names) if n.endswith("_collision") and n.split("_")[0] in
                            ("head", "bill", "jaw", "body")}
        leg_bodies = set()
        for g in self.feet:
            b = m.geom_bodyid[g]
            while b > self.trunk:
                leg_bodies.add(b); b = m.body_parentid[b]
        self.leg_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] in leg_bodies
                          and (m.geom_contype[g] or m.geom_conaffinity[g])}
        self.head_bodies = [b for b in range(m.nbody) if m.body(b).name in ("neck", "neck_pitch", "yaw_roll_motion",
                                                                             "jaw_soft", "jaw")]
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "mouth")
        self.mouth = a if a >= 0 else None
        # defaults for non-accumulating randomization
        self.d0 = dict(gain=m.actuator_gainprm[:ACT_DIM, 0].copy(), bias1=m.actuator_biasprm[:ACT_DIM, 1].copy(),
                       bias2=m.actuator_biasprm[:ACT_DIM, 2].copy(), frc=m.actuator_forcerange[:ACT_DIM].copy(),
                       damp=m.dof_damping.copy(), fric=m.dof_frictionloss.copy(), arm=m.dof_armature.copy(),
                       mass=m.body_mass.copy(), inertia=m.body_inertia.copy(), ipos=m.body_ipos.copy(),
                       gfric=m.geom_friction.copy())
        self.base_delay = None
        for i in range(m.nnumeric):
            if m.numeric(i).name == "servo_delay_s":
                self.base_delay = float(m.numeric_data[m.numeric_adr[i]])
        self.cmd = np.zeros(13)
        self.last = np.zeros(ACT_DIM)
        self.reset()

    # ------------------------------------------------------------------ randomization
    def _randomize(self):
        c, m, r, d0 = self.cfg, self.m, self.rng, self.d0
        U = lambda lo_hi, n=None: r.uniform(lo_hi[0], lo_hi[1], n)
        kp = U(c["kp"]) * U((0.95, 1.05), ACT_DIM)
        m.actuator_gainprm[:ACT_DIM, 0] = d0["gain"] * kp
        m.actuator_biasprm[:ACT_DIM, 1] = d0["bias1"] * kp
        m.actuator_biasprm[:ACT_DIM, 2] = d0["bias2"] * U(c["kv"])
        m.actuator_forcerange[:ACT_DIM] = d0["frc"] * U(c["torque"])
        m.dof_damping[:] = d0["damp"] * U(c["joint_damping"])
        m.dof_frictionloss[:] = d0["fric"] * U(c["joint_friction"])
        m.dof_armature[:] = d0["arm"] * U(c["armature"])
        s = U(c["mass"], m.nbody)
        m.body_mass[:] = d0["mass"] * s
        m.body_inertia[:] = d0["inertia"] * s[:, None]
        m.body_ipos[:] = d0["ipos"]
        for b in [self.trunk] + self.head_bodies:
            m.body_ipos[b] += r.uniform(-c["com_m"], c["com_m"], 3)
        m.geom_friction[:] = d0["gfric"]
        for g in self.feet:
            m.geom_friction[g, 0] = U(c["foot_friction"])
        self.delay_steps = int(round(U(c["delay_s"]) / PHYS_DT))
        ang = math.radians(c["imu_tilt_deg"]) * r.uniform()
        ax = r.normal(size=3); ax /= np.linalg.norm(ax)
        self.imu_rot = np.zeros(9); mujoco.mju_quat2Mat(self.imu_rot, np.r_[math.cos(ang / 2), math.sin(ang / 2) * ax])
        self.imu_rot = self.imu_rot.reshape(3, 3)
        self.enc_bias = r.uniform(-c["encoder_bias"], c["encoder_bias"], ACT_DIM)

    def _resample_cmd(self):
        c, r = self.cfg, self.rng
        self.cmd[:] = 0.0
        u = r.uniform()
        if u < c["zero_cmd_prob"]:
            pass
        elif u < c["zero_cmd_prob"] + c["turn_in_place_prob"]:
            self.cmd[2] = r.uniform(*c["wz"])
        else:
            self.cmd[0], self.cmd[1], self.cmd[2] = r.uniform(*c["vx"]), r.uniform(*c["vy"]), r.uniform(*c["wz"])
            for i in range(3):                                    # small commands mean "stand": snap them to zero
                if abs(self.cmd[i]) < (0.05 if i < 2 else 0.1):
                    self.cmd[i] = 0.0
        if r.uniform() < 0.5:
            self.cmd[3:7] = r.uniform(-c["head"], c["head"], 4)
        self.cmd[7:13] = r.uniform(-c["body"], c["body"], 6)      # keeps those input neurons alive; not tracked
        self.t_cmd = r.uniform(*c["resample_s"])

    def reset(self):
        c, m, d, r = self.cfg, self.m, self.d, self.rng
        self._randomize()
        mujoco.mj_resetDataKeyframe(m, d, self.key)
        yaw = r.uniform(-math.pi, math.pi)
        d.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        d.qpos[self.qadr] += r.uniform(-c["init_joint_noise"], c["init_joint_noise"], ACT_DIM)
        d.qpos[self.qadr] = np.clip(d.qpos[self.qadr], self.jrange[:, 0], self.jrange[:, 1])
        d.ctrl[:] = 0.0
        d.ctrl[:ACT_DIM] = self.default
        mujoco.mj_forward(m, d)
        self.delay_buf = [self.default.copy()] * (self.delay_steps + 1)
        self.last[:] = 0.0
        self.t = 0.0
        self.t_push = r.uniform(*c["push_interval_s"])
        self.air = np.zeros(2); self.contact_prev = np.ones(2, bool)
        self.foot_pos = np.array([d.site_xpos[s].copy() for s in self.foot_sites])
        self.head_ema = np.zeros(4)
        self._resample_cmd()
        self.recovering, self.t_upright = False, 0.0
        if r.uniform() < c["recover_prob"]:
            self._drop()
        self.phi = self._potential()
        return self.obs()

    def _drop(self):
        """Lay the duck down: random attitude (front, back, sides or anything between) 0.2 m up, settled 1.5 s with the
        stand targets, so the episode starts from a real resting pose on the floor."""
        m, d, r = self.m, self.d, self.rng
        ax = r.normal(size=3); ax[2] *= 0.3; ax /= np.linalg.norm(ax)
        ang = r.uniform(math.radians(70), math.radians(180))
        q = np.zeros(4); mujoco.mju_axisAngle2Quat(q, ax, ang)
        yaw = r.uniform(-math.pi, math.pi); qy = np.array([math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)])
        out = np.zeros(4); mujoco.mju_mulQuat(out, qy, q)
        d.qpos[2] = 0.2; d.qpos[3:7] = out; d.qvel[:] = 0
        for _ in range(int(1.5 / PHYS_DT)):
            mujoco.mj_step(m, d)
        self.recovering = True
        self.cmd[:3] = 0.0
        self.foot_pos = np.array([d.site_xpos[s_].copy() for s_ in self.foot_sites])

    def _potential(self):
        up_z = -quat_rotate_inv(self.d.qpos[3:7], np.array([0.0, 0.0, -1.0]))[2]
        return 0.5 * up_z + 5.0 * min(float(self.d.qpos[2]), 0.12)

    # ------------------------------------------------------------------ observation
    def _grav(self):
        g = quat_rotate_inv(self.d.qpos[3:7], np.array([0.0, 0.0, -1.0]))
        return self.imu_rot.T @ g

    def obs(self, noise=True):
        c, d, r = self.cfg, self.d, self.rng
        gyro = self.imu_rot.T @ d.sensordata[self.gyro:self.gyro + 3]
        grav = self._grav()
        q = d.qpos[self.qadr] - self.default + self.enc_bias
        v = d.qvel[self.vadr].copy()
        if noise:
            gyro = gyro + r.uniform(-c["n_gyro"], c["n_gyro"], 3)
            grav = grav + r.uniform(-c["n_grav"], c["n_grav"], 3)
            q = q + r.uniform(-c["n_qpos"], c["n_qpos"], ACT_DIM)
            v = v + r.uniform(-c["n_qvel"], c["n_qvel"], ACT_DIM)
        return np.concatenate([gyro, grav, q, v, self.last, self.cmd]).astype(np.float32)

    def critic_extra(self):
        """Privileged state for the critic only: true base velocity in the yaw frame, height, foot contacts, push."""
        lv = self._yaw_frame(self.d.qvel[0:3])
        return np.concatenate([lv, [self.d.qpos[2]], self.contact_prev.astype(float), [self.delay_steps * PHYS_DT],
                               [self.m.actuator_gainprm[0, 0]], [self.t_push]]).astype(np.float32)

    def _yaw_frame(self, v):
        q = self.d.qpos[3:7]
        yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.array([cy * v[0] + sy * v[1], -sy * v[0] + cy * v[1], v[2]])

    # ------------------------------------------------------------------ step
    def step(self, action):
        c, m, d = self.cfg, self.m, self.d
        action = np.asarray(action, dtype=float)
        target = self.default + action
        for _ in range(SUBSTEPS):
            self.delay_buf.append(target)
            d.ctrl[:ACT_DIM] = self.delay_buf.pop(0)
            mujoco.mj_step(m, d)
        self.t += CTRL_DT
        # pushes and command resampling
        self.t_push -= CTRL_DT
        if self.t_push <= 0:
            d.qvel[0:2] += self.rng.uniform(-c["push_dv"], c["push_dv"], 2)
            self.t_push = self.rng.uniform(*c["push_interval_s"])
        self.t_cmd -= CTRL_DT
        if self.t_cmd <= 0:
            self._resample_cmd()
        if self.recovering:
            self.cmd[:3] = 0.0                                    # get up first; walking commands come afterwards
        rew, info, fell = self._reward(action)
        self.last = action.copy()
        done = fell or self.t >= c["episode_s"]
        return self.obs(), rew, done, fell, info

    def _reward(self, action):
        c, m, d = self.cfg, self.m, self.d
        cmd = self.cmd
        moving = np.linalg.norm(cmd[:3]) > 0.01
        lv = self._yaw_frame(d.qvel[0:3])
        av = quat_rotate_inv(d.qpos[3:7], d.qvel[3:6])           # body-frame angular velocity
        grav = quat_rotate_inv(d.qpos[3:7], np.array([0.0, 0.0, -1.0]))
        q = d.qpos[self.qadr]
        # contacts
        contact = np.zeros(2, bool); self_hits = 0; body_floor = False
        for k in range(d.ncon):
            g1, g2 = d.contact[k].geom1, d.contact[k].geom2
            if self.floor in (g1, g2):
                o = g2 if g1 == self.floor else g1
                if o == self.feet[0]: contact[0] = True
                elif o == self.feet[1]: contact[1] = True
                elif o in self.fall_geoms: body_floor = True
            elif (g1 in self.leg_geoms and g2 in self.proxy_geoms) or (g2 in self.leg_geoms and g1 in self.proxy_geoms):
                self_hits += 1
        first = contact & ~self.contact_prev
        air_r = float(np.sum(first * (np.minimum(self.air, c["air_max"]) - c["air_min"])))
        self.air = np.where(contact, 0.0, self.air + CTRL_DT)
        self.contact_prev = contact
        fp = np.array([d.site_xpos[s] for s in self.foot_sites])
        fv = (fp - self.foot_pos) / CTRL_DT; self.foot_pos = fp
        fvxy = np.linalg.norm(fv[:, :2], axis=1)
        clear = float(np.sum((~contact) * np.square(np.minimum(fp[:, 2] - c["clear_h"], 0.0)) * fvxy))
        slip = float(np.sum(contact * np.square(fvxy)))
        std = STD_WALK if moving else STD_STAND
        pose = math.exp(-float(np.mean(np.square((q[LEG] - self.default[LEG]) / std))))
        self.head_ema += 0.02 / 1.0 * ((q[HEAD] - self.default[HEAD]) - self.head_ema)      # 1 s EMA (upstream lesson)
        head = math.exp(-float(np.sum(np.abs(self.head_ema - cmd[3:7]))) / 0.3)
        lo, hi = self.jrange[:, 0], self.jrange[:, 1]; span = hi - lo
        lim = float(np.sum(np.maximum(0.0, lo + 0.05 * span - q) + np.maximum(0.0, q - hi + 0.05 * span)))
        torque = float(np.sum(np.square(d.actuator_force[:ACT_DIM])))
        terms = dict(
            lin=c["w_lin"] * math.exp(-float(np.sum(np.square(cmd[:2] - lv[:2]))) / c["s_lin"]),
            ang=c["w_ang"] * math.exp(-(cmd[2] - av[2]) ** 2 / c["s_ang"]),
            up=c["w_up"] * math.exp(-float(np.sum(np.square(grav[:2]))) / c["s_up"]),
            pose=c["w_pose"] * pose,
            head=c["w_head"] * head,
            air=(c["w_air"] * air_r / CTRL_DT) if moving else 0.0,
            clear=(c["w_clear"] * clear) if moving else 0.0,
            slip=c["w_slip"] * slip,
            rate=c["w_rate"] * float(np.sum(np.square(action - self.last))),
            angvel=c["w_angvel"] * float(np.sum(np.square(av[:2]))),
            limits=c["w_limits"] * lim,
            self=c["w_self"] * self_hits,
            torque=c["w_torque"] * torque,
        )
        up_z = -grav[2]
        # task terms only pay when upright (1 above 0.9, 0 below 0.5 up-axis): lying still with a zero command would
        # otherwise collect the velocity/pose rewards (the "positive reward in a bad state" trap)
        gate = min(1.0, max(0.0, (up_z - 0.5) / 0.4)) * min(1.0, max(0.0, (float(d.qpos[2]) - 0.06) / 0.04))
        for k in ("lin", "ang", "up", "pose", "head", "air"):
            if terms[k] > 0:
                terms[k] *= gate
        down = bool(up_z < 0.5 or d.qpos[2] < 0.07 or body_floor)
        fell = down or not np.all(np.isfinite(d.qpos))
        if self.recovering:
            phi = self._potential()
            terms["rise"] = c["w_rise"] * (phi - self.phi) / CTRL_DT
            self.phi = phi
            self.t_upright = self.t_upright + CTRL_DT if (up_z > 0.9 and d.qpos[2] > 0.1) else 0.0
            if self.t_upright >= c["recovered_s"]:
                self.recovering = False                           # from now on a fall ends the episode as usual
            fell = (not np.all(np.isfinite(d.qpos))) or self.t > c["max_recover_s"]
            terms["self"] = 0.0                                   # legs may touch the body while scrambling up
        if fell:
            terms["term"] = c["w_term"] / CTRL_DT
        rew = sum(terms.values()) * CTRL_DT
        return rew, terms, fell
