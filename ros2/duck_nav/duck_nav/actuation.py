"""actuation -- servo command delay for the walking simulation (numpy only; used by world_sim and scripts/motions.py).

The real servos act on a target some time after the policy computed it (bus transfer, servo firmware cycle). The model
carries that delay as <custom><numeric name="servo_delay_s"> (scripts/make_duck.py: an ESTIMATE of 20 ms, the middle
of the upstream training lag of 15-30 ms, or the calibration's measured servo.delay_s). `ServoDelay` is a pure
transport delay at physics-step resolution: the target applied at physics step k is the one issued at step k - n,
n = round(delay / timestep). It is implemented here rather than with MuJoCo's actuator `delay` attribute so the model
loads in every MuJoCo version the project uses (the attribute needs MuJoCo >= 3.4 and the CI pins 3.3.6).
"""
from collections import deque

import numpy as np


def model_servo_delay(m, default=0.0):
    """The model's servo command delay in seconds (custom numeric `servo_delay_s`), `default` if it has none."""
    for i in range(m.nnumeric):
        if m.numeric(i).name == "servo_delay_s":
            return float(m.numeric_data[m.numeric_adr[i]])
    return float(default)


class ServoDelay:
    """Delay line for actuator targets, advanced once per physics step."""

    def __init__(self, delay_s, timestep, initial):
        if delay_s < 0:
            raise ValueError("servo delay must be >= 0")
        self.steps = int(round(delay_s / timestep))
        self.delay_s = self.steps * timestep          # what is actually simulated (physics-step resolution)
        self.reset(initial)

    def reset(self, initial):
        self.buf = deque([np.array(initial, dtype=float)] * (self.steps + 1), maxlen=self.steps + 1)

    def step(self, target):
        """Push this physics step's commanded target; return the target the servos act on now."""
        self.buf.append(np.array(target, dtype=float))
        return self.buf[0]


def find_mouth(m):
    """(actuator id, qpos address, dof address) of the V4 jaw servo `mouth`, or None for a model without it."""
    import mujoco
    a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "mouth")
    if a < 0:
        return None
    j = m.actuator_trnid[a, 0]
    return a, int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j])


class MouthCommand:
    """Jaw opening target, kept apart from the walking policy's 14-joint observation/action contract.

    `set` clips to the actuator's range and ignores non-finite values; `target` falls back to `closed` once the last
    command is older than `timeout` seconds, the way the head and posture commands expire (nothing stays latched).
    """

    def __init__(self, lo, hi, timeout, closed=None):
        self.lo, self.hi, self.timeout = float(lo), float(hi), float(timeout)
        self.closed = self.lo if closed is None else float(np.clip(closed, self.lo, self.hi))
        self.value, self.t = self.closed, -np.inf

    def set(self, angle, t):
        if not np.isfinite(angle):
            return False
        self.value, self.t = float(np.clip(angle, self.lo, self.hi)), float(t)
        return True

    def target(self, t):
        return self.value if t - self.t <= self.timeout else self.closed
