"""Servo command delay (duck_nav/actuation.py), used by world_sim and scripts/motions.py."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from duck_nav.actuation import ServoDelay, model_servo_delay

mujoco = pytest.importorskip("mujoco")

XML = """<mujoco><option timestep="0.005"/>{custom}
<worldbody><body><joint name="j" type="hinge" axis="0 0 1" damping="0.05"/>
<geom type="box" size=".1 .02 .02" pos=".1 0 0" mass="0.5"/></body></worldbody>
<actuator><position joint="j" kp="0.5" kv="0.02"/></actuator></mujoco>"""


def test_delay_line_returns_the_target_issued_n_steps_ago():
    s = ServoDelay(0.02, 0.005, [0.0])
    assert s.steps == 4 and s.delay_s == pytest.approx(0.02)
    out = [float(s.step([k])[0]) for k in range(1, 9)]
    assert out == [0, 0, 0, 0, 1, 2, 3, 4]


def test_zero_delay_passes_through_and_delay_rounds_to_physics_steps():
    s = ServoDelay(0.0, 0.005, [0.0, 0.0])
    np.testing.assert_array_equal(s.step([1.0, 2.0]), [1.0, 2.0])
    assert ServoDelay(0.012, 0.005, [0.0]).steps == 2
    with pytest.raises(ValueError):
        ServoDelay(-0.01, 0.005, [0.0])


def test_model_delay_comes_from_the_custom_numeric():
    m = mujoco.MjModel.from_xml_string(XML.format(custom='<custom><numeric name="servo_delay_s" data="0.03"/></custom>'))
    assert model_servo_delay(m) == pytest.approx(0.03)
    assert model_servo_delay(mujoco.MjModel.from_xml_string(XML.format(custom="")), default=0.007) == 0.007


def test_delayed_servo_starts_moving_only_after_the_delay():
    m = mujoco.MjModel.from_xml_string(XML.format(custom='<custom><numeric name="servo_delay_s" data="0.02"/></custom>'))
    first_motion = {}
    for delay in (0.0, model_servo_delay(m)):
        d = mujoco.MjData(m)
        s = ServoDelay(delay, m.opt.timestep, [0.0])
        for k in range(20):
            d.ctrl[:] = s.step([0.5])
            mujoco.mj_step(m, d)
            if abs(d.qpos[0]) > 1e-9 and delay not in first_motion:
                first_motion[delay] = k
    assert first_motion[0.0] == 0
    assert first_motion[0.02] == 4


def test_same_response_as_mujoco_native_actuator_delay_where_available():
    try:
        native = mujoco.MjModel.from_xml_string(XML.replace('kv="0.02"/>', 'kv="0.02" delay="0.02" nsample="6"/>')
                                                .format(custom=""))
    except ValueError:
        pytest.skip("this MuJoCo has no actuator delay attribute (needs >= 3.4)")
    plain = mujoco.MjModel.from_xml_string(XML.format(custom=""))
    dn, dp = mujoco.MjData(native), mujoco.MjData(plain)
    s = ServoDelay(0.02, plain.opt.timestep, [0.0])
    for k in range(60):
        target = 0.5 if k >= 3 else 0.0
        dn.ctrl[:] = target
        dp.ctrl[:] = s.step([target])
        mujoco.mj_step(native, dn); mujoco.mj_step(plain, dp)
    assert dp.qpos[0] == pytest.approx(dn.qpos[0], abs=1e-6)
