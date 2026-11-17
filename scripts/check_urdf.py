"""check_urdf.py -- forward-kinematics check: URDF link positions vs MuJoCo body positions."""
import math, sys
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco, numpy as np
ROOT = Path(__file__).resolve().parents[1]
m = mujoco.MjModel.from_xml_path(str(ROOT / "model" / "duck_v3_scene.xml")); d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "STAND")); mujoco.mj_forward(m, d)
q = {m.joint(i).name: d.qpos[m.jnt_qposadr[i]] for i in range(1, m.njnt)}
def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
def rot_axis(a, t):
    a = np.array(a) / np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K
urdf = ET.parse(ROOT / "ros2" / "duck_description" / "urdf" / "duck_v3.urdf").getroot()
T = {"trunk_base": (d.xmat[1].reshape(3, 3).copy(), d.xpos[1].copy())}
worst = 0.0
for j in urdf.findall("joint"):                      # joints are written parent-before-child
    R, p = T[j.find("parent").get("link")]
    o = j.find("origin"); xyz = np.array([float(v) for v in o.get("xyz").split()]); rpy = [float(v) for v in o.get("rpy").split()]
    Rc = R @ rot_rpy(*rpy)
    if j.get("type") == "revolute":
        Rc = Rc @ rot_axis([float(v) for v in j.find("axis").get("xyz").split()], q[j.get("name")])
    child = j.find("child").get("link"); T[child] = (Rc, p + R @ xyz)
    err = np.linalg.norm(T[child][1] - d.xpos[m.body(child).id]); rerr = np.abs(Rc - d.xmat[m.body(child).id].reshape(3, 3)).max()
    worst = max(worst, err, rerr)
print(f"URDF vs MuJoCo at STAND pose: worst position/rotation error {worst:.2e} over {len(T)} links ->", "OK" if worst < 1e-4 else "MISMATCH")
