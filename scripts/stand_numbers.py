"""stand_numbers.py -- settle the duck in its STAND pose and print the numbers MATLAB needs.

Usage:  .venv\\Scripts\\python.exe scripts\\stand_numbers.py model\\duck_v3_scene.xml
Writes model/stand_numbers.json (read by matlab/duck_numbers.m).
"""
import json, sys
from pathlib import Path
import mujoco
import numpy as np

path = Path(sys.argv[1])
m = mujoco.MjModel.from_xml_path(str(path))
d = mujoco.MjData(m)
kid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "STAND")
mujoco.mj_resetDataKeyframe(m, d, kid)
home = m.key_ctrl[kid].copy()
mujoco.mj_forward(m, d)                   # nominal STAND pose, before anything sags

torso = m.body("trunk_base").id
com = d.subtree_com[torso].copy()
mass = float(m.body_subtreemass[torso])
pitch = 0.0

# ankle pivots (world) and the sole outline: lowest vertices of the two foot collision meshes
ank = {s: d.xanchor[m.joint(f"{s}_ankle").id].copy() for s in ("left", "right")}
pts = []
for s in ("left", "right"):
    g = m.geom(f"{s}_foot_collision").id
    mid = m.geom_dataid[g]
    v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
    w = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
    pts.append(w[w[:, 2] < w[:, 2].min() + 0.001])
pts = np.vstack(pts)
floor_z = float(pts[:, 2].min())
ankle_x = float(np.mean([ank["left"][0], ank["right"][0]]))
ankle_z = float(np.mean([ank["left"][2], ank["right"][2]])) - floor_z
com[2] -= floor_z
out = dict(
    timestep=m.opt.timestep, mass=mass, com=com.tolist(), settled_pitch_rad=pitch,
    torso_z=float(d.xpos[torso][2]),
    ankle_x=ankle_x, ankle_z=ankle_z, ankle_y=[float(ank["left"][1]), float(ank["right"][1])],
    h=float(com[2] - ankle_z),                       # CoM height above the ankle pivot
    com_height_floor=float(com[2]),
    toe_x=float(pts[:, 0].max()), heel_x=float(pts[:, 0].min()),
    a_toe=float(pts[:, 0].max() - com[0]),           # CoM -> front edge of support
    a_heel=float(com[0] - pts[:, 0].min()),          # CoM -> rear edge of support
    half_width=float((pts[:, 1].max() - pts[:, 1].min()) / 2),
    knee_x=float(d.xanchor[m.joint("left_knee").id][0]),
    kp=float(m.actuator_gainprm[0, 0]), stall=float(m.actuator_forcerange[0, 1]),
    home=home[:14].tolist(),
    actuators=[m.actuator(i).name for i in range(m.nu)],
)
out["d_knee"] = abs(out["knee_x"] - out["com"][0])
for k, v in out.items():
    print(f"{k:18s} {v}")
(path.parent / "stand_numbers.json").write_text(json.dumps(out, indent=1))
