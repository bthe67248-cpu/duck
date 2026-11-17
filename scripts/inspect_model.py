import sys, mujoco
m = mujoco.MjModel.from_xml_path(sys.argv[1])
d = mujoco.MjData(m)
if m.nkey: mujoco.mj_resetDataKeyframe(m, d, 0)
mujoco.mj_forward(m, d)
print("timestep s      :", m.opt.timestep)
print("total mass kg   :", m.body_mass.sum())
print("centre of mass m:", d.subtree_com[0] if m.nbody == 1 else d.subtree_com[1])
print("nq", m.nq, "nv", m.nv, "nu", m.nu)
print("\nBODIES (id, name, mass kg)")
for i in range(m.nbody):
    print(f"  {i:2d} {m.body(i).name:28s} {m.body_mass[i]:.4f}")
print("\nJOINTS (name, type, range rad)")
for i in range(m.njnt):
    print(f"  {m.joint(i).name:28s} {m.jnt_type[i]} {m.jnt_range[i]}")
print("\nACTUATORS in input order (name, gain, bias, force limit N.m)")
for i in range(m.nu):
    print(f"  {i:2d} {m.actuator(i).name:24s} kp={m.actuator_gainprm[i,0]:.3f} bias={m.actuator_biasprm[i,:3]} {m.actuator_forcerange[i]}")
print("\nSENSORS in output order")
for i in range(m.nsensor):
    print(f"  {i:2d} {m.sensor(i).name:24s} dim {m.sensor_dim[i]}")
print("\nSITES:", [m.site(i).name for i in range(m.nsite)])
print("\nKEYFRAMES (stored poses)")
for i in range(m.nkey):
    print(f"  {m.key(i).name}: qpos={m.key_qpos[i]} ctrl={m.key_ctrl[i]}")
