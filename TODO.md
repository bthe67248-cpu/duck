# Duckling V3: what is left

Everything in this repo runs in simulation on estimated numbers. These steps make it match the real robot. Do them in order. The carrier board is at [bthe67248-cpu/UNO_Q_HAT_IMU_SERVOS_XT60](https://github.com/bthe67248-cpu/UNO_Q_HAT_IMU_SERVOS_XT60). The step-by-step board plan is [UNO_Q_GUIDE.md](UNO_Q_GUIDE.md).

**Software progress:** [hardware-readiness handoff](results/hardware_readiness/HANDOFF.md). Measurement JSON input and strict contact checking are implemented. Sensor acquisition interfaces and TF pose lookup are implemented, with offline tests; the actual VL53L5CX bus adapter and hardware encoder/IMU TF remain missing. Weak/stale stopping is implemented. The patch's strict scan-match rules were replaced by a correlative matcher with slip estimation and local relocalisation ([SLAM tuning](results/slam_tuning/README.md)); loop closure and hardware localisation remain open.

## 1. Board bring-up (UNO_Q_HAT_IMU_SERVOS_XT60)

- [ ] Order the board with 2 oz outer copper (gerbers: `fabrication/UNOQ_HAT_gerbers_2026-11-13.zip` in the HAT repo)
- [ ] Assemble, then power from a current-limited bench supply before any battery
- [ ] Check the 5 V and 3.3 V rails, and that a reversed input is blocked (LM74502)
- [ ] Talk to one Feetech HD-1910 servo over the half-duplex bus
- [ ] Read the QMI8658A IMU over I2C

## 2. Print and measure the robot (goes into `scripts/make_duck_v3.py`, top block)

- [ ] Print the visible rigid parts in black PLA at 30% infill. Save the slicer project and record the slicer version, filament density, orientation, walls, top/bottom layers, infill and support mass
- [ ] Weigh each V3 print after removing supports → `V3_PART_MASS_G` (provisional now: upper head 51.5 g, lower head 43.1 g, rear shield 32.3 g)
- [ ] Weigh each link with servos and screws: `trunk_base`, `jaw_soft`, `upper_leg_left/right`, `leg`, `leg_2`, `ankle_left/right` → `MEASURED_MASS`
- [ ] Build the installed mass ledger (prints, servos, UNO Q, HAT, battery, BMS, camera, ToF, wiring, fasteners). Do not count both the old Raspberry Pi electronics and their replacements
- [ ] Measure the shield centre in the torso frame → `SHIELD_POS`. **This matters most:** at the guessed position the shield hits the legs and blocks get-up in simulation. Check leg and head clearance through their full range
- [ ] Check the V3 head position against the real robot → `HEAD_SHIFT`
- [ ] Check the new bill's wall thickness, seam fit and internal clearance
- [ ] Count the servos (model has 14, the XGO-Duck parts list names 15)

## 3. Servo bench test (goes into `SERVO` in the same block)

- [ ] Log 10° steps of one servo from the bus
- [ ] Fit stiffness `kp`, damping `kv`, delay `td`, stall torque (the model still uses Dynamixel XL330 numbers: kp 0.55 N·m/rad, 0.96 N·m)
- [ ] Put `td` into `matlab/duck_numbers.m` (now an example 30 ms)

## 4. Re-run the simulation with real numbers

- [ ] `make_duck_v3.py`, `make_duck_v3.py --base robot_groundcontact.xml --tag gc`, `stand_numbers.py`, then in MATLAB `startup_duck`, `duck_numbers`, `build_duck_sim`, `run_tests`
- [ ] `scripts/motions.py all`: no leg-on-shield contacts; walk, obstacles and get-up pass
- [ ] `scripts/tune_gains.py` again: the safe gain band depends on servo stiffness and delay
- [ ] Gate: model mass within 5% of the scale, all Simulink assertions pass, p·td below 0.3
- [ ] Re-export the URDF and rebuild the ROS 2 workspace (`mjcf_to_urdf.py`, `check_urdf.py`, `sync_ros2_assets.py`, `setup_ws.sh`, `setup_nav.sh`)

## 5. Camera and ToF (UNO_Q_GUIDE.md, Step 6)

- [ ] Get the Arducam B0385 (OV9782 USB UVC) and a Pololu #3417 VL53L5CX carrier
- [ ] Check the drawings, mass, current and V4L2 modes; check USB streaming on the UNO Q (hub or not)
- [ ] ToF at 3.3 V logic; zone order and range convention; a rigid shared bracket; a clear optical opening
- [ ] Calibrate camera intrinsics, then camera and ToF extrinsics
- [x] Write `duck_tof` acquisition wrapper and exact-stamp TF pose publisher, with offline contract tests
- [ ] Supply the actual VL53L5CX backend, synchronized hardware acquisition stamps, measured extrinsics and encoder/IMU TF; validate nodes on ROS and hardware

## 6. On the UNO Q (see UNO_Q_GUIDE.md)

- [ ] STM32 sketch: servo bus, IMU, 100 Hz loop, watchdog
- [ ] Port `tiltFilter` and the two gains (about ten lines of C); stand with the loop on, a hand ready
- [ ] Compare the real knee sag with the simulated 4.6°
- [ ] Walking: use the XGO-Duck project's own runtime and policy, not the Microduck one
- [ ] Replay the same `/cmd_vel` in `world_sim` and on the robot; fix the model first, gains second

## 7. Open software work

Post-merge implementation and validation status: [review handoff](results/post_merge/HANDOFF.md).
Experimental pose-graph closure is opt-in; offline tests do not close the realistic-trial accuracy gap.
The new trial runner and replay-set tools are in the repository, but the historical bags and the original
Windows-only `job2.sh`/`patch_ort.py` were not supplied. Ten-seed/layout and paired 1x/2x trials remain unrun.

- [x] Scan matching that keeps the duck exploring with the 8×8 ToF: correlative matcher over the head sweep, slip estimation, stop when the match is weak ([SLAM tuning](results/slam_tuning/README.md))
- [ ] Validate experimental loop closure / pose graph on recorded and closed-loop trials: realistic baseline runs still drift late (up to 2.4 m at 600 s); implementation alone is not evidence of improved accuracy
- [ ] Re-localise after a fall: local relocalisation (±0.8 m / ±25° after 3 s unhealthy) exists; a global one does not
- [ ] Stop the escape / plan loop near walls (pure pursuit cuts corners into the clearance margin)
- [ ] Roll (sideways) balance: only pitch is controlled now
- [ ] YOLO: domain randomization, then fine-tune on real labelled camera photos
- [ ] Retrain walking policies on measured masses and a Feetech servo model (needs a rented NVIDIA GPU); now required, since the pretrained walk fails on the corrected model
- [ ] Finish `ros2/parked_commands` (steps in its README.txt), after command ownership is settled
- [ ] Update `scripts/slam_nav.py` to use the fixed planner, or retire it
- [ ] Fresh-workspace build check (dependency versions are pinned in `requirements-test.txt`; CI runs the tests)
- [ ] Optional: Simscape Multibody route and linear margins (needs toolboxes not installed)
