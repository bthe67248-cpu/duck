# Duckling V3: from the simulation to the Arduino UNO Q

This is the plan for moving what now runs in simulation (`C:\dev\duckling_sim`) onto the duck's own board. **None of it has been run on hardware.** Updated for the V3 handoff revision (17 Nov 2026): chosen camera and ToF, black PLA at 30% infill, and the corrected-model test results. Board facts come from the UNO Q datasheet and Arduino pages; anything I could not confirm is marked *check*.

## The board, in the numbers that matter here

| Part | What it is | What it means for the duck |
|-|-|-|
| Qualcomm QRB2210 | 4 × Cortex-A53 at 2.0 GHz, Adreno 702 GPU, Debian Linux | Runs the policy, camera, YOLO, SLAM, navigation, ROS 2. Roughly 5 to 10 times slower than your PC per core |
| RAM / storage | Yours is the 4 GB model (ABX00173): 4 GB RAM, 32 GB eMMC | Enough for ROS 2 in Docker plus YOLO, SLAM and the policy together. Others have run a TurtleBot3 ROS 2 container on this model |
| STM32U585 | Cortex-M33, 160 MHz, 2 MB flash, 786 KB RAM, Arduino sketches on Zephyr | Servo bus, IMU, 100 Hz loop, safety watchdog |
| Bridge | MessagePack-RPC over an internal UART, `arduino-router` service on Linux, `Arduino_RouterBridge` library on the MCU | The only link between the two halves. One forum report puts it near 112 kbps, so messages must be small and binary (*check on your board*) |

## What moves over as-is, and what does not

| Piece | Status for the real duck |
|-|-|
| `slam_node`, `nav_node`, `duckslam.py` | Move over. The ToF path (`sensor_mode:=tof`) is written and runs in simulation with the realistic profile. It needs a real ToF driver node and a hardware `/tof/pose_level` + `/camera/pose_level` publisher (Step 6), and slower rates (Step 8) |
| `yolo_node` | Code moves over. **The trained model does not**: it has only ever seen MuJoCo renders. Retrain on photos (Step 7) |
| Walking / get-up policies | **Do not transfer.** They were trained for the Microduck with Dynamixel XL330 servos; your duck has Feetech bus servos. Even in simulation, with the corrected V3 masses and the rear shield's collision shapes, walk, obstacles and get-up now fail (README Part D). Start from the XGO-Duck project's own runtime and policy (Step 4) |
| Balance loop from Simulink (2.34·pitch + 0.186·rate) | The structure moves over. The gains are for the simulated servo stiffness and provisional masses; recompute after the bench test and weighing (Step 3). The backward push margin is small (about 1 N for 0.05 s in simulation) |
| `world_sim` | Stays on the PC. On the robot it is replaced by two small driver nodes (Step 5) |
| Model masses, servo model | Still estimates. Print masses are provisional 30% infill numbers (upper head 51.5 g, lower head 43.1 g, rear shield 32.3 g); the whole chain is only as good as these (Step 3) |

## Step 1. Get onto the board and take stock

1. Power the UNO Q, connect it to Wi-Fi with Arduino App Lab, and open a shell on it (App Lab terminal, `adb shell`, or SSH; *check the Arduino docs for the default user*).
2. Record four numbers:

```bash
free -h
```

```bash
df -h /
```

```bash
cat /etc/os-release
```

```bash
nproc
```

**Gate:** `free -h` shows about 4 GB, you know the Debian version, and there is at least 6 GB of free disk for Docker + ROS (the 32 GB eMMC should leave plenty).

## Step 2. Bring up the microcontroller side first

The STM32 owns everything time-critical. Write (or take from the XGO-Duck runtime) one sketch that:

1. Runs a fixed 100 Hz loop.
2. Reads the IMU and every servo position each cycle (the simulated model has 14 servos; the XGO-Duck parts list names 15, so count yours).
3. Writes servo targets each cycle.
4. Exposes two Bridge calls to Linux: `set_targets` (14 joint targets) and `get_state` (joint positions, gyro, accelerometer).
5. **Watchdog:** if no `set_targets` arrives for 100 ms, hold the last pose with reduced torque; after 500 ms, go limp. This is what saves the robot when Linux stalls.

Keep the Bridge traffic small. 14 targets down and 34 values up at 50 Hz, packed as 16-bit integers, is about 27 kbps. Sending the same as text or 64-bit floats will not fit if the link really is near 112 kbps.

**Gate:** with the duck on its stand, a Linux-side script moves each joint alone by ±10° and reads it back within 2°. Each joint moves the right part in the right direction (compare with `scripts\inspect_model.py` joint order: 0–4 left leg, 5–8 neck and head, 9–13 right leg).

## Step 3. Close the gaps in the model (needs the physical robot)

Nothing below is new; it is the part of the two PDFs that is still open.

1. Print the visible rigid parts in black PLA, 30% infill (start: 0.4 mm nozzle, 0.2 mm layers, 3 walls, 5 top and 5 bottom layers, gyroid). Save the slicer project, and record the slicer version, filament density, orientation and support mass. Weigh every part after removing supports, and every link. Type the numbers into `V3_PART_MASS_G` and `MEASURED_MASS` at the top of `scripts\make_duck_v3.py`. Do not scale masses by the infill percentage, and do not reduce servo, electronics or battery masses.
2. Bench-test one servo (10° steps, logged from the bus). Get stiffness `kp`, damping `kv`, delay `td`, stall torque. Put them in the `SERVO` line of the same file.
3. Measure the real V3 head and shield positions with a ruler; set `HEAD_SHIFT` and `SHIELD_POS`. **The shield matters most:** at its guessed position it hits the legs while walking and stops get-up in simulation. Move each leg and the head through their full range by hand against the mounted shield before you finalise it.
4. Regenerate and rerun everything:

```powershell
cd C:\dev\duckling_sim; .\.venv\Scripts\python.exe scripts\make_duck_v3.py; .\.venv\Scripts\python.exe scripts\stand_numbers.py model\duck_v3_scene.xml
```

Then in MATLAB: `startup_duck`, `build_duck_sim`, `run_tests`. Then `scripts\tune_gains.py` again, because the safe gain band depends on servo stiffness and delay.

**Gate:** model mass within 5% of the scale. All six Simulink tests pass with the measured servo numbers (`run_tests` stops with an error if any assertion fails). The measured delay check p·td is below 0.3. `scripts\motions.py all` (ground-contact model with the head and shield collision shapes) shows no leg-on-shield contacts.

## Step 4. Standing and walking on the real robot

1. **Standing first, with the balance loop on the STM32.** Port `tiltFilter` and the two gains into the 100 Hz loop (it is about ten lines of C). Robot on the floor, a hand ready. Compare the knee sag with the simulated 4.6°: that single number checks mass and servo stiffness together.
2. **Walking: use the XGO-Duck policy, not the Microduck one.** Your printed parts and servos are the XGO-Duck's, and that project ships a runtime and policy for this hardware. Get that walking before anything else.
3. If you want the Microduck behaviours (get-up, roll, sit): retrain in `microduck_rl` with your masses and a Feetech servo model. That needs an NVIDIA GPU (rented; your RX 9070 will not run it). Check each retrained policy in `scripts\motions.py` before it goes on the robot.
4. On Linux the policy is small: one ONNX Runtime call at 50 Hz. `world_sim.py` shows the exact observation layout (gyro, gravity direction, joint positions relative to STAND, joint velocities, last action, 13 command values).

**Gate:** stands 60 s. Recovers from a light push. Walks 2 m in a straight line on the surface you will use, 5 times out of 5.

## Step 5. ROS 2 on the UNO Q

1. ROS 2 Jazzy has apt packages for Ubuntu, not for Debian, so run it in Docker on the board (others have run TurtleBot3 this way on the 4 GB UNO Q):

```bash
docker pull ros:jazzy-ros-base
```

2. Start the container with host networking and the package folder mounted:

```bash
docker run -it --net=host --privileged -v ~/duck_ws:/duck_ws ros:jazzy-ros-base
```

3. Copy `ros2\duck_nav` and `ros2\duck_description` into `~/duck_ws/src`, then inside the container install `python3-scipy`, `python3-numpy`, `python3-yaml`, `python3-opencv`, `ros-jazzy-vision-msgs`, `ros-jazzy-sensor-msgs-py`, `onnxruntime` (pip), and run `colcon build`.
4. Write two new nodes to replace `world_sim`:
   - `duck_hw`: calls the Bridge. Publishes `/joint_states` and `/imu`; subscribes `/cmd_vel` and `/head_yaw_cmd`; runs the walking policy and sends `set_targets`.
   - Camera: `usb_camera_node` is already written (V4L2 through OpenCV, newest frame only, stale-frame watchdog, calibrated `CameraInfo`). It has not been run on hardware. Use the stable `/dev/v4l/by-id/...` path, start near 640 × 480 at 15 to 30 fps, and lock the exposure. YOLO letterboxes any size.
   - `duck_tof` (not written yet): reads the VL53L5CX and publishes `/tof/zones` exactly as `world_sim` does (PointCloud2 with x y z range status zone, stamped at acquisition), plus `/tof/pose_level` at the same stamp.
   - Only one node may own `/cmd_vel` at a time. `nav_node` sends a zero command when it stops or hands over, but the MCU watchdog (Step 2) is the real safety net.
5. Launch every node with `use_sim_time:=false`. In the launch file, drop `world_sim` and `score_node`.
6. Reaching the container from the Bridge: the `arduino-router` socket lives on the host, so mount it into the container (*check its path on your board*).

**Gate:** `ros2 topic hz /joint_states` shows 50 Hz and `/imu` shows 50 Hz on the board, with CPU use under 50% of one core for `duck_hw`.

## Step 6. Range sensing: chosen hardware, and what is left

**Decision (handoff, 17 Nov 2026): option B.** Main camera: **Arducam B0385**, OV9782 colour global shutter, USB 2.0 UVC board with a low-distortion M12 lens (alternative: Arducam B0196, IMX219 UVC; do not use a Pi CSI ribbon camera). Range: **ST VL53L5CX on a Pololu #3417 carrier**, 8×8 zones over I²C, start at 10 Hz (15 Hz is the 8×8 ceiling). Neither is bought or tested yet.

In simulation this is `sensors:=realistic` (README Part F). It works, but it is clearly weaker than the ideal depth camera: the map drifts and doubles walls, and runs vary a lot. For reference, the options that were considered:

| Option | What changes | Trade-off |
|-|-|-|
| **A. Single ToF on the sweeping head** | Replace `scan_from_depth` with "one range per head angle": the head already sweeps ±0.9 rad every 4 s | No new hardware. About 20 to 40 range readings per sweep, so the map fills slowly and scan matching has little to work with |
| **B. Multi-zone ToF (8 × 8, for example VL53L5CX)** | Feed its 64 ranges through the same code as a tiny depth image | Cheap, light, a few metres of range. Coarse, but the head sweep widens it |
| **C. Small depth camera** | Use the code as it is | Best map. Weight on the head (the head is already 23 g heavier), USB bandwidth, and power |

Before freezing the head CAD, check on the real parts:

1. **Camera:** board revision, lens envelope, connector direction, mounting-hole drawing, mass, current, V4L2 modes. Check that it enumerates and streams on the UNO Q, and whether it needs the USB-C hub. Count the hub's mass and power.
2. **ToF power:** on a 3.3 V I²C bus, feed the carrier 3.3 V. Its SDA/SCL pull-ups follow VIN, so 5 V on VIN gives 5 V bus highs. Bare carrier: about 13 × 18 × 3 mm, 0.5 g without headers; wires and bracket come on top.
3. **ToF bus:** Linux side if the UNO Q exposes the bus there, otherwise an MCU task with a timestamped bridge. Watch driver memory, firmware loading and contention with the IMU. Keep it out of the 100 Hz loop's blocking path.
4. **Optics:** a clear opening for the ToF. Do not assume black PLA, or any dark window, passes its infrared light. The camera and ToF must clear the bill through the whole head sweep (simulation says they do: `results\post_fix\fov_check.txt`).
5. **Bracket:** one rigid bracket for both, so the camera-to-ToF transform stays fixed; strain relief for the cables through head motion.
6. **Zone order and range convention:** point at targets left, right, up and down to confirm zone order (`tof_zone_rays` assumes row-major, top-left first). Check whether the driver reports range along the ray or along the boresight, and set `tof_range_convention`.
7. **Calibrate** camera intrinsics and distortion (after the focus is final), then camera-to-head and ToF-to-head.

`slam_node` needs each sensor's pose relative to the level body frame at its own acquisition stamp. In simulation that is the exact `/camera/pose_level` and `/tof/pose_level`; on the robot compute them from the four neck and head encoder angles plus IMU roll and pitch (MuJoCo camera convention: looks along −z, y up).

**Gate:** with the duck standing still in a known spot, the map of a simple box-walled pen matches a tape measure within 5 cm.

## Step 7. YOLO on real images

1. Collect 300 to 500 photos from the duck's own camera (the B0385 at its final focus) at the height and tilt it walks at: your floor, your obstacles, your beacon (or whatever you want it to find), different lighting, head yaw and pitch, motion blur, empty scenes, and magenta things that are not the beacon. Split validation by recording session or location, not by random frame.
2. Label them (any YOLO-format labelling tool). Keep 15% aside for validation.
3. Fine-tune from the sim-trained weights rather than from scratch: point `scripts\yolo_train.py` at the new `data.yaml` and at `yolo\runs\duck\weights\best.pt`.
4. Export to ONNX (320 × 256 as now, or larger; `yolo_node` reads the input size from the model and letterboxes any camera image). Compare detections before and after export, then copy to the board.
5. Time it on the board. On four A53 cores expect a few frames per second, not 30. Two per second is enough here: detections only place the goal.

**Gate:** on the held-out real photos, recall above 0.9 for the class you navigate to, and no false "beacon" in 100 frames of an empty room.

## Step 8. Fit it in the CPU budget

In WSL everything ran in real time on a desktop CPU. On the QRB2210, measure and then cut:

| Node | PC rate | Start with on the UNO Q | If still too slow |
|-|-|-|-|
| Policy (`duck_hw`) | 50 Hz | 50 Hz, never lower | Give it its own core (`taskset`) |
| `yolo_node` | 5 Hz | 2 Hz | Smaller input, or an int8-quantised model |
| `slam_node` scan matching | 5 Hz | 2 Hz, search window ±5 cm | Port the triple loop in `match` to NumPy broadcasting or C++ |
| `nav_node` planner | 1 Hz | 0.5 Hz | The wavefront is a Python loop over about 30,000 cells: port to C++ or use a coarser grid (5 cm cells) |
| Map publishing | 1 Hz | 0.5 Hz | Publish only to the PC when RViz is connected |

Measure with `top` and `ros2 topic hz`. The policy loop must never miss its 20 ms; everything else can be late.

**Gate:** all nodes running, policy loop jitter under 5 ms, total CPU below 85%, RAM not swapping.

On the 4 GB board memory is not the limit: expect roughly 1 to 1.5 GB in use with everything running (my estimate, not measured). The four A53 cores are the limit, so the rate cuts above still apply.

## Step 9. Watch it from the PC

Run RViz on the PC and the nodes on the duck, same `ROS_DOMAIN_ID`, same Wi-Fi. WSL2's default network hides it from ROS discovery; switch WSL to mirrored networking (`networkingMode=mirrored` in `.wslconfig`) or use a native Linux machine. Use the same `duck_nav.rviz`.

## Step 10. Bring-up order, each with a stop condition

1. On the stand, feet off the ground, torque limits low: replay joint sweeps.
2. Floor, standing, balance loop only. Hand ready.
3. Walking by `/cmd_vel` from the keyboard, on a tether.
4. Camera + YOLO while you carry the duck around. No motion commands from YOLO yet.
5. SLAM while you drive it by hand: does the map look like the room?
6. `nav_node` in a small pen (1.5 × 1.5 m, soft walls), one box, beacon visible from the start.
7. Bigger room, beacon hidden.

Add a stop topic before step 3: any node or a keyboard key publishes `/estop`, and `duck_hw` stops sending targets, which trips the MCU watchdog from Step 2.

## Step 11. Close the loop with the simulation

Record every run on the robot:

```bash
ros2 bag record /joint_states /imu /cmd_vel /camera/image_raw /map /slam/pose
```

Replay the same `/cmd_vel` through `world_sim` on the PC. Where the simulated duck and the real one disagree, fix the model first (mass, servo, friction), gains second. That is the step that makes the simulation worth having.

## Order of work

| # | Do | Needs |
|-|-|-|
| 1 | Steps 1 and 2: board inventory, MCU sketch with watchdog | Board, servos |
| 2 | Step 3: weigh, bench-test, rerun the simulation tests | Scale, one servo, PC |
| 3 | Step 4: stand, then walk with the XGO-Duck policy | Finished robot |
| 4 | Step 5: ROS 2 in Docker, `duck_hw`, `usb_camera_node`, `duck_tof` | Robot walking |
| 5 | Step 6: buy, check, mount and calibrate the B0385 camera and VL53L5CX | Parts |
| 6 | Step 7: photos, labels, retrain YOLO | Camera working |
| 7 | Steps 8 to 10: budget, RViz link, staged bring-up | Everything above |
| 8 | Step 11: bags back into the simulation | Ongoing |

## Not verified

- Nothing here was run on a UNO Q. I do not have the board's default login, the router socket path, or its real Bridge throughput.
- Whether the XGO-Duck repository's policy walks your V3 with the heavier head and the shield. With the corrected masses the simulation now says the Microduck policy does **not** walk straight (107° heading change in 8 s) and cannot get up past the shield; the real answer needs the real robot.
- YOLO and SLAM timing on the QRB2210. The rates in Step 8 are starting guesses.
- The B0385 camera and VL53L5CX carrier are a selection, not tested parts. Nothing about their fit, power, enumeration or performance on this robot is verified.

Sources for the board facts: [UNO Q datasheet](https://mischianti.org/wp-content/uploads/2026/08/Arduino_UNO_Q_ABX00162-ABX00173-datasheet.pdf), [Qualcomm UNO Q page](https://www.qualcomm.com/developer/hardware/arduino-uno-q), [Arduino_RouterBridge](https://github.com/arduino-libraries/Arduino_RouterBridge), [forum: ROS 2 on UNO Q](https://forum.arduino.cc/t/ros2-on-uno-q-making-good-progress/1440548), [forum: TurtleBot3 on UNO Q via Docker](https://forum.arduino.cc/t/installing-running-turtlebot3-on-uno-q-4gb-via-docker-ros2-container/1433149).
