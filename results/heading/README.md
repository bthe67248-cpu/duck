# Heading from a full attitude filter (branch `measured-heading`)

19 November 2026. Simulation only (realistic profile: USB camera + simulated 8×8 ToF, 600 s, default room).

## What changed

`slam_node` used to get heading for odometry by projecting the gyro onto a filtered "up" vector
(`TiltFilter` + `world_yaw_rate`). That is a planar approximation: while the torso rolls and pitches through the
gait, the projected rate is not the true yaw rate, and the error accumulates. `duckslam.AttitudeFilter` tracks
the full 3-D attitude as a quaternion (Mahony-style complementary filter) and feeds odometry the change of the
quaternion's yaw:

- roll/pitch are pulled towards the accelerometer only when |a| is within 15 % of 1 g; their gyro bias is learned
  from the same error
- yaw is never corrected by the filter (scan matching does that)
- yaw gyro bias cannot be learned while walking: an online zero-velocity update was tried and dropped, because
  the torso keeps turning about 1 deg/s while commanded to stand and it learned that as bias (0.016 rad/s on a
  bias-free recording). Use `gyro_calib_s:=N` to measure the bias over the first N s with the robot held still
  (on hardware; the simulated IMU has no bias or noise)

`attitude_filter:=tilt` selects the old behaviour. Unit tests: `ros2/duck_nav/test/test_attitude.py`.

## Open-loop heading error (recorded runs, perfect simulated gyro)

Mean |heading error| against ground truth until the first fall (`tools/heading_sweep.py`):

| Run | Old (`TiltFilter`) mean / final | New (`AttitudeFilter`) mean / final |
|-|-|-|
| `new_s1` (605 s) | 11.0 / 16.0° | 6.2 / 10.1° |
| `new_s2` (327 s) | 9.1 / 31.2° | 3.7 / 5.5° |
| `main_s1` (237 s) | 13.8 / 30.7° | 7.3 / 12.3° |
| `main_s2` (72 s) | 2.8 / 21.5° | 2.2 / 0.5° |

The filter gain barely matters (pure gyro integration gives the same ~5° average). What is left is a sampling
limit of the simulated gyro (read at 50 Hz while the gait oscillates faster); a real IMU integrates internally at
a higher rate. With an injected gyro bias (0.01 / −0.01 / 0.005 rad/s) plus a startup calibration, the new filter
is unaffected; the old filter, which has no bias handling, drifts to 100–167°.

## Closed loop, seeds 0–5, same conditions

Both filters ran from the same commit at the same time (three trials in parallel, each checked to stay at
simulated time = wall time, GPU rendering, YOLO capped at 2 threads). Cells: mean SLAM error, room covered,
beacon reached.

| Seed | `attitude_filter:=quaternion` (new default) | `attitude_filter:=tilt` (old) |
|-|-|-|
| 0 | 0.72 m, 92 %, 79 s | 3.48 m, 35 %, not reached |
| 1 | 0.47 m, 73 %, 207 s | 2.14 m, 22 %, not reached |
| 2 | 0.41 m, 92 %, 118 s | 0.41 m, 86 %, 138 s |
| 3 | 0.13 m, 93 %, 88 s | 0.82 m, 86 %, 140 s |
| 4 | 0.15 m, 93 %, 148 s (stalled at 300 s) | 0.70 m, 79 %, 120 s (stalled at 210 s) |
| 5 | 0.52 m, 74 %, 120 s | 0.49 m, 69 %, not reached |
| **Mean** | **0.40 m, 86 %, 6 / 6 reached** | **1.34 m, 63 %, 3 / 6 reached** |

The new filter is clearly better on seeds 0, 1, 3 and 4 and level on 2 and 5. Earlier old-filter runs on
seeds 0–2 (`results/post_fix`, `results/post_merge/sim_runs`) spread from 0.12 to 1.84 m mean error, so single
runs remain noisy; six paired seeds in one room is still a modest sample. Falls: 4 of 6 new runs and 2 of 6
old runs had at least one fall. The filter cannot change the gait; the new runs walked further (mean 25 m vs
18 m), which gives more chances to trip, so this is no evidence of a heading effect.

Offline SLAM replay of the four older bags (`runs/replay/`) was mixed (one bag much worse, one better, two
level): replay is open loop, so one early wrong match changes the whole map. Closed-loop trials are the
deciding test here.

Run folders: `runs/hdg_s*` (new) and `runs/tilt_s*` (old).

## Recordings now include the camera

`tools/trial_matrix.py` (and the trials above) record `/camera/image_raw` as well as camera info, head pose
(`/camera/pose_level`), joint states and detections, about 0.7 GB per 600 s run. The bags of the twelve
trials above are in WSL `~/duck_bags/{hdg,tilt}_s0..5` (not in the repository). They are the input for the
next step: estimating forward motion from the camera (floor optical flow with the known camera height and head
joint angles) and scoring it against the gait-command odometry on these identical recordings.
