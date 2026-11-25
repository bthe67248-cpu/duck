# ToF + camera SLAM improvements (branch `depth-camera-opt`)

18 November 2026. Simulation only (realistic profile: noisy RGB camera + simulated 8×8 VL53L5CX). Nothing run on
hardware. Builds on the hardware-readiness patch (see `results/hardware_readiness/REVIEW_BY_CLAUDE.md`).

## Result: closed-loop ROS 2 room trial, seed 0, 600 s

| | `main` (before the patch) | Patch | **This branch** |
|-|-|-|-|
| Beacon | reached at 100 s | not reached | **reached at 360 s** |
| Beacon position error | 0.19 m | – | **0.10 m** |
| Path walked | 33.6 m | 6.4 m | **39.4 m** |
| Room covered | 90 % | 30 % | **97 %** |
| SLAM error, mean | 0.25 m | 0.36 m | 0.26 m |
| SLAM error, max / final | 1.02 / 0.29 m | 0.58 / 0.51 m | 2.41 / 2.29 m |

Run folder: `results/post_fix/ros2_nav_realistic_new3_s0`. Error per 40 s stays 0.12–0.28 m until about 500 s,
then drifts (0.6 m mean in the last 80 s, 2.3 m at the end): no loop closure yet. `main` also varies a lot
between runs (a second `main` run recorded here had 1.00 m mean error).

## Seeds 0–2, this branch vs `main` (realistic profile, 600 s)

| | Seed 0 branch | Seed 0 `main` | Seed 1 branch | Seed 1 `main` | Seed 2 branch | Seed 2 `main` |
|-|-|-|-|-|-|-|
| Beacon | reached, 360 s | reached, 100 s | reached, 202 s | reached, 180 s | not reached (1.48 m) | not reached (1.04 m) |
| Beacon position error | 0.10 m | 0.19 m | 0.19 m | 1.32 m | 1.04 m | 0.35 m |
| Path walked | 39.4 m | 33.6 m | 35.2 m | 36.6 m | 22.1 m | 12.5 m |
| Room covered | 97 % | 90 % | 98 % | 86 % | 77 % | 61 % |
| SLAM error, mean / max | 0.26 / 2.41 m | 0.25 / 1.02 m | 0.44 / 1.87 m | 1.06 / 1.43 m | 0.87 / 1.86 m | 0.21 / 0.52 m |
| Falls | 0 | 0 | 0 | 1 | 1 | 2 |

Folders: `results/post_fix/ros2_nav_realistic_{new3_s0, v3model, new_s1, main_s1, new_s2, main_s2}`
(seed 0 `main` is the earlier `v3model` run; it ended at "exploration stalled" before the time limit).

**Honest reading.** The branch explores more on every seed (coverage 97/98/77 % vs 90/86/61 %, mean 91 vs
79 %), and both reach the beacon on 2 of 3 seeds. SLAM accuracy is **not** better overall: mean error over the
three seeds is 0.52 m for the branch and 0.51 m for `main`. The branch wins seed 1 clearly (0.44 vs 1.06 m),
`main` wins seed 2 clearly (0.21 vs 0.87 m; `main` walked only 12.5 m there, so it had less chance to drift).
The branch's worst errors come late in long runs, from drift without loop closure. Three seeds in one room
is still a small sample.

Seeds 1 and 2 were run with the faster test setup below (GPU rendering, YOLO capped at 2 threads, up to three
trials at once, each checked to stay at simulated time = wall time); seed 0 ran alone on the CPU renderer.
Rendering and thread count do not change the algorithms, but they change timing slightly, so seed 0 and
seeds 1–2 are not bit-for-bit comparable setups.

## Ideal (depth camera) profile, seed 0

| | Old grid matcher (`main` code, `ideal_v3model_final`) | Correlative, single frames | Grid, single frames (patch rules) | Grid + 1 s sweep | **Correlative + 1 s sweep (new default)** |
|-|-|-|-|-|-|
| Beacon | reached, 49 s | not reached | not reached | reached, 156 s | **reached, 54 s** |
| Beacon position error | 0.15 m | – | – | 0.40 m | **0.14 m** |
| Path walked | 24.3 m | 3.8 m | 1.6 m | 16.5 m | **20.9 m** |
| Room covered | 99 % | 36 % | 26 % | 84 % | **99 %** |
| SLAM error, mean / max | 0.09 / 0.31 m | 0.68 / 1.62 m | 0.01 / 0.03 m | 0.35 / 0.53 m | **0.09 / 0.25 m** |
| End | stalled at 142 s | stalled at 52 s | timeout, standing | timeout | stalled at ~180 s |

The correlative matcher first ran in depth mode on single camera frames. A ±30 cm search over one narrow
view locked onto wrong alignments: 163 of 232 matches were "accepted" with 0.96 overlap while the true error
grew, so map and pose drifted together. The grid matcher with the patch's rules was accurate but rejected
almost every single-wall frame as `degenerate_geometry`, so `/slam/state` stayed `weak` and the duck stood
still. Matching over the same 1 s sweep window as the ToF fixes both: `depth_sweep` now defaults to true.
`matcher:=grid` and `depth_sweep:=false` are launch arguments for comparison (`matcher` was a node parameter
before but was not passed through the launch file). Folders: `results/post_fix/ros2_nav_ideal_{corr,
grid, grid_sweep, corr_sweep}`. A repeat with the new defaults and no launch arguments
(`ros2_nav_ideal_default`): beacon at 50 s, 0.13 m beacon error, 98 % covered, SLAM error 0.07 / 0.18 m.

## Faster test runs

Each trial runs at real time (600 s simulated = 10 min), so the speed-up is running several at once. Under
WSL the camera was rendered by Mesa's CPU rasteriser (llvmpipe); `GALLIUM_DRIVER=d3d12` renders on the GPU
(test scene 22.6 → 6.2 ms per frame; `run_nav_headless.sh` now sets it when `/dev/dxg` exists). ONNX
Runtime took every core for YOLO (~3.6 cores per run); `DUCK_ORT_THREADS=2` caps it without changing the
detections (14 ms per inference). With both, three trials ran side by side at simulated = wall time (load
about 8–12 on 12 cores); before, three trials fell to 2.5× slower than real time. Physics (MuJoCo) and the
Python planner still use about 2 cores per trial; parallel runs need their own colcon workspace and
`ROS_DOMAIN_ID`.

Faster than real time: `real_time:=false` (unpaced) or `rt_factor:=N` (at most N× real time). Seed 1, branch:

| | 1× (reference) | `rt_factor:=2.0` | `real_time:=false` (~3.3×) |
|-|-|-|-|
| Wall time to 600 s simulated | 10 min | 5 min | ~3 min |
| End | timeout at 600 s | stalled at ~320 s | stalled at ~170 s |
| Beacon | reached, 202 s | reached, 96 s | reached, 136 s |
| Room covered | 98 % | 93 % | 65 % |
| SLAM error, mean / max | 0.44 / 1.87 m | 0.17 / 0.77 m | 0.20 / 0.76 m |
| `stale pose` / expired-command events | 0 | 2 | 3 |

Unpaced, SLAM output falls behind the faster clock (`stale pose` stops the duck) and exploration ended early,
so unpaced runs are not used for comparisons. 2× is usable for quick checks, but one seed cannot show that
it matches 1× (closed-loop runs diverge from small timing differences anyway); all tables above are 1×.
Folders: `results/post_fix/ros2_nav_realistic_new_s1_{rt2,unpaced}`.

## What changed, and why (each step measured)

Method: three runs were recorded (`ros2 bag`, ToF, IMU, commands, ground truth) and replayed offline through the
SLAM pipeline with `ros2/duck_nav/tools/slam_replay.py` (seconds per variant instead of 11-minute trials). The
replay and the node share the per-frame logic (`duckslam.FrameLocalizer`).

1. **Correlative scan matching** (Olson, *Real-Time Correlative Scan Matching*, ICRA 2009). Log-likelihood table
   from distance-to-wall blurred by sensor noise; ±30 cm / ±12° search with an exact two-level branch and bound;
   Gaussian odometry prior; covariance from the score distribution (Olson eq. 3). Along a featureless wall only
   the across-wall position is corrected. Synthetic corner, start errors up to 28 cm / 8°: 150/150 recovered,
   1.8 cm residual (old ±7.5 cm grid search: 11/150).
2. **Online slip estimation.** `tools/odom_check.py` showed the biggest dead-reckoning error is the duck pushing
   against obstacles: gait odometry +1.3 m, true motion 0.1 m. The ratio of scan-corrected to modelled forward
   motion (only from corrections that constrain the forward direction) scales the odometry.
3. **Bootstrap deadlock fixed.** Ending bootstrap at the first accepted match let that frame's free-space update
   push the map back under the matcher's minimum; nothing was ever accepted again (live run = dead reckoning).
4. **Map growth while healthy**, not only on accepted matches (the patch froze the map at its first view).
5. **Relocalisation**: after 3 s without healthy localisation, the search widens to ±0.8 m / ±25°
   (synthetic: 20/20 from up to 85 cm / 20°, 6 ms).
6. **Empty ToF frames are not failed matches** (counting them reset the health streak and diverged).
7. **Faster matcher**: the old grid search vectorised (identical result, ~30×); the correlative search runs in
   ~8 ms per frame on a desktop CPU (expect 5–10× on the UNO Q's A53 cores).
8. **Camera + ToF beacon ranging**: camera ray through the box centre at the ToF depth (zones are 5.6° apart,
   the camera bearing is far more precise); known target height gives a box-size depth used as a fallback and
   to reject ToF hits on the background.

Replay, mean error over three recordings (308 / 598 / 596 s): **0.42 / 0.27 / 0.53 m**; `main`'s logic replayed on
the first: 0.50 m; the patch's rules: 2.35 m.

Tried and rejected (measured worse): finer sub-cell search (map cells set the floor), narrow ±10 cm window,
the patch's current-frame agreement check, GMapping-style keyframe map gating, gain 1.0, 1.5 s sweep.

## Honest limits

- Results are sensitive to small policy changes, and three closed-loop seeds in one room are not enough to
  call the tuning final; the branch explores more but is not more accurate than `main` on average.
- No loop closure or pose graph: long runs still drift (the last 100 s above).
- Simulated ToF: point rays, not real zone cones; the real sensor's noise and zone order are unmeasured.
- Camera visual odometry (VINS-RGBD-style) was researched but not built.

## Later: heading filter

Heading now comes from a full attitude filter; on seeds 0-5 it cut mean error from 1.34 m to 0.40 m and
reached the beacon 6/6 times instead of 3/6. See [`results/heading/README.md`](../heading/README.md).

## Next

1. Loop closure: keep submaps/keyframes and match the current sweep against older parts of the map
   (correlative search over a wide window is already cheap), then a small pose-graph correction.
2. More seeds and room layouts in closed loop; record each run and add it to the replay set
   (seeds 1–2 are recorded as bags but not yet in the replay set).
3. Visual odometry from the USB camera with ToF depth for scale, to replace the gait-command odometry.
