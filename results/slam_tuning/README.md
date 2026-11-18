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
then drifts (0.6 m mean in the last 80 s, 2.3 m at the end): no loop closure yet. One seed only; `main` also
varies a lot between runs (a second `main` run recorded here had 1.00 m mean error).

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
   ~8 ms per frame on the PC (expect 5–10× on the UNO Q's A53 cores).
8. **Camera + ToF beacon ranging**: camera ray through the box centre at the ToF depth (zones are 5.6° apart,
   the camera bearing is far more precise); known target height gives a box-size depth used as a fallback and
   to reject ToF hits on the background.

Replay, mean error over three recordings (308 / 598 / 596 s): **0.42 / 0.27 / 0.53 m**; `main`'s logic replayed on
the first: 0.50 m; the patch's rules: 2.35 m.

Tried and rejected (measured worse): finer sub-cell search (map cells set the floor), narrow ±10 cm window,
the patch's current-frame agreement check, GMapping-style keyframe map gating, gain 1.0, 1.5 s sweep.

## Honest limits

- Results are sensitive to small policy changes; three recordings and one closed-loop seed are not enough to
  call the tuning final. Run more seeds and room layouts before relying on the numbers.
- No loop closure or pose graph: long runs still drift (the last 100 s above).
- Simulated ToF: point rays, not real zone cones; the real sensor's noise and zone order are unmeasured.
- Camera visual odometry (VINS-RGBD-style) was researched but not built.

## Next

1. Loop closure: keep submaps/keyframes and match the current sweep against older parts of the map
   (correlative search over a wide window is already cheap), then a small pose-graph correction.
2. More seeds/layouts in closed loop; record each run and add it to the replay set.
3. Visual odometry from the USB camera with ToF depth for scale, to replace the gait-command odometry.
