# Floor optical flow vs gait-command odometry in the SLAM replay

19 November 2026. Simulation only. Offline replay (`tools/slam_replay.py`) of the 12 camera-recorded realistic-profile
bags from `results/heading` (`hdg_s0..5`: quaternion heading filter; `tilt_s0..5`: older filter, same seeds), 600 s each,
through the live node's per-frame SLAM policy. Ground truth is used only for scoring. Raw numbers: `replay.json`.

## How flow replaces the command (`translation` config key)

- `command` (current `slam_node`): body translation = gait command × gain × online slip factor.
- `flow`: translation from camera floor optical flow (`tools/flow_odom.py`, no ground truth). Between images the pose
  advances at the last measured body velocity; when pair (s0, s1) is available at s1 + 0.03 s, the measured
  displacement minus what was extrapolated over [s0, s1] is added to the current pose (causal). Slip scaling off,
  heading still from the attitude filter, fall gate unchanged. A pair with no fit counts as zero motion.
- `flow_or_command`: as `flow`, but the gait command is used for pairs with no fit.
- `flow_method`: `image` (LK in the camera image; under-estimates motion by ~4-8 %), `rect` (LK in floor-rectified
  top-down images, `floor_flow.track_rect`; scale 0.96-1.00 but fewer fits) or `rect_or_image`. See `NOTES.md`.

## Result: mean SLAM error (m) per bag

| Bag | Command | Flow or command, image | Flow or command, rect | Flow or command, rect then image | Flow only, rect then image |
|-|-|-|-|-|-|
| hdg_s0 | 1.65 | 0.52 | 0.26 | 0.37 | 0.52 |
| hdg_s1 | 0.39 | 0.38 | 0.38 | 0.55 | 0.21 |
| hdg_s2 | 0.38 | 0.51 | 0.43 | 0.61 | 0.98 |
| hdg_s3 | 0.23 | 0.12 | 0.19 | 0.14 | 1.42 |
| hdg_s4 | 0.46 | 0.17 | 0.37 | 0.11 | 1.31 |
| hdg_s5 | 0.79 | 0.07 | 0.22 | 0.13 | 0.35 |
| tilt_s0 | 0.38 | 0.30 | 1.59 | 0.40 | 1.92 |
| tilt_s1 | 0.87 | 2.25 | 0.74 | 1.82 | 1.23 |
| tilt_s2 | 0.82 | 0.35 | 0.87 | 0.33 | 0.90 |
| tilt_s3 | 0.61 | 0.54 | 0.58 | 0.63 | 1.20 |
| tilt_s4 | 0.25 | 0.28 | 0.22 | 0.22 | 0.29 |
| tilt_s5 | 0.34 | 0.20 | 0.20 | 0.05 | 0.37 |
| **Mean** | **0.60** | **0.47** | 0.50 | 0.45 | 0.89 |
| **Median** | 0.42 | **0.32** | 0.37 | 0.35 | 0.94 |
| Better than command | – | **9 / 12** | 9 / 12 | 7 / 12 | 3 / 12 |

## Reading

- Flow with the command as fallback lowers SLAM error on 9 of 12 recordings: mean 0.60 → 0.45-0.50 m, median
  0.42 → 0.32-0.37 m. Pure flow is worse: about a third to half of the pairs have no fit, and counting them as zero
  motion loses distance.
- Single bags swing a lot (tilt_s1: 0.87 → 2.25 m with the image tracker but 0.74 m with rect; tilt_s0: 0.38 →
  1.59 m with rect). The replay is open loop: the estimate cannot change where the robot actually walked, and one
  early wrong scan match changes a whole run. The three trackers cannot be ranked from these numbers, even though per
  frame pair the rect tracker is clearly more accurate (`NOTES.md`).
- The deciding test is closed loop: flow in `slam_node`, several seeds, as for the heading filter. Cost measured
  here: 11-28 ms per image pair in Python + OpenCV on the PC at 5 Hz (expect several times more on the UNO Q's A53
  cores, still within a 200 ms frame budget if it gets its own core; not measured on hardware).
- The simulated floor is a strongly textured checker. Real floors may give far fewer features; plain or glossy
  floors could defeat floor flow entirely. Check feature counts on real camera images before relying on it.
