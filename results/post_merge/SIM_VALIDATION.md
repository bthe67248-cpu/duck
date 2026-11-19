# Simulation check of the post-merge patch

19 November 2026. The patch in this folder was applied unchanged to `d08af6c` (`main` after PR #1) and run in the
ROS 2 / MuJoCo simulation. Realistic profile (USB camera + 8×8 ToF) unless noted, 600 s, default room, real time.
Up to three trials ran in parallel (own colcon workspace and `ROS_DOMAIN_ID` each, GPU rendering, YOLO capped at
2 threads); every trial was checked to keep simulated time = wall time. Run folders: `sim_runs/<name>/`.

## Default path (loop closure off): no measurable change, large run-to-run spread

| | Seed 0 | Seed 1 | Seed 2 |
|-|-|-|-|
| `main`, earlier runs (`results/post_fix/*new3_s0, new_s1, new_s2`) | 0.26 m, 97 %, beacon 360 s | 0.44 m, 98 %, beacon 202 s | 0.87 m, 77 %, no beacon |
| `main`, rerun today (`mainm_s*`) | 0.98 m, 83 %, beacon 159 s | 0.69 m, 67 %, no beacon | 0.12 m, 96 %, beacon 217 s |
| This patch (`pmr_s*`) | 1.84 m, 38 %, no beacon; repeat (`pmr_s0b`): 0.21 m, 97 %, beacon 221 s | 0.76 m, 50 %, no beacon | 1.10 m, 69 %, beacon 135 s |

Cells: mean SLAM error, room covered, beacon. The same code on the same seed ranges from 0.21 to 1.84 m mean error
(patch seed 0) and from 0.12 to 0.87 m (`main` seed 2), so three seeds cannot rank the two. The only default-path
code change is carrying a scan-match correction to the current pose as a rigid SE(2) transform instead of adding
x, y and heading. The lever arm is the motion between acquisition and processing (centimetres), so the effect is
millimetres. Averages (patch 0.98 m / 64 % over 4 runs, `main` 0.56 m / 86 % over 6) lean towards `main`, but
that is inside the observed spread. A ten-seed matrix is needed to say more.

Ideal profile, seed 0 (`ideal_pmr`): 0.06 m mean error, beacon at 77 s, 99 % covered. No regression.

## Experimental loop closure (`loop_closure:=true`): worse, never closed a loop. Keep it off.

| | Seed 0 | Seed 1 | Seed 2 | Ideal, seed 0 |
|-|-|-|-|-|
| Live trial (`pmr_lc_s*`, `ideal_pmr_lc`) | 0.94 / 2.85 m, 30 %, no beacon | 2.38 / 6.01 m, 26 %, no beacon | 1.28 / 3.26 m, 32 %, no beacon | 0.12 / 0.21 m, 97 %, beacon 101 s |

Offline replay of the recorded `new_s1` / `new_s2` bags with `tools/slam_replay.py` (`sim_runs/replay/`):

| Replay, mean / max error | Loop closure off | Loop closure on |
|-|-|-|
| Seed 1 | 0.11 / 0.79 m | 1.13 / 2.29 m |
| Seed 2 | 1.06 / 1.84 m | 0.90 / 3.34 m |

In both replays the graph reached its 160-keyframe cap with `revision` 0, i.e. no closure was ever accepted. The
damage comes from what graph mode changes even without a closure: map keyframes are coarsened to 15 cm / 15°
(default 0 / 0 in the node), so the map the matcher works against is sparser. Before loop closure can help, it
needs closures that fire on real revisits and a frontend that keeps its normal map spacing while the graph is on.

## Tools

- `trial_matrix.py` planning works (6 planned entries for 3 seeds × rates 1, 2). Execution was not exercised here.
  It launches ROS directly, so it does not set `GALLIUM_DRIVER=d3d12` (GPU rendering under WSL) or
  `DUCK_ORT_THREADS`; export both before running parallel workers.
- `bag_suite.py trim` correctly refused the seed-0 bag (`~/duck_bags/realistic_new3_s0`): a stray recorder kept
  writing later runs into it, so its clock restarts. There is no clean seed-0 bag; re-record seed 0 instead.
- Unit tests: 135 pass; CI passes on the branch.
