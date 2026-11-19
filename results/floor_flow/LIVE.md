# Floor optical flow inside `slam_node` (closed loop)

20 November 2026. Simulation only (realistic profile, 600 s, default room). `translation:=flow_or_command` runs the
floor-flow estimator (`duck_nav/flowodom.py`, `flow_method:=rect_or_image`) inside `slam_node` and uses the gait
command only for image pairs without a fit. Both configurations ran from the same commit at the same time (three
trials in parallel, each checked to stay at simulated time = wall time). Cells: mean SLAM error, room covered,
beacon. Run folders: `live_runs/flw_s*` (flow) and `live_runs/cmd_s*` (command).

| Seed | Flow + command fallback | Command only |
|-|-|-|
| 0 | 0.12 m, 97 %, 133 s | 0.20 m, 96 %, 157 s |
| 1 | 0.59 m, 68 %, 582 s | 0.65 m, 90 %, not reached |
| 2 | 0.71 m, 86 %, 84 s | 0.54 m, 24 %, not reached (stalled after 4.8 m) |
| 3 | 1.33 m, 58 %, not reached | 0.68 m, 69 %, not reached |
| 4 | 0.42 m, 84 %, 128 s | 0.68 m, 48 %, not reached |
| 5 | 2.15 m, 65 %, not reached | 0.83 m, 81 %, 183 s |
| **Mean** | **0.89 m** (median 0.65), 76 %, **4 / 6**, 0 falls | **0.60 m** (median 0.67), 68 %, 2 / 6, 3 falls |

## Reading

- **No clear winner.** Flow reached the beacon in 4 of 6 runs (command 2 of 6) and covered more of the room, the
  medians are the same, but two flow runs (seeds 3 and 5) ended with large errors and pull its mean above the
  command's.
- **Those two failures are not floor-flow failures.** On both recordings the flow estimate itself was accurate
  (median 0.22 cm per image pair, scale 0.98, 0.1-0.2 % of pairs off by more than 3 cm; `tools/floor_flow.py`). An
  offline replay of the same recordings with *command* odometry fails too: on seed 3 the error rises to 3-3.7 m in
  the first two minutes in both modes (flow later recovers to 0.5 m, command stays at 3.7 m); on seed 5 both sit at
  1-2 m. The trajectories went where the scan matcher loses track, whichever odometry feeds it.
- **Run-to-run noise is as large as the effect.** The command-only runs here (mean 0.60 m, beacon 2 / 6) are much
  worse than the same configuration a day earlier (`results/heading`: 0.40 m, 6 / 6). Six paired seeds in one room
  cannot separate the two odometry sources at the SLAM level.
- Live cost: 28-39 ms per image pair at 5 Hz with three trials sharing the PC; 62-92 % of pairs had a fit.

## Decision

`translation` stays `command` by default; `flow_or_command` is available as an opt-in. Floor flow measures motion
much better than the gait command (offline: per-pair error ~0.2 cm vs ~1 cm, and it sees pushing against obstacles),
but SLAM accuracy here is limited by scan-matching failures that hit both. The next improvements to localisation
should target those (what the ToF sees when the matcher loses track), and a larger seed matrix is needed before
switching the default. On hardware, check floor texture first: the simulated checker floor is ideal for this method.
