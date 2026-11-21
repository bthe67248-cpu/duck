# Where the ToF scan matcher loses track

21 November 2026. Simulation only. Offline analysis of recorded realistic-profile runs (600 s) with a new per-frame
dump in `tools/slam_replay.py` (`--config '{"dump": "out.json"}'`): for every ToF frame the match reason, whether it
was accepted, whether the matcher was relocalising, the correction applied and the TRUE position/heading error before
and after it. `tools/frame_attribution.py` and `tools/reloc_attribution.py` summarise the dumps.

## Finding: relocalisation causes the failures

`FrameLocalizer` switches to relocalisation after 3 s without a healthy state: a ±0.8 m / ±25° search with a weak prior
(1 m, 0.6 rad) applied at full gain, accepted under the same rule as normal matching (overlap ≥ 0.6). With only a
few ToF returns per sweep (1-8 hits in the current frame), that wide search finds wrong alignments that fit as well
as the right one.

- In the two failed closed-loop runs (`flw_s3`, `flw_s5`) and a failed command run (`cmd_s3`), the worst corrections
  of the whole run were all relocalisation matches: accepted jumps of 0.35-0.9 m that made the position worse, with
  overlap as low as 0.63-0.66. In `flw_s3`, three relocalisation jumps between 98 and 112 s took the error from
  0.3 to 2.2 m and the heading error from 9° to 32°, then 93°; the run never recovered.
- **Every heading jump over 10° came from relocalisation** (`reloc_attribution.txt`: 4/0, 3/0, 1/0, 3/0 on the bags
  that had any; 0 from normal matching). A wrong heading is the most damaging error: everything mapped afterwards is
  rotated, and normal matching then locks onto that wrong map (normal matching's net effect is also harmful on
  the failed runs, after the heading went wrong).
- On the failed runs relocalisation's net effect is harmful (+0.94, +0.81, +0.44 m); on good runs it is small.
- What triggers it: normal matching first fails with `search_boundary` (the needed correction exceeds ±30 cm) and
  `low_overlap`, typically after a period of odometry drift.

## Candidate fixes (offline replay, 18 recordings × 600 s)

Recordings: `hdg_s0..5`, `flw_s0..5`, `cmd_s0..5` (`results/heading`, `results/floor_flow/live_runs`), replayed with
the current policy and command translation. Raw: `reloc_grid.json`.

| Relocalisation | Mean error | Median | Runs > 1 m | Better / worse than current (±2 cm) |
|-|-|-|-|-|
| Current (±0.8 m, ±25°) | 0.86 m | 0.50 m | 6 | – |
| Off | 0.72 m | 0.62 m | 4 | 9 / 6 |
| ±0.8 m, ±5° | 0.69 m | 0.58 m | 6 | 9 / 6 |
| **±0.4 m, ±10°** | **0.66 m** | **0.33 m** | **4** | **9 / 5** |

The smaller window lowers the mean by 23 % and the median by 34 %, and cuts runs over 1 m from 6 to 4, but it is
worse on 5 of 18 recordings (e.g. `hdg_s2` 0.38 → 0.72 m, `flw_s0` 0.19 → 0.30 m), and replay is open loop. It is
not yet the default; a closed-loop seed matrix should confirm it first.

## Further ideas (not tested)

- Require a relocalisation result to be confirmed by a second, independent sweep before applying it (the main
  failure is a single confident wrong jump).
- Do not let relocalisation change the heading by more than a few degrees unless the scan has enough geometry
  (corner, two walls) to pin the heading down.
- Apply relocalisation at reduced gain, like normal matching (0.6).

## Closed-loop check: ±0.4 m / ±10° vs ±0.8 m / ±25°

21 November 2026. Simulation only. The window is now a `slam_node` parameter and a launch argument
(`reloc_window:=0.4 reloc_ang:=0.1745`; defaults unchanged, 0.8 m and 25°). `slam_node` logs the window it uses at
start, and every trial's `launch.log` shows the intended value. 18 runs: realistic profile, seeds 0-8, both settings
from the same commit. They ran three at a time in interleaved pairs, with GPU rendering and YOLO capped at 2
threads. All 18 held real time (`sim N s, wall N s`) and no node died. Run folders: `closed_loop/rwA_s*` (current, A)
and `closed_loop/rwB_s*` (±0.4 m / ±10°, B).

| Seed | Mean SLAM err A / B (m) | Max err A / B (m) | Coverage A / B | Beacon A / B | Falls A / B | Walked A / B (m) |
|-|-|-|-|-|-|-|
| 0 | 1.26 / **0.42** | 2.28 / 1.35 | 75 / 94 % | 94 s / 150 s | 0 / 0 | 17.5 / 37.6 |
| 1 | 0.24 / 0.21 | 1.10 / 0.50 | 97 / 95 % | 109 s / 125 s | 0 / 0 | 35.6 / 27.0 |
| 2 | 3.53 / **1.77** | 4.69 / 2.27 | 30 / 37 % | no / no | 0 / 0 | 14.4 / 8.1 |
| 3 | **0.24** / 1.57 | 0.94 / 3.63 | 98 / 69 % | 241 s / no | 0 / 0 | 39.1 / 23.1 |
| 4 | 1.54 / **0.27** | 2.59 / 0.89 | 73 / 96 % | no / 280 s | 0 / 0 | 20.4 / 33.8 |
| 5 | 2.25 / 1.94 | 4.46 / 3.33 | 76 / 61 % | 96 s / no | 0 / 0 | 15.9 / 24.8 |
| 6 | 1.48 / **0.79** | 3.78 / 1.92 | 94 / 86 % | 150 s / 215 s | 0 / 0 | 37.2 / 33.1 |
| 7 | **0.51** / 1.02 | 2.05 / 1.95 | 95 / 61 % | 125 s / 133 s | 0 / 0 | 28.2 / 19.9 |
| 8 | 0.75 / **0.26** | 2.41 / 0.95 | 97 / 94 % | 115 s / 129 s | 0 / 2 | 33.1 / 39.2 |

| Summary (9 seeds) | A: ±0.8 m / ±25° | B: ±0.4 m / ±10° |
|-|-|-|
| Mean of mean errors | 1.31 m | 0.92 m |
| Median of mean errors | 1.26 m | 0.79 m |
| Runs with mean error > 1 m | 5 | 4 |
| Runs with max error > 2 m | 7 | 3 |
| Better / worse on mean error (±2 cm) | – | 7 / 2 |
| Beacon reached | 7 / 9 | 6 / 9 |
| Room coverage, mean / median | 82 / 94 % | 77 / 86 % |
| Falls | 0 | 2 (seed 8) |
| Path walked, mean | 26.8 m | 27.4 m |

**Heading jumps** (`closed_loop/reloc_attribution.txt`, replay dump of the new bags, each with its own window):
relocalisation caused 9 heading jumps over 10° under A (4/0, 1/0, 1/0, 3/0 on seeds 0, 3, 4, 6) and none under B.
Normal matching caused none under either setting. Part of B's result is built in: a ±10° search cannot make a single
jump of more than 10°. Relocalisation's net effect on the error is mixed under both settings.

A new failure mode appears in the replay. On the seed 6 and 7 bags, B spends most of the run relocalising (4574 and
4213 of 6000 frames, only 5 and 27 accepted), and the replayed error is about 2.1 m. It gets stuck because the
smaller window cannot reach the true pose once the drift is larger than 0.4 m. The live runs on those seeds stayed
lower (0.79 and 1.02 m), and replay is open loop, so how often this happens live is not known.

**Decision: the default stays at ±0.8 m / ±25°.** The smaller window clearly lowers the SLAM error: the mean drops
by 30 %, the median by 37 %, it is better on 7 of 9 seeds, and it removes the large heading jumps. But the robot
does not do better on the task: it reached the beacon less often (6 vs 7), covered less of the room (median 86 vs
94 %), fell twice, and seed 3 went from a clean run (0.24 m) to a failed one (1.57 m, beacon not reached). This
falls short of the bar we set (fewer failures and better mean and median, not one metric). The parameters stay, so
B can be used with `reloc_window:=0.4 reloc_ang:=0.1745`.

Limits: 9 paired seeds and one run per seed and setting. Run-to-run noise on one seed has been 0.12-1.84 m, so a
single pair can flip. The A runs here are worse than earlier current-default runs (`results/heading`), which may be
the noise or the three-at-a-time load. Next ideas: a middle setting (e.g. ±0.6 m / ±10°: keep the translation reach,
cap the heading), or keep ±25° but require a second sweep to confirm a relocalisation jump (see "Further ideas").
