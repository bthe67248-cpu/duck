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
