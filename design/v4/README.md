# Duckling V4: A010 depth sensor, boards and LiPo inside the body, opening mouth

24 November 2026. **Simulation and concept CAD only.** No screw bosses, ribs, print splits or wire routing yet; every
clearance must be checked on the real parts. V4 is now the only robot model: the V3 model, its rear shield, the V3 head
STLs and the V3 generator have been deleted (they are in git history).

## What changed against V3

| Area | V3 (deleted) | V4 |
|-|-|-|
| Body | upstream left/right shells + rear shield (clashed with the head in 129/600 sampled poses) | one body shell (`body_shell_v4.stl`, 57 cm³, about 79 g at 30 % infill with 2 mm walls) |
| Boards | old Pi Zero + HAT in the head; 110 mm HAT behind the rear shield | UNO Q + resized 70 × 53.34 mm HAT stack in a rear bay behind the leg sweep |
| Depth sensor | VL53L5CX 8×8 on the head | Sipeed MaixSense A010 in a chest pocket with a front window, level, 0.13 m above the floor |
| Battery | NP-F970-style pack visual | 2S LiPo about 800–1000 mAh (60 × 28 × 15 mm placeholder) in a narrow keel, XT60 pair below it (XT30 also fits) |
| Head | rear servo stuck out about 7 mm; eye plate/ring (`face_part`, `noenoeil`) poked through the sides | 10 % wider, 16 mm longer at the back; eye plate and old head boards removed |
| Mouth | fixed bill | lower half of the beak is a `jaw` on a `mouth` hinge (0–30°), position actuator `mouth` (actuator 16, after the push motors), XL330-size servo under the forehead |
| Limits | upstream | neck pitch capped at 0.70 rad (was 1.047): beyond that, with the head tipped back, the back of the head reaches the board bay |

Mass (`model/mass_ledger.json`, estimates): robot 0.760 → 0.849 kg. Torso 302 g (body shell about 79 g, boards 45 g).
The 2S pack is assumed to weigh the same as the upstream battery. Thinner body walls (1.6 mm) would save about 15 g.

## Fit check (`check_v4_fit.py`, penetration > 0.8 mm)

| | V3 (600 poses) | V4 (3000 poses) |
|-|-|-|
| Poses where a moving part enters a torso part | 129 (head into the rear shield) | **0** (1 without the neck limit: head into the board-bay top) |
| Jaw against the head over 0–30° | – | none |
| Head parts inside the shell walls at rest | lens 1.2, servos 1.4/1.9, eye plate 2.4 mm | lens 1.3 (sits in the opening), servos 1.4/2.1 (same mounting faces as V3), mouth servo 1.0 mm |

Coarse kinematic sample (head joints neck_pitch ±0.4, head_pitch ±0.6, head_yaw ±1.2, head_roll ±0.44 rad; legs ±0.3 rad
around standing), not the gait. An earlier "2 shank clashes" result came from a keyframe bug (the jaw joint shifted the
right leg's start pose by one joint); with the bug fixed no leg reaches the body, so no hip limit is needed.

## SLAM for a body-fixed depth sensor (`ros2/duck_nav`)

The head-mounted ToF mapped by sweeping with the head. A sensor fixed to the body cannot, so:

- **Start-up survey** (`survey_turn`, default 6.6 rad): the duck turns in place a full circle before exploring, and SLAM
  keeps mapping every frame until it has turned that far (`bootstrap_turn`; otherwise a partly new view is rarely
  accepted and never mapped).
- **Confirmed relocalisation** (`reloc_confirm`, default on): a relocalisation jump is held until a match at least
  0.5 s later proposes the same correction (±6 cm / ±4°). Window back at ±0.8 m / ±25° (the narrow ±0.4 m / ±10°
  could not reach heading errors after turns).
- **Look-around** (`weak_look_after`, default 3 s): while localisation is weak or the sensor is *blind* (frames arrive
  but nothing is in range, e.g. closer than the A010's 0.2 m minimum), turn 1.5 s / hold 2 s, never drive forward.
- ToF simulation is batched (`mj_multiRay`), so 100 × 100 zones run in real time.

**First attempt (branch `a010-sim`): better, not solved.** Closed loop, 600 s, realistic profile (all runs held real time):

| Run | Zones | SLAM mean / max error | Beacon | Coverage | Walked | Falls |
|-|-|-|-|-|-|-|
| `v4d_s0` | 25×25 | 0.74 / 1.43 m | no | 50 % | 10.1 m | 1 |
| `v4d_s1` | 25×25 | 0.37 / 1.17 m | no (seen, estimate 0.41 m off) | 70 % | 22.1 m | 1 |
| `v4d100_s0` | 100×100 | 2.69 / 4.72 m | no | 96 % | 19.6 m | 0 |
| `v4c_*` (before the blind fix) | 25 / 100 | 0.22–1.55 m | no | 62–68 % | 8–19 m | 0 |
| `v4_s0_old_keyframe_bug` (no fix) | 25×25 | 1.79 m | no | 58 % | 5.9 m | 0 |
| `a010_head_s0` (V3 body, A010 on the head) | 25×25 | 0.08 m until a fall at 86 s | 46 s | 98 % | 14.6 m | 1 |

The robot no longer freezes, maps much more of the room, and the survey map is good (error under 0.2 m for the first
20–40 s). It then drifts while matches are still *accepted* (0.2 → 0.4–1.5 m within 25–60 s), so matching locks onto a
slightly wrong map; and it spends most of the run "weak". The head-mounted A010 run localised far better. The body
mount needs more SLAM work (likely: keyframe map updates only from confirmed-good poses, a wider normal search while
turning, or a short head-sweep substitute such as slow body yaw oscillation) or the A010 goes on the head. One run per
case; run-to-run noise is large.

### Fixing the drift (branch `v4-body-slam`)

**Diagnosis.** `tools/slam_replay.py` now replays V4 bags (`--profile v4`: 25 × 25 zones, 70 × 60°, 2.5 m; `--zones`,
`--fov`, `--fov-v`, `--max-range` override it) and writes a per-frame CSV (`--dump`: truth and estimate at acquisition,
match reason, overlap, match std, accepted / mapped / relocalising / bootstrap flags). On `v4d_s0`, `v4d_s1` and `v4c_s0`
it showed three separate faults:

1. **Map and pose ratchet together while turning.** In the survey turn the forward command is zero, yet the estimate
   slid 0.1 m sideways in 1 s through a run of *accepted* matches of about 5 mm each, all in the same direction. Zones
   that land just past the mapped end of a wall score as badly as zones in known free space, so the matcher shifts the
   pose to pull them onto the mapped end; the frame is then mapped at that pose, and the next frame repeats it. Every
   frame was mapped (`map_move` = `map_turn` = 0), so the error went straight into the map.
2. **Unconfirmed heading jumps while relocalising.** `reloc_confirm` held *accepted* relocalisation matches but not
   *partial* ones (one direction + heading). With the wide relocalisation prior those swung the heading by 10–30° per
   frame; once the error passed the ±25° window every match hit the search boundary and the robot stayed lost.
3. **Relocalisation that never ends.** Even at 0.2 m error, 85 % of frames were "relocalising": confirmation also held
   corrections of a few cm, so every other frame was `reloc_unconfirmed`, the accepted streak reset, health never
   returned to "ok", and the search stayed wide. In closed loop that is the "weak most of the run" behaviour.

**Changes** (`FrameLocalizer` / `DuckSLAM`; launch `body_slam:=true`, the default, sets them for `sensors:=realistic`;
the node defaults keep the head-sweep behaviour, and `sensors:=ideal` is unchanged):

| Parameter | V4 value | Effect |
|-|-|-|
| `map_move`, `map_turn` | 0.1 m, 0.17 rad | keyframe mapping after bootstrap instead of every frame |
| `unknown_ll` | -1.125 (1.5 σ) | never-observed cells score above known free space (fault 1) |
| `reloc_partial` | false | partial matches are not applied while relocalising (fault 2) |
| `confirm_free_xy`, `confirm_free_th` | 0.08 m, 0.07 rad | smaller relocalisation corrections need no confirmation (fault 3) |

Also new, off by default: `map_max_std` (map only tightly matched frames), `map_accepted_only`, and in the navigator
`yaw_sweep_rate` / `yaw_sweep_period` (body yaw oscillation while walking).

**Replay** (open loop on the recorded path; mean error in m, "ok" = share of time the motion gate would allow driving):

| Policy | `v4d_s0` | `v4d_s1` | `v4c_s0` |
|-|-|-|-|
| recorded live run | 0.74 | 0.37 | 1.33 |
| `a010-sim` policy | 0.34 (ok 8 %) | 0.58 (9 %) | 1.69 (9 %) |
| + `reloc_partial` false | 0.30 | 3.56 | 0.48 |
| + keyframes 0.1 m / 10° | 0.24 | 0.38 | 0.37 |
| + `unknown_ll` -1.125 | 0.25 | 0.24 | 0.15 |
| + `confirm_free` 0.08 m / 4° (**the V4 defaults**) | 0.25 (ok 46 %) | 0.23 (60 %) | 0.16 (82 %) |
| defaults with `map_max_std` 0.02 instead of keyframes | 0.65 | 1.75 | 1.49 |
| `unknown_ll` -0.5 | 0.14 | 0.44 | 0.25 |
| relocalisation window 1.0 m / 40° | 0.24 | 0.55 | 0.27 |
| match gain 1.0 | 0.23 | 0.57 | 0.33 |

Mapping only frames with a tight match covariance (the "confirmed-good poses" idea) was worse: the map stops growing,
so the duck relocalises more. A wider relocalisation window or full match gain did not help.

**Closed loop**, paired on seeds 0–5: `a010-sim` (`v4sbase_*`) against this branch (`v4snew_*`), and this branch with
`yaw_sweep_rate:=0.4` (`v4syaw_*`). 600 s, realistic profile, 25 × 25 zones, launch defaults, one run per cell; every run
held real time (`sim N s, wall N s` equal). A run ends early when exploration stalls with the beacon already reached.

| Seed | `a010-sim`: SLAM mean / max, beacon, coverage, walked | this branch | + yaw sweep 0.4 rad/s |
|-|-|-|-|
| 0 | 0.46 / 0.89 m, no, 70 %, 18.9 m | 0.22 / 0.70 m, **495 s**, 74 %, 36.6 m | 0.17 / 0.57 m, **120 s**, 96 %, 29.1 m |
| 1 | 1.08 / 2.05 m, no, 46 %, 17.5 m | 0.72 / 2.12 m, **253 s**, 82 %, 30.5 m (1 fall) | 0.46 / 0.96 m, **211 s**, 77 %, 37.9 m |
| 2 | 1.13 / 2.39 m, no, 54 %, 18.3 m | 0.45 / 1.81 m, **223 s**, 94 %, 44.6 m | 0.27 / 0.51 m, **193 s**, 73 %, 27.2 m |
| 3 | 2.04 / 3.19 m, no, 62 %, 11.1 m (1 fall) | 0.29 / 0.60 m, **269 s**, 96 %, 32.1 m | 0.32 / 0.76 m, **167 s**, 84 %, 36.6 m |
| 4 | 0.67 / 1.45 m, no, 43 %, 16.3 m | 0.17 / 0.38 m, **153 s**, 94 %, 38.2 m | 0.28 / 0.67 m, **278 s**, 94 %, 32.3 m |
| 5 | 0.25 / 1.23 m, no, 68 %, 12.8 m (1 fall) | 0.52 / 2.13 m, **273 s**, 96 %, 35.4 m | 0.52 / 0.92 m, **230 s**, 81 %, 36.9 m |
| **mean** | **0.94 m, beacon 0/6, 57 %, 15.8 m** | **0.40 m, beacon 6/6 (278 s), 89 %, 36.2 m** | **0.34 m, beacon 6/6 (200 s), 84 %, 33.3 m** |

Beacon estimate error when the run ended: `a010-sim` 0.47 / 2.60 m on the two seeds that ranged it at all; this branch
0.25–0.85 m (median 0.31 m); with yaw sweep 0.14–1.00 m (median 0.44 m).

**Result: the body-mounted A010 now works for this task in simulation.** The duck reached the beacon on 6/6 seeds (0/6
before), covered 89 % of the room instead of 57 %, and walked twice as far; mean SLAM error fell from 0.94 to 0.40 m. It is
still worse than the head-mounted A010 (0.08 m), and late drift remains: three seeds end with 1.4–2.1 m error after
the beacon was reached. The yaw sweep cut the worst-case error on those seeds (max under 1 m on all six, against 2.1 m)
and reached the beacon sooner, at slightly lower coverage. On some seeds it was worse (seeds 3 and 4 by mean error),
and with one run per seed the difference is within the noise, so it stays opt-in until a repeat confirms it. Frame cost
is unchanged (mean 39–54 ms per 25 × 25 frame on a desktop CPU).

Ideas not adopted: a wider normal search while turning was not needed once the ratchet was fixed (the relocalisation
window variant above did nothing). The 0.13 m sensor height is not a problem for `scan_from_points`: at 60° vertical
FOV the lowest zones reach the floor at about 0.23 m, floor returns stay below the 3 cm obstacle threshold with the
simulated 1 % range noise, and the residual error in the dumps was in translation along walls, not in floor/obstacle
classification. On the real A010, range noise and multipath near the floor can be larger; check the floor threshold there.

**100 × 100 vs 25 × 25:** SLAM frame cost on a desktop CPU 16–52 ms (25×25) vs 36–91 ms (100×100) per frame, about 2× more.
On the UNO Q (A53, roughly 5–10× slower than a desktop CPU) 25×25 at about 5–10 Hz is plausible, 100×100 only at 1–3 Hz.
Accuracy was not better at 100×100 in these runs. Binning to 25×25 (or 50×50) is the sensible default.

## Files

- `v4_params.json`: all dimensions (mm). Body, boards, A010, battery and XT60 in the torso frame; head parts in the
  lower-head mesh frame. The `head` block also records how the head shells were made from the V3 head.
- `build_v4.py` (Blender): builds the body shell, placeholders, and splits the jaw from `head_lower_v4_whole.stl`:
  `E:\blender\blender.exe --background --python design/v4/build_v4.py`. Sources: `head_upper_v4.stl`,
  `head_lower_v4_whole.stl`.
- Model: `scripts/prep_meshes.py`, then `scripts/make_duck.py` (`--base robot_groundcontact.xml --tag gc`,
  `--torso-scale 1.1 --tag mplus`, `--torso-scale 0.9 --tag mminus`), `scripts/worldgen.py`,
  `scripts/sync_ros2_assets.py`, `scripts/mjcf_to_urdf.py`.
- `check_v4_fit.py`: the fit check (writes `fit_check.json`).
- `preview.py`: `preview/duck_v4.glb`, `.blend` and renders (`preview/` is git-ignored: upstream meshes).
- `sim_runs/`: `summary.txt`, `result.json`, `map.png`, `launch.log` per run (`v4sbase_*`, `v4snew_*`, `v4syaw_*`: the
  paired trials above).

The simulator launch defaults are this V4 configuration (`ros2 launch duck_nav duck_nav.launch.py sensors:=realistic`);
`tof_zones:=100` for the full A010 array. The old head-mounted VL53L5CX is still available through launch arguments.
