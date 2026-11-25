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

**Result: better, not solved.** Closed loop, 600 s, realistic profile (all runs held real time):

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

**100 × 100 vs 25 × 25:** SLAM frame cost on this PC 16–52 ms (25×25) vs 36–91 ms (100×100) per frame, about 2× more.
On the UNO Q (A53, roughly 5–10× slower than this PC) 25×25 at about 5–10 Hz is plausible, 100×100 only at 1–3 Hz.
Accuracy was not better at 100×100 in these runs. Binning to 25×25 (or 50×50) is the sensible default.

## Monocular depth (`mono_depth/`)

Depth Anything V2 Metric Indoor Small (24.8 M parameters) on 30 simulated head-camera images vs exact depth:
metric depth was 3–4× too far (median scale 0.23–0.34; relative error 2.5–4.0), about 8 % relative error only after a
per-frame scale fit, and the example image shows it barely separates a near wall from the background. Simulator
renders from 13 cm height are far from its training photos, so this says little about real images. The decisive number
is speed: 0.30 s (266 × 350 input) to 0.67 s (392 × 518) per frame on this PC with 4 threads, so several seconds per
frame on the UNO Q. Not useful for navigation; the A010 gives metric depth directly.

## ESP32 camera + YOLO

Moving the camera to an ESP32-S3 board (e.g. OV2640) frees UNO Q cores for the 100 × 100 depth, but YOLO itself on an
ESP32-S3 is slow: Espressif's own workshop lists seconds per frame for YOLO11n (0.5–2.5 s even on the faster ESP32-P4);
only much smaller detectors (YOLOX-Nano int8, 4–6 fps reported) run near video rate, with lower accuracy. The practical
split: the ESP32 streams JPEG frames over Wi-Fi or USB, and YOLO stays on the UNO Q (2 threads), with the depth at 25×25.

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
- `mono_depth/`: the monocular depth test, `results.json`, `example.png`.
- `sim_runs/`: `summary.txt`, `result.json`, `map.png`, `launch.log` per run.

The simulator launch defaults are this V4 configuration (`ros2 launch duck_nav duck_nav.launch.py sensors:=realistic`);
`tof_zones:=100` for the full A010 array. The old head-mounted VL53L5CX is still available through launch arguments.
