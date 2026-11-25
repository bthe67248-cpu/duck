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

Mass (`model/mass_ledger.json`, estimates): robot 0.760 (upstream) → 0.789 kg. Torso 243 g (body shell about 71 g,
boards 45 g, 2S LiPo 55 g). See "Mechanical validation and dynamics" below for the battery, head budget and servo delay.

## Mechanical validation and dynamics (review fixes, 24 November 2026)

A review found that the fit check missed deep penetration, the LiPo kept the upstream battery's mass, the servo delay
was recorded but not simulated, and the realistic profile published exact IMU readings and sensor poses. All four are
fixed; the fit check now reports real interferences that the old check could not see.

### Fit check (`check_v4_fit.py`, penetration > 0.8 mm)

**Method.** The old check looked for a surface sample of the other part within 2.5 mm and projected onto its normal,
one direction only, so a point 3 mm inside a part reported 0 mm, and a thin wall pushed through a part was invisible.
Now every visual mesh is a closed solid (`solid.py`): containment from signed vertical ray crossings (a generalised
winding number, robust to small gaps), depth = distance to the surface with no cut-off, and every pair is checked in
both directions (A's samples in B and B's in A). Tested against analytic solids (`tests/test_fit_solid.py`: sphere
boundary within 0.3 mm, depth within 0.25 mm, a 3 mm-deep point reports 3.0 mm where the old method gave 0, a 2 mm
wall through a block reports its 19 mm reach where the old one-way check saw 1 mm). The deepest results were confirmed
on the full-resolution STLs (the simulation meshes are decimated to 20k faces): same or deeper. The same pairs run
over the random pose envelope and over recorded motions (`scripts/motions.py --record DIR`, then `--traj DIR/*.npz`),
next to the MuJoCo collision proxies' self-contacts. The upstream Microduck head, checked the same way, has no pair
deeper than 1.1 mm (CAD contact faces), so 0.8 mm is close to the noise floor for upstream parts.

**Static fit (standing, mouth closed): pairs deeper than 0.8 mm.** Before = the old method's number for the same
pair, where the old check covered it at all (it only checked parts inside their own body's shells).

| Pair | new (mm) | old method (mm) | Cause |
|-|-|-|-|
| upper head shell / camera lens | **6.7** | 1.4 | the lens opening is smaller than the lens after the 10 % widening |
| mouth servo / M12 lens holder | **6.1** | 2.5 (band limit) | the mouth servo placeholder sits on the camera holder |
| lower head shell / head-roll servo (`yaw_roll_motion` XL330) | **5.1** | not checked | V4 reshaping (upstream shell: < 0.3 mm) |
| lower head shell / neck-pitch bearing | **3.7** | not checked | V4 reshaping (upstream: none) |
| lower head shell / neck-pitch bracket | **2.5** | not checked | V4 reshaping (upstream: none) |
| lower head shell / head XL330 (in the head) | 2.3 | 2.3 | mounting face, as upstream (≤ 1.1 mm there) |
| upper head shell / mouth servo | 2.2 | 2.2 | mouth servo placeholder |
| motor support / mouth servo | 1.6 | 1.5 | mouth servo placeholder |
| 2S LiPo / upstream power support (old battery holder) | 1.1 | 1.0 | the holder is still in the model; trim or delete it |
| lower head shell / roll bearing, mouth servo, motor support | 0.9–1.1 | 0.9–1.1 | contact faces |
| keep-out envelopes | 2.0–5.8 | – | see "Connector envelopes" below |

The old method also reported 1.4 mm for the closed jaw against the lower shell; the new one finds no overlap there.
The table in the earlier V4 write-up ("lens 1.3 mm, servos 1.4/2.1, mouth servo 1.0 mm") under-reported every pair.

**Pose envelope (3000 random poses, 1697 part pairs, seed 1).** Every pose clashes, because the four head/neck
overlaps above already exist at rest. Clashes in pairs that are clear at rest (caused by motion): **800 / 3000 poses**.

| Pair | poses (new) | poses (old method) | worst (mm) |
|-|-|-|-|
| jaw / lower head shell | 619 | 377 | 1.0 |
| lower head shell / neck servo (`neck` XL330) | 106 | 100 | 2.7 |
| left foot / right foot (random hip yaw/roll ±0.3 rad crossing the feet) | 76 | 74 | 3.3 |
| `usb_c_top` envelope / head shells (head tipped back over the board bay) | 60 | 48 | 3.2 |
| any moving part / body shell, boards, LiPo, XT60, A010, power support | 0 | 0 | – |

The jaw rubs the lower shell from about 15° opening: 0.48 mm at 15°, 0.72 at 20°, 1.0 mm at 30° (confirmed on the
full-resolution meshes). Move the hinge axis back/down or chamfer the jaw's rear edge, or limit the mouth to ~12°.

**Recorded motions** (`motions.py all --record`, final model with the LiPo and 20 ms servo delay, every 50 Hz frame).
"Frames" counts clashes in pairs that are clear at rest; proxy = MuJoCo collision-proxy self-contacts > 0.8 mm.

| Motion | frames | clashes caused by motion | proxy contacts |
|-|-|-|-|
| balance, walk, pick, dance | 550, 500, 315, 1100 | none | none |
| sit/stand | 475 | none | 2 frames, right foot / body front |
| roulade | 375 | 11: lower head shell 1.2 mm and jaw 1.1 mm into the body shell (head tucked); head over the `usb_c_top` space | 17: head/jaw proxies on the body proxies |
| fall + get-up | 889 | 668: right foot up to **7.0 mm** into the head XL330 and the head shells (the foot swings into the head on the ground), left foot 2.2 mm into the body shell, lower head shell 2.8 mm into the neck servo; the head lying back over the `usb_c_top` space (628 frames) | 22: feet on the head and board-bay proxies |
| obstacles | 1601 | 7: the right shank sweeps through the `xt60_mating_plug` envelope (4.1 mm), so a plug hanging below the keel would be kicked | none |

The walking gaits stay clear. Contact only happens in falls and rolls, where parts really do strike each other. The
simulator lets meshes pass through because only coarse proxies collide, so in a fall the real foot hits the head
shell and the head XL330: those parts need to survive that impact. The roll presses the head into the chest, so the
body shell's chest corner needs about 1.5 mm relief or the head stops on it. Proxy contacts and mesh clashes agree
on where these happen.

### Missing hardware detail (not modelled anywhere yet)

Fasteners and bosses
- The body shell is one closed solid: no split, lid or cover, so it cannot be printed around the boards or opened. It
  needs a parting line (e.g. a removable rear-bay lid and keel cover) with M2 heat-set insert bosses (about 3.5 mm
  diameter, 4–5 mm deep), and a mounting interface to the trunk frame (the upstream shells' screw points are gone).
- UNO Q + HAT: no standoffs. The stack has 2.0 mm behind it (rear wall), 1.3 mm at the sides, 2.0 / 1.5 mm below /
  above. That is less than the through-hole solder tails of a HAT, and there is no room for side bosses; the UNO
  mounting holes (3.2 mm) need standoffs from the bay wall, i.e. at least 4–5 mm more bay depth.
- 2S LiPo: no strap, cradle or clip, and 0.5 mm clearance per side in x (1.0 mm in y) in the keel; real packs vary by
  ±1 mm and swell. Give it 1.5–2 mm per side and a strap slot.
- A010: no bosses or clips in the chest pocket (6.3 mm free behind it, 1.65 mm at the sides).
- Mouth servo: no bosses for the XL330's M2 case holes; it also intersects the M12 lens holder (6.1 mm), the upper
  shell (2.2 mm) and the motor support (1.6 mm), see the static table.
- Jaw: no hinge pin, bushing or bosses, and the servo-to-jaw link is not modelled.
- Head shells: no screws or bosses joining the upper and lower shells.
- XT60: no retention; the pack-side connector floats in the keel.

Connector envelopes (keep-out boxes in `v4_params.json` `envelopes`, checked like parts; ESTIMATES)
- `usb_c_top` (straight USB-C on the UNO Q's short edge, which is the top of the upright stack): 3.2 mm into the shell
  roof. With the board upright there is no room for a plug above it.
- `usb_c_rear` (a rear-facing plug through `usb_access`): clear. The cut only works with a right-angle receptacle or a
  short right-angle adapter; the UNO form factor's connector points up.
- `xt60_mating_plug` (female XT60 + lead pushed on from below): 2.0 mm into the keel floor. The `xt60_access` opening is
  5 × 12 mm; an XT60 housing is 8.2 × 15.8 mm, so the pack cannot be unplugged through it.
- `balance_plug` (2S JST-XH beside the XT60): 2.9 mm into the keel walls; there is nowhere to stow it, and it must be
  reachable for charging.
- `a010_usb` (straight USB-C at the A010's rear face): clear at rest (touches the A010, 0.5 mm).
- `mouth_servo_cable` (JST-EH plug + bend at the servo's end): 5.8 mm into the upper shell, 4.4 mm into the lower.

Cable bends and routes (no envelope yet)
- LiPo leads: 1.0 mm between the pack's bottom and the XT60 placeholder, which is also only 9.5 mm long (a real XT60
  half is about 16 mm). 14 AWG silicone leads need roughly a 10 mm bend radius, so the keel needs 15–20 mm more depth
  or the leads leave the pack sideways.
- Neck pass-through: the head servos' bus, the mouth servo and the head camera's cable must run from the torso through
  the neck with slack for head yaw ±1.2 rad, pitch and roll; not modelled.
- Leg servo bus from the HAT to both hips past the battery keel; A010 USB from the chest pocket to the rear bay
  (about 120 mm past the pack); neither routed.

### Mass: battery and head budget

**2S LiPo (`make_duck.py`).** The upstream torso inertial includes its battery (`np_f970`); before, V4 kept that mass
and only assumed the 2S pack weighed the same. Its contribution is not published, so it is estimated: the upstream neck,
shank and hip links imply an XL330 of 17.9 g and printed parts at 1.08 g/cm³, which leaves 123 g of the 199 g torso
for the battery; fits to the torso COM give 103–125 g. `UPSTREAM_BATTERY_G = 115` (ESTIMATE) is now subtracted at the
battery's pose (uniform density over its mesh) and the LiPo is added as a 55 g uniform box at its V4 location
(`v4_params.json` `battery`, COM (−37.5, 0, 5.0) mm in the torso frame; principal inertia 4.6, 17.5, 20.1 kg·mm²).

| | before | after |
|-|-|-|
| Torso link | 302.6 g, COM (−31.4, 0, 2.7) mm | 242.6 g, COM (−30.6, 0, 6.4) mm |
| Torso inertia Ixx / Iyy / Izz | 232 / 489 / 431 kg·mm² | 304 / 549 / 452 kg·mm² |
| Robot | 0.849 kg, COM height 139.0 mm | 0.789 kg, COM height 141.6 mm |

The inertia rises although the torso is lighter: the 60 mm-tall pack sits higher and further from the torso COM than the
flat upstream battery it replaces. Weigh the real pack and the torso link (`config/robot_measurements.example.json`).

**Head (`head_mass.py`).** Budget: the upstream head link the policies were trained with and the neck servos carry,
188.8 g. The V4 head is 192.7 g (shells 108.6 g + mouth servo 18 g on the upstream remainder) plus the 4.4 g jaw =
**197.1 g, 8.3 g (4.4 %) over**. That figure subtracts the replaced upstream parts at solid PLA density (122.6 g); if the
upstream CAD used the 1.08 g/cm³ its other links imply, 16 g too much was subtracted and the head is about 213 g (13 %
over). The shells are solid meshes about 3 mm thick on average (upper 2.9, lower 3.5 mm), so at 1.2 mm perimeters per
face they are almost all wall: infill barely matters.

| Shells + jaw (g), wall per face × infill | 10 % | 15 % | 20 % | 30 % |
|-|-|-|-|-|
| 0.8 mm | 76.7 | 80.0 | 83.2 | 89.8 |
| 1.0 mm | 91.8 | 94.2 | 96.6 | 101.5 |
| 1.2 mm (model default) | 106.6 | 108.2 | 109.8 | **113.0** |

Proposal (recorded as `print._proposed_head` in `v4_params.json`, not applied): 0.8 mm walls (two 0.4 mm perimeters) and
15 % infill on both head shells and the jaw, −33 g, head about 164 g (180 g in the pessimistic case), under budget with
margin. 1.0 mm / 15 % (−19 g, 178 g) if the dome proves too soft in a drop. Thinning the shell geometry itself to 2.0 mm
(a CAD offset) would save 26 g at the current print settings. A 9 g-class micro servo for the jaw instead of the 18 g
XL330 saves about 9 g more. `print` settings per part are now read from `v4_params.json` by `make_duck.py`.

### Servo delay

`make_duck.py` now writes the servo command delay into every model as `<custom><numeric name="servo_delay_s">`: the
calibration's measured `servo.delay_s`, else an ESTIMATE of 20 ms (the upstream policies are trained with 3–6 physics
steps, 15–30 ms, of actuator lag). `scripts/motions.py` and `world_sim` apply it with `duck_nav/actuation.py`, a pure
transport delay at physics-step resolution (the servos act on the target issued 4 physics steps earlier). It is not
MuJoCo's actuator `delay` attribute because that needs MuJoCo ≥ 3.4 and CI pins 3.3.6; where it exists the response is
identical (`test_actuation.py`). Overrides: `motions.py --servo-delay S`, launch `servo_delay:=S`. The ledger keeps
`servo_delay_s` as the measured value (null until measured; MATLAB reads it) and adds `servo_delay_sim_s`.

**Effect on the motions** (`scripts/motions.py all`, ground-contact model; passes out of 3 runs per row, servo stiffness
×0.95 / 1.0 / 1.05, because single deterministic runs of the marginal motions flip with tiny changes):

| Model, servo delay | balance | walk | obstacles | fall + get-up | sit/stand | roulade | pick | dance | walk speed (m/s) |
|-|-|-|-|-|-|-|-|-|-|
| before: old mass, no delay | 3 | 0 | 0 | 0 | 0 | 3 | 3 | 3 | 0.077–0.095 |
| new mass (LiPo), 0 ms | 3 | 2 | 0 | 2 | 2 | 3 | 3 | 3 | 0.085–0.103 |
| new mass, 10 ms | 3 | 0 | 0 | 2 | 3 | 3 | 3 | 3 | 0.045–0.082 |
| **new mass, 20 ms (default)** | 3 | 0 | 2 | 1 | 3 | 3 | 3 | 3 | 0.063–0.090 |
| new mass, 30 ms | 3 | 0 | 2 | 2 | 3 | 2 | 3 | 3 | 0.036–0.098 |
| new mass, 40 ms | 3 | 1 | 2 | 1 | 3 | 3 | 3 | 3 | 0.100–0.109 |

No delay up to 40 ms breaks standing, sit/stand, pick or dance. The lighter LiPo makes sit/stand and the fall recovery
work where the old mass failed (0/3 → 2–3/3). Walk (0.8 m in 8 s at a 0.3 m/s command), the plank course and the
fall recovery are marginal in every configuration and change between neighbouring stiffness values as much as between
delays, so these rows show robustness, not a ranking. The walk criterion failed before this change too.

### Realistic sensor profile (`world_sim`, `duck_nav/sensor_errors.py`)

New parameters (defaults are ESTIMATES for a BMI088/ICM-42688-class IMU and XL330 servos; `sensor_errors:=false`
switches them all off, `sensors:=ideal` never has them):

| Error | Parameters (default) | Where it shows |
|-|-|-|
| IMU noise | `imu_gyro_noise` 0.002 rad/s, `imu_accel_noise` 0.02 m/s² per 50 Hz sample | `/imu` |
| IMU bias | `imu_gyro_bias` 0.005 rad/s and `imu_accel_bias` 0.1 m/s² (turn-on, σ per axis), random walks `imu_gyro_bias_walk` 1e-5 rad/s/√s, `imu_accel_bias_walk` 1e-4 m/s²/√s | `/imu`; the accelerometer bias tilts the attitude estimate by bias / g (0.1 m/s² ≈ 0.6°) |
| IMU misalignment | `imu_misalign_deg` 0.5 (σ per axis) | `/imu` axes; roll/pitch part tilts the attitude estimate |
| Attitude noise | `attitude_noise_deg` 0.1 | the tilt used for the sensor poses and the `base_level` TF |
| Encoders | `enc_offset_deg` 0.5 (zero offset σ), `enc_noise_deg` 0.05, `enc_counts_per_rev` 4096, `enc_vel_noise` 0.05 rad/s | `/joint_states`; the head camera pose (forward kinematics with the measured angles) |
| Extrinsics | `tof_extrinsic_deg` 1.0 / `tof_extrinsic_mm` 2.0, `cam_extrinsic_deg` 1.0 / `cam_extrinsic_mm` 2.0 (σ per axis) | the simulated A010 and camera look from a mount perturbed once per run; `/tof/pose_level` and `/camera/pose_level` report the CAD mount |
| Timing | `imu_stamp_jitter` 0.5 ms, `cam_stamp_jitter` 3 ms, `tof_stamp_jitter` 2 ms (σ, stamps kept increasing), `tof_latency_jitter` 10 ms (uniform extra delay, frames stay in order) | message stamps |

Published sensor poses are now computed the way the robot would: nominal mounts, measured joint angles, true heading
and the IMU-derived tilt (`level_pose`, which reproduces the old world-frame formula exactly when the errors are zero,
`test_sensor_errors.py`). The walking policy still sees exact gyro, gravity and joints unless
`policy_sensor_errors:=true`. Start-up log line: the drawn biases, offsets and extrinsic errors.

`score_node` now pairs each SLAM estimate with the nearest ground-truth sample within `gt_match_tol` (10 ms) instead
of an identical stamp: with jittered ToF stamps the first smoke run scored 0 samples.

**Closed-loop smoke runs** (300 s, seed 0, realistic profile, 20 ms servo delay; these check that it runs, they do not
compare):

| Run | sensor errors | sim / wall | SLAM mean / max error | walked | falls |
|-|-|-|-|-|-|
| `mech_noerr_s0` | off | 300 / 300 s | 0.16 / 0.36 m | 6.5 m | 0 |
| `mech_err2_s0` | on (defaults) | 300 / 357 s (**not real time**: CPU shared with fit-check jobs) | 0.32 / 0.64 m | 9.3 m | 0 |

Both start, survey, explore and stay localised, with no falls; neither reaches the beacon in 300 s (the earlier
600 s trials needed 153–495 s). The run with errors fell behind real time, and one seed is within run-to-run noise, so
how much the errors cost SLAM is still open. That needs paired 600 s trials on seeds 0–5 (`sensor_errors:=true` vs
`false`) held in real time. `gyro_calib_s` (off by default in the launch file) would remove most of the 0.005 rad/s
gyro turn-on bias.

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
- `check_v4_fit.py`: the fit check (writes `fit_check.json`): static fit, pose envelope and recorded motions.
  `python design/v4/check_v4_fit.py --samples 3000 --traj "DIR/*.npz"` after
  `python scripts/motions.py all --record DIR`. `solid.py`: the containment / depth geometry it uses.
- `head_mass.py`: head mass against its budget and the print-setting options.
- `v4_params.json` also holds the keep-out `envelopes` (connectors, cable bends) and the `print` settings.
- `preview.py`: `preview/duck_v4.glb`, `.blend` and renders (`preview/` is git-ignored: upstream meshes).
- `sim_runs/`: `summary.txt`, `result.json`, `map.png`, `launch.log` per run (`v4sbase_*`, `v4snew_*`, `v4syaw_*`: the
  paired trials above).

The simulator launch defaults are this V4 configuration (`ros2 launch duck_nav duck_nav.launch.py sensors:=realistic`);
`tof_zones:=100` for the full A010 array. The old head-mounted VL53L5CX is still available through launch arguments.
