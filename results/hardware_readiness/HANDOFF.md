# Duck hardware-readiness software handoff

17 November 2026. Based on public repository commit `99c407d`, with one primary
agent and two GPT-6.1 Sol agents. Work is on branch `codex/duck-hardware-readiness`.
The numerical/model toolchain uses Python 3.12 and `requirements-test.txt`;
upstream model assets were checked out at
`pollen-robotics/microduck_rl@0daab160d3fb1c2cbaba16a4a19052f36709dd9e`.

> **Localisation update (18 Nov 2026, branch `depth-camera-opt`).** In closed-loop room trials the patch's
> strict localisation rules (2-D geometry, overlap, current-frame agreement and search-boundary checks on a
> ±7.5 cm grid search) rejected almost every match and stopped exploration (30% of the room covered, 6.4 m
> walked). They were replaced by a correlative scan matcher (Olson 2009) with online slip estimation,
> relocalisation and a bootstrap fix; the per-frame policy is `duck_nav/duckslam.py: FrameLocalizer`, shared
> by `slam_node` and the offline replay tool. The navigation gate below (drive only while `/slam/state` is
> `ok`, two accepted constraints after a failure) is unchanged. Rationale, measurements and closed-loop
> results: [`results/slam_tuning/README.md`](../slam_tuning/README.md). Rows and paragraphs below that
> describe the old rules are marked.

## Can this run the actual duck?

**Not as a complete robot controller.** This update makes measured model inputs,
sensor integration and localization stopping testable. It does not supply a
hardware-tested servo/Bridge controller, a Feetech-trained walking policy, real
sensor calibration or verified shield mounting geometry.

The black PLA design, integral rounded bill, provisional print masses and actual
mounting geometry were not changed. No measurements or hardware test results
were invented. Existing historical reports are preserved.

| Handoff issue | What changed | What is still required |
|---|---|---|
| Printed/installed mass and servo numbers | Validated JSON input overrides; all 15 model bodies can receive measured mass; exact input hash recorded | Print and weigh; include installed UNO Q, HAT, Feetech servos, battery/BMS, sensors, wiring and fasteners in the correct rigid links |
| Complete head measurement | Shell addition is skipped and shell metadata retained; regression test verifies no double counting | A complete installed head weighing; COM and inertia still need CAD/measurement |
| Tagged model bookkeeping | Correct ledger selected by scene tag; robot XML hash rejects stale ledgers; optional 5% whole-robot mass gate | Supply actual measurements, then regenerate and rerun |
| Servo delay | Recorded and passed to the MATLAB delay check | Delay is **not** automatically inserted into the actuator dynamics; configure/validate the delay experiments separately |
| Rear shield | Contact checker now fails on operating-envelope clashes; counts distinct poses, records seed and compiled-model hash | Measure mount, change placement/CAD and verify full mechanical clearance and get-up |
| Sparse ToF localization | *Patch:* bounded 1.5 s sweep constraints, 2-D geometry/overlap/current-frame agreement and search-boundary checks. *Now:* correlative matcher over a 1.0 s sweep (±30 cm / ±12°, odometry prior, score-based covariance), online slip scale, local relocalisation (±0.8 m / ±25° after 3 s unhealthy); the agreement check is off by default (`fresh_check`) | Measured noise/extrinsics, better odometry, loop closure; relocalisation is local, not global |
| Weak/stale localization | Forward drive is stopped; head keeps sweeping; recovery needs two fresh accepted constraints | ROS transport/executor and real command-owner/MCU watchdog tests |
| Walls/corner cutting | Commanded forward arc checked against footprint clearance and known-free cells; escape cannot reduce clearance; target selection no longer chases an old path start | Measure the speed bound and turning footprint; tight corners may intentionally stall |
| ToF hardware software | `duck_tof` acquisition wrapper, explicit backend contract, offline JSONL replay and invalid-zone rejection | Actual VL53L5CX bus/firmware adapter, status normalization, clock synchronization and zone-direction tests |
| Head/sensor pose software | Exact-acquisition TF lookup, bounded waiting, camera optical-frame conversion | Encoder/IMU TF source, measured sensor extrinsics and correct body height; this node does not read encoders or the IMU itself |
| USB camera | Reject image/CameraInfo dimension mismatch; explicit latency-duration import | On-board streaming, intrinsics, distortion handling, exposure and latency calibration |
| Walking and balance | Rechecked original policy and a short standing smoke test; no actuation adapter added | Real servo test; gains recomputed; compatible gait and hardware controller |
| YOLO | Shipped ONNX loads during preflight | Real-camera labelled dataset, retraining and board timing |
| Repeatability | Direct dependency versions, pinned upstream CI asset checkout and numerical/model test workflow | CI must run after upload; ROS/MATLAB/hardware validation is a separate stage |

## Enter measurements without editing Python

### PCB resizing target and shield placement

**Layout target: 70.0 mm tall × 53.34 mm wide**, with the 70 mm dimension
vertical on the duck. This shortens the present 110 × 53.34 mm board by 40 mm.
The public HAT Gerber outline at commit
`b9003682f1af219fd9451dd8f47010c787b0682c` confirms the original dimensions.
Keep the exact UNO Q mating-header geometry and mounting datums; relocate parts
and reroute into the new outline rather than cutting 40 mm off the old board.
The [Arduino mechanical drawing](https://docs.arduino.cc/resources/datasheets/ABX00162-datasheet.pdf)
gives an approximately 69 × 53.34 mm UNO Q outline, making 70 mm a useful compact
carrier target. Component placement/routing within this area has not been checked.

**Board size alone does not fix the clash.** Use this provisional combined
board/shield/bracket target for the next CAD revision:

| Item | Target |
|---|---|
| HAT PCB outline | **70.0 mm vertical × 53.34 mm across** |
| Matching shield outer envelope | **76.6 mm tall × 59.94 mm wide × 18.5 mm deep** |
| Shield clear width / back-face clearance | 54.94 mm / 16 mm, retaining the existing 2.5 mm walls |
| Shield geometric centre in torso coordinates | **`[-0.070, 0.000, 0.000]` m**, x forward, y left, z up |
| Change from the old guessed shield centre | **20 mm farther back and 20 mm lower** |
| Mechanical clearance design allowance | At least **3 mm** to moving parts; confirm with the complete installed assembly |

`scripts/check_pcb_target.py` compares the old shield, shortening alone and the
combined target over 6,000 operating-envelope poses (seed 1). The combined
70 mm target at the new centre has **zero sampled shield contacts**, including
a conservative check with every shield collision box expanded by 3 mm on each
side. Shortening alone at the original mount still produces contacts. Full
results and model hashes are in `pcb_target_check.json`.

This is a **model-derived layout target**, not a verified clash-free production
board. Only shield proxies were changed for this study; the actual shield STL,
PCB routing and default model remain unchanged. The UNO Q/HAT stack, standoffs,
fuse, XT60, servo plugs, cable bends and fastening access must be included in the
new CAD assembly. The 16 mm shield cavity is not proof that the whole UNO Q/HAT
stack fits. Do not shrink copper/protection spacing simply to meet the outline;
use a separate power/connector board if necessary. Preserve mating-header
alignment when relocating the electronics bracket. Recheck all robot contacts,
actual walking trajectories, full joint travel and get-up; this study does not
clear unrelated head-to-torso contact. Moving electronics rearward also changes
balance, so update mass/COM and repeat standing/backward-push tests before use.

Copy `config/robot_measurements.example.json` to a working measurement file.
Keep unknown values `null`; the supplied example contains **no measurements**.
Numbers have explicit units: complete links in kilograms, print masses in grams,
positions in metres, servo `kp` in N·m/rad, `kv` in N·m·s/rad, stall in N·m,
delay in seconds. Finite positive values are required, except damping/delay may
be zero. Unknown fields and wrong link names fail before regeneration.

Count a component exactly once on the body to which it is rigidly attached:

- `jaw_soft` includes the two installed V3 shells and all attached components.
  Its complete measured mass replaces the whole head; shell values are then
  informational and are not added a second time.
- `trunk_base` includes its installed electronics, battery and fasteners but
  excludes the V3 shield, which the generator adds separately.
- Link mass replacement rescales the existing inertia at its existing COM.
  A scale reading does **not** measure COM or inertia. The ledger stays provisional.
- `whole_robot_mass_kg` is a separate final assembled weighing. It is a comparison,
  never an added mass. Agreement alone does not certify correct mass distribution.

With the upstream assets prepared as in README Part A:

```bash
python -m pip install -r requirements-test.txt
python scripts/prep_v3_meshes.py
python scripts/make_duck_v3.py --calibration config/my_robot.json
python scripts/make_duck_v3.py --base robot_groundcontact.xml --tag gc --calibration config/my_robot.json
python scripts/stand_numbers.py model/duck_v3_scene.xml --strict-mass
python scripts/check_contacts.py --scene model/duck_v3_gc_scene.xml
```

`--strict-mass` fails when the final weighing is missing or differs by more than
5%. The contact command currently **fails** on the guessed shield mount.
`--report-only` exists for diagnosis; it is not a successful clearance gate.
Random sampling does not certify the whole joint range or get-up trajectory.
Use `--output` on the contact/standing-number tools to preserve separate runs.

After measurements, rerun the gain/delay/standing/motion tests, then regenerate
the URDF and ROS assets. The new measurements do not silently change the checked-in
model or the running robot.

For the servo bench test, log timestamps, commanded/measured joint angle, supply
voltage, load setup and available current/torque readings. A no-load position
step alone cannot determine absolute torque stiffness or stall torque. Keep
unknown quantities unknown until the test can identify them. Recompute gains
only after the model and measured actuator assumptions have been reconciled.

## Bring up sensing separately

See `ros2/duck_nav/HARDWARE_SENSORS.md` for the adapter schema, exact-stamp TF
requirements and parameter file structure. After supplying those inputs:

```bash
ros2 launch duck_nav hardware_sensors.launch.py sensor_config:=/absolute/path/sensors.yaml
```

This launches only USB camera, ToF acquisition and sensor pose publication.
It starts no navigation, simulated world, walking policy or servo commands.
The replay adapter is explicitly offline and is not a real VL53L5CX driver.
The official XGO runtime already contains a real sensor implementation, but its
exported timestamp is read completion without a synchronized ROS acquisition
clock. `HARDWARE_SENSORS.md` pins the inspected source commit and records the
exact ABI/timing changes needed before connecting it to this SLAM interface.

SLAM publishes `initializing`, `weak`, `stale`, `recovering` or `ok` on
`/slam/state`, and the match reason/overlap on `/slam/match`. `ok` means two
recent matches were accepted; it is not a position-error bound. A featureless
wall constrains only the across-wall position; a weak view can leave the duck
standing still. Frames with no ToF return are not counted as failed matches.
`scan_matching:=false` is diagnostic-only and cannot release autonomous motion;
`matcher:=grid` selects the old ±7.5 cm search. After 3 s without healthy
localisation (for example after a fall) the matcher widens its window to
±0.8 m / ±25°; this is local relocalisation, not global, so a larger error
than that is not recovered.

`speed_bound_mps` is the conservative SI forward-speed bound used by the local
arc guard. Calibrate it with the real gait; a gait command value is not a speed
measurement. The guard is sampled, uses the map footprint and does not model
braking, noncircular body motion or floor slip.

## Verification and limits

**94 tests passed, with no skips** in this environment when the patch was
delivered; the captured aggregate result is in `tests.txt`. On `depth-camera-opt`
106 pass, and the GitHub workflow runs them on every push. Both Sol agents reviewed adjacent work; fixes from
review include sensor-outage recovery streaks, stale ledger attribution and
contact-count/provenance reporting.

```bash
python -m pytest tests ros2/duck_nav/test -q
python -m compileall -q scripts tests ros2/duck_nav/duck_nav ros2/duck_nav/launch
```

Tests cover invalid measurement inputs, true model regeneration with complete
head/torso masses, unchanged other bodies, actuator overrides, ledger
provenance, sparse matching and stale/weak navigation stopping. Node safety
tests use message/ROS shims and execute actual node methods; constructors,
executors, DDS and devices are not exercised.

*(Patch-era check, kept for the record.)* The stationary synthetic corner replay uses eight horizontal rays, a ±0.9 rad
head sweep with a 4 s period, 10 Hz acquisitions, 1 cm Gaussian noise and 10%
independent dropouts. Seeds 0/7/42 accept 47/46/45 of 100 scans; the motion gate
is `ok` for 43/43/41 acquisitions. This verifies that bootstrap can succeed
while weak views stay gated. It does not include body dynamics, measured sensor
noise, zone cones or drifting hardware odometry, and is not a real-room score.

Additional commands and captured results:

| Check | Result |
|---|---|
| `python scripts/preflight.py` | 0 errors, 1 expected provisional-mass warning; all five shipped scene/world models compile, all ten policies load and recorded checksums match, YOLO loads |
| `python scripts/check_contacts.py --scene model/duck_v3_gc_scene.xml --samples 6000` | Expected FAIL: zero STAND self-contacts, **12 operating-envelope clash pairs**; head/shield and leg/shield contacts persist |
| `python scripts/motions.py walk --out results/hardware_readiness/motions` | FAIL: 0.74 m travel vs 0.8 m minimum, 0.50 m lateral drift vs 0.30 m maximum, 84° heading change vs 45° limit; no fall |
| `python scripts/push_test.py model/duck_v3_scene.xml 0 --T 10` | Short simulated stand passes using Python defaults 2.5/0.20, zero extra command delay; mean final-second pitch −0.16°, max absolute pitch 1.69° |

The walking result differs numerically from the archived 107° result under the
older environment. No walking change was made and no improvement is claimed.
The short stand is not the 60 s hardware gate, not a new backward-push margin,
and not validation of measured Feetech dynamics. Full ROS room trials,
Simulink/MATLAB, UNO Q execution, physical sensors and real servos were not run
in this environment. Existing ideal/realistic room accuracy numbers are historical; full ROS room trials
of the current localisation are in `results/slam_tuning/README.md` (simulation only).

## Next physical work

The official [XGO UNO Q runtime](https://github.com/LuwuDynamics/xgoduck_runtime_arduino)
and [training project](https://github.com/LuwuDynamics/xgoduck_rl) are concrete
adaptation starting points. Their [policy mapping](https://github.com/LuwuDynamics/xgoduck_runtime_arduino/blob/master/python/rl_core.py)
uses 14 body outputs plus a separate mouth servo (ID 34), explaining the stock
15-servo count. Check whether the integral-bill build retains that actuator.
The [official interface documentation](https://wiki.xgorobot.com/kb/the-xgo-series-robot/duck-architecture?model=duck)
specifies a 2 Mbps Bridge configuration, a 1 Mbps servo bus and separate sensor
I²C buses. The old guide's 112 kbps assumption and 27 kbps payload arithmetic
were corrected: its own 48 two-byte values at 50 Hz total **38.4 kbps** before
protocol overhead. These rates do not validate timing or pin assignments on
the custom HAT. No XGO policy was integrated or represented as compatible with
the modified mass/geometry model.

1. Print/weigh parts, identify every installed component and count the servos.
2. Bench-test the actual Feetech servo and measure delay; update the model.
3. Measure the shield mount, clear both legs and the entire head sweep, and rerun
   strict contact and motion checks with collisions enabled.
4. Bring up standing on a support with the measured controller, then a compatible
   hardware gait. A policy for a different actuator/robot is not a drop-in.
5. Bring up camera/ToF acquisition and measured TF, test timing during head motion,
   then repeat the measured-pen mapping gate before autonomous navigation.

## Applying this delivery

The public repository was cloned successfully. GitHub writing is unavailable in
this session (no authenticated git credentials), so these changes were not
pushed and no pull request was created. The delivered ZIP is the updated tracked
repository without downloaded upstream assets/caches; the patch applies to
`99c407d`. On a clean clone of that base:

```bash
git switch -c duck-hardware-readiness
git apply --check /path/to/duck_hardware_readiness.patch
git apply /path/to/duck_hardware_readiness.patch
python -m pip install -r requirements-test.txt
python -m pytest tests ros2/duck_nav/test -q
```

Without upstream assets the four model integration tests skip explicitly;
clone the pinned upstream commit to exercise them too. The GitHub workflow does
that automatically when uploaded. *(Since then it runs on GitHub; its pip cache needed
`cache-dependency-path: requirements-test.txt`.)*
