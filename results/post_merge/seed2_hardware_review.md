# Seed 2 failure and hardware dependency review

Reviewed baseline `d08af6c0b4de31e31b8721592c1bd4317a017c6d`, 17 November 2026 (America/Los_Angeles). Simulation and offline checks only. No hardware calibration, firmware changes, or motor commands were performed.

## What the retained Seed 2 evidence establishes

The 0.87 m mean-error / one-fall result is `results/post_fix/ros2_nav_realistic_new_s2`, the 600 s correlative-matcher trial. It is **not** `ros2_nav_realistic_v3model_seed2` (a separate 300 s trial with 1.244 m mean error and zero falls).

| Observation | Evidence | Implication |
|---|---|---|
| Mean / max / final position error 0.868 / 1.855 / 0.926 m; 22.1 m walked; 77% room coverage | `new_s2/summary.txt` and `result.json` | Exploration improved over some baselines, but accuracy and completion did not meet a hardware gate. |
| Beacon estimate around (5.05, 1.48) rejected as outside map at 302.5 s; repeated through 324 s | `new_s2/summary.txt` | Wrong map-frame target coordinates were already present before the fall. A fall-only explanation for all drift is unsupported. This log cannot separate localization error from beacon ranging error. |
| True-pose fall at 327.2 s; `slam recovering` at 327.5 s | `new_s2/summary.txt` | Recovery gating was entered after the detected fall. |
| Filtered-tilt warning at 36 degrees; no `upright again` line and no later navigation-state event before timeout | `new_s2/launch.log` | The observed recovery state persisted for the remaining approximately 272.5 s. No successful upright transition is recorded. |

`slam_node.on_frame` returns while its internal state is not `ok`, before calling `FrameLocalizer.step`. `on_imu` suspends gait-command translation while recovering but continues heading integration. Recovery requires filtered tilt below 15 degrees continuously for more than 1 s; entry threshold is 35 degrees. `reset_after_fall` invalidates match health and clears the sweep. Therefore local relocalization is not a get-up controller: its ±0.8 m / ±25 degree search after 3 s unhealthy cannot run until the robot has passed the upright gate. Enlarging that search cannot resolve this retained trial's persistent recovering state.

Even upright, the search is relative to the current estimate and the existing map; it has no global pose graph or mechanism to repair an already distorted map. The 1.855 m maximum error alone does not prove the true correction lay outside the search at any particular attempt (axis-aligned search bounds, map drift, and timing matter). The retained files contain no per-frame match diagnostics or bag files, so there is no defensible attempt-by-attempt replay, fall contact attribution, or pre/post-fall error curve here. The branch report says bags existed elsewhere; they are not in this checkout. These findings were sent to the localization owner for the core fix and recovery-duration diagnostics.

## Mechanical checks rerun

Artifacts: `results/post_merge/contact_check.txt`, `results/post_merge/pcb_target_check.json`.

Committed model XML was copied into `/tmp/duck-postmerge-hardware`; missing ignored meshes were supplied from the CI-pinned upstream `microduck_rl` commit `0daab160d3fb1c2cbaba16a4a19052f36709dd9e` and generated V3 design meshes from the integration-test fixture. Tracked model/CAD files were not regenerated. MuJoCo 3.3.6 was used. The contact artifact records the compiled-model hash, scene hash, random seed 1, and 6000 poses per envelope. Initial direct checks failed because the checkout had no `model/assets`; reruns below use the explicit temporary assembly.

| Check | Rerun result | Meaning |
|---|---|---|
| Strict contact gate, current ground-contact model | Expected exit 1: zero STAND self-contact; 12 distinct operating-envelope contact pairs | Current guessed installation remains mechanically blocked. |
| Head lower shell versus shield back | 868/6000 sampled poses (14.5%) | Head sweep cannot be treated as clear. |
| Left/right shank versus corresponding shield side | 7/6000 each | Leg/shield clashes persist; contacts remain enabled. |
| Current shield/board proxy (110 mm board length, center [-50, 0, 20] mm) | 1132/6000 poses have shield contacts | Baseline mounting guess is not acceptable. |
| Shorten board proxy to 70 mm at same center | 332/6000 contact poses | Shortening alone does not solve clearance. |
| 70 mm board proxy, center [-70, 0, 0] mm | Zero shield contacts in these samples, including 3 mm inflation | Candidate for measured CAD review only. It is not an installed-fit, full-range or dynamic certificate. |

The shortened/moved study changes only shield collision boxes. It does not change actual shield STL, PCB routing, mounting bracket, mass, center of mass, wiring, battery, or UNO/HAT stack. Other body clashes still exist. The current shield geometry is retained; collision suppression is not a fix.

Historical dynamic A/B evidence is stronger than a random-pose check for get-up: `results/post_fix/motions/summary.txt` fails both forward/backward get-up within 12 s, while `motions_ab_noshieldproxy/summary.txt` passes recovery in 1.2/0.8 s. Obstacle traversal also changes from fail to pass when shield proxies are disabled. Walking remains failed in both, at 0.59 m forward travel, 0.63 m lateral drift, 107 degrees heading change. This supports two separate concerns: shield interference affects recovery, and gait/model mismatch persists independently. Those A/B runs are historical, not a new dynamic reproduction of the Seed 2 fall. No retained contact trace establishes that the shield caused the exact 327.2 s fall.

## Hardware sensor/interface gaps

`ros2/duck_nav/HARDWARE_SENSORS.md`, `tof_node.py`, `sensor_pose_node.py`, and `launch/hardware_sensors.launch.py` define a perception-only boundary. They do not complete the physical driver or hardware motion owner.

- A live ToF adapter must provide 64 metre ranges with normalized statuses, measured row/column orientation and radial/axial convention, and a positive acquisition stamp mapped to the ROS clock. It must keep firmware/I2C blocking work out of polling and close its worker reliably. The existing backend choice is explicit; no unverified live backend is selected automatically.
- The retained vendor integration finding identifies a read-only `DTF1` snapshot as a starting point, but records MCU read-completion time rather than bounded sensor acquisition time. MCU/ROS synchronization, acquisition offset and reset/wrap behavior remain unimplemented and unmeasured. Its UI grid reverses columns relative to raw arrays. Do not substitute receipt time, raw vendor status values, or visual grid order for the package contract.
- Hardware encoder joint states, IMU tilt/height TF, camera optical extrinsics, ToF body extrinsics and synchronized clocks must be supplied. `sensor_pose_node` requests each acquisition's exact timestamp and drops missing transforms after its timeout; it does not synthesize a latest-pose calibration. These sources are absent from the bring-up launch.
- ToF `ok` reports a recent well-formed frame, including a frame whose zones are all invalid. It does not mean useful localization geometry or safe movement. The robot motion owner must enforce stale/weak/recovering stopping independently, including if the host process dies.
- The simulated 12 mm ToF optical-origin offset is a modeled clearance workaround, not a measurement of the purchased module/bracket. Real optical opening, zone coverage, timing during head motion and noise still require bench tests.

No hardware driver patch is justified without those measurements. The existing explicit failure on unconfigured backend/extrinsics should remain.

## Measurement and policy dependencies

`config/robot_measurements.example.json` intentionally contains nulls. Current head print masses (51.5/43.11 g), shield (32.3 g), shield center (-50, 0, 20 mm), electronics mass distribution and actuator behavior are provisional. Measure the installed assembly, supports removed, before replacing them. Complete-link `jaw_soft` includes shells; `trunk_base` excludes the separately added shield; avoid counting old electronics and replacements together. Weight rescaling alone does not measure center of mass/inertia, and `servo.delay_s` is recorded rather than automatically simulated.

`policies/manifest.json` identifies Microduck/XL330 policies with a 61-observation, 14-action, 50 Hz interface. The hardware guide identifies the custom robot's Feetech actuation and treats the official XGO runtime/policy only as an adaptation starting point. Neither the imported policies nor that alternative are validated for these V3 parts. Required evidence includes actual servo variant/firmware, IDs, zero/sign/limits, observation/action ordering, IMU axes, stiffness/damping/torque/delay, measured command odometry and watchdog timing. A standing pass or a software test pass cannot establish these.

## Verification and remaining work

Executed `python -m pytest tests/test_calibration.py tests/test_model_calibration.py ros2/duck_nav/test/test_sensor_contract.py -q`: **43 passed**, including actual model integration with pinned upstream assets. These exercise measurement semantics, mass/inertia regeneration, stale ledger rejection, radial/axial rays, invalid-zone handling, clock freshness, replay, and camera convention conversion. They do not execute ROS nodes, physical TF, I2C, MCU timing or policies on the robot. ROS 2 is not installed in this environment; no new closed-loop ROS trial or hardware test was run.

Next physical dependencies are: measure mount and installed masses; revise and assemble the shield/bracket/PCB candidate; check clearance through the commanded sweep and recovery trajectory; calibrate actuator/sensor/TF timing; validate an appropriate policy and hardware stop owner; then repeat measured-pen and fall/relocalization trials with acquisition data, tilt, match diagnostics and contacts recorded. Software localization improvements and better trial tooling can proceed independently, but persistent physical recovery is still a separate open item.
