# Post-merge review and implementation handoff

> **Simulation check after delivery (19 Nov 2026):** [SIM_VALIDATION.md](SIM_VALIDATION.md). Default path: no measurable change (inside the large run-to-run spread). `loop_closure:=true` made every live and replayed trial worse and never accepted a closure: keep it off.

18 November 2026 (America/Los_Angeles; work began 17 November). Reviewed merge **d08af6c0b4de31e31b8721592c1bd4317a017c6d** and its tracked source, tests, summaries and logs. Three GPT-6.1 Sol workers handled localisation, trial/replay tooling, and failure/hardware analysis; the orchestrator reviewed their code and requested corrections. This update is software work and offline validation, not a successful robot deployment or a new closed-loop accuracy result.

## What changed

| Requested item | Delivered / finding | Still required |
|---|---|---|
| 1. Loop closure / pose graph | Opt-in bounded SE(2) keyframe graph, conservative repeated-revisit checks, robust anchored optimization, retained-scan map rebuild, rigid correction propagation and invalidation of stale navigation/beacon caches. Default remains off. | Recorded-bag and ROS closed-loop evaluation. This nearby-revisit prototype does not solve global relocation or demonstrate recovery from 2.4 m drift. |
| 2. More seeds / rooms | Repository trial-matrix runner with isolated ROS domains, explicit layout overrides and paired rate summaries. | Execute at least ten seeds per layout/configuration with complete evidence; no new trials were run here. |
| 3. Seed 2 error and fall | Identified sustained recovery gating after the fall, plus evidence of incorrect target coordinates beforehand. Replay now follows the live fall gate and shared localisation defaults. | Actual bags/tilt/contact traces to quantify the error trajectory and determine the fall cause. |
| 4. Replay set and 600 s trim | Bag inventory, clock-based non-destructive trimming and replay-set tools; missing historical recordings are explicit. | Supply seed 0/1/2 bags, produce the seed-0 600 s copy, run baseline and candidate replays. No bag has been recovered, added or trimmed in this environment. |
| 5. External trial helpers | In-repo replacement runner; existing `DUCK_ORT_THREADS` support already makes the external ORT source patch unnecessary. Fixed `setup_nav.sh` using `$P` before assigning it, quoted paths and created missing workspace directory. | The original `C:\dev\duck_review_zip\job2.sh` and `patch_ort.py` were not accessible, so these are replacements, not claimed copies. |
| 6. 2x versus real time | Paired seed/layout/rate execution and comparison support. | Run the matrix. 2x equivalence is unproven, and unpaced results remain unsuitable as a baseline. |
| 7. Shield clearance | Reran the strict 6000-pose check and PCB/shield proxy study. Current assembly fails with 12 contact pairs. | Measured CAD/PCB/bracket revision and dynamic clearance checks. |
| 8. Measurements | Reviewed calibration interfaces and reran model tests; retained provisional values. | Servo bench measurements, printed/installed masses and actual servo count. |
| 9. Live ToF and hardware TF | Reviewed existing acquisition/TF contracts and documented remaining timing/geometry dependencies. | Actual VL53L5CX adapter, MCU/ROS clock mapping, encoder/IMU source and measured extrinsics. No fabricated calibration or receipt-time substitute was introduced. |
| 10. Walking policy | Confirmed policy/model mismatch is independent of shield clearance. | Fit the Feetech actuator model and measured masses, then retrain/adapt and validate a policy. |

## Seed 2: why local relocalisation did not rescue the run

The relevant folder is `results/post_fix/ros2_nav_realistic_new_s2`: mean error 0.868 m, maximum 1.855 m, one fall. A beacon estimate around (5.05, 1.48) was rejected as outside the map at 302.5 s. The true-pose fall occurred at 327.2 s; `slam recovering` started at 327.5 s and no successful upright transition was logged before 600 s.

While recovering, `slam_node` deliberately skips scan matching and mapping. Local relocalisation cannot run until filtered tilt has stayed below 15 degrees for more than one second. It is not a get-up controller. The retained logs cannot establish which contact caused that fall or whether localisation or ranging dominated the earlier beacon error. See [the detailed evidence review](seed2_hardware_review.md).

## Loop closure limits and integration review

`loop_closure:=true` enables the experimental path. It uses local scan clouds and frontend relative-pose constraints, not ground-truth poses. Closure requires reciprocal geometric support, two spatially separated consistent revisit observations, and an accepted graph solve. Repeated room geometry can still alias; these are heuristic gates, not proof of place identity. The search only handles nearby revisits with relatively small correction. It cannot be described as global recovery.

The graph retains at most 160 keyframes by default. At capacity, graph optimization/rebuild is disabled and ordinary mapping continues; previously integrated evidence is retained. Increase the bound only after measuring performance. Graph-enabled mapping uses coarser keyframe spacing than the default frontend, so an on/off trial measures the combined experimental system, not just an isolated optimizer.

Review specifically addressed stationary bootstrap head sweeps, behavior at graph capacity, independent confirmation, and old goals/paths surviving a map revision. Rotational corrections now carry delayed poses as rigid SE(2) transforms rather than adding translation and heading independently. This also changes default frontend correction propagation; baseline measurements must be repeated rather than assuming bit-for-bit equivalence with the merge.

Map revisions clear detection/goal caches, stop navigation, invalidate in-flight plans and require fresh corrected observations. Offline node shims are not a substitute for ROS transport/executor validation. Keep loop closure disabled for hardware use until that integration is exercised.

## PCB/shield candidate retained for the next design pass

**Target board outline: 70 mm x 53.34 mm**, with the modeled shield center at **(-70, 0, 0) mm in the torso frame**. This is the existing candidate reproduced by the current checks, not an order-ready PCB specification.

The current 110 mm proxy at (-50, 0, 20) mm has shield contacts in 1132/6000 poses. Shortening it to 70 mm at the same center still has 332/6000. Shortening **and moving** it gives zero sampled shield contacts, including 3 mm proxy inflation. It does not certify full-range motion, real connectors/stack height, wiring, bracket mounting, mass or get-up clearance. Actual CAD, shield STL and PCB routing were not changed. Do not remove collision masks to obtain a passing result.

## Reproduce and continue

Final validation: **135 passed, zero skipped**, including model integration with pinned assets, real synthetic ICP/confirmation/graph optimization, map rebuild and capacity behavior, navigation cache invalidation, replay fall gating, trial-tool checks and scoring completion. Python compilation, shell syntax and `git diff --check` pass. These checks do not establish closed-loop navigation accuracy. See [trial and replay commands](../../ros2/duck_nav/tools/TRIALS.md).

Apply the supplied patch to a clean checkout at `d08af6c` on a new branch:

```bash
git switch -c post-merge-localisation-review d08af6c
git apply --check /path/to/duck_post_merge.patch
git apply /path/to/duck_post_merge.patch
python -m pytest tests ros2/duck_nav/test -q
```

The archive also contains the changed files under `changed_files/`, preserving repository-relative paths. The patch is the preferred application route; do not overwrite unrelated local edits.

For model integration tests, use the same upstream asset revision as CI: `microduck_rl` at `0daab160d3fb1c2cbaba16a4a19052f36709dd9e`. Keep generated meshes and pretrained model dependencies available when preparing the ROS workspace. Read the trial/replay tooling documentation beside the new tools before executing a matrix.

This environment has no ROS 2 installation or robot connection. The GitHub connector is unavailable and the CLI is not authenticated; changes were prepared locally and packaged, not pushed or merged. Historical result folders were preserved. See `validation.txt` for the final executed checks; `contact_check.txt` intentionally records a failing hardware gate.
