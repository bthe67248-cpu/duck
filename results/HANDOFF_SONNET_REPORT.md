# Handoff report: Sonnet (scripts / model / matlab / design / URDF / meshes)

Branch `v3-handoff-fixes`. Commits (mine): `4b7eab4` (A2 meshes + design/), `7e64607` (A1/A2/S1/S8/S9 generator, model, URDF, tools),
`5156db2` (S10 python tests), and the final commit containing the MATLAB tests, the post_fix logs, renders and this report
(hash in the reply; `git log --oneline -6`). Nothing pushed. Baseline tag `pre-handoff-baseline` untouched.

Hardware-only work (slicing, weighing, camera/ToF purchase, mounts, calibration) is NOT done and stays open.

## Honest headline
* With the more physical model the pretrained Microduck policies get WORSE, and the new tests say so. Under the new criteria
  (`results/post_fix/motions_all.log`): balance PASS, sitstand PASS, roulade PASS, pick PASS, dance PASS, **walk FAIL**,
  **obstacles FAIL**, **fall_getup FAIL**. The pre-fix model run through the SAME new criteria passes all eight
  (`ab_motions_prefix_model_new_criteria.log`), so the criteria are not just stricter-for-the-sake-of-it.
* Causes, isolated with A/B runs (`ab_motions_shield_proxies_off.log`, scratch experiments described below):
  1. The rear shield now has contact proxies. Its lower end/side walls touch the shanks during walking (`leg ~ shield_side` contacts)
     and on its back the 18.5 mm thick shield props the torso up so the robot does not get up. With shield proxies off, obstacles
     and fall_getup PASS again. This is a real design finding (shield position/length is an unmeasured guess, `SHIELD_POS`) and a
     policy-training gap (policies were never trained with a rear shield), not a test bug.
  2. Walk fails even with the shield proxies off: the new head/torso mass and inertia (below) change the pretrained gait
     (0.59 m travel, 107 deg heading change vs 1.03 m / 23 deg before). The gait is very sensitive: head mass x1.18 alone gives 0.85 m.
     Head INERTIA alone is not the cause (scaling it 0.55x-1.5x changes nothing). The old model's head inertia was ~2x too SMALL
     (extra shell mass was modelled as tiny cubes), so the old walking result flattered the robot.
* MATLAB Simulink tests (balance loop, feet-only walk model) all pass with the new assertions: `results/post_fix/test_results.txt`.

## Per item

### A1 black PLA - done (sim + URDF); not verified in a real RViz window
* Commit `7e64607`. Files: `scripts/make_duck_v3.py`, `scripts/mjcf_to_urdf.py`, regenerated `model/duck_v3*.xml`, `ros2/duck_description/urdf/duck_v3.urdf`.
* Material `black_pla` rgba 0.025 0.025 0.025 1, specular 0.35, shininess 0.30 on: V3 upper/lower head, shield, trunk, shells, power_support,
  banana_pcb_locker, motor_support, neck, neck_pitch, yaw2roll, yaw_roll_motion, bearing_roll, hip_l, upper legs + plate, leg, ankles, feet,
  face_part, noenoeil (the yellow lens ring is now black; revert by removing it from `PRINTED_RIGID_MESHES`).
  Left alone: servos (xl330), bearings, battery, both PCBs, speaker, lens holder, TPU soles (yellow), world/targets. Lens -> `lens_glass` (dark blue, high specular).
* URDF: each MJCF material becomes one named top-level `<material>`; specular/shininess are MuJoCo-only (URDF has none).
* Verified: `results/geometry_v3/{front,side,rear,three_quarter,mouth_closeup,mouth_side_closeup}.png` (MuJoCo render of the visual meshes at the
  `sim_start` pose). Black on the blue background is low contrast; the contours are readable in side/closeup but not great. No RViz run.

### A2 mouth - done (geometry), picking re-scored; seam fit not independently verified
* Commits `4b7eab4`, `7e64607`. `design/build_design.py`, `design/modified/duck_head_lower_v3.stl` (old one kept as `*_pre_mouthfix.stl`), `design/mesh_checks.json`
  (pre-change copy `mesh_checks_pre_mouthfix.json`).
* The lip is gone. The bill is a smooth forward displacement of the ORIGINAL lower-shell front wall (push 12 mm, 42 mm half-width plateau, smooth falloff, slight
  downward lean). No boolean: `manifold3d` is not installed here and I did not download it, so the lip ellipsoid union could not be reused; the displacement keeps
  the shell hollow and one closed manifold. Seam plane (z = 22.35), the rear/side mount region and the front-top edge are untouched.
* Checks (`design/mesh_checks.json`): all three parts watertight, positive volume, 1 component. Lower head 86.76 x 116.59 x 22.35 mm (was 117.36 long), volume 41.25 cm3
  (was 44.89 with the lip). Local wall thickness, screw seats and internal clearance were NOT measured (no tool for it here); the displacement is smooth but unproven for printing.
* Removed from the generated model: visuals of `jaw`, `jaw_soft`, `soft_mouth_top`, `top_head_shell`, `bottom_head_shell`, and the collision meshes of `jaw`, top/bottom shell.
  Kept: `face_part` (front sensor plate) and `noenoeil` (lens ring) as black printed parts, lens, lens holder, PCBs. If you want the face plate gone as well, add `face_part`
  to `REMOVED_VISUAL_MESHES` (its mass then must be subtracted too, see S9).
* `jaw_soft` BODY, camera `head_cam`, site `tof`, sensors, keyframes all kept (Opus request). `tof` pose and axes unchanged: +x is boresight forward at STAND (checked).
* `mouth_tip` moved to the bill's lowest-front vertex near the centreline: head-frame (-0.0106, -0.0019, -0.0904) m.
* FOV (`scripts/check_fov.py`, `results/post_fix/fov_check.txt`): 135 head poses, centre rays only. Camera: 0 % rays blocked. ToF at its site: ALL rays hit the front wall
  2 mm away (pre-existing: identical on the baseline model; the site sits behind the front surface, while `head_cam` is 12 mm in front). ToF 12 mm further forward: 0 % blocked.
  **Opus: any mj_ray based ToF/depth that includes visual group 2 or head geoms must start 12 mm forward of the `tof` site (or exclude the head body), or every zone reads ~2 mm.**
* Picking: `motions.py pick` scores the new `mouth_tip` + bill-proxy floor contact. Bill tip got to 9 mm above the floor (pass < 30 mm); the bill proxy itself touched the floor for 0 steps
  (the proxy sits slightly above the mesh tip), so "contact" is informational only.

### A3 shield orientation - not changed by me (checked only)
Shield mesh unchanged (rebuild skipped: no manifold3d). Orientation used by the generator is unchanged (solid face rearward, open ends vertical). Physical checks against legs/head: see S1 findings.

### S1 collision - done, with real findings
* Commit `7e64607`. `scripts/collision_proxies.json` (editable, mm in mesh axes), `make_duck_v3.py`, `scripts/check_collision_proxies.py`, `scripts/check_contacts.py`.
* Head: ellipsoid dome + skirt box + lower-shell box + bill ellipsoid; rear shield: separate back box and two side boxes (open top, bottom, inner side; no hull). Detail meshes stay `contype=0` visuals.
  Proxies exist only in the ground-contact (`gc`) model, like the upstream collision set; the walk/Simulink model is still feet-only.
* Fit (`collision_proxy_check.json`): shield proxies 0 % of samples > 2 mm outside; head 7.5 % of samples > 2 mm outside (worst 5.4 mm, at the rounded edges); the head boxes overreach
  the rounded plan corners by up to ~15 mm in places. Primitives are coarse on purpose.
* Contacts (`contact_check.txt`): no self-contact at STAND. Dropped on front/back/left/right/upside down the expected proxies reach the floor (bill on front; shield on back; dome/lower on sides).
  **Design finding:** inside a modest head envelope (neck +-0.4, head pitch +-0.6, yaw +-1.2 rad) the head clashes with the shield/battery in many poses (the head rear sits only a few mm from the shield top); over the full joint
  range clashes are common. Contacts are left enabled (they are physical). Legs also touch the shield sides during walking.
* Exclusions: none added. I did not need any; parent-child filtering handles the rest.

### S8 reproducibility - partial (done in my scope; package manifests/pinning not done)
* Done: `make_duck_v3.py` copies missing upstream files even when `model/` exists (never overwrites), creates output dirs; roots from `__file__` with `DUCKLING_ROOT` / `DUCKLING_UPSTREAM` /
  `DUCKLING_DESIGN` overrides; `prep_v3_meshes.py` reads the versioned `design/` (original parts only for the volume bookkeeping, constants recorded) and writes no absolute paths;
  `design/build_design.py` locates `original_parts` via argv/env/`design/original_parts` (git-ignored, CC BY-NC-SA upstream parts not versioned).
  New `scripts/preflight.py` (XML includes, mesh refs, MuJoCo compile of every scene, V3 meshes and design checks, policy ONNX load + 61->14 shape, detector ONNX, world.json vs world XML,
  `--ros` installed package assets, sha256 checksums in `model/checksums.json` via `--write-checksums`) and `scripts/sync_ros2_assets.py` (replaces the manual cp in setup_nav.sh for the model files).
  Current preflight: 0 errors, 4 warnings (PROVISIONAL mass, 3 optional policies not downloaded: roller, roller_crouch, ball_kick_right).
* The checksums were taken from the files already on this machine; they are NOT verified against an upstream signature.
* Process cleanup: the ROS shell scripts had already been narrowed by Opus (own process group); I did not touch them. I removed no hard-coded `C:\dev` from `matlab/build_blockset.m` comment and README (README text left for Opus).
* Open: missing original head meshes are located via `design/original_parts` or env only (upstream commit not pinned); Python/ROS dependency pinning and package manifests; fresh-workspace build not tested.

### S9 mass - done (as PROVISIONAL), whole-robot numbers regenerated
* Commit `7e64607`. `make_duck_v3.py` (EDIT block `V3_PART_MASS_G` for slicer/scale values), `model/mass_ledger*.json`, `model/stand_numbers.json`, `matlab/duck_numbers.m`.
* Per-part table, all PROVISIONAL (handoff wall-proxy formula, not a slicer): upper head 51.5 g, lower head 43.11 g (recomputed for the new bill: V 41 254.8 mm3, A 26 654.5 mm2), shield 32.3 g.
* Head link = upstream head (188.8 g) minus the replaced upstream parts plus the V3 shells, with mass, COM and full inertia tensor recomputed (parallel axis), checked physical (no clamping): 178.8 g.
  The replaced upstream parts (top/bottom shell 37.0 + 29.5 g, jaw 11.6, jaw_soft 23.1, soft_mouth_top 3.3 = 104.5 g) are an ESTIMATE (mesh volume x 1240 kg/m3, solid): the true Onshape masses are not in the repo. If those parts were lighter,
  the remaining head is heavier. Torso = upstream 199.2 g + shield 32.3 g = 231.5 g (shield is a separate addition, never counted in the upstream link). A weighed `jaw_soft` value is documented to mean the head WITH shells and skips the composite.
* Whole robot 759.6 g (was 789.2 g), CoM 2 mm further back (x -0.0029 m, was -0.0009), CoM height above ankle 0.1166 m (was 0.1196). Provenance (scene hash, ledger) is in `stand_numbers.json`; `duck_numbers.m` prints it and saves `MODEL_REV`.
  Gains recomputed: ankle 2.34*pitch + 0.186*rate (was 2.49/0.200); hand-calc push limits fwd 0.227, bwd 0.090 N.s. `duck_numbers.m` also now saves `ANKLE_MAP` (re-running it used to delete it from `duck_params.mat` and break `run_tests`).
* Not done: `duck_personal/TODO.md` (not in this repo); servo model (still upstream XL330 numbers); README numbers table is stale (left to Opus per request).

### S10 tests - done; results include genuine failures
* Commits `5156db2` and the final commit. `scripts/motions.py`, `scripts/push_test.py`, `matlab/run_tests.m`. Old outputs moved with `git mv` to `results/pre_fix/` (+ README saying superseded).
* motions.py: balance (tilt <= 10 deg, height >= 87 %, drift <= 0.10 m, settled by 4 s, joint margin >= 0.02 rad, longest actuator saturation <= 1 s, no fall), walk (travel >= 0.8 m in 8 s, lateral drift <= 0.30 m, heading change <= 45 deg, continuous fall
  monitor, ends upright), recovery (escalating 12/20/30/45 N pushes, "got up" only after a confirmed fall, else INCONCLUSIVE "push absorbed", 45 N labelled as a search cap), pick (new tip), dance/obstacles use the monitor. Exit 0/1/3. Thresholds were fixed before looking at the new model's numbers; they are mine, not the handoff's.
  Fall monitor = tilt > 60 deg, torso < 0.07 m, or head/shield proxy on the floor for >= 3 control steps.
* push_test.py: continuous fall judgement; `--search` reports a cap as "bounded below". Result (`push_test_search.log`): forward survives 6.0 N, falls at 6.5 N; backward survives 1.0 N, falls at 1.5 N (was 5.0/5.5 and 2.0/2.5).
* run_tests.m: assertions with declared criteria (tilt/height/drift/settling/joint margin/saturation/recovery/limit sanity/delay/mass) and an `error()` (nonzero `matlab -batch` exit) on any failure; results to `results/post_fix/`. Run result: 0 assertions failed (feet-only Simulink model, no head/shield contact).
  Limits: only knee/ankle joints are logged, saturation is the left-ankle command clip, `torso_pos` is the IMU site; the plant logs an all-zero first sample, so references use t >= 0.1 s. The nonzero-exit path was only exercised by an earlier crashed run (exit 1), not by a deliberately failing assertion.
* Not done: time-aligned estimate-vs-truth scoring, multi-layout/seed/delay/outage sweeps (score_node / ROS, Opus).

## Other open items / things I am unsure about
* Head collision primitives are coarse; the ToF-origin offset above affects any raycast sensor using visual meshes.
* `model/duck_v3_gc_obstacles.xml` (tracked) is rewritten by `motions.py obstacles`; `duck_v3_gc_room.xml`/`world` were not regenerated by me except `worldgen.py` (content unchanged).
* MATLAB `duck_sim.slx` was not rebuilt (it reads the regenerated XML at run time); MATLAB complains it cannot close the changed model at exit (harmless).
* I did not run `slam_nav.py`, ROS or YOLO; Opus's `ros2/duck_nav/mjcf/` copies were refreshed with `scripts/sync_ros2_assets.py` (git-ignored copies).
