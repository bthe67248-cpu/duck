# Review of Sonnet's commits (4b7eab4, 7e64607, 5156db2, a47e6b1, 7e2b658), by Opus

Scope: A1, A2, A3, S1, S8, S9, S10 (scripts/, model/, matlab/, design/, URDF). Read the diffs and checked
the evidence files; re-ran nothing heavy myself except where marked.

## Verdict
The work is careful and honest. Nothing found that makes a result wrong. The important output is the set of
**new failing tests**, and the A/B logs support Sonnet's diagnosis of them.

## Checked and agreed
- **S9 composite mass** (`scripts/make_duck_v3.py` `combine`, `check_physical`, head/torso blocks): the
  replaced upstream parts are subtracted as negative-mass parts with their own inertia, the V3 shells are added at
  their provisional masses, and the parallel-axis transforms are about the new COM. There is no clamping, and a
  non-physical result raises. The torso adds the shield once. The ledger (`model/mass_ledger*.json`) records the
  provenance. Total 759.6 g (was 789.2 g).
- **S10 A/B evidence**: `ab_motions_prefix_model_new_criteria.log` (old model + new criteria: 8/8 pass) and
  `ab_motions_shield_proxies_off.log` (obstacles and fall_getup pass again; walk still fails) support both causes:
  the shield proxies (leg-side contacts x25 in `motions_all.log`; shield props the torso on its back) and the
  changed head/torso mass for walk.
- **S1 proxies**: MuJoCo half-extents are used correctly (shield 29.97 x 58.3 half-sizes = the 59.94 x 116.6 mm
  envelope; walls 1.25 mm half = 2.5 mm). The shield is three separate boxes, so the U stays open.
- **A2**: the lip ellipsoid is gone; all three meshes are watertight, one component, positive volume
  (`design/mesh_checks.json`); `mouth_tip` is moved to the new bill tip and picking scores it.
- **FOV**: `results/post_fix/fov_check.txt`. Camera 0% blocked over 135 head poses. It also caught a real
  issue for my ToF sim (fixed in 322fab7, see below).

## Findings
1. **Medium: `motions_all.log` obstacles.** `head_upper_skirt_collision ~ right_foot_collision x8`. The head's
   skirt proxy touches a foot during the plank walk. Either the proxy (40 x 44 x 9 mm half-sizes, low on the
   head) is too big, or the head really pitches that low. Check the proxy against the mesh in that pose before
   reading the obstacle result as a shield-only problem.
2. **Medium: the shield position is a guess, and the tests now depend on it.** `SHIELD_POS` was never
   measured, and the obstacle and get-up failures come from where it sits. This is the most important input
   to measure before printing the shield.
3. **Low-medium: removed upstream parts at solid density** (`UPSTREAM_PART_DENSITY`, 104.5 g removed). If the
   upstream parts were printed sparse, this over-subtracts and the head ends up lighter than reality. It is
   labelled ESTIMATE, and weighing the real head replaces it.
4. **Low: A1 also blackens the lens ring** (`noenoeil` in `PRINTED_RIGID_MESHES`). It is not a printed rigid
   part in the strict sense. Cosmetic; your call.
5. **Low: the bill reads as a short ledge under the camera face plate** (`results/geometry_v3/mouth_side_closeup.png`).
   Keeping the face plate is right (it holds the camera), but whether this is "one integrated duck bill" is a
   design judgement for you.
6. **Info: A3 (shield rebuild) is not done.** The shield STL is unchanged (needs a mesh boolean library that was
   not installed). Its orientation in the model is unchanged and matches the handoff (STL Y vertical, Z depth).
7. **Info: S8 is partial.** No dependency pinning or package manifest work, and no fresh-workspace build.
   Checksums are of local files.

## Sonnet's review of my work, and what I did with it (see REVIEW_BY_SONNET.md)
- Item 0, "the ROS runs used the pre-fix model": **disagree, checked.** The installed package's
  `duck_v3_gc.xml` contains `bill_collision`, and the sync (15:50) was before both runs (about 15:56). The run
  folders are now named `*_v3model*`.
- 1 (`self.path` race), 3 (a walk command after a stop), lower-priority items (atomic `info`/`grid`, unstick
  timer, sit counted as a fall, `rm -rf $OUT`, 4- vs 8-connected frontiers, robot not in camera images, double-applied
  correction at high latency): **fixed in 4cbf87f.**
- 2 (head/posture commands never expire): **fixed**. They now expire like `cmd_vel`.
- 4 (ToF projected with a camera pose up to 150 ms apart): **fixed**. The points are moved through the pose history.
- "Beacon reached" uses a straight-line distance with no occlusion check: **left as is**, noted as open.
