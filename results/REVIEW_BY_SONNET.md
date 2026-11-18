# Review of Opus commits f1db09d and 644e68e (read-only review by Sonnet)

Scope: `ros2/duck_nav/*`, `ros2/duck_description/duck_description/duck_mujoco_sim.py`, ROS scripts, unit tests. Nothing was edited or committed.
Line numbers refer to the working tree at HEAD (`git show` of the two commits matches it).

## Verdict
No show-stopper found. The planner, stop semantics, timing and ToF simulation are well structured and mostly match the handoff. Real issues, in order:
1. A thread race in `nav_node.replan` that can crash the node (MEDIUM, by reading).
2. Head yaw / pose commands never expire (MEDIUM-LOW, by reading + code inspection).
3. A disable/finish race can leave one non-zero Twist after the zero (MEDIUM-LOW, by reading; bounded by world_sim expiry).
4. The ToF-to-camera projection ignores body motion between the two stamps (LOW-MEDIUM, by reading).
Everything else below is low severity or a limitation that should be stated in docs.

## Confirmed by running
* `.venv/Scripts/python.exe -m pytest ros2/duck_nav/test -q`: 21 passed.
* **Your question: does the ToF raycast self-hit with my new head primitives? NO.** I ran `mj_ray` with `geomgroup = [1,1,0,1,0,0]`,
  `bodyexclude=-1`, from the `tof` site, 64 zones from `duckslam.tof_zone_rays()`, on `model/duck_v3_gc_world.xml`:
  - level head, STAND: 0 self-hits; nearest return 723 mm (the room partition `p1`), rest floor / no hit.
  - 135 head poses (neck +-0.4, head pitch +-0.6, yaw +-1.2, roll +-0.43): 0 rays hit any robot geom within 150 mm.
  - Why my earlier "2 mm" finding does not bite you: that was a ray against VISUAL meshes (group 2) from inside the face plate. The collision
    primitives (group 3) are coarse and the `tof` site lies outside all of them, so the ToF effectively sees THROUGH the front wall and the face plate.
    So: no spurious ~2 mm returns, but also the sim ToF is not blocked by the real front surface / any bracket (none is modelled), and the bill only shows
    up if it enters a ray. State this as a limitation in the realistic-profile docs, do not claim the field of view is verified against the real head.
* `tof` site axes in world at STAND are identity: +x forward, +y left, +z up, which matches `tof_zone_rays` (sensor frame +x boresight, +y left, +z up).
  Zone 0 (top-left) points (0.89, +0.32, +0.32), zone 7 (top-right) (0.89, -0.32, +0.32): consistent with the docstring ("row-major, top-left as seen from behind").
* Planner probes on the default map (`ds.plan`, `ds.exploration_state`): all-free map plan to the beacon 0.15 s (161 cells), exploration plan 0.03 s,
  all-unknown grid -> "waiting"; closed room -> "complete"; closed room with a 6-cell (15 cm) unknown hole in the wall -> "complete"; 12-cell hole -> "stalled".
  Plan time is small enough that the separate callback group is not critical, though BFS is pure Python (GIL).
* The post-fix ROS runs in `results/post_fix/ros2_nav_*_baseline_model` ran on the PRE-FIX robot model (the label says so). They say nothing about the new black/bill/shield model.
  On the baseline model the ideal run ended "exploration stalled ... open frontier 30 cells", beacon estimate error 0.20 m, SLAM mean error 0.175 m: the stated outcomes are honest, not success claims.

## By reading (not reproduced)

### MEDIUM
1. **`nav_node.py:150-178` race between `replan` and `drive`/`on_enable`/`finish`.** `replan` assigns `self.path`, then 25 lines later does
   `self.path[::3] + [self.path[-1]]` (line 175). `drive()` runs in another callback group and calls `self.stop()` (line 187 via `fault()`), which sets `self.path = None`
   (line 117); `on_enable(False)` (default group) does the same. If either lands between lines 150 and 175, `replan` raises `TypeError: 'NoneType' object is not subscriptable`.
   In a `MultiThreadedExecutor` an exception in a callback propagates out of `spin()` (line 215-220 re-raises), so the navigator dies (the `finally` publishes a zero, so it fails safe but the mission is lost).
   Window is small (plan takes 30-150 ms, a fault must start in that window) but real, e.g. a stale-pose blip during a plan.
   Fix: `path = self.path = result`, use the local `path` for lines 154-178; same for `self.mode`. Better, one `threading.Lock` around (path, mode, grid, info, pose) updates, or make `stop()` not touch `self.path`.
2. **`world_sim.py:177-182` head and pose commands never expire (handoff S4: "a previously latched command must not persist indefinitely").** `on_cmd` is covered by `expire_command`, but
   `head_yaw` (via `/head_yaw_cmd`) and `pose_cmd` are latched forever. If `nav_node` dies or is disabled mid-sweep, the head stays turned by up to 0.9 rad and is added into the policy command
   (`c[5] += self.head_yaw`, line 213) for ever; `nav_node.drive` returns early when disabled so nothing resets it, and `stop()` sends only a Twist.
   Fix: apply the same sim/steady-clock expiry to `head_yaw` and `pose_cmd`, or have `stop()`/`finish()`/shutdown also publish `head_yaw_cmd = 0`.

### LOW-MEDIUM
3. **`nav_node.py:99-107, 121-124, 181-207` non-zero command after the zero.** `drive()` checks `self.enabled`/`self.finished` once at the top, then publishes at line 207.
   `on_enable(False)` (default group) or `finish()` (replan group) can publish the zero Twist in between, after which `drive` publishes one more walking command.
   In simulation `cmd_timeout` (0.5 s) bounds this to a short unwanted walk at the hand-over; on hardware without that expiry it would persist. The "hand over with the robot commanded still"
   guarantee in `on_enable` is therefore not strict. Fix: a lock shared by `drive` publish and `stop/finish/on_enable`, or re-check `enabled/finished` immediately before `publish`, and publish zero a few times (or at the cmd rate until the first stale tick).
4. **`slam_node.py:269-277, 328-335` ToF/camera association ignores motion between stamps.** ToF points are in the level frame at the ToF stamp; they are projected with the camera pose at the camera
   stamp, matched "nearest within 150 ms". The level frame rotates with the body yaw (up to 1.2 rad/s turn command) and translates at ~0.13-0.2 m/s: 50 ms is ~3.4 deg / 1 cm, 150 ms ~10 deg.
   The accept test is only +-(w/4+2) px, so a turning robot can associate the wrong zones or fail. You already have `pose_hist`: transform the points from the ToF acquisition pose to the camera acquisition pose before projecting,
   and tighten the 150 ms window to ~60 ms.
5. **`nav_node.py:68-82, 150` non-atomic `self.info` / `self.grid`.** `on_map` sets `info` then `grid` while `replan` reads `self.grid` then `self.info`; if a map with different dimensions ever arrives, `plan` raises `ValueError`
   (shape mismatch, `duckslam.py:345`) in the timer callback, which ends `spin()`. Not reachable with the current fixed-size SLAM map. Store them as one tuple.
6. **`nav_node.py:192-196` unstick timer survives a fault.** `progress_ref` is not reset while `fault()` returns early, so after a >8 s stale-pose/map stop the first `drive` tick after recovery sees `t - progress_ref[2] > 8` and starts a 2.5 s spin in place.
   Reset `progress_ref` whenever `fault()` is true.

### LOW (correctness nits / semantics)
7. `nav_node.py:147-149` "beacon reached" uses the straight-line distance of the ESTIMATED pose (0.36 m), with no line-of-sight/path check. A beacon behind a thin wall flips `visited_goal` while the robot passes on the other side.
   Score uses the true distance (0.45 m), so the two can disagree (that is the intended separation, but the nav-side flag stops further beacon pursuit). Consider requiring a reachable path or a recent ranged sighting.
8. `nav_node.py:159-165` `finish("exploration stalled")` is permanent after 5 non-consecutive-reset plans (~5-7 s). In the baseline ideal run the mission ended 'stalled' at 104 s with 19 m2 known and 30 open frontier cells.
   `duckslam.py:374` ignores frontiers closer than 0.35 m of path distance, and the planner's frontier labelling is 4-connected (`ndimage.label` default, line 375) while `exploration_state` uses 8-connectivity: a diagonal frontier chain is "no frontier" to the planner but an "open frontier >= 8" to the state check,
   so these cases end as 'stalled', not 'complete'. Safe direction (never false success), but it terminates instead of retrying. Consider continuing to spin/relocate for longer or until map change stops.
9. `duckslam.py:402-409` `exploration_state` labels with 8-connectivity over `grid == 0`, so the region leaks through diagonal gaps between occupied cells that the planner forbids (line 317). Conservative (can only turn "complete" into "stalled").
   Also "complete" tolerates any wall hole under 8 frontier cells (confirmed: 15 cm). Given 0.125 m clearance a robot cannot use it, so acceptable, but say so.
10. `duckslam.py:353-356` unknown cells inside the robot footprint disc (radius = clearance) are treated as traversable. Intended and documented; note it also lets BFS escape through unseen cells up to 12.5 cm from the robot.
11. `world_sim.py:133` the RGB/depth renderer hides geom group 2, i.e. the duck's own visuals never appear in the camera. This contradicts handoff H3 ("do not train solely on ideal renders with the duck hidden if the real bill is visible in frame").
    Acceptable for the sim, but the YOLO training data and the docs should say the bill/face plate are not rendered.
12. `world_sim.py:140` rays use collision groups only. Same-scale limitation as above: no visual-only parts (face plate, lens ring, brackets) block the ToF.
13. `world_sim.py:262-296` ToF realism vs handoff: 45 deg/axis (OK), 10 Hz (OK, <= 15 Hz), 60 ms latency (pose published at acquisition, cloud delayed: good), invalid != free (good, `scan_from_tof` drops them), noise + zone/frame dropouts (good).
    Missing and not claimed: mixed-target / multi-surface zones, ambient/dark-surface range loss, per-zone confidence/sigma, floor-return handling. The docstring says centre ray per zone; keep that statement in any result write-up.
    Range convention 'radial' by default is a documented assumption that must be checked on the real driver.
14. `score_node.py:68-69` fall = true z < 0.085. The sit/stand and pick behaviours lower the torso to ~0.06 m, so a commanded sit would be counted as a fall. Not used by nav today.
15. `score_node.py:79-83` pairing is exact-stamp (good), but it assumes the SLAM map frame origin equals the MuJoCo world origin (both start at (0,0,0)); say so, or the "error" includes any start offset.
16. `slam_node.py:299-305` docstring says "within 0.3 m" but the acceptance gate vs the running estimate is 0.5 m; after the goal exists any sighting within 0.5 m moves it (running mean). A wrong second object <0.5 m away shifts the goal.
17. `slam_node.py:218-224` the carried correction is added to the current pose but not to older `pose_hist` entries. This is fine at 10 Hz ToF with 60 ms latency (the next acquisition stamp is after the correction), but breaks (correction applied twice) if latency >= frame period (e.g. `tof_latency` > 0.1 s). Shift the stored history by `corr` for stamps newer than the frame.
18. `usb_camera_node.py` stamps with the host clock at dequeue (documented) and publishes in the REP-103 optical frame while `slam_node` expects the MuJoCo camera convention (documented): fine, but there is no hardware pose publisher, so the realistic path is simulation-only today. Do not report H2/H4 as hardware-ready.
19. `ros2/run_nav_headless.sh`: `rm -rf "$OUT"` on an env-supplied `OUT` with no sanity check (a bad `OUT=/` style value would be destructive); `kill -- -$PGID` assumes `setsid` execs rather than forks (true in a non-interactive script, not under job control, where the cleanup would silently kill nothing).

## Checked and found correct
* Corner rule (`duckslam.py:317`) and clearance via EDT (`:300-304`), no start-disk override of occupied cells, `start_blocked` when the robot cell is occupied (`:351`), out-of-map on floor index (`:347-349`), escape path over known-free cells to the nearest fully cleared cell (`:359-364`).
* `level_to_pixel` / `pixel_to_level` are inverse and follow the MuJoCo camera convention (looks along -z, y up); `CameraInfo` K is built from the same `fovy`.
* `world_sim.step`: `mj_forward` after reset and after integration; stamps from `k*dt`; camera and ToF poses are expressed in the level frame computed from the same post-forward state; ToF pose published at the acquisition stamp, cloud delayed by the latency queue; `cmd_vel` expiry on both sim and steady clocks with `cmd_expired` initially True.
* `yolo_node.letterbox`: scale/pad and the inverse mapping are consistent (integer floor pad used in both directions); it is the old behaviour for 320x240 into a 320x256 model; class count check; stride-aware decoding.
* `score_node`: stamp-matched SLAM error, true-pose falls and beacon distance, TIMEOUT never reported as success, results separated into exploration vs beacon.
