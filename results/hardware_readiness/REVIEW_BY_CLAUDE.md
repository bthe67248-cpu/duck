# Review of `duck_hardware_readiness.zip` (patch on 99c407d), by Claude

18 November 2026. Patch applied on local branch `review/hardware-readiness` (commit `bb0e18c`); follow-up work on
`depth-camera-opt`. Nothing pushed.

## What was checked

| Check | Result |
|-|-|
| Zip contents vs patch | Identical file set (36 files); no extra files, no Pollen meshes or policies (only `policies/manifest.json`, already public) |
| `pytest tests ros2/duck_nav/test` with the pinned upstream assets | **94 passed**, as claimed |
| `scripts/check_pcb_target.py` rerun | Reproduced: 1132 → 333 → 0 → 0 bad poses (worst penetration 25.4 / 16.8 / 0 / 0 mm; theirs 24.3 / 15.6) |
| Full ROS 2 realistic-profile room trial (they ran none) | **Regression**, see below |

## Main finding: the stricter localisation gate stops the duck exploring

Same world and seed, realistic sensors, 600 s:

| | Before the patch (`main`) | With the patch |
|-|-|-|
| Beacon | reached at 100 s | **not reached** |
| Path walked | 33.6 m | 6.4 m |
| Room covered | 90 % | 30 % |
| Mean SLAM error | 0.25 m | 0.36 m |

Log: `slam weak` almost continuously; the navigator gets about 0.5 s of driving per 4 s head sweep.
Causes, confirmed by replaying a recorded run offline (`ros2/duck_nav/tools/slam_replay.py`):

1. **The map only grows on an accepted match** (or while standing at the origin). New territory has nothing to
   match against, so the map freezes at its first view.
2. **Bootstrap needs the duck at the origin.** From the room centre the walls are beyond the 2 m ToF range; in an
   open-loop replay where the duck walks off at 2 s the map never starts at all (3003 of 3081 frames
   `map_not_ready`).
3. **The degenerate-geometry check rejects most real ToF sweeps** (1870 of 3081 frames with the old matcher),
   and the current-frame agreement check rejects most of the rest. With few accepted matches, odometry drift
   outgrows the ±7.5 cm search window and never comes back.

The idea behind the gate (do not drive on stale or weak localisation) is right; the thresholds are not, and they
were only checked on a stationary synthetic corner.

## Other notes

- **PCB / shield study:** the zero-contact result moves the shield 20 mm farther back. That may simply float it
  clear of the torso; the study says nothing about how it mounts. The handoff itself is careful to call it a
  layout target, which is correct.
- **CI workflow** clones Pollen's `microduck_rl` at test time. Nothing from Pollen is committed, so it respects the
  "no Pollen files in the repo" rule, but it is a runtime dependency on that repository.
- **Code quality** is good: careful validation, honest documentation, real tests of node methods with shims.
  `sensor_contract`, `tof_node`, `sensor_pose_node`, the calibration JSON and the stricter contact gate are
  worth keeping.
- The 112 kbps → 2 Mbps Bridge correction and the 38.4 kbps payload arithmetic are right.

## Recommendation

Keep the patch's hardware interfaces, calibration workflow, contact gate and documentation. Do not keep its
localisation rules as they are; `depth-camera-opt` replaces them (see `results/slam_tuning/README.md`).
