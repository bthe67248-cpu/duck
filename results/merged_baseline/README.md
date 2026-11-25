# Baseline of the merged code (main at 1bb43b3)

A fresh trial matrix on the merged configuration: PRs #11 (dynamics: explicit LiPo, 100 g upstream battery,
20 ms servo delay, opt-in sensor errors), #12 (corner navigation) and #13 (opt-in second-peak test, layout b) together,
with launch defaults. It replaces the per-branch results as the reference for the combined code. PR #14 (merged after
these runs) does not change default simulated behaviour: the published ToF xyz equals the reconstructed one in
simulation, the A010 fix is hardware-only, and the second-peak test is off by default.

**Setup.** 600 s closed-loop runs, realistic profile, A010 25 × 25 in the body, seeds 0–5, one run per cell:

- layout A: the default room (`duck_v4_gc_world.xml`), beacon at (4.0, 1.3);
- layout B: `scripts/worldgen.py --layout b`, one 3 m featureless partition and a shielded beacon at (3.95, -1.55)
  (`mjcf:=… world_b.xml world_json:=… world_b.json beacon_x:=3.95 beacon_y:=-1.55`);
- sensor errors off (default) and on (`sensor_errors:=true`: IMU bias/noise, encoder offsets, extrinsic and timing
  errors, world_sim `SENSOR_ERROR_DEFAULTS`).

Every run held real time (sim = wall in `launch.log`). Model and policy hashes, the commit and every run's launch
arguments are in `manifest.txt`; per-run `summary.txt`, `result.json`, `map.png` and `launch.log` are in `runs/`.
Three runs at a time, only while no other simulation was running.

| Configuration | Beacon reached | SLAM mean error (median of runs) | Worst max error | Walked (mean) | Coverage | Falls | Exploration completed |
|-|-|-|-|-|-|-|-|
| Layout A, errors off | 5/6 | 0.33 m (0.26 m) | 1.82 m | 23.0 m | 84 % | 0 | 0/6 |
| Layout A, errors on | 4/6 | 0.57 m (0.29 m) | 3.99 m | 21.4 m | 78 % | 0 | 0/6 |
| Layout B, errors off | 2/6 | 0.78 m (0.51 m) | 2.83 m | 16.4 m | 56 % | 0 | 0/6 |
| Layout B, errors on | 2/6 | 0.83 m (0.82 m) | 3.09 m | 17.3 m | 63 % | 0 | 0/6 |

Per run: `table.txt`.

**What it says.**

- In the default room the merged code reaches the beacon on 5/6 seeds with a median mean error of 0.26 m. One seed
  (5) still drifts late (0.70 m mean, 1.82 m max). Nothing finishes exploring, and nothing falls.
- Sensor errors at the estimated levels mostly leave the median unchanged (0.29 m), but one seed (3) diverged
  (1.99 m mean, 3.99 m max) and one more missed the beacon. They make the bad cases worse rather than shifting every
  run, so the robustness work should be judged with them on.
- Layout B is clearly harder: 2/6 beacons, mean error 0.78–0.83 m and coverage around 60 %, whether or not errors are
  on. The long featureless partition is where the pose slides (the drift mode described in `design/v4/README.md`,
  "Late drift and the yaw sweep"), and the shielded beacon needs more of the room explored. That is the case to tune
  SLAM against next, not layout A.
- One run per cell: differences of one beacon or a few tenths of a metre between rows are within run-to-run noise.
  The layout A vs B gap and the diverging errors-on run are the robust findings.

**Model.** These runs predate the head and body fit fixes (PR #16): the torso was 257.6 g, now 278.2 g with a lower COM
and different inertia. They are the earlier reference, not a measurement of the current model.

**Not covered.** Hardware; the yaw sweep (off by default, no measured benefit in PR #13's repeated trials); repeated
runs per cell.
