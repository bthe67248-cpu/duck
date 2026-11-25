# Corner navigation: planner and controller agree near walls

Simulation only (MuJoCo + ROS 2, realistic sensor profile, body-mounted A010 25 × 25 zones, launch defaults).
Nothing here has run on hardware.

## Problem

The wavefront planner kept every path cell `clearance` (0.125 m) from obstacles, but it took the shortest path, so at
corners the path ran exactly along the clearance margin. Pure pursuit then aimed at a point 0.22 m ahead. At a corner
that chord cuts inside the path, the commanded arc failed `motion_clear`, and the guard stopped forward motion
("path clearance guard: turning in place and replanning"). Replanning gave the same margin-hugging path, so the guard
fired again. On `main` this happened 24–162 times per 600 s run, and the 8 s no-progress unstick fired 4–7 times.

## Change (`duckslam.py`, `nav_node.py`)

- **Planner keeps off walls where there is room.** `plan` now runs Dijkstra (scipy csgraph, about 25 ms on the full
  260 × 220 map) instead of a hop-count BFS. Every cell within `plan_comfort` = 0.10 m beyond the clearance costs up to
  3× more per metre (quadratic). Paths follow the middle of corridors and swing wide round corners, and only use the
  margin in gaps narrower than clearance + comfort. The hard clearance and the no-corner-cutting rule are unchanged.
- **Adaptive lookahead and speed.** `adaptive_lookahead` shrinks the lookahead from 0.22 m to 0.10 m as the path turns
  ahead (heading change between the two halves of the next 0.35 m), and also as the robot nears the margin. The walk
  command drops from 0.5 to 0.35 at a right-angle corner (below 0.25 the gait stands).
- **Arc search before stopping.** When the pure-pursuit arc fails `motion_clear`, `choose_arc` tries 13 turn rates from
  −1.2 to 1.2 rad/s. Each arc is checked with the same conservative speed bound and footprint check as the guard. The
  search keeps the clear arc that gains the most arc length along the path, minus cross-track error, and needs at least
  3 cm of progress. That arc is driven at the slow walk command.
- **Guard stays the safety net.** If no arc is clear, forward motion stops and the robot turns in place, as before (now
  at least 0.5 rad/s, so it never sits at a near-zero turn rate). A final `motion_clear` check runs before any forward
  command is published.
- **Recovery after repeated guard failures.** After 2 s of guard without 0.10 m of progress, the robot first skips
  that frontier: the planner's new `avoid` list keeps it out for 60 s. A skipped frontier never ends exploration
  (`no_frontier` with skips replans without them). The second time in the same spot, or when heading for the beacon,
  it makes a local manoeuvre instead: it turns to the clearest of 12 headings and takes a short step (up to 1.2 s),
  guarded the whole way. When nothing is clear all round, the old 2.5 s turn in place runs.
- `corner_handling:=false` (launch argument and node parameter) restores plain pursuit + guard with shortest paths.

Unit tests (`ros2/duck_nav/test/test_nav_safety.py`) use an L-shaped 0.5 m corridor. They check that the weighted plan
clears the inner corner by at least 0.17 m (shortest path: 0.125 m) and that the lookahead shrinks at the corner and
near the margin. At a pose where the 0.22 m pursuit arc clips the corner, the shortened lookahead walks on without the
guard; where even that arc fails, the arc search finds a clear one. Facing the wall at the margin, no arc is offered, so
the guard stops and the recovery sequence runs (frontier skip, then manoeuvre). The manoeuvre walks only while clear,
boxed-in falls back to turning in place, `corner_handling` off keeps the old behaviour, and avoided frontiers never end
exploration.

## Closed loop, seeds 0–5

600 s per run, one run per seed and arm, `job3.sh` (own workspace and domain, GPU rendering, YOLO 2 threads), at most
3 at a time. Every run in the table held real time (`sim N s, wall N s` equal at the end and throughout). A first
after-run of seeds 3–5 lagged (up to 46 s behind while another session loaded the PC), so it was discarded and rerun.
Its results were close to the rerun (beacon 113–186 s, 93–97 % coverage, 3–7 guard events). A run ends early when the
beacon was reached and exploration is judged stalled.

`main` = c1cf128 (`cbase_*`), this branch (`cnew_*`). Guard = number of times the status switched to the clearance
guard. Arc = switches to "steering on a clear arc". Recov. = frontier skips + manoeuvres. Unstick = the 8 s
no-progress turn.

| Seed | `main`: beacon, coverage, guard, unstick, SLAM mean / max | this branch: beacon, coverage, guard, arc, recov., unstick, SLAM mean / max |
|-|-|-|
| 0 | 365 s, 81 %, 162, 5, 0.18 / 0.47 m | **135 s**, 76 %, **9**, 3, 2, 1, 0.35 / 0.74 m |
| 1 | not reached, 86 %, 24, 0, 1.42 / 2.26 m | **157 s**, **97 %**, **4**, 2, 0, 1, 0.25 / 0.59 m |
| 2 | 324 s, 97 %, 120, 7, 0.18 / 0.48 m (stalled at 570 s) | **148 s**, 96 %, **3**, 3, 1, 1, 0.35 / 0.61 m |
| 3 | not reached, 72 %, 137, 4, 0.25 / 1.00 m | **150 s**, **95 %**, **12**, 5, 0, 0, 0.26 / 0.56 m |
| 4 | 195 s, 70 %, 42, 4, 1.51 / 2.16 m | **154 s**, **96 %**, **1**, 3, 0, 0, 0.23 / 0.76 m (stalled at 330 s) |
| 5 | 217 s, 72 %, 100, 5, 0.34 / 0.69 m | 328 s, **96 %**, **2**, 2, 0, 0, 0.22 / 0.56 m |
| **all** | **beacon 4/6 (mean 275 s), coverage 80 %, guard 585 (98 per run), unstick 25, SLAM 0.65 m** | **beacon 6/6 (mean 179 s), coverage 93 %, guard 31 (5 per run), unstick 3, SLAM 0.28 m** |

Falls: 0 in all 12 runs. Path walked: `main` 19–40 m (mean 30 m), this branch 27–33 m (mean 30 m).
Beacon estimate error at the end: `main` 0.13–0.44 m, plus 2.24 m on seed 1; this branch 0.14–0.46 m.

**Result.** Guard events fell from 98 to 5 per run, and no-progress unsticks from 25 to 3 in total. The duck reached
the beacon on 6/6 seeds (4/6 on `main`). It was sooner on 3 of the 4 seeds that both arms reached
(mean 179 s over 6 seeds against 275 s over 4). Room coverage rose from 80 % to 93 %. Almost all of
this came from the planner and the shorter lookahead. Once paths stopped hugging the margin, the arc search was needed
only 2–5 times per run and the recovery manoeuvres 0–2 times.

Caveats:
- One run per seed and arm. Seed 5 reached the beacon later than on `main` (328 s against 217 s).
- The SLAM error difference (0.65 m against 0.28 m mean) comes mostly from two `main` runs (seeds 1 and 4) whose
  localisation drifted to 1.4–1.5 m mean. Less turning in place against walls plausibly helps the body-fixed depth
  sensor, but these runs do not show that. Treat it as noise until repeated.
- **Exploration still never completes:** 0/6 on both arms. Runs time out at 93–97 % coverage or are judged stalled
  (one per arm) with an open frontier of 35–52 cells left that the planner cannot reach with the clearance. Where those
  frontiers are and whether they matter was not examined. It looks like a frontier-reachability or completion-criterion
  question, separate from the corner disagreement fixed here.

## Files

- `runs/cbase_s*`, `runs/cnew_s*`: `summary.txt`, `result.json`, `map.png`, `launch.log` per run.
- `trial_metrics.py`: builds the per-run numbers above from those folders
  (`python trial_metrics.py runs cbase cnew`).
