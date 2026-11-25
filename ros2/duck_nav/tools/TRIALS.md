# Repeatable navigation trials and replay

These tools were written in this repository as replacements. The original Windows
`C:\dev\duck_review_zip\job2.sh` and `patch_ort.py` were unavailable and were not
imported. No ONNX Runtime patch is applied by this runner.

## Prepare one immutable workspace

On the ROS 2 Jazzy host, regenerate/validate the model and run the repository's
`ros2/setup_nav.sh`, then source `/opt/ros/jazzy/setup.bash` and
`~/duck_ws/install/setup.bash`. Rebuild after these changes: the runner requires the
new launch overrides and score node's final `run_complete.json` marker. Do not
rebuild or mutate the installed workspace while a matrix runs. Keep a clean git
checkout when collecting comparison evidence; source hashes also identify dirty
code. Choose unused domain IDs and stop other publishers on those domains.

The matrix records source revision, dirty paths, source/model/policy hashes,
installed package hashes, ROS workspace paths and runtime versions. Recursive
installed MJCF assets are hashed where accessible. External include dependencies
of custom layout XMLs are **not** discovered automatically; retain and hash the
complete custom layout bundle separately. A manifest describes the source
checkout and installed package independently: compare them before interpreting
results if the installed workspace came from a different checkout.

## Plan and execute

From the repository root:

```bash
python3 ros2/duck_nav/tools/trial_matrix.py --out /tmp/duck_plan --seeds 0,1,2,3,4,5,6,7,8,9
python3 ros2/duck_nav/tools/trial_matrix.py --out /tmp/duck_runs --seeds 0,1,2,3,4,5,6,7,8,9 --rt-factors 1,2 --duration 600 --workers 1 --execute
```

Planning requires only Python and never starts ROS. Output directories must be
new. Execution starts a recorder before each launch, waits for storage creation,
and records realistic ToF/IMU/commands/truth, SLAM health/match/map revision and
navigation outcomes. It waits for a validated score result and final artifact
marker, then stops only its own process groups so the bag closes normally. Known
node crash messages fail the trial. Wall timeouts and recorder failures produce
failed trial manifests and logs; a timeout exploration outcome can still be a
completed trial execution. `completed` does not mean exploration succeeded.

`--workers 2` runs two isolated ROS domains (default 50 and 51), each with its own
output directory, launch process group and recorder. Establish the serial
baseline first. Domains isolate ROS traffic; CPU/GPU/memory remain shared, so
parallel runs can change callback timing. `--wall-timeout` overrides the generous
per-trial wall deadline. Ctrl-C terminates active process groups and cancels
queued trials. `--loop-closure` opts into the experimental graph; default is off.

Each layout/seed gets paired `rt_factor=1` and `2` entries and actual metric deltas
in `matrix.json` after execution. Equal seeds do not guarantee equal navigation:
ROS scheduling and rendering affect callbacks and the closed-loop path. Check
exploration/beacon outcomes, stamp-matched SLAM samples, coverage, falls, logs and
bags before concluding either rate is usable. These tools impose no unvalidated
pass threshold.

Supply `--layouts layouts.json` to cross product layouts too. Names must be safe
filenames. Nondefault layouts must provide all five overrides so scoring uses the
correct room and beacon. Asset paths resolve relative to the layouts manifest:

```json
{
  "default": {},
  "alternate": {
    "mjcf": "layouts/alternate/world.xml",
    "world_json": "layouts/alternate/world.json",
    "beacon_x": 3.0,
    "beacon_y": 1.0,
    "room": [-0.67, 4.47, -1.97, 1.97]
  }
}
```

The alternate files above are illustrative; no alternate layout is bundled.
MJCF geometry, world JSON and scoring bounds must describe the same scene.

## Inventory, trim and replay bags

```bash
python3 ros2/duck_nav/tools/bag_suite.py inventory /path/to/bag --output inventory.json
python3 ros2/duck_nav/tools/bag_suite.py trim /path/to/long_bag /path/to/new_600s_bag --duration 600
python3 ros2/duck_nav/tools/slam_replay.py /path/to/bag --duration 600 --output replay.json
python3 ros2/duck_nav/tools/bag_suite.py replay-set bags.json --out /tmp/replays --duration 600
```

SQLite inventory works without ROS, counts populated required topics, records
file hashes and reports receive-time spans. Its simulated duration is explicitly
unknown until `/clock` is decoded. Missing/empty/incomplete bags return nonzero.
Trim/replay require ROS 2 Python packages. Trim accepts SQLite or MCAP through
ROS readers and writes a new SQLite bag; originals are untouched. It uses elapsed
`/clock` time, so 600 simulated seconds at rate 2 occupy about 300 receive seconds.
Clock resets/out-of-order clocks are rejected. A shorter source is preserved and
its actual available clock span is reported rather than padded.

The trim boundary is the receive timestamp of a `/clock` sample, with clock
cadence uncertainty and possible delayed acquisitions near the cutoff. It does
not inspect every topic's acquisition stamp. Use replay's acquisition-time
`--duration` to limit the scored interval too. Nonzero `--start` loses earlier map
and transient-local state; use first-600-second trims for comparable warmup.

Replay-set manifest format:

```json
{"bags": [{"name": "seed2", "path": "bags/seed2"}]}
```

Bag paths resolve relative to the manifest. Missing bags are retained as
`missing` and make the command fail. A replay is completed only with scored
samples and usable frames; stderr/logs are retained for failures. Duplicate or
unsafe names are rejected.

Replay now defaults to the same `FrameLocalizer` policy as the realistic launch:
min hits 8, 1-second sweep, gain 0.6, fresh check false, slip enabled and map motion
thresholds zero. It suspends translation and scan/map updates above 35-degree
filtered tilt, resets localization after a fall, and waits below 15 degrees for
more than one second before recovery. Heading continues through recovery.
SE(2) corrections carry later poses rigidly, including graph rotations. Metrics
include recovery entries, graph revision/node/edge counts and match reasons.

Use `--config '{"loc":{"gain":0.6}}'` for shared policy overrides;
`--loop-closure` enables the graph. Top-level `matcher`, `sweep`, `min_hits`, and
`fresh` are supported. Historical `map`, `kw`, `slip`, and `slip_kw` options require
`--legacy`; unknown top-level options fail. Legacy mode retains the earlier
matcher experiment policy. It is not the current live default.

The second-peak options go through `loc` too, e.g.
`--config '{"loc":{"reloc_peak_margin":1.0}}'` or `{"loc":{"match_kw":{"peak_margin":1.0},"map_min_gap":2.0}}`;
`--dump` then also writes each match's `gap`. Replays are chaotic (one changed frame can move a run's mean error by
0.5 m), so compare policies over many bags, not one.

`tools/trial_metrics.py <bag>... --world <world.json>` scores live runs from their bags: SLAM error split near walls
(true clearance below `--near`, default 0.25 m) and in the open, time above 0.5 m error, share of time
`/slam/state` was "ok", and the number and length of not-ok spells (recovery time).

Offline replay holds the recorded path fixed and sorts events by acquisition
time. It does not reproduce ROS queue loss, callback timing or publication
latency, and therefore cannot establish live navigation or rate-2 safety.
Custom live parameters must be supplied explicitly when replaying. Independent
live trials at both rates are required for the rate comparison.

## Evidence available in this checkout

`results/post_merge/trial_tooling/plan_10seed/matrix.json` is an unexecuted plan
for seeds 0–9 at rates 1 and 2 on the default layout, with every entry `planned`.
The illustrative missing-bag manifest and its report explicitly show that no
historical bag files were supplied. No ROS installation or actual navigation
bags were available in the implementation environment. Unit tests cover matrix
construction, validation, process-start failure evidence, rate deltas, split
SQLite inventory, clock boundaries/resets, missing replay sets, dry-run CLI and
a replay pipeline using ROS serialization shims plus the real SLAM code.
