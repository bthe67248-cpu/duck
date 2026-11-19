# Independent review of post-merge worker changes

17 November 2026 (America/Los_Angeles). Reviewed the shared worktree while changes were in progress. This is a source/offline review, not a ROS or hardware validation.

## Findings addressed during review

- Trial recorder topic `/slam/status` did not exist: changed to `/slam/state`, with `/slam/match`, `/slam/map_revision` and `/duck/mode` included for diagnosing recovery and closures.
- Trial runner initially accepted any JSON object as completed evidence: it now validates realistic profile, outcome, finite metrics and positive stamp-matched SLAM samples, and checks recorder health at collection. Exploration success remains a separate score field.
- Replay initially used `fresh=True` and default map motion thresholds unlike the live node: it now uses `fresh=False` and explicit zero map motion/turn thresholds matching node construction.
- Trial provenance initially omitted default/source/installed code identities: it now records repository revision, worktree state, module/launch/model/policy fingerprints, installed package identity and runtime versions. Nested mesh/include coverage was raised with the owner for completion.
- Graph confirmation initially relied only on consecutive keyframe indices: latest code requires at least 0.1 m spatial displacement and consistent inferred correction; competing geometrically similar places can be rejected. Rotation-only second-observation smoke check does not close a loop.
- Two intermediate graph-test failures were reported to the owner (corner ICP tolerance and consistency fixture interference from a competing candidate). Latest focused rerun passes.

## Independent checks

`python -m pytest ros2/duck_nav/test/test_pose_graph.py ros2/duck_nav/test/test_nav_safety.py -q`: **17 passed in 0.65 s** after fixes. Coverage includes rigid correction transport, registration degeneracy, optimization anchoring, map rebuild/sweep invalidation, bootstrap views, graph capacity handling, confirmation consistency, ambiguity rejection, and navigation safety.

A direct tooling smoke check rejected `{}`, incomplete realistic metrics and wrong-profile results, accepted the retained realistic Seed 2 result, produced six planned seed/rate combinations, and confirmed the recorded localization topic is `/slam/state`.

Navigation revision integration clears cached path/goal/progress, gates movement on corrected map and pose timestamps, rejects old goals, and discards a plan computed across a revision. No remaining blocking issue was found in the reviewed opt-in graph/navigation integration. This does not establish real ROS callback behavior or closed-loop navigation performance.

## Limits

No ROS installation is present, so process isolation, rosbag recording readiness/finalization, actual launch parameter transport, node publication ordering, timeout cancellation, and closed-loop graph behavior were not exercised here. Conservative geometric tests cannot rule out physically identical places; retain experimental opt-in status until recorded multi-layout, multi-seed trials establish benefit and false-closure behavior. Persistent physical falls remain separate from localization recovery (see `seed2_hardware_review.md`).
