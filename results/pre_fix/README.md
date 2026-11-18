# PRE-FIX results (superseded)

These logs, plots and GIFs were produced by the model **before** the handoff changes (tag `pre-handoff-baseline`):
old lip-style mouth, upstream jaw/shell visuals and collision meshes, no rear-shield collision, `PRINT_FILL = 0.9`
mass heuristic (head inertia far too small), and the old, weaker pass criteria (final uprightness / height only,
"got up" claimed even when the push never caused a fall).

Keep them for comparison only. The regenerated results, with the new criteria, are in `../post_fix/`.
`../post_fix/ab_motions_prefix_model_new_criteria.log` re-runs the pre-fix model through the NEW test criteria.
