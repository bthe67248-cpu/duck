Unfinished, untested: a word-command layer for the duck (command_node.py + its test).
Not installed or launched. The simulator (world_sim.py) already accepts /duck/behavior and /duck/pose_cmd,
and nav_node.py accepts /nav/enable, which this node would publish to.
To finish later: move command_node.py back into duck_nav/duck_nav/, add the two entry points to setup.py,
add the node to the launch file, rebuild, then run run_cmd_test.sh.
