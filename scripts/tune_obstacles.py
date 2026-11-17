r"""tune_obstacles.py -- sweep walking speed and servo stiffness over the plank course.

Usage:  .venv\Scripts\python.exe scripts\tune_obstacles.py
Prints, for each (speed command, kp scale), how many planks of 3/5/8/10/12 mm were cleared.
kp scale multiplies the servo position gain (1.0 = upstream XL330 model).
"""
import sys, types
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import motions

heights = [3, 5, 8, 10, 12]
print("speed  kp_scale | result")
for speed in (0.3, 0.4, 0.5, 0.6):
    for kp in (1.0, 1.5, 2.0):
        a = types.SimpleNamespace(kp_scale=kp, gif=False, view=False, obstacle_speed=speed, heights=heights)
        ok, note = motions.m_obstacles(a)
        print(f"{speed:5.2f} {kp:8.1f} | {'PASS' if ok else 'fail'} {note}", flush=True)
