r"""sync_ros2_assets.py -- copy the regenerated model files into the ROS 2 packages (what ros2/setup_nav.sh does).

Usage:  .venv\Scripts\python.exe scripts\sync_ros2_assets.py
Copies model/duck_v4_gc.xml, duck_v4_gc_world.xml, world.json and all model/assets/*.stl into
ros2/duck_nav/mjcf/ and (xml + assets) ros2/duck_description/mjcf/, plus model/duck_v4.xml and duck_v4_scene.xml
into duck_description/mjcf. Stale head meshes (files in the package assets that model/assets no longer has) are
removed from the package copies only. The copies are git-ignored; this only refreshes them.
"""
import os, shutil
from pathlib import Path

ROOT = Path(os.environ.get("DUCKLING_ROOT", Path(__file__).resolve().parents[1]))
M = ROOT / "model"
jobs = {ROOT / "ros2" / "duck_nav" / "mjcf": ["duck_v4_gc.xml", "duck_v4_gc_world.xml", "world.json"],
        ROOT / "ros2" / "duck_description" / "mjcf": ["duck_v4_gc.xml", "duck_v4_gc_world.xml", "world.json",
                                                      "duck_v4.xml", "duck_v4_scene.xml"]}
for dst, files in jobs.items():
    (dst / "assets").mkdir(parents=True, exist_ok=True)
    for f in files:
        shutil.copy2(M / f, dst / f)
    src_assets = {p.name for p in (M / "assets").glob("*.stl")}
    for p in (M / "assets").glob("*.stl"):
        shutil.copy2(p, dst / "assets" / p.name)
    for p in (dst / "assets").glob("*.stl"):
        if p.name not in src_assets:
            p.unlink()
    print("synced", dst)
