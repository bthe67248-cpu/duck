"""Shared fixtures: generated models built into a temporary root from the pinned upstream microduck_rl assets.

The committed scene XML files reference meshes that only exist after scripts/prep_meshes.py and make_duck.py have
run, so a test must never load model/*.xml from a fresh checkout directly. These fixtures build the models into
their own temporary directory (DUCKLING_ROOT), the way CI's clean checkout needs it. They skip only when the pinned
upstream model is absent (CI fetches it; see .github/workflows/software-checks.yml).
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = Path(os.environ.get("DUCKLING_UPSTREAM", ROOT / "microduck_rl/src/mjlab_microduck/robot/microduck"))


def make_model_root(tmp_dir):
    """A DUCKLING_ROOT with the design inputs copied in, and a runner for the repository's scripts against it."""
    if not (UPSTREAM / "robot_walk.xml").exists():
        pytest.skip("clone the pinned microduck_rl model to run model integration tests")
    root = Path(tmp_dir)
    # Copy rather than symlink: Windows symlinks can require elevated rights.
    shutil.copytree(ROOT / "design/v4", root / "design/v4",
                    ignore=shutil.ignore_patterns("preview", "sim_runs", "print", "*.py"))
    (root / "scripts").mkdir()
    (root / "scripts/collision_proxies.json").write_bytes((ROOT / "scripts/collision_proxies.json").read_bytes())
    env = {**os.environ, "DUCKLING_ROOT": str(root), "DUCKLING_UPSTREAM": str(UPSTREAM)}

    def run(script, *args, check=True):
        return subprocess.run([sys.executable, str(ROOT / "scripts" / script), *map(str, args)],
                              env=env, text=True, capture_output=True, check=check)
    return root, run


@pytest.fixture(scope="session")
def gc_scene(tmp_path_factory):
    """Path of a freshly generated ground-contact scene (the model the walking policy is trained on)."""
    pytest.importorskip("mujoco")
    root, run = make_model_root(tmp_path_factory.mktemp("gc_model"))
    run("prep_meshes.py")
    run("make_duck.py", "--base", "robot_groundcontact.xml", "--tag", "gc")
    return root / "model" / "duck_v4_gc_scene.xml"
