"""Training environment and policy-export contract (training/), and the chamfered body proxies (make_duck)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "training"))
GC = ROOT / "model" / "duck_v4_gc_scene.xml"
needs_model = pytest.mark.skipif(not GC.exists(), reason="generated model missing (scripts/make_duck.py --tag gc)")


def test_chamfered_box_covers_its_fillets():
    trimesh = pytest.importorskip("trimesh")
    src = (ROOT / "scripts" / "make_duck.py").read_text()
    ns = {}
    exec(src[src.index("def chamfered_box_points"):src.index("def body_proxies")], {"np": np}, ns)
    lo, hi, r = [0.0, 0.0, 0.0], [40.0, 30.0, 20.0], 5.0
    hull = trimesh.convex.convex_hull(ns["chamfered_box_points"](lo, hi, r))
    assert np.allclose(hull.bounds, [lo, hi])
    # points on every fillet arc of a rounded box with radius r must be inside or on the hull
    pts = []
    for a in np.linspace(0, math.pi / 2, 7):
        off = r - r * math.cos(a), r - r * math.sin(a)
        pts += [(20.0, off[0], off[1]), (off[0], 15.0, off[1]), (off[0], off[1], 10.0)]
    d = trimesh.proximity.signed_distance(hull, np.array(pts))
    assert np.all(d > -1e-6)                     # trimesh: positive inside


@needs_model
def test_env_contract_and_randomization_does_not_accumulate():
    from duck_env import ACT_DIM, OBS_DIM, DuckEnv
    e = DuckEnv(xml=GC, seed=0)
    o = e.reset()
    assert o.shape == (OBS_DIM,) and o.dtype == np.float32 and ACT_DIM == 14
    assert np.allclose(o[48:61], e.cmd)          # the 13-D command block ends the observation
    m0 = e.d0["mass"].copy()
    for _ in range(5):
        e.reset()
    s = e.m.body_mass[1:] / m0[1:]
    assert np.all((s > 0.89) & (s < 1.11))       # re-applied from the defaults, never compounded
    o, r, done, fell, info = e.step(np.zeros(ACT_DIM))
    assert np.isfinite(r) and o.shape == (OBS_DIM,)


@needs_model
def test_lying_still_earns_nothing():
    from duck_env import DuckEnv
    e = DuckEnv(xml=GC, seed=3, cfg=dict(recover_prob=1.0))
    e.reset()
    assert e.recovering
    total = sum(e.step(np.zeros(14))[1] for _ in range(100))
    assert total < 0.5                           # task rewards are gated on being upright


def test_onnx_export_roundtrip_and_warm_start(tmp_path):
    torch = pytest.importorskip("torch"); pytest.importorskip("onnx"); ort = pytest.importorskip("onnxruntime")
    from train_ppo import ActorCritic, Exported, export_onnx, warm_start_from_onnx
    torch.manual_seed(0)
    ac = ActorCritic()
    ac.obs_norm.update(torch.randn(256, 61) * 2 + 1)
    with torch.no_grad():
        ac.actor[-1].weight.normal_(0, 0.1)
    path = tmp_path / "p.onnx"
    export_onnx(ac, path, dict(test=1))
    sess = ort.InferenceSession(str(path))
    assert [i.name for i in sess.get_inputs()] == ["obs"]
    x = np.random.default_rng(0).normal(size=(4, 61)).astype(np.float32)
    with torch.no_grad():
        ref = Exported(ac)(torch.as_tensor(x)).numpy()
    assert np.allclose(sess.run(None, {"obs": x})[0], ref, atol=1e-5)
    # an exported policy warm-starts a fresh actor to the same outputs (normalizer folded back in)
    ac2 = ActorCritic()
    assert warm_start_from_onnx(ac2, path) == 4
    with torch.no_grad():
        assert np.allclose(Exported(ac2)(torch.as_tensor(x)).numpy(), ref, atol=1e-3)
