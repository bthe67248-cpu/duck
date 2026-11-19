"""Experimental loop closure: synthetic geometry and integration, no accuracy claims."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from duck_nav import duckslam as ds
from duck_nav.pose_graph import PoseGraph, Keyframe, relative, compose, transform


def scan(points):
    return np.arctan2(points[:, 1], points[:, 0]), np.linalg.norm(points, axis=1), np.ones(len(points), bool)


def shape():
    return np.vstack((np.column_stack((np.ones(40), np.linspace(-1, 1, 40))),
                      np.column_stack((np.linspace(-1, 1, 40), np.ones(40)))))


def test_rigid_correction_rotates_later_translation_and_wraps_heading():
    out = ds.carry_pose_correction(np.array([2., 0., 3.]), np.array([1., 0., 0.]),
                                  np.array([1., 1., math.pi / 2]))
    assert out[:2] == pytest.approx([1, 2])
    assert -math.pi <= out[2] <= math.pi
    assert relative(np.zeros(3), compose(np.array([0, 0, 3.1]), [0, 0, .2]))[2] == pytest.approx(-2.983185307)


def test_registration_rejects_wall_and_recovers_corner():
    points = shape()
    correction = np.array([.07, -.03, .03])
    got = PoseGraph.register(points, transform(points, correction), np.zeros(3))
    assert got is not None
    assert np.linalg.norm(compose(got[0], correction)) < .03  # finite-sample ICP can stop between samples
    assert PoseGraph.register(points[:40], points[:40], np.zeros(3)) is None
    assert PoseGraph.register(points[:8], points[:8], np.zeros(3)) is None


def test_optimizer_anchors_first_pose_and_reduces_drift():
    graph = PoseGraph()
    truth = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [.1, 0, 0]], float)
    estimates = truth.copy(); estimates[:, 0] += np.arange(5) * .03
    graph.nodes = [Keyframe(i, p, (), np.zeros((0, 2)), i) for i, p in enumerate(estimates)]
    edges = [(i-1, i, relative(estimates[i-1], estimates[i]), np.array([.06, .06, .05])) for i in range(1, 5)]
    edges.append((0, 4, truth[4], np.array([.025, .025, .025])))
    optimized = graph.optimize(edges)
    assert optimized is not None
    assert optimized[0] == pytest.approx(estimates[0])
    assert np.linalg.norm(optimized[-1, :2] - truth[-1, :2]) < .03


def test_graph_rebuilds_every_raw_scan_and_invalidates_sweep(monkeypatch):
    slam = ds.DuckSLAM(); loc = ds.FrameLocalizer(slam, loop_closure=True, slip=False)
    raw = scan(shape()); first = np.zeros(3)
    loc.step(0, first, *raw, match=False)
    before = np.array([.3, 0., 0.]); after = np.array([.2, .1, .1])
    original_add = loc.graph.add

    def close(*args, **kwargs):
        original_add(*args, **kwargs)
        loc.graph.nodes[-1].pose = after.copy()
        loc.graph.revision += 1
        return True

    monkeypatch.setattr(loc.graph, 'add', close)
    accepted, corr = loc.step(1, before, *raw, match=False)
    expected = ds.DuckSLAM()
    for node in loc.graph.nodes:
        expected.pose = node.pose.copy(); expected.update_map(*node.scan)
    assert slam.logodds == pytest.approx(expected.logodds)
    assert before + corr == pytest.approx(after)
    assert not loc.sweep.frames
    assert loc.health.state(1) == 'weak'
    assert not accepted


def test_stationary_bootstrap_maps_new_head_views():
    slam = ds.DuckSLAM(); loc = ds.FrameLocalizer(slam, loop_closure=True, slip=False)
    loc.step(0, np.zeros(3), *scan(shape()[:40]), match=False)
    loc.step(.1, np.zeros(3), *scan(shape()[40:]), match=False)
    assert len(loc.graph.nodes) == 2
    assert np.count_nonzero(slam.logodds > .8) >= 70


def test_capacity_continues_mapping_without_unsafe_future_rebuild(monkeypatch):
    slam = ds.DuckSLAM(); loc = ds.FrameLocalizer(slam, loop_closure=True, graph_max_nodes=12, slip=False)
    raw = scan(shape())
    for i in range(12):
        loc.step(i, np.zeros(3), *raw, match=False)
    old_map = slam.logodds.copy(); version = slam.map_version
    monkeypatch.setattr(loc.graph, 'add', lambda *a: pytest.fail('exhausted graph must never add/rebuild'))
    loc.step(13, np.array([.4, 0, 0]), *raw, match=False)
    assert slam.map_version == version + 1
    assert loc.graph.reason == 'disabled_capacity'
    assert len(loc.graph.nodes) == 12
    assert not np.array_equal(slam.logodds, old_map)


def populated_graph():
    graph = PoseGraph()
    for i in range(9):
        graph.nodes.append(Keyframe(i, np.array([float(i)*2, 0, 0]), (), shape(), float(i)*2))
    return graph


def test_revisit_requires_spatial_and_correction_consistency(monkeypatch):
    graph = populated_graph(); raw = scan(shape())
    monkeypatch.setattr(graph, 'register', lambda *a: (np.zeros(3), 0.0))
    graph.add(30, np.array([.05, 0, 0]), raw, raw)
    assert graph.reason == 'awaiting_confirmation'
    graph.add(31, np.array([.05, 0, .3]), raw, raw)
    assert graph.reason == 'awaiting_confirmation'
    assert graph.revision == 0
    graph.add(32, np.array([.3, 0, .3]), raw, raw)
    assert graph.reason == 'awaiting_confirmation'  # inferred correction differs by 25 cm
    assert graph.revision == 0


def test_indistinguishable_competing_places_rejected(monkeypatch):
    graph = populated_graph(); raw = scan(shape())
    graph.nodes[1].pose = np.array([.4, 0, 0])
    # Add enough intervening frames to make both old places eligible.
    graph.nodes.append(Keyframe(9, np.array([4., 0, 0]), (), shape(), 9.))
    monkeypatch.setattr(graph, 'register', lambda *a: (np.zeros(3), 0.0))
    assert not graph.add(30, np.array([.2, 0, 0]), raw, raw)
    assert graph.reason == 'ambiguous_revisit'
    assert graph.revision == 0


def test_consistent_moved_revisit_adds_anchored_loop(monkeypatch):
    graph = populated_graph(); raw = scan(shape())
    def register(ref, points, initial):
        out = initial.copy(); out[0] -= .08
        return out, 0.0
    monkeypatch.setattr(graph, 'register', register)
    assert not graph.add(30, np.array([.1, 0, 0]), raw, raw)
    assert graph.add(31, np.array([.25, 0, 0]), raw, raw)
    assert graph.revision == 1 and graph.reason == 'loop_closed'
    assert graph.nodes[0].pose == pytest.approx(np.zeros(3))
    assert graph.nodes[-1].pose[0] == pytest.approx(.17, abs=.01)


def test_matching_disabled_and_fall_clear_pending(monkeypatch):
    slam = ds.DuckSLAM(); loc = ds.FrameLocalizer(slam, loop_closure=True, slip=False)
    monkeypatch.setattr(loc.graph, 'register', lambda *a: pytest.fail('matching disabled'))
    loc.step(1, np.zeros(3), *scan(shape()), match=False)
    assert loc.graph.reason == 'matching_disabled'
    loc.graph.pending = (0, 1, np.zeros(3), np.zeros(3))
    loc.reset_after_fall()
    assert loc.graph.pending is None


def test_graph_cannot_discard_an_untracked_initial_map():
    slam = ds.DuckSLAM(); slam.logodds[1, 1] = 2
    with pytest.raises(ValueError, match='empty initial map'):
        ds.FrameLocalizer(slam, loop_closure=True)


def test_real_registration_confirmation_and_optimizer_close_synthetic_revisit():
    # Complete graph path, without mocked registration or optimization.
    world = np.vstack((np.column_stack((np.ones(60), np.linspace(-1, 1, 60))),
                       np.column_stack((np.linspace(-1, 1, 60), np.ones(60)))))
    poses = [[0, 0, 0], [1, 0, 0], [2, 0, 0], [2, 1, 0], [2, 2, 0],
             [1, 2, 0], [0, 2, 0], [0, 1, 0], [0, .8, 0], [.1, 0, 0], [.25, 0, 0]]
    graph = PoseGraph()
    for i, pose in enumerate(poses):
        truth = np.array(pose, dtype=float)
        raw = scan(transform(world, relative(truth, np.zeros(3))))
        estimate = truth.copy(); estimate[0] += .008 * i
        closed = graph.add(i * 4., estimate, raw, raw)
        if i == 9:
            assert not closed and graph.reason == 'awaiting_confirmation'
    assert closed and graph.revision == 1
    assert np.linalg.norm(graph.nodes[-1].pose[:2] - truth[:2]) < .03
    assert np.linalg.norm(estimate[:2] - truth[:2]) > .07
    assert graph.nodes[0].pose == pytest.approx(np.zeros(3))
