"""design/v4/solid.py: containment and penetration depth used by the V4 fit check (no model assets needed)."""
import sys
from pathlib import Path

import numpy as np
import pytest

trimesh = pytest.importorskip("trimesh")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "design" / "v4"))
from solid import Solid, box_mesh, mutual_depth  # noqa: E402

R = 0.02


@pytest.fixture(scope="module")
def sphere():
    s = trimesh.creation.icosphere(5, R)
    return Solid(s.vertices, s.faces)


def test_containment_matches_an_analytic_sphere(sphere):
    X = np.random.default_rng(2).uniform(-1.25 * R, 1.25 * R, (40000, 3))
    r = np.linalg.norm(X, axis=1)
    wrong = sphere.inside(X) != (r < R)
    # only points within the column spacing / tessellation error of the surface may disagree
    assert np.all(np.abs(r[wrong] - R) < 3e-4)
    assert wrong.mean() < 0.005


def test_depth_is_the_distance_to_the_surface_with_no_band(sphere):
    X = np.array([[0, 0, R - d] for d in (0.001, 0.003, 0.008, 0.015)])
    np.testing.assert_allclose(sphere.depth(X), [0.001, 0.003, 0.008, 0.015], atol=2.5e-4)
    # the previous estimate only saw 2.5 mm below the surface: a point 3 mm inside reported 0
    assert sphere.band_depth(X[1:2])[0] == 0.0
    assert sphere.depth(np.array([[0, 0, R + 0.001], [0.05, 0, 0]])).max() == 0.0


def test_a_torus_hole_is_outside_and_its_tube_inside():
    t = trimesh.creation.torus(0.03, 0.01)
    S = Solid(t.vertices, t.faces)
    assert not S.inside([[0, 0, 0]])[0]
    assert S.inside([[0.03, 0, 0]])[0]
    assert S.depth([[0.03, 0, 0]])[0] == pytest.approx(0.01, abs=3e-4)


def test_inside_out_meshes_still_contain_their_interior():
    V, F = box_mesh([0, 0, 0], [0.02, 0.01, 0.03])
    S = Solid(V, F[:, ::-1])
    assert S.volume < 0
    assert S.inside([[0.01, 0.005, 0.015]])[0] and not S.inside([[0.03, 0.005, 0.015]])[0]


def test_mutual_depth_finds_a_thin_wall_through_a_part():
    """A 2 mm wall pushed through a 40 mm block: the block's surface samples are at most 1 mm inside the wall (all
    the one-directional check can see), but the wall's samples reach 19 mm into the block."""
    block = Solid(*box_mesh([0, 0, 0], [0.04, 0.04, 0.04]), n_samples=20000)
    wall = Solid(*box_mesh([0.019, -0.01, -0.01], [0.021, 0.05, 0.05]), n_samples=20000)
    assert wall.depth(block.P).max() == pytest.approx(0.001, abs=3e-4)
    assert wall.band_depth(block.P).max() <= 0.0011
    assert mutual_depth(block, block.P, wall, wall.P) == pytest.approx(0.019, abs=3e-4)
    inner = Solid(*box_mesh([0.005, 0.005, 0.005], [0.035, 0.035, 0.035]))   # entirely inside: 5 mm from every face
    assert mutual_depth(block, block.P, inner, inner.P) == pytest.approx(0.005, abs=3e-4)
    apart = Solid(*box_mesh([0.05, 0, 0], [0.06, 0.01, 0.01]))
    assert mutual_depth(block, block.P, apart, apart.P) == 0.0
