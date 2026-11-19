"""Offline helpers around duck_nav.flowodom (the floor-flow estimator shared with slam_node).

`compute_flow_pairs` runs the estimator over every consecutive image pair of a recording, for tools/slam_replay.py.
`estimate_pair` (flowodom.track_image, images first), `estimate`, the geometry helpers and the trackers are re-exported from the
package module, so the replay, the analysis tool (floor_flow.py) and the live node use the same code.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from duck_nav.flowodom import (METHODS, estimate, floor_points, project, rigid_fit, ransac_rigid,  # noqa: E402,F401
                               rectify, track_image, track_rect)



def estimate_pair(img0, img1, pose0, pose1, K, rng, guess=None):
    """Image tracker with the replay's argument order (images first): flowodom.track_image."""
    return track_image(img0, pose0, img1, pose1, K, rng, guess=guess)


HELPERS = "duck_nav.flowodom"


def compute_flow_pairs(imgs, poses, K, seed=0, max_t=None, method="image"):
    """Estimates for every consecutive stamp pair 0.1 s < dt < 0.3 s: [(s0, s1, estimate_or_None), ...].
    imgs / poses: {stamp: gray image / (R, t)}. No ground truth involved; falls are handled by the replay's own
    tilt gate (it discards pairs overlapping a non-upright interval)."""
    rng = np.random.default_rng(seed)
    stamps = sorted(set(imgs) & set(poses))
    out = []
    for s0, s1 in zip(stamps[:-1], stamps[1:]):
        if not 0.1 < s1 - s0 < 0.3 or (max_t is not None and s1 > max_t):
            continue
        out.append((s0, s1, estimate(imgs[s0], imgs[s1], poses[s0], poses[s1], K, rng, method)))
    return out
