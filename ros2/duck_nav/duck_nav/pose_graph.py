"""Small experimental SE(2) keyframe graph. No ground truth or ROS dependencies.

Frontend relative-pose edges are constraints, not calibrated odometry covariance.
Closure uses bounded, bidirectional ICP against old keyframes and requires a second
revisit observation. This is deliberately conservative; similar places can still alias.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])])


def relative(a, b):
    c, s = math.cos(a[2]), math.sin(a[2]); d = np.asarray(b[:2]) - a[:2]
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], wrap(b[2] - a[2])])


def carry_pose_correction(pose, before, after):
    """Apply after * inverse(before) to a later pose (including its translation)."""
    return compose(after, relative(before, pose))


def transform(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


def cloud(scan, resolution=0.04):
    a, r, h = scan
    good = np.asarray(h, bool) & np.isfinite(a) & np.isfinite(r) & (r > 0)
    p = np.column_stack((r[good] * np.cos(a[good]), r[good] * np.sin(a[good])))
    if len(p):
        _, idx = np.unique(np.floor(p / resolution), axis=0, return_index=True)
        p = p[np.sort(idx)]
    return p


@dataclass
class Keyframe:
    stamp: float
    pose: np.ndarray
    scan: tuple
    points: np.ndarray
    distance: float


class PoseGraph:
    def __init__(self, max_nodes=160):
        if isinstance(max_nodes, bool) or not isinstance(max_nodes, int) or max_nodes < 12:
            raise ValueError("max_nodes must be an integer >= 12")
        self.max_nodes = max_nodes
        self.nodes = []; self.edges = []; self.revision = 0
        self.reason = "initializing"; self.pending = None

    @property
    def full(self):
        return len(self.nodes) >= self.max_nodes

    @staticmethod
    def register(reference, points, initial):
        """Return a current->reference transform only with distributed reciprocal support."""
        if min(len(reference), len(points)) < 20:
            return None
        if min(np.linalg.eigvalsh(np.cov(p.T))[0] for p in (reference, points)) < 0.01:
            return None
        tree = cKDTree(reference); pose = initial.copy()
        for _ in range(20):
            q = transform(points, pose); dist, idx = tree.query(q)
            good = dist < 0.18
            if good.sum() < 20 or good.mean() < 0.8:
                return None
            src, dst = points[good], reference[idx[good]]
            u, _, vt = np.linalg.svd((src - src.mean(0)).T @ (dst - dst.mean(0)))
            rot = vt.T @ u.T
            if np.linalg.det(rot) < 0:
                vt[-1] *= -1; rot = vt.T @ u.T
            t = dst.mean(0) - rot @ src.mean(0)
            new = np.array([*t, math.atan2(rot[1, 0], rot[0, 0])])
            delta = relative(pose, new); pose = new
            if np.linalg.norm(delta) < 1e-5:
                break
        q = transform(points, pose); d, _ = tree.query(q)
        back, _ = cKDTree(q).query(reference)
        if (d < 0.06).mean() < 0.85 or (back < 0.06).mean() < 0.85:
            return None
        rmse = float(np.sqrt(np.mean(np.minimum(d, 0.18) ** 2)))
        innovation = relative(initial, pose)
        if rmse > 0.04 or np.linalg.norm(innovation[:2]) > 0.45 or abs(innovation[2]) > math.radians(15):
            return None
        return pose, rmse

    def add(self, stamp, pose, scan, closure_scan, allow_closure=True):
        """Add a mapped keyframe. At capacity retain all history and refuse additions."""
        if self.full:
            self.reason = "disabled_capacity"; return False
        distance = self.nodes[-1].distance + np.linalg.norm(pose[:2] - self.nodes[-1].pose[:2]) if self.nodes else 0.0
        node = Keyframe(stamp, pose.copy(), tuple(np.array(x, copy=True) for x in scan), cloud(closure_scan), distance)
        j = len(self.nodes)
        if j:
            self.edges.append((j - 1, j, relative(self.nodes[-1].pose, pose), np.array([0.06, 0.06, 0.05])))
        self.nodes.append(node)
        if not allow_closure:
            self.pending = None; self.reason = "matching_disabled"; return False
        candidates = []
        for i, old in enumerate(self.nodes[:-8]):
            initial = relative(old.pose, pose)
            if stamp - old.stamp < 20 or distance - old.distance < 2 or np.linalg.norm(initial[:2]) > 0.75 \
                    or abs(initial[2]) > math.radians(30):
                continue
            result = self.register(old.points, node.points, initial)
            if result is not None:
                candidates.append((result[1], i, result[0]))
        if not candidates:
            self.pending = None; self.reason = "no_verified_revisit"; return False
        best_error, i, measurement = min(candidates, key=lambda x: x[0])
        target = compose(self.nodes[i].pose, measurement)
        for error, other_i, other_measurement in candidates:
            alternative = compose(self.nodes[other_i].pose, other_measurement)
            delta = relative(target, alternative)
            if error <= best_error + 0.01 and (np.linalg.norm(delta[:2]) > 0.1 or abs(delta[2]) > math.radians(5)):
                self.pending = None; self.reason = "ambiguous_revisit"; return False
        # A separate, spatially moved keyframe must confirm the same old neighbourhood.
        correction = carry_pose_correction(np.zeros(3), pose, compose(self.nodes[i].pose, measurement))
        pending = self.pending; self.pending = (i, j, correction, pose.copy())
        consistent = pending is not None and abs(pending[0] - i) <= 2 and j - pending[1] <= 2
        if consistent:
            delta = relative(pending[2], correction)
            consistent = np.linalg.norm(delta[:2]) <= 0.08 and abs(delta[2]) <= math.radians(4) \
                and np.linalg.norm(pose[:2] - pending[3][:2]) >= 0.1
        if not consistent:
            self.reason = "awaiting_confirmation"; return False
        edge = (i, j, measurement, np.array([0.025, 0.025, 0.025]))
        optimized = self.optimize(self.edges + [edge])
        if optimized is None:
            self.reason = "optimization_rejected"; return False
        self.edges.append(edge)
        for item, updated in zip(self.nodes, optimized):
            item.pose = updated
        self.revision += 1; self.pending = None; self.reason = "loop_closed"
        return True

    def optimize(self, edges):
        """Fix node zero; reject failed solves and large/unresolved edge residuals."""
        old = np.array([n.pose for n in self.nodes]); n = len(old)

        def unpack(x):
            return np.vstack((old[0], x.reshape(n - 1, 3)))

        def residual(x):
            poses = unpack(x); out = []
            for i, j, measured, std in edges:
                e = relative(poses[i], poses[j]) - measured; e[2] = wrap(e[2])
                out.extend(e / std)
            return np.asarray(out)

        # Sparse finite differencing keeps this bounded graph usable without dense cubic work.
        from scipy.sparse import lil_matrix
        sparsity = lil_matrix((len(edges) * 3, (n - 1) * 3), dtype=int)
        for k, (i, j, _, _) in enumerate(edges):
            for index in (i, j):
                if index:
                    sparsity[3*k:3*k+3, 3*(index-1):3*index] = 1
        fit = least_squares(residual, old[1:].ravel(), jac_sparsity=sparsity.tocsr(),
                            loss="soft_l1", max_nfev=100)
        poses = unpack(fit.x)
        if not fit.success or not np.isfinite(poses).all() or np.max(np.abs(residual(fit.x))) > 4:
            return None
        if np.max(np.linalg.norm(poses[:, :2] - old[:, :2], axis=1)) > 0.6 \
                or np.max(np.abs(wrap(poses[:, 2] - old[:, 2]))) > math.radians(20):
            return None
        poses[:, 2] = wrap(poses[:, 2])
        return poses
