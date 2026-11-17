"""duckslam.py -- the DuckSLAM core (no ROS in here): predict, depth->scan, scan matching, grid map,
and the wavefront planner. Same algorithm as scripts/slam_nav.py, parameterised for the ROS nodes.
"""
import math
from collections import deque

import numpy as np
from scipy import ndimage

RES = 0.025                                    # m per cell
X0, Y0, NX, NY = -1.5, -2.75, 260, 220         # map covers x -1.5..5.0, y -2.75..2.75
W, H, FOVY = 320, 240, 60.0
MAX_R = 2.5
FOCAL = (H / 2) / math.tan(math.radians(FOVY) / 2)


def cell(x, y):
    return int((x - X0) / RES), int((y - Y0) / RES)


def pixel_to_level(u, v, z, R_bc, t_bc):
    """Pixel (u, v) at depth z -> point in the level body frame."""
    pc = np.array([(u + 0.5 - W / 2) * z / FOCAL, -(v + 0.5 - H / 2) * z / FOCAL, -z])
    return R_bc @ pc + t_bc


class DuckSLAM:
    def __init__(self, v_gain):
        self.pose = np.array([0.0, 0.0, 0.0])
        self.logodds = np.zeros((NX, NY), np.float32)
        self.v_gain = v_gain
        self.n_match = 0

    def predict(self, cmd_vx, gyro_z, dt):
        v = self.v_gain * cmd_vx if cmd_vx >= 0.25 else 0.0
        self.pose[2] += gyro_z * dt
        self.pose[0] += v * math.cos(self.pose[2]) * dt
        self.pose[1] += v * math.sin(self.pose[2]) * dt

    @staticmethod
    def scan_from_depth(depth, R_bc, t_bc, stride=2):
        d = depth[::stride, ::stride]
        u, v = np.meshgrid(np.arange(0, W, stride) + 0.5 - W / 2, np.arange(0, H, stride) + 0.5 - H / 2)
        ok = (d > 0.03) & (d < MAX_R)
        pc = np.stack([u * d / FOCAL, -v * d / FOCAL, -d], -1)[ok]
        pb = pc @ R_bc.T + t_bc
        rng = np.hypot(pb[:, 0], pb[:, 1]); bear = np.arctan2(pb[:, 1], pb[:, 0])
        obst = (pb[:, 2] > 0.03) & (pb[:, 2] < 0.30)
        floor = pb[:, 2] < 0.03
        bins = np.round(np.degrees(bear) / 2).astype(int)
        ang, r, hit = [], [], []
        for b in np.unique(bins):
            sel = bins == b
            o = sel & obst
            if o.sum() >= 3:
                ang.append(b); r.append(float(np.percentile(rng[o], 10))); hit.append(True)
            else:
                fl = sel & floor
                if fl.sum() >= 3:
                    ang.append(b); r.append(float(rng[fl].max())); hit.append(False)
        return np.radians(2.0 * np.array(ang)), np.array(r), np.array(hit, bool)

    def match(self, ang, r, hit):
        occ = (self.logodds > 0.8).astype(np.float32)
        if hit.sum() < 12 or occ.sum() < 80:
            return
        field = ndimage.gaussian_filter(occ, 1.6)
        px, py = r[hit] * np.cos(ang[hit]), r[hit] * np.sin(ang[hit])
        best, best_s = (0.0, 0.0, 0.0), -1.0
        for dth in np.radians(np.arange(-6, 6.1, 1.5)):
            c, s = math.cos(self.pose[2] + dth), math.sin(self.pose[2] + dth)
            wx, wy = c * px - s * py, s * px + c * py
            for dx in np.arange(-0.075, 0.076, 0.025):
                for dy in np.arange(-0.075, 0.076, 0.025):
                    ix = ((self.pose[0] + dx + wx - X0) / RES).astype(int)
                    iy = ((self.pose[1] + dy + wy - Y0) / RES).astype(int)
                    inside = (ix >= 0) & (ix < NX) & (iy >= 0) & (iy < NY)
                    score = field[ix[inside], iy[inside]].sum() / len(px)
                    score -= 0.02 * (abs(dx) + abs(dy)) / 0.075 + 0.02 * abs(dth) / math.radians(6)
                    if score > best_s:
                        best_s, best = score, (dx, dy, dth)
        if best_s > 0.12:
            self.pose += 0.6 * np.array(best)
            self.n_match += 1

    def update_map(self, ang, r, hit):
        x, y, th = self.pose
        steps = np.arange(RES, MAX_R, RES)
        for a, rr, h in zip(ang, r, hit):
            ca, sa = math.cos(th + a), math.sin(th + a)
            s = steps[steps < rr - RES]
            ix = ((x + ca * s - X0) / RES).astype(int); iy = ((y + sa * s - Y0) / RES).astype(int)
            k = (ix >= 0) & (ix < NX) & (iy >= 0) & (iy < NY)
            self.logodds[ix[k], iy[k]] -= 0.35
            if h:
                i, j = cell(x + ca * rr, y + sa * rr)
                if 0 <= i < NX and 0 <= j < NY:
                    self.logodds[i, j] += 1.2
        np.clip(self.logodds, -4, 6, out=self.logodds)

    def occupancy(self):
        """int8 grid in ROS convention: -1 unknown, 0 free, 100 occupied. Shape (NX, NY)."""
        g = np.full((NX, NY), -1, np.int8)
        g[self.logodds < -0.5] = 0
        g[self.logodds > 0.8] = 100
        return g


def plan(grid, pose, goal=None, goal_radius=0.34):
    """Wavefront plan on an occupancy grid (NX, NY; -1/0/100). Returns (path [(x, y)...] or None, mode)."""
    occ = grid == 100
    free = grid == 0
    blocked = ndimage.binary_dilation(occ, iterations=5)                 # 12.5 cm clearance
    trav = free & ~blocked
    ci, cj = cell(pose[0], pose[1])
    ii, jj = np.ogrid[:NX, :NY]
    trav[(ii - ci) ** 2 + (jj - cj) ** 2 <= 16] = True
    dist = np.full((NX, NY), -1, np.int32); parent = {}
    dist[ci, cj] = 0; q = deque([(ci, cj)])
    while q:
        i, j = q.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            a, b = i + di, j + dj
            if 0 <= a < NX and 0 <= b < NY and trav[a, b] and dist[a, b] < 0:
                dist[a, b] = dist[i, j] + 1; parent[(a, b)] = (i, j); q.append((a, b))
    reached = dist >= 0
    target, mode = None, "explore"
    if goal is not None:
        gi, gj = cell(*goal)
        d2 = (ii - gi) ** 2 + (jj - gj) ** 2
        cand = reached & (d2 <= (goal_radius / RES) ** 2)
        if cand.any():
            target, mode = np.unravel_index(np.argmin(np.where(cand, d2, 1e9)), d2.shape), "goal"
    if target is None:
        near_unknown = ndimage.binary_dilation(grid == -1, iterations=1)
        frontier = reached & near_unknown & (dist > 0.35 / RES)
        lab, n = ndimage.label(frontier)
        if n:
            sizes = ndimage.sum(frontier, lab, range(1, n + 1))
            big = np.isin(lab, np.nonzero(sizes >= 8)[0] + 1)
            if big.any():
                target = np.unravel_index(np.argmin(np.where(big, dist, 1e9)), dist.shape)
    if target is None:
        return None, "done"
    path = [tuple(int(v) for v in target)]
    while path[-1] != (ci, cj):
        path.append(parent[path[-1]])
    path.reverse()
    return [(X0 + (i + 0.5) * RES, Y0 + (j + 0.5) * RES) for i, j in path], mode
