r"""slam_nav.py -- camera + depth SLAM and navigation for the Duckling V3 in MuJoCo.

Usage:  .venv\Scripts\python.exe scripts\slam_nav.py [--view] [--gif] [--time 240] [--seed 0]

The duck starts in a room it has never seen and must reach a magenta beacon hidden behind walls.
Sensors used by the algorithm:
  - head camera RGB (160x120)     -> finds the beacon by colour
  - head camera depth (160x120)   -> obstacle points, turned into a 2-D scan
  - gyro z                        -> heading prediction
  - servo encoders + IMU attitude -> where the camera is relative to a level body frame
Position and heading come ONLY from the SLAM below; MuJoCo's true pose is used for scoring.

Algorithm ("DuckSLAM", written for this project):
  1. Predict   pose += calibrated walking speed along heading, heading += gyro z   (50 Hz)
  2. Scan      depth image -> 3-D points -> keep 3..30 cm above the floor -> nearest hit per 2 deg
  3. Match     search a small window of (dx, dy, dheading) around the prediction for the pose
               whose scan endpoints land best on the blurred occupancy map; move towards it
  4. Map       log-odds occupancy grid, 2.5 cm cells: free along each ray, occupied at hits
  5. Goal      magenta pixels in RGB + their depth -> beacon position in the map (running mean)
  6. Plan      breadth-first wavefront over inflated free space to the beacon if known,
               else to the nearest frontier (free cell next to unknown)                (1 Hz)
  7. Drive     pure pursuit of the path -> twist command to the walking policy; the head sweeps
               left-right so a 75 deg camera covers about 190 deg
"""
import argparse, math, sys
from collections import deque
from pathlib import Path
import mujoco, numpy as np
from scipy import ndimage
sys.path.insert(0, str(Path(__file__).parent))
import motions

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results" / "slam"
RES = 0.025                                   # m per cell
X0, Y0, NX, NY = -1.0, -2.0, 220, 160         # map covers x -1..4.5, y -2..2
W, H, FOVY = 160, 120, 60.0
MAX_R = 2.5                                   # m, depth beyond this is ignored
GOAL = (3.3, 1.2)


def make_room():
    """A 4.2 x 3.2 m room with two partition walls (an S-shaped route) and three boxes."""
    txt = motions.SCENE.read_text()
    g = '<geom name="{n}" type="box" size="{sx} {sy} {sz}" pos="{x} {y} {sz}" rgba="{c}"/>\n'
    walls = [("w_south", 1.6, -1.6, 2.2, 0.03, "0.75 0.75 0.7 1"), ("w_north", 1.6, 1.6, 2.2, 0.03, "0.75 0.75 0.7 1"),
             ("w_west", -0.6, 0.0, 0.03, 1.6, "0.7 0.75 0.75 1"), ("w_east", 3.8, 0.0, 0.03, 1.6, "0.7 0.75 0.75 1"),
             ("p1", 0.9, 0.5, 0.03, 1.1, "0.3 0.6 0.4 1"),          # partition from the north wall, gap at the south
             ("p2", 2.3, -0.5, 0.03, 1.1, "0.3 0.4 0.7 1")]         # partition from the south wall, gap at the north
    out = "".join(g.format(n=n, x=x, y=y, sx=sx, sy=sy, sz=0.15, c=c) for n, x, y, sx, sy, c in walls)
    boxes = [("b1", 0.2, -1.0, 0.10, 0.10, 0.06, "0.9 0.6 0.1 1"), ("b2", 1.6, 0.9, 0.12, 0.08, 0.05, "0.2 0.7 0.8 1"),
             ("b3", 3.0, -0.9, 0.10, 0.12, 0.08, "0.8 0.8 0.2 1")]
    out += "".join(g.format(n=n, x=x, y=y, sx=sx, sy=sy, sz=sz, c=c) for n, x, y, sx, sy, sz, c in boxes)
    out += (f'<geom name="beacon" type="cylinder" size="0.04 0.14" pos="{GOAL[0]} {GOAL[1]} 0.14" '
            f'material="beacon_mat"/>\n')
    txt = txt.replace("</worldbody>", out + "</worldbody>", 1)
    txt = txt.replace("</asset>", '<material name="beacon_mat" rgba="1 0 1 1" emission="0.8"/>\n</asset>', 1)
    path = motions.SCENE.parent / "duck_v4_gc_room.xml"
    path.write_text(txt)
    return path


def cell(x, y):
    return int((x - X0) / RES), int((y - Y0) / RES)


class DuckSLAM:
    def __init__(self, v_gain):
        self.pose = np.array([0.0, 0.0, 0.0])          # x, y, heading -- the SLAM estimate
        self.logodds = np.zeros((NX, NY), np.float32)
        self.v_gain = v_gain
        self.goal, self.goal_n = None, 0
        self.match_shift = []

    # 1. predict -------------------------------------------------------------------------
    def predict(self, cmd_vx, gyro_z, dt):
        v = self.v_gain * cmd_vx if cmd_vx >= 0.25 else 0.0
        self.pose[2] += gyro_z * dt
        self.pose[0] += v * math.cos(self.pose[2]) * dt
        self.pose[1] += v * math.sin(self.pose[2]) * dt

    # 2. depth image -> scan in the level body frame --------------------------------------
    @staticmethod
    def scan_from_depth(depth, R_bc, t_bc):
        """depth (H,W) metres; R_bc,t_bc: camera pose in the level body frame (z = height above floor)."""
        f = (H / 2) / math.tan(math.radians(FOVY) / 2)
        u, v = np.meshgrid(np.arange(W) + 0.5 - W / 2, np.arange(H) + 0.5 - H / 2)
        z = depth
        ok = (z > 0.03) & (z < MAX_R)
        pc = np.stack([u * z / f, -v * z / f, -z], -1)[ok]              # camera frame: x right, y up, -z forward
        pb = pc @ R_bc.T + t_bc
        rng = np.hypot(pb[:, 0], pb[:, 1]); bear = np.arctan2(pb[:, 1], pb[:, 0])
        obst = (pb[:, 2] > 0.03) & (pb[:, 2] < 0.30)
        bins = np.round(np.degrees(bear) / 2).astype(int)
        rays = {}
        for b in np.unique(bins):
            sel = bins == b
            o = sel & obst
            if o.sum() >= 3:
                rays[b] = (float(np.percentile(rng[o], 10)), True)       # nearest obstacle in this direction
            else:
                fl = sel & (pb[:, 2] < 0.03)
                if fl.sum() >= 3:
                    rays[b] = (float(rng[fl].max()), False)              # only floor seen: free that far
        ang = np.radians(2.0 * np.array(sorted(rays)))
        r = np.array([rays[b][0] for b in sorted(rays)]); hit = np.array([rays[b][1] for b in sorted(rays)], bool)
        return ang, r, hit, pc, pb

    # 3. scan-to-map matching ---------------------------------------------------------------
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
                    score -= 0.02 * (abs(dx) + abs(dy)) / 0.075 + 0.02 * abs(dth) / math.radians(6)  # prefer the prediction
                    if score > best_s:
                        best_s, best = score, (dx, dy, dth)
        if best_s > 0.12:                                               # enough of the scan agrees with the map
            self.pose += 0.6 * np.array(best)                           # move part of the way (smooths jitter)
            self.match_shift.append(math.hypot(best[0], best[1]))

    # 4. map update -----------------------------------------------------------------------------
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

    # 5. beacon from RGB + depth ------------------------------------------------------------------
    def see_goal(self, rgb, depth, R_bc, t_bc):
        mask = (rgb[..., 0] > 150) & (rgb[..., 2] > 150) & (rgb[..., 1] < 100) & (depth < MAX_R)
        if mask.sum() < 10:
            return
        f = (H / 2) / math.tan(math.radians(FOVY) / 2)
        vv, uu = np.nonzero(mask); z = np.median(depth[mask])
        pc = np.array([(np.median(uu) + 0.5 - W / 2) * z / f, -(np.median(vv) + 0.5 - H / 2) * z / f, -z])
        pb = R_bc @ pc + t_bc
        pb[:2] *= (np.hypot(*pb[:2]) + 0.04) / max(np.hypot(*pb[:2]), 1e-6)   # surface -> centre of the 4 cm cylinder
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        g = np.array([self.pose[0] + c * pb[0] - s * pb[1], self.pose[1] + s * pb[0] + c * pb[1]])
        self.goal_n += 1
        self.goal = g if self.goal is None else self.goal + (g - self.goal) / min(self.goal_n, 20)

    # 6. planning ---------------------------------------------------------------------------------
    def plan(self):
        occ = self.logodds > 0.8
        free = self.logodds < -0.5
        blocked = ndimage.binary_dilation(occ, iterations=5)            # 12.5 cm clearance around obstacles
        trav = free & ~blocked
        ci, cj = cell(self.pose[0], self.pose[1])
        ii, jj = np.ogrid[:NX, :NY]
        trav[(ii - ci) ** 2 + (jj - cj) ** 2 <= 16] = True               # the duck's own footprint is passable
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
        if self.goal is not None:
            gi, gj = cell(*self.goal)
            d2 = (ii - gi) ** 2 + (jj - gj) ** 2
            cand = reached & (d2 <= (0.34 / RES) ** 2)
            if cand.any():
                k = np.argmin(np.where(cand, d2, 1e9)); target, mode = np.unravel_index(k, d2.shape), "goal"
        if target is None:
            unknown = np.abs(self.logodds) <= 0.5
            near_unknown = ndimage.binary_dilation(unknown, iterations=1)
            frontier = reached & near_unknown & (dist > 0.35 / RES)
            lab, n = ndimage.label(frontier)
            if n:
                sizes = ndimage.sum(frontier, lab, range(1, n + 1))
                big = np.isin(lab, np.nonzero(sizes >= 6)[0] + 1)        # ignore tiny slivers
                if big.any():
                    k = np.argmin(np.where(big, dist, 1e9)); target = np.unravel_index(k, dist.shape)
        if target is None:
            return None, mode
        path = [tuple(int(v) for v in target)]
        while path[-1] != (ci, cj):
            path.append(parent[path[-1]])
        path.reverse()
        return [(X0 + (i + 0.5) * RES, Y0 + (j + 0.5) * RES) for i, j in path], mode


def calibrate(vx):
    """Measure the real forward speed for the walk command used (like timing the robot over a tape)."""
    d = motions.Duck(); d.use("velstand"); d.run(1.0); d.cmd[0] = vx; d.run(2.0)
    x0, y0 = d.d.qpos[0], d.d.qpos[1]; d.run(6.0)
    return math.hypot(d.d.qpos[0] - x0, d.d.qpos[1] - y0) / 6.0 / vx


def draw_map(slam, true_xy, est_xy, path, scale=4):
    from PIL import Image, ImageDraw
    img = np.full((NX, NY, 3), 128, np.uint8)
    img[slam.logodds < -0.5] = 255; img[slam.logodds > 0.8] = 0
    im = Image.fromarray(np.flipud(img.transpose(1, 0, 2))).resize((NX * scale, NY * scale), Image.NEAREST)
    dr = ImageDraw.Draw(im)
    px = lambda x, y: ((x - X0) / RES * scale, (NY - (y - Y0) / RES) * scale)
    if path: dr.line([px(*p) for p in path], fill=(255, 160, 0), width=2)
    if len(true_xy) > 1: dr.line([px(*p) for p in true_xy], fill=(0, 170, 0), width=2)
    if len(est_xy) > 1: dr.line([px(*p) for p in est_xy], fill=(220, 0, 0), width=2)
    gx, gy = px(*GOAL); dr.ellipse([gx - 6, gy - 6, gx + 6, gy + 6], outline=(255, 0, 255), width=3)
    if slam.goal is not None:
        ex, ey = px(*slam.goal); dr.line([ex - 7, ey, ex + 7, ey], fill=(255, 0, 255), width=2); dr.line([ex, ey - 7, ex, ey + 7], fill=(255, 0, 255), width=2)
    x, y, th = slam.pose; cx, cy = px(x, y)
    dr.ellipse([cx - 5, cy - 5, cx + 5, cy + 5], fill=(220, 0, 0)); dr.line([cx, cy, cx + 14 * math.cos(th), cy - 14 * math.sin(th)], fill=(220, 0, 0), width=2)
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true"); ap.add_argument("--gif", action="store_true")
    ap.add_argument("--time", type=float, default=240.0); ap.add_argument("--walk", type=float, default=0.4)
    ap.add_argument("--no-match", action="store_true", help="disable scan matching (dead reckoning only), for comparison")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    gain = calibrate(a.walk)
    print(f"calibration: walk command {a.walk} -> {gain * a.walk:.3f} m/s (gain {gain:.2f})", flush=True)

    d = motions.Duck(make_room(), view=a.view); d.use("velstand")
    m, dd = d.m, d.d
    rgb_r = mujoco.Renderer(m, H, W); dep_r = mujoco.Renderer(m, H, W); dep_r.enable_depth_rendering()
    cam = m.camera("head_cam").id
    slam = DuckSLAM(gain)
    true_xy, est_xy, errs, frames, path, mode = [], [], [], [], None, "explore"
    t, done, falls, was_up, sample_saved = 0.0, False, 0, True, False
    d.run(1.0)
    for k in range(round(a.time * 50)):
        t = k / 50.0
        up = d.up_z() > 0.9 and dd.qpos[2] > 0.09
        if was_up and not up: falls += 1
        was_up = up
        slam.predict(float(d.cmd[0]) if up else 0.0, float(dd.sensordata[d.gyro + 2]), 0.02)

        if k % 10 == 0 and up:                                    # ---- perception + SLAM, 5 Hz
            rgb_r.update_scene(dd, camera=cam); rgb = rgb_r.render()
            dep_r.update_scene(dd, camera=cam); depth = dep_r.render()
            # camera pose in the level body frame: FK from encoders + IMU roll/pitch. Computed here from
            # MuJoCo by removing the true position and heading, which the algorithm never sees.
            q = dd.xquat[d.trunk]; yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
            Rz = np.array([[math.cos(yaw), math.sin(yaw), 0], [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
            R_bc = Rz @ dd.cam_xmat[cam].reshape(3, 3)
            t_bc = Rz @ (dd.cam_xpos[cam] - np.array([dd.qpos[0], dd.qpos[1], 0.0]))
            ang, r, hit, _, _ = slam.scan_from_depth(depth, R_bc, t_bc)
            if len(ang):
                if not a.no_match: slam.match(ang, r, hit)
                slam.update_map(ang, r, hit)
            slam.see_goal(rgb, depth, R_bc, t_bc)
            true_xy.append((dd.qpos[0], dd.qpos[1])); est_xy.append((slam.pose[0], slam.pose[1]))
            errs.append(math.hypot(dd.qpos[0] - slam.pose[0], dd.qpos[1] - slam.pose[1]))
            if not sample_saved and t > 6:
                from PIL import Image
                dv = (np.clip(depth / MAX_R, 0, 1) * 255).astype(np.uint8)
                both = np.concatenate([rgb, np.stack([dv] * 3, -1)], 1)
                Image.fromarray(both).resize((W * 2 * 3, H * 3), Image.NEAREST).save(OUT / "camera_rgb_depth.png"); sample_saved = True

        if k % 50 == 0 and up:                                    # ---- planning, 1 Hz
            path, mode = slam.plan()
            if a.gif and k % 100 == 0: frames.append(draw_map(slam, true_xy, est_xy, path, scale=3))
            if k % 500 == 0:
                print(f"t={t:5.0f}s mode={mode:7s} est=({slam.pose[0]:+.2f},{slam.pose[1]:+.2f}) true=({dd.qpos[0]:+.2f},{dd.qpos[1]:+.2f}) "
                      f"err={errs[-1] if errs else 0:.2f} m goal_seen={slam.goal is not None}", flush=True)

        # ---- drive: pure pursuit, 50 Hz
        d.cmd[:] = 0
        d.cmd[5] = 0.9 * math.sin(2 * math.pi * t / 4.0)          # head sweep
        if slam.goal is not None and math.hypot(slam.goal[0] - slam.pose[0], slam.goal[1] - slam.pose[1]) < 0.36:
            done = True
        elif path and up:
            ahead = [p for p in path if math.hypot(p[0] - slam.pose[0], p[1] - slam.pose[1]) > 0.22]
            tgt = ahead[0] if ahead else path[-1]
            e = (math.atan2(tgt[1] - slam.pose[1], tgt[0] - slam.pose[0]) - slam.pose[2] + math.pi) % (2 * math.pi) - math.pi
            if abs(e) > 0.6:
                d.cmd[2] = 1.2 * np.sign(e)                         # turn on the spot
            else:
                d.cmd[0] = a.walk; d.cmd[2] = float(np.clip(2.0 * e, -1.0, 1.0))
        elif up and path is None and t > 3:
            d.cmd[2] = 1.0                                          # nothing to plan to yet: look around
        if done:
            break
        d.step()

    true_d = math.hypot(dd.qpos[0] - GOAL[0], dd.qpos[1] - GOAL[1])
    ok = done and true_d < 0.45
    known = float((np.abs(slam.logodds) > 0.5).mean())
    lines = [
        f"result              : {'REACHED the beacon' if ok else ('stopped, but not at the beacon' if done else 'did NOT reach the beacon')}",
        f"time                : {t:.0f} s simulated",
        f"true distance to beacon at the end : {true_d:.2f} m",
        f"beacon position     : estimated {None if slam.goal is None else tuple(round(float(v), 2) for v in slam.goal)}, true {GOAL}",
        f"SLAM position error : mean {np.mean(errs):.3f} m, max {np.max(errs):.3f} m, final {errs[-1]:.3f} m",
        f"scan matching       : {'OFF' if a.no_match else f'{len(slam.match_shift)} corrections, mean shift {1000*np.mean(slam.match_shift or [0]):.0f} mm'}",
        f"path walked         : {sum(math.hypot(b[0]-a_[0], b[1]-a_[1]) for a_, b in zip(true_xy, true_xy[1:])):.1f} m, falls {falls}",
        f"map                 : {100*known:.0f}% of the {NX*RES:.1f} x {NY*RES:.1f} m grid classified",
    ]
    print("\n".join(lines))
    tag = "_nomatch" if a.no_match else ""
    (OUT / f"summary{tag}.txt").write_text("\n".join(lines) + "\n")
    draw_map(slam, true_xy, est_xy, path).save(OUT / f"map{tag}.png")
    if frames:
        frames.append(draw_map(slam, true_xy, est_xy, path, scale=3))
        frames[0].save(OUT / f"slam{tag}.gif", save_all=True, append_images=frames[1:], duration=150, loop=0)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
