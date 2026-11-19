"""Offline SLAM replay against ground truth (ToF / realistic profile). Runs in the ROS 2 Python environment.

    python3 tools/slam_replay.py <bag_dir> [--config JSON | --grid JSON_LIST] [--src <dir containing duck_nav/>]

Reads /imu, /cmd_vel, /tof/zones, /tof/pose_level, /ground_truth/pose and /clock (plus the recorded /slam/pose
for reference) from a bag recorded with the realistic profile, runs the slam_node pipeline (tilt filter,
gait-command prediction, acquisition-time ToF processing, optional sweep window, matching, mapping) without
ROS, and reports pose error against same-stamp ground truth. Open loop: the robot's path is the recorded one,
so this measures localisation, not navigation (a variant that would have stopped the robot is still scored on
the recorded motion; "ok" is how often its motion gate would have allowed driving). It mirrors
slam_node.on_imu / on_frame; keep the two in step when either changes.

Config keys (defaults = current slam_node behaviour):
  matcher   correlative | grid | none          sweep    seconds of accumulated frames (0 = single frame)
  min_hits  hits needed to match               loc      FrameLocalizer keyword overrides
  fresh     current-frame agreement check      attitude quaternion (default) | tilt (older TiltFilter)
  translation  command (default) | flow | flow_or_command
               command: body translation = gait command x gain x online slip (current slam_node behaviour).
               flow / flow_or_command: translation from camera floor optical flow (tools/flow_odom.py; needs
               /camera/image_raw, /camera/pose_level, /camera/camera_info in the bag); slip scaling is off.
               Heading still comes from the attitude filter and the fall gate still applies. Between images
               the pose advances at the last measured body velocity (constant-velocity extrapolation, flow
               mode); when the pair (s0, s1) arrives at s1 + flow_latency, the difference between the
               measured displacement and what was extrapolated over [s0, s1] is added to the current pose
               (causal: the correction lands only after the second image). A pair overlapping a fall, or with
               no fit, counts as zero motion in "flow"; in "flow_or_command" it falls back to the gait
               command (model displacement over the window, and as the extrapolation velocity until the next
               good pair). Flow uses no ground truth.
  flow_latency seconds from the second image's stamp until its estimate is usable (default 0.03)
  flow_method  image (default) | rect | rect_or_image: per-pair tracker (see flow_odom.estimate)
  Historical map/kw/slip options require --legacy. Offline ordering does not reproduce ROS queueing.
"""
import argparse
import bisect
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--config", default="{}")
ap.add_argument("--grid", help="JSON list of configs, evaluated on one read of the bag")
ap.add_argument("--output", type=Path, help="write machine-readable metrics JSON")
ap.add_argument("--duration", type=float, default=600.0, help="maximum simulated seconds from first event")
ap.add_argument("--loop-closure", action="store_true", help="enable shared FrameLocalizer pose graph")
ap.add_argument("--legacy", action="store_true", help="use historical matcher policy instead of shared node policy")
ap.add_argument("--src", default=str(Path(__file__).resolve().parents[1]))
args = ap.parse_args()
if not math.isfinite(args.duration) or args.duration <= 0:
    ap.error("--duration must be finite and positive")
sys.path.insert(0, args.src)
import duck_nav.duckslam as ds  # noqa: E402

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402

TOPICS = ["/imu", "/cmd_vel", "/tof/zones", "/tof/pose_level", "/ground_truth/pose", "/clock", "/slam/pose"]
CAM_TOPICS = ["/camera/image_raw", "/camera/pose_level", "/camera/camera_info"]
TRANSLATIONS = ("command", "flow", "flow_or_command")
DEFAULT = dict(matcher="correlative", sweep=1.0, min_hits=8, map="gated", fresh=False, kw={}, loc={},
               translation="command", flow_latency=0.03, flow_method="image")
LEGACY = dict(matcher="correlative", sweep=1.5, min_hits=12, map="gated", fresh=True, kw={}, attitude="tilt",
              translation="command", flow_latency=0.03, flow_method="image")
# {"min_hits": 8, "loc": {}} replays the current slam_node defaults through duckslam.FrameLocalizer


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def quat_to_mat(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


configs = json.loads(args.grid) if args.grid else [json.loads(args.config)]
if not isinstance(configs, list) or not configs or not all(isinstance(c, dict) for c in configs):
    ap.error("--grid must be a nonempty list of configuration objects")
for _c in configs:
    if _c.get("translation", "command") not in TRANSLATIONS:
        ap.error("translation must be one of " + ", ".join(TRANSLATIONS))
need_flow = any(_c.get("translation", "command") != "command" for _c in configs)
if need_flow and args.legacy:
    ap.error("translation flow modes require the shared policy (omit --legacy)")
if need_flow:
    import flow_odom  # noqa: E402  (tools/; needs cv2)
    import cv2  # noqa: E402
cam = dict(imgs={}, poses={}, K=None)


def read_bag(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    topics = TOPICS + (CAM_TOPICS if need_flow else [])
    if need_flow and not set(CAM_TOPICS) <= set(types):
        ap.error("flow translation needs topics " + ", ".join(CAM_TOPICS))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    out = []
    while r.has_next():
        topic, data, t_rx = r.read_next()
        if topic in CAM_TOPICS:                      # images are reduced to gray on the spot (memory)
            m = deserialize_message(data, types[topic])
            if topic == "/camera/image_raw":
                a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
                cam["imgs"][round(stamp(m.header), 4)] = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
            elif topic == "/camera/pose_level":
                o, p = m.pose.orientation, m.pose.position
                cam["poses"][round(stamp(m.header), 4)] = (quat_to_mat(o.w, o.x, o.y, o.z), np.array([p.x, p.y, p.z]))
            elif cam["K"] is None:
                cam["K"] = np.array(m.k).reshape(3, 3)
            continue
        out.append((topic, deserialize_message(data, types[topic]), t_rx * 1e-9))
    return out


msgs = read_bag(args.bag)
missing = set(TOPICS[:-1]) - {topic for topic, _, _ in msgs}
if missing:
    ap.error("bag is missing required populated topics: " + ", ".join(sorted(missing)))
clock = sorted((t_rx, m.clock.sec + m.clock.nanosec * 1e-9) for tp, m, t_rx in msgs if tp == "/clock")
crx = [c[0] for c in clock]


def sim_time(t_rx):                                   # receive (wall) time -> sim time, for header-less /cmd_vel
    if not clock:
        ap.error("/clock is required to align headerless /cmd_vel; do not substitute wall time")
    if len(clock) == 1:
        return clock[0][1]
    i = min(max(bisect.bisect_left(crx, t_rx), 1), len(clock) - 1)
    return clock[i - 1][1] if t_rx - clock[i - 1][0] < clock[i][0] - t_rx else clock[i][1]


def read_tof(msg):
    ranges = np.full(64, np.nan); valid = np.zeros(64, bool)
    for rng, st, z in point_cloud2.read_points(msg, field_names=("range", "status", "zone"), skip_nans=False):
        if st == 0 and np.isfinite(rng):
            ranges[int(z)] = rng; valid[int(z)] = True
    return ranges, valid


events, tof_pose, gt, live = [], {}, [], []
for tp, m, t_rx in msgs:
    if tp == "/imu":
        g = m.angular_velocity; a = m.linear_acceleration
        events.append((stamp(m.header), 0, (np.array([g.x, g.y, g.z]), np.array([a.x, a.y, a.z]))))
    elif tp == "/cmd_vel":
        events.append((sim_time(t_rx), 1, (m.linear.x, m.linear.y)))
    elif tp == "/tof/zones":
        events.append((stamp(m.header) + 1e-6, 2, (round(stamp(m.header), 6), read_tof(m))))
    elif tp == "/tof/pose_level":
        o, p = m.pose.orientation, m.pose.position
        tof_pose[round(stamp(m.header), 6)] = (quat_to_mat(o.w, o.x, o.y, o.z), np.array([p.x, p.y, p.z]))
    elif tp == "/slam/pose":
        live.append((stamp(m.header), m.pose.position.x, m.pose.position.y))
    elif tp == "/ground_truth/pose":
        gt.append((stamp(m.header), m.pose.position.x, m.pose.position.y))
events.sort(key=lambda e: (e[0], e[1]))
gt.sort(); gts = [g[0] for g in gt]
if not events or not gt:
    ap.error("bag must contain IMU/ToF events and /ground_truth/pose")
start_t = events[0][0]
events = [event for event in events if event[0] < start_t + args.duration]
dur = events[-1][0] - start_t
flow_pairs = {}
if need_flow:                                         # image-pair estimates, computed once; no ground truth involved
    if cam["K"] is None or not cam["imgs"]:
        ap.error("flow translation needs camera images and camera_info in the bag")
    t_flow = time.perf_counter()
    for _m in sorted({_c.get("flow_method", "image") for _c in configs if _c.get("translation", "command") != "command"}):
        t_flow = time.perf_counter()
        flow_pairs[_m] = flow_odom.compute_flow_pairs(cam["imgs"], cam["poses"], cam["K"], max_t=start_t + args.duration,
                                                      method=_m)
        print(f"flow {_m}: {len(flow_pairs[_m])} pairs, {sum(p_[2] is not None for p_ in flow_pairs[_m])} with an "
              f"estimate ({time.perf_counter() - t_flow:.0f} s; helpers from {flow_odom.HELPERS})", flush=True)
    del cam["imgs"]


class Track:
    """Cumulative world-frame displacement over time with linear interpolation (for window differences)."""

    def __init__(self):
        self.t, self.x, self.y = [], [], []

    def add(self, t, dx, dy):
        self.t.append(t)
        self.x.append((self.x[-1] if self.x else 0.0) + dx)
        self.y.append((self.y[-1] if self.y else 0.0) + dy)

    def at(self, t):
        i = bisect.bisect_left(self.t, t)
        if i <= 0:
            return np.array([self.x[0], self.y[0]]) if self.t else np.zeros(2)
        if i >= len(self.t):
            return np.array([self.x[-1], self.y[-1]])
        w = (t - self.t[i - 1]) / max(self.t[i] - self.t[i - 1], 1e-9)
        return np.array([self.x[i - 1] + w * (self.x[i] - self.x[i - 1]), self.y[i - 1] + w * (self.y[i] - self.y[i - 1])])


def truth_err(t, x, y):
    i = bisect.bisect_left(gts, t)
    if i < len(gt) and abs(gts[i] - t) < 0.011:
        return math.hypot(gt[i][1] - x, gt[i][2] - y)
    return None


def run(cfg):
    allowed = set(DEFAULT) | {"debug", "trace", "slip", "slip_kw", "attitude"}
    if set(cfg) - allowed:
        ap.error("unknown config keys: " + ", ".join(sorted(set(cfg) - allowed)))
    if not args.legacy and set(cfg) & {"map", "kw", "slip", "slip_kw"}:
        ap.error("map/kw/slip/slip_kw require --legacy; use loc overrides for shared policy")
    c = {**(LEGACY if args.legacy else DEFAULT), **cfg}
    mode = c["translation"]
    use_flow = mode != "command"
    if args.loop_closure:
        if args.legacy:
            ap.error("--loop-closure requires the shared policy (omit --legacy)")
        c["loc"] = {**c.get("loc", {}), "loop_closure": True}
    slam = ds.DuckSLAM(ds.OdomGains(), max_r=2.0, min_hits=c["min_hits"])
    rays = ds.tof_zone_rays(8, 45.0, 45.0)
    tilt = ds.make_attitude_filter(c.get("attitude", "quaternion"))
    health = ds.LocalizationHealth(0.75, 1.5)
    sweep = ds.ScanWindow(c["sweep"]) if c["sweep"] > 0 else None
    cmd, cmd_t, last_imu = (0.0, 0.0), -1e9, None
    hist, hist_keys = {}, []
    bootstrap = True
    errs, oks, n_frames, n_acc, match_s, reasons = [], [], 0, 0, [], {}
    slips = []
    relocs = 0
    loc_kw = dict(matcher=c["matcher"], sweep=c["sweep"], fresh=c["fresh"], map_move=0.0, map_turn=0.0)
    loc_kw.update(c.get("loc", {}))
    if use_flow:
        loc_kw["slip"] = False                           # flow is metric: no online slip scaling
    loc = ds.FrameLocalizer(slam, **loc_kw) if "loc" in c else None
    state, upright_since, recoveries = "ok", None, 0
    if loc is not None:
        health = loc.health
    next_err_t = 0.0
    # flow modes: odo / cmd_model = cumulative world displacement actually applied / the gait-command model's
    # (no slip) at every IMU step; v_flow = body velocity used between images (set by the latest good pair)
    odo, cmd_model, v_flow, v_flow_t, bad_t = Track(), Track(), None, -1e9, -1e9
    evs = events
    if use_flow:
        evs = sorted(events + [(s1 + c["flow_latency"], 3, (s0, s1, est)) for s0, s1, est in flow_pairs[c["flow_method"]]
                               if s1 + c["flow_latency"] < start_t + args.duration], key=lambda e: (e[0], e[1]))
    flow_ok = flow_missing = 0
    for t, kind, data in evs:
        if kind == 3:                                    # a flow pair (s0, s1) becomes available at s1 + latency
            s0, s1, est = data
            k0 = hist_keys[max(bisect.bisect_left(hist_keys, s0) - 1, 0)] if hist_keys else None
            if k0 is None or bad_t >= s0:                # no history yet, or the body was not upright in the window
                v_flow = None
                continue
            ext = odo.at(s1) - odo.at(s0)                # what the extrapolation applied over [s0, s1]
            if est is not None:
                th0 = hist[k0][2]
                cth, sth = math.cos(th0), math.sin(th0)
                meas = np.array([cth * est[0] - sth * est[1], sth * est[0] + cth * est[1]])
                v_flow, v_flow_t = (est[0] / (s1 - s0), est[1] / (s1 - s0)), s1
                flow_ok += 1
            else:
                flow_missing += 1
                v_flow = None
                meas = (cmd_model.at(s1) - cmd_model.at(s0)) if mode == "flow_or_command" else np.zeros(2)
            resid = meas - ext
            slam.pose[:2] += resid
            for kk in hist_keys[bisect.bisect_right(hist_keys, s1):]:
                hist[kk] = hist[kk] + np.array([resid[0], resid[1], 0.0])
            continue
        if kind == 0:
            gyro, acc = data
            dt = t - last_imu if last_imu is not None else 0.0
            last_imu = t
            up = tilt.update(gyro, acc, dt if 0 < dt < 0.2 else 0.0)
            tilt_deg = math.degrees(tilt.tilt())
            if state == "ok" and tilt_deg > 35.0:
                state, upright_since = "recovering", None
                recoveries += 1
                if loc is not None:
                    loc.reset_after_fall()
                elif sweep is not None:
                    sweep = ds.ScanWindow(c["sweep"])
            elif state == "recovering":
                if tilt_deg < 15.0:
                    if upright_since is None:
                        upright_since = t
                    if t - upright_since > 1.0:
                        state = "ok"
                else:
                    upright_since = None
            if state != "ok":
                bad_t = t
            if 0 < dt < 0.2:
                vx, vy = cmd if t - cmd_t < 0.5 and state == "ok" else (0.0, 0.0)
                if not use_flow:
                    slam.predict(vx, tilt.heading_rate(gyro, dt), dt, cmd_vy=vy)
                else:
                    cb = slam.body_velocity(vx, vy)
                    if state != "ok":
                        vb = (0.0, 0.0)
                    elif v_flow is not None and t - v_flow_t < 0.5:
                        vb = v_flow
                    else:
                        vb = cb if mode == "flow_or_command" else (0.0, 0.0)
                    before = slam.pose[:2].copy()
                    slam.predict_measured(vb[0], vb[1], tilt.heading_rate(gyro, dt), dt)
                    d = slam.pose[:2] - before
                    odo.add(t, d[0], d[1])
                    cth, sth = math.cos(slam.pose[2]), math.sin(slam.pose[2])
                    cmd_model.add(t, (cth * cb[0] - sth * cb[1]) * dt if state == "ok" else 0.0,
                                  (sth * cb[0] + cth * cb[1]) * dt if state == "ok" else 0.0)
            hist[t] = slam.pose.copy(); hist_keys.append(t)
            if t >= next_err_t:
                next_err_t = t + 0.1
                e = truth_err(t, *slam.pose[:2])
                if e is not None:
                    errs.append(e); oks.append(state == "ok" and health.state(t) == "ok")
            continue
        if kind == 1:
            cmd, cmd_t = data, t
            continue
        key, (ranges, valid) = data
        ts = t - 1e-6
        sp = tof_pose.get(key)
        i = bisect.bisect_left(hist_keys, ts)
        cand = [hist_keys[j] for j in (i - 1, i) if 0 <= j < len(hist_keys) and abs(hist_keys[j] - ts) <= 0.03]
        if sp is None or not cand or c["matcher"] == "none" or state != "ok":
            continue
        k = min(cand, key=lambda q: abs(q - ts))
        pose_acq = hist[k].copy()
        R, tt = sp
        ang, r, hit = slam.scan_from_tof(ranges, valid, rays, R, tt, "radial", fov_h_deg=45.0)
        n_frames += 1
        if not len(ang):                                 # empty frames are not health observations (slam_node too)
            continue
        if loc is not None:                              # the shared node policy (duckslam.FrameLocalizer)
            current = slam.pose.copy()
            t0 = time.perf_counter()
            accepted, corr = loc.step(ts, pose_acq, ang, r, hit)
            match_s.append(time.perf_counter() - t0)
            reasons[slam.match_reason] = reasons.get(slam.match_reason, 0) + 1
            n_acc += bool(accepted); slips.append(slam.slip)
            relocs += loc.relocalizing
            corrected = pose_acq + corr
            slam.pose = ds.carry_pose_correction(current, pose_acq, corrected)
            if np.any(corr):
                for kk in hist_keys[bisect.bisect_right(hist_keys, ts):]:
                    hist[kk] = ds.carry_pose_correction(hist[kk], pose_acq, corrected)
            continue
        current = slam.pose.copy(); slam.pose = pose_acq.copy()
        if sweep is not None:
            sweep.add(ts, pose_acq, ang, r, hit)
            cons = sweep.scan(pose_acq)
        else:
            cons = (ang, r, hit)
        fresh = (ang, r, hit) if c["fresh"] else None
        t0 = time.perf_counter()
        if c["matcher"] == "correlative":
            accepted = slam.match_correlative(*cons, current_scan=fresh, **c["kw"])
        else:
            accepted = slam.match(*cons, current_scan=fresh)
        match_s.append(time.perf_counter() - t0)
        reasons[slam.match_reason] = reasons.get(slam.match_reason, 0) + 1
        corr = slam.pose - pose_acq
        if accepted:
            n_acc += 1
            if sweep is not None:
                sweep.correct(pose_acq, slam.pose)
        healthy = health.state(ts) == "ok" and slam.match_reason not in (
            "fresh_scan_disagrees", "search_boundary", "ambiguous", "ambiguous_heading")
        if c.get("debug") and n_frames % 300 == 0:
            print(f"   t={ts:6.1f} occ={(slam.logodds > 0.8).sum():5d} hits={int(hit.sum())} cons={int(cons[2].sum())} "
                  f"bootstrap={bootstrap} reason={slam.match_reason} pose={np.round(slam.pose, 2)}")
        if c["map"] == "always" or accepted or healthy or bootstrap or slam.match_reason == "map_not_ready":
            slam.update_map(ang, r, hit)
        slam.pose = current + corr
        if np.any(corr):
            for kk in hist_keys[bisect.bisect_right(hist_keys, ts):]:
                hist[kk] = hist[kk] + corr
        health.observe(ts, accepted)
        if bootstrap and health.state(ts) == "ok":
            bootstrap = False
        if c.get("slip"):
            slam.observe_slip(**c.get("slip_kw", {}))
            slips.append(slam.slip)
    e = np.array(errs)
    if c.get("trace"):
        w = int(20 / 0.1)
        if slips:
            k = max(1, len(slips) // 15)
            print("   slip per window:", " ".join(f"{np.mean(slips[i:i + k]):.2f}" for i in range(0, len(slips), k)))
        print("   error per 20 s:", " ".join(f"{e[i:i + w].mean():.2f}" for i in range(0, len(e), w)))
    if not len(e) or (not n_frames and c["matcher"] != "none"):
        ap.error("replay has no scored samples or no usable ToF frames in selected duration")
    graph = loc.graph if loc is not None else None
    metrics = dict(graph_revision=graph.revision if graph is not None else 0,
                   graph_nodes=len(graph.nodes) if graph is not None else 0,
                   graph_edges=len(graph.edges) if graph is not None else 0,
                   graph_reason=graph.reason if graph is not None else "disabled", config=cfg, effective_config=c, policy="legacy" if loc is None else "shared",
                   samples=len(e), error_m={"mean": float(e.mean()) if len(e) else None,
                   "median": float(np.median(e)) if len(e) else None, "max": float(e.max()) if len(e) else None},
                   motion_gate_ok_pct=float(100 * np.mean(oks)) if oks else None,
                   flow_pairs_used=flow_ok, flow_pairs_missing=flow_missing,
                   recovery_entries=recoveries, relocalizing_frames=relocs, accepted_frames=int(n_acc),
                   frames=n_frames, match_mean_ms=float(1e3 * np.mean(match_s)) if match_s else None,
                   match_reasons=reasons)
    print(json.dumps(metrics), flush=True)
    return metrics


reference = None
if live:
    le = np.array([v for v in (truth_err(t_, x_, y_) for t_, x_, y_ in live[::5]
                              if start_t <= t_ < start_t + args.duration) if v is not None])
    if len(le):
        reference = dict(samples=len(le), error_m=dict(mean=float(le.mean()), median=float(np.median(le)), max=float(le.max())))
        print(f"{dur:.0f} s bag; recorded live /slam/pose: {json.dumps(reference)}")
results = [run(cfg) for cfg in configs]
if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(schema_version=1, bag=str(Path(args.bag).resolve()), duration_s=dur,
        recorded_live=reference, results=results,
        limitations=["Open-loop localisation only; recorded path is fixed.",
                     "Events are sorted by acquisition time; ROS callback loss, queueing and publication latency are not reproduced.",
                     "Default parameters mirror the launch defaults; custom node parameters must be supplied explicitly."]),
        indent=2, allow_nan=False) + "\n")
