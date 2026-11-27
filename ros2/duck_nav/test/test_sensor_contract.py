"""Hardware acquisition boundary regression tests, no ROS or device required."""
import json
import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from duck_nav.sensor_contract import TofFrame, optical_to_mujoco_quaternion, tof_records
from duck_nav.tof_replay import create_backend


def frame(**changes):
    args = dict(stamp_ns=1_000_000_000, ranges=np.ones(64), status=np.zeros(64, int))
    args.update(changes)
    return TofFrame(**args)


def test_radial_and_axial_conventions_and_zone_order():
    radial = np.array(tof_records(frame(), "radial"))
    axial = np.array(tof_records(frame(), "axial"))
    np.testing.assert_allclose(np.linalg.norm(radial[:, :3], axis=1), 1)
    np.testing.assert_allclose(axial[:, 0], 1)
    assert radial[0, 1] > 0 and radial[0, 2] > 0  # top left
    assert radial[-1, 1] < 0 and radial[-1, 2] < 0
    np.testing.assert_array_equal(radial[:, 5], np.arange(64))


def test_invalid_zones_are_never_free_space():
    ranges, status = np.ones(64), np.zeros(64, int)
    ranges[:5] = [np.nan, np.inf, -1, 0, 3]
    status[5:7] = [1, 2]
    records = np.array(tof_records(frame(ranges=ranges, status=status), "radial"))
    assert np.isnan(records[:7, :4]).all()
    np.testing.assert_array_equal(records[:7, 4], [2, 2, 2, 2, 2, 1, 2])
    assert np.isfinite(records[7:, :4]).all()


@pytest.mark.parametrize("changes", [dict(stamp_ns=0), dict(stamp_ns=1.5),
    dict(ranges=[1] * 63), dict(status=[5] * 64), dict(status=[0.5] * 64)])
def test_malformed_backend_data_rejected(changes):
    with pytest.raises(ValueError):
        tof_records(frame(**changes), "radial")


def test_unknown_convention_rejected():
    with pytest.raises(ValueError):
        tof_records(frame(), "guessed")


@pytest.mark.parametrize("angles", [[0, 0, 0], [.2, -.4, .9], [-1, .6, -2]])
def test_camera_conversion_preserves_forward_and_up(angles):
    source = Rotation.from_euler("xyz", angles)
    output = Rotation.from_quat(optical_to_mujoco_quaternion(source.as_quat()))
    np.testing.assert_allclose(output.apply([0, 0, -1]), source.apply([0, 0, 1]), atol=1e-12)
    np.testing.assert_allclose(output.apply([0, 1, 0]), source.apply([0, -1, 0]), atol=1e-12)


def test_replay_preserves_acquisition_and_validity(tmp_path):
    path = tmp_path / "tof.jsonl"
    status = [0] * 64
    status[9] = 1
    path.write_text(json.dumps(dict(stamp_ns=1000, ranges=[1] * 64, status=status)) + "\n")
    backend = create_backend(dict(path=str(path), stamp_offset_ns=2000))
    try:
        result = backend.read()
        assert result.stamp_ns == 3000
        rows = tof_records(result, "axial")
        assert rows[9][4:] == (1, 9) and np.isnan(rows[9][0])
        assert backend.read() is None
    finally:
        backend.close()
    assert backend.stream.closed


@pytest.mark.parametrize("stamp,now,last,expected", [
    (1_000_000_000, 1_100_000_000, 900_000_000, True),
    (1_000_000_000, 1_500_000_000, None, True),
    (1_000_000_000, 1_500_000_001, None, False),
    (1_000_000_000, 900_000_000, None, False),
    (1_000_000_000, 1_100_000_000, 1_000_000_000, False),
    (900_000_000, 1_100_000_000, 1_000_000_000, False),
    (0, 0, None, False), (True, 1, None, False), (1.5, 2, None, False),
])
def test_acquisition_age_and_monotonicity(stamp, now, last, expected):
    from duck_nav.sensor_contract import acquisition_is_fresh
    assert acquisition_is_fresh(stamp, now, .5, last) is expected


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf"), 0, -1])
def test_timeouts_must_be_finite_and_positive(value):
    from duck_nav.sensor_contract import positive_seconds
    with pytest.raises(ValueError):
        positive_seconds(value, "timeout")


def test_a010_grid_sizes_and_rectangular_fov_are_accepted():
    for n in (25, 100):
        f = TofFrame(1_000_000_000, np.ones(n * n), np.zeros(n * n, int))
        rec = np.array(tof_records(f, "axial", 70.0, 0.2, 2.5, zones=n, fov_v_deg=60.0))
        assert rec.shape == (n * n, 6)
        np.testing.assert_allclose(rec[:, 0], 1.0)                       # axial: x equals the range
        top_left, bottom_right = rec[0, :3], rec[-1, :3]
        assert top_left[1] > 0 and top_left[2] > 0 and bottom_right[1] < 0 and bottom_right[2] < 0
        assert abs(np.degrees(np.arctan2(top_left[1], 1.0))) < 35 and abs(np.degrees(np.arctan2(top_left[2], 1.0))) < 30
    with pytest.raises(ValueError):                                      # 64 values for a 25 x 25 grid
        tof_records(frame(), "axial", zones=25)


def test_calibrated_rays_replace_the_pinhole_model():
    from duck_nav.sensor_contract import zone_rays
    rays = np.tile([2.0, 0.0, 0.0], (4, 1)); rays[1] = [1.0, 1.0, 0.0]
    out = zone_rays(2, 45.0, rays=rays)
    np.testing.assert_allclose(np.linalg.norm(out, axis=1), 1.0)
    rec = np.array(tof_records(TofFrame(1, np.ones(4), np.zeros(4, int)), "radial", zones=2, rays=rays))
    np.testing.assert_allclose(rec[1, :3], [2 ** -0.5, 2 ** -0.5, 0.0])
    for bad in (np.ones((3, 3)), -rays):
        with pytest.raises(ValueError):
            zone_rays(2, 45.0, rays=bad)


def a010_bytes(img, frame_id=7):
    rows, cols = img.shape
    info = bytes([0, 0, 30, 30, 0, 0, 0, 0, 0, 0, rows, cols, frame_id & 0xFF, frame_id >> 8, 1, 0])
    body = info + img.astype(np.uint8).tobytes()
    head = b"\x00\xff" + len(body).to_bytes(2, "little")
    return head + body + bytes([sum(head + body) & 0xFF, 0xDD])


def test_a010_parser_recovers_frames_from_a_noisy_stream():
    from duck_nav.a010_backend import parse_frames
    img = np.arange(625, dtype=np.uint32).reshape(25, 25) % 250 + 1
    good = a010_bytes(img)
    bad = bytearray(a010_bytes(img, 8)); bad[30] ^= 0xFF                 # checksum mismatch
    stream = b"\x12\x00junk" + bytes(bad) + good + good[:40]
    frames, rest = parse_frames(bytearray(stream))
    assert len(frames) == 1 and frames[0][:3] == (25, 25, 7)
    np.testing.assert_array_equal(frames[0][3], img)
    frames, rest = parse_frames(bytearray(rest) + good[40:])             # the split frame completes
    assert len(frames) == 1 and not rest


def test_a010_backend_converts_depth_and_marks_no_target():
    from duck_nav.a010_backend import A010Backend, depth_mm
    assert depth_mm([51], 0)[0] == pytest.approx(100.0) and depth_mm([20], 5)[0] == 100.0
    assert np.isnan(depth_mm([0, 255])).all()
    img = np.full((25, 25), 51); img[0, 0] = 0; img[0, 1] = 255
    data = [a010_bytes(img)]

    class FakeSerial:
        def __init__(self):
            self.written = []
        def write(self, b):
            self.written.append(b)
        def read(self, n):
            return data.pop() if data else b""
        def close(self):
            pass
    dev = FakeSerial()
    b = A010Backend({"binning": 4, "latency_s": 0.05}, serial_factory=lambda: dev, clock_ns=lambda: 2_000_000_000)
    import time as _t
    for _ in range(100):
        f = b.read()
        if f is not None:
            break
        _t.sleep(0.01)
    b.close()
    assert f.stamp_ns == 1_950_000_000
    assert f.ranges.shape == (625,) and f.ranges[2] == pytest.approx(0.1)
    assert list(f.status[:3]) == [1, 1, 0]
    assert b"AT+BINN=4\r" in dev.written
    rec = tof_records(f, "axial", 70.0, 0.05, 2.5, zones=25, fov_v_deg=60.0)
    assert len(rec) == 625 and rec[0][4] == 1 and np.isnan(rec[0][0])


def run_backend(chunks, read_error=None):
    from duck_nav.a010_backend import A010Backend
    import time as _t
    data = list(chunks)

    class FakeSerial:
        def write(self, b):
            pass
        def read(self, n):
            if data:
                return data.pop(0)
            if read_error is not None:
                raise read_error
            _t.sleep(0.005); return b""
        def close(self):
            pass
    clock = iter(range(1_000_000_000, 10_000_000_000, 100_000_000))
    b = A010Backend({"binning": 4, "latency_s": 0.0}, serial_factory=FakeSerial, clock_ns=lambda: next(clock))
    _t.sleep(0.2)
    return b


def test_a010_keeps_only_the_newest_frame_of_a_read_and_counts_gaps():
    img = np.full((25, 25), 51)
    b = run_backend([a010_bytes(img, 1) + a010_bytes(img, 2), a010_bytes(img, 5)])
    frames = [f for f in (b.read(), b.read(), b.read()) if f is not None]
    b.close()
    assert len(frames) == 2                                       # 1 superseded by 2 in the same read
    assert frames[0].stamp_ns < frames[1].stamp_ns                # never two frames with the same stamp
    assert b.stats["superseded"] == 1 and b.stats["seq_gaps"] == 1 and b.stats["frames"] == 3


def test_a010_reader_failure_is_raised_by_read():
    b = run_backend([], read_error=OSError("device unplugged"))
    with pytest.raises(RuntimeError, match="unplugged"):
        b.read()
    b.stop.set()


def test_a010_queue_eviction_survives_consumer_draining_in_between():
    """Review ff34442 A02: the reader saw a full queue, the consumer drained it, and the eviction's queue.Empty ended
    the reader thread; every later read() then raised. Deterministic interleaving, no threads."""
    import queue as _q
    from duck_nav.a010_backend import A010Backend

    class RacyQueue(_q.Queue):
        """full() reports True once, then the 'consumer' empties the queue before the producer evicts."""
        def __init__(self):
            super().__init__(maxsize=2); self.armed = False
        def full(self):
            f = super().full()
            if f and self.armed:
                self.armed = False
                while True:
                    try:
                        super().get_nowait()
                    except _q.Empty:
                        break
            return f

    b = object.__new__(A010Backend)
    b.q = RacyQueue(); b.stats = dict(queue_dropped=0); b.error = None
    b._offer("f1"); b._offer("f2")
    b.q.armed = True
    b.q.put_nowait = _racy_put(b.q)                    # the put sees Full, then the consumer drains the queue
    b._offer("f3")                                     # used to raise queue.Empty (via full() + get_nowait())
    assert b.read() == "f3" and b.read() is None and b.error is None
    # plain overflow still evicts the oldest and counts it
    b.q = _q.Queue(maxsize=2); b.stats = dict(queue_dropped=0)
    for f in ("a", "b", "c", "d"):
        b._offer(f)
    assert b.stats["queue_dropped"] == 2 and b.read() == "c" and b.read() == "d"


def _racy_put(q):
    """put_nowait that raises Full once while the queue is full, after which the consumer drains everything."""
    import queue as _q
    orig = _q.Queue.put_nowait
    state = {"done": False}

    def put(item):
        if not state["done"] and q.qsize() >= q.maxsize:
            state["done"] = True
            while q.qsize():
                _q.Queue.get_nowait(q)               # consumer drains between the Full and the eviction
            raise _q.Full
        return orig(q, item)
    return put
