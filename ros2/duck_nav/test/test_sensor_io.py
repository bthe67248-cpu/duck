"""Sensor input handling in the nodes (review ff34442: A01 QoS, A04 image row stride), with ROS shims only."""
import importlib
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_shims  # noqa: E402
from duck_nav import image_codec  # noqa: E402


class FakeNode:
    """Records what a node asks of rclpy; parameters come from their declared defaults plus OVERRIDES."""
    OVERRIDES = {}

    def __init__(self, name, *a, **k):
        self._params, self.subs, self.warnings = {}, [], []

    def declare_parameter(self, name, default=None):
        self._params[name] = self.OVERRIDES.get(name, default)

    def get_parameter(self, name):
        return SimpleNamespace(value=self._params[name])

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subs.append((msg_type.__name__, topic, qos))

    def create_publisher(self, *a, **k):
        return SimpleNamespace(publish=lambda m: None)

    def create_timer(self, *a, **k):
        return None

    def get_logger(self):
        node = self
        return SimpleNamespace(info=lambda *a, **k: None, warn=lambda m, **k: node.warnings.append(m),
                               error=lambda *a, **k: None)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0))


@pytest.fixture
def slam(monkeypatch):
    ros_shims.install(monkeypatch, {"rclpy.node": {"Node": FakeNode},
                                    "tf2_ros": {"TransformBroadcaster": lambda node: None}})
    monkeypatch.delitem(sys.modules, "duck_nav.slam_node", raising=False)
    mod = importlib.import_module("duck_nav.slam_node")
    yield mod
    monkeypatch.delitem(sys.modules, "duck_nav.slam_node", raising=False)


SENSOR_TOPICS = {"imu", "camera/camera_info", "camera/pose_level", "tof/pose_level", "tof/zones", "camera/depth",
                 "camera/image_raw"}


@pytest.mark.parametrize("mode", ["tof", "depth"])
def test_sensor_subscriptions_are_best_effort_by_default(slam, mode):
    FakeNode.OVERRIDES = {"sensor_mode": mode}
    try:
        n = slam.SlamNode()
    finally:
        FakeNode.OVERRIDES = {}
    subs = {topic: qos for _t, topic, qos in n.subs}
    sensors = SENSOR_TOPICS & set(subs)
    assert {"imu", "camera/camera_info", "camera/pose_level"} <= sensors
    assert ("tof/zones" in sensors) if mode == "tof" else ("camera/depth" in sensors)
    for topic in sensors:
        assert subs[topic].reliability == ros_shims.ReliabilityPolicy.BEST_EFFORT, topic
    assert subs["imu"].depth == 50                       # history is kept, not shortened to the sensor-data default
    # commands and detections are internal topics and stay reliable (plain depth = the default reliable profile)
    assert isinstance(subs["cmd_vel"], int) and isinstance(subs["yolo/detections"], int)


def test_sensor_qos_override_and_validation(slam):
    assert slam.sensor_qos_factory("reliable")(5).reliability == ros_shims.ReliabilityPolicy.RELIABLE
    with pytest.raises(ValueError):
        slam.sensor_qos_factory("fast")


# ---------------------------------------------------------------- A04: image decoding honours `step`
def image(enc, w, h, pixels, pad=0):
    """sensor_msgs/Image-like message; `pixels` is (h, w, c) uint8; each row padded with `pad` junk bytes."""
    rows = [np.concatenate([pixels[r].reshape(-1), np.full(pad, 0xEE, np.uint8)]) for r in range(h)]
    return SimpleNamespace(encoding=enc, width=w, height=h, step=w * pixels.shape[2] + pad,
                           data=np.concatenate(rows).tobytes())


@pytest.mark.parametrize("enc,c", [("rgb8", 3), ("bgr8", 3), ("mono8", 1)])
@pytest.mark.parametrize("pad", [0, 1, 6])
def test_padded_and_contiguous_rows_decode_identically(enc, c, pad):
    rng = np.random.default_rng(1)
    px = rng.integers(0, 256, (2, 5, c), dtype=np.uint8)
    a, b = image_codec.decode(image(enc, 5, 2, px)), image_codec.decode(image(enc, 5, 2, px, pad))
    assert np.array_equal(a, px) and np.array_equal(b, px)
    assert np.array_equal(image_codec.to_rgb(image(enc, 5, 2, px, pad)), image_codec.to_rgb(image(enc, 5, 2, px)))
    assert np.array_equal(image_codec.to_gray(image(enc, 5, 2, px, pad)), image_codec.to_gray(image(enc, 5, 2, px)))


def test_review_case_5x2_rgb8_step16():
    """The review's reproduction: a valid 5 x 2 RGB8 image with 16-byte rows (32 data bytes) used to raise."""
    px = np.arange(30, dtype=np.uint8).reshape(2, 5, 3)
    msg = image("rgb8", 5, 2, px, pad=1)
    assert msg.step == 16 and len(msg.data) == 32
    assert np.array_equal(image_codec.to_rgb(msg), px)


def test_gray_matches_opencv_when_available():
    cv2 = pytest.importorskip("cv2")
    px = np.random.default_rng(2).integers(0, 256, (4, 6, 3), dtype=np.uint8)
    for enc in ("rgb8", "bgr8"):
        assert np.array_equal(image_codec.to_gray(image(enc, 6, 4, px, 2), cv2), image_codec.to_gray(image(enc, 6, 4, px)))
        assert np.abs(image_codec.to_gray(image(enc, 6, 4, px), cv2).astype(int)
                      - image_codec.to_gray(image(enc, 6, 4, px)).astype(int)).max() <= 1


@pytest.mark.parametrize("bad", [
    dict(encoding="yuv422"),                         # unsupported
    dict(step=10),                                   # shorter than a row (5 x 3)
    dict(data=b"\x00" * 20),                         # shorter than height x step
    dict(width=0),
])
def test_malformed_images_raise_decode_error(bad):
    msg = image("rgb8", 5, 2, np.zeros((2, 5, 3), np.uint8), pad=1)
    for k, v in bad.items():
        setattr(msg, k, v)
    with pytest.raises(image_codec.ImageDecodeError):
        image_codec.decode(msg)


def test_slam_flow_path_accepts_padded_and_survives_malformed(slam):
    """on_image (translation:=flow) must decode padded rows and drop a malformed frame with a warning."""
    n = object.__new__(slam.SlamNode)
    FakeNode.__init__(n, "x")
    n.flow_prev, n.flow_interval, n.K = None, 0.15, None
    n.flowodom = SimpleNamespace(cv2=None)
    n.cam_poses = SimpleNamespace(get=lambda ns: None)
    n.flow_pending = []
    px = np.random.default_rng(3).integers(0, 256, (2, 5, 3), dtype=np.uint8)
    msg = image("rgb8", 5, 2, px, pad=1); msg.header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0))
    n.on_image(msg)
    assert len(n.flow_pending) == 1 and n.flow_pending[0][1].shape == (2, 5)
    assert np.array_equal(n.flow_pending[0][1], image_codec.to_gray(image("rgb8", 5, 2, px)))
    bad = image("rgb8", 5, 2, px, pad=1); bad.header = msg.header; bad.data = bad.data[:10]
    n.on_image(bad)                                       # must not raise
    assert len(n.flow_pending) == 1 and n.warnings


# ---------------------------------------------------------------- A03: camera capture that blocks in the driver
import threading  # noqa: E402
import time  # noqa: E402

from duck_nav.capture_supervisor import CaptureSupervisor  # noqa: E402


class FakeCap:
    """read() returns frames; `block` makes it hang until `unblock` is set (a wedged V4L2 read)."""
    def __init__(self, name, block=None, fail_after=None):
        self.name, self.block, self.fail_after, self.n, self.released = name, block, fail_after, 0, False
        self.unblock = threading.Event()

    def read(self):
        if self.block is not None and self.n >= self.block:
            self.unblock.wait()                          # stuck inside the driver
        self.n += 1
        time.sleep(0.01)
        if self.fail_after is not None and self.n > self.fail_after:
            return False, None
        return True, (self.name, self.n)

    def release(self):
        self.released = True


def wait_for(cond, timeout=3.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.01)
    return False


def run_supervisor(caps, **kw):
    frames, opened = [], []
    it = iter(caps)

    def open_fn():
        c = next(it, None)
        if c is not None:
            opened.append(c)
        return c
    sup = CaptureSupervisor(open_fn, lambda t, f: frames.append((t, f)), clock_fn=time.monotonic,
                            stale_after=0.2, reopen_delay=0.05, join_timeout=0.5, **kw)
    sup.start()
    return sup, frames, opened


def test_blocked_read_is_abandoned_and_a_new_handle_delivers():
    stuck = FakeCap("A", block=3)                         # delivers 3 frames, then hangs in read()
    sup, frames, opened = run_supervisor([stuck, FakeCap("B")])
    assert wait_for(lambda: any(f[1][0] == "A" for f in frames))
    states = []
    assert wait_for(lambda: states.append(sup.check()) or "stale" in states)     # watchdog restarts the worker
    assert wait_for(lambda: any(f[1][0] == "B" for f in frames))                # recovered on a new handle
    t_b = [t for t, f in frames if f[0] == "B"]
    assert all(b > a for a, b in zip(t_b, t_b[1:]))       # acquisition stamps keep increasing
    assert not stuck.released                             # nobody touched the blocked handle from outside
    stuck.unblock.set()                                   # the driver finally returns ...
    assert wait_for(lambda: stuck.released)               # ... and the abandoned worker releases its own handle
    n = len(frames); time.sleep(0.05)
    assert all(f[0] == "B" for _t, f in frames[n - 3:])   # its late frame was discarded
    assert sup.close() == []


def test_read_failure_reopens_and_unplug_replug_recovers():
    # A fails after 2 reads (unplugged), then the device cannot be opened twice, then B (replugged) works
    caps = [FakeCap("A", fail_after=2), None, None, FakeCap("B")]
    sup, frames, opened = run_supervisor(caps)
    assert wait_for(lambda: any(f[1][0] == "B" for f in frames))
    assert caps[0].released and "read_failed" in sup.events and "open_failed" in sup.events
    assert sup.close() == []


def test_close_is_bounded_with_a_permanently_blocked_read_and_workers_are_capped():
    caps = [FakeCap(str(i), block=0) for i in range(6)]   # every handle hangs at its first read
    sup, frames, opened = run_supervisor(caps, max_abandoned=2)
    states = []
    for _ in range(40):
        states.append(sup.check()); time.sleep(0.05)
    assert "stuck" in states and len(opened) <= 3         # 1 current + at most 2 abandoned, not one per check
    t0 = time.monotonic()
    left = sup.close()
    assert time.monotonic() - t0 < 1.0 and left           # returns on time and reports the blocked workers
    for c in caps:
        c.unblock.set()
