"""Sipeed MaixSense A010 backend for duck_tof (USB serial). Written from the vendor's frame description; NOT yet run
against a physical sensor. Verify on the bench before trusting it: frame layout, depth unit, invalid codes, zone
order (left/right/up/down targets) and latency.

Use:  ros2 run duck_nav duck_tof --ros-args -p backend:=duck_nav.a010_backend:create_backend \
        -p backend_config:='{"port": "/dev/ttyUSB0", "binning": 4}' -p zones:=25 -p fov_deg:=70.0 -p fov_v_deg:=60.0 \
        -p range_convention:=axial -p min_range:=0.2 -p max_range:=2.5 -p zone_order_verified:=true

Frame (little endian):  0x00 0xFF | length u16 (bytes from the 16-byte info block through the image) |
  info: reserved, output mode, sensor temp, driver temp, exposure u32, error code, reserved, rows, cols, frame id u16,
        ISP version, reserved | image: rows*cols u8, row-major | checksum u8 (sum of all previous bytes) | 0xDD
Depth: with AT+UNIT=0 the pixel p maps to (p / 5.1)^2 mm (fine near, coarse far); with AT+UNIT=n (1..10), p * n mm.
p = 0 and p = 255 are treated as no target (status 1).

Acquisition time = host receive time of the frame's last byte minus `latency_s` (exposure + readout + USB; measure
it, e.g. against a light or motion event). Host time must be the ROS clock the consumer uses. That stamp is only right
for the newest frame of a read, so when one read holds several complete frames (the host fell behind) only the newest
is kept; the rest are counted in `stats["superseded"]`. Gaps in the sensor's frame id are counted in
`stats["seq_gaps"]` (frames lost on the link or dropped by the sensor). A failure in the reader thread is kept and
re-raised by read(), so duck_tof logs it instead of going quietly stale.
"""
import queue
import threading
import time

import numpy as np

from .sensor_contract import TofFrame

HEADER = b"\x00\xff"
TAIL = 0xDD
INFO_LEN = 16
MAX_SIDE = 100


def depth_mm(pixels, unit=0):
    """Raw u8 pixels -> depth in mm (float), NaN where the sensor reports no target (0 or 255)."""
    p = np.asarray(pixels, float)
    d = (p / 5.1) ** 2 if unit == 0 else p * float(unit)
    return np.where((p <= 0) | (p >= 255), np.nan, d)


def parse_frames(buf):
    """Extract complete frames from a byte buffer. Returns (frames, rest) where each frame is
    (rows, cols, frame_id, pixels uint8 array) and rest is the unconsumed tail. Corrupt frames are skipped."""
    frames = []
    i = 0
    n = len(buf)
    while True:
        j = buf.find(HEADER, i)
        if j < 0:
            return frames, buf[max(i, n - 1):]                 # keep a trailing 0x00 that may start a header
        if j + 4 > n:
            return frames, buf[j:]
        length = buf[j + 2] | (buf[j + 3] << 8)
        end = j + 4 + length + 2                             # + checksum + tail
        if length < INFO_LEN or length > INFO_LEN + MAX_SIDE * MAX_SIDE:
            i = j + 1; continue
        if end > n:
            return frames, buf[j:]
        info = buf[j + 4:j + 4 + INFO_LEN]
        rows, cols = info[10], info[11]
        ok = (rows * cols == length - INFO_LEN and buf[end - 1] == TAIL
              and (sum(buf[j:end - 2]) & 0xFF) == buf[end - 2])
        if not ok:
            i = j + 1; continue
        img = np.frombuffer(bytes(buf[j + 4 + INFO_LEN:end - 2]), np.uint8).reshape(rows, cols)
        frames.append((rows, cols, info[12] | (info[13] << 8), img))
        i = end


def frame_from_pixels(img, stamp_ns, unit=0, flip_lr=False, flip_ud=False):
    """Pixels (rows, cols) -> TofFrame in metres, canonical row-major top-left first as seen from behind the sensor.
    Status 0 valid, 1 no target. Flips correct the zone order once it has been measured on the bench."""
    if flip_lr:
        img = img[:, ::-1]
    if flip_ud:
        img = img[::-1, :]
    d = depth_mm(img, unit) / 1000.0
    status = np.where(np.isfinite(d), 0, 1).astype(np.uint8)
    return TofFrame(int(stamp_ns), d.ravel(), status.ravel())


class A010Backend:
    """Background serial reader with a bounded queue; read() never blocks."""

    def __init__(self, config, serial_factory=None, clock_ns=None):
        self.unit = int(config.get("unit", 0))
        binning = int(config.get("binning", 4))
        if binning not in (1, 2, 4):
            raise ValueError("binning must be 1 (100x100), 2 (50x50) or 4 (25x25)")
        self.side = MAX_SIDE // binning
        self.latency_ns = int(float(config.get("latency_s", 0.06)) * 1e9)
        self.flip_lr, self.flip_ud = bool(config.get("flip_lr", False)), bool(config.get("flip_ud", False))
        self.clock_ns = clock_ns or time.time_ns
        if serial_factory is None:
            import serial                                    # pyserial, only needed with a real device
            serial_factory = lambda: serial.Serial(config.get("port", "/dev/ttyUSB0"),
                                                   int(config.get("baud", 115200)), timeout=0.05)
        self.dev = serial_factory()
        commands = config.get("init", ["AT+DISP=3", f"AT+BINN={binning}", f"AT+UNIT={self.unit}",
                                       f"AT+FPS={int(config.get('fps', 15))}"])
        for c in commands:                                   # vendor AT commands; check against the firmware docs
            self.dev.write((c + "\r").encode())
            time.sleep(0.05)
        self.q = queue.Queue(maxsize=2)
        self.stats = dict(frames=0, superseded=0, queue_dropped=0, wrong_size=0, seq_gaps=0)
        self.last_id = None
        self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            self._loop()
        except Exception as exc:                             # noqa: BLE001  surfaced by read()
            self.error = exc

    def _loop(self):
        buf = bytearray()
        while not self.stop.is_set():
            chunk = self.dev.read(4096)
            if not chunk:
                continue
            t = self.clock_ns()
            buf += chunk
            frames, rest = parse_frames(buf)
            buf = bytearray(rest)
            good = []
            for rows, cols, fid, img in frames:
                if rows != self.side or cols != self.side:
                    self.stats["wrong_size"] += 1; continue
                if self.last_id is not None and fid != (self.last_id + 1) & 0xFFFF:
                    self.stats["seq_gaps"] += 1
                self.last_id = fid
                good.append(img)
            if not good:
                continue
            self.stats["frames"] += len(good)
            self.stats["superseded"] += len(good) - 1       # only the newest has a correct receive-time stamp
            self._offer(frame_from_pixels(good[-1], t - self.latency_ns, self.unit, self.flip_lr, self.flip_ud))

    def _offer(self, fr):
        """Queue a frame without ever blocking, evicting the oldest one when full (newest data wins). `full()`
        followed by `get_nowait()` is not atomic: the consumer can drain the queue in between, and the eviction's
        queue.Empty used to end the reader thread for good. This thread is the only producer, so once an eviction
        succeeds or finds the queue already drained, the next put has room."""
        while True:
            try:
                self.q.put_nowait(fr)
                return
            except queue.Full:
                try:
                    self.q.get_nowait()
                    self.stats["queue_dropped"] += 1         # keep the newest acquisitions
                except queue.Empty:
                    pass                                     # the consumer took them meanwhile: just retry the put

    @property
    def dropped(self):
        return self.stats["queue_dropped"]

    @property
    def wrong_size(self):
        return self.stats["wrong_size"]

    def read(self):
        if self.error is not None:
            raise RuntimeError(f"A010 reader thread failed: {self.error!r}")
        try:
            return self.q.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.0)
        self.dev.close()


def create_backend(config):
    return A010Backend(config)
