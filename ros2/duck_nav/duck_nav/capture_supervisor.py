"""capture_supervisor -- keep a blocking frame source (OpenCV/V4L2 capture) alive without ever waiting on it forever.

A V4L2 read can block indefinitely (unplugged camera, wedged driver). A flag the capture thread checks between
reads cannot recover that, because the thread never gets back to the check. Here every device handle belongs to one
worker thread, which opens it, reads it and releases it; nothing else touches that handle. When frames stop for
`stale_after` seconds, the supervisor abandons the worker (it will release its own handle if its read ever returns,
and its late frames are discarded) and starts a new worker with a new handle. At most `max_abandoned` abandoned
workers may still be stuck; beyond that no new worker is started and the state is "stuck", so a wedged driver cannot
make the node pile up threads. `close()` returns within `join_timeout` seconds whatever the workers do.

Pure Python (threading only), so it is tested without a camera: test/test_sensor_io.py.
"""
import threading
import time


class _Worker:
    def __init__(self, sup, gen):
        self.sup, self.gen = sup, gen
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, name=f"capture-{gen}", daemon=True)

    def run(self):
        sup, cap = self.sup, None
        try:
            while not self.stop.is_set():
                if cap is None:
                    cap = sup.open_fn()
                    if cap is None:
                        sup._note("open_failed")
                        self.stop.wait(sup.reopen_delay); continue
                    sup._note("opened")
                ok, frame = cap.read()                       # may block: only this worker waits on it
                t = sup.clock_fn()                           # acquisition stamp: as soon as the read returns
                if self.stop.is_set():
                    break                                    # abandoned or closed meanwhile: discard the frame
                if not ok:
                    sup._note("read_failed")
                    _release(cap); cap = None
                    self.stop.wait(sup.reopen_delay); continue
                sup._deliver(self.gen, t, frame)
        finally:
            if cap is not None:
                _release(cap)
            sup._note("worker_exit")


def _release(cap):
    try:
        cap.release()
    except Exception:                                        # noqa: BLE001  a dead device may fail to release
        pass


class CaptureSupervisor:
    def __init__(self, open_fn, on_frame, clock_fn=time.monotonic, stale_after=1.0, reopen_delay=1.0,
                 max_abandoned=2, join_timeout=1.0, monotonic=time.monotonic):
        """open_fn() -> object with read() -> (ok, frame) and release(), or None if the device cannot be opened.
        on_frame(t, frame) is called from the worker thread for every frame of the current worker."""
        self.open_fn, self.on_frame, self.clock_fn = open_fn, on_frame, clock_fn
        self.stale_after, self.reopen_delay = float(stale_after), float(reopen_delay)
        self.max_abandoned, self.join_timeout, self.monotonic = int(max_abandoned), float(join_timeout), monotonic
        self.lock = threading.Lock()
        self.gen = 0
        self.worker = None
        self.abandoned = []
        self.last_frame = monotonic()
        self.events = []                                     # (event name) history, for status and tests
        self.closed = False

    # -- called from worker threads
    def _note(self, ev):
        with self.lock:
            self.events.append(ev)
            del self.events[:-50]

    def _deliver(self, gen, t, frame):
        with self.lock:
            if gen != self.gen or self.closed:
                return                                       # a late frame from an abandoned worker
            self.last_frame = self.monotonic()
        self.on_frame(t, frame)

    # -- called from the node (timer / main thread)
    def start(self):
        with self.lock:
            self._spawn()

    def _spawn(self):
        self.gen += 1
        self.worker = _Worker(self, self.gen)
        self.last_frame = self.monotonic()                   # give the new handle stale_after s to deliver
        self.worker.thread.start()

    def check(self):
        """Watchdog step: returns 'ok', 'stale' (restarted the worker), 'stuck' (too many workers still blocked),
        or 'closed'."""
        with self.lock:
            if self.closed:
                return "closed"
            self.abandoned = [w for w in self.abandoned if w.thread.is_alive()]
            if self.monotonic() - self.last_frame <= self.stale_after:
                return "ok"
            if len(self.abandoned) >= self.max_abandoned:
                return "stuck"
            old = self.worker
            if old is not None:
                old.stop.set()
                if old.thread.is_alive():
                    self.abandoned.append(old)
            self._spawn()
            self.events.append("restarted")
            return "stale"

    def close(self):
        """Stop every worker; returns within about join_timeout s even if a read never returns (workers are daemon
        threads and release their own handles if they ever wake up)."""
        with self.lock:
            self.closed = True
            workers = ([self.worker] if self.worker else []) + list(self.abandoned)
        for w in workers:
            w.stop.set()
        deadline = time.monotonic() + self.join_timeout
        for w in workers:
            w.thread.join(timeout=max(0.0, deadline - time.monotonic()))
        return [w.thread.name for w in workers if w.thread.is_alive()]
