"""Finite JSONL contract replay backend. This never accesses hardware.

Each line: {"stamp_ns": 123, "ranges": [64 metre values], "status": [64 codes]}.
read() returns one recorded frame then None at EOF. stamp_offset_ns explicitly
aligns recorded acquisition times to the ROS clock used by the consumer. Replay
cadence is not a model of device timing; use rosbag for timed ROS integration.
"""
import json

from .sensor_contract import TofFrame


class ReplayBackend:
    def __init__(self, config):
        self.stream = open(config["path"], encoding="utf-8")
        self.offset = int(config.get("stamp_offset_ns", 0))

    def read(self):
        line = self.stream.readline()
        if not line:
            return None
        row = json.loads(line)
        return TofFrame(row["stamp_ns"] + self.offset, row["ranges"], row["status"])

    def close(self):
        self.stream.close()


def create_backend(config):
    return ReplayBackend(config)
