#!/usr/bin/env python3
"""Inventory SQLite bags without ROS; trim and replay ROS 2 bag sets with ROS installed.

No bag files are bundled by this tool. Missing paths are explicit errors.
"""
from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import subprocess
import sys

REQUIRED = {'/clock', '/imu', '/cmd_vel', '/tof/zones', '/tof/pose_level', '/ground_truth/pose'}


def checksum(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def inventory(path):
    if not path.exists():
        return dict(path=str(path), status='missing', required_topics_missing=sorted(REQUIRED))
    files = [path] if path.is_file() and path.suffix == '.db3' else sorted(path.glob('*.db3'))
    if not files:
        return dict(path=str(path), status='unsupported_or_empty',
                    detail='Inventory supports SQLite .db3 bags. MCAP trim/replay requires ROS 2.')
    topics, timestamps = {}, []
    for file in files:
        with sqlite3.connect(file.resolve().as_uri() + '?mode=ro', uri=True) as connection:
            rows = connection.execute('SELECT topics.name,topics.type,COUNT(messages.id),MIN(messages.timestamp),MAX(messages.timestamp) '
                                      'FROM topics LEFT JOIN messages ON topics.id=messages.topic_id GROUP BY topics.id')
            for name, msg_type, count, low, high in rows:
                entry = topics.setdefault(name, dict(type=msg_type, messages=0))
                entry['messages'] += count
                if low is not None:
                    entry['receive_first_ns'] = min(entry.get('receive_first_ns', low), low)
                    entry['receive_last_ns'] = max(entry.get('receive_last_ns', high), high)
                if low is not None:
                    timestamps.extend([low, high])
    populated = {name for name, entry in topics.items() if entry['messages'] > 0}
    missing = sorted(REQUIRED - populated)
    clock = topics.get('/clock', {})
    clock_span = (clock.get('receive_last_ns', 0) - clock.get('receive_first_ns', 0)) * 1e-9
    return dict(path=str(path.resolve()), status='inventoried' if not missing and clock_span > 0 else 'incomplete', topics=topics,
                required_topics_missing=missing, clock_receive_duration_s=clock_span,
                simulated_duration_s=None,
                receive_duration_s=(max(timestamps) - min(timestamps)) * 1e-9 if timestamps else 0,
                note='Receive duration is not simulated duration; use /clock when trimming accelerated trials.',
                files=[dict(name=f.name, bytes=f.stat().st_size, sha256=checksum(f)) for f in files])


def clock_bounds(clocks, start, duration):
    """Select receive-time half-open bounds from monotonic (receive_ns, sim_ns) clocks."""
    if not clocks:
        raise ValueError('/clock is required for simulated-time trimming')
    if any(b[0] < a[0] or b[1] < a[1] for a, b in zip(clocks, clocks[1:])):
        raise ValueError('clock resets or out-of-order clocks: split the bag into monotonic runs first')
    times = [s for _, s in clocks]
    target = times[0] + round(start * 1e9)
    if target > times[-1]:
        raise ValueError('requested start is beyond clock coverage')
    begin = None if start == 0 else clocks[bisect.bisect_left(times, target)][0]
    stop_i = bisect.bisect_left(times, target + round(duration * 1e9))
    stop = clocks[stop_i][0] if stop_i < len(clocks) else None
    return begin, stop


def trim_bag(source, output, start, duration):
    if not source.exists():
        raise ValueError(f'missing bag: {source}')
    if output.exists():
        raise ValueError(f'refusing to overwrite: {output}')
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    def reader():
        result = rosbag2_py.SequentialReader()
        result.open(rosbag2_py.StorageOptions(uri=str(source), storage_id=''), rosbag2_py.ConverterOptions('', ''))
        return result
    first = reader()
    first.set_filter(rosbag2_py.StorageFilter(topics=['/clock']))
    clocks = []
    clock_type = get_message('rosgraph_msgs/msg/Clock')
    while first.has_next():
        _, data, timestamp = first.read_next()
        msg = deserialize_message(data, clock_type)
        clocks.append((timestamp, msg.clock.sec * 1_000_000_000 + msg.clock.nanosec))
    begin, stop = clock_bounds(clocks, start, duration)
    del first
    original = reader()
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    for topic in original.get_all_topics_and_types():
        writer.create_topic(topic)
    count = 0
    while original.has_next():
        topic, data, timestamp = original.read_next()
        if stop is not None and timestamp >= stop:
            break
        if begin is None or timestamp >= begin:
            writer.write(topic, data, timestamp)
            count += 1
    del writer  # Finalize metadata before reporting success.
    report = dict(source=str(source.resolve()), output=str(output.resolve()), messages=count,
                  requested_start_s=start, requested_duration_s=duration,
                  available_clock_s=(clocks[-1][1] - clocks[0][1]) * 1e-9,
                  receive_begin_ns=begin, receive_end_exclusive_ns=stop,
                  time_base='elapsed /clock from first clock; original timestamps preserved',
                  limitations='Receive-time /clock boundary has clock-cadence uncertainty and may exclude delayed acquisitions near cutoff. Sub-window trims do not preserve the pre-window map or transient-local topic state.')
    (output / 'trim.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def replay_set(manifest_path, out, duration, loop_closure):
    manifest = json.loads(manifest_path.read_text())
    bags = manifest.get('bags')
    if not isinstance(bags, list) or not bags:
        raise ValueError('replay manifest requires a nonempty bags list of {name,path}')
    names = [item.get('name') for item in bags]
    if any(not isinstance(n, str) or not n or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in n) for n in names) or len(set(names)) != len(names):
        raise ValueError('bag names must be unique safe filenames')
    out.mkdir(parents=True, exist_ok=False)
    results = []
    for bag in bags:
        path = (manifest_path.parent / bag['path']).resolve()
        entry = dict(name=bag['name'], path=str(path))
        if not path.exists():
            entry['status'] = 'missing'
        else:
            output = out / (bag['name'] + '.json')
            cmd = [sys.executable, str(Path(__file__).with_name('slam_replay.py')), str(path),
                   '--duration', str(duration), '--output', str(output)]
            if loop_closure:
                cmd.append('--loop-closure')
            entry['command'] = cmd
            with (out / (bag['name'] + '.log')).open('w') as log:
                proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
            entry.update(status='failed', returncode=proc.returncode)
            if proc.returncode == 0 and output.is_file():
                metrics = json.loads(output.read_text()).get('results', [])
                if metrics and all(item.get('samples', 0) > 0 and item.get('frames', 0) > 0 for item in metrics):
                    entry['status'] = 'completed'
                else:
                    entry['error'] = 'replay produced no usable scored localisation metrics'
        results.append(entry)
    report = dict(manifest=str(manifest_path.resolve()), results=results)
    (out / 'replay_set.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    inv = commands.add_parser('inventory')
    inv.add_argument('bags', nargs='+', type=Path)
    inv.add_argument('--output', type=Path)
    trim = commands.add_parser('trim')
    trim.add_argument('bag', type=Path)
    trim.add_argument('output', type=Path)
    trim.add_argument('--start', type=float, default=0.)
    trim.add_argument('--duration', type=float, default=600.)
    replay = commands.add_parser('replay-set')
    replay.add_argument('manifest', type=Path)
    replay.add_argument('--out', required=True, type=Path)
    replay.add_argument('--duration', type=float, default=600.)
    replay.add_argument('--loop-closure', action='store_true')
    args = parser.parse_args()
    if hasattr(args, 'duration') and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error('--duration must be finite and positive')
    if hasattr(args, 'start') and (not math.isfinite(args.start) or args.start < 0):
        parser.error('--start must be finite and nonnegative')
    try:
        if args.command == 'inventory':
            report = dict(bags=[inventory(p) for p in args.bags])
            if args.output:
                args.output.write_text(json.dumps(report, indent=2) + '\n')
            failed = any(item['status'] != 'inventoried' for item in report['bags'])
        elif args.command == 'trim':
            report = trim_bag(args.bag, args.output, args.start, args.duration)
            failed = False
        else:
            report = replay_set(args.manifest, args.out, args.duration, args.loop_closure)
            failed = any(item['status'] != 'completed' for item in report['results'])
    except (ValueError, OSError, KeyError, ImportError, sqlite3.Error) as exc:
        parser.exit(2, f'{exc}\n')
    print(json.dumps(report, indent=2))
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
