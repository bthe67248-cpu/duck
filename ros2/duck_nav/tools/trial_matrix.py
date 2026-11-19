#!/usr/bin/env python3
"""Isolated ROS 2 navigation trials. Plan by default; --execute starts processes.

Replacement tooling written in this repository; not the unavailable job2.sh.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

TOPICS = ['/clock', '/imu', '/cmd_vel', '/tof/zones', '/tof/pose_level',
          '/ground_truth/pose', '/slam/pose', '/slam/state', '/slam/match', '/slam/map_revision', '/duck/mode', '/nav/status', '/nav/result']


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return number


def load_layouts(path):
    if path is None:
        return {'default': {}}
    layouts = json.loads(path.read_text())
    allowed = {'mjcf', 'world_json', 'beacon_x', 'beacon_y', 'room'}
    if not isinstance(layouts, dict) or not layouts:
        raise ValueError('layouts must be a nonempty object mapping names to overrides')
    for name, overrides in layouts.items():
        if not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name):
            raise ValueError(f'invalid layout name: {name!r}')
        if not isinstance(overrides, dict) or set(overrides) - allowed:
            raise ValueError(f'{name}: supported overrides are {sorted(allowed)}')
        for key in ('mjcf', 'world_json'):
            if key in overrides:
                asset = (path.parent / overrides[key]).resolve()
                if not asset.is_file():
                    raise ValueError(f'{name}: missing {key}: {asset}')
                overrides[key] = str(asset)
        if name != 'default' and set(overrides) != allowed:
            raise ValueError(f'{name}: alternate layouts require assets, beacon coordinates, and room bounds')
        for key in ('beacon_x', 'beacon_y'):
            if key in overrides and (not isinstance(overrides[key], (int, float)) or not math.isfinite(overrides[key])):
                raise ValueError(f'{name}: {key} must be finite')
        if 'room' in overrides:
            room = overrides['room']
            if not isinstance(room, list) or len(room) != 4 or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in room):
                raise ValueError(f'{name}: room must contain four finite numbers')
            if room[0] >= room[1] or room[2] >= room[3]:
                raise ValueError(f'{name}: invalid room bounds')
    return layouts


def make_trials(layouts, seeds, factors, root, duration, loop_closure=False):
    trials = []
    for (layout, overrides), seed, factor in itertools.product(layouts.items(), seeds, factors):
        name = f'{layout}_seed{seed}_rt{factor:g}'
        out = root / name
        params = dict(sensors='realistic', rviz=False, autonomous=True, real_time=True,
                      rt_factor=factor, timeout=duration, seed=seed, out_dir=str(out),
                      loop_closure=loop_closure, **overrides)
        def value(v):
            return json.dumps(v, separators=(',', ':')) if isinstance(v, (bool, list)) else str(v)
        command = ['ros2', 'launch', 'duck_nav', 'duck_nav.launch.py'] + [f'{k}:={value(v)}' for k, v in params.items()]
        trials.append(dict(name=name, layout=layout, seed=seed, rt_factor=factor,
                           out_dir=str(out), command=command, status='planned'))
    if len({t['name'] for t in trials}) != len(trials):
        raise ValueError('duplicate trial names; use distinct seeds and rt factors')
    return trials


def stop_process(proc):
    if proc is None or proc.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(proc.pid, sig)
            proc.wait(timeout=15 if sig == signal.SIGINT else 5)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


def validate_result(result):
    if not isinstance(result, dict) or result.get('profile') != 'realistic':
        raise ValueError('result must identify the realistic sensor profile')
    if not isinstance(result.get('exploration'), str) or not result['exploration']:
        raise ValueError('result has no exploration outcome')
    for key in ('sim_time_s', 'room_coverage_pct', 'falls', 'walked_m'):
        if not isinstance(result.get(key), (int, float)) or not math.isfinite(result[key]):
            raise ValueError(f'invalid result metric {key}')
    errors = result.get('slam_error_m', {})
    if not isinstance(errors, dict) or errors.get('samples', 0) <= 0:
        raise ValueError('result has no stamp-matched SLAM samples')
    for key in ('mean', 'max', 'final'):
        if not isinstance(errors.get(key), (int, float)) or not math.isfinite(errors[key]):
            raise ValueError(f'invalid SLAM metric {key}')
    return result


def run_trial(trial, domain, wall_timeout, cancel):
    out = Path(trial['out_dir'])
    out.mkdir()  # Never overwrite prior evidence.
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    trial.update(domain_id=domain, status='running')
    recorder = launch = None
    started = time.monotonic()
    try:
        with (out / 'record.log').open('w') as record_log, (out / 'launch.log').open('w') as launch_log:
            try:
                recorder = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(out / 'bag'), *TOPICS],
                                            stdout=record_log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
                # Wait until recorder has created metadata/storage, before starting publishers.
                ready_by = time.monotonic() + 30
                while not any((out / 'bag').glob('*.db3')) and not any((out / 'bag').glob('*.mcap')):
                    if recorder.poll() is not None:
                        raise RuntimeError(f'bag recorder exited {recorder.returncode}; see record.log')
                    if cancel.is_set() or time.monotonic() > ready_by:
                        raise RuntimeError('bag recorder did not become ready')
                    time.sleep(.1)
                launch = subprocess.Popen(trial['command'], stdout=launch_log, stderr=subprocess.STDOUT,
                                          env=env, start_new_session=True)
                while True:
                    log_text = (out / 'launch.log').read_text(errors='replace')
                    if 'process has died' in log_text or 'Traceback (most recent call last)' in log_text:
                        raise RuntimeError('node failure recorded in launch.log')
                    result = out / 'result.json'
                    if result.is_file() and (out / 'run_complete.json').is_file():
                        try:
                            trial['result'] = validate_result(json.loads(result.read_text()))
                            if recorder.poll() is not None:
                                raise RuntimeError('recorder exited before result collection')
                            trial['status'] = 'completed'
                            break
                        except json.JSONDecodeError:
                            pass  # score_node may still be writing.
                    if cancel.is_set():
                        raise RuntimeError('cancelled')
                    if launch.poll() is not None:
                        raise RuntimeError(f'launch exited {launch.returncode} before a result')
                    if recorder.poll() is not None:
                        raise RuntimeError(f'recorder exited {recorder.returncode} during trial')
                    if time.monotonic() - started > wall_timeout:
                        raise RuntimeError(f'wall timeout ({wall_timeout:g} s)')
                    time.sleep(.2)
            finally:
                stop_process(launch)
                stop_process(recorder)
    except Exception as exc:
        trial.update(status='failed', error=str(exc))
    finally:
        trial['wall_elapsed_s'] = round(time.monotonic() - started, 3)
        (out / 'trial.json').write_text(json.dumps(trial, indent=2) + '\n')
    return trial


def compare_rates(trials):
    """Pair identical seed/layout trials at rt_factor 1 and 2; no equivalence claim."""
    pairs = []
    for layout, seed in sorted({(t['layout'], t['seed']) for t in trials}):
        rates = {t['rt_factor']: t for t in trials if t['layout'] == layout and t['seed'] == seed}
        pair = dict(layout=layout, seed=seed, status='missing_completed_pair')
        if all(r in rates and rates[r]['status'] == 'completed' for r in (1., 2.)):
            a, b = (rates[r]['result'] for r in (1., 2.))
            pair.update(status='paired', rt1=a, rt2=b)
            for key in ('room_coverage_pct', 'sim_time_s', 'falls', 'walked_m'):
                if isinstance(a.get(key), (int, float)) and isinstance(b.get(key), (int, float)):
                    pair[key + '_delta_rt2_minus_rt1'] = b[key] - a[key]
            ea, eb = (x.get('slam_error_m', {}).get('mean') for x in (a, b))
            if isinstance(ea, (int, float)) and isinstance(eb, (int, float)):
                pair['slam_mean_error_delta_m'] = eb - ea
        pairs.append(pair)
    return pairs


def provenance(execute):
    repo = Path(__file__).resolve().parents[3]
    def git(*args):
        return subprocess.run(['git', '-C', str(repo), *args], capture_output=True, text=True).stdout.strip()
    sources = list((repo / 'ros2/duck_nav/duck_nav').glob('*.py')) + list((repo / 'ros2/duck_nav/launch').glob('*.py'))
    sources += [p for directory in ('policies', 'yolo', 'model') for p in (repo / directory).rglob('*')
                if p.is_file() and p.suffix in ('.onnx', '.xml', '.json', '.stl', '.STL', '.obj', '.png')]
    info = dict(repository=str(repo), revision=git('rev-parse', 'HEAD'),
                worktree_changes=git('status', '--porcelain').splitlines(),
                source_sha256={str(p.relative_to(repo)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(sources)},
                ament_prefix_path=os.environ.get('AMENT_PREFIX_PATH', ''),
                pythonpath=os.environ.get('PYTHONPATH', ''),
                ros_distro=os.environ.get('ROS_DISTRO', ''), installed_package=None)
    if execute:
        prefix = subprocess.run(['ros2', 'pkg', 'prefix', 'duck_nav'], capture_output=True, text=True)
        if prefix.returncode:
            raise ValueError('duck_nav is not available; source the built ROS workspace before --execute')
        installed = Path(prefix.stdout.strip())
        files = list(installed.glob('lib/python*/site-packages/duck_nav/*.py'))
        files += list(installed.glob('share/duck_nav/launch/*.py'))
        files += [p for directory in ('models', 'mjcf') for p in (installed / 'share/duck_nav' / directory).rglob('*') if p.is_file()]
        info['installed_package'] = dict(prefix=str(installed),
            sha256={str(p.relative_to(installed)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)})
        deps = subprocess.run(['python3', '-c',
            'import importlib.metadata as m,json,sys; print(json.dumps(dict(python=sys.version, packages={n:m.version(n) for n in ("numpy","scipy","mujoco","onnxruntime")})))'],
            capture_output=True, text=True)
        info['runtime_versions'] = deps.stdout.strip() or deps.stderr.strip()
    return info


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True, help='new evidence directory (must not already exist)')
    parser.add_argument('--layouts', type=Path, help='JSON mapping layout names to launch overrides')
    parser.add_argument('--seeds', default='0,1,2')
    parser.add_argument('--rt-factors', default='1,2')
    parser.add_argument('--duration', type=positive, default=600.)
    parser.add_argument('--workers', type=int, default=1, help='concurrent isolated ROS domains; first establish serial baseline')
    parser.add_argument('--domain-base', type=int, default=50)
    parser.add_argument('--wall-timeout', type=positive, help='per-trial wall seconds, including startup')
    parser.add_argument('--loop-closure', action='store_true')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    try:
        layouts = load_layouts(args.layouts)
        seeds = [int(s) for s in args.seeds.split(',')]
        if any(seed < 0 for seed in seeds):
            raise ValueError('seeds must be nonnegative')
        factors = [positive(s) for s in args.rt_factors.split(',')]
        if not 1 <= args.workers <= 32 or not 0 <= args.domain_base <= 100 - args.workers:
            raise ValueError('workers must be 1..32 and domain range must fit 0..99')
        trials = make_trials(layouts, seeds, factors, args.out.resolve(), args.duration, args.loop_closure)
        source_info = provenance(args.execute)
        args.out.mkdir(parents=True, exist_ok=False)
    except (ValueError, OSError, argparse.ArgumentTypeError) as exc:
        parser.error(str(exc))
    assets = {value for item in layouts.values() for key, value in item.items() if key in ('mjcf', 'world_json')}
    manifest = dict(schema_version=1, provenance=source_info, duration_s=args.duration,
                    source='repository replacement; original job2.sh / patch_ort.py unavailable',
                    asset_sha256={p: hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in sorted(assets)},
                    layouts=layouts, recorded_topics=TOPICS, workers=args.workers, trials=trials)
    manifest_path = args.out / 'matrix.json'
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    if args.execute:
        cancel = threading.Event()
        previous = {s: signal.signal(s, lambda *_: cancel.set()) for s in (signal.SIGINT, signal.SIGTERM)}
        try:
            # Partition into lanes so each concurrent process set owns a distinct domain.
            def lane(index):
                for trial in trials[index::args.workers]:
                    if cancel.is_set():
                        trial['status'] = 'cancelled'
                        continue
                    deadline = args.wall_timeout or max(180., args.duration / trial['rt_factor'] * 5 + 60)
                    run_trial(trial, args.domain_base + index, deadline, cancel)
            with ThreadPoolExecutor(max_workers=args.workers) as executor:
                list(executor.map(lane, range(args.workers)))
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
            manifest['rate_comparisons'] = compare_rates(trials)
            manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    print(manifest_path)
    return int(args.execute and any(t['status'] != 'completed' for t in trials))


if __name__ == '__main__':
    raise SystemExit(main())
