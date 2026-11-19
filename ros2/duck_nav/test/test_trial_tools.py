"""Offline tests for trial isolation, evidence validation and simulated-time bag selection."""
import importlib.util
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import threading

import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'


def load(name):
    spec = importlib.util.spec_from_file_location(name, TOOLS / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


matrix = load('trial_matrix')
bags = load('bag_suite')


def test_matrix_pairs_seeds_layouts_and_rates_without_shell(tmp_path):
    runs = matrix.make_trials({'a': {}, 'b': {}}, [0, 2], [1, 2], tmp_path, 600)
    assert len(runs) == 8
    assert len({r['name'] for r in runs}) == 8
    assert 'rt_factor:=2' in runs[1]['command']
    assert 'seed:=2' in runs[2]['command']
    assert all('real_time:=true' in r['command'] for r in runs)
    assert len(matrix.compare_rates(runs)) == 4
    assert all(p['status'] == 'missing_completed_pair' for p in matrix.compare_rates(runs))
    with pytest.raises(ValueError, match='duplicate'):
        matrix.make_trials({'a': {}}, [0, 0], [1], tmp_path, 600)


def test_alternate_layout_requires_scoring_geometry(tmp_path):
    manifest = tmp_path / 'layouts.json'
    manifest.write_text(json.dumps({'variant': {}}))
    with pytest.raises(ValueError, match='alternate layouts'):
        matrix.load_layouts(manifest)
    manifest.write_text(json.dumps({'../escape': {}}))
    with pytest.raises(ValueError, match='invalid layout'):
        matrix.load_layouts(manifest)


def result():
    return dict(profile='realistic', exploration='TIMEOUT at 600 s (not complete)', sim_time_s=600,
                room_coverage_pct=40, falls=1, walked_m=3,
                slam_error_m=dict(samples=120, mean=.1, max=.3, final=.2))


def test_completed_execution_does_not_imply_exploration_success():
    value = result()
    assert matrix.validate_result(value) is value
    for invalid in ({}, dict(value, profile='ideal'), dict(value, slam_error_m={'samples': 0}),
                    dict(value, sim_time_s=float('nan'))):
        with pytest.raises(ValueError):
            matrix.validate_result(invalid)


def test_rate_comparison_reports_actual_delta(tmp_path):
    runs = matrix.make_trials({'default': {}}, [2], [1, 2], tmp_path, 600)
    for run in runs:
        run.update(status='completed', result=result())
    runs[1]['result']['slam_error_m']['mean'] = .15
    pair = matrix.compare_rates(runs)[0]
    assert pair['status'] == 'paired'
    assert pair['slam_mean_error_delta_m'] == pytest.approx(.05)


def test_process_start_failure_is_retained_as_trial_evidence(tmp_path, monkeypatch):
    trial = matrix.make_trials({'default': {}}, [0], [1], tmp_path, 600)[0]
    def missing(*args, **kwargs):
        raise FileNotFoundError('ros2 unavailable')
    monkeypatch.setattr(matrix.subprocess, 'Popen', missing)
    completed = matrix.run_trial(trial, 50, 1, threading.Event())
    assert completed['status'] == 'failed'
    assert 'ros2 unavailable' in completed['error']
    assert json.loads((Path(trial['out_dir']) / 'trial.json').read_text())['domain_id'] == 50


def test_clock_trim_uses_simulation_not_wall_time():
    # 2x accelerated: 600 simulated seconds occupy 300 receive seconds.
    clocks = [(int(1000e9 + t * .5e9), int(t * 1e9)) for t in range(1001)]
    begin, end = bags.clock_bounds(clocks, 0, 600)
    assert begin is None
    assert end == int(1300e9)
    assert bags.clock_bounds(clocks, 100, 600) == (int(1050e9), int(1350e9))
    assert bags.clock_bounds(clocks, 0, 1200)[1] is None
    with pytest.raises(ValueError, match='beyond'):
        bags.clock_bounds(clocks, 1001, 10)


@pytest.mark.parametrize('clocks', [[], [(0, 3), (1, 2)], [(2, 1), (1, 2)]])
def test_clock_reset_empty_and_out_of_order_rejected(clocks):
    with pytest.raises(ValueError):
        bags.clock_bounds(clocks, 0, 600)


def test_inventory_counts_split_sqlite_topics_and_missing_files(tmp_path):
    for i in range(2):
        with sqlite3.connect(tmp_path / f'bag_{i}.db3') as connection:
            connection.executescript('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT); '
                'CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER); '
                'INSERT INTO topics VALUES(1,"/clock","rosgraph_msgs/msg/Clock"); '
                f'INSERT INTO messages VALUES(1,1,{i * 1000000000});')
    report = bags.inventory(tmp_path)
    assert report['topics']['/clock']['messages'] == 2
    assert report['receive_duration_s'] == 1
    assert '/imu' in report['required_topics_missing']
    assert len(report['files']) == 2
    assert bags.inventory(tmp_path / 'absent')['status'] == 'missing'


def test_missing_replay_set_is_explicit_and_non_success(tmp_path):
    manifest = tmp_path / 'bags.json'
    manifest.write_text(json.dumps({'bags': [{'name': 'seed2', 'path': 'missing_bag'}]}))
    report = bags.replay_set(manifest, tmp_path / 'out', 600, False)
    assert report['results'][0]['status'] == 'missing'
    assert (tmp_path / 'out/replay_set.json').is_file()


def test_cli_plan_runs_without_ros_and_does_not_overwrite(tmp_path):
    cmd = [sys.executable, str(TOOLS / 'trial_matrix.py'), '--out', str(tmp_path / 'plan')]
    planned = subprocess.run(cmd, capture_output=True, text=True)
    assert planned.returncode == 0, planned.stderr
    manifest = json.loads((tmp_path / 'plan/matrix.json').read_text())
    assert len(manifest['trials']) == 6
    assert all(t['status'] == 'planned' for t in manifest['trials'])
    assert manifest['provenance']['installed_package'] is None
    assert subprocess.run(cmd, capture_output=True).returncode != 0


def test_replay_shared_defaults_recovery_and_machine_metrics(tmp_path, monkeypatch):
    """Exercise the replay pipeline with ROS serialization shims, real SLAM and scripted tilt."""
    import runpy
    import types
    import numpy as np
    monkeypatch.syspath_prepend(str(TOOLS.parent))
    import duck_nav.duckslam as ds
    ns = types.SimpleNamespace
    def header(t):
        return ns(stamp=ns(sec=int(t), nanosec=round((t % 1) * 1e9)))
    def pose(t):
        return ns(header=header(t), pose=ns(position=ns(x=0., y=0., z=.2), orientation=ns(w=1., x=0., y=0., z=0.)))
    records = []
    for i in range(101):
        t = i * .02
        rx = round((1000 + t) * 1e9)
        records.extend([('/clock', ns(clock=header(t).stamp), rx),
                        ('/imu', ns(header=header(t), angular_velocity=ns(x=0., y=0., z=0.),
                                    linear_acceleration=ns(x=0., y=0., z=9.81)), rx),
                        ('/ground_truth/pose', pose(t), rx)])
        if i % 5 == 0:
            records.extend([('/tof/zones', ns(header=header(t)), rx), ('/tof/pose_level', pose(t), rx)])
    records.append(('/cmd_vel', ns(linear=ns(x=0., y=0.)), int(1000e9)))
    records.sort(key=lambda r: r[2])
    class Reader:
        def open(self, *args):
            self.index = 0
        def get_all_topics_and_types(self):
            return [ns(name=n, type=n) for n in {r[0] for r in records}]
        def set_filter(self, unused):
            pass
        def has_next(self):
            return self.index < len(records)
        def read_next(self):
            item = records[self.index]
            self.index += 1
            return item
    monkeypatch.setitem(sys.modules, 'rosbag2_py', ns(SequentialReader=Reader,
        StorageOptions=lambda **kw: None, ConverterOptions=lambda *a: None, StorageFilter=lambda **kw: None))
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', ns(deserialize_message=lambda msg, typ: msg))
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.utilities', ns(get_message=lambda typ: typ))
    monkeypatch.setitem(sys.modules, 'sensor_msgs_py', ns(point_cloud2=ns(read_points=lambda *a, **kw: [(1., 0, z) for z in range(64)])))
    class ScriptedTilt:
        def __init__(self):
            self.count = 0
        def update(self, *args):
            self.count += 1
            return np.array([0., 0., 1.])
        def tilt(self):
            return np.deg2rad(40 if self.count == 2 else 0)
        def heading_rate(self, gyro, dt):
            return 0.0
    monkeypatch.setattr(ds, 'make_attitude_filter', lambda mode='quaternion': ScriptedTilt())
    output = tmp_path / 'replay.json'
    monkeypatch.setattr(sys, 'argv', ['slam_replay.py', 'fake_ros_bag', '--output', str(output)])
    runpy.run_path(str(TOOLS / 'slam_replay.py'), run_name='__main__')
    metric = json.loads(output.read_text())['results'][0]
    assert metric['policy'] == 'shared'
    assert metric['effective_config']['min_hits'] == 8
    assert metric['effective_config']['fresh'] is False
    assert metric['recovery_entries'] == 1
    assert 0 < metric['frames'] < 21  # frames during fall and upright hold were suspended
    assert metric['samples'] > 0
    assert metric['graph_revision'] == 0
