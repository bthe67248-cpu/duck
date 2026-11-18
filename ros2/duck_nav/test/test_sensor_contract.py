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
