"""L1 (sensor→world transform vs CARLA's own matrix) and L3 (frame-exact collection from the sensor queue)."""

import types

import numpy as np
import pytest

from src.lidar.carla_backend import CarlaLidarHandle, _SEMANTIC_RAW
from src.lidar.config import LidarConfig
from src.lidar.geometry import sensor_to_world

POSES = [(0, 0, 0), (30, 0, 0), (30, 10, 5), (-120, -7, 12), (90, 20, -15)]  # (yaw, pitch, roll)


def _old_hand_rolled(points_xyz, loc, yaw, pitch, roll):
    """The removed collector's matrix (kept only to demonstrate defect L1)."""
    y, p, r = np.radians([yaw, pitch, roll])
    cy, sy, cp, sp, cr, sr = np.cos(y), np.sin(y), np.cos(p), np.sin(p), np.cos(r), np.sin(r)
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    return points_xyz @ R.T + np.array(loc)


@pytest.fixture(scope="module")
def carla():
    return pytest.importorskip("carla")


@pytest.mark.parametrize("yaw,pitch,roll", POSES)
def test_sensor_to_world_equals_carla_transform(carla, yaw, pitch, roll):
    t = carla.Transform(carla.Location(1.5, -2.0, 3.0), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))
    pts = np.zeros(4, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")])
    pts["x"], pts["y"], pts["z"] = [4, -1, 0, 10], [5, 2, 0, -3], [6, 7, 1, 0.5]
    out = sensor_to_world(pts, np.array(t.get_matrix()))
    for i in range(4):
        expected = t.transform(carla.Location(float(pts["x"][i]), float(pts["y"][i]), float(pts["z"][i])))
        assert (out["x"][i], out["y"][i], out["z"][i]) == pytest.approx((expected.x, expected.y, expected.z), abs=1e-4)


def test_the_removed_matrix_was_wrong_for_pitch_and_roll(carla):
    """Documents L1: right for flat ground, ~2.3 m off for a vehicle pitched 10 deg and rolled 5 deg."""
    pt = np.array([[4.0, 5.0, 6.0]])
    flat = carla.Transform(carla.Location(1, 2, 3), carla.Rotation(yaw=30))
    tilted = carla.Transform(carla.Location(1, 2, 3), carla.Rotation(pitch=10, yaw=30, roll=5))
    for t, (yaw, pitch, roll), should_match in ((flat, (30, 0, 0), True), (tilted, (30, 10, 5), False)):
        truth = t.transform(carla.Location(4, 5, 6))
        old = _old_hand_rolled(pt, (1, 2, 3), yaw, pitch, roll)[0]
        err = np.linalg.norm(old - np.array([truth.x, truth.y, truth.z]))
        assert (err < 1e-4) == should_match
    assert err > 2.0


def test_sensor_to_world_rejects_a_non_4x4_matrix():
    with pytest.raises(ValueError):
        sensor_to_world(np.zeros(1, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")]), np.eye(3))


# ── frame-exact collection (L3) with a fake CARLA sensor ─────────────────────

class _FakeSensor:
    def __init__(self):
        self.callback, self.events = None, []

    def listen(self, cb):
        self.callback = cb

    def stop(self):
        self.events.append("stop")

    def destroy(self):
        self.events.append("destroy")


def _measurement(frame, n=3):
    raw = np.zeros(n, dtype=_SEMANTIC_RAW)
    raw["x"] = frame
    raw["object_idx"] = 2**25
    tr = types.SimpleNamespace(location=types.SimpleNamespace(x=1.0, y=2.0, z=3.0),
                               rotation=types.SimpleNamespace(pitch=1.0, yaw=2.0, roll=3.0))
    return types.SimpleNamespace(frame=frame, raw_data=raw.tobytes(), transform=tr, timestamp=frame * 0.05)


def test_handle_returns_exactly_the_requested_frame_and_discards_older_ones():
    sensor = _FakeSensor()
    h = CarlaLidarHandle(sensor, LidarConfig())
    for f in (10, 11, 12):
        sensor.callback(_measurement(f))
    m = h.get_measurement(12, timeout_s=0.01)
    assert m.frame == 12 and float(m.points["x"][0]) == 12.0 and int(m.points["object_idx"][0]) == 2**25
    assert m.location == (1.0, 2.0, 3.0) and m.rotation == (1.0, 2.0, 3.0)
    assert h.get_measurement(12, timeout_s=0.01) is None            # consumed; older ones are gone


def test_handle_does_not_hand_out_a_future_frame_for_the_current_tick():
    sensor = _FakeSensor()
    h = CarlaLidarHandle(sensor, LidarConfig())
    sensor.callback(_measurement(20))
    assert h.get_measurement(19, timeout_s=0.01) is None


def test_handle_waits_for_a_late_measurement():
    import threading
    sensor = _FakeSensor()
    h = CarlaLidarHandle(sensor, LidarConfig())
    threading.Timer(0.05, lambda: sensor.callback(_measurement(5))).start()
    assert h.get_measurement(5, timeout_s=1.0).frame == 5


def test_destroy_stops_before_destroying_and_never_sleeps():
    sensor = _FakeSensor()
    CarlaLidarHandle(sensor, LidarConfig()).destroy()
    assert sensor.events == ["stop", "destroy"]
