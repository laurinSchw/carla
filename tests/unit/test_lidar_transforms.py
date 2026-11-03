import math
import random

import numpy as np
import pytest


from src.lidar.transforms import compose, to_sensor, to_world, transform_matrix


def test_identity_pose_is_the_identity():
    p = np.array([[1.0, 2.0, 3.0], [-4.0, 0.5, 9.0]])
    assert np.allclose(to_world(p, (0, 0, 0), (0, 0, 0)), p)


def test_translation_only():
    assert np.allclose(to_world(np.array([[1.0, 0, 0]]), (10, 20, 30), (0, 0, 0)), [[11, 20, 30]])


@pytest.mark.parametrize("yaw,local,world", [
    (90.0, (1, 0, 0), (0, 1, 0)),      # forward axis points to +y
    (90.0, (0, 1, 0), (-1, 0, 0)),     # right axis points to -x
    (180.0, (1, 2, 0), (-1, -2, 0)),
    (-90.0, (1, 0, 0), (0, -1, 0)),
])
def test_yaw_rotations(yaw, local, world):
    assert np.allclose(to_world(np.array([local], float), (0, 0, 0), (0, yaw, 0)), [world], atol=1e-12)


def test_pitch_up_lifts_forward_points():
    out = to_world(np.array([[10.0, 0, 0]]), (0, 0, 0), (30.0, 0, 0))
    assert np.allclose(out, [[10 * math.cos(math.radians(30)), 0, 10 * 0.5]])


def test_roundtrip_for_random_poses():
    rng = random.Random(3)
    for _ in range(100):
        loc = (rng.uniform(-500, 500), rng.uniform(-500, 500), rng.uniform(0, 30))
        rot = (rng.uniform(-40, 40), rng.uniform(-180, 180), rng.uniform(-40, 40))
        pts = np.array([[rng.uniform(-50, 50) for _ in range(3)] for _ in range(20)])
        assert np.allclose(to_sensor(to_world(pts, loc, rot), loc, rot), pts, atol=1e-9)


def test_rotation_preserves_distances():
    rng = random.Random(4)
    pts = np.array([[rng.uniform(-50, 50) for _ in range(3)] for _ in range(50)])
    out = to_world(pts, (7, 8, 9), (12.0, 77.0, -20.0))
    assert np.allclose(np.linalg.norm(out - [7, 8, 9], axis=1), np.linalg.norm(pts, axis=1))


def test_matrix_is_orthonormal_with_determinant_one():
    m = transform_matrix((1, 2, 3), (15.0, 40.0, -25.0))
    r = m[:3, :3]
    assert np.allclose(r.T @ r, np.eye(3)) and np.isclose(np.linalg.det(r), 1.0)


def test_pitched_pose_differs_from_the_old_hand_rolled_matrix():
    """L1: the old collector's third row was [-sin p, cos p sin r, cos p cos r] (right-handed Z-Y-X
    Euler); CARLA's is [sin p, -cos p sin r, cos p cos r]. They agree only for pitch = roll = 0."""
    yaw, pitch, roll = map(math.radians, (30.0, 10.0, 5.0))
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    old = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                    [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                    [-sp, cp*sr, cp*cr]])
    new = transform_matrix((0, 0, 0), (10.0, 30.0, 5.0))[:3, :3]
    assert not np.allclose(old, new, atol=1e-6)
    flat_old = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    assert np.allclose(transform_matrix((0, 0, 0), (0.0, 30.0, 0.0))[:3, :3], flat_old)   # agree when level


def test_compose_places_a_mounted_sensor_on_a_turned_vehicle():
    loc, rot = compose((100, 50, 0.3), (0, 90.0, 0), (1.0, 0.0, 2.4), (0, 0, 0))
    assert np.allclose(loc, [100, 51, 2.7])        # 1 m ahead of a vehicle that faces +y, 2.4 m up
    assert np.allclose(rot @ [1, 0, 0], [0, 1, 0])


@pytest.mark.parametrize("pyr", [(0.0, 0.0, 0.0), (10.0, 30.0, 5.0), (-35.0, -120.0, 20.0), (90.0, 90.0, 90.0), (-7.0, 200.0, -45.0)])
def test_matrix_equals_carla_transform(pyr):
    """The pure-numpy matrix is CARLA's own (``carla.Transform.get_matrix`` / ``transform``); skipped without carla."""
    carla = pytest.importorskip("carla")
    loc = (1.5, -2.0, 3.0)
    t = carla.Transform(carla.Location(*loc), carla.Rotation(pitch=pyr[0], yaw=pyr[1], roll=pyr[2]))
    assert np.allclose(np.array(t.get_matrix()), transform_matrix(loc, pyr), atol=1e-6)
    local = (4.0, -1.0, 6.0)
    expected = t.transform(carla.Location(*local))   # note: transform() also mutates its argument in place
    assert np.allclose(to_world(np.array([local]), loc, pyr)[0], [expected.x, expected.y, expected.z], atol=1e-4)
