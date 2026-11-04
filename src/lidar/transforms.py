"""Pose transforms without a CARLA import (pure numpy).

``transform_matrix`` reproduces ``carla.Transform.get_matrix()``: the rotation columns are the object's forward,
right and up axes in world coordinates (CARLA / Unreal: left-handed, x forward, y right, z up, degrees, yaw turns
x towards y) and the last column is the location:

    forward = ( cp*cy,                cp*sy,                sp    )
    right   = ( cy*sp*sr - sy*cr,     sy*sp*sr + cy*cr,    -cp*sr )
    up      = (-cy*sp*cr - sy*sr,    -sy*sp*cr + cy*sr,     cp*cr )

(c/s = cos/sin of p = pitch, y = yaw, r = roll.) ``tests/unit/test_lidar_transforms.py`` compares it against
``carla.Transform.get_matrix()`` / ``transform()`` when the carla module is installed. The server itself uses
``src/lidar/geometry.py`` with CARLA's own matrix; this module exists for code without CARLA (the browser
fixtures in ``tests/unit/test_js_modules.py``, the fake world, offline analysis of recorded frames, whose pose
is in the frame header).
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]


def basis_vectors(yaw_deg: float, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> Tuple[Vec3, Vec3, Vec3]:
    """(forward, right, up) unit vectors of a rotation in world coordinates."""
    y, p, r = math.radians(yaw_deg), math.radians(pitch_deg), math.radians(roll_deg)
    cy, sy, cp, sp, cr, sr = math.cos(y), math.sin(y), math.cos(p), math.sin(p), math.cos(r), math.sin(r)
    forward = (cp * cy, cp * sy, sp)
    right = (cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, -cp * sr)
    up = (-cy * sp * cr - sy * sr, -sy * sp * cr + cy * sr, cp * cr)
    return forward, right, up


def transform_matrix(location: Sequence[float], rotation_pyr_deg: Sequence[float]) -> np.ndarray:
    """4x4 local -> world matrix of a pose; ``rotation_pyr_deg`` = (pitch, yaw, roll) in degrees (CARLA order,
    the order of the LiDAR v3 pose block)."""
    pitch, yaw, roll = rotation_pyr_deg
    forward, right, up = basis_vectors(yaw, pitch, roll)
    m = np.eye(4, dtype=np.float64)
    m[:3, 0], m[:3, 1], m[:3, 2] = forward, right, up
    m[:3, 3] = location
    return m


def to_world(points_xyz: np.ndarray, location: Sequence[float], rotation_pyr_deg: Sequence[float]) -> np.ndarray:
    """Nx3 points in the frame of the pose -> world coordinates (float64)."""
    m = transform_matrix(location, rotation_pyr_deg)
    return np.asarray(points_xyz, dtype=np.float64) @ m[:3, :3].T + m[:3, 3]


def to_sensor(points_xyz: np.ndarray, location: Sequence[float], rotation_pyr_deg: Sequence[float]) -> np.ndarray:
    """Inverse of ``to_world`` (the rotation is orthonormal: inverse = transpose)."""
    m = transform_matrix(location, rotation_pyr_deg)
    return (np.asarray(points_xyz, dtype=np.float64) - m[:3, 3]) @ m[:3, :3]


def compose(parent_loc: Sequence[float], parent_rot_pyr: Sequence[float], child_loc: Sequence[float],
            child_rot_pyr: Sequence[float]) -> Tuple[np.ndarray, np.ndarray]:
    """World pose of a child given in its parent's frame (a sensor mount): (location xyz, 3x3 rotation)."""
    m = transform_matrix(parent_loc, parent_rot_pyr) @ transform_matrix(child_loc, child_rot_pyr)
    return m[:3, 3], m[:3, :3]
