"""Geometry types with the CARLA Python API shape (only what this project uses).

Axes as in CARLA: x forward/east, y right/south, z up; angles in degrees; yaw rotates x towards y.
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from src.lidar.transforms import transform_matrix


class Vector3D:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    def length(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def __iter__(self):
        return iter((self.x, self.y, self.z))

    def __repr__(self) -> str:
        return f"Vector3D({self.x:.3f}, {self.y:.3f}, {self.z:.3f})"


class Location(Vector3D):
    def distance(self, other: "Location") -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2)

    def __add__(self, other: Vector3D) -> "Location":
        return Location(self.x + other.x, self.y + other.y, self.z + other.z)


class Rotation:
    def __init__(self, pitch: float = 0.0, yaw: float = 0.0, roll: float = 0.0):
        self.pitch, self.yaw, self.roll = float(pitch), float(yaw), float(roll)

    def __repr__(self) -> str:
        return f"Rotation(pitch={self.pitch:.2f}, yaw={self.yaw:.2f}, roll={self.roll:.2f})"


class Transform:
    def __init__(self, location: Location = None, rotation: Rotation = None):  # type: ignore[assignment]
        self.location = location or Location()
        self.rotation = rotation or Rotation()

    def get_matrix(self):
        return transform_matrix((self.location.x, self.location.y, self.location.z),
                                (self.rotation.pitch, self.rotation.yaw, self.rotation.roll)).tolist()

    def _axes(self):
        m = np.array(self.get_matrix())
        return m[:3, 0], m[:3, 1], m[:3, 2]

    def get_forward_vector(self) -> Vector3D:
        return Vector3D(*self._axes()[0])

    def get_right_vector(self) -> Vector3D:
        return Vector3D(*self._axes()[1])

    def get_up_vector(self) -> Vector3D:
        return Vector3D(*self._axes()[2])

    def transform(self, loc: Location) -> Location:
        m = np.array(self.get_matrix())
        p = m @ np.array([loc.x, loc.y, loc.z, 1.0])
        return Location(*p[:3])

    def __repr__(self) -> str:
        return f"Transform({self.location}, {self.rotation})"


class BoundingBox:
    def __init__(self, location: Location = None, extent: Vector3D = None):  # type: ignore[assignment]
        self.location = location or Location()
        self.extent = extent or Vector3D()
        self.rotation = Rotation()


def yaw_from_vector(dx: float, dy: float) -> float:
    return math.degrees(math.atan2(dy, dx))


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def xy(t: Transform) -> Tuple[float, float]:
    return (t.location.x, t.location.y)
