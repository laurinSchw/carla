"""LiDAR binary protocol v3 (handoff §5.4). Little-endian.

    Header (48 bytes)  ``<4sHHQddIIIHH``
        magic b'LIDR' | version=3 | flags | frame | sim_time | wall_time_unix |
        num_points | ego_actor_id | sensor_id | point_stride_bytes | field_mask
    Pose (48 bytes)    ``<6d``  sensor location x, y, z (m) and rotation pitch, yaw, roll (deg, CARLA order),
                       both in the WORLD frame at the measurement
    Points             ``num_points`` records of ``point_stride_bytes`` each, fields in this fixed order when
                       their ``field_mask`` bit is set:
        bit0 x, y, z        float32 x3
        bit1 tag            uint8     semantic tag (sensor.lidar.ray_cast_semantic)
        bit2 object_idx     uint32    hit actor id (exact; the v1/v2 float32 loses ids > 2**24, defect L8)
        bit3 intensity      float32   (sensor.lidar.ray_cast)
        bit4 cos_inc        float32   cosine of the incidence angle (semantic)

``flags``: bit0 = point block is zlib-compressed, bit1 = points are in the WORLD frame,
bit2 = points are in the SENSOR frame. Exactly one of bit1/bit2 is set; this server sends the sensor frame
(raw as CARLA delivers it) and the pose, so the client applies the pose and recorded data stays raw.

The Python side is the specification; ``web/static/js/lidar-decoder.js`` (Phase 6) mirrors it and must pass
the same fixture.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from typing import Tuple

import numpy as np

MAGIC = b"LIDR"
VERSION = 3

FLAG_ZLIB = 1
FLAG_WORLD_FRAME = 2
FLAG_SENSOR_FRAME = 4

FIELD_XYZ = 1
FIELD_TAG = 2
FIELD_OBJECT_IDX = 4
FIELD_INTENSITY = 8
FIELD_COS_INC = 16
ALL_FIELDS = FIELD_XYZ | FIELD_TAG | FIELD_OBJECT_IDX | FIELD_INTENSITY | FIELD_COS_INC

#: field mask of sensor.lidar.ray_cast_semantic and sensor.lidar.ray_cast measurements
SEMANTIC_FIELDS = FIELD_XYZ | FIELD_TAG | FIELD_OBJECT_IDX | FIELD_COS_INC
RAYCAST_FIELDS = FIELD_XYZ | FIELD_INTENSITY

_HEADER = struct.Struct("<4sHHQddIIIHH")
_POSE = struct.Struct("<6d")
HEADER_SIZE = _HEADER.size + _POSE.size  # 96

_FIELD_LAYOUT = (
    (FIELD_XYZ, (("x", "<f4"), ("y", "<f4"), ("z", "<f4"))),
    (FIELD_TAG, (("tag", "u1"),)),
    (FIELD_OBJECT_IDX, (("object_idx", "<u4"),)),
    (FIELD_INTENSITY, (("intensity", "<f4"),)),
    (FIELD_COS_INC, (("cos_inc", "<f4"),)),
)


class ProtocolError(ValueError):
    """Malformed or unsupported LiDAR frame."""


def point_dtype(field_mask: int) -> np.dtype:
    """Packed (no padding) record layout for ``field_mask``."""
    if field_mask & ~ALL_FIELDS or not field_mask & FIELD_XYZ:
        raise ProtocolError(f"invalid field_mask {field_mask:#x}: xyz is mandatory, unknown bits are not allowed")
    fields = [f for bit, group in _FIELD_LAYOUT if field_mask & bit for f in group]
    return np.dtype(fields)


@dataclass(frozen=True)
class LidarHeader:
    flags: int
    frame: int
    sim_time: float
    wall_time: float
    num_points: int
    ego_actor_id: int
    sensor_id: int
    stride: int
    field_mask: int
    location: Tuple[float, float, float]
    rotation: Tuple[float, float, float]  # pitch, yaw, roll (deg)

    @property
    def compressed(self) -> bool:
        return bool(self.flags & FLAG_ZLIB)

    @property
    def sensor_frame(self) -> bool:
        return bool(self.flags & FLAG_SENSOR_FRAME)


def encode(
    points: np.ndarray,
    *,
    frame: int,
    sim_time: float,
    wall_time: float,
    ego_actor_id: int,
    sensor_id: int,
    location: Tuple[float, float, float],
    rotation: Tuple[float, float, float],
    field_mask: int = SEMANTIC_FIELDS,
    world_frame: bool = False,
    compress: bool = False,
) -> bytes:
    """Pack ``points`` (a structured array with at least the fields of ``field_mask``) into one v3 frame."""
    dtype = point_dtype(field_mask)
    missing = [n for n in dtype.names if n not in (points.dtype.names or ())]
    if missing:
        raise ProtocolError(f"points lack the fields required by field_mask: {missing}")
    packed = np.empty(len(points), dtype=dtype)
    for name in dtype.names:
        packed[name] = points[name]
    body = packed.tobytes()
    flags = FLAG_WORLD_FRAME if world_frame else FLAG_SENSOR_FRAME
    if compress:
        body = zlib.compress(body, 1)
        flags |= FLAG_ZLIB
    header = _HEADER.pack(
        MAGIC, VERSION, flags, frame, sim_time, wall_time,
        len(points), ego_actor_id, sensor_id, dtype.itemsize, field_mask,
    )
    return header + _POSE.pack(*location, *rotation) + body


def decode(data: bytes) -> Tuple[LidarHeader, np.ndarray]:
    """Parse one v3 frame into (header, structured point array). Raises ``ProtocolError`` on bad input."""
    if len(data) < HEADER_SIZE:
        raise ProtocolError(f"frame too short: {len(data)} < {HEADER_SIZE} bytes")
    (magic, version, flags, frame, sim_time, wall_time, n_points, ego, sensor_id, stride, mask) = _HEADER.unpack_from(data, 0)
    if magic != MAGIC:
        raise ProtocolError(f"bad magic {magic!r}")
    if version != VERSION:
        raise ProtocolError(f"unsupported version {version} (expected {VERSION})")
    if bool(flags & FLAG_WORLD_FRAME) == bool(flags & FLAG_SENSOR_FRAME):
        raise ProtocolError("exactly one of the world-frame / sensor-frame flags must be set")
    dtype = point_dtype(mask)
    if stride != dtype.itemsize:
        raise ProtocolError(f"point_stride_bytes {stride} does not match field_mask (expects {dtype.itemsize})")
    pose = _POSE.unpack_from(data, _HEADER.size)
    body = data[HEADER_SIZE:]
    if flags & FLAG_ZLIB:
        try:
            body = zlib.decompress(body)
        except zlib.error as exc:
            raise ProtocolError(f"corrupt compressed point block: {exc}") from exc
    if len(body) != n_points * stride:
        raise ProtocolError(f"point block is {len(body)} bytes, header promises {n_points} x {stride}")
    points = np.frombuffer(body, dtype=dtype).copy()
    header = LidarHeader(flags, frame, sim_time, wall_time, n_points, ego, sensor_id, stride, mask,
                         tuple(pose[:3]), tuple(pose[3:]))
    return header, points
