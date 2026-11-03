"""LiDAR binary protocol v4 (handoff §5.4). Little-endian.

    Header (48 bytes)  ``<4sHHQddIIIHH``
        magic b'LIDR' | version=4 | flags | frame | sim_time | wall_time_unix |
        num_points | ego_actor_id | sensor_id | point_stride_bytes | field_mask
    Pose (48 bytes)    ``<6d``  sensor location x, y, z (m) and rotation pitch, yaw, roll (deg, CARLA order),
                       both in the WORLD frame at the measurement
    Points             ``num_points`` records of ``point_stride_bytes`` each, fields in this fixed order when
                       their ``field_mask`` bit is set:
        bit0 x, y, z        float32 x3
        bit1 tag (TAG_U8)   uint8     semantic tag (sensor.lidar.ray_cast_semantic)
        bit2 tag (TAG_U32)  uint32    semantic tag when one does not fit in a byte (never together with bit1)
        bit3 object_idx     uint32    hit actor id (exact; the v1/v2 float32 loses ids > 2**24, defect L8)
        bit4 intensity      float32   (sensor.lidar.ray_cast)
        bit5 cos_inc        float32   cosine of the incidence angle (semantic)

The bit assignment is the one of the web UI's decoder (``static/js/lidar-decoder.js``, ported from the archived
branch); until 2026-11-03 this server used tag=bit1, object_idx=bit2, intensity=bit3, cos_inc=bit4 under the same
version number (see docs/science/CHANGELOG.md; recorded runs say which layout they use in ``manifest.json``).

``flags``: bit0 = point block is zlib-compressed, bit1 = points are in the WORLD frame,
bit2 = points are in the SENSOR frame. Exactly one of bit1/bit2 is set; this server sends the sensor frame
(raw as CARLA delivers it) and the pose, so the client applies the pose and recorded data stays raw.

``src/visualization/web/static/js/lidar-decoder.js`` mirrors it; tests/unit/test_js_modules.py decodes frames made
here with it.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from typing import Tuple

import numpy as np

MAGIC = b"LIDR"
VERSION = 4

FLAG_ZLIB = 1
FLAG_WORLD_FRAME = 2
FLAG_SENSOR_FRAME = 4

FIELD_XYZ = 1
FIELD_TAG_U8 = 2
FIELD_TAG_U32 = 4
FIELD_OBJECT_IDX = 8
FIELD_INTENSITY = 16
FIELD_COS_INC = 32
FIELD_TAG = FIELD_TAG_U8
ALL_FIELDS = FIELD_XYZ | FIELD_TAG_U8 | FIELD_TAG_U32 | FIELD_OBJECT_IDX | FIELD_INTENSITY | FIELD_COS_INC

#: names the field layout in recorded-run manifests (the bit assignment above)
FIELD_LAYOUT_ID = "xyz=1,tag_u8=2,tag_u32=4,object_idx=8,intensity=16,cos_inc=32"

#: field mask of sensor.lidar.ray_cast_semantic and sensor.lidar.ray_cast measurements
SEMANTIC_FIELDS = FIELD_XYZ | FIELD_TAG | FIELD_OBJECT_IDX | FIELD_COS_INC
RAYCAST_FIELDS = FIELD_XYZ | FIELD_INTENSITY

_HEADER = struct.Struct("<4sHHQddIIIHH")
_POSE = struct.Struct("<6d")
HEADER_SIZE = _HEADER.size + _POSE.size  # 96

_FIELD_LAYOUT = (
    (FIELD_XYZ, (("x", "<f4"), ("y", "<f4"), ("z", "<f4"))),
    (FIELD_TAG_U8, (("tag", "u1"),)),
    (FIELD_TAG_U32, (("tag", "<u4"),)),
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
    if field_mask & FIELD_TAG_U8 and field_mask & FIELD_TAG_U32:
        raise ProtocolError(f"invalid field_mask {field_mask:#x}: TAG_U8 and TAG_U32 are mutually exclusive")
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
    """Pack ``points`` (a structured array with at least the fields of ``field_mask``) into one v4 frame."""
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
    """Parse one v4 frame into (header, structured point array). Raises ``ProtocolError`` on bad input."""
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
