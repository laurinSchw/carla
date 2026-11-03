"""LiDAR protocol v3: round trip, compression, malformed input (handoff §5.4, defects L2/L8)."""

import struct

import numpy as np
import pytest

from src.lidar import protocol
from src.lidar.protocol import ProtocolError

KW = dict(frame=1234, sim_time=61.7, wall_time=1.7e9, ego_actor_id=77, sensor_id=3,
          location=(10.0, -20.5, 2.4), rotation=(1.0, 90.0, -2.0))


def _semantic(n=50):
    pts = np.zeros(n, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("cos_inc", "f4"),
                             ("object_idx", "u4"), ("tag", "u4")])
    rng = np.random.default_rng(1)
    for f in ("x", "y", "z", "cos_inc"):
        pts[f] = rng.normal(size=n)
    pts["object_idx"] = rng.integers(0, 2**31, n)  # far above 2**24
    pts["tag"] = rng.integers(0, 30, n)
    return pts


def test_round_trip_semantic_frame():
    pts = _semantic()
    h, out = protocol.decode(protocol.encode(pts, field_mask=protocol.SEMANTIC_FIELDS, **KW))
    assert (h.frame, h.sim_time, h.sensor_id, h.ego_actor_id, h.num_points) == (1234, 61.7, 3, 77, 50)
    assert h.location == (10.0, -20.5, 2.4) and h.rotation == (1.0, 90.0, -2.0)
    assert h.sensor_frame and not h.compressed
    for f in ("x", "y", "z", "cos_inc", "object_idx", "tag"):
        assert np.array_equal(out[f], pts[f].astype(out[f].dtype)), f


def test_object_idx_is_exact_above_2_pow_24():
    """L8: float32 object_idx lost actor ids > 16 777 216."""
    pts = _semantic(1)
    pts["object_idx"] = 2**24 + 1
    _, out = protocol.decode(protocol.encode(pts, field_mask=protocol.SEMANTIC_FIELDS, **KW))
    assert int(out["object_idx"][0]) == 2**24 + 1


def test_raycast_fields_and_stride():
    pts = np.zeros(4, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("intensity", "f4")])
    pts["intensity"] = [0.1, 0.2, 0.3, 0.4]
    data = protocol.encode(pts, field_mask=protocol.RAYCAST_FIELDS, **KW)
    h, out = protocol.decode(data)
    assert h.stride == 16 and len(data) == protocol.HEADER_SIZE + 4 * 16
    assert np.allclose(out["intensity"], pts["intensity"])
    assert out.dtype.names == ("x", "y", "z", "intensity")


def test_compression_round_trip_and_smaller_for_repetitive_data():
    pts = _semantic(2000)
    pts["x"] = 1.0
    raw = protocol.encode(pts, field_mask=protocol.SEMANTIC_FIELDS, **KW)
    packed = protocol.encode(pts, field_mask=protocol.SEMANTIC_FIELDS, compress=True, **KW)
    assert len(packed) < len(raw)
    h, out = protocol.decode(packed)
    assert h.compressed and np.array_equal(out["tag"], pts["tag"])


def test_empty_frame():
    pts = np.zeros(0, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("cos_inc", "f4"), ("object_idx", "u4"), ("tag", "u4")])
    h, out = protocol.decode(protocol.encode(pts, field_mask=protocol.SEMANTIC_FIELDS, **KW))
    assert h.num_points == 0 and len(out) == 0


def test_world_frame_flag():
    pts = _semantic(2)
    h, _ = protocol.decode(protocol.encode(pts, field_mask=protocol.SEMANTIC_FIELDS, world_frame=True, **KW))
    assert not h.sensor_frame and h.flags & protocol.FLAG_WORLD_FRAME


@pytest.mark.parametrize("mutate,message", [
    (lambda b: b"XXXX" + b[4:], "magic"),
    (lambda b: b[:4] + struct.pack("<H", 2) + b[6:], "version"),
    (lambda b: b[:10], "too short"),
    (lambda b: b[:-3], "point block"),
    (lambda b: b[:6] + struct.pack("<H", 0) + b[8:], "flags"),
    (lambda b: b[:6] + struct.pack("<H", 6) + b[8:], "flags"),
    (lambda b: b[:46] + struct.pack("<H", 99) + b[48:], "field_mask"),
])
def test_malformed_frames_are_rejected(mutate, message):
    good = protocol.encode(_semantic(3), field_mask=protocol.SEMANTIC_FIELDS, **KW)
    with pytest.raises(ProtocolError, match=message):
        protocol.decode(mutate(good))


def test_missing_field_is_rejected():
    pts = np.zeros(2, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")])
    with pytest.raises(ProtocolError, match="lack"):
        protocol.encode(pts, field_mask=protocol.SEMANTIC_FIELDS, **KW)


def test_header_layout_is_fixed():
    """The JS decoder mirrors these offsets: any change here is a protocol version bump."""
    data = protocol.encode(_semantic(1), field_mask=protocol.SEMANTIC_FIELDS, **KW)
    assert protocol.HEADER_SIZE == 96
    assert data[:4] == b"LIDR"
    assert struct.unpack_from("<H", data, 4)[0] == 3                 # version
    assert struct.unpack_from("<H", data, 6)[0] == protocol.FLAG_SENSOR_FRAME
    assert struct.unpack_from("<Q", data, 8)[0] == 1234              # frame
    assert struct.unpack_from("<d", data, 16)[0] == 61.7             # sim_time
    assert struct.unpack_from("<d", data, 24)[0] == 1.7e9            # wall_time
    assert struct.unpack_from("<I", data, 32)[0] == 1                # num_points
    assert struct.unpack_from("<I", data, 36)[0] == 77               # ego_actor_id
    assert struct.unpack_from("<I", data, 40)[0] == 3                # sensor_id
    assert struct.unpack_from("<H", data, 44)[0] == 12 + 1 + 4 + 4   # stride: xyz f32, tag u8, object_idx u32, cos_inc f32
    assert struct.unpack_from("<H", data, 46)[0] == protocol.SEMANTIC_FIELDS
    assert struct.unpack_from("<6d", data, 48) == (10.0, -20.5, 2.4, 1.0, 90.0, -2.0)  # location, then pitch/yaw/roll


def test_field_bits_are_the_web_decoders():
    """The bit assignment of static/js/lidar-decoder.js (the UI is the contract)."""
    assert (protocol.FIELD_XYZ, protocol.FIELD_TAG_U8, protocol.FIELD_TAG_U32, protocol.FIELD_OBJECT_IDX,
            protocol.FIELD_INTENSITY, protocol.FIELD_COS_INC) == (1, 2, 4, 8, 16, 32)
    assert protocol.SEMANTIC_FIELDS == 43 and protocol.RAYCAST_FIELDS == 17


def test_tag_u32_round_trip_and_both_tag_widths_rejected():
    pts = _semantic(5)
    pts["tag"] = [0, 255, 256, 70000, 2**31]
    mask = protocol.FIELD_XYZ | protocol.FIELD_TAG_U32 | protocol.FIELD_OBJECT_IDX
    h, out = protocol.decode(protocol.encode(pts, field_mask=mask, **KW))
    assert h.stride == 12 + 4 + 4 and list(out["tag"]) == [0, 255, 256, 70000, 2**31]
    with pytest.raises(ProtocolError, match="mutually exclusive"):
        protocol.point_dtype(protocol.FIELD_XYZ | protocol.FIELD_TAG_U8 | protocol.FIELD_TAG_U32)
