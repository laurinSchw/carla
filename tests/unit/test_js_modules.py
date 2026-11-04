"""Run the Node unit tests of the browser modules (tests/js/*.test.mjs) against fixtures written by the Python side.

Python writes what the JS side must agree with: poses and transformed points from ``src.lidar.transforms`` (itself
checked against ``carla.Transform``), LiDAR v4 frames from ``src.lidar.protocol``, the JSON schemas the forms are
generated from (``config_schema``, ``lidar_schema``) and the DENM cause codes. So the two languages cannot drift
apart silently. Skipped without Node (e.g. in the Docker image).
"""

import json
import os
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from src.lidar import protocol
from src.lidar.config import lidar_schema
from src.lidar.transforms import to_world, transform_matrix
from src.server.world_config import config_schema
from src.v2x.safety_messages import DENMCauseCode

ROOT = Path(__file__).resolve().parents[2]
NODE = shutil.which("node")

pytestmark = pytest.mark.skipif(NODE is None, reason="node not installed")


def _encode(points: dict, mask: int, *, ego: int, loc, pyr, frame: int = 77, sim_time: float = 3.85,
            compress: bool = False) -> bytes:
    """Encode a dict of equal-length arrays with OUR encoder (protocol.encode takes a structured array)."""
    dtype = protocol.point_dtype(mask)
    arr = np.zeros(len(points["x"]), dtype=dtype)
    for name in dtype.names:
        arr[name] = points[name]
    return protocol.encode(arr, frame=frame, sim_time=sim_time, wall_time=1.0, ego_actor_id=ego, sensor_id=3,
                           location=loc, rotation=pyr, field_mask=mask, compress=compress)


def write_fixtures(path: Path) -> None:
    rng = np.random.default_rng(11)
    poses = [((1.5, -2.0, 0.5), (0.0, 0.0, 0.0)), ((100.0, 50.0, 2.4), (10.0, 30.0, 5.0)),
             ((-5.0, 7.0, 1.0), (-35.0, -120.0, 20.0)), ((0.0, 0.0, 0.0), (90.0, 90.0, 90.0))]
    poses += [(tuple(rng.uniform(-200, 200, 3)), tuple(rng.uniform(-80, 80, 3))) for _ in range(12)]
    out = []
    for loc, pyr in poses:
        pts = rng.uniform(-30, 30, (6, 3)).astype(np.float32)
        out.append({"location": list(map(float, loc)), "pyr": list(map(float, pyr)),
                    "matrix": transform_matrix(loc, pyr).reshape(-1).tolist(),
                    "points": pts.tolist(), "world": to_world(pts, loc, pyr).tolist()})
    (path / "coords.json").write_text(json.dumps({"poses": out}))
    write_lidar_frames(path)
    (path / "world_config_schema.json").write_text(json.dumps(config_schema()))
    (path / "denm_causes.json").write_text(json.dumps([[c.value, c.name] for c in DENMCauseCode]))
    (path / "lidar_schema.json").write_text(json.dumps(lidar_schema()))


def write_lidar_frames(path: Path) -> None:
    """Frames with a known pitched + rolled pose; the expected render positions are computed with the Python
    transform (rotation only, origin at the sensor) and the CARLA -> Three.js y mirror."""
    rng = np.random.default_rng(5)
    n = 200
    pts = {"x": rng.uniform(-40, 40, n).astype(np.float32), "y": rng.uniform(-40, 40, n).astype(np.float32),
           "z": rng.uniform(-3, 8, n).astype(np.float32), "cos_inc": rng.random(n).astype(np.float32),
           "object_idx": rng.choice([0, 7, 42], n).astype(np.uint32), "tag": rng.choice([1, 3, 14], n).astype(np.uint32)}
    loc, pyr = (120.5, -33.25, 2.4), (8.0, 35.0, -4.0)
    (path / "lidar_semantic.bin").write_bytes(_encode(pts, protocol.SEMANTIC_FIELDS, ego=7, loc=loc, pyr=pyr))
    rot = to_world(np.stack([pts["x"], pts["y"], pts["z"]], 1), (0.0, 0.0, 0.0), pyr)
    expected = rot * np.array([1.0, -1.0, 1.0])
    ray = {k: pts[k] for k in ("x", "y", "z")}
    ray["intensity"] = rng.random(n).astype(np.float32)
    (path / "lidar_raycast.bin").write_bytes(_encode(ray, protocol.RAYCAST_FIELDS, ego=7, loc=loc, pyr=pyr))
    (path / "lidar_expected.json").write_text(json.dumps({
        "positions": expected.tolist(), "object_idx": pts["object_idx"].tolist(), "tag": pts["tag"].tolist(),
        "intensity": ray["intensity"].tolist(),
        "range": np.linalg.norm(np.stack([pts["x"], pts["y"], pts["z"]], 1), axis=1).tolist(),
        "ego": 7, "pyr": list(pyr), "loc": list(loc)}))


def test_node_unit_tests_pass(tmp_path):
    write_fixtures(tmp_path)
    files = sorted(str(f) for f in (ROOT / "tests/js").glob("*.test.mjs"))
    assert files
    res = subprocess.run([NODE, "--test", *files], capture_output=True, text=True, timeout=180,
                         env={**os.environ, "JS_FIXTURES": str(tmp_path)})
    assert res.returncode == 0, res.stdout[-4000:] + res.stderr[-2000:]
    assert "# fail 0" in res.stdout
    skipped = [ln for ln in res.stdout.splitlines() if ln.startswith("# skipped")]
    assert skipped == ["# skipped 0"], f"JS tests were skipped: {skipped}"


def test_python_frames_decode_identically_in_the_browser_decoder(tmp_path):
    """Cross-language wire check (replaces the removed legacy-decoder test): every field, ids above 2**24 (L8),
    a uint32 tag, zlib compression, the world-frame flag; malformed frames are rejected by both sides."""
    rng = np.random.default_rng(3)
    n = 150
    sem = {"x": rng.normal(size=n), "y": rng.normal(size=n), "z": rng.normal(size=n), "cos_inc": rng.random(n),
           "object_idx": rng.integers(2**24, 2**31, n), "tag": rng.integers(0, 29, n)}
    kw = dict(ego=77, loc=(10.0, -20.5, 2.4), pyr=(1.0, 90.0, -2.0), frame=2**33 + 5, sim_time=61.7)
    frames = {
        "a_semantic.bin": (_encode(sem, protocol.SEMANTIC_FIELDS, **kw), protocol.SEMANTIC_FIELDS, sem),
        "b_semantic_zlib.bin": (_encode(sem, protocol.SEMANTIC_FIELDS, compress=True, **kw), protocol.SEMANTIC_FIELDS, sem),
        "c_raycast.bin": (_encode({**sem, "intensity": rng.random(n)}, protocol.RAYCAST_FIELDS, **kw), protocol.RAYCAST_FIELDS, None),
    }
    big = {**sem, "tag": rng.integers(0, 2**31, n)}
    mask32 = protocol.FIELD_XYZ | protocol.FIELD_TAG_U32 | protocol.FIELD_OBJECT_IDX
    frames["d_tag_u32.bin"] = (_encode(big, mask32, **kw), mask32, big)
    for name, (data, _, _) in frames.items():
        (tmp_path / name).write_bytes(data)
    good = frames["a_semantic.bin"][0]
    (tmp_path / "e_bad_mask.bin").write_bytes(good[:46] + (99).to_bytes(2, "little") + good[48:])
    (tmp_path / "f_truncated.bin").write_bytes(good[:-3])

    res = subprocess.run([NODE, str(ROOT / "tests/js/decode_fixtures.mjs"), str(tmp_path)],
                         capture_output=True, text=True, timeout=60)
    assert res.returncode == 0, res.stderr
    js = json.loads(res.stdout)
    js_names = {"tag": "tag", "object_idx": "objectIdx", "intensity": "intensity", "cos_inc": "cosInc"}
    for name, (data, mask, _) in frames.items():
        h, py = protocol.decode(data)
        got = js[name]
        assert "error" not in got, (name, got)
        jh = got["header"]
        assert (jh["frame"], jh["simTime"], jh["numPoints"], jh["egoActorId"], jh["sensorId"], jh["fieldMask"],
                jh["stride"], jh["compressed"], jh["sensorFrame"]) == (
            h.frame, h.sim_time, h.num_points, h.ego_actor_id, h.sensor_id, mask, h.stride, h.compressed, h.sensor_frame)
        assert jh["location"] == list(h.location) and jh["rotationPYR"] == list(h.rotation)
        for field in py.dtype.names:
            key = js_names.get(field, field)
            want = py[field].astype(np.float64) if py[field].dtype.kind == "f" else py[field].astype(np.int64)
            assert np.array_equal(np.asarray(got["points"][key], dtype=want.dtype), want), (name, field)
    assert "error" in js["e_bad_mask.bin"] and "error" in js["f_truncated.bin"]
