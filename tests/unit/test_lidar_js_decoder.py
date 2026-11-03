"""Cross-language check: frames encoded by src/lidar/protocol.py must decode identically in the browser decoder.

Runs the JS decoder under Node (skipped when Node is not installed, e.g. in the Docker image).
"""

import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from src.lidar import protocol

DECODER = Path(__file__).parents[2] / "src" / "visualization" / "web" / "static" / "legacy" / "lidar-decoder.js"
NODE = shutil.which("node")
pytestmark = pytest.mark.skipif(NODE is None, reason="node is not installed")

SCRIPT = r"""
const fs = require('fs');
const D = require(process.argv[2]);
(async () => {
  const out = [];
  for (const file of process.argv.slice(3)) {
    const b = fs.readFileSync(file);
    const f = await D.decode(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength));
    out.push({ header: f.header, x: Array.from(f.x), y: Array.from(f.y), z: Array.from(f.z),
               tag: f.tag && Array.from(f.tag), objectIdx: f.objectIdx && Array.from(f.objectIdx),
               intensity: f.intensity && Array.from(f.intensity), cosInc: f.cosInc && Array.from(f.cosInc) });
  }
  console.log(JSON.stringify(out));
})().catch(e => { console.error(String(e)); process.exit(1); });
"""

KW = dict(frame=2**33 + 5, sim_time=61.7, wall_time=1.7e9, ego_actor_id=77, sensor_id=3,
          location=(10.0, -20.5, 2.4), rotation=(1.0, 90.0, -2.0))


def _semantic(n=200):
    rng = np.random.default_rng(3)
    pts = np.zeros(n, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("cos_inc", "f4"), ("object_idx", "u4"), ("tag", "u4")])
    for f in ("x", "y", "z", "cos_inc"):
        pts[f] = rng.normal(size=n)
    pts["object_idx"] = rng.integers(2**24, 2**31, n)   # above float32's exact range
    pts["tag"] = rng.integers(0, 29, n)
    return pts


def _run(tmp_path, frames):
    files = []
    for i, data in enumerate(frames):
        p = tmp_path / f"f{i}.bin"
        p.write_bytes(data)
        files.append(str(p))
    script = tmp_path / "decode.js"
    script.write_text(SCRIPT)
    res = subprocess.run([NODE, str(script), str(DECODER), *files], capture_output=True, text=True, timeout=30)
    assert res.returncode == 0, res.stderr
    return json.loads(res.stdout)


def test_js_decodes_semantic_and_raycast_and_compressed_frames_like_python(tmp_path):
    sem = _semantic()
    ray = np.zeros(7, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("intensity", "f4")])
    ray["x"], ray["intensity"] = np.arange(7), np.linspace(0, 1, 7)
    frames = [
        protocol.encode(sem, field_mask=protocol.SEMANTIC_FIELDS, **KW),
        protocol.encode(sem, field_mask=protocol.SEMANTIC_FIELDS, compress=True, **KW),
        protocol.encode(ray, field_mask=protocol.RAYCAST_FIELDS, **KW),
    ]
    decoded = _run(tmp_path, frames)

    for js, raw in zip(decoded[:2], frames[:2]):
        h, py = protocol.decode(raw)
        assert js["header"]["frame"] == 2**33 + 5 and js["header"]["sensorId"] == 3 and js["header"]["egoActorId"] == 77
        assert js["header"]["simTime"] == 61.7 and js["header"]["numPoints"] == len(sem)
        assert js["header"]["location"] == [10.0, -20.5, 2.4] and js["header"]["rotation"] == [1.0, 90.0, -2.0]
        assert js["header"]["sensorFrame"] is True
        for name in ("x", "y", "z"):
            assert np.array_equal(np.array(js[name], dtype="f4"), py[name])
        assert js["tag"] == py["tag"].tolist() and js["objectIdx"] == py["object_idx"].tolist()   # exact above 2**24
        assert np.array_equal(np.array(js["cosInc"], dtype="f4"), py["cos_inc"])
    assert decoded[1]["header"]["compressed"] is True and decoded[0]["header"]["compressed"] is False

    js = decoded[2]
    assert js["tag"] is None and js["objectIdx"] is None
    assert np.allclose(js["intensity"], ray["intensity"])


@pytest.mark.parametrize("mutate,message", [
    (lambda b: b"XXXX" + b[4:], "bad magic"),
    (lambda b: b[:4] + b"\x02\x00" + b[6:], "version"),
    (lambda b: b[:20], "too short"),
    (lambda b: b[:-2], "mismatch"),
])
def test_js_rejects_malformed_frames(tmp_path, mutate, message):
    good = protocol.encode(_semantic(5), field_mask=protocol.SEMANTIC_FIELDS, **KW)
    script = tmp_path / "decode.js"
    script.write_text(SCRIPT)
    bad = tmp_path / "bad.bin"
    bad.write_bytes(mutate(good))
    res = subprocess.run([NODE, str(script), str(DECODER), str(bad)], capture_output=True, text=True, timeout=30)
    assert res.returncode != 0 and message in res.stderr
