"""Browser-side LiDAR code under Node: the viewer's drawing maths and a syntax check of the page script.

The viewer only needs a 2-D context, so a recording stub stands in for the canvas (skipped without Node).
"""

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

WEB = Path(__file__).parents[2] / "src" / "visualization" / "web"
STATIC = WEB / "static" / "legacy"
NODE = shutil.which("node")
pytestmark = pytest.mark.skipif(NODE is None, reason="node is not installed")

SCRIPT = r"""
const fs = require('fs');
global.self = global;
require(process.argv[2]);                      // sets self.LidarView
const calls = [];
const ctx = new Proxy({}, { get: (t, k) => (k in t ? t[k] : (...a) => { if (k === 'fillRect') calls.push([ctx.fillStyle, ...a]); }),
                            set: (t, k, v) => { t[k] = v; return true; } });
const canvas = { width: 800, height: 800, getContext: () => ctx, addEventListener() {}, setPointerCapture() {} };

function frame(pts, ego = 7) {
  const n = pts.length;
  return { header: { egoActorId: ego, numPoints: n },
           x: Float32Array.from(pts.map(p => p.x)), y: Float32Array.from(pts.map(p => p.y)), z: Float32Array.from(pts.map(p => p.z || 0)),
           tag: Uint8Array.from(pts.map(p => p.tag || 0)), objectIdx: Uint32Array.from(pts.map(p => p.obj || 0)),
           intensity: null, cosInc: null };
}
const out = {};
const v = new LidarView(canvas, { rangeM: 40, pointSize: 2 });
v.setTagTable([{ id: 14, name: 'Car', color: [1, 2, 3] }]);

// top-down: scale = 400 px / 40 m = 10 px/m; x forward is UP, y right is RIGHT
v.setFrame(frame([{ x: 10, y: 5, tag: 14 }]));
out.top = calls.filter(c => c[0] === 'rgb(1,2,3)');
out.drawn1 = v.lastDrawn;

// own-vehicle hits are hidden by default, shown on request; points outside the view are skipped
calls.length = 0;
const pts = [{ x: 1, y: 1, obj: 7 }, { x: 2, y: 2, obj: 9 }, { x: 500, y: 0, obj: 9 }];
v.setFrame(frame(pts)); out.hidden = v.lastDrawn;
v.hideOwn = false; v.setFrame(frame(pts)); out.shown = v.lastDrawn;

// empty and missing frames must not throw
v.frame = null; v.render(); v.setFrame(frame([])); out.empty = v.lastDrawn;

// 3-D mode draws too
v.mode = '3d'; v.hideOwn = true; v.setFrame(frame([{ x: 5, y: 0, z: 1 }, { x: 8, y: 3, z: 0.5 }])); out.d3 = v.lastDrawn;
console.log(JSON.stringify(out));
"""


def _run_view(tmp_path):
    script = tmp_path / "view.js"
    script.write_text(SCRIPT)
    res = subprocess.run([NODE, str(script), str(STATIC / "lidar-view.js")], capture_output=True, text=True, timeout=30)
    assert res.returncode == 0, res.stderr
    return json.loads(res.stdout)


def test_top_down_view_puts_forward_up_and_right_right(tmp_path):
    out = _run_view(tmp_path)
    (style, x, y, w, h), = out["top"]
    assert (x, y) == (400 + 5 * 10, 400 - 10 * 10)   # centre + 5 m right, centre - 10 m forward


def test_view_hides_own_hits_and_skips_points_outside_the_range(tmp_path):
    out = _run_view(tmp_path)
    assert out["drawn1"] == 1
    assert out["hidden"] == 1          # ego hit hidden, 500 m point outside the 40 m view
    assert out["shown"] == 2
    assert out["empty"] == 0 and out["d3"] == 2


def test_page_script_parses_and_references_existing_elements():
    html = (WEB / "client_join.html").read_text()
    scripts = re.findall(r"<script>(.*?)</script>", html, re.S)
    assert scripts, "no inline script"
    for i, code in enumerate(scripts):
        res = subprocess.run([NODE, "--check", "-"], input=code, capture_output=True, text=True, timeout=30)
        assert res.returncode == 0, res.stderr
    ids = set(re.findall(r'id="([^"]+)"', html))
    used = set(re.findall(r"getElementById\('([^']+)'\)", html))
    assert used <= ids, f"script uses ids that do not exist: {sorted(used - ids)}"
    for src in re.findall(r'<script src="([^"]+)"', html):
        assert (WEB / src.lstrip("/")).exists(), src
