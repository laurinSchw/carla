import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import {
  basisVectors, transformMatrix, toWorld, toSensor, carlaToThree, createViewport, worldToScreen, screenToWorld,
  yawToCanvasAngle, zoomAt, fitBounds,
} from '../../src/web/static/js/coords.js';

const near = (a, b, eps = 1e-9) => assert.ok(Math.abs(a - b) <= eps, `${a} != ${b}`);
const fixtures = process.env.JS_FIXTURES ? JSON.parse(readFileSync(join(process.env.JS_FIXTURES, 'coords.json'), 'utf8')) : null;

test('matrix equals the Python transform_matrix for poses incl. pitched + rolled', { skip: !fixtures }, () => {
  for (const c of fixtures.poses) {
    const m = transformMatrix(c.location, c.pyr);
    for (let i = 0; i < 16; i++) near(m[i], c.matrix[i], 1e-12);
  }
  assert.ok(fixtures.poses.some((c) => Math.abs(c.pyr[0]) > 5 && Math.abs(c.pyr[2]) > 3), 'fixture must contain pitched+rolled poses');
});

test('toWorld / toSensor agree with Python and invert each other', { skip: !fixtures }, () => {
  for (const c of fixtures.poses) {
    const pts = Float32Array.from(c.points.flat());
    const w = toWorld(pts, c.location, c.pyr);
    c.world.flat().forEach((v, i) => near(w[i], v, 1e-4));
    const back = toSensor(w, c.location, c.pyr);
    pts.forEach((v, i) => near(back[i], v, 1e-6));
  }
});

test('basis vectors are orthonormal', () => {
  for (const [yaw, pitch, roll] of [[30, 10, 5], [-120, 45, -30], [0, 0, 0], [90, -60, 80]]) {
    const { forward: f, right: r, up: u } = basisVectors(yaw, pitch, roll);
    const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    near(dot(f, f), 1); near(dot(r, r), 1); near(dot(u, u), 1);
    near(dot(f, r), 0); near(dot(f, u), 0); near(dot(r, u), 0);
  }
});

test('yaw 90 turns forward (x) into +y; pitch up raises forward', () => {
  const a = basisVectors(90).forward;
  near(a[0], 0); near(a[1], 1);
  const b = basisVectors(0, 30).forward;
  near(b[2], 0.5);
});

test('CARLA -> Three mirrors y only', () => {
  assert.deepEqual(carlaToThree([1, 2, 3]), [1, -2, 3]);
});

test('map: +y moves DOWN the screen, +x moves right (not mirrored)', () => {
  const vp = createViewport({ width: 800, height: 600, cx: 10, cy: 20, scale: 2 });
  const [x0, y0] = worldToScreen(vp, 10, 20);
  assert.deepEqual([x0, y0], [400, 300]);
  const [, yPlus] = worldToScreen(vp, 10, 30);
  assert.ok(yPlus > y0, 'a station moving +y must move down the screen');
  const [xPlus] = worldToScreen(vp, 20, 20);
  assert.ok(xPlus > x0);
});

test('map: a heading of yaw 90 points DOWN, yaw 0 points right (ctx.rotate semantics)', () => {
  // ctx.rotate(a) maps the canvas x axis (1,0) to (cos a, sin a); canvas y points down.
  for (const [yaw, expectDown] of [[90, true], [-90, false], [0, null]]) {
    const a = yawToCanvasAngle(yaw);
    const dir = [Math.cos(a), Math.sin(a)];
    // the same heading from the world forward vector through worldToScreen
    const vp = createViewport({ width: 100, height: 100, scale: 1 });
    const f = basisVectors(yaw).forward;
    const s0 = worldToScreen(vp, 0, 0), s1 = worldToScreen(vp, f[0], f[1]);
    near(s1[0] - s0[0], dir[0], 1e-12); near(s1[1] - s0[1], dir[1], 1e-12);
    if (expectDown === true) assert.ok(dir[1] > 0.99);
    if (expectDown === false) assert.ok(dir[1] < -0.99);
  }
});

test('screenToWorld inverts worldToScreen', () => {
  const vp = createViewport({ width: 321, height: 123, cx: -7.5, cy: 99, scale: 0.37 });
  for (const [x, y] of [[0, 0], [-100, 50], [3.3, -8]]) {
    const [sx, sy] = worldToScreen(vp, x, y);
    const [wx, wy] = screenToWorld(vp, sx, sy);
    near(wx, x, 1e-9); near(wy, y, 1e-9);
  }
});

test('zoomAt keeps the world point under the cursor fixed and clamps', () => {
  const vp = createViewport({ width: 800, height: 600, cx: 0, cy: 0, scale: 1 });
  const before = screenToWorld(vp, 123, 456);
  const z = zoomAt(vp, 123, 456, 2);
  assert.equal(z.scale, 2);
  const after = screenToWorld(z, 123, 456);
  near(after[0], before[0], 1e-9); near(after[1], before[1], 1e-9);
  assert.equal(zoomAt(vp, 0, 0, 1e9).scale, 40);
  assert.equal(zoomAt(vp, 0, 0, 1e-9).scale, 0.05);
});

test('fitBounds centres and scales to fit', () => {
  const vp = fitBounds(createViewport({ width: 800, height: 400 }), -50, -10, 250, 190, 1);
  near(vp.cx, 100); near(vp.cy, 90); near(vp.scale, Math.min(800 / 300, 400 / 200));
});
