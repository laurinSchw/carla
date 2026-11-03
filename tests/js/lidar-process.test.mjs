import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { decodeFrame } from '../../src/web/static/js/lidar-decoder.js';
import { processFrame, availableModes, ramp, paletteLut, RateMeter, frameGapSeconds } from '../../src/web/static/js/lidar-process.js';

const dir = process.env.JS_FIXTURES;
const load = (name) => {
  const b = readFileSync(join(dir, name));
  return decodeFrame(b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength));
};
const exp = dir ? JSON.parse(readFileSync(join(dir, 'lidar_expected.json'), 'utf8')) : null;

test('positions = header rotation (pitched + rolled pose), origin at the sensor, y mirrored for Three.js', { skip: !dir }, () => {
  const f = load('lidar_semantic.bin');
  const r = processFrame(f, { colorMode: 'semantic', egoFilter: false });
  assert.equal(r.counts.displayed, 200);
  for (let i = 0; i < 200; i++) for (let k = 0; k < 3; k++) {
    assert.ok(Math.abs(r.positions[3 * i + k] - exp.positions[i][k]) < 2e-4, `point ${i} axis ${k}: ${r.positions[3 * i + k]} vs ${exp.positions[i][k]}`);
  }
});

test('ego self-hits are filtered and counted; the filter can be switched off', { skip: !dir }, () => {
  const f = load('lidar_semantic.bin');
  const nEgo = exp.object_idx.filter((v) => v === exp.ego).length;
  assert.ok(nEgo > 0);
  const on = processFrame(f, { colorMode: 'range' });
  assert.equal(on.counts.filteredEgo, nEgo); assert.equal(on.counts.displayed, 200 - nEgo); assert.equal(on.positions.length, 3 * (200 - nEgo));
  const off = processFrame(f, { colorMode: 'range', egoFilter: false });
  assert.equal(off.counts.filteredEgo, 0); assert.equal(off.counts.displayed, 200);
  // the surviving points are exactly the non-ego ones, in order
  let j = 0;
  for (let i = 0; i < 200; i++) if (exp.object_idx[i] !== exp.ego) {
    assert.ok(Math.abs(on.positions[3 * j] - exp.positions[i][0]) < 2e-4); j++;
  }
});

test('buffer capacity: extra points are dropped and counted, never silently', { skip: !dir }, () => {
  const r = processFrame(load('lidar_semantic.bin'), { colorMode: 'range', egoFilter: false, maxPoints: 50 });
  assert.equal(r.counts.displayed, 50); assert.equal(r.counts.truncated, 150); assert.equal(r.positions.length, 150);
});

test('semantic colours come from the palette by tag; unknown tags are grey', { skip: !dir }, () => {
  const lut = paletteLut([{ id: 1, color: [255, 0, 0] }, { id: 3, color: [0, 255, 0] }, { id: 14, color: [0, 0, 255] }]);
  const r = processFrame(load('lidar_semantic.bin'), { colorMode: 'semantic', lut, egoFilter: false });
  const want = { 1: [1, 0, 0], 3: [0, 1, 0], 14: [0, 0, 1] };
  for (let i = 0; i < 200; i++) {
    const w = want[exp.tag[i]];
    assert.deepEqual([r.colors[3 * i], r.colors[3 * i + 1], r.colors[3 * i + 2]].map((v) => Math.round(v)), w);
  }
  assert.deepEqual(Array.from(paletteLut([]).slice(0, 3)), [0.5, 0.5, 0.5]);
});

test('highlight colours exactly the points of the selected actor', { skip: !dir }, () => {
  const r = processFrame(load('lidar_semantic.bin'), { colorMode: 'highlight', highlightId: 42, egoFilter: false });
  for (let i = 0; i < 200; i++) {
    const hot = r.colors[3 * i] > 0.9;
    assert.equal(hot, exp.object_idx[i] === 42);
  }
});

test('modes offered depend on what the frame contains; an unavailable mode is an error, not a fallback', { skip: !dir }, () => {
  const sem = load('lidar_semantic.bin'), ray = load('lidar_raycast.bin');
  assert.deepEqual(availableModes(sem.points).sort(), ['height', 'highlight', 'range', 'semantic']);
  assert.deepEqual(availableModes(ray.points).sort(), ['height', 'intensity', 'range']);
  assert.throws(() => processFrame(ray, { colorMode: 'semantic' }), /not available/);
  assert.throws(() => processFrame(sem, { colorMode: 'intensity' }), /not available/);
  const r = processFrame(ray, { colorMode: 'intensity' });
  for (let i = 0; i < 5; i++) assert.ok(Math.abs(r.colors[3 * i] - exp.intensity[i]) < 1e-6);
  assert.equal(r.counts.filteredEgo, 0, 'a raycast frame has no object ids to filter');
});

test('range colours follow the distance to the sensor', { skip: !dir }, () => {
  const r = processFrame(load('lidar_semantic.bin'), { colorMode: 'range', egoFilter: false, rangeMax: 60 });
  const order = exp.range.map((d, i) => [d, i]).sort((a, b) => a[0] - b[0]);
  const near = order[0][1], far = order[order.length - 1][1];
  // blue-ish near, red-ish far on this ramp
  assert.ok(r.colors[3 * near + 2] > r.colors[3 * near], 'near points are bluish');
  assert.ok(r.colors[3 * far] >= r.colors[3 * far + 2], 'far points are reddish');
});

test('ramp stays in [0,1] and is continuous at the ends', () => {
  for (const t of [-1, 0, 0.25, 0.5, 1, 2]) for (const c of ramp(t)) assert.ok(c >= 0 && c <= 1);
  assert.deepEqual(ramp(-5), ramp(0)); assert.deepEqual(ramp(7), ramp(1));
});

test('rate meter: fps and bytes per second over a window', () => {
  let t = 0;
  const m = new RateMeter(2000, () => t);
  for (let k = 0; k < 11; k++) { t = k * 100; m.add(1000); }
  const r = m.rates();
  assert.ok(Math.abs(r.fps - 10) < 1e-9); assert.ok(Math.abs(r.bytesPerSecond - 10000) < 1e-6);
  t = 10_000;
  assert.deepEqual(m.rates(), { fps: 0, bytesPerSecond: 0 });
});

test('LiDAR frame vs V2X frame gap in simulated seconds', () => {
  assert.equal(frameGapSeconds(1010, 1000, 0.05), 0.5);
  assert.equal(frameGapSeconds(1000, null, 0.05), null);
  assert.equal(frameGapSeconds(1000, 1000, 0.05), 0);
});
