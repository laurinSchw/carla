import test from 'node:test';
import assert from 'node:assert/strict';
import * as u from '../../src/web/static/js/units.js';

test('numbers are rounded for display only, missing values are an em dash', () => {
  assert.equal(u.metres(12.3456), '12.3 m');
  assert.equal(u.metres(null), u.DASH);
  assert.equal(u.metres(undefined), u.DASH);
  assert.equal(u.metres(NaN), u.DASH);
  assert.equal(u.dBm(-63.94), '-63.9 dBm');
  assert.equal(u.degrees(90), '90.0°');
  assert.equal(u.hertz(2), '2.0 Hz');
});

test('speed shows m/s with km/h as secondary', () => {
  assert.equal(u.speed(10), '10.0 m/s (36 km/h)');
  assert.equal(u.speed(null), u.DASH);
});

test('age, bytes, percent, stamp', () => {
  assert.equal(u.age(0.5), '0.50 s');
  assert.equal(u.age(12.34), '12.3 s');
  assert.equal(u.age(125), '2 min 5 s');
  assert.equal(u.bytes(512), '512 B');
  assert.equal(u.bytes(2048), '2.0 kB');
  assert.equal(u.bytes(5 * 1024 * 1024), '5.00 MB');
  assert.equal(u.percent(0.8), '80.0 %');
  assert.equal(u.stamp(1200, 3.5), 'frame 1200 · t=3.50 s');
  assert.equal(u.stamp(null, null), u.DASH);
});

test('stale means older than two update periods', () => {
  assert.equal(u.isStale(1.0, 0.5), false);
  assert.equal(u.isStale(1.01, 0.5), true);
  assert.equal(u.isStale(null, 0.5), false);
});

test('orDash gives the reason as a tooltip, never a fallback number', () => {
  assert.deepEqual(u.orDash(null, u.seconds, 'SPAT timing unknown'), { text: u.DASH, title: 'SPAT timing unknown' });
  assert.equal(u.orDash(1.5, u.seconds).text, '1.50 s');
  assert.equal(u.orDash(0, u.seconds).text, '0.00 s');
});

test('escapeHtml and csvCell', () => {
  assert.equal(u.escapeHtml('<a href="x">&\'</a>'), '&lt;a href=&quot;x&quot;&gt;&amp;&#39;&lt;/a&gt;');
  assert.equal(u.csvCell('a,b'), '"a,b"');
  assert.equal(u.csvCell('say "hi"'), '"say ""hi"""');
  assert.equal(u.csvCell(null), '');
  assert.equal(u.csvCell(1.5), '1.5');
});
