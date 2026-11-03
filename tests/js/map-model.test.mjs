import test from 'node:test';
import assert from 'node:assert/strict';
import {
  LINK_STYLES, linkOutcome, linkOpacity, rowsToObjects, stationGlyph, ageClass, hitTestStations, distToSegment,
  hitTestLinks, Trails, Perspective, geometryBounds,
} from '../../src/web/static/js/map-model.js';
import { createViewport } from '../../src/web/static/js/coords.js';

test('every link outcome has a distinct dash pattern (not colour only) and a distinct colour token', () => {
  const dashes = Object.values(LINK_STYLES).map((s) => s.dash.join(','));
  assert.equal(new Set(dashes).size, dashes.length);
  assert.equal(new Set(Object.values(LINK_STYLES).map((s) => s.token)).size, 4);
  assert.deepEqual(LINK_STYLES.delivered.dash, []);
});

test('linkOutcome and opacity', () => {
  assert.equal(linkOutcome({ delivered: true }), 'delivered');
  assert.equal(linkOutcome({ delivered: false, drop_cause: 'mac' }), 'mac');
  assert.equal(linkOutcome({ delivered: false, drop_cause: 'out_of_awareness_range' }), 'out_of_awareness_range');
  assert.equal(linkOutcome({ delivered: false, drop_cause: null }), 'channel');
  assert.equal(linkOpacity(1), 1); assert.equal(linkOpacity(0.5), 0.5); assert.equal(linkOpacity(0), 0.15);
  assert.equal(linkOpacity(NaN), 0.35);
});

test('rowsToObjects', () => {
  assert.deepEqual(rowsToObjects(['a', 'b'], [[1, 2], [3, 4]]), [{ a: 1, b: 2 }, { a: 3, b: 4 }]);
});

test('glyphs differ by kind and equipment', () => {
  assert.deepEqual(stationGlyph({ kind: 'RSU' }), { shape: 'square', filled: true });
  assert.deepEqual(stationGlyph({ kind: 'pedestrian' }), { shape: 'circle', filled: true });
  assert.deepEqual(stationGlyph({ kind: 'vehicle', v2x: true }), { shape: 'arrow', filled: true });
  assert.deepEqual(stationGlyph({ kind: 'vehicle', v2x: false }), { shape: 'arrow', filled: false });
});

test('age classes follow the update period', () => {
  assert.equal(ageClass(0.4, 0.5), 'fresh'); assert.equal(ageClass(0.9, 0.5), 'aging');
  assert.equal(ageClass(1.01, 0.5), 'stale'); assert.equal(ageClass(NaN, 0.5), 'stale');
});

test('hit testing picks the nearest station within the radius, in screen space', () => {
  const vp = createViewport({ width: 200, height: 200, cx: 0, cy: 0, scale: 2 });
  const st = [{ id: 1, x: 10, y: 0 }, { id: 2, x: 11, y: 0 }, { id: 3, x: 50, y: 50 }];
  assert.equal(hitTestStations(st, vp, 100 + 21, 100, 10).id, 2);
  assert.equal(hitTestStations(st, vp, 100 + 19, 100, 10).id, 1);
  assert.equal(hitTestStations(st, vp, 5, 5, 10), null);
});

test('segment distance and link hit testing', () => {
  assert.equal(distToSegment(5, 5, 0, 0, 10, 0), 5);
  assert.equal(distToSegment(-3, 4, 0, 0, 10, 0), 5);
  assert.equal(distToSegment(1, 1, 0, 0, 0, 0), Math.SQRT2);
  const drawn = [{ id: 'a', ax: 0, ay: 0, bx: 100, by: 0 }, { id: 'b', ax: 0, ay: 10, bx: 100, by: 10 }];
  assert.equal(hitTestLinks(drawn, 50, 3, 5).id, 'a');
  assert.equal(hitTestLinks(drawn, 50, 8, 5).id, 'b');
  assert.equal(hitTestLinks(drawn, 50, 30, 5), null);
});

test('trails keep the last N simulated seconds and drop vanished stations', () => {
  const tr = new Trails(2);
  for (let k = 0; k <= 10; k++) tr.push(k * 0.5, [{ id: 1, x: k, y: 0 }, ...(k < 5 ? [{ id: 2, x: 0, y: k }] : [])]);
  assert.equal(tr.get(1)[0].t, 3);
  assert.equal(tr.get(1).at(-1).x, 10);
  assert.equal(tr.get(2).length, 0, 'station 2 left the world');
  tr.push(0.1, [{ id: 1, x: 0, y: 0 }]);                         // the simulation restarted
  assert.equal(tr.get(1).length, 1);
  tr.push(0.1, [{ id: 1, x: 5, y: 5 }]);                         // same sim time twice: no duplicate
  assert.equal(tr.get(1).length, 1);
});

function bsm(id, x, y, ts) { return { tx_id: id, msg_type: 'BSM', message: { x_m: x, y_m: y, heading: 90, speed: 5, timestamp: ts, vehicle_length: 4.5, vehicle_width: 1.8 }, link: { rx_power_dbm: -70 } }; }

test('perspective: only received messages, at the positions in the messages, with age', () => {
  const p = new Perspective({ ghostSeconds: 5 });
  p.reset(7);
  p.update({ station_id: 7, messages: [bsm(1, 10, 20, 0.5), bsm(2, 30, 40, 0.5)] }, 10, 0.5);
  let e = p.entities(0.5).sort((a, b) => a.id - b.id);
  assert.deepEqual(e.map((x) => [x.id, x.x, x.y, x.ghost, x.age]), [[1, 10, 20, false, 0], [2, 30, 40, false, 0]]);
  // next update: station 2 was not heard -> a ghost at its LAST RECEIVED position, aged
  p.update({ station_id: 7, messages: [bsm(1, 15, 20, 1.0)] }, 20, 1.0);
  e = p.entities(1.0).sort((a, b) => a.id - b.id);
  assert.deepEqual(e.map((x) => [x.id, x.x, x.ghost, x.age]), [[1, 15, false, 0], [2, 30, true, 0.5]]);
  // ghosts disappear after ghostSeconds
  p.update({ station_id: 7, messages: [bsm(1, 20, 20, 7.0)] }, 140, 7.0);
  assert.deepEqual(p.entities(7.0).map((x) => x.id), [1]);
});

test('perspective: ignores other stations\' inboxes and resets on a new station or a restarted simulation', () => {
  const p = new Perspective();
  p.reset(7);
  p.update({ station_id: 8, messages: [bsm(1, 0, 0, 0)] }, 1, 0);
  assert.equal(p.entities(0).length, 0);
  p.update({ station_id: 7, messages: [bsm(1, 0, 0, 5)] }, 100, 5);
  p.update({ station_id: 7, messages: [] }, 50, 2);              // frame went backwards
  assert.equal(p.entities(2).length, 0);
  p.reset(9); assert.equal(p.stationId, 9);
});

test('perspective builds RSUs from MAP, SPAT by intersection, VRUs and DENMs from their messages', () => {
  const p = new Perspective();
  p.reset(7);
  p.update({ station_id: 7, messages: [
    { tx_id: -3, msg_type: 'MAP', message: { intersection_id: 3, ref_point: [5, 6, 0], timestamp: 1 }, link: {} },
    { tx_id: -3, msg_type: 'SPAT', message: { intersection_id: 3, states: [{ signal_group_id: 0, movement_event: 'stop-And-Remain', likely_time_s: null }] }, link: {} },
    { tx_id: 300, msg_type: 'VRUAM', message: { position: [8, 9, 0], heading: 10, speed: 1, timestamp: 1 }, link: {} },
    { tx_id: -1000001, msg_type: 'DENM', message: { action_id: 1, event_position: [50, 60, 0], relevance_distance: 120, cause_code: 'ACCIDENT', reference_time: 1 }, link: {} },
  ] }, 20, 1.0);
  const kinds = p.entities(1.0).map((e) => e.kind).sort();
  assert.deepEqual(kinds, ['denm', 'rsu', 'vru']);
  assert.equal(p.spat.get(3).message.states[0].likely_time_s, null);
  assert.equal(p.entities(1.0).find((e) => e.kind === 'denm').radius, 120);
});

test('geometry bounds', () => {
  assert.deepEqual(geometryBounds({ lanes: [[[0, 5], [10, -3]], [[-4, 2]]] }), { minX: -4, minY: -3, maxX: 10, maxY: 5 });
  assert.equal(geometryBounds({ lanes: [] }), null);
});
