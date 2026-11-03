import test from 'node:test';
import assert from 'node:assert/strict';
import { Store } from '../../src/web/static/js/store.js';

const env = (topic, seq, frame, simTime, data = {}) => ({ topic, seq, frame, sim_time: simTime, data });

test('keeps the latest envelope and counts updates and skipped publications', () => {
  let t = 1000;
  const s = new Store({ now: () => t });
  assert.equal(s.get('world.state'), null);
  assert.equal(s.put(env('world.state', 1, 10, 0.5)), 0);
  t = 1100;
  assert.equal(s.put(env('world.state', 4, 13, 0.65)), 2, 'seq 2 and 3 were skipped');
  assert.equal(s.get('world.state').frame, 13);
  const i = s.info('world.state');
  assert.equal(i.count, 2); assert.equal(i.skipped, 2);
  t = 1350;
  assert.equal(s.ageMs('world.state'), 250);
  assert.equal(s.ageMs('nothing'), null);
});

test('listeners get the envelope and the skipped count; unsubscribe works', () => {
  const s = new Store();
  const seen = [];
  const off = s.on('a', (e, skipped) => seen.push([e.seq, skipped]));
  s.put(env('a', 1, 1, 0)); s.put(env('a', 5, 5, 0.2));
  off();
  s.put(env('a', 6, 6, 0.3));
  assert.deepEqual(seen, [[1, 0], [5, 3]]);
});

test('connection state is observable', () => {
  const s = new Store();
  const states = [];
  s.onConnection((c) => states.push(c.state));
  s.setConnection('open'); s.setConnection('reconnecting', 'retry in 1 s');
  assert.deepEqual(states, ['closed', 'open', 'reconnecting']);
});

test('v2x ring buffer holds about two simulated seconds and never returns a newer frame', () => {
  const s = new Store({ ringSimSeconds: 2.0 });
  for (let k = 0; k < 10; k++) s.put(env('v2x.frame', k + 1, 100 + 10 * k, 0.5 * k, { k }));
  const frames = s.v2xRing.map((f) => f.frame);
  assert.ok(frames.length <= 5 && frames.length >= 4, `ring has ${frames.length}`);
  assert.equal(s.v2xAtOrBefore(10_000).frame, 190);
  assert.equal(s.v2xAtOrBefore(185).frame, 180, 'newest frame <= the LiDAR frame, never a newer one');
  assert.equal(s.v2xAtOrBefore(150).frame, 150);
  assert.equal(s.v2xAtOrBefore(5), null, 'older than everything in the buffer');
});

test('ring: same frame twice is ignored, a restarted simulation clears it', () => {
  const s = new Store();
  s.put(env('v2x.frame', 1, 100, 1.0)); s.put(env('v2x.frame', 2, 100, 1.0));
  assert.equal(s.v2xRing.length, 1);
  s.put(env('v2x.frame', 3, 110, 1.5));
  s.put(env('v2x.frame', 4, 50, 0.1));              // frame went backwards
  assert.deepEqual(s.v2xRing.map((f) => f.frame), [50]);
});

test('the inbox of the selected station has its own ring, so overlays can use the frame that matches a LiDAR frame', () => {
  const s = new Store({ ringSimSeconds: 2.0 });
  for (let k = 0; k < 6; k++) s.put(env('v2x.inbox', k + 1, 100 + 10 * k, 0.5 * k, { station_id: 7, k }));
  assert.equal(s.atOrBefore('v2x.inbox', 125).data.k, 2);
  assert.equal(s.atOrBefore('v2x.inbox', 125).frame, 120);
  assert.equal(s.atOrBefore('v2x.frame', 125), null, 'rings are per topic');
});
