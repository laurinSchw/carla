import test from 'node:test';
import assert from 'node:assert/strict';
import { LidarStream } from '../../src/web/static/js/lidar-stream.js';

class FakeWS {
  static all = [];
  constructor(url) { this.url = url; this.readyState = 0; this.sent = []; FakeWS.all.push(this); }
  send(s) { this.sent.push(JSON.parse(s)); }
  close() { this.readyState = 3; }
  open() { this.readyState = 1; this.onopen(); }
  text(o) { this.onmessage({ data: JSON.stringify(o) }); }
  bin(buf) { this.onmessage({ data: buf }); }
  drop() { this.readyState = 3; this.onclose(); }
}

function make(extra = {}) {
  FakeWS.all = [];
  const timers = [], events = { frames: [], msgs: [], status: [], closed: [] };
  const s = new LidarStream('ws://x/ws/lidar/3', {
    WebSocketImpl: FakeWS,
    setTimeoutImpl: (fn, ms) => { const t = { fn, ms }; timers.push(t); return t; }, clearTimeoutImpl: () => {},
    onFrame: (b) => events.frames.push(b), onMessage: (m) => events.msgs.push(m), onStatus: (x) => events.status.push(x), onClose: (c) => events.closed.push(c),
    ...extra,
  });
  return { s, timers, events, sock: () => FakeWS.all.at(-1) };
}

test('binary messages are frames, text messages are state/profile/error', () => {
  const { s, events, sock } = make();
  s.connect(); sock().open();
  sock().bin(new ArrayBuffer(8)); sock().text({ type: 'state', state: 'active', frame: 5 }); sock().text({ type: 'stream_profile', max_points: 10000 });
  assert.equal(events.frames.length, 1); assert.deepEqual(events.msgs.map((m) => m.type), ['state', 'stream_profile']);
});

test('the profile is sent when set while open and again after a reconnect', () => {
  const { s, timers, sock } = make();
  s.setProfile({ max_points: 25000, max_hz: 10, compress: false });
  s.connect(); sock().open();
  assert.deepEqual(sock().sent, [{ op: 'profile', max_points: 25000, max_hz: 10, compress: false }]);
  s.setProfile({ max_points: null, max_hz: 5, compress: true });
  assert.equal(sock().sent.length, 2);
  sock().drop();
  timers.at(-1).fn();
  sock().open();
  assert.deepEqual(sock().sent, [{ op: 'profile', max_points: null, max_hz: 5, compress: true }]);
});

test('an unexpected close reconnects; a destroyed or failed sensor does not', () => {
  const a = make();
  a.s.connect(); a.sock().open();
  a.sock().drop();
  assert.equal(a.timers.length, 1); assert.equal(a.events.status.at(-1), 'reconnecting');
  const b = make();
  b.s.connect(); b.sock().open();
  b.sock().text({ type: 'state', state: 'destroyed', frame: 9 });
  b.sock().drop();
  assert.equal(b.timers.length, 0); assert.equal(b.events.status.at(-1), 'closed'); assert.deepEqual(b.events.closed, [{ reason: 'destroyed' }]);
  const c = make();
  c.s.connect(); c.sock().open();
  c.sock().text({ type: 'error', message: 'no LiDAR sensor 3' });
  c.sock().drop();
  assert.equal(c.timers.length, 0);
});

test('close() does not reconnect and ignores late events of the old socket', () => {
  const { s, timers, events, sock } = make();
  s.connect(); const old = sock(); old.open();
  s.close();
  old.onclose && old.onclose();
  old.bin(new ArrayBuffer(4));
  assert.equal(timers.length, 0); assert.equal(events.frames.length, 0);
});
