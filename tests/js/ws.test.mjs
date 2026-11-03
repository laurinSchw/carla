import test from 'node:test';
import assert from 'node:assert/strict';
import { HubSocket } from '../../src/web/static/js/ws.js';

class FakeWS {
  static instances = [];
  constructor(url) { this.url = url; this.readyState = 0; this.sent = []; this.closedWith = null; FakeWS.instances.push(this); }
  send(s) { this.sent.push(JSON.parse(s)); }
  close(code) { this.closedWith = code; this.readyState = 3; }
  open() { this.readyState = 1; this.onopen(); }
  message(obj) { this.onmessage({ data: JSON.stringify(obj) }); }
  drop(code = 1006) { this.readyState = 3; this.onclose({ code }); }
}

function harness(extra = {}) {
  FakeWS.instances = [];
  const timers = [];
  const hub = new HubSocket('ws://x/ws', {
    WebSocketImpl: FakeWS,
    setTimeoutImpl: (fn, ms) => { const t = { fn, ms, cancelled: false }; timers.push(t); return t; },
    clearTimeoutImpl: (t) => { if (t) t.cancelled = true; },
    random: () => 0.5,
    ...extra,
  });
  const states = [];
  hub.onState = (s, d) => states.push(s + (d ? ` (${d})` : ''));
  return { hub, timers, states, sock: () => FakeWS.instances[FakeWS.instances.length - 1] };
}

test('auth is the first message, then the subscriptions', () => {
  const { hub, sock } = harness({ credentials: () => ({ session_token: 'tok' }) });
  hub.subscribe('session.self', { max_hz: 5 });
  hub.subscribe('v2x.inbox', { filter: { station_id: 7 } });
  hub.connect();
  sock().open();
  assert.deepEqual(sock().sent.map((m) => m.op), ['auth', 'subscribe', 'subscribe']);
  assert.deepEqual(sock().sent[0], { op: 'auth', session_token: 'tok' });
  assert.deepEqual(sock().sent[2], { op: 'subscribe', topic: 'v2x.inbox', filter: { station_id: 7 } });
});

test('no credentials: no auth message (observer)', () => {
  const { hub, sock } = harness();
  hub.subscribe('world.state', {});
  hub.connect(); sock().open();
  assert.deepEqual(sock().sent.map((m) => m.op), ['subscribe']);
});

test('subscribing while open sends at once; unsubscribe sends and forgets', () => {
  const { hub, sock } = harness();
  hub.connect(); sock().open();
  hub.subscribe('events', { max_hz: 1 });
  assert.deepEqual(sock().sent.at(-1), { op: 'subscribe', topic: 'events', max_hz: 1 });
  hub.unsubscribe('events');
  assert.deepEqual(sock().sent.at(-1), { op: 'unsubscribe', topic: 'events' });
  hub.close(); hub.connect();
  sock().open();
  assert.equal(sock().sent.some((m) => m.op === 'subscribe'), false);
});

test('envelopes reach the handler with the skipped count from seq gaps', () => {
  const { hub, sock } = harness();
  const got = [];
  hub.subscribe('world.state', {}, (data, env, gap) => got.push([env.seq, gap, data.k]));
  hub.connect(); sock().open();
  sock().message({ topic: 'world.state', seq: 1, frame: 1, sim_time: 0, data: { k: 'a' } });
  sock().message({ topic: 'world.state', seq: 4, frame: 4, sim_time: 0, data: { k: 'b' } });
  sock().message({ topic: 'other', seq: 1, data: {} });             // not subscribed: ignored
  assert.deepEqual(got, [[1, 0, 'a'], [4, 2, 'b']]);
  assert.equal(hub.skipped.get('world.state'), 2);
});

test('errors and control replies go to their callbacks; auth sets the role', () => {
  const { hub, sock } = harness();
  const errs = [], ctl = [];
  hub.onError = (m) => errs.push(m.code); hub.onControl = (m) => ctl.push(m.topic);
  hub.connect(); sock().open();
  sock().message({ topic: 'error', code: 'forbidden', message: 'x' });
  sock().message({ topic: 'auth', data: { role: 'admin' } });
  sock().message({ topic: 'subscribed', data: {} });
  sock().message({ topic: 'pong', data: {} });
  assert.deepEqual(errs, ['forbidden']); assert.deepEqual(ctl, ['auth', 'subscribed', 'pong']); assert.equal(hub.role, 'admin');
});

test('reconnect with exponential backoff and jitter, then re-auth and re-subscribe', () => {
  const { hub, timers, states, sock } = harness({ credentials: () => ({ admin_token: 'a' }), random: () => 1 });
  hub.subscribe('v2x.stats', { max_hz: 1 });
  hub.connect(); sock().open();
  const first = sock();
  first.drop();
  assert.equal(states.at(-1).startsWith('reconnecting'), true);
  const retry1 = timers.at(-1);
  assert.equal(retry1.ms, 500, 'attempt 0: full jitter 1.0 of 500 ms');
  retry1.fn();
  assert.notEqual(sock(), first);
  sock().drop();                                           // fails before opening
  assert.equal(timers.at(-1).ms, 1000);
  timers.at(-1).fn(); sock().drop();
  assert.equal(timers.at(-1).ms, 2000);
  timers.at(-1).fn();
  sock().open();
  assert.deepEqual(sock().sent.map((m) => m.op), ['auth', 'subscribe']);
  assert.equal(hub.attempt, 0, 'a successful open resets the backoff');
  assert.equal(states.at(-1), 'open');
});

test('backoff is capped and jittered into [cap/2, cap]', () => {
  const { hub } = harness({ random: () => 0 });
  hub.attempt = 20;
  assert.equal(hub.nextDelay(), 7500);
  const h2 = harness({ random: () => 1 }).hub; h2.attempt = 20;
  assert.equal(h2.nextDelay(), 15000);
});

test('close() stops reconnecting; authentication failure (4401) does not retry', () => {
  const a = harness();
  a.hub.connect(); a.sock().open();
  const s = a.sock();
  a.hub.close();
  assert.equal(s.closedWith, 1000);
  s.onclose({ code: 1000 });                              // the close event after close() must not reconnect
  assert.equal(a.timers.filter((t) => !t.cancelled && t.ms >= 500 && t.ms <= 15000 && t.ms !== 15000).length, 0);
  assert.equal(a.states.at(-1), 'closed');
  const b = harness({ credentials: () => ({ admin_token: 'bad' }) });
  b.hub.connect(); b.sock().open();
  const n = b.timers.length;
  b.sock().drop(4401);
  assert.equal(b.timers.length, n, 'no retry scheduled');
  assert.match(b.states.at(-1), /closed \(authentication failed\)/);
});

test('late events of an old socket are ignored after a reconnect', () => {
  const { hub, timers, sock } = harness();
  const got = [];
  hub.subscribe('t', {}, (d) => got.push(d));
  hub.connect(); sock().open();
  const old = sock();
  old.drop();
  timers.at(-1).fn();
  sock().open();
  const n = timers.length;
  old.onclose({ code: 1006 });                              // a late close of the OLD socket must not schedule anything
  assert.equal(timers.length, n);
});
