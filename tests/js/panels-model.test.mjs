import test from 'node:test';
import assert from 'node:assert/strict';
import {
  messageAge, inboxRows, DeliveryTracker, SentCounter, spatCountdown, LinkLogBuffer, prrSeries, marginBars, TimeSeries,
} from '../../src/web/static/js/panels-model.js';

test('message age comes from the message time stamp, null when it has none', () => {
  assert.equal(messageAge({ timestamp: 4.5 }, 5), 0.5);
  assert.equal(messageAge({ reference_time: 1 }, 5), 4);
  assert.equal(messageAge({ foo: 1 }, 5), null);
});

test('inbox rows carry the link fields next to the message', () => {
  const inbox = { messages: [{ tx_id: 3, msg_type: 'BSM', message: { timestamp: 9.5 },
    link: { link_type: 'V2V', distance_m: 12.5, los: true, rx_power_dbm: -61.2, rx_margin_db: 20.1, p_total: 0.99 } }] };
  const [r] = inboxRows(inbox, 10);
  assert.deepEqual({ ...r, raw: undefined }, { tx_id: 3, link_type: 'V2V', msg_type: 'BSM', age_s: 0.5, distance_m: 12.5, los: true,
    rx_power_dbm: -61.2, rx_margin_db: 20.1, p_total: 0.99, raw: undefined });
  assert.equal(r.raw.tx_id, 3);
});

test('delivery ratio over the last N updates ignores awareness-filtered links', () => {
  const d = new DeliveryTracker(2);
  d.push(1, [{ delivered: true }, { delivered: false, drop_cause: 'channel' }, { delivered: false, drop_cause: 'out_of_awareness_range' }]);
  assert.deepEqual(d.summary(), { updates: 1, counted: 2, delivered: 1, ratio: 0.5 });
  d.push(1, [{ delivered: true }]);                                 // same frame again: ignored
  d.push(2, [{ delivered: true }, { delivered: true }]);
  assert.equal(d.summary().ratio, 0.75);
  d.push(3, [{ delivered: false, drop_cause: 'mac' }]);             // frame 1 falls out of the window of 2
  assert.deepEqual(d.summary(), { updates: 2, counted: 3, delivered: 2, ratio: 2 / 3 });
  d.push(0, []);                                                    // simulation restarted
  assert.equal(d.summary().updates, 1);
  assert.equal(new DeliveryTracker().summary().ratio, null);
});

test('sent counter: counts per type and a rate per simulated second', () => {
  const s = new SentCounter(10);
  for (let k = 0; k < 5; k++) s.push(10 * k, 0.5 * k, [{ type: 'BSMCore' }, ...(k % 2 ? [{ type: 'DENM' }] : [])]);
  const sum = s.summary();
  assert.equal(sum.BSMCore.count, 5); assert.equal(sum.DENM.count, 2);
  // 5 updates spaced 0.5 s span 2.5 s of simulated time
  assert.ok(Math.abs(sum.BSMCore.perSecond - 2.0) < 1e-9, `rate ${sum.BSMCore.perSecond}`);
  const one = new SentCounter(); one.push(1, 0, [{ type: 'BSMCore' }]);
  assert.equal(one.summary().BSMCore.perSecond, null, 'one update gives no rate');
});

test('SPAT countdown: unknown stays unknown', () => {
  assert.equal(spatCountdown({ likely_time_s: null }, 5).text, 'unknown');
  assert.match(spatCountdown({ likely_time_s: null }, 5).title, /no end time/);
  assert.equal(spatCountdown({ likely_time_s: 12.5 }, 10).text, '2.5 s');
  assert.equal(spatCountdown({ likely_time_s: 9 }, 10).text, '0.0 s');
});

test('link log: ring buffer, pause, dedupe by frame, newest first', () => {
  const b = new LinkLogBuffer(5);
  const data = (n) => ({ columns: ['tx_id', 'rx_id'], rows: Array.from({ length: n }, (_, i) => [i, 100 + i]) });
  assert.equal(b.push(1, 0.5, data(3)), 3);
  assert.equal(b.push(1, 0.5, data(3)), 0, 'same frame twice');
  b.paused = true;
  assert.equal(b.push(2, 1.0, data(3)), 0, 'paused');
  b.paused = false;
  b.push(3, 1.5, data(4));
  assert.equal(b.rows.length, 5); assert.equal(b.dropped, 2);
  assert.deepEqual(b.newest(2).map((r) => r.tx_id), [3, 2]);
  b.clear();
  assert.equal(b.rows.length, 0);
});

test('CSV: comment line says it is a live filtered view; header; escaping', () => {
  const b = new LinkLogBuffer(10);
  b.push(1, 0, { columns: ['a', 'b'], rows: [[1, 'x,y'], [2, null], [3, 'say "hi"']] });
  const lines = b.toCsv(['a', 'b'], 'outcome=dropped').trimEnd().split('\n');
  assert.match(lines[0], /^# live view, filtered: outcome=dropped; 3 rows held/);
  assert.deepEqual(lines.slice(1), ['a,b', '1,"x,y"', '2,', '3,"say ""hi"""']);
});

test('prr series: bin centres, gaps stay null, intervals pass through', () => {
  const s = prrSeries({ prr: [1, null, 0.5], ci95_low: [0.7, null, 0.2], ci95_high: [1, null, 0.8], counted: [10, 0, 4] }, [0, 10, 20], 10);
  assert.deepEqual(s.x, [5, 15, 25]); assert.deepEqual(s.y, [1, null, 0.5]); assert.deepEqual(s.n, [10, 0, 4]);
});

test('margin bars use bin centres', () => {
  const m = marginBars({ edges_db: [-30, -25, -20], delivered: [1, 2], dropped: [3, 4] });
  assert.deepEqual(m.x, [-27.5, -22.5]); assert.deepEqual(m.delivered, [1, 2]);
});

test('time series keeps a horizon of simulated seconds and resets when time goes backwards', () => {
  const t = new TimeSeries(2);
  for (let k = 0; k <= 8; k++) t.push(k * 0.5, k);
  const [x, y] = t.data();
  assert.deepEqual(x, [2, 2.5, 3, 3.5, 4]); assert.deepEqual(y, [4, 5, 6, 7, 8]);
  t.push(4, 99);                                                    // same time: ignored
  assert.equal(t.data()[1].length, 5);
  t.push(1, 1);
  assert.deepEqual(t.data(), [[1], [1]]);
});
