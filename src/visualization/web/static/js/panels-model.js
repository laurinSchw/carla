// Pure logic of the side panels (inspector, link log, channel analytics). No DOM; unit tested in Node.

import { csvCell } from './units.js';
import { rowsToObjects } from './map-model.js';

const TIMESTAMP_FIELDS = ['timestamp', 'reference_time'];

/** Age in simulated seconds of a received message, or null when the message type carries no time stamp. */
export function messageAge(message, simTime) {
  for (const f of TIMESTAMP_FIELDS) {
    if (typeof message[f] === 'number' && Number.isFinite(message[f])) return simTime - message[f];
  }
  return null;
}

/** Rows of the inbox table: one per received message, with the link outcome fields next to it. */
export function inboxRows(inbox, simTime) {
  return inbox.messages.map((m) => ({
    tx_id: m.tx_id,
    link_type: m.link.link_type,
    msg_type: m.msg_type,
    age_s: messageAge(m.message, simTime),
    distance_m: m.link.distance_m,
    los: m.link.los,
    rx_power_dbm: m.link.rx_power_dbm,
    rx_margin_db: m.link.rx_margin_db,
    p_total: m.link.p_total,
    raw: m,
  }));
}

/** Delivery ratio of a station as a receiver over the last N V2X updates, from its v2x.links rows.
 *  Links removed by the awareness filter are not counted (the radio did not decide them): same rule as v2x.stats. */
export class DeliveryTracker {
  constructor(updates = 10) {
    this.updates = updates;
    this.buf = [];
  }

  push(frame, links) {
    if (this.buf.length && this.buf[this.buf.length - 1].frame === frame) return;
    if (this.buf.length && frame < this.buf[this.buf.length - 1].frame) this.buf = [];
    let counted = 0, delivered = 0;
    for (const l of links) {
      if (l.drop_cause === 'out_of_awareness_range') continue;
      counted += 1;
      if (l.delivered) delivered += 1;
    }
    this.buf.push({ frame, counted, delivered });
    if (this.buf.length > this.updates) this.buf.shift();
  }

  summary() {
    const counted = this.buf.reduce((a, b) => a + b.counted, 0);
    const delivered = this.buf.reduce((a, b) => a + b.delivered, 0);
    return { updates: this.buf.length, counted, delivered, ratio: counted ? delivered / counted : null };
  }

  reset() {
    this.buf = [];
  }
}

/** Messages a station transmitted per type over the last N updates (counts and rate per simulated second). */
export class SentCounter {
  constructor(updates = 10) {
    this.updates = updates;
    this.buf = [];
  }

  push(frame, simTime, sent) {
    if (this.buf.length && this.buf[this.buf.length - 1].frame === frame) return;
    if (this.buf.length && frame < this.buf[this.buf.length - 1].frame) this.buf = [];
    const counts = {};
    for (const m of sent) counts[m.type] = (counts[m.type] || 0) + 1;
    this.buf.push({ frame, simTime, counts });
    if (this.buf.length > this.updates) this.buf.shift();
  }

  summary() {
    const out = {};
    const n = this.buf.length;
    // n updates, spaced one period apart, cover n periods of simulated time (the span measures only n - 1 of them)
    const period = n > 1 ? (this.buf[n - 1].simTime - this.buf[0].simTime) / (n - 1) : null;
    for (const b of this.buf) {
      for (const [type, count] of Object.entries(b.counts)) {
        out[type] = out[type] || { count: 0, perSecond: null };
        out[type].count += count;
      }
    }
    for (const e of Object.values(out)) e.perSecond = period ? e.count / (n * period) : null;
    return out;
  }
}

/** Countdown to the end of a signal phase. Unknown timing is shown as unknown, never as a made-up number (S17). */
export function spatCountdown(state, simTime) {
  if (state.likely_time_s === null || state.likely_time_s === undefined) {
    return { text: 'unknown', title: 'the message carries no end time for this signal group' };
  }
  return { text: `${Math.max(0, state.likely_time_s - simTime).toFixed(1)} s`, title: `likely end of phase at t=${state.likely_time_s} s` };
}

// ── link log ─────────────────────────────────────────────────────────────────────────────────────

/** Client ring buffer of link rows (objects with ``frame`` and ``sim_time`` added). */
export class LinkLogBuffer {
  constructor(max = 5000) {
    this.max = max;
    this.rows = [];
    this.paused = false;
    this.lastFrame = null;
    this.dropped = 0;
  }

  /** ``data`` is a v2x.links payload ({columns, rows, ...}); ignored while paused or when the frame was seen. */
  push(frame, simTime, data) {
    if (this.paused || frame === this.lastFrame) return 0;
    this.lastFrame = frame;
    const objs = rowsToObjects(data.columns, data.rows);
    for (const o of objs) this.rows.push(o);
    const over = this.rows.length - this.max;
    if (over > 0) { this.rows.splice(0, over); this.dropped += over; }
    return objs.length;
  }

  clear() {
    this.rows = [];
    this.dropped = 0;
    this.lastFrame = null;
  }

  newest(n) {
    return this.rows.slice(-n).reverse();
  }

  /** CSV of the rows currently held, with a comment line that says what it is (a live, filtered view). */
  toCsv(columns, filterDescription) {
    const lines = [`# live view, filtered: ${filterDescription}; ${this.rows.length} rows held (client ring buffer of ${this.max}); not a recording`];
    lines.push(columns.map(csvCell).join(','));
    for (const r of this.rows) lines.push(columns.map((c) => csvCell(r[c])).join(','));
    return lines.join('\n') + '\n';
  }
}

// ── charts ───────────────────────────────────────────────────────────────────────────────────────

/** Series for the PRR-vs-distance chart from one ``prr_by_distance`` entry of v2x.stats. x = bin centres in metres.
 *  Bins without any link have null values (a gap, never a zero). */
export function prrSeries(entry, binStarts, binM) {
  const x = binStarts.map((s, i) => (i === binStarts.length - 1 ? s + binM / 2 : s + binM / 2));
  return { x, y: entry.prr, lo: entry.ci95_low, hi: entry.ci95_high, n: entry.counted };
}

export function marginBars(hist) {
  const e = hist.edges_db;
  const x = e.slice(0, -1).map((v, i) => (v + e[i + 1]) / 2);
  return { x, delivered: hist.delivered, dropped: hist.dropped };
}

/** Time series with a horizon in simulated seconds. */
export class TimeSeries {
  constructor(horizonS = 60) {
    this.horizonS = horizonS;
    this.t = [];
    this.v = [];
  }

  push(t, v) {
    if (this.t.length && t < this.t[this.t.length - 1]) { this.t = []; this.v = []; }
    if (this.t.length && t === this.t[this.t.length - 1]) return;
    this.t.push(t);
    this.v.push(v);
    while (this.t.length > 1 && this.t[0] < t - this.horizonS) { this.t.shift(); this.v.shift(); }
  }

  data() {
    return [this.t.slice(), this.v.slice()];
  }
}
