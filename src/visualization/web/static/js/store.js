// Latest value per topic, update counters, skipped-update (seq gap) tracking and a ring buffer of V2X frames.
// No DOM, no network: fed by ws.js, read by the views.

const RING_TOPICS = ['v2x.frame', 'v2x.inbox'];

export class Store {
  /** @param {{now?: () => number, ringSimSeconds?: number}} opts */
  constructor({ now = () => Date.now(), ringSimSeconds = 2.0 } = {}) {
    this.now = now;
    this.ringSimSeconds = ringSimSeconds;
    this.topics = new Map();          // topic -> {envelope, receivedAt, count, skipped, lastSeq}
    this.listeners = new Map();       // topic -> Set<fn>
    this.rings = new Map();           // topic -> [{frame, simTime, data}] ascending by frame (v2x.frame, v2x.inbox)
    this.connection = { state: 'closed', detail: '' };
    this.connectionListeners = new Set();
  }

  /** Called with every data envelope {topic, frame, sim_time, seq, data}. Returns the skipped count of this update. */
  put(envelope) {
    const t = envelope.topic;
    const prev = this.topics.get(t);
    let skipped = 0;
    if (prev && typeof envelope.seq === 'number' && typeof prev.lastSeq === 'number' && envelope.seq > prev.lastSeq + 1) {
      skipped = envelope.seq - prev.lastSeq - 1;
    }
    this.topics.set(t, {
      envelope,
      receivedAt: this.now(),
      count: (prev ? prev.count : 0) + 1,
      skipped: (prev ? prev.skipped : 0) + skipped,
      lastSeq: envelope.seq,
    });
    if (RING_TOPICS.includes(t)) this._pushRing(t, envelope);
    const ls = this.listeners.get(t);
    if (ls) for (const fn of ls) fn(envelope, skipped);
    return skipped;
  }

  get(topic) {
    const e = this.topics.get(topic);
    return e ? e.envelope : null;
  }

  info(topic) {
    return this.topics.get(topic) || null;
  }

  /** Wall-clock ms since the last message on a topic, or null if none yet. */
  ageMs(topic) {
    const e = this.topics.get(topic);
    return e ? this.now() - e.receivedAt : null;
  }

  on(topic, fn) {
    if (!this.listeners.has(topic)) this.listeners.set(topic, new Set());
    this.listeners.get(topic).add(fn);
    return () => this.listeners.get(topic).delete(fn);
  }

  setConnection(state, detail = '') {
    this.connection = { state, detail };
    for (const fn of this.connectionListeners) fn(this.connection);
  }

  onConnection(fn) {
    this.connectionListeners.add(fn);
    fn(this.connection);
    return () => this.connectionListeners.delete(fn);
  }

  clear() {
    this.topics.clear();
    this.rings.clear();
  }

  // ── V2X ring buffers ──────────────────────────────────────────────────────────────────────────
  get v2xRing() {
    return this.rings.get('v2x.frame') || [];
  }

  _pushRing(topic, envelope) {
    if (typeof envelope.frame !== 'number') return;
    let ring = this.rings.get(topic);
    if (!ring) { ring = []; this.rings.set(topic, ring); }
    const last = ring[ring.length - 1];
    if (last && last.frame === envelope.frame) return;
    if (last && envelope.frame < last.frame) ring.length = 0;           // the simulation restarted
    ring.push({ frame: envelope.frame, simTime: envelope.sim_time, summary: envelope.data, data: envelope.data });
    const horizon = envelope.sim_time - this.ringSimSeconds;
    while (ring.length > 1 && ring[0].simTime < horizon) ring.shift();
  }

  /** The newest entry of ``topic``'s ring with frame <= maxFrame (never a newer one), or null. */
  atOrBefore(topic, maxFrame) {
    let best = null;
    for (const f of this.rings.get(topic) || []) {
      if (f.frame <= maxFrame) best = f;
      else break;
    }
    return best;
  }

  v2xAtOrBefore(maxFrame) {
    return this.atOrBefore('v2x.frame', maxFrame);
  }
}
