// Pure logic behind the map view (no DOM): link styling, trails, hit testing, and the "perspective of station X"
// model that remembers only what X actually received. Unit tested in Node.

import { worldToScreen } from './coords.js';

// ── links ───────────────────────────────────────────────────────────────────────────────────────
// Outcome is never colour-only: every outcome has its own dash pattern as well.
export const LINK_STYLES = {
  delivered: { token: '--link-ok', dash: [], label: 'delivered' },
  channel: { token: '--link-channel', dash: [7, 4], label: 'lost: channel' },
  mac: { token: '--link-mac', dash: [2, 4], label: 'lost: MAC contention' },
  out_of_awareness_range: { token: '--link-aware', dash: [9, 3, 2, 3], label: 'filtered: outside awareness range' },
};

export function linkOutcome(link) {
  if (link.delivered) return 'delivered';
  return LINK_STYLES[link.drop_cause] ? link.drop_cause : 'channel';
}

/** Opacity proportional to the success probability, but never invisible. */
export function linkOpacity(pTotal) {
  if (!Number.isFinite(pTotal)) return 0.35;
  return Math.min(1, Math.max(0.15, pTotal));
}

/** {columns, rows} (the v2x.links format) -> array of objects. */
export function rowsToObjects(columns, rows) {
  return rows.map((r) => {
    const o = {};
    for (let i = 0; i < columns.length; i++) o[columns[i]] = r[i];
    return o;
  });
}

// ── stations ─────────────────────────────────────────────────────────────────────────────────────

/** Glyph per station kind (shape differs, so kind is not conveyed by colour alone). */
export function stationGlyph(s) {
  if (s.kind === 'RSU') return { shape: 'square', filled: true };
  if (s.kind === 'VRU' || s.kind === 'pedestrian') return { shape: 'circle', filled: true };
  const equipped = s.v2x !== false;                 // world.state: v2x flag; v2x.frame stations are all equipped
  return { shape: 'arrow', filled: equipped };
}

/** Colour by age of the information: fresh = ok, older = warn, beyond two update periods = stale grey. */
export function ageClass(ageS, periodS) {
  if (!Number.isFinite(ageS)) return 'stale';
  if (ageS <= 1.01 * periodS) return 'fresh';
  if (ageS <= 2 * periodS) return 'aging';
  return 'stale';
}

export function hitTestStations(stations, vp, sx, sy, radiusPx = 10) {
  let best = null;
  let bestD = radiusPx * radiusPx;
  for (const s of stations) {
    const [px, py] = worldToScreen(vp, s.x, s.y);
    const d = (px - sx) ** 2 + (py - sy) ** 2;
    if (d <= bestD) { best = s; bestD = d; }
  }
  return best;
}

/** Distance in px from a point to a segment. */
export function distToSegment(px, py, ax, ay, bx, by) {
  const dx = bx - ax, dy = by - ay;
  const len2 = dx * dx + dy * dy;
  let t = len2 === 0 ? 0 : ((px - ax) * dx + (py - ay) * dy) / len2;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

export function hitTestLinks(drawn, sx, sy, tolPx = 5) {
  let best = null;
  let bestD = tolPx;
  for (const l of drawn) {
    const d = distToSegment(sx, sy, l.ax, l.ay, l.bx, l.by);
    if (d <= bestD) { best = l; bestD = d; }
  }
  return best;
}

// ── trails ───────────────────────────────────────────────────────────────────────────────────────

/** Positions of the last ``seconds`` of simulation time per station id. Ground truth only: no smoothing. */
export class Trails {
  constructor(seconds = 5) {
    this.seconds = seconds;
    this.byId = new Map();
    this.lastSim = -Infinity;
  }

  push(simTime, stations) {
    if (simTime < this.lastSim) this.byId.clear();             // the simulation restarted
    this.lastSim = simTime;
    const seen = new Set();
    for (const s of stations) {
      seen.add(s.id);
      let t = this.byId.get(s.id);
      if (!t) { t = []; this.byId.set(s.id, t); }
      const last = t[t.length - 1];
      if (!last || last.t !== simTime) t.push({ t: simTime, x: s.x, y: s.y });
      while (t.length && t[0].t < simTime - this.seconds) t.shift();
    }
    for (const id of [...this.byId.keys()]) if (!seen.has(id)) this.byId.delete(id);
  }

  get(id) {
    return this.byId.get(id) || [];
  }
}

// ── perspective of one station ───────────────────────────────────────────────────────────────────

/** What a station knows: built only from the messages it received (topic v2x.inbox), drawn where the
 *  *messages* say things are, with their age. Stations not heard in the latest update stay as ghosts. */
export class Perspective {
  constructor({ ghostSeconds = 10 } = {}) {
    this.ghostSeconds = ghostSeconds;
    this.stationId = null;
    this.known = new Map();           // key -> entity
    this.lastFrame = null;
    this.spat = new Map();            // intersection_id -> {message, simTime}
  }

  reset(stationId) {
    this.stationId = stationId;
    this.known.clear();
    this.spat.clear();
    this.lastFrame = null;
  }

  /** @param inbox the ``data`` of a v2x.inbox envelope; frame/simTime from the envelope. */
  update(inbox, frame, simTime) {
    if (inbox.station_id !== this.stationId) return;
    if (this.lastFrame !== null && frame < this.lastFrame) this.reset(this.stationId);
    this.lastFrame = frame;
    for (const e of this.known.values()) e.fresh = false;
    for (const m of inbox.messages) {
      const msg = m.message;
      const base = { fresh: true, frame, rxSimTime: simTime, link: m.link, txId: m.tx_id, type: m.msg_type };
      if (m.msg_type === 'BSM') {
        this._set(`v${m.tx_id}`, { ...base, kind: 'vehicle', id: m.tx_id, x: msg.x_m, y: msg.y_m, heading: msg.heading,
          speed: msg.speed, generatedAt: msg.timestamp, length: msg.vehicle_length, width: msg.vehicle_width });
      } else if (m.msg_type === 'VRUAM') {
        this._set(`p${m.tx_id}`, { ...base, kind: 'vru', id: m.tx_id, x: msg.position[0], y: msg.position[1],
          heading: msg.heading, speed: msg.speed, generatedAt: msg.timestamp });
      } else if (m.msg_type === 'MAP') {
        this._set(`r${msg.intersection_id}`, { ...base, kind: 'rsu', id: m.tx_id, x: msg.ref_point[0], y: msg.ref_point[1],
          generatedAt: msg.timestamp, intersection: msg.intersection_id });
      } else if (m.msg_type === 'SPAT') {
        this.spat.set(msg.intersection_id, { message: msg, simTime });
      } else if (m.msg_type === 'DENM') {
        this._set(`d${m.tx_id}:${msg.action_id}`, { ...base, kind: 'denm', id: msg.action_id, x: msg.event_position[0],
          y: msg.event_position[1], generatedAt: msg.reference_time, radius: msg.relevance_distance, cause: msg.cause_code });
      }
    }
    for (const [k, e] of [...this.known]) {
      if (simTime - e.rxSimTime > this.ghostSeconds) this.known.delete(k);
    }
  }

  _set(key, entity) {
    this.known.set(key, entity);
  }

  /** Entities with their age at ``simTime`` (age of the information: now minus the message's generation time). */
  entities(simTime) {
    return [...this.known.values()].map((e) => ({
      ...e, ghost: !e.fresh, age: Number.isFinite(e.generatedAt) ? simTime - e.generatedAt : simTime - e.rxSimTime,
    }));
  }
}

export function geometryBounds(geometry) {
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const lane of geometry.lanes) {
    for (const [x, y] of lane) {
      if (x < minX) minX = x; if (x > maxX) maxX = x;
      if (y < minY) minY = y; if (y > maxY) maxY = y;
    }
  }
  return Number.isFinite(minX) ? { minX, minY, maxX, maxY } : null;
}
