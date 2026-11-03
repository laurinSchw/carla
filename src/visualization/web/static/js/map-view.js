// Canvas map: roads, stations, trails, links of the selected station, DENMs, signal states, LiDAR markers.
//
// Two modes:
//   truth        - ground truth from topic world.state (observer/admin);
//   perspective  - only what station X received (topic v2x.inbox), drawn where the received messages say things are,
//                  with their age; stations X did not hear in the latest update stay as ghosts.
// Nothing is interpolated or extrapolated between updates: the latest state is drawn and its frame is shown.

import {
  createViewport, worldToScreen, screenToWorld, yawToCanvasAngle, zoomAt, fitBounds,
} from './coords.js';
import {
  LINK_STYLES, linkOutcome, linkOpacity, rowsToObjects, stationGlyph, ageClass, hitTestStations, hitTestLinks,
  Trails, Perspective, geometryBounds,
} from './map-model.js';
import * as u from './units.js';

export const DEFAULT_LAYERS = {
  roads: true, stations: true, labels: false, trails: true, awareness: true, links: true, denm: true,
  signals: true, lidar: true,
};

const SIGNAL_COLOURS = { protectedMovementAllowed: ['#2e9e4f', 'G'], permissiveMovementAllowed: ['#2e9e4f', 'G'],
  protectedClearance: ['#d9a400', 'Y'], permissiveClearance: ['#d9a400', 'Y'], 'stop-And-Remain': ['#d03b3b', 'R'],
  'stop-Then-Proceed': ['#d03b3b', 'R'], 'caution-Conflicting-Traffic': ['#d9a400', '!'], dark: ['#666', '-'],
  unavailable: ['#666', '?'] };

export class MapView {
  constructor(canvas, { store, onSelect = () => {}, onSelectLink = () => {}, getConfig = () => ({}), getSelf = () => null } = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.store = store;
    this.onSelect = onSelect;
    this.onSelectLink = onSelectLink;
    this.getConfig = getConfig;
    this.getSelf = getSelf;             // vehicle page: the own position from own telemetry (no ground truth available)
    this.layers = { ...DEFAULT_LAYERS };
    this.vp = createViewport({ width: 800, height: 520, cx: 100, cy: 100, scale: 2 });
    this.geometry = null;
    this.mode = 'truth';
    this.selected = null;               // station id (truth or perspective subject)
    this.follow = false;
    this.trails = new Trails(5);
    this.perspective = new Perspective();
    this.drawnLinks = [];
    this.drawnStations = [];
    this.selectedLink = null;
    this.pickHandler = null;           // admin: the next click on empty map returns world coordinates instead of selecting
    this.dirty = true;
    this.fitted = false;
    this.lastTrailFrame = null;
    this._bind();
    this._resize();
    new ResizeObserver(() => { this._resize(); this.requestRender(); }).observe(canvas);
  }

  // ── configuration ───────────────────────────────────────────────────────────────────────────
  setGeometry(g) {
    this.geometry = g;
    if (!this.fitted) this.fit();
    this.requestRender();
  }

  setLayers(partial) {
    Object.assign(this.layers, partial);
    this.requestRender();
  }

  setMode(mode, stationId = this.selected) {
    this.mode = mode;
    if (mode === 'perspective') this.perspective.reset(stationId);
    this.requestRender();
  }

  setSelected(id) {
    this.selected = id;
    if (this.mode === 'perspective') this.perspective.reset(id);
    this.selectedLink = null;
    this.requestRender();
  }

  setFollow(on) {
    this.follow = !!on;
    this.requestRender();
  }

  fit() {
    const b = this.geometry && geometryBounds(this.geometry);
    if (b) {
      this.vp = fitBounds(this.vp, b.minX - 20, b.minY - 20, b.maxX + 20, b.maxY + 20, 1);
      this.fitted = true;
    }
    this.requestRender();
  }

  /** Feed ground-truth stations so trails can be built (called by the page for every world.state). */
  onWorldState(env) {
    if (env.frame === this.lastTrailFrame) return;
    this.lastTrailFrame = env.frame;
    this.trails.push(env.sim_time, env.data.stations);
  }

  onInbox(env) {
    if (this.mode === 'perspective') this.perspective.update(env.data, env.frame, env.sim_time);
  }

  // ── events ──────────────────────────────────────────────────────────────────────────────────
  _bind() {
    const c = this.canvas;
    c.tabIndex = 0;
    c.setAttribute('role', 'application');
    c.setAttribute('aria-label', 'Map. Arrow keys pan, plus and minus zoom, zero fits, F follows the selected station.');
    let drag = null;
    c.addEventListener('pointerdown', (e) => {
      c.setPointerCapture(e.pointerId);
      drag = { x: e.clientX, y: e.clientY, cx: this.vp.cx, cy: this.vp.cy, moved: false };
    });
    c.addEventListener('pointermove', (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      if (Math.abs(dx) + Math.abs(dy) > 3) drag.moved = true;
      if (drag.moved) {
        this.vp = { ...this.vp, cx: drag.cx - dx / this.vp.scale, cy: drag.cy - dy / this.vp.scale };
        this.follow = false;
        this.requestRender();
      }
    });
    c.addEventListener('pointerup', (e) => {
      const d = drag;
      drag = null;
      if (d && !d.moved) this._click(e);
    });
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const r = c.getBoundingClientRect();
      this.vp = zoomAt(this.vp, e.clientX - r.left, e.clientY - r.top, Math.exp(-e.deltaY * 0.0015));
      this.requestRender();
    }, { passive: false });
    c.addEventListener('dblclick', () => this.fit());
    c.addEventListener('keydown', (e) => {
      const step = 40 / this.vp.scale;
      const k = e.key;
      if (k === 'ArrowLeft') this.vp.cx -= step;
      else if (k === 'ArrowRight') this.vp.cx += step;
      else if (k === 'ArrowUp') this.vp.cy -= step;
      else if (k === 'ArrowDown') this.vp.cy += step;
      else if (k === '+' || k === '=') this.vp = zoomAt(this.vp, this.vp.width / 2, this.vp.height / 2, 1.25);
      else if (k === '-') this.vp = zoomAt(this.vp, this.vp.width / 2, this.vp.height / 2, 0.8);
      else if (k === '0') this.fit();
      else if (k === 'f' || k === 'F') this.follow = !this.follow;
      else if (k === 'Escape') { this.onSelect(null); }
      else return;
      e.preventDefault();
      this.requestRender();
    });
  }

  _click(e) {
    const r = this.canvas.getBoundingClientRect();
    const sx = e.clientX - r.left, sy = e.clientY - r.top;
    if (this.pickHandler) {
      const [wx, wy] = screenToWorld(this.vp, sx, sy);
      const h = this.pickHandler;
      this.pickHandler = null;
      this.canvas.style.cursor = '';
      h(wx, wy);
      return;
    }
    const st = hitTestStations(this.drawnStations, this.vp, sx, sy, 11);
    if (st) { this.onSelect(st.id, st); return; }
    const l = hitTestLinks(this.drawnLinks, sx, sy, 5);
    if (l) { this.selectedLink = l.link; this.onSelectLink(l.link); this.requestRender(); return; }
    this.onSelect(null);
  }

  _resize() {
    const dpr = window.devicePixelRatio || 1;
    const w = this.canvas.clientWidth || 800, h = this.canvas.clientHeight || 520;
    this.canvas.width = Math.round(w * dpr);
    this.canvas.height = Math.round(h * dpr);
    this.dpr = dpr;
    this.vp = { ...this.vp, width: w, height: h };
  }

  requestRender() {
    if (this.dirty && this._raf) return;
    this.dirty = true;
    this._raf = requestAnimationFrame(() => { this._raf = null; this.render(); });
  }

  // ── drawing ─────────────────────────────────────────────────────────────────────────────────
  _colours() {
    const cs = getComputedStyle(this.canvas);
    const v = (n, d) => cs.getPropertyValue(n).trim() || d;
    return { bg: v('--panel-2', '#eee'), road: v('--border', '#ccc'), text: v('--text', '#111'), muted: v('--muted', '#666'),
      accent: v('--accent', '#06c'), ok: v('--ok', '#1a7f37'), warn: v('--warn', '#9a6700'), bad: v('--bad', '#c62828'),
      stale: v('--stale', '#888'), panel: v('--panel', '#fff'),
      links: Object.fromEntries(Object.entries(LINK_STYLES).map(([k, s]) => [k, v(s.token, '#888')])) };
  }

  render() {
    this.dirty = false;
    const ctx = this.ctx;
    const col = this._colours();
    const vpFollow = this._followed();
    if (vpFollow) this.vp = { ...this.vp, cx: vpFollow.x, cy: vpFollow.y };
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, this.vp.width, this.vp.height);
    ctx.fillStyle = col.bg;
    ctx.fillRect(0, 0, this.vp.width, this.vp.height);
    if (this.layers.roads) this._roads(ctx, col);

    const world = this.store.get('world.state');
    const v2x = this.store.get('v2x.frame');
    this.drawnStations = [];
    this.drawnLinks = [];
    if (this.mode === 'perspective') this._renderPerspective(ctx, col, world, v2x);
    else this._renderTruth(ctx, col, world, v2x);
    this._lidarMarkers(ctx, col, world);
    this._overlay(ctx, col, world, v2x);
    const d = this.canvas.dataset;
    d.mode = this.mode;
    d.stations = String(this.drawnStations.length);
    d.links = String(this.drawnLinks.length);
    d.frame = world ? String(world.frame) : '';
    d.selected = this.selected === null ? '' : String(this.selected);
  }

  _followed() {
    if (!this.follow || this.selected === null) return null;
    const me = this.getSelf();
    if (me && me.id === this.selected) return me;
    const world = this.store.get('world.state');
    const st = world && world.data.stations.find((s) => s.id === this.selected);
    return st || null;
  }

  _roads(ctx, col) {
    const g = this.geometry;
    if (!g) return;
    ctx.save();
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.strokeStyle = col.road;
    ctx.lineWidth = Math.max(1, 3.5 * this.vp.scale);
    ctx.globalAlpha = 0.55;
    ctx.beginPath();
    for (const lane of g.lanes) {
      const [x0, y0] = worldToScreen(this.vp, lane[0][0], lane[0][1]);
      ctx.moveTo(x0, y0);
      for (let i = 1; i < lane.length; i++) {
        const [x, y] = worldToScreen(this.vp, lane[i][0], lane[i][1]);
        ctx.lineTo(x, y);
      }
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.strokeStyle = col.muted;
    ctx.lineWidth = 1;
    for (const j of g.junctions) {
      const [x0, y0] = worldToScreen(this.vp, j.bbox[0], j.bbox[1]);
      const [x1, y1] = worldToScreen(this.vp, j.bbox[2], j.bbox[3]);
      ctx.setLineDash([3, 3]);
      ctx.strokeRect(x0, y0, x1 - x0, y1 - y0);
    }
    ctx.restore();
  }

  _glyph(ctx, col, s, { x, y, yaw, colour, ghost = false, selected = false }) {
    const [sx, sy] = worldToScreen(this.vp, x, y);
    const g = stationGlyph(s);
    ctx.save();
    ctx.translate(sx, sy);
    ctx.strokeStyle = colour;
    ctx.fillStyle = colour;
    ctx.lineWidth = selected ? 3 : 1.5;
    if (ghost) ctx.setLineDash([2, 2]);
    ctx.beginPath();
    if (g.shape === 'arrow') {
      ctx.rotate(yawToCanvasAngle(yaw || 0));
      ctx.moveTo(9, 0); ctx.lineTo(-6, 5); ctx.lineTo(-3, 0); ctx.lineTo(-6, -5); ctx.closePath();
    } else if (g.shape === 'square') {
      ctx.rect(-5, -5, 10, 10);
    } else {
      ctx.arc(0, 0, 4, 0, Math.PI * 2);
    }
    if (g.filled && !ghost) ctx.fill();
    ctx.stroke();
    if (selected) {
      ctx.setLineDash([]);
      ctx.beginPath();
      ctx.arc(0, 0, 13, 0, Math.PI * 2);
      ctx.strokeStyle = col.accent;
      ctx.stroke();
    }
    ctx.restore();
    return [sx, sy];
  }

  _label(ctx, col, text, sx, sy) {
    ctx.save();
    ctx.fillStyle = col.text;
    ctx.font = '11px system-ui, sans-serif';
    ctx.fillText(text, sx + 9, sy - 8);
    ctx.restore();
  }

  _renderTruth(ctx, col, world, v2x) {
    if (!world) return;
    const period = 0.5;
    const staleWorld = this.store.ageMs('world.state') > 1500;
    const byId = new Map();
    for (const s of world.data.stations) byId.set(s.id, { ...s, kind: s.kind });
    for (const r of world.data.rsus || []) byId.set(r.station_id, { id: r.station_id, kind: 'RSU', x: r.x, y: r.y, name: `RSU ${r.intersection_id}` });
    this.stationIndex = byId;

    if (this.layers.trails) {
      ctx.save();
      ctx.strokeStyle = col.muted;
      ctx.globalAlpha = 0.6;
      ctx.lineWidth = 1;
      for (const s of world.data.stations) {
        const t = this.trails.get(s.id);
        if (t.length < 2) continue;
        ctx.beginPath();
        t.forEach((p, i) => { const [x, y] = worldToScreen(this.vp, p.x, p.y); if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
        ctx.stroke();
      }
      ctx.restore();
    }

    const sel = this.selected !== null ? byId.get(this.selected) : null;
    if (sel && this.layers.awareness) {
      const aw = this.getConfig().awareness_range_m;
      if (aw) {
        const [sx, sy] = worldToScreen(this.vp, sel.x, sel.y);
        ctx.save();
        ctx.setLineDash([6, 6]);
        ctx.strokeStyle = col.accent;
        ctx.beginPath();
        ctx.arc(sx, sy, aw * this.vp.scale, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
      }
    }

    if (this.layers.links) this._links(ctx, col, byId);
    if (this.layers.denm && v2x) this._denms(ctx, col, v2x.data.denms || []);
    if (this.layers.signals && v2x) this._signals(ctx, col, world, v2x);

    if (this.layers.stations) {
      const all = [...byId.values()];
      for (const s of all) {
        const colour = staleWorld ? col.stale : s.kind === 'RSU' ? col.warn : s.kind === 'pedestrian' ? col.bad
          : s.role === 'client' ? col.accent : col.ok;
        const [sx, sy] = this._glyph(ctx, col, s, { x: s.x, y: s.y, yaw: s.yaw, colour, selected: s.id === this.selected });
        this.drawnStations.push({ id: s.id, x: s.x, y: s.y, kind: s.kind, name: s.name });
        if (this.layers.labels) this._label(ctx, col, s.name || `#${s.id}`, sx, sy);
      }
    }
    void period;
  }

  _links(ctx, col, byId) {
    const env = this.store.get('v2x.links');
    if (!env || !env.data.columns) return;
    const rows = rowsToObjects(env.data.columns, env.data.rows);
    for (const l of rows) {
      const a = byId.get(l.tx_id), b = byId.get(l.rx_id);
      if (!a || !b) continue;
      const out = linkOutcome(l);
      const st = LINK_STYLES[out];
      const [ax, ay] = worldToScreen(this.vp, a.x, a.y);
      const [bx, by] = worldToScreen(this.vp, b.x, b.y);
      ctx.save();
      ctx.strokeStyle = col.links[out];
      ctx.globalAlpha = linkOpacity(l.p_total);
      ctx.lineWidth = l === this.selectedLink ? 3 : 1.5;
      ctx.setLineDash(st.dash);
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
      ctx.restore();
      this.drawnLinks.push({ ax, ay, bx, by, link: l });
    }
  }

  _denms(ctx, col, denms) {
    for (const d of denms) {
      const p = d.event_position;
      const [sx, sy] = worldToScreen(this.vp, p[0], p[1]);
      ctx.save();
      ctx.strokeStyle = col.bad;
      ctx.setLineDash([4, 3]);
      ctx.beginPath(); ctx.arc(sx, sy, d.relevance_distance * this.vp.scale, 0, Math.PI * 2); ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = col.bad;
      ctx.beginPath(); ctx.moveTo(sx, sy - 9); ctx.lineTo(sx + 8, sy + 6); ctx.lineTo(sx - 8, sy + 6); ctx.closePath(); ctx.fill();
      ctx.fillStyle = col.panel;
      ctx.font = 'bold 10px system-ui'; ctx.textAlign = 'center'; ctx.fillText('!', sx, sy + 5);
      ctx.restore();
    }
  }

  _signals(ctx, col, world, v2x) {
    const states = v2x.data.rsu_states || {};
    for (const r of world.data.rsus || []) {
      const spat = (states[String(r.station_id)] || {}).spat;
      if (!spat) continue;
      const [sx, sy] = worldToScreen(this.vp, r.x, r.y);
      spat.states.forEach((st, i) => {
        const [c, letter] = SIGNAL_COLOURS[st.movement_event] || ['#666', '?'];
        const a = (i / Math.max(spat.states.length, 1)) * Math.PI * 2 - Math.PI / 2;
        const px = sx + Math.cos(a) * 12, py = sy + Math.sin(a) * 12;
        ctx.save();
        ctx.fillStyle = c;
        ctx.beginPath(); ctx.arc(px, py, 4.5, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = '#fff'; ctx.font = 'bold 7px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillText(letter, px, py + 0.5);
        ctx.restore();
      });
    }
  }

  _lidarMarkers(ctx, col, world) {
    if (!this.layers.lidar) return;
    const env = this.store.get('lidar.sensors');
    if (!env || !world) return;
    const byId = new Map(world.data.stations.map((s) => [s.id, s]));
    for (const s of env.data.sensors) {
      if (s.state !== 'active') continue;
      const v = byId.get(s.actor_id);
      if (!v) continue;
      const [sx, sy] = worldToScreen(this.vp, v.x, v.y);
      const range = s.config.range_m * this.vp.scale;
      const yaw = yawToCanvasAngle(v.yaw + (s.config.mount ? s.config.mount.yaw_deg : 0));
      const half = (s.config.horizontal_fov_deg / 2) * Math.PI / 180;
      ctx.save();
      ctx.strokeStyle = col.accent;
      ctx.fillStyle = col.accent;
      ctx.globalAlpha = 0.12;
      ctx.beginPath();
      if (s.config.horizontal_fov_deg >= 359.9) ctx.arc(sx, sy, range, 0, Math.PI * 2);
      else { ctx.moveTo(sx, sy); ctx.arc(sx, sy, range, yaw - half, yaw + half); ctx.closePath(); }
      ctx.fill();
      ctx.globalAlpha = 0.8;
      ctx.setLineDash([1, 3]);
      ctx.stroke();
      ctx.restore();
      this._label(ctx, col, `LiDAR #${s.sensor_id}`, sx, sy + 18);
    }
  }

  _renderPerspective(ctx, col, world, v2x) {
    const subject = this.selected;
    const w = this.getSelf() || (world && world.data.stations.find((s) => s.id === subject));
    const inbox = this.store.get('v2x.inbox');
    const simTime = inbox ? inbox.sim_time : world ? world.sim_time : 0;
    const period = 0.5;
    // the subject knows its own position; everything else only through received messages
    if (w) {
      this._glyph(ctx, col, { kind: 'vehicle', v2x: true }, { x: w.x, y: w.y, yaw: w.yaw, colour: col.accent, selected: true });
      this.drawnStations.push({ id: w.id ?? subject, x: w.x, y: w.y, kind: 'vehicle', name: w.name });
      const aw = this.getConfig().awareness_range_m;
      if (aw && this.layers.awareness) {
        const [sx, sy] = worldToScreen(this.vp, w.x, w.y);
        ctx.save(); ctx.setLineDash([6, 6]); ctx.strokeStyle = col.accent; ctx.beginPath(); ctx.arc(sx, sy, aw * this.vp.scale, 0, Math.PI * 2); ctx.stroke(); ctx.restore();
      }
    }
    const ents = this.perspective.entities(simTime);
    for (const e of ents) {
      const cls = e.ghost ? 'stale' : ageClass(e.age, period);
      const colour = cls === 'fresh' ? col.ok : cls === 'aging' ? col.warn : col.stale;
      if (e.kind === 'denm') {
        this._denms(ctx, col, [{ event_position: [e.x, e.y, 0], relevance_distance: e.radius }]);
        continue;
      }
      if (w && this.layers.links && e.link && e.link.delivered !== undefined) {
        const [ax, ay] = worldToScreen(this.vp, e.x, e.y), [bx, by] = worldToScreen(this.vp, w.x, w.y);
        ctx.save();
        ctx.strokeStyle = col.links.delivered;
        ctx.globalAlpha = e.ghost ? 0.15 : linkOpacity(e.link.p_total);
        ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
        ctx.restore();
        if (!e.ghost) this.drawnLinks.push({ ax, ay, bx, by, link: e.link });
      }
      const kind = e.kind === 'rsu' ? 'RSU' : e.kind === 'vru' ? 'VRU' : 'vehicle';
      const [sx, sy] = this._glyph(ctx, col, { kind, v2x: true }, { x: e.x, y: e.y, yaw: e.heading, colour, ghost: e.ghost });
      this.drawnStations.push({ id: e.id, x: e.x, y: e.y, kind, name: `#${e.id}` });
      this._label(ctx, col, `#${e.id} ${u.age(e.age)}${e.ghost ? ' (ghost)' : ''}`, sx, sy);
    }
    if (v2x && this.layers.signals) {
      for (const [iid, sp] of this.perspective.spat) {
        const rsu = ents.find((e) => e.kind === 'rsu' && e.intersection === iid);
        if (!rsu) continue;
        const [sx, sy] = worldToScreen(this.vp, rsu.x, rsu.y);
        sp.message.states.forEach((st, i) => {
          const [c, letter] = SIGNAL_COLOURS[st.movement_event] || ['#666', '?'];
          const a = (i / Math.max(sp.message.states.length, 1)) * Math.PI * 2 - Math.PI / 2;
          ctx.save();
          ctx.fillStyle = c;
          ctx.beginPath(); ctx.arc(sx + Math.cos(a) * 12, sy + Math.sin(a) * 12, 4.5, 0, Math.PI * 2); ctx.fill();
          ctx.fillStyle = '#fff'; ctx.font = 'bold 7px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
          ctx.fillText(letter, sx + Math.cos(a) * 12, sy + Math.sin(a) * 12 + 0.5);
          ctx.restore();
        });
      }
    }
  }

  _overlay(ctx, col, world, v2x) {
    ctx.save();
    ctx.fillStyle = col.text;
    ctx.font = '11px ui-monospace, monospace';
    const label = this.mode === 'perspective' ? `PERSPECTIVE of #${this.selected} (received data only)` : 'GROUND TRUTH';
    const ageW = this.store.ageMs('world.state');
    const stale = ageW !== null && ageW > 1500;
    const txt = `${label}  ·  world ${world ? u.stamp(world.frame, world.sim_time) : u.DASH}  ·  V2X ${v2x ? u.stamp(v2x.frame, v2x.sim_time) : u.DASH}${stale ? '  ·  STALE' : ''}`;
    ctx.fillText(txt, 8, 14);
    // scale bar
    const metres = [10, 20, 50, 100, 200, 500].find((m) => m * this.vp.scale >= 60) || 500;
    const w = metres * this.vp.scale;
    const y = this.vp.height - 12;
    ctx.strokeStyle = col.text; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(10, y); ctx.lineTo(10 + w, y); ctx.stroke();
    ctx.fillText(`${metres} m`, 14, y - 5);
    // north-up note: CARLA +x is east, +y is south on screen (not mirrored)
    ctx.fillText('x →   y ↓', this.vp.width - 60, this.vp.height - 8);
    ctx.restore();
  }

  /** The next click on the map calls ``handler(x, y)`` with CARLA world coordinates (metres). */
  pick(handler) {
    this.pickHandler = handler;
    this.canvas.style.cursor = 'crosshair';
  }

  /** For tests and the HUD: where a station is drawn on screen. */
  screenPosOf(id) {
    const s = this.drawnStations.find((d) => d.id === id);
    return s ? worldToScreen(this.vp, s.x, s.y) : null;
  }

  worldAt(sx, sy) {
    return screenToWorld(this.vp, sx, sy);
  }
}
