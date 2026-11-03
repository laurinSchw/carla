// 3D point-cloud viewer (Three.js, vendored). One preallocated BufferGeometry (no allocation per frame, F9);
// decoding and colouring happen in a Web Worker with latest-wins; overlays show what V2X reported next to what the
// LiDAR sees, using only V2X frames that are not newer than the point cloud.
//
// Scene origin = the sensor's location (see lidar-process.js); CARLA -> Three mirrors y; camera.up = +z.

import * as THREE from 'three';
import { OrbitControls } from '/static/vendor/three/OrbitControls.js';
import { LidarStream } from './lidar-stream.js';
import { RateMeter, frameGapSeconds, paletteLut, COLOR_MODES } from './lidar-process.js';
import * as u from './units.js';

const MAX_POINTS = 400_000;
const POINT_STEPS = [['10k', 10000], ['25k', 25000], ['50k', 50000], ['all', null]];

function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

export class LidarView {
  /**
   * @param {HTMLElement} container
   * @param {object} o
   * @param {number} o.sensorId
   * @param {object} o.schema        GET /api/lidar/schema
   * @param {() => object} o.getOverlays  ({ego, received, truth, rsus, vrus, denms, v2xFrame, v2xSimTime, awareness}) in CARLA world coords
   * @param {number} o.simRateHz
   * @param {(info: object) => void} [o.onState]
   * @param {() => void} [o.onClose]
   * @param {boolean} [o.canShowTruth]  ground-truth boxes (observer and admin only)
   * @param {number} [o.maxPoints]
   */
  constructor(container, { sensorId, schema, getOverlays = () => ({}), simRateHz = 20, onState = () => {}, onClose = () => {},
    canShowTruth = false, maxPoints = MAX_POINTS, highlightId = null } = {}) {
    this.container = container;
    this.sensorId = sensorId;
    this.schema = schema;
    this.getOverlays = getOverlays;
    this.simRateHz = simRateHz;
    this.onState = onState;
    this.onCloseCb = onClose;
    this.canShowTruth = canShowTruth;
    this.maxPoints = maxPoints;
    this.highlightId = highlightId;
    this.colorMode = 'semantic';
    this.egoFilter = true;
    this.frozen = false;
    this.camMode = 'orbit';
    this.overlayFlags = { ego: true, received: true, truth: false, infra: true, rings: true };
    this.state = 'connecting';
    this.profile = { max_points: 25000, max_hz: Math.min(10, simRateHz), compress: false };
    this.streamProfile = null;
    this.latest = null;               // last processed frame {header, counts, modes}
    this.lastRaw = null;              // raw bytes of the displayed frame (for export)
    this.rate = new RateMeter(2000);
    this.renderRate = new RateMeter(2000);
    this.dropped = 0;
    this.busy = false;
    this.pending = null;
    this.msgId = 0;
    this.lut = paletteLut((schema.semantic_tags || []).map((t) => ({ id: t.id, color: t.color })));
    this.disposed = false;
    this._build();
    this._startWorker();
    this._startStream();
    this._loop = this._loop.bind(this);
    this._raf = requestAnimationFrame(this._loop);
  }

  // ── DOM ───────────────────────────────────────────────────────────────────────────────────────
  _build() {
    const c = this.container;
    c.textContent = '';
    this.wrap = el('div', 'lidarwrap');
    this.canvas = el('canvas');
    this.canvas.tabIndex = 0;
    this.canvas.setAttribute('aria-label', 'LiDAR point cloud. Mouse drag orbits, wheel zooms; W A S D Q E fly in free-fly mode.');
    this.hud = el('div', 'hud');
    this.wrap.append(this.canvas, this.hud);
    this.controlsBar = el('div', 'row');
    c.append(this.wrap, this.controlsBar);

    const sel = (label, options, value, onChange, title) => {
      const l = el('label', '', `${label} `);
      const s = document.createElement('select');
      for (const [v, text] of options) { const o = el('option', '', text); o.value = String(v); s.appendChild(o); }
      s.value = String(value);
      s.addEventListener('change', () => onChange(s.value));
      if (title) l.title = title;
      l.appendChild(s);
      this.controlsBar.appendChild(l);
      return s;
    };
    const chk = (label, checked, onChange, title) => {
      const l = el('label');
      const i = document.createElement('input');
      i.type = 'checkbox';
      i.checked = checked;
      i.addEventListener('change', () => onChange(i.checked));
      l.append(i, ` ${label}`);
      if (title) l.title = title;
      this.controlsBar.appendChild(l);
      return i;
    };
    this.colorSelect = sel('Colour', COLOR_MODES.map((m) => [m, m]), this.colorMode, (v) => { this.colorMode = v; this._reprocess(); });
    chk('hide own vehicle', this.egoFilter, (v) => { this.egoFilter = v; this._reprocess(); }, 'Drop points whose object id is the ego vehicle (semantic sensors only)');
    sel('Camera', [['orbit', 'orbit'], ['follow', 'follow (heading-locked)'], ['top', 'top-down'], ['fly', 'free-fly']], this.camMode, (v) => this.setCamera(v));
    sel('Points', POINT_STEPS.map(([t, v]) => [v === null ? 'all' : v, t]), this.profile.max_points === null ? 'all' : this.profile.max_points,
      (v) => { this.profile.max_points = v === 'all' ? null : Number(v); this.stream.setProfile(this.profile); }, 'Display subsampling (random but fixed per sensor); recordings are never subsampled');
    sel('Rate', [[this.simRateHz, `${this.simRateHz} Hz`], [10, '10 Hz'], [5, '5 Hz'], [2, '2 Hz'], [1, '1 Hz']].filter(([v], i, a) => a.findIndex((x) => x[0] === v) === i),
      this.profile.max_hz, (v) => { this.profile.max_hz = Number(v); this.stream.setProfile(this.profile); });
    chk('compress', this.profile.compress, (v) => { this.profile.compress = v; this.stream.setProfile(this.profile); }, 'zlib on the wire');
    this.overlayBoxes = {};
    for (const [k, label, title] of [['ego', 'own box', 'Bounding box of the sensor vehicle'], ['received', 'V2X-received', 'Stations as the V2X messages describe them (not newer than this point cloud)'],
      ['truth', 'ground truth', 'Ground-truth boxes (observer and admin only)'], ['infra', 'RSU/VRU/DENM', ''], ['rings', 'rings', 'LiDAR range and awareness range']]) {
      if (k === 'truth' && !this.canShowTruth) continue;
      this.overlayBoxes[k] = chk(label, this.overlayFlags[k], (v) => { this.overlayFlags[k] = v; }, title);
    }
    this.freezeBtn = el('button', '', 'Freeze');
    this.freezeBtn.addEventListener('click', () => this.setFrozen(!this.frozen));
    this.exportBtn = el('button', '', 'Export frame');
    this.exportBtn.addEventListener('click', () => this.exportFrame());
    this.closeBtn = el('button', '', 'Close');
    this.closeBtn.addEventListener('click', () => this.dispose());
    this.controlsBar.append(this.freezeBtn, this.exportBtn, this.closeBtn);

    // Three.js
    this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setClearColor(0x0b0e12, 1);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(60, 1, 0.1, 2000);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-18, 0, 12);
    this.camera.lookAt(0, 0, 0);
    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.target.set(0, 0, 0);
    this.controls.enableDamping = false;
    this.controls.addEventListener('change', () => { this._dirty = true; });
    this._dirty = true;

    const g = new THREE.BufferGeometry();
    this.posAttr = new THREE.BufferAttribute(new Float32Array(this.maxPoints * 3), 3).setUsage(THREE.DynamicDrawUsage);
    this.colAttr = new THREE.BufferAttribute(new Float32Array(this.maxPoints * 3), 3).setUsage(THREE.DynamicDrawUsage);
    g.setAttribute('position', this.posAttr);
    g.setAttribute('color', this.colAttr);
    g.setDrawRange(0, 0);
    g.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 1000);
    this.geometry = g;
    this.points = new THREE.Points(g, new THREE.PointsMaterial({ size: 2.2, sizeAttenuation: false, vertexColors: true }));
    this.points.frustumCulled = false;
    this.scene.add(this.points);

    this.overlayGroup = new THREE.Group();
    this.scene.add(this.overlayGroup);
    this.boxGeo = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1));
    this.boxPool = [];
    this.ringPool = [];
    this.axes = new THREE.AxesHelper(2);
    this.scene.add(this.axes);
    this._resize();
    this._ro = new ResizeObserver(() => this._resize());
    this._ro.observe(this.wrap);
    this._keys = new Set();
    this._kd = (e) => { if (document.activeElement === this.canvas) this._keys.add(e.key.toLowerCase()); };
    this._ku = (e) => this._keys.delete(e.key.toLowerCase());
    window.addEventListener('keydown', this._kd);
    window.addEventListener('keyup', this._ku);
  }

  _resize() {
    const w = this.wrap.clientWidth || 600, h = this.wrap.clientHeight || 420;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this._dirty = true;
  }

  // ── worker and stream ─────────────────────────────────────────────────────────────────────────
  _startWorker() {
    this.worker = new Worker('/static/js/lidar-worker.js', { type: 'module' });
    this.worker.onmessage = (ev) => {
      const m = ev.data;
      this.busy = false;
      if (m.type === 'frame') this._applyFrame(m);
      else if (m.type === 'error') this._setState(`decode error: ${m.message}`);
      if (this.pending) { const p = this.pending; this.pending = null; this._submit(p); }
    };
    this.worker.onerror = (e) => this._setState(`worker error: ${e.message}`);
  }

  _startStream() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    this.stream = new LidarStream(`${proto}://${location.host}/ws/lidar/${this.sensorId}`, {
      onFrame: (buf) => this._onRawFrame(buf),
      onMessage: (m) => this._onMessage(m),
      onStatus: (s) => { if (s !== 'open') this._setState(s); },
      onClose: (info) => { this._setState(`ended (${info.reason})`); this.onState({ sensorId: this.sensorId, state: info.reason, ended: true }); },
    });
    this.stream.setProfile(this.profile);
    this.stream.connect();
  }

  _onMessage(m) {
    if (m.type === 'state') { this.sensorState = m; this._setState(m.state); this.onState({ sensorId: this.sensorId, state: m.state, frame: m.frame, error: m.error }); }
    else if (m.type === 'stream_profile') this.streamProfile = m;
    else if (m.type === 'error') this._setState(`error: ${m.message}`);
  }

  _setState(s) {
    this.state = s;
  }

  _onRawFrame(buffer) {
    this.rate.add(buffer.byteLength);
    const job = { buffer, id: ++this.msgId };
    if (this.busy) {
      if (this.pending) this.dropped += 1;                  // latest wins: the older waiting frame is dropped, and counted
      this.pending = job;
      return;
    }
    this._submit(job);
  }

  _opts() {
    return { colorMode: this.colorMode, egoFilter: this.egoFilter, highlightId: this.highlightId === null ? undefined : this.highlightId,
      maxPoints: this.maxPoints, lut: this.lut };
  }

  _submit(job) {
    this.busy = true;
    this.lastSubmitted = job.buffer.slice(0);               // raw bytes for "export frame"; the original is transferred
    this.worker.postMessage({ type: 'frame', id: job.id, buffer: job.buffer, opts: { ...this._opts(), lut: this.lut } }, [job.buffer]);
  }

  _reprocess() {
    if (this.lastRaw && !this.busy) {
      this.busy = true;
      this.lastSubmitted = this.lastRaw;
      const copy = this.lastRaw.slice(0);
      this.worker.postMessage({ type: 'frame', id: ++this.msgId, buffer: copy, opts: { ...this._opts(), lut: this.lut } }, [copy]);
    }
  }

  _applyFrame(m) {
    if (this.frozen) return;
    const n = m.counts.displayed;
    this.posAttr.array.set(m.positions, 0);
    this.colAttr.array.set(m.colors, 0);
    for (const a of [this.posAttr, this.colAttr]) {            // upload only the used part of the preallocated buffers
      a.clearUpdateRanges();
      a.addUpdateRange(0, n * 3);
      a.needsUpdate = true;
    }
    this.geometry.setDrawRange(0, n);
    this.latest = m;
    this._dirty = true;
    this.lastRaw = this.lastSubmitted;
    // colour modes this frame can honestly offer
    const opts = [...this.colorSelect.options];
    for (const o of opts) o.disabled = !m.modes.includes(o.value);
    if (!m.modes.includes(this.colorMode)) { this.colorMode = m.modes[0]; this.colorSelect.value = this.colorMode; }
    this.frameCount = (this.frameCount || 0) + 1;
    this.rendered = true;
  }

  // ── camera ────────────────────────────────────────────────────────────────────────────────────
  setCamera(mode) {
    this.camMode = mode;
    this._dirty = true;
    this.controls.enabled = mode === 'orbit';
    if (mode === 'top') {
      this.camera.up.set(0, 1, 0);                              // screen up = Three +y = CARLA -y, so +x is right and +y is down
      this.camera.position.set(0, 0, 90);
      this.camera.lookAt(0, 0, 0);
    } else {
      this.camera.up.set(0, 0, 1);
      if (mode === 'orbit') { this.camera.position.set(-18, 0, 12); this.controls.target.set(0, 0, 0); this.camera.lookAt(0, 0, 0); }
      if (mode === 'fly') this.camera.position.set(-10, 0, 6);
    }
  }

  _updateCamera(dt) {
    const h = this.latest && this.latest.header;
    if (this.camMode === 'follow' && h) {
      const yaw = (h.rotationPYR[1] * Math.PI) / 180;           // CARLA yaw; Three mirrors y so the heading angle is -yaw
      const a = -yaw;
      this.camera.position.set(Math.cos(a) * -16, Math.sin(a) * -16, 8);
      this.camera.up.set(0, 0, 1);
      this.camera.lookAt(Math.cos(a) * 6, Math.sin(a) * 6, 0);
    } else if (this.camMode === 'fly') {
      const k = this._keys, speed = 15 * dt;
      const dir = new THREE.Vector3();
      this.camera.getWorldDirection(dir);
      const right = new THREE.Vector3().crossVectors(dir, this.camera.up).normalize();
      if (k.has('w')) this.camera.position.addScaledVector(dir, speed);
      if (k.has('s')) this.camera.position.addScaledVector(dir, -speed);
      if (k.has('d')) this.camera.position.addScaledVector(right, speed);
      if (k.has('a')) this.camera.position.addScaledVector(right, -speed);
      if (k.has('e')) this.camera.position.z += speed;
      if (k.has('q')) this.camera.position.z -= speed;
      if (k.has('arrowleft')) this.camera.rotateOnWorldAxis(new THREE.Vector3(0, 0, 1), 1.2 * dt);
      if (k.has('arrowright')) this.camera.rotateOnWorldAxis(new THREE.Vector3(0, 0, 1), -1.2 * dt);
    } else if (this.camMode === 'orbit') {
      this.controls.update();
    }
  }

  // ── overlays ──────────────────────────────────────────────────────────────────────────────────
  _box(i, colour) {
    let b = this.boxPool[i];
    if (!b) {
      b = new THREE.LineSegments(this.boxGeo, new THREE.LineBasicMaterial({ color: colour }));
      this.boxPool[i] = b;
      this.overlayGroup.add(b);
    }
    b.material.color.set(colour);
    b.visible = true;
    return b;
  }

  _ring(i, radius, colour, dashed) {
    let r = this.ringPool[i];
    const key = `${radius}|${colour}|${dashed}`;
    if (!r || r.userData.key !== key) {
      if (r) { this.overlayGroup.remove(r); r.geometry.dispose(); }
      const pts = [];
      for (let k = 0; k <= 128; k++) { const a = (k / 128) * Math.PI * 2; pts.push(new THREE.Vector3(Math.cos(a) * radius, Math.sin(a) * radius, 0)); }
      const geo = new THREE.BufferGeometry().setFromPoints(pts);
      const mat = dashed ? new THREE.LineDashedMaterial({ color: colour, dashSize: 2, gapSize: 2 }) : new THREE.LineBasicMaterial({ color: colour });
      r = new THREE.Line(geo, mat);
      if (dashed) r.computeLineDistances();
      r.userData.key = key;
      this.ringPool[i] = r;
      this.overlayGroup.add(r);
    }
    r.visible = true;
    return r;
  }

  _updateOverlays() {
    for (const b of this.boxPool) if (b) b.visible = false;
    for (const r of this.ringPool) if (r) r.visible = false;
    const h = this.latest && this.latest.header;
    if (!h) return;
    const ov = this.getOverlays(h.frame) || {};
    this.overlaySnapshot = ov;
    const [sx, sy, sz] = h.location;
    const place = (b, x, y, z, yawDeg, l, w, hgt) => {
      b.position.set(x - sx, -(y - sy), z - sz + hgt / 2);
      b.rotation.set(0, 0, -(yawDeg * Math.PI) / 180);
      b.scale.set(l, w, hgt);
    };
    let bi = 0, ri = 0;
    const f = this.overlayFlags;
    if (f.ego && ov.ego) {
      place(this._box(bi++, 0x4da3ff), ov.ego.x, ov.ego.y, ov.ego.z || 0, ov.ego.yaw, ov.ego.length || 4.6, ov.ego.width || 1.9, ov.ego.height || 1.5);
    }
    if (f.received) {
      for (const e of ov.received || []) {
        const colour = e.ghost ? 0x777777 : e.age <= 0.6 ? 0x4cc26a : e.age <= 1.1 ? 0xe3b341 : 0x888888;
        place(this._box(bi++, colour), e.x, e.y, e.z || 0, e.heading, e.length || 4.5, e.width || 1.8, e.height || 1.5);
      }
    }
    if (f.truth && this.canShowTruth) {
      for (const e of ov.truth || []) place(this._box(bi++, 0xff6b6b), e.x, e.y, e.z || 0, e.yaw, e.kind === 'pedestrian' ? 0.6 : 4.5, e.kind === 'pedestrian' ? 0.6 : 1.8, e.kind === 'pedestrian' ? 1.8 : 1.5);
    }
    if (f.infra) {
      for (const r of ov.rsus || []) place(this._box(bi++, 0xf0883e), r.x, r.y, 0, 0, 0.8, 0.8, 5);
      for (const v of ov.vrus || []) place(this._box(bi++, 0xb69cff), v.x, v.y, 0, 0, 0.6, 0.6, 1.8);
      for (const d of ov.denms || []) {
        place(this._box(bi++, 0xff4d4d), d.x, d.y, 0, 0, 1.2, 1.2, 3);
        const ring = this._ring(ri++, d.radius, 0xff4d4d, true);
        ring.position.set(d.x - sx, -(d.y - sy), -sz + 0.05);
      }
    }
    if (f.rings) {
      const cfg = (this.sensorInfo && this.sensorInfo.config) || {};
      if (cfg.range_m) this._ring(ri++, cfg.range_m, 0x4da3ff, false).position.set(0, 0, 0);
      if (ov.awareness) this._ring(ri++, ov.awareness, 0xb69cff, true).position.set(0, 0, 0);
    }
  }

  // ── HUD, loop ─────────────────────────────────────────────────────────────────────────────────
  setSensorInfo(info) {
    this.sensorInfo = info;
  }

  setFrozen(on) {
    this.frozen = on;
    this.freezeBtn.textContent = on ? 'Resume' : 'Freeze';
    if (!on) this._reprocess();
  }

  _hudText() {
    const h = this.latest && this.latest.header;
    const c = this.latest && this.latest.counts;
    const r = this.rate.rates();
    const gap = h && this.overlaySnapshot ? frameGapSeconds(h.frame, this.overlaySnapshot.v2xFrame, 1 / this.simRateHz) : null;
    const sp = this.streamProfile;
    const lines = [
      `sensor #${this.sensorId}  state: ${this.state}${this.frozen ? '  FROZEN (stream keeps running)' : ''}`,
      h ? `LiDAR frame F_l=${h.frame}  t=${u.num(h.simTime, 2)} s` : 'waiting for the first frame…',
      h ? `V2X frame F_v=${this.overlaySnapshot && this.overlaySnapshot.v2xFrame != null ? this.overlaySnapshot.v2xFrame : u.DASH}  gap ${gap === null ? u.DASH : `${gap.toFixed(2)} s`}` : '',
      c ? `points ${u.count(c.received)} received · ${u.count(c.displayed)} displayed · ${u.count(c.filteredEgo)} own-vehicle filtered${c.truncated ? ` · ${u.count(c.truncated)} over buffer` : ''}` : '',
      sp ? `stream: max ${sp.max_points === null ? 'all' : u.count(sp.max_points)} points, ${u.num(sp.max_hz, 1)} Hz${sp.compress ? ', zlib' : ''}${sp.subsampled ? ` — SUBSAMPLED (${u.count(sp.points_sent)} of ${u.count(sp.points_total)})` : ''}` : '',
      `received ${r.fps.toFixed(1)} fps (requested ${this.profile.max_hz}) · ${u.bytes(r.bytesPerSecond)}/s · dropped ${this.dropped} · render ${this.renderRate.rates().fps.toFixed(0)} fps`,
    ];
    return lines.filter(Boolean).join('\n');
  }

  _loop(t) {
    if (this.disposed) return;
    this._raf = requestAnimationFrame(this._loop);
    const dt = this._last ? Math.min((t - this._last) / 1000, 0.1) : 0.016;
    this._last = t;
    this._updateCamera(dt);
    if (!this.frozen) {
      this._updateOverlays();
      const ov = this.overlaySnapshot || {};
      const sig = `${ov.v2xFrame}|${ov.ego && ov.ego.x}|${ov.ego && ov.ego.y}|${(ov.truth || []).length}|${(ov.truth || [])[0] && ov.truth[0].x}|${Object.values(this.overlayFlags).join()}`;
      if (sig !== this._sig) { this._sig = sig; this._dirty = true; }
    }
    if (this._dirty || this.camMode === 'follow' || this.camMode === 'fly') {       // render only when something changed
      this._dirty = false;
      this.renderer.render(this.scene, this.camera);
      this.renderRate.add(0);
    }
    this.hud.textContent = this._hudText();
    const d = this.canvas.dataset;
    d.state = this.state;
    d.frame = this.latest ? String(this.latest.header.frame) : '';
    d.displayed = this.latest ? String(this.latest.counts.displayed) : '0';
    d.frames = String(this.frameCount || 0);
    d.frozen = String(this.frozen);
  }

  /** Raw v3 bytes as received plus the V2X frame used for the overlays (labelled with its stream profile). */
  exportFrame() {
    if (!this.lastRaw || !this.latest) return null;
    const F = this.latest.header.frame;
    const dl = (blob, name) => {
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = name;
      document.body.appendChild(a);
      a.click();
      a.remove();
      setTimeout(() => URL.revokeObjectURL(a.href), 1000);
    };
    dl(new Blob([this.lastRaw], { type: 'application/octet-stream' }), `lidar_sensor${this.sensorId}_frame_${F}.v3.bin`);
    const meta = { lidar_frame: F, v2x_frame_used_for_overlays: this.overlaySnapshot ? this.overlaySnapshot.v2xFrame : null,
      stream_profile: this.streamProfile, note: 'Subsampled for display if stream_profile.subsampled; full resolution comes only from recordings.',
      overlays: this.overlaySnapshot || null };
    dl(new Blob([JSON.stringify(meta, null, 2)], { type: 'application/json' }), `frame_${F}_v2x.json`);
    return { frame: F, bytes: this.lastRaw.byteLength };
  }

  dispose() {
    if (this.disposed) return;
    this.disposed = true;
    cancelAnimationFrame(this._raf);
    this.stream.close();
    this.worker.terminate();
    this._ro.disconnect();
    window.removeEventListener('keydown', this._kd);
    window.removeEventListener('keyup', this._ku);
    this.geometry.dispose();
    this.renderer.dispose();
    this.container.textContent = '';
    this.onCloseCb();
  }
}
