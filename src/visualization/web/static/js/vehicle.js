// Vehicle page: join, "my radio" (received view only), LiDAR on request for the own vehicle, own-perspective map.
// This page never subscribes to ground-truth topics: the server would refuse them (and a test checks the server's view).

import { createApp, renderTopbar, showFakeBanner, credentials } from './app.js';
import { MapView } from './map-view.js';
import { Inspector } from './inspector.js';
import { LidarView } from './lidar-view.js';
import { LidarConfigForm } from './lidar-controls.js';
import { buildOverlays } from './lidar-overlays.js';
import { SentCounter } from './panels-model.js';
import { toast } from './toasts.js';
import * as u from './units.js';

const app = createApp({ role: 'vehicle' });
const { store, hub, api } = app;
renderTopbar(document.getElementById('top'), { title: 'Vehicle', store });
showFakeBanner(api);
const $ = (id) => document.getElementById(id);

let session = credentials.session;            // {id, token, name, actor_id}
let config = {};
let schema = null;
let lidarForm = null;
let view = null;
let me = null;                                 // own telemetry as a station
let started = false;
let lostAt = null;
const sentCounter = new SentCounter(10);

// ── join ────────────────────────────────────────────────────────────────────────────────────────
async function loadBlueprints() {
  try {
    const r = await api.get('/client/blueprints', { quiet: true });
    for (const b of r.blueprints) $('j-bp').appendChild(Object.assign(document.createElement('option'), { value: b, textContent: b.replace('vehicle.', '') }));
    $('j-bp').value = r.default;
  } catch { /* the form still works with the default blueprint */ }
}

$('join-form').addEventListener('submit', async (e) => {
  e.preventDefault();
  $('join-btn').disabled = true;
  try {
    const body = { client_name: $('j-name').value, vehicle_blueprint: $('j-bp').value || undefined, enable_lidar: $('j-lidar').checked };
    if ($('j-dur').value) body.duration_seconds = Number($('j-dur').value);
    const r = await api.post('/client/join', body);
    session = { id: r.session_id, token: r.session_token, name: $('j-name').value, actor_id: r.actor_id };
    credentials.session = session;
    if (r.lidar_error) toast(`Joined, but no LiDAR: ${r.lidar_error.reason || r.lidar_error.message}`, 'error');
    await start();
  } catch { /* api.js already showed the error */ } finally {
    $('join-btn').disabled = false;
  }
});

$('leave').addEventListener('click', async () => {
  try { await api.delete(`/client/leave/${session.id}`, { quiet: true }); } catch { /* it may be gone already */ }
  endSession('You left. Your vehicle and its LiDAR were removed.');
});

function endSession(message) {
  hub.close();
  if (view) view.dispose();
  credentials.session = null;
  session = null;
  started = false;
  $('app-left').hidden = $('app-right').hidden = true;
  $('join-panel').hidden = false;
  toast(message, 'info', 12000);
}

// ── session running ─────────────────────────────────────────────────────────────────────────────
async function start() {
  if (started) return;
  started = true;
  $('join-panel').hidden = true;
  $('app-left').hidden = $('app-right').hidden = false;
  try { config = await api.get('/api/config', { quiet: true }); } catch { config = {}; }
  schema = await api.get('/api/lidar/schema');
  const rate = config.fixed_delta_seconds ? 1 / config.fixed_delta_seconds : 20;
  lidarForm = new LidarConfigForm($('lidar-form'), schema, { simRateHz: rate });

  const map = new MapView($('map'), { store, getConfig: () => config, getSelf: () => me, onSelect: () => {} });
  app.map = map;
  map.setSelected(session.actor_id);
  map.setMode('perspective', session.actor_id);
  map.setFollow($('follow').checked);
  $('follow').addEventListener('change', (e) => map.setFollow(e.target.checked));
  try { map.setGeometry(await api.get('/api/map/geometry')); } catch { /* the map still shows stations */ }

  const inspector = new Inspector($('radio'), { onSelectStation: () => {} });
  inspector.clear('Waiting for the first V2X update…');

  app.watch('session.self', { max_hz: 5 }, (data, env) => {
    const t = data.telemetry;
    me = { id: session.actor_id, kind: 'vehicle', role: 'client', name: session.name, x: t.x, y: t.y, z: t.z, yaw: t.yaw, speed_mps: t.speed_kmh / 3.6, v2x: true };
    drawMe(data, env);
    map.requestRender();
  });
  hub.subscribe('v2x.inbox', { max_hz: 2 }, (data, env) => {
    store.put(env);
    sentCounter.push(env.frame, env.sim_time, data.sent);
    map.onInbox(env);
    inspector.render({ station: me || { id: session.actor_id, kind: 'vehicle', name: session.name }, inbox: env, delivery: null,
      sent: sentCounter.summary(), lidar: ownSensor(), truthLabel: 'My telemetry (own ground truth)' });
    $('radio-stamp').textContent = u.stamp(env.frame, env.sim_time);
    map.requestRender();
  });
  app.watch('lidar.sensors', { max_hz: 10 }, (data) => { updateLidar(data); });

  store.onConnection(onConnection);
  hub.connect();
}

function ownSensor() {
  const e = store.get('lidar.sensors');
  return e ? e.data.sensors.find((s) => s.actor_id === session.actor_id) || null : null;
}

function drawMe(data, env) {
  const t = data.telemetry;
  const dl = $('me');
  dl.textContent = '';
  const row = (k, v, title) => { dl.append(Object.assign(document.createElement('dt'), { textContent: k }), Object.assign(document.createElement('dd'), { textContent: v, title: title || '' })); };
  row('name', `${session.name} (station #${session.actor_id})`);
  row('speed', u.speed(t.speed_kmh / 3.6, 1));
  row('position', `${u.num(t.x, 1)}, ${u.num(t.y, 1)} m`);
  row('yaw', u.degrees(t.yaw, 1));
  row('remaining', data.remaining_seconds === null ? 'until you leave' : u.age(data.remaining_seconds));
  row('data', u.stamp(env.frame, env.sim_time));
}

// ── connection loss and the grace period (decision D4) ──────────────────────────────────────────
function onConnection({ state, detail }) {
  const banner = $('conn-banner');
  if (!started) return;
  if (state === 'open') { lostAt = null; banner.hidden = true; return; }
  if (state === 'closed' && /authentication failed/.test(detail)) {
    endSession('Your session has ended on the server (left, expired or kicked). Join again to get a new vehicle.');
    return;
  }
  if (state === 'reconnecting' || state === 'connecting') {
    if (lostAt === null) lostAt = Date.now();
    tickBanner();
  }
}

function tickBanner() {
  const banner = $('conn-banner');
  if (lostAt === null) return;
  const grace = config.session_disconnect_grace_s ?? 30;
  const left = Math.max(0, grace - (Date.now() - lostAt) / 1000);
  banner.hidden = false;
  banner.textContent = left > 0
    ? `Connection lost. The server keeps your vehicle for another ${left.toFixed(0)} s while this page reconnects.`
    : 'Connection lost for longer than the grace period: the server may have removed your vehicle. Reconnecting…';
  if (lostAt !== null) setTimeout(tickBanner, 500);
}

// ── LiDAR on request ────────────────────────────────────────────────────────────────────────────
const BADGE_CLASS = { requested: 'warn', spawning: 'warn', warming_up: 'warn', active: 'ok', reconfiguring: 'warn', stopping: 'warn', error: 'bad' };

function updateLidar(data) {
  const s = ownSensor();
  const b = data.budget;
  $('lidar-budget').textContent = `budget: ${u.count(b.used_pps)} of ${u.count(b.max_pps)} points/s used, ${b.sensors} of ${b.max_sensors} sensors`;
  const badge = $('lidar-badge');
  badge.textContent = s ? s.state : 'off';
  badge.className = `badge state-${s ? s.state : 'off'}`;
  badge.dataset.state = s ? s.state : 'off';
  $('lidar-off').hidden = !!s;
  $('lidar-request').hidden = !!s;
  $('lidar-apply').hidden = !s;
  $('lidar-release').hidden = !s;
  if (s && !view && s.state !== 'stopping') openView(s);
  if (s && view) view.setSensorInfo(s);
  if (s && s.error) $('lidar-badge').title = s.error;
}

function openView(sensor) {
  const rate = config.fixed_delta_seconds ? 1 / config.fixed_delta_seconds : 20;
  lidarForm.setConfig(sensor.config);
  $('lidar-off').hidden = true;
  view = new LidarView($('lidar-view'), {
    sensorId: sensor.sensor_id, schema, simRateHz: rate, canShowTruth: false,
    getOverlays: (frame) => buildOverlays({ store, stationId: session.actor_id, lidarFrame: frame,
      lidarSimTime: view && view.latest ? view.latest.header.simTime : 0, world: null, includeTruth: false, ego: me, awareness: config.awareness_range_m }),
    onClose: () => { view = null; $('lidar-view').textContent = ''; },
  });
  view.setSensorInfo(sensor);
}

$('lidar-request').addEventListener('click', async () => {
  const r = lidarForm.read();
  if (r.errors) { toast(`Fix the sensor settings first: ${Object.entries(r.errors).map(([k, v]) => `${k} ${v}`).join('; ')}`, 'error'); return; }
  try {
    await api.post(`/api/sessions/${session.id}/lidar`, r.config);
  } catch { /* 409 budget etc.: api.js shows the server's explanation */ }
});

$('lidar-apply').addEventListener('click', async () => {
  const s = ownSensor();
  const r = lidarForm.read();
  if (!s) return;
  if (r.errors) { toast(`Fix the sensor settings first: ${Object.entries(r.errors).map(([k, v]) => `${k} ${v}`).join('; ')}`, 'error'); return; }
  try { await api.put(`/api/lidar/sensors/${s.sensor_id}`, r.config); } catch { /* shown */ }
});

$('lidar-release').addEventListener('click', async () => {
  const s = ownSensor();
  if (!s) return;
  if (view) view.dispose();
  try { await api.delete(`/api/lidar/sensors/${s.sensor_id}`); } catch { /* shown */ }
});

// ── start ───────────────────────────────────────────────────────────────────────────────────────
loadBlueprints();
if (session) {
  // a reload keeps the session: if the server forgot it, the hub refuses the token and we return to the join form
  start();
}
if (new URLSearchParams(location.search).has('debug')) {
  window.carlaDebug = { app, store, hub, get view() { return view; }, get session() { return session; }, get map() { return app.map; } };
}
