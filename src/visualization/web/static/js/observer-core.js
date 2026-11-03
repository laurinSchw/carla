// Observer page: the main V2X research view. Everything shown is data the server published, labelled with its frame.

import { MapView, DEFAULT_LAYERS } from './map-view.js';
import { LINK_STYLES } from './map-model.js';
import { Inspector, renderLinkDetails } from './inspector.js';
import { LinkLog } from './link-log.js';
import { PrrChart, MarginChart, LineChart } from './charts.js';
import { DeliveryTracker, SentCounter, TimeSeries } from './panels-model.js';
import { LidarView } from './lidar-view.js';
import { buildOverlays } from './lidar-overlays.js';
import * as u from './units.js';

/**
 * The observer panels (map, inspector, link log, channel analytics, health, LiDAR views) on the page that has their markup.
 * @param {object} app  createApp(...) result
 * @param {{role?: 'observer'|'admin'}} [opts] an admin may also create sensors on any vehicle
 */
export function initObserver(app, { role = 'observer' } = {}) {
  const { store, hub, api } = app;

  const $ = (id) => document.getElementById(id);
  let config = {};
  let selected = null;                         // station id
  const delivery = new DeliveryTracker(10);
  const sentCounter = new SentCounter(10);
  const nbSeries = new TimeSeries(60);
  const rateSeries = new TimeSeries(60);
  let lastV2xTime = null;

  // ── panels ──────────────────────────────────────────────────────────────────────────────────────
  const inspector = new Inspector($('inspector'), { onSelectStation: (id) => select(id) });
  inspector.clear();
  renderLinkDetails($('link-details'), null);
  const linkLog = new LinkLog($('log'), { onSelectLink: (l) => showLink(l) });
  const prrChart = new PrrChart($('chart-prr'));
  const marginChart = new MarginChart($('chart-margin'));
  const nbChart = new LineChart($('chart-nb'), { label: 'received', unit: 'messages' });
  const rateChart = new LineChart($('chart-rate'), { label: 'V2X update rate', unit: 'Hz', color: '--ok' });

  const map = new MapView($('map'), {
    store,
    getConfig: () => config,
    onSelect: (id) => select(id),
    onSelectLink: (l) => showLink(l),
  });

  function showLink(link) {
    map.selectedLink = link;
    renderLinkDetails($('link-details'), link);
    map.requestRender();
  }

  // layers
  const LAYER_LABELS = { roads: 'roads', stations: 'stations', labels: 'labels', trails: 'trails', awareness: 'awareness ring',
    links: 'links', denm: 'DENMs', signals: 'signals', lidar: 'LiDAR' };
  for (const [k, label] of Object.entries(LAYER_LABELS)) {
    const l = document.createElement('label');
    const cb = document.createElement('input');
    cb.type = 'checkbox';
    cb.checked = DEFAULT_LAYERS[k];
    cb.addEventListener('change', () => map.setLayers({ [k]: cb.checked }));
    l.append(cb, ` ${label}`);
    $('layers').appendChild(l);
  }
  $('fit').addEventListener('click', () => map.fit());
  $('follow').addEventListener('change', (e) => map.setFollow(e.target.checked));
  $('mode').addEventListener('change', (e) => {
    map.setMode(e.target.value, selected);
    subscribeSelected();
  });

  // legend: outcome = colour + dash pattern
  (function legend() {
    const el = $('legend');
    for (const [k, s] of Object.entries(LINK_STYLES)) {
      const span = document.createElement('span');
      span.innerHTML = `<svg width="34" height="8" aria-hidden="true"><line x1="0" y1="4" x2="34" y2="4" stroke="var(${s.token})" stroke-width="2" stroke-dasharray="${s.dash.join(' ')}"/></svg> ${s.label}`;
      span.dataset.outcome = k;
      el.appendChild(span);
    }
    el.insertAdjacentHTML('beforeend',
      '<span>▶ equipped vehicle</span><span>▷ vehicle without V2X</span><span>■ RSU</span><span>● pedestrian</span><span>▲ DENM</span><span>G/Y/R signal state</span>');
  })();

  // ── selection and subscriptions ─────────────────────────────────────────────────────────────────
  function logFilter() {
    const f = {};
    const ids = $('f-ids').value.split(',').map((s) => s.trim()).filter(Boolean).map(Number).filter(Number.isInteger);
    if (ids.length) f.station_ids = ids;
    else if (selected !== null) f.station_ids = [selected];
    if (f.station_ids) f.role = $('f-role').value;
    if ($('f-outcome').value !== 'all' || f.station_ids) f.outcome = $('f-outcome').value;
    if ($('f-msg').value) f.msg_types = [$('f-msg').value];
    if ($('f-link').value) f.link_types = [$('f-link').value];
    return Object.keys(f).some((k) => ['station_ids', 'msg_types', 'link_types', 'outcome'].includes(k)) ? f : null;
  }

  function subscribeSelected() {
    if (selected === null) {
      hub.unsubscribe('v2x.inbox');
    } else {
      hub.subscribe('v2x.inbox', { max_hz: 2, filter: { station_id: selected } }, (data, env) => {
        store.put(env);
        if (data.station_id !== selected) return;
        sentCounter.push(env.frame, env.sim_time, data.sent);
        map.onInbox(env);
        renderInspector();
        map.requestRender();
      });
    }
    const f = logFilter();
    linkLog.setFilter(f ? JSON.stringify(f) : '(none)');
    $('log-filter-text').textContent = f ? `filter ${JSON.stringify(f)}` : 'select a station or set a filter';
    if (!f) {
      hub.unsubscribe('v2x.links');
      store.topics.delete('v2x.links');
    } else {
      hub.subscribe('v2x.links', { max_hz: 2, filter: f }, (data, env) => {
        store.put(env);
        linkLog.onData(env);
        if (selected !== null) {
          const rx = data.columns ? data.rows.filter((r) => r[data.columns.indexOf('rx_id')] === selected) : [];
          delivery.push(env.frame, rx.map((r) => ({ delivered: r[data.columns.indexOf('delivered')], drop_cause: r[data.columns.indexOf('drop_cause')] })));
        }
        map.requestRender();
      });
    }
  }

  function select(id) {
    if (id === selected) return;
    selected = id;
    delivery.reset();
    sentCounter.buf = [];
    nbSeries.t = []; nbSeries.v = [];
    store.topics.delete('v2x.inbox');
    store.topics.delete('v2x.links');
    linkLog.buf.clear();
    map.setSelected(id);
    const perspective = $('mode').querySelector('option[value="perspective"]');
    perspective.disabled = id === null || id < 0;
    if (perspective.disabled && $('mode').value === 'perspective') { $('mode').value = 'truth'; map.setMode('truth'); }
    $('nb-h').textContent = id === null ? 'Messages received per update (selected station)' : `Messages received per update (station #${id})`;
    subscribeSelected();
    renderInspector();
  }

  for (const id of ['f-ids', 'f-role', 'f-outcome', 'f-msg', 'f-link']) $(id).addEventListener('change', () => { linkLog.buf.clear(); subscribeSelected(); });
  $('log-filter').addEventListener('submit', (e) => e.preventDefault());

  function renderInspector() {
    if (selected === null) { inspector.clear(); $('insp-stamp').textContent = ''; return; }
    const world = store.get('world.state');
    let station = world && world.data.stations.find((s) => s.id === selected);
    if (!station && world) station = (world.data.rsus || []).map((r) => ({ id: r.station_id, kind: 'RSU', x: r.x, y: r.y, z: r.z, name: `RSU ${r.intersection_id}` })).find((s) => s.id === selected);
    const inbox = store.get('v2x.inbox');
    const sensors = store.get('lidar.sensors');
    const lidar = sensors && station ? sensors.data.sensors.find((s) => s.actor_id === station.id) : null;
    inspector.render({ station: station || { id: selected, kind: 'unknown' }, inbox: inbox && inbox.data.station_id === selected ? inbox : null,
      delivery: delivery.summary(), sent: sentCounter.summary(), lidar });
    $('insp-stamp').textContent = inbox ? u.stamp(inbox.frame, inbox.sim_time) : '';
    if (station && station.kind === 'vehicle') $('inspector').prepend(lidarButton(station));
  }

  // ── LiDAR views ─────────────────────────────────────────────────────────────────────────────────
  const MAX_LIDAR_VIEWS = 2;                  // an observer may open at most this many streams at once (ui.max_lidar_views)
  const views = new Map();                    // sensor id -> LidarView
  let lidarSchema = null;

  async function openLidar(sensor) {
    if (views.has(sensor.sensor_id)) return;
    if (views.size >= MAX_LIDAR_VIEWS) { toastLimit(); return; }
    if (!lidarSchema) lidarSchema = await api.get('/api/lidar/schema');
    select(sensor.actor_id);                  // the overlay needs the inbox of the sensor's vehicle
    const panel = $('lidar-panel');
    panel.hidden = false;
    const box = document.createElement('div');
    box.className = 'lidarview';
    box.dataset.sensor = String(sensor.sensor_id);
    $('lidar').appendChild(box);
    const sim = store.get('run.status');
    const rate = sim ? 1 / sim.data.fixed_delta_seconds : config.fixed_delta_seconds ? 1 / config.fixed_delta_seconds : 20;
    const view = new LidarView(box, {
      sensorId: sensor.sensor_id, schema: lidarSchema, simRateHz: rate, canShowTruth: true,
      getOverlays: (frame) => {
        const s = (store.get('lidar.sensors') || { data: { sensors: [] } }).data.sensors.find((x) => x.sensor_id === sensor.sensor_id) || sensor;
        view.setSensorInfo(s);
        return buildOverlays({ store, stationId: sensor.actor_id, lidarFrame: frame, lidarSimTime: view.latest ? view.latest.header.simTime : 0,
          world: store.get('world.state'), awareness: config.awareness_range_m, includeTruth: true });
      },
      onClose: () => { views.delete(sensor.sensor_id); box.remove(); if (!views.size) panel.hidden = true; },
    });
    view.setSensorInfo(sensor);
    views.set(sensor.sensor_id, view);
  }

  function toastLimit() {
    import('./toasts.js').then((m) => m.toast(`At most ${MAX_LIDAR_VIEWS} LiDAR views can be open at once; close one first.`, 'info'));
  }

  function lidarButton(station) {
    const sensors = (store.get('lidar.sensors') || { data: { sensors: [] } }).data.sensors;
    const mine = station ? sensors.find((s) => s.actor_id === station.id) : null;
    const b = document.createElement('button');
    b.id = 'show-lidar';
    if (mine) {
      b.textContent = `Show LiDAR (sensor #${mine.sensor_id}, ${mine.state})`;
      b.disabled = mine.state === 'stopping' || mine.state === 'error';
      b.addEventListener('click', () => openLidar(mine));
    } else if (role === 'admin') {
      b.textContent = 'Create LiDAR and show';
      b.addEventListener('click', async () => {
        b.disabled = true;
        try {
          const r = await api.post('/api/lidar/sensors', { actor_id: station.id });
          const t0 = Date.now();
          const wait = () => {
            const s = ((store.get('lidar.sensors') || { data: { sensors: [] } }).data.sensors).find((x) => x.sensor_id === r.sensor_id);
            if (s) openLidar(s);
            else if (Date.now() - t0 < 5000) setTimeout(wait, 100);
          };
          wait();
        } catch { b.disabled = false; }
      });
    } else {
      b.textContent = 'Show LiDAR';
      b.disabled = true;
      b.title = 'This vehicle has no LiDAR sensor. Observers cannot create sensors: the vehicle\'s own session or an admin can.';
    }
    return b;
  }

  // ── data ────────────────────────────────────────────────────────────────────────────────────────
  app.watch('world.state', { max_hz: 10 }, (data, env) => {
    map.onWorldState(env);
    map.requestRender();
    if (selected !== null && !(env.frame % 10)) renderInspector();
  });
  app.watch('v2x.frame', { max_hz: 2 }, (data, env) => {
    if (lastV2xTime !== null && env.sim_time > lastV2xTime) rateSeries.push(env.sim_time, 1 / (env.sim_time - lastV2xTime));
    lastV2xTime = env.sim_time;
    if (selected !== null) {
      const me = data.stations.find((s) => s.station_id === selected);
      if (me) nbSeries.push(env.sim_time, me.received);
      nbChart.update(nbSeries.data());
    }
    const target = config.v2x_update_rate_hz;
    const have = rateSeries.v.length ? rateSeries.v[rateSeries.v.length - 1] : null;
    $('rate-text').textContent = `achieved ${u.hertz(have, 2)} per simulated second, target ${u.hertz(target, 2)}  ·  ${u.stamp(env.frame, env.sim_time)}`;
    rateChart.update(rateSeries.data());
    map.requestRender();
  });
  app.watch('v2x.stats', { max_hz: 1 }, (data) => drawStats(data));
  app.watch('run.status', { max_hz: 1 }, (data, env) => {
    const dl = $('health');
    dl.textContent = '';
    const row = (k, v, title) => {
      const dt = Object.assign(document.createElement('dt'), { textContent: k });
      const dd = Object.assign(document.createElement('dd'), { textContent: v });
      if (title) dd.title = title;
      dl.append(dt, dd);
    };
    row('real-time factor', u.num(data.real_time_factor, 3), '1.0 = simulated time runs as fast as wall-clock time');
    row('tick duration', `p50 ${u.num(data.tick_duration_ms.p50, 1)} / p95 ${u.num(data.tick_duration_ms.p95, 1)} / max ${u.num(data.tick_duration_ms.max, 1)} ms`,
      `budget ${u.num(data.fixed_delta_seconds * 1000, 0)} ms per step`);
    row('V2X update', `p50 ${u.num(data.v2x_update_ms.p50, 1)} / p95 ${u.num(data.v2x_update_ms.p95, 1)} ms`);
    row('population', `${data.background_vehicles} background vehicles, ${data.pedestrians} pedestrians, ${data.sessions} sessions`);
    $('health-stamp').textContent = u.stamp(env.frame, env.sim_time);
  });
  app.watch('lidar.sensors', { max_hz: 2 }, (data) => {
    drawSensors(data);
    map.requestRender();
    if (selected !== null) renderInspector();
  });

  function drawStats(data) {
    const block = $('stats-window').value;
    prrChart.update(data, block);
    marginChart.update(data, block);
  }
  $('stats-window').addEventListener('change', () => { const e = store.get('v2x.stats'); if (e) drawStats(e.data); });

  function drawSensors(data) {
    const box = $('sensors');
    box.textContent = '';
    const b = data.budget;
    const p = document.createElement('p');
    p.textContent = `budget: ${u.count(b.used_pps)} of ${u.count(b.max_pps)} points/s, ${b.sensors} of ${b.max_sensors} sensors`;
    box.appendChild(p);
    if (!data.sensors.length) { box.appendChild(Object.assign(document.createElement('p'), { className: 'empty', textContent: 'No LiDAR sensor is running.' })); return; }
    const t = document.createElement('table');
    t.innerHTML = '<thead><tr><th>sensor</th><th>vehicle</th><th>owner</th><th>state</th><th class="num">viewers</th><th class="num">points/frame</th></tr></thead>';
    const tb = document.createElement('tbody');
    for (const s of data.sensors) {
      const tr = document.createElement('tr');
      tr.className = 'selectable';
      tr.innerHTML = `<td>#${s.sensor_id}</td><td>#${s.actor_id}</td><td></td><td class="state-${s.state}">${s.state}</td><td class="num">${s.viewers}</td><td class="num">${u.count(s.points_per_frame)}</td>`;
      tr.children[2].textContent = s.owner;
      tr.addEventListener('click', () => select(s.actor_id));
      tb.appendChild(tr);
    }
    t.appendChild(tb);
    box.appendChild(t);
  }

  // ── start ───────────────────────────────────────────────────────────────────────────────────────
  subscribeSelected();                          // nothing selected yet: sets the "select a station" hint
  (async () => {
    try {
      config = await api.get('/api/config', { quiet: true });
    } catch { config = {}; }
    try {
      map.setGeometry(await api.get('/api/map/geometry'));
    } catch { /* the toast shows the reason; the map still shows stations */ }
    hub.connect();
  })();

  return { app, map, select, views, openLidar, inspector, linkLog, getConfig: () => config, get selected() { return selected; }, renderInspector, subscribeSelected };
}
