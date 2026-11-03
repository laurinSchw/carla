// Admin page: everything the observer sees plus connection, population, configuration, RSUs, DENM injection,
// sessions, LiDAR management and WebSocket health. The token lives in this tab (sessionStorage) and goes in a header.

import { createApp, renderTopbar, showFakeBanner, credentials } from './app.js';
import { initObserver } from './observer-core.js';
import { LidarConfigForm } from './lidar-controls.js';
import { fieldsFromJsonSchema, renderForm, readForm, validateAll, valuesToConfig, configToValues } from './schema-form.js';
import { DENM_CAUSES } from './denm-causes.js';
import { toast } from './toasts.js';
import * as u from './units.js';

const app = createApp({ role: 'admin' });
const { store, api } = app;
renderTopbar(document.getElementById('top'), { title: 'Admin', store });
showFakeBanner(api);
const $ = (id) => document.getElementById(id);

function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

/** Re-render a table body only when its content changed (keeps focus and half-typed values in inputs). */
function setRows(tbody, rows, build) {
  const sig = JSON.stringify(rows);
  if (tbody.dataset.sig === sig) return;
  tbody.dataset.sig = sig;
  tbody.textContent = '';
  for (const r of rows) tbody.appendChild(build(r));
}

function td(content, cls) {
  const c = el('td', cls);
  if (content instanceof Node) c.appendChild(content); else c.textContent = content;
  return c;
}

function button(text, onClick, cls = '') {
  const b = el('button', cls, text);
  b.addEventListener('click', onClick);
  return b;
}

// ── login ───────────────────────────────────────────────────────────────────────────────────────
async function tryToken(token) {
  credentials.adminToken = token;
  try {
    await api.get('/admin/status', { quiet: true });
    return true;
  } catch (e) {
    credentials.adminToken = '';
    return e.status === 0 ? null : false;
  }
}

$('login-form').addEventListener('submit', async (e) => {
  e.preventDefault();
  const ok = await tryToken($('token').value);
  if (ok) { showAdmin(); return; }
  toast(ok === null ? 'Server unreachable.' : 'That token was refused.', 'error');
});

(async () => {
  if (credentials.adminToken && await tryToken(credentials.adminToken)) showAdmin();
})();

let page = null;
let config = {};

async function showAdmin() {
  $('login').hidden = true;
  $('admin').hidden = false;
  page = initObserver(app, { role: 'admin' });
  if (new URLSearchParams(location.search).has('debug')) window.carlaDebug = { ...page, app, get config() { return config; } };
  app.hub.connect();
  await Promise.all([connectionCard(), runCard(), populationCard(), configCard(), rsuCard(), denmCard(), sessionsCard(), lidarCard(), hubCard()]);
}

// ── connection ──────────────────────────────────────────────────────────────────────────────────
let lastConnectState = null;

async function connectionCard() {
  async function refresh() {
    try {
      const s = await api.get('/admin/status', { quiet: true });
      $('conn-state').textContent = s.connected ? `${s.connect_state}${s.running ? ' (running)' : ''}` : s.connect_state;
      $('conn-state').className = `badge state-${s.connect_state === 'running' ? 'active' : s.connect_state === 'error' ? 'error' : 'requested'}`;
      $('conn-error').textContent = s.connect_error || '';
      if (lastConnectState === 'connecting' && s.connect_state === 'running') location.reload();    // new world: new hub
      lastConnectState = s.connect_state;
    } catch { /* the toast of a failed poll would be noise */ }
  }
  $('connect-form').addEventListener('submit', async (e) => {
    e.preventDefault();
    const body = { carla_host: $('cn-host').value, carla_port: Number($('cn-port').value), tm_port: Number($('cn-tm').value),
      v2x_enabled: $('cn-v2x').checked, v2x_update_rate_hz: Number($('cn-rate').value) };
    if ($('cn-rsu').value !== '') body.rsu_max_count = Number($('cn-rsu').value);
    try { await api.post('/admin/connect', body); toast('Connecting… the page reloads when the world is up.', 'info'); lastConnectState = 'connecting'; } catch { /* shown */ }
    refresh();
  });
  $('disconnect-btn').addEventListener('click', async () => {
    if (!confirm('Stop the world? Sessions, sensors and everything the server spawned are removed.')) return;
    try { await api.post('/admin/stop'); toast('Shutdown started.', 'info'); } catch { /* shown */ }
    setTimeout(refresh, 1000);
  });
  await refresh();
  setInterval(refresh, 2000);
}

// ── run and recording ───────────────────────────────────────────────────────────────────────────
const RECORD_KINDS = [['links', 'every link (v2x_links.jsonl)', true], ['stations', 'station ground truth', true], ['messages', 'sent messages', true],
  ['lidar_meta', 'LiDAR frame metadata', true], ['raw_lidar', 'raw LiDAR frames (large)', false]];

async function runCard() {
  const box = $('run-record');
  for (const [k, label, on] of RECORD_KINDS) {
    const l = el('label');
    l.style.display = 'block';
    const cb = el('input');
    cb.type = 'checkbox'; cb.checked = on; cb.value = k;
    l.append(cb, ` ${label}`);
    box.appendChild(l);
  }
  $('run-start').addEventListener('click', async () => {
    const record = [...box.querySelectorAll('input:checked')].map((i) => i.value);
    try {
      const r = await api.post('/api/runs', { name: $('run-name').value, record });
      toast(`run ${r.run_id} started at frame ${r.start_frame}`, 'ok', 5000);
      refresh();
    } catch { /* 409 when a run is active: shown */ }
  });
  $('run-stop').addEventListener('click', async () => {
    try { const r = await api.post('/api/runs/current/stop'); toast(`run ${r.run_id}: ${r.status}`, r.status === 'finished' ? 'ok' : 'error', 6000); refresh(); } catch { /* shown */ }
  });
  async function refresh() {
    let r;
    try { r = await api.get('/api/runs', { quiet: true }); } catch { return; }
    const a = r.active;
    $('run-badge').textContent = a ? 'recording' : 'no run';
    $('run-badge').className = `badge ${a ? 'state-active' : ''}`;
    $('run-start').disabled = !!a;
    $('run-stop').disabled = !a;
    const dl = $('run-status');
    dl.textContent = '';
    if (a) {
      const row = (k, v, title) => dl.append(Object.assign(el('dt'), { textContent: k }), Object.assign(el('dd'), { textContent: v, title: title || '' }));
      row('run', a.run_id);
      row('V2X updates written', u.count(a.v2x_updates));
      row('links written', u.count(a.links_written));
      row('LiDAR frames', u.count(a.lidar_frames));
      row('written', u.bytes(a.bytes_written));
      row('writer queue', `${u.percent(a.queue_fill, 0)} of ${a.queue_max}`, 'if it ever fills, the run FAILS instead of dropping data');
      row('disk free', u.bytes(a.disk_free_bytes));
      if (a.error) row('error', a.error);
    }
    setRows($('run-table').tBodies[0], r.runs.slice(0, 15), (run) => {
      const tr = el('tr');
      tr.append(td(run.run_id), td(run.status || u.DASH), td(String(run.frames ?? u.DASH), 'num'),
        td(button('Manifest', async () => {
          try {
            const m = await api.get(`/api/runs/${run.run_id}/manifest`);
            const pre = $('run-manifest');
            pre.hidden = false;
            pre.textContent = JSON.stringify(m, null, 2);
          } catch { /* shown */ }
        })));
      return tr;
    });
  }
  store.on('run.status', refresh);
  await refresh();
}

// ── population ──────────────────────────────────────────────────────────────────────────────────
async function populationCard() {
  const act = (fn, ok) => async () => {
    try { const r = await fn(); toast(ok(r), 'ok', 4000); } catch { /* shown */ }
  };
  $('pop-veh-add').addEventListener('click', act(() => api.post('/admin/traffic', { count: Number($('pop-veh').value) }), (r) => `spawned ${r.spawned ?? r.count ?? ''} vehicles`));
  $('pop-veh-clear').addEventListener('click', act(() => api.delete('/admin/traffic'), (r) => `removed ${r.removed} vehicles`));
  $('pop-ped-add').addEventListener('click', act(() => api.post('/admin/pedestrians', { count: Number($('pop-ped').value) }), (r) => `spawned ${r.spawned ?? r.count ?? ''} pedestrians`));
  $('pop-ped-clear').addEventListener('click', act(() => api.delete('/admin/pedestrians'), (r) => `removed ${r.removed} pedestrians`));
  $('pop-clear-all').addEventListener('click', async () => {
    try { await api.delete('/admin/traffic'); await api.delete('/admin/pedestrians'); toast('cleared', 'ok', 3000); } catch { /* shown */ }
  });
  store.on('run.status', (env) => {
    const d = env.data;
    $('pop-counts').textContent = `${d.background_vehicles} vehicles, ${d.pedestrians} pedestrians, ${d.sessions} sessions · ${u.stamp(env.frame, env.sim_time)}`;
  });
}

// ── configuration (hot fields only are applied) ─────────────────────────────────────────────────
let cfgFields = [];

function nullableHot(f) {
  return f.apply === 'hot' && f.nullable;
}

function diffHot(raw) {
  const typed = valuesToConfig(cfgFields, raw);
  const changes = {};
  for (const f of cfgFields) {
    if (f.apply !== 'hot') continue;
    let v = typed[f.name];
    if (v === undefined && nullableHot(f)) v = null;                  // an emptied nullable field means "no value"
    if (v === undefined) continue;
    if (v !== config[f.name]) changes[f.name] = v;
  }
  return changes;
}

async function configCard() {
  const schema = await api.get('/api/config/schema');
  config = await api.get('/api/config');
  cfgFields = fieldsFromJsonSchema(schema).filter((f) => !['host', 'port', 'web_port'].includes(f.name));
  function draw() {
    const badges = Object.fromEntries(cfgFields.map((f) => [f.name, f.apply]));
    renderForm($('cfg-form'), cfgFields, configToValues(cfgFields, config), { badges, idPrefix: 'cfg' });
    for (const f of cfgFields) {
      const input = $('cfg-form').querySelector(`[name="${CSS.escape(f.name)}"]`);
      if (input && f.apply !== 'hot') { input.disabled = true; input.title = 'needs a new world: change it in Connection and reconnect'; }
      else if (input && f.effect) input.title = `takes effect: ${f.effect}`;
    }
    $('cfg-stamp').textContent = `${cfgFields.filter((f) => f.apply === 'hot').length} hot, ${cfgFields.filter((f) => f.apply !== 'hot').length} need a restart`;
  }
  draw();
  $('cfg-reset').addEventListener('click', draw);
  $('cfg-apply').addEventListener('click', async () => {
    const raw = readForm($('cfg-form'), cfgFields);
    const errors = validateAll(cfgFields.filter((f) => f.apply === 'hot'), raw);
    if (Object.keys(errors).length) { toast(`Fix first: ${Object.entries(errors).map(([k, v]) => `${k} ${v}`).join('; ')}`, 'error'); return; }
    const changes = diffHot(raw);
    if (!Object.keys(changes).length) { $('cfg-result').textContent = 'nothing changed'; return; }
    try {
      const r = await api.request('PATCH', '/api/config', changes);
      config = await api.get('/api/config');
      $('cfg-result').textContent = `applied at frame ${r.applied_at_frame} (t=${u.num(r.sim_time, 2)} s): ` +
        Object.entries(r.applied).map(([k, v]) => `${k}: ${JSON.stringify(v.old)} → ${JSON.stringify(v.new)} (${v.effective})`).join('; ');
      draw();
    } catch { /* 409/422 text is shown as a toast */ }
  });
}

// ── RSUs ────────────────────────────────────────────────────────────────────────────────────────
async function rsuCard() {
  store.on('world.state', (env) => {
    const rows = (env.data.rsus || []).map((r) => ({ id: r.station_id, i: r.intersection_id, en: r.enabled, tx: r.tx_power_dbm, rng: r.broadcast_range_m }));
    $('rsu-stamp').textContent = `${rows.length} RSUs · ${u.stamp(env.frame, env.sim_time)}`;
    setRows($('rsu-table').tBodies[0], rows, (r) => {
      const tr = el('tr', 'selectable');
      tr.dataset.station = String(r.id);
      const cb = el('input');
      cb.type = 'checkbox';
      cb.checked = r.en;
      cb.setAttribute('aria-label', `RSU ${r.id} enabled`);
      cb.addEventListener('click', (e) => e.stopPropagation());
      cb.addEventListener('change', async () => {
        try { await api.request('PATCH', `/api/v2x/rsus/${r.id}`, { enabled: cb.checked }); } catch { cb.checked = !cb.checked; }
      });
      const tx = el('input');
      tx.type = 'number'; tx.step = 'any'; tx.min = '0'; tx.max = '40'; tx.value = String(r.tx); tx.style.width = '6em';
      tx.setAttribute('aria-label', `RSU ${r.id} TX power`);
      tx.addEventListener('click', (e) => e.stopPropagation());
      const set = button('Set', async (e) => {
        e.stopPropagation();
        try { await api.request('PATCH', `/api/v2x/rsus/${r.id}`, { tx_power_dbm: Number(tx.value) }); } catch { /* shown */ }
      });
      tr.append(td(String(r.id), 'num'), td(String(r.i), 'num'), td(cb), td(tx, 'num'), td(u.num(r.rng, 0), 'num'), td(set));
      tr.addEventListener('click', () => page.select(r.id));
      return tr;
    });
  });
}

// ── DENM injection ──────────────────────────────────────────────────────────────────────────────
async function denmCard() {
  for (const [code, name] of DENM_CAUSES) {
    if (code === 0) continue;
    const o = el('option', '', `${code} ${name}`);
    o.value = String(code);
    $('dn-cause').appendChild(o);
  }
  $('dn-cause').value = '2';
  let pos = null;
  $('dn-pick').addEventListener('click', () => {
    $('dn-pos').textContent = 'click on the map\u2026';
    page.map.canvas.scrollIntoView({ block: 'center' });          // the cards are below the map: bring the map back
    page.map.pick((x, y) => {
      pos = { x: Math.round(x * 10) / 10, y: Math.round(y * 10) / 10 };
      $('dn-pos').textContent = `x=${pos.x} m, y=${pos.y} m`;
      $('dn-inject').disabled = false;
    });
  });
  $('dn-inject').addEventListener('click', async () => {
    if (!pos) return;
    const body = { x: pos.x, y: pos.y, cause_code: Number($('dn-cause').value), subcause_code: Number($('dn-sub').value),
      relevance_distance_m: Number($('dn-rad').value) };
    if ($('dn-dur').value) body.duration_s = Number($('dn-dur').value);
    if ($('dn-orig').value) body.station_id = Number($('dn-orig').value);
    try {
      const r = await api.post('/api/v2x/denm', body);
      toast(`DENM ${r.action_id} injected (from station ${r.station_id}); it is transmitted from the next V2X update.`, 'ok', 6000);
    } catch { /* shown */ }
  });
  store.on('v2x.frame', (env) => {
    const opts = [['', 'fixed roadside transmitter at the event']];
    for (const s of env.data.stations) {
      if (s.kind === 'RSU' && s.station_id > -1000000) opts.push([String(s.station_id), `RSU ${s.station_id}`]);
      if (s.kind === 'OBU') opts.push([String(s.station_id), `vehicle #${s.station_id}`]);
    }
    const sel = $('dn-orig');
    const sig = JSON.stringify(opts);
    if (sel.dataset.sig !== sig) {
      const cur = sel.value;
      sel.dataset.sig = sig;
      sel.textContent = '';
      for (const [v, t] of opts) { const o = el('option', '', t); o.value = v; sel.appendChild(o); }
      sel.value = opts.some(([v]) => v === cur) ? cur : '';
    }
    const box = $('dn-active');
    const denms = env.data.denms;
    const dsig = JSON.stringify(denms.map((d) => [d.station_id, d.action_id]));
    if (box.dataset.sig === dsig) return;
    box.dataset.sig = dsig;
    box.textContent = '';
    if (!denms.length) { box.appendChild(el('p', 'empty', 'none')); return; }
    const t = el('table');
    t.innerHTML = '<thead><tr><th>from</th><th>action</th><th>cause</th><th>at (m)</th><th class="num">radius</th><th></th></tr></thead>';
    const tb = el('tbody');
    for (const d of denms) {
      const tr = el('tr');
      tr.append(td(String(d.station_id), 'num'), td(String(d.action_id), 'num'), td(String(d.cause_code)),
        td(`${u.num(d.event_position[0], 1)}, ${u.num(d.event_position[1], 1)}`), td(u.metres(d.relevance_distance, 0), 'num'),
        td(button('Cancel', async () => { try { await api.delete(`/api/v2x/denm/${d.station_id}/${d.action_id}`); } catch { /* shown */ } })));
      tb.appendChild(tr);
    }
    t.appendChild(tb);
    box.appendChild(t);
  });
}

// ── sessions ────────────────────────────────────────────────────────────────────────────────────
async function sessionsCard() {
  async function refresh() {
    try {
      const rows = (await api.get('/admin/sessions', { quiet: true })).filter((s) => s.is_active);
      const sensors = (store.get('lidar.sensors') || { data: { sensors: [] } }).data.sensors;
      const view = rows.map((s) => ({ id: s.session_id, name: s.client_name, ip: s.client_ip, actor: s.vehicle_actor_id,
        rem: s.remaining_seconds, conn: s.connected, lidar: sensors.filter((x) => x.owner === s.client_name).map((x) => `#${x.sensor_id} ${x.state}`).join(', ') }));
      setRows($('sess-table').tBodies[0], view.map((v) => ({ ...v, rem: v.rem === null ? null : Math.round(v.rem) })), (v) => {
        const tr = el('tr', 'selectable');
        tr.dataset.session = v.id;
        tr.append(td(v.name), td(v.ip), td(String(v.actor), 'num'), td(v.rem === null ? 'until they leave' : u.age(v.rem)),
          td(v.conn ? 'connected' : 'no socket'), td(v.lidar || u.DASH),
          td(button('Kick', async (e) => { e.stopPropagation(); try { await api.delete(`/admin/sessions/${v.id}`); refresh(); } catch { /* shown */ } }, 'danger')));
        tr.addEventListener('click', () => v.actor != null && page.select(v.actor));
        return tr;
      });
    } catch { /* poll failure: keep the old table */ }
  }
  await refresh();
  setInterval(refresh, 2000);
}

// ── LiDAR ───────────────────────────────────────────────────────────────────────────────────────
async function lidarCard() {
  const schema = await api.get('/api/lidar/schema');
  const rate = config.fixed_delta_seconds ? 1 / config.fixed_delta_seconds : 20;
  const form = new LidarConfigForm($('lid-form'), schema, { simRateHz: rate });
  store.on('lidar.sensors', (env) => {
    const b = env.data.budget;
    $('lid-budget').textContent = `budget ${u.count(b.used_pps)} / ${u.count(b.max_pps)} points/s · ${b.sensors} / ${b.max_sensors} sensors`;
    $('lid-budget').dataset.used = String(b.used_pps);
    const rows = env.data.sensors.map((s) => ({ id: s.sensor_id, veh: s.actor_id, owner: s.owner, state: s.state, viewers: s.viewers, pps: s.config.points_per_second, pinned: s.pinned }));
    setRows($('lid-table').tBodies[0], rows, (r) => {
      const tr = el('tr', 'selectable');
      tr.dataset.sensor = String(r.id);
      const pin = el('input');
      pin.type = 'checkbox'; pin.checked = r.pinned; pin.setAttribute('aria-label', `sensor ${r.id} pinned`);
      pin.addEventListener('click', (e) => e.stopPropagation());
      pin.addEventListener('change', async () => { try { await api.request('PATCH', `/api/lidar/sensors/${r.id}`, { pinned: pin.checked }); } catch { pin.checked = !pin.checked; } });
      const actions = el('span');
      actions.append(button('View', (e) => { e.stopPropagation(); const s = env.data.sensors.find((x) => x.sensor_id === r.id); if (s) page.openLidar(s); }),
        button('Detach', async (e) => { e.stopPropagation(); try { await api.delete(`/api/lidar/sensors/${r.id}`); } catch { /* shown */ } }, 'danger'));
      tr.append(td(`#${r.id}`), td(`#${r.veh}`), td(r.owner), td(el('span', `state-${r.state}`, r.state)), td(String(r.viewers), 'num'), td(u.count(r.pps), 'num'), td(pin), td(actions));
      tr.addEventListener('click', () => page.select(r.veh));
      return tr;
    });
  });
  store.on('world.state', (env) => {
    const vs = env.data.stations.filter((s) => s.kind === 'vehicle').map((s) => [String(s.id), `#${s.id}${s.name ? ` ${s.name}` : ''} (${s.role})`]);
    const sel = $('lid-veh');
    const sig = JSON.stringify(vs);
    if (sel.dataset.sig === sig) return;
    const cur = sel.value;
    sel.dataset.sig = sig;
    sel.textContent = '';
    for (const [v, t] of vs) { const o = el('option', '', t); o.value = v; sel.appendChild(o); }
    if (vs.some(([v]) => v === cur)) sel.value = cur;
  });
  $('lid-attach').addEventListener('click', async () => {
    const r = form.read();
    if (r.errors) { toast(`Fix the sensor settings: ${Object.entries(r.errors).map(([k, v]) => `${k} ${v}`).join('; ')}`, 'error'); return; }
    if (!$('lid-veh').value) { toast('No vehicle to attach to.', 'error'); return; }
    try {
      const resp = await api.post('/api/lidar/sensors', { actor_id: Number($('lid-veh').value), config: r.config, pinned: $('lid-pin').checked });
      toast(`sensor ${resp.sensor_id} requested`, 'ok', 3000);
    } catch { /* 409 budget: shown with the numbers */ }
  });
}

// ── WebSocket clients ───────────────────────────────────────────────────────────────────────────
async function hubCard() {
  let last = null;
  async function refresh() {
    try {
      const r = await api.get('/api/hub/connections', { quiet: true });
      const now = Date.now();
      if (last) $('hub-rate').textContent = `${u.bytes((r.bytes_sent - last.bytes) / ((now - last.t) / 1000))}/s outbound, ${r.connections.length} clients`;
      last = { bytes: r.bytes_sent, t: now };
      const rows = r.connections.map((c) => ({ role: c.role, name: c.client_name || u.DASH, topics: Object.keys(c.topics).join(', '), s: Math.round(c.connected_s), n: c.messages_sent }));
      setRows($('hub-table').tBodies[0], rows, (c) => {
        const tr = el('tr');
        tr.append(td(c.role), td(c.name), td(c.topics || u.DASH), td(String(c.s), 'num'), td(u.count(c.n), 'num'));
        return tr;
      });
    } catch { /* ignore a failed poll */ }
  }
  await refresh();
  setInterval(refresh, 2000);
}
