// Station inspector: identity, ground truth next to the last transmitted message, inbox table, threats, signals,
// VRUs and DENMs. Every message can be shown as raw JSON. Every number is formatted by units.js only.

import * as u from './units.js';
import { inboxRows, spatCountdown } from './panels-model.js';

function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

function kv(rows) {
  const dl = el('dl', 'kv');
  for (const [k, v, title] of rows) {
    dl.appendChild(el('dt', '', k));
    const dd = el('dd', '', v);
    if (title) dd.title = title;
    dl.appendChild(dd);
  }
  return dl;
}

function table(columns, rows, { onRow } = {}) {
  const t = el('table');
  const thead = el('thead');
  const hr = el('tr');
  for (const c of columns) hr.appendChild(el('th', c.num ? 'num' : '', c.label));
  thead.appendChild(hr);
  t.appendChild(thead);
  const tb = el('tbody');
  for (const r of rows) {
    const tr = el('tr', onRow ? 'selectable' : '');
    for (const c of columns) {
      const cell = el('td', c.num ? 'num' : '');
      const v = c.get(r);
      if (v && typeof v === 'object') { cell.textContent = v.text; cell.title = v.title || ''; } else cell.textContent = v;
      tr.appendChild(cell);
    }
    if (onRow) tr.addEventListener('click', () => onRow(r));
    tb.appendChild(tr);
  }
  t.appendChild(tb);
  return t;
}

function rawToggle(label, obj) {
  const d = el('details');
  d.appendChild(el('summary', '', label));
  const pre = el('pre', 'json');
  pre.textContent = JSON.stringify(obj, null, 2);
  d.appendChild(pre);
  return d;
}

export class Inspector {
  constructor(container, { onSelectStation = () => {} } = {}) {
    this.root = container;
    this.onSelectStation = onSelectStation;
  }

  clear(message = 'Click a station on the map.') {
    this.root.textContent = '';
    this.root.appendChild(el('p', 'empty', message));
  }

  /**
   * @param {object} m
   * @param {object} m.station ground-truth entry of world.state (or an RSU entry)
   * @param {object|null} m.inbox envelope of v2x.inbox ({data, frame, sim_time})
   * @param {object|null} m.summary the station's entry of v2x.frame
   * @param {object|null} m.delivery DeliveryTracker.summary()
   * @param {object} m.sent SentCounter.summary()
   * @param {object|null} m.lidar sensor info owned by this station's actor
   */
  render({ station, inbox, summary, delivery, sent, lidar, truthLabel = 'Ground truth (world.state)' }) {
    const root = this.root;
    root.textContent = '';
    if (!station) return this.clear();
    const simTime = inbox ? inbox.sim_time : null;

    root.appendChild(el('h3', '', `${station.name || (station.kind === 'RSU' ? 'RSU' : 'Station')} #${station.id}`));
    root.appendChild(kv([
      ['kind', `${station.kind}${station.role ? ` (${station.role})` : ''}`],
      ['V2X equipped', station.v2x === undefined ? (station.kind === 'RSU' ? 'yes (infrastructure)' : u.DASH) : station.v2x ? 'yes' : 'no',
        station.v2x === undefined ? 'not part of this record' : ''],
      ['blueprint', u.DASH, 'the blueprint is not published by the server'],
      ['owner session', station.role === 'client' ? station.name : u.DASH, station.role === 'client' ? '' : 'background actor'],
      ['LiDAR', lidar ? `#${lidar.sensor_id} ${lidar.state}` : 'none'],
    ]));

    root.appendChild(el('h3', '', truthLabel));
    root.appendChild(kv([
      ['position', `${u.num(station.x, 2)}, ${u.num(station.y, 2)}, ${u.num(station.z, 2)} m`, `${station.x}, ${station.y}, ${station.z}`],
      ['yaw', u.degrees(station.yaw, 1), String(station.yaw)],
      ['speed', u.speed(station.speed_mps, 2), String(station.speed_mps)],
    ]));

    if (!inbox) {
      root.appendChild(el('p', 'empty', 'Waiting for the V2X data of this station (v2x.inbox)…'));
      return;
    }
    const d = inbox.data;
    root.appendChild(el('p', 'stamp', `inbox: ${u.stamp(inbox.frame, inbox.sim_time)}`));

    // last transmitted message(s), full precision
    root.appendChild(el('h3', '', 'Last transmitted message(s)'));
    if (!d.sent.length) root.appendChild(el('p', 'empty', 'nothing sent in the last V2X update'));
    for (const m of d.sent) {
      const rows = Object.entries(m).filter(([k, v]) => k !== 'type' && typeof v !== 'object' || v === null)
        .map(([k, v]) => [k, v === null ? u.DASH : String(v), v === null ? 'no value in the message' : '']);
      root.appendChild(el('strong', '', m.type));
      root.appendChild(kv(rows.slice(0, 40)));
      root.appendChild(rawToggle(`raw ${m.type}`, m));
    }
    const sentRows = Object.entries(sent || {});
    if (sentRows.length) {
      root.appendChild(kv(sentRows.map(([t, s]) => [`sent ${t}`, `${s.count} in ${Object.keys(sent).length ? 'the last updates' : ''}` +
        (s.perSecond ? `  ·  ${u.hertz(s.perSecond, 2)}` : '')])));
    }

    // inbox
    root.appendChild(el('h3', '', `Inbox (${d.messages.length} received in this update)`));
    if (delivery && delivery.ratio !== null) {
      root.appendChild(el('p', '', `delivery ratio over the last ${delivery.updates} updates: ${u.percent(delivery.ratio, 1)} (${delivery.delivered}/${delivery.counted} links; awareness-filtered links not counted)`));
    }
    const rows = inboxRows(d, simTime);
    const columns = [
      { label: 'tx', num: true, get: (r) => String(r.tx_id) }, { label: 'link', get: (r) => r.link_type }, { label: 'msg', get: (r) => r.msg_type },
      { label: 'age', num: true, get: (r) => u.orDash(r.age_s, u.age, 'this message type has no time stamp') },
      { label: 'dist', num: true, get: (r) => u.metres(r.distance_m, 1) }, { label: 'LOS', get: (r) => (r.los ? 'LOS' : 'NLOS') },
      { label: 'rx power', num: true, get: (r) => u.dBm(r.rx_power_dbm, 1) }, { label: 'margin', num: true, get: (r) => u.dB(r.rx_margin_db, 1) },
      { label: 'p_total', num: true, get: (r) => u.num(r.p_total, 3) },
    ];
    const wrap = el('div', 'scroll');
    wrap.appendChild(table(columns, rows, { onRow: (r) => { this._showRaw(r.raw); } }));
    root.appendChild(wrap);
    this.rawBox = el('div');
    root.appendChild(this.rawBox);

    // threats computed from the inbox
    root.appendChild(el('h3', '', 'Threats (from the inbox)'));
    if (!d.threats.length) root.appendChild(el('p', 'empty', 'none'));
    else {
      root.appendChild(table([
        { label: 'other', num: true, get: (r) => String(r.other_id) }, { label: 'level', num: true, get: (r) => String(r.level) },
        { label: 'TTC', num: true, get: (r) => u.orDash(r.ttc_s ?? r.ttc, u.seconds, 'no finite time to collision') },
        { label: 'distance', num: true, get: (r) => u.metres(r.distance_m, 1) },
      ], d.threats, { onRow: (r) => this.onSelectStation(r.other_id) }));
    }

    // signals, VRUs, DENMs received
    const spat = d.messages.filter((m) => m.msg_type === 'SPAT');
    root.appendChild(el('h3', '', 'Signals received'));
    if (!spat.length) root.appendChild(el('p', 'empty', 'no SPAT in range'));
    for (const m of spat) {
      const msg = m.message;
      root.appendChild(el('strong', '', `intersection ${msg.intersection_id} ${msg.intersection_name || ''}`));
      root.appendChild(table([
        { label: 'group', num: true, get: (s) => String(s.signal_group_id) }, { label: 'state', get: (s) => s.movement_event },
        { label: 'ends in', num: true, get: (s) => spatCountdown(s, simTime) },
      ], msg.states));
    }
    const vru = d.messages.filter((m) => m.msg_type === 'VRUAM');
    root.appendChild(el('h3', '', `VRUs heard (${vru.length})`));
    if (vru.length) {
      root.appendChild(table([
        { label: 'id', num: true, get: (m) => String(m.tx_id) }, { label: 'profile', get: (m) => String(m.message.vru_profile) },
        { label: 'speed', num: true, get: (m) => u.speed(m.message.speed, 1) }, { label: 'dist', num: true, get: (m) => u.metres(m.link.distance_m, 1) },
      ], vru, { onRow: (m) => this.onSelectStation(m.tx_id) }));
    }
    const denm = d.messages.filter((m) => m.msg_type === 'DENM');
    root.appendChild(el('h3', '', `DENMs received (${denm.length})`));
    for (const m of denm) {
      const x = m.message;
      root.appendChild(kv([
        ['cause', `${x.cause_code} / ${x.subcause_code}`], ['event at', `${u.num(x.event_position[0], 1)}, ${u.num(x.event_position[1], 1)} m`],
        ['relevance', u.metres(x.relevance_distance, 0)], ['from', `#${m.tx_id} via ${m.link.link_type}`],
        ['detected', u.seconds(x.detection_time, 2)],
      ]));
    }
  }

  _showRaw(msg) {
    if (!this.rawBox) return;
    this.rawBox.textContent = '';
    this.rawBox.appendChild(rawToggle(`raw ${msg.msg_type} from #${msg.tx_id}`, msg));
    this.rawBox.querySelector('details').open = true;
  }
}

/** Every field of one LinkResult (click on a link). */
export function renderLinkDetails(container, link) {
  container.textContent = '';
  if (!link) { container.appendChild(el('p', 'empty', 'Click a link on the map or in the log to see all of its fields.')); return; }
  const rows = Object.entries(link).map(([k, v]) => [k, v === null ? u.DASH : String(v), v === null ? 'no value (for example no drop cause: delivered)' : '']);
  container.appendChild(kv(rows));
}
