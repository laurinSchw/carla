// Live link table (from the v2x.links subscription) with pause, a client ring buffer and CSV export of the view.

import { LinkLogBuffer } from './panels-model.js';
import * as u from './units.js';
import { linkOutcome, LINK_STYLES } from './map-model.js';

const SHOWN = 200;

export class LinkLog {
  constructor(container, { onSelectLink = () => {}, max = 5000 } = {}) {
    this.buf = new LinkLogBuffer(max);
    this.onSelectLink = onSelectLink;
    this.columns = null;
    this.filterText = '(no filter)';
    const bar = document.createElement('div');
    bar.className = 'row';
    this.pauseBtn = Object.assign(document.createElement('button'), { textContent: 'Pause' });
    this.csvBtn = Object.assign(document.createElement('button'), { textContent: 'Download CSV of this view' });
    this.clearBtn = Object.assign(document.createElement('button'), { textContent: 'Clear' });
    this.info = document.createElement('span');
    this.info.className = 'stamp';
    bar.append(this.pauseBtn, this.csvBtn, this.clearBtn, this.info);
    this.scroll = document.createElement('div');
    this.scroll.className = 'scroll';
    container.append(bar, this.scroll);
    this.pauseBtn.addEventListener('click', () => {
      this.buf.paused = !this.buf.paused;
      this.pauseBtn.textContent = this.buf.paused ? 'Resume' : 'Pause';
      this.render();
    });
    this.clearBtn.addEventListener('click', () => { this.buf.clear(); this.render(); });
    this.csvBtn.addEventListener('click', () => this.download());
  }

  setFilter(text) {
    this.filterText = text;
  }

  onData(env) {
    this.columns = env.data.columns;
    const added = this.buf.push(env.frame, env.sim_time, env.data);
    this.lastEnv = env;
    if (added || this.buf.paused) this.render();
  }

  render() {
    const rows = this.buf.newest(SHOWN);
    const t = document.createElement('table');
    t.innerHTML = '<thead><tr><th class="num">frame</th><th class="num">tx</th><th class="num">rx</th><th>msg</th><th>link</th><th>outcome</th><th class="num">dist</th><th>LOS</th><th class="num">rx power</th><th class="num">margin</th><th class="num">p_total</th></tr></thead>';
    const tb = document.createElement('tbody');
    for (const r of rows) {
      const out = linkOutcome(r);
      const tr = document.createElement('tr');
      tr.className = 'selectable';
      const cells = [
        [String(r.frame), 'num'], [String(r.tx_id), 'num'], [String(r.rx_id), 'num'], [r.msg_type], [r.link_type],
        [LINK_STYLES[out].label], [u.metres(r.distance_m, 1), 'num'], [r.los ? 'LOS' : 'NLOS'],
        [u.dBm(r.rx_power_dbm, 1), 'num'], [u.dB(r.rx_margin_db, 1), 'num'], [u.num(r.p_total, 3), 'num'],
      ];
      for (const [text, cls] of cells) {
        const td = document.createElement('td');
        td.textContent = text;
        if (cls) td.className = cls;
        tr.appendChild(td);
      }
      tr.addEventListener('click', () => this.onSelectLink(r));
      tb.appendChild(tr);
    }
    t.appendChild(tb);
    this.scroll.textContent = '';
    this.scroll.appendChild(t);
    const e = this.lastEnv;
    const trunc = e && e.data.truncated ? `  ·  TRUNCATED: ${e.data.matching} match, ${e.data.max_rows} sent` : '';
    this.info.textContent = `${this.buf.rows.length} rows held${this.buf.dropped ? ` (${this.buf.dropped} older dropped)` : ''}`
      + `${this.buf.paused ? '  ·  PAUSED' : ''}${trunc}${e ? `  ·  ${u.stamp(e.frame, e.sim_time)}` : ''}`;
  }

  download() {
    if (!this.columns) return;
    const first = this.buf.rows[0] ? this.buf.rows[0].frame : 0, last = this.buf.rows.at(-1) ? this.buf.rows.at(-1).frame : 0;
    const blob = new Blob([this.buf.toCsv(this.columns, this.filterText)], { type: 'text/csv' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `v2x_links_live-filtered_frames_${first}-${last}.csv`;
    document.body.appendChild(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }
}
