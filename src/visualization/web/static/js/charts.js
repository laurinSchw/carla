// Channel analytics with vendored uPlot: PRR vs distance (LOS and NLOS apart, Wilson 95 % band, n per bin),
// receive-margin histogram and small time series. The numbers come from v2x.stats (src/v2x/stats.py).

import uPlot from '/static/vendor/uplot/uPlot.esm.js';
import { prrSeries, marginBars } from './panels-model.js';

function cssVar(name, fallback) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
}

const SERIES_STYLE = {
  'V2V/LOS': { dash: [], width: 2 }, 'V2V/NLOS': { dash: [6, 4], width: 2 },
  'I2V/LOS': { dash: [], width: 1.5 }, 'I2V/NLOS': { dash: [6, 4], width: 1.5 },
  'P2V/LOS': { dash: [], width: 1.5 }, 'P2V/NLOS': { dash: [6, 4], width: 1.5 },
};
const PALETTE = { V2V: '--accent', I2V: '--warn', P2V: '--link-aware', V2I: '--ok', V2P: '--bad' };

export class PrrChart {
  constructor(container, { height = 240 } = {}) {
    this.container = container;
    this.height = height;
    this.plot = null;
    this.legend = document.createElement('div');
    this.legend.className = 'legend';
    this.note = document.createElement('p');
    this.note.className = 'stamp';
    this.chart = document.createElement('div');
    container.append(this.chart, this.legend, this.note);
  }

  update(stats, blockName) {
    const block = stats[blockName];
    const keys = Object.keys(block.prr_by_distance);
    this.note.textContent = block.updates
      ? `${block.updates} V2X updates, frames ${block.first_frame}–${block.last_frame}; ${stats.definition}`
      : 'no V2X update yet';
    if (!keys.length) { this.destroy(); this.chart.textContent = ''; return; }
    const starts = stats.distance_bin_starts_m;
    const x = prrSeries(block.prr_by_distance[keys[0]], starts, stats.bin_m).x;
    const data = [x];
    const series = [{}];
    this.legend.textContent = '';
    for (const k of keys) {
      const s = prrSeries(block.prr_by_distance[k], starts, stats.bin_m);
      const colour = cssVar(PALETTE[k.split('/')[0]] || '--accent', '#06c');
      const st = SERIES_STYLE[k] || { dash: [], width: 1.5 };
      data.push(s.y);
      series.push({ label: k, stroke: colour, width: st.width, dash: st.dash, spanGaps: false, points: { show: true, size: 4 } });
      data.push(s.lo); series.push({ label: `${k} lo`, stroke: colour, width: 0.5, dash: [1, 3], show: true, points: { show: false } });
      data.push(s.hi); series.push({ label: `${k} hi`, stroke: colour, width: 0.5, dash: [1, 3], show: true, points: { show: false } });
      const n = s.n.reduce((a, b) => a + b, 0);
      const item = document.createElement('span');
      item.textContent = `${k} (n=${n})`;
      item.style.borderBottom = `2px ${st.dash.length ? 'dashed' : 'solid'} ${colour}`;
      this.legend.appendChild(item);
    }
    const opts = {
      width: this.container.clientWidth || 400, height: this.height, series,
      scales: { x: { time: false }, y: { range: [0, 1.02] } },
      axes: [{ label: 'distance (m)', stroke: cssVar('--muted', '#666'), grid: { stroke: cssVar('--border', '#ddd') } },
        { label: 'PRR', stroke: cssVar('--muted', '#666'), grid: { stroke: cssVar('--border', '#ddd') } }],
      legend: { show: false },
    };
    this.destroy();
    this.chart.textContent = '';
    this.plot = new uPlot(opts, data, this.chart);
    this.keys = keys;
  }

  destroy() {
    if (this.plot) { this.plot.destroy(); this.plot = null; }
  }
}

export class MarginChart {
  constructor(container, { height = 180 } = {}) {
    this.container = container;
    this.height = height;
    this.plot = null;
    this.chart = document.createElement('div');
    container.append(this.chart);
  }

  update(stats, blockName) {
    const h = stats[blockName].rx_margin_hist;
    if (!h) { if (this.plot) { this.plot.destroy(); this.plot = null; } return; }
    const m = marginBars(h);
    const bars = uPlot.paths.bars({ size: [0.8, 40] });
    const opts = {
      width: this.container.clientWidth || 400, height: this.height,
      series: [{}, { label: 'delivered', stroke: cssVar('--ok', 'green'), fill: cssVar('--ok', 'green'), paths: bars, points: { show: false } },
        { label: 'dropped', stroke: cssVar('--bad', 'red'), fill: cssVar('--bad', 'red'), paths: bars, points: { show: false } }],
      scales: { x: { time: false } },
      axes: [{ label: 'rx margin above sensitivity (dB)', stroke: cssVar('--muted', '#666') }, { label: 'links', stroke: cssVar('--muted', '#666') }],
      legend: { show: true },
    };
    if (this.plot) this.plot.destroy();
    this.chart.textContent = '';
    this.plot = new uPlot(opts, [m.x, m.delivered, m.dropped], this.chart);
  }
}

export class LineChart {
  constructor(container, { label, unit = '', height = 130, color = '--accent' } = {}) {
    this.container = container;
    this.label = label;
    this.unit = unit;
    this.height = height;
    this.color = color;
    this.plot = null;
    this.chart = document.createElement('div');
    container.append(this.chart);
  }

  update(data) {
    if (data[0].length < 2) return;
    if (!this.plot) {
      this.plot = new uPlot({
        width: this.container.clientWidth || 400, height: this.height,
        series: [{}, { label: this.label, stroke: cssVar(this.color, '#06c'), width: 1.5, points: { show: false } }],
        scales: { x: { time: false } },
        axes: [{ label: 'simulation time (s)', stroke: cssVar('--muted', '#666') }, { label: this.unit, stroke: cssVar('--muted', '#666') }],
        legend: { show: true },
      }, data, this.chart);
    } else {
      this.plot.setData(data);
    }
  }
}
