/* Canvas LiDAR viewer (no dependencies): top-down and 3-D orbit view of one decoded frame.
 *
 * Points arrive in the SENSOR frame (CARLA: x forward, y right, z up), so the vehicle is at the origin and
 * "up" on the top-down screen is the driving direction. Frames are drawn exactly as decoded; nothing is
 * accumulated or interpolated.
 */
(function (root) {
  'use strict';

  const FALLBACK_TAG_COLORS = { 0: [0, 0, 0], 1: [128, 64, 128], 2: [244, 35, 232], 3: [70, 70, 70], 12: [220, 20, 60], 14: [0, 0, 142] };

  function hsv(h, s, v) {
    const f = (n, k = (n + h * 6) % 6) => v - v * s * Math.max(Math.min(k, 4 - k, 1), 0);
    return [Math.round(f(5) * 255), Math.round(f(3) * 255), Math.round(f(1) * 255)];
  }

  class LidarView {
    constructor(canvas, options = {}) {
      this.canvas = canvas;
      this.ctx = canvas.getContext('2d');
      this.mode = options.mode || 'top';        // 'top' | '3d'
      this.color = options.color || 'tag';      // 'tag' | 'height' | 'distance' | 'intensity'
      this.hideOwn = options.hideOwn !== false; // drop hits on the ego vehicle itself (object_idx == ego_actor_id)
      this.pointSize = options.pointSize || 2;
      this.rangeM = options.rangeM || 40;       // metres from the centre to the edge of the top-down view
      this.yaw = -0.6; this.pitch = 0.9;        // 3-D orbit angles (rad)
      this.tagColors = Object.assign({}, FALLBACK_TAG_COLORS);
      this.frame = null;
      this._drag = null;
      this._bind();
    }

    setTagTable(tags) {
      for (const t of tags) this.tagColors[t.id] = t.color;
    }

    setFrame(frame) {
      this.frame = frame;
      this.render();
    }

    _bind() {
      const c = this.canvas;
      c.addEventListener('wheel', (e) => {
        e.preventDefault();
        this.rangeM = Math.min(250, Math.max(5, this.rangeM * (e.deltaY > 0 ? 1.12 : 0.89)));
        this.render();
      }, { passive: false });
      c.addEventListener('pointerdown', (e) => { this._drag = [e.clientX, e.clientY]; c.setPointerCapture(e.pointerId); });
      c.addEventListener('pointerup', () => { this._drag = null; });
      c.addEventListener('pointermove', (e) => {
        if (!this._drag || this.mode !== '3d') return;
        this.yaw += (e.clientX - this._drag[0]) * 0.01;
        this.pitch = Math.min(1.5, Math.max(0.05, this.pitch + (e.clientY - this._drag[1]) * 0.01));
        this._drag = [e.clientX, e.clientY];
        this.render();
      });
    }

    _color(f, i, dist) {
      switch (this.color) {
        case 'tag':
          if (f.tag) return this.tagColors[f.tag[i]] || [200, 200, 200];
        // falls through when the sensor has no tags (ray_cast)
        case 'intensity':
          if (f.intensity) { const v = Math.round(60 + 195 * Math.min(1, f.intensity[i])); return [v, v, v]; }
        // falls through
        case 'height': return hsv(0.66 - 0.66 * Math.min(1, Math.max(0, (f.z[i] + 3) / 6)), 0.85, 1);
        case 'distance': return hsv(0.66 * Math.min(1, dist / this.rangeM), 0.85, 1);
      }
      return [255, 255, 255];
    }

    render() {
      const { canvas, ctx } = this;
      const w = canvas.width, h = canvas.height;
      ctx.fillStyle = '#0b0e14';
      ctx.fillRect(0, 0, w, h);
      if (this.mode === 'top') this._grid(w, h);
      const f = this.frame;
      if (!f) { this._text('waiting for LiDAR data…', w / 2, h / 2); return; }

      const ego = f.header.egoActorId, n = f.header.numPoints;
      const buckets = new Map(); // colour -> flat list of screen coords, to batch fillStyle changes
      let drawn = 0;
      const cy = Math.cos(this.yaw), sy = Math.sin(this.yaw), cp = Math.cos(this.pitch), sp = Math.sin(this.pitch);
      const scale = (Math.min(w, h) / 2) / this.rangeM;
      for (let i = 0; i < n; i++) {
        if (this.hideOwn && f.objectIdx && f.objectIdx[i] === ego) continue;
        const x = f.x[i], y = f.y[i], z = f.z[i];
        let sx, sy2;
        if (this.mode === 'top') {
          sx = w / 2 + y * scale; sy2 = h / 2 - x * scale;
        } else {
          const rx = x * cy - y * sy, ry = x * sy + y * cy;          // rotate about z
          const depth = ry * cp + z * sp, up = z * cp - ry * sp;      // tilt
          const persp = 1 / (1 + Math.max(-0.8, depth / (this.rangeM * 2.2)));
          sx = w / 2 + rx * scale * persp; sy2 = h * 0.62 - up * scale * persp;
        }
        if (sx < 0 || sy2 < 0 || sx >= w || sy2 >= h) continue;
        const col = this._color(f, i, Math.hypot(x, y));
        const key = (col[0] >> 3) << 10 | (col[1] >> 3) << 5 | (col[2] >> 3);
        let b = buckets.get(key);
        if (!b) { b = { col, xy: [] }; buckets.set(key, b); }
        b.xy.push(sx, sy2);
        drawn++;
      }
      const s = this.pointSize;
      for (const { col, xy } of buckets.values()) {
        ctx.fillStyle = `rgb(${col[0]},${col[1]},${col[2]})`;
        for (let k = 0; k < xy.length; k += 2) ctx.fillRect(xy[k], xy[k + 1], s, s);
      }
      this._ego(w, h, scale);
      this.lastDrawn = drawn;
    }

    _grid(w, h) {
      const { ctx } = this;
      const scale = (Math.min(w, h) / 2) / this.rangeM;
      ctx.strokeStyle = '#1d2433'; ctx.fillStyle = '#44506b'; ctx.lineWidth = 1; ctx.font = '11px sans-serif';
      const step = this.rangeM > 120 ? 50 : this.rangeM > 50 ? 20 : this.rangeM > 20 ? 10 : 5;
      for (let r = step; r <= this.rangeM; r += step) {
        ctx.beginPath(); ctx.arc(w / 2, h / 2, r * scale, 0, 2 * Math.PI); ctx.stroke();
        ctx.fillText(r + ' m', w / 2 + 3, h / 2 - r * scale - 2);
      }
      ctx.beginPath(); ctx.moveTo(w / 2, 0); ctx.lineTo(w / 2, h); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
    }

    _ego(w, h, scale) {
      const { ctx } = this;
      ctx.fillStyle = '#4f8ef7';
      if (this.mode === 'top') {
        ctx.beginPath();
        ctx.moveTo(w / 2, h / 2 - 2.3 * scale); ctx.lineTo(w / 2 + 0.9 * scale, h / 2 + 2.3 * scale);
        ctx.lineTo(w / 2 - 0.9 * scale, h / 2 + 2.3 * scale); ctx.closePath(); ctx.fill();
      }
    }

    _text(msg, x, y) {
      this.ctx.fillStyle = '#7d8590'; this.ctx.font = '14px sans-serif'; this.ctx.textAlign = 'center';
      this.ctx.fillText(msg, x, y); this.ctx.textAlign = 'start';
    }
  }

  root.LidarView = LidarView;
})(typeof self !== 'undefined' ? self : this);
