// Turns a decoded LiDAR frame into render-ready arrays (pure; runs in the worker and in Node tests).
//
// Coordinates: points arrive in the SENSOR frame. The header pose rotates them into the CARLA world orientation
// (same matrix as CARLA's Transform::GetMatrix, see coords.js); the scene origin is the sensor's location, which keeps
// float32 precise far from the map origin; CARLA -> Three.js then mirrors y. Overlays use the same shift:
// three = mirrorY(carla_world - sensor_location).

import { transformMatrix } from './coords.js';

export const COLOR_MODES = ['semantic', 'range', 'height', 'intensity', 'highlight'];

/** Which colour modes a frame can honestly offer (no silent fallback: unavailable modes are not offered). */
export function availableModes(points) {
  const m = ['range', 'height'];
  if (points.tag) m.unshift('semantic');
  if (points.intensity) m.push('intensity');
  if (points.objectIdx) m.push('highlight');
  return m;
}

/** A compact "turbo-like" ramp, t in [0,1] -> [r,g,b] in [0,1]. */
export function ramp(t) {
  const x = Math.min(1, Math.max(0, t));
  const r = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * x - 3)));
  const g = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * x - 2)));
  const b = Math.min(1, Math.max(0, 1.5 - Math.abs(4 * x - 1)));
  return [r, g, b];
}

/** palette: [{id, color: [r,g,b] 0..255}] from GET /api/lidar/schema -> Float32Array(256*3) in 0..1. */
export function paletteLut(palette) {
  const lut = new Float32Array(256 * 3).fill(0.5);
  for (const p of palette || []) {
    if (p.id >= 0 && p.id < 256) { lut[p.id * 3] = p.color[0] / 255; lut[p.id * 3 + 1] = p.color[1] / 255; lut[p.id * 3 + 2] = p.color[2] / 255; }
  }
  return lut;
}

/**
 * @param {{header: object, points: object}} decoded
 * @param {object} o
 * @param {string} o.colorMode  one of COLOR_MODES (must be available for this frame)
 * @param {Float32Array} [o.lut]  semantic palette from paletteLut
 * @param {boolean} [o.egoFilter]  drop points whose objectIdx is the ego vehicle (default true)
 * @param {number} [o.highlightId] actor id for the 'highlight' mode
 * @param {number} [o.maxPoints] capacity of the render buffers; extra points are dropped (and counted)
 * @param {number} [o.rangeMax]  metres at which the range ramp saturates
 * @param {number} [o.heightSpan] metres above/below the sensor covered by the height ramp
 */
export function processFrame(decoded, o = {}) {
  const { header, points: p } = decoded;
  const n = header.numPoints;
  const mode = o.colorMode || 'range';
  const modes = availableModes(p);
  if (!modes.includes(mode)) throw new Error(`colour mode ${mode} is not available for this frame (has: ${modes.join(', ')})`);
  const egoFilter = o.egoFilter !== false && !!p.objectIdx;
  const max = o.maxPoints || n;
  const lut = o.lut || paletteLut([]);
  const rangeMax = o.rangeMax || 100, heightSpan = o.heightSpan || 6;
  const m = header.sensorFrame ? transformMatrix([0, 0, 0], header.rotationPYR) : null;
  const loc = header.sensorFrame ? [0, 0, 0] : header.location;      // world-frame frames are shifted by the sensor location
  const positions = new Float32Array(Math.min(n, max) * 3);
  const colors = new Float32Array(Math.min(n, max) * 3);
  let out = 0, filtered = 0, truncated = 0;
  for (let i = 0; i < n; i++) {
    if (egoFilter && p.objectIdx[i] === header.egoActorId) { filtered++; continue; }
    if (out >= max) { truncated++; continue; }
    const x = p.x[i], y = p.y[i], z = p.z[i];
    let wx, wy, wz;
    if (m) {
      wx = m[0] * x + m[1] * y + m[2] * z; wy = m[4] * x + m[5] * y + m[6] * z; wz = m[8] * x + m[9] * y + m[10] * z;
    } else {
      wx = x - loc[0]; wy = y - loc[1]; wz = z - loc[2];
    }
    positions[3 * out] = wx; positions[3 * out + 1] = -wy; positions[3 * out + 2] = wz;       // CARLA -> Three: mirror y
    let c;
    switch (mode) {
      case 'semantic': { const t = p.tag[i] & 255; c = [lut[3 * t], lut[3 * t + 1], lut[3 * t + 2]]; break; }
      case 'range': c = ramp(Math.sqrt(x * x + y * y + z * z) / rangeMax); break;
      case 'height': c = ramp(0.5 + wz / (2 * heightSpan)); break;
      case 'intensity': { const v = Math.min(1, Math.max(0, p.intensity[i])); c = [v, v, v]; break; }
      default: c = p.objectIdx[i] === o.highlightId ? [1, 0.2, 0.9] : [0.35, 0.37, 0.4];
    }
    colors[3 * out] = c[0]; colors[3 * out + 1] = c[1]; colors[3 * out + 2] = c[2];
    out++;
  }
  return {
    header: { ...header, buffer: undefined },
    positions: positions.subarray(0, out * 3), colors: colors.subarray(0, out * 3),
    counts: { received: n, displayed: out, filteredEgo: filtered, truncated },
    modes,
  };
}

/** Frames per second and kB/s over a sliding wall-clock window. */
export class RateMeter {
  constructor(windowMs = 2000, now = () => Date.now()) {
    this.windowMs = windowMs;
    this.now = now;
    this.events = [];
  }

  add(bytes = 0) {
    const t = this.now();
    this.events.push([t, bytes]);
    this._trim(t);
  }

  _trim(t) {
    while (this.events.length && this.events[0][0] < t - this.windowMs) this.events.shift();
  }

  rates() {
    const t = this.now();
    this._trim(t);
    if (this.events.length < 2) return { fps: 0, bytesPerSecond: 0 };
    const span = (this.events[this.events.length - 1][0] - this.events[0][0]) / 1000;
    if (span <= 0) return { fps: 0, bytesPerSecond: 0 };
    const bytes = this.events.slice(1).reduce((a, e) => a + e[1], 0);
    return { fps: (this.events.length - 1) / span, bytesPerSecond: bytes / span };
  }
}

/** Gap between the LiDAR frame and the V2X frame used for the overlays, in simulated seconds. */
export function frameGapSeconds(lidarFrame, v2xFrame, dt) {
  return v2xFrame === null || v2xFrame === undefined ? null : (lidarFrame - v2xFrame) * dt;
}
