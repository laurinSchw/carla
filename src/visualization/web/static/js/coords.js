// Coordinate conventions in one place (mirror of src/v2x/geometry.py and src/lidar/transforms.py).
//
// CARLA (Unreal): left-handed, x forward, y right, z up, angles in degrees, yaw turns x towards y.
// The columns of a CARLA transform matrix are the object's forward, right and up axes in world coordinates.
//
//  * canvas map:  screen_x = (x - cx) * s + W/2,  screen_y = (y - cy) * s + H/2. CARLA seen from above (z towards
//    the viewer) has the same handedness as a canvas whose y points down, so nothing is mirrored; a heading is drawn
//    with ctx.rotate(yaw * pi / 180).
//  * Three.js:    right-handed, so CARLA -> Three mirrors y: (x, y, z) -> (x, -y, z), camera.up = (0, 0, 1).

export function basisVectors(yawDeg, pitchDeg = 0, rollDeg = 0) {
  const d = Math.PI / 180;
  const y = yawDeg * d, p = pitchDeg * d, r = rollDeg * d;
  const cy = Math.cos(y), sy = Math.sin(y), cp = Math.cos(p), sp = Math.sin(p), cr = Math.cos(r), sr = Math.sin(r);
  return {
    forward: [cp * cy, cp * sy, sp],
    right: [cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, -cp * sr],
    up: [-cy * sp * cr - sy * sr, -sy * sp * cr + cy * sr, cp * cr],
  };
}

/** 4x4 row-major local -> world matrix (Float64Array(16)) for location [x,y,z] and [pitch, yaw, roll] degrees. */
export function transformMatrix(location, rotationPYR) {
  const [pitch, yaw, roll] = rotationPYR;
  const { forward, right, up } = basisVectors(yaw, pitch, roll);
  const m = new Float64Array(16);
  for (let i = 0; i < 3; i++) {
    m[i * 4 + 0] = forward[i];
    m[i * 4 + 1] = right[i];
    m[i * 4 + 2] = up[i];
    m[i * 4 + 3] = location[i];
  }
  m[15] = 1;
  return m;
}

/** Apply a pose to interleaved xyz (Float32Array/Array, length 3N): sensor frame -> world. Returns Float64Array. */
export function toWorld(xyz, location, rotationPYR) {
  const m = transformMatrix(location, rotationPYR);
  const n = xyz.length / 3;
  const out = new Float64Array(xyz.length);
  for (let i = 0; i < n; i++) {
    const x = xyz[3 * i], y = xyz[3 * i + 1], z = xyz[3 * i + 2];
    out[3 * i] = m[0] * x + m[1] * y + m[2] * z + m[3];
    out[3 * i + 1] = m[4] * x + m[5] * y + m[6] * z + m[7];
    out[3 * i + 2] = m[8] * x + m[9] * y + m[10] * z + m[11];
  }
  return out;
}

/** Inverse of toWorld (the rotation is orthonormal, so the inverse is the transpose). */
export function toSensor(xyz, location, rotationPYR) {
  const m = transformMatrix(location, rotationPYR);
  const n = xyz.length / 3;
  const out = new Float64Array(xyz.length);
  for (let i = 0; i < n; i++) {
    const x = xyz[3 * i] - m[3], y = xyz[3 * i + 1] - m[7], z = xyz[3 * i + 2] - m[11];
    out[3 * i] = m[0] * x + m[4] * y + m[8] * z;
    out[3 * i + 1] = m[1] * x + m[5] * y + m[9] * z;
    out[3 * i + 2] = m[2] * x + m[6] * y + m[10] * z;
  }
  return out;
}

/** CARLA world point -> Three.js point. */
export function carlaToThree(p) {
  return [p[0], -p[1], p[2]];
}

/** In place on an interleaved array: mirror y. */
export function mirrorYInPlace(xyz) {
  for (let i = 1; i < xyz.length; i += 3) xyz[i] = -xyz[i];
  return xyz;
}

// ── map viewport ────────────────────────────────────────────────────────────────────────────────

export function createViewport({ width, height, cx = 0, cy = 0, scale = 1 }) {
  return { width, height, cx, cy, scale };
}

export function worldToScreen(vp, x, y) {
  return [(x - vp.cx) * vp.scale + vp.width / 2, (y - vp.cy) * vp.scale + vp.height / 2];
}

export function screenToWorld(vp, sx, sy) {
  return [(sx - vp.width / 2) / vp.scale + vp.cx, (sy - vp.height / 2) / vp.scale + vp.cy];
}

/** Canvas rotation angle (radians, for ctx.rotate) of a CARLA yaw in degrees. */
export function yawToCanvasAngle(yawDeg) {
  return (yawDeg * Math.PI) / 180;
}

/** Zoom about a screen point so that the world point under it stays put. */
export function zoomAt(vp, sx, sy, factor, minScale = 0.05, maxScale = 40) {
  const [wx, wy] = screenToWorld(vp, sx, sy);
  const scale = Math.min(maxScale, Math.max(minScale, vp.scale * factor));
  const next = { ...vp, scale };
  next.cx = wx - (sx - vp.width / 2) / scale;
  next.cy = wy - (sy - vp.height / 2) / scale;
  return next;
}

export function fitBounds(vp, minX, minY, maxX, maxY, margin = 0.9) {
  const w = Math.max(maxX - minX, 1), h = Math.max(maxY - minY, 1);
  return { ...vp, cx: (minX + maxX) / 2, cy: (minY + maxY) / 2, scale: Math.min(vp.width / w, vp.height / h) * margin };
}
