/* LiDAR protocol v3 decoder. Mirrors src/lidar/protocol.py byte for byte (little-endian).
 *
 * Header 48 bytes  <4sHHQddIIIHH : magic 'LIDR', version, flags, frame, sim_time, wall_time,
 *                  num_points, ego_actor_id, sensor_id, point_stride_bytes, field_mask
 * Pose   48 bytes  <6d : location x,y,z (m), rotation pitch,yaw,roll (deg), world frame
 * Points           per field_mask bit: 1 xyz f32x3 | 2 tag u8 | 4 object_idx u32 | 8 intensity f32 | 16 cos_inc f32
 * flags: bit0 zlib-compressed point block, bit1 world frame, bit2 sensor frame
 *
 * Works in browsers (window.LidarDecoder) and Node (module.exports). tests/unit/test_lidar_js_decoder.py
 * checks it against frames produced by the Python encoder.
 */
(function (root) {
  'use strict';
  const HEADER_SIZE = 96;
  const FLAG_ZLIB = 1, FLAG_WORLD = 2, FLAG_SENSOR = 4;
  const F_XYZ = 1, F_TAG = 2, F_OBJ = 4, F_INT = 8, F_COS = 16;

  function parseHeader(buffer) {
    if (buffer.byteLength < HEADER_SIZE) throw new Error('LiDAR frame too short: ' + buffer.byteLength);
    const v = new DataView(buffer);
    const magic = String.fromCharCode(v.getUint8(0), v.getUint8(1), v.getUint8(2), v.getUint8(3));
    if (magic !== 'LIDR') throw new Error('bad magic ' + magic);
    const version = v.getUint16(4, true);
    if (version !== 3) throw new Error('unsupported LiDAR protocol version ' + version);
    const flags = v.getUint16(6, true);
    if (!!(flags & FLAG_WORLD) === !!(flags & FLAG_SENSOR)) throw new Error('bad frame-of-reference flags');
    return {
      version, flags,
      compressed: !!(flags & FLAG_ZLIB),
      sensorFrame: !!(flags & FLAG_SENSOR),
      frame: Number(v.getBigUint64(8, true)),
      simTime: v.getFloat64(16, true),
      wallTime: v.getFloat64(24, true),
      numPoints: v.getUint32(32, true),
      egoActorId: v.getUint32(36, true),
      sensorId: v.getUint32(40, true),
      stride: v.getUint16(44, true),
      fieldMask: v.getUint16(46, true),
      location: [v.getFloat64(48, true), v.getFloat64(56, true), v.getFloat64(64, true)],
      rotation: [v.getFloat64(72, true), v.getFloat64(80, true), v.getFloat64(88, true)], // pitch, yaw, roll
    };
  }

  function strideFor(mask) {
    return (mask & F_XYZ ? 12 : 0) + (mask & F_TAG ? 1 : 0) + (mask & F_OBJ ? 4 : 0) + (mask & F_INT ? 4 : 0) + (mask & F_COS ? 4 : 0);
  }

  function readPoints(header, body) {
    const n = header.numPoints, mask = header.fieldMask;
    if (!(mask & F_XYZ) || (mask & ~31)) throw new Error('invalid field_mask ' + mask);
    if (strideFor(mask) !== header.stride) throw new Error('stride does not match field_mask');
    if (body.byteLength !== n * header.stride) throw new Error('point block size mismatch');
    const v = new DataView(body.buffer, body.byteOffset, body.byteLength);
    const out = { x: new Float32Array(n), y: new Float32Array(n), z: new Float32Array(n),
                  tag: mask & F_TAG ? new Uint8Array(n) : null,
                  objectIdx: mask & F_OBJ ? new Uint32Array(n) : null,
                  intensity: mask & F_INT ? new Float32Array(n) : null,
                  cosInc: mask & F_COS ? new Float32Array(n) : null };
    for (let i = 0, o = 0; i < n; i++) {
      out.x[i] = v.getFloat32(o, true); out.y[i] = v.getFloat32(o + 4, true); out.z[i] = v.getFloat32(o + 8, true);
      o += 12;
      if (out.tag) { out.tag[i] = v.getUint8(o); o += 1; }
      if (out.objectIdx) { out.objectIdx[i] = v.getUint32(o, true); o += 4; }
      if (out.intensity) { out.intensity[i] = v.getFloat32(o, true); o += 4; }
      if (out.cosInc) { out.cosInc[i] = v.getFloat32(o, true); o += 4; }
    }
    return out;
  }

  /** Synchronous decode of an UNcompressed frame; throws for a compressed one (use decode()). */
  function decodeSync(buffer) {
    const header = parseHeader(buffer);
    if (header.compressed) throw new Error('compressed frame: use decode()');
    return Object.assign({ header }, readPoints(header, new Uint8Array(buffer, HEADER_SIZE)));
  }

  /** Decode any frame. Compressed frames need DecompressionStream (browsers) or Node's zlib. */
  async function decode(buffer) {
    const header = parseHeader(buffer);
    let body = new Uint8Array(buffer, HEADER_SIZE);
    if (header.compressed) body = await inflate(body);
    return Object.assign({ header }, readPoints(header, body));
  }

  async function inflate(bytes) {
    if (typeof DecompressionStream !== 'undefined') {
      const stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream('deflate'));
      return new Uint8Array(await new Response(stream).arrayBuffer());
    }
    if (typeof require === 'function') return new Uint8Array(require('zlib').inflateSync(bytes));
    throw new Error('no zlib inflate available');
  }

  const api = { HEADER_SIZE, parseHeader, decodeSync, decode };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.LidarDecoder = api;
})(typeof self !== 'undefined' ? self : this);
