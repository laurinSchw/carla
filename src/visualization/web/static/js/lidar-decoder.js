// LiDAR binary protocol v3 decoder (mirror of src/lidar/protocol.py - keep in step with it).
// Pure ES module, no dependencies: runs in the browser (a Web Worker) and in Node.
//
// Header (48 bytes, little-endian): magic 'LIDR', u16 version=3, u16 flags, u64 frame, f64 sim_time,
// f64 wall_time_unix, u32 num_points, u32 ego_actor_id, u32 sensor_id, u16 point_stride_bytes, u16 field_mask;
// then the sensor pose: 6 x f64 = x, y, z (m), pitch, yaw, roll (deg); then the packed points.

export const VERSION = 3;
export const HEADER_SIZE = 48;
export const POSE_SIZE = 48;
export const PREAMBLE_SIZE = HEADER_SIZE + POSE_SIZE;

export const FLAG_ZLIB = 1;
export const FLAG_WORLD_FRAME = 2;
export const FLAG_SENSOR_FRAME = 4;

export const FIELD_XYZ = 1;
export const FIELD_TAG_U8 = 2;
export const FIELD_TAG_U32 = 4;
export const FIELD_OBJECT_IDX = 8;
export const FIELD_INTENSITY = 16;
export const FIELD_COS_INC = 32;

export class ProtocolError extends Error {}

function strideFor(mask) {
  let s = 0;
  if (mask & FIELD_XYZ) s += 12;
  if (mask & FIELD_TAG_U8) s += 1;
  if (mask & FIELD_TAG_U32) s += 4;
  if (mask & FIELD_OBJECT_IDX) s += 4;
  if (mask & FIELD_INTENSITY) s += 4;
  if (mask & FIELD_COS_INC) s += 4;
  return s;
}

/** Parse the fixed header and pose. Throws ProtocolError on anything malformed. */
export function parseHeader(buffer) {
  const buf = buffer instanceof ArrayBuffer ? buffer : buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength);
  if (buf.byteLength < PREAMBLE_SIZE) throw new ProtocolError('buffer shorter than the v3 header');
  const v = new DataView(buf);
  const magic = String.fromCharCode(v.getUint8(0), v.getUint8(1), v.getUint8(2), v.getUint8(3));
  if (magic !== 'LIDR') throw new ProtocolError('bad magic');
  const version = v.getUint16(4, true);
  if (version !== VERSION) throw new ProtocolError(`unsupported protocol version ${version}`);
  const flags = v.getUint16(6, true);
  const worldFrame = (flags & FLAG_WORLD_FRAME) !== 0;
  const sensorFrame = (flags & FLAG_SENSOR_FRAME) !== 0;
  if (worldFrame === sensorFrame) throw new ProtocolError('exactly one of the world-frame / sensor-frame flags must be set');
  const mask = v.getUint16(46, true);
  const stride = v.getUint16(44, true);
  if (!(mask & FIELD_XYZ) || ((mask & FIELD_TAG_U8) && (mask & FIELD_TAG_U32))) throw new ProtocolError('invalid field_mask');
  if (strideFor(mask) !== stride) throw new ProtocolError('point_stride_bytes does not match field_mask');
  return {
    buffer: buf,
    frame: Number(v.getBigUint64(8, true)),
    simTime: v.getFloat64(16, true),
    wallTimeUnix: v.getFloat64(24, true),
    numPoints: v.getUint32(32, true),
    egoActorId: v.getUint32(36, true),
    sensorId: v.getUint32(40, true),
    stride, fieldMask: mask, compressed: (flags & FLAG_ZLIB) !== 0, sensorFrame,
    location: [v.getFloat64(48, true), v.getFloat64(56, true), v.getFloat64(64, true)],
    rotationPYR: [v.getFloat64(72, true), v.getFloat64(80, true), v.getFloat64(88, true)],
  };
}

function readPoints(header, body) {
  const n = header.numPoints, stride = header.stride, mask = header.fieldMask;
  if (body.byteLength !== n * stride) throw new ProtocolError(`point data is ${body.byteLength} bytes, expected ${n * stride}`);
  const v = new DataView(body.buffer || body, body.byteOffset || 0, body.byteLength);
  const out = { x: new Float32Array(n), y: new Float32Array(n), z: new Float32Array(n) };
  const tagU8 = (mask & FIELD_TAG_U8) !== 0, tagU32 = (mask & FIELD_TAG_U32) !== 0;
  if (tagU8) out.tag = new Uint8Array(n);
  if (tagU32) out.tag = new Uint32Array(n);
  if (mask & FIELD_OBJECT_IDX) out.objectIdx = new Uint32Array(n);
  if (mask & FIELD_INTENSITY) out.intensity = new Float32Array(n);
  if (mask & FIELD_COS_INC) out.cosInc = new Float32Array(n);
  for (let i = 0, o = 0; i < n; i++, o += stride) {
    let p = o;
    out.x[i] = v.getFloat32(p, true); out.y[i] = v.getFloat32(p + 4, true); out.z[i] = v.getFloat32(p + 8, true); p += 12;
    if (tagU8) { out.tag[i] = v.getUint8(p); p += 1; }
    if (tagU32) { out.tag[i] = v.getUint32(p, true); p += 4; }
    if (mask & FIELD_OBJECT_IDX) { out.objectIdx[i] = v.getUint32(p, true); p += 4; }
    if (mask & FIELD_INTENSITY) { out.intensity[i] = v.getFloat32(p, true); p += 4; }
    if (mask & FIELD_COS_INC) { out.cosInc[i] = v.getFloat32(p, true); p += 4; }
  }
  return out;
}

/** Decode an uncompressed frame synchronously. Use decodeFrameAsync when the frame may be zlib-compressed. */
export function decodeFrame(buffer) {
  const header = parseHeader(buffer);
  if (header.compressed) throw new ProtocolError('compressed frame: use decodeFrameAsync');
  return { header, points: readPoints(header, new Uint8Array(header.buffer, PREAMBLE_SIZE)) };
}

/** Decode any frame; zlib data is inflated with the platform DecompressionStream('deflate'). */
export async function decodeFrameAsync(buffer) {
  const header = parseHeader(buffer);
  let body = new Uint8Array(header.buffer, PREAMBLE_SIZE);
  if (header.compressed) {
    const stream = new Blob([body]).stream().pipeThrough(new DecompressionStream('deflate'));
    body = new Uint8Array(await new Response(stream).arrayBuffer());
  }
  return { header, points: readPoints(header, body) };
}
