// Decode every <name>.bin in a directory with the browser decoder and print one JSON object per file.
// Used by tests/unit/test_lidar_protocol_v3.py to prove Python-encoded bytes decode identically in JS.
import { readFileSync, readdirSync } from 'node:fs';
import { join } from 'node:path';
import { decodeFrameAsync } from '../../src/web/static/js/lidar-decoder.js';

const dir = process.argv[2];
const out = {};
for (const name of readdirSync(dir).filter((f) => f.endsWith('.bin')).sort()) {
  const bytes = readFileSync(join(dir, name));
  const buf = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
  try {
    const { header, points } = await decodeFrameAsync(buf);
    const h = { ...header }; delete h.buffer;
    const p = {}; for (const [k, a] of Object.entries(points)) p[k] = Array.from(a);
    out[name] = { header: h, points: p };
  } catch (e) {
    out[name] = { error: String(e.message) };
  }
}
process.stdout.write(JSON.stringify(out));
