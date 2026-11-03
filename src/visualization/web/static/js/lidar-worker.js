// Decodes and colours LiDAR frames off the main thread. Protocol:
//   in : {type: 'frame', id, buffer: ArrayBuffer (transferred), opts}
//   out: {type: 'frame', id, header, positions, colors (transferred), counts, modes, decodeMs}
//        {type: 'error', id, message}
import { decodeFrameAsync } from './lidar-decoder.js';
import { processFrame } from './lidar-process.js';

self.onmessage = async (ev) => {
  const m = ev.data;
  if (m.type !== 'frame') return;
  const t0 = performance.now();
  try {
    const decoded = await decodeFrameAsync(m.buffer);
    const o = { ...m.opts, lut: m.opts.lut ? new Float32Array(m.opts.lut) : undefined };
    let r;
    try {
      r = processFrame(decoded, o);
    } catch (e) {
      // the requested colour mode does not exist for this frame: say so, never substitute silently
      self.postMessage({ type: 'error', id: m.id, message: String(e.message), modes: Object.keys(decoded.points) });
      return;
    }
    self.postMessage({ type: 'frame', id: m.id, header: r.header, positions: r.positions, colors: r.colors, counts: r.counts,
      modes: r.modes, decodeMs: performance.now() - t0 }, [r.positions.buffer, r.colors.buffer]);
  } catch (e) {
    self.postMessage({ type: 'error', id: m.id, message: String(e.message) });
  }
};
