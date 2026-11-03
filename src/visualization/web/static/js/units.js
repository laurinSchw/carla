// Display formatting only. Values are rounded here and nowhere else; hovering or the raw view shows the full value.
// A missing value is shown as an em dash with the reason in a tooltip (never a made-up number).

export const DASH = '—';

const isNum = (v) => typeof v === 'number' && Number.isFinite(v);

export function num(v, digits = 2) {
  return isNum(v) ? v.toFixed(digits) : DASH;
}

export function withUnit(v, unit, digits = 2) {
  return isNum(v) ? `${v.toFixed(digits)} ${unit}` : DASH;
}

export const metres = (v, d = 1) => withUnit(v, 'm', d);
export const seconds = (v, d = 2) => withUnit(v, 's', d);
export const dBm = (v, d = 1) => withUnit(v, 'dBm', d);
export const dB = (v, d = 1) => withUnit(v, 'dB', d);
export const degrees = (v, d = 1) => withUnit(v, '°', d).replace(' ', '');
export const hertz = (v, d = 1) => withUnit(v, 'Hz', d);

/** Speed in m/s with km/h as the secondary unit. */
export function speed(mps, d = 1) {
  return isNum(mps) ? `${mps.toFixed(d)} m/s (${(mps * 3.6).toFixed(0)} km/h)` : DASH;
}

export function percent(fraction, d = 1) {
  return isNum(fraction) ? `${(fraction * 100).toFixed(d)} %` : DASH;
}

export function bytes(n) {
  if (!isNum(n)) return DASH;
  if (n < 1024) return `${n.toFixed(0)} B`;
  if (n < 1024 * 1024) return `${(n / 1024).toFixed(1)} kB`;
  return `${(n / 1024 / 1024).toFixed(2)} MB`;
}

export function count(n) {
  return isNum(n) ? Math.round(n).toLocaleString('en-US') : DASH;
}

/** Age in seconds of simulation time: "0.50 s" / "2.0 s" / "1 min 5 s". */
export function age(s) {
  if (!isNum(s)) return DASH;
  if (s < 10) return `${s.toFixed(2)} s`;
  if (s < 60) return `${s.toFixed(1)} s`;
  return `${Math.floor(s / 60)} min ${Math.round(s % 60)} s`;
}

/** Frame stamp shown with every live panel. */
export function stamp(frame, simTime) {
  if (!isNum(frame)) return DASH;
  return `frame ${frame}` + (isNum(simTime) ? ` · t=${simTime.toFixed(2)} s` : '');
}

/** Data older than two update periods is stale. */
export function isStale(ageS, updatePeriodS) {
  return isNum(ageS) && isNum(updatePeriodS) && ageS > 2 * updatePeriodS;
}

/** {text, title}: the dash plus the reason when the value is missing. */
export function orDash(value, format, reason) {
  if (value === null || value === undefined || (typeof value === 'number' && !Number.isFinite(value))) {
    return { text: DASH, title: reason || 'not available' };
  }
  return { text: format ? format(value) : String(value), title: String(value) };
}

export function escapeHtml(s) {
  return String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}

/** CSV cell per RFC 4180. */
export function csvCell(v) {
  if (v === null || v === undefined) return '';
  const s = String(v);
  return /[",\n\r]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
}
