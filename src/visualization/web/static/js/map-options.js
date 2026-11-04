// Options of the map selects (Connection card and connect dialog). Pure, so it is tested without a DOM.

// Maps of a default CARLA 0.9.16 install; shown until the server tells us what it really has (GET /admin/maps).
export const FALLBACK_MAPS = ['Town01', 'Town01_Opt', 'Town02', 'Town02_Opt', 'Town03', 'Town03_Opt', 'Town04', 'Town04_Opt',
  'Town05', 'Town05_Opt', 'Town10HD', 'Town10HD_Opt'];

const lastPart = (m) => String(m || '').split('/').filter(Boolean).pop() || '';

/**
 * @param {string[]|null} available  names from the server, or null/empty to use the fallback list
 * @param {string} current           currently loaded map (any path form) or ''
 * @param {string} selected          value to keep selected (e.g. what the user chose before the list was refreshed)
 * @returns {{value: string, label: string, selected: boolean}[]}  first entry = keep the server's map (value '')
 */
export function mapOptions(available, current = '', selected = '') {
  const names = [...new Set((available && available.length ? available : FALLBACK_MAPS).map(lastPart).filter(Boolean))].sort();
  const cur = lastPart(current);
  const want = selected || '';
  const out = [{ value: '', label: cur ? `keep current (${cur})` : "keep the server's current map", selected: want === '' }];
  for (const n of names) out.push({ value: n, label: n === cur ? `${n}  (loaded)` : n, selected: n === want });
  return out;
}
