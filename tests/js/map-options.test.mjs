import test from 'node:test';
import assert from 'node:assert/strict';
import { mapOptions, FALLBACK_MAPS } from '../../src/visualization/web/static/js/map-options.js';

test('without a server list the fallback maps are offered, first entry keeps the current map', () => {
  const o = mapOptions(null);
  assert.equal(o[0].value, '');
  assert.deepEqual(o.slice(1).map((x) => x.value), [...FALLBACK_MAPS].sort());
  assert.equal(o.filter((x) => x.selected).length, 1);
  assert.equal(o[0].selected, true);
});

test('the server list is used, paths are cut, duplicates dropped, the loaded map is marked', () => {
  const o = mapOptions(['/Game/Carla/Maps/Town03', 'Town03', 'Town10HD_Opt'], 'Carla/Maps/Town10HD_Opt');
  assert.deepEqual(o.slice(1).map((x) => x.value), ['Town03', 'Town10HD_Opt']);
  assert.match(o[0].label, /Town10HD_Opt/);
  assert.match(o[2].label, /loaded/);
});

test('a previous choice stays selected', () => {
  const o = mapOptions(['Town01', 'Town02'], '', 'Town02');
  assert.equal(o.find((x) => x.selected).value, 'Town02');
});
