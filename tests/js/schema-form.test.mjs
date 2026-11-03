import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { fieldsFromJsonSchema, fieldsFromLidarSchema, validateField, valuesToConfig, configToValues, validateAll } from '../../src/web/static/js/schema-form.js';

const dir = process.env.JS_FIXTURES;
const world = dir ? JSON.parse(readFileSync(join(dir, 'world_config_schema.json'), 'utf8')) : null;
const lidar = dir ? JSON.parse(readFileSync(join(dir, 'lidar_schema.json'), 'utf8')) : null;
const byName = (fields) => Object.fromEntries(fields.map((f) => [f.name, f]));

test('pydantic schema: kinds, bounds, nullable and required', { skip: !world }, () => {
  const f = byName(fieldsFromJsonSchema(world));
  assert.equal(f.host.required, true);
  assert.equal(f.fixed_delta_seconds.kind, 'float');
  assert.equal(f.lidar_max_sensors.kind, 'int'); assert.equal(f.lidar_max_sensors.max, 64); assert.equal(f.lidar_max_sensors.min, 0);
  assert.equal(f.v2x_enabled.kind, 'bool');
  assert.equal(f.awareness_range_m.nullable, true); assert.equal(f.awareness_range_m.min, 10); assert.equal(f.awareness_range_m.max, 500);
  assert.equal(f.rsu_max_count.nullable, true);
  assert.equal(f.v2x_update_rate_hz.exclusiveMin, 0);
  assert.equal(f.session_disconnect_grace_s.default, 30);
});

test('lidar schema: only the attributes valid for a sensor type', { skip: !lidar }, () => {
  const sem = byName(fieldsFromLidarSchema(lidar, 'ray_cast_semantic'));
  const ray = byName(fieldsFromLidarSchema(lidar, 'ray_cast'));
  assert.ok(sem.channels && sem['mount.z_m'] && !sem.noise_stddev);
  assert.ok(ray.noise_stddev && ray.noise_stddev.nullable);
  assert.equal(sem.channels.kind, 'int'); assert.equal(sem.range_m.unit, 'm');
  assert.throws(() => fieldsFromLidarSchema(lidar, 'nope'));
});

test('validation: ranges, integers, required, exclusive minimum', { skip: !world }, () => {
  const f = byName(fieldsFromJsonSchema(world));
  assert.equal(validateField(f.lidar_max_sensors, '4'), null);
  assert.match(validateField(f.lidar_max_sensors, '65'), /at most 64/);
  assert.match(validateField(f.lidar_max_sensors, '-1'), /at least 0/);
  assert.match(validateField(f.lidar_max_sensors, '2.5'), /whole number/);
  assert.match(validateField(f.lidar_max_sensors, 'abc'), /not a number/);
  assert.match(validateField(f.v2x_update_rate_hz, '0'), /greater than 0/);
  assert.equal(validateField(f.awareness_range_m, ''), null, 'nullable: empty means no value');
  assert.equal(validateField(f.host, ''), 'required');
});

test('values <-> nested config; empty optional fields are left out', { skip: !lidar }, () => {
  const fields = fieldsFromLidarSchema(lidar, 'ray_cast');
  const cfg = valuesToConfig(fields, { channels: '64', 'mount.z_m': '2.5', 'mount.yaw_deg': '0', noise_stddev: '', range_m: '80.5' });
  assert.deepEqual(cfg, { channels: 64, mount: { z_m: 2.5, yaw_deg: 0 }, range_m: 80.5 });
  const vals = configToValues(fields, { channels: 16, mount: { x_m: 1 } });
  assert.equal(vals.channels, 16); assert.equal(vals['mount.x_m'], 1); assert.equal(vals['mount.y_m'], 0);
});

test('validateAll collects every error', { skip: !lidar }, () => {
  const fields = fieldsFromLidarSchema(lidar, 'ray_cast_semantic');
  const e = validateAll(fields, { channels: '0', range_m: '1000', points_per_second: '10000' });
  assert.ok(e.channels && e.range_m && !e.points_per_second);
});

test('fields carry whether they are hot or need a restart, and when a hot change takes effect', { skip: !world }, () => {
  const f = byName(fieldsFromJsonSchema(world));
  assert.equal(f.awareness_range_m.apply, 'hot'); assert.equal(f.awareness_range_m.effect, 'next V2X update');
  assert.equal(f.fixed_delta_seconds.apply, 'restart'); assert.equal(f.fixed_delta_seconds.effect, '');
  assert.ok(Object.values(f).every((x) => x.apply === 'hot' || x.apply === 'restart'));
});
