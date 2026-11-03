import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { DENM_CAUSES } from '../../src/web/static/js/denm-causes.js';

const dir = process.env.JS_FIXTURES;

test('the cause code list equals the server enum', { skip: !dir }, () => {
  const py = JSON.parse(readFileSync(join(dir, 'denm_causes.json'), 'utf8'));
  assert.deepEqual(DENM_CAUSES, py);
  assert.ok(DENM_CAUSES.length > 10 && DENM_CAUSES.some(([c, n]) => c === 2 && n === 'ACCIDENT'));
});
