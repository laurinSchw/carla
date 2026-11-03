import test from 'node:test';
import assert from 'node:assert/strict';
import { Api, ApiError, describeDetail } from '../../src/web/static/js/api.js';

function fakeFetch(responses) {
  const calls = [];
  const f = async (url, init) => {
    calls.push({ url, init });
    const r = responses.shift();
    if (r instanceof Error) throw r;
    return { ok: r.status < 400, status: r.status, text: async () => (r.body === undefined ? '' : typeof r.body === 'string' ? r.body : JSON.stringify(r.body)) };
  };
  f.calls = calls;
  return f;
}

test('sends credentials as headers and JSON bodies', async () => {
  const f = fakeFetch([{ status: 200, body: { ok: 1 } }]);
  const api = new Api({ fetchImpl: f, credentials: () => ({ adminToken: 'A', sessionToken: 'S' }) });
  assert.deepEqual(await api.post('/api/x', { a: 1 }), { ok: 1 });
  const { url, init } = f.calls[0];
  assert.equal(url, '/api/x'); assert.equal(init.method, 'POST');
  assert.equal(init.headers['X-Admin-Token'], 'A'); assert.equal(init.headers['X-Session-Token'], 'S');
  assert.equal(init.headers['Content-Type'], 'application/json'); assert.equal(init.body, '{"a":1}');
});

test('GET has no body and no content type; no credentials: no headers', async () => {
  const f = fakeFetch([{ status: 200, body: [1] }]);
  const api = new Api({ fetchImpl: f });
  await api.get('/api/y');
  assert.equal(f.calls[0].init.body, undefined);
  assert.deepEqual(f.calls[0].init.headers, {});
});

test('errors are typed, carry the server text and call onError once', async () => {
  const seen = [];
  const api = new Api({ fetchImpl: fakeFetch([{ status: 403, body: { detail: 'Invalid admin token' } }]), onError: (e) => seen.push(e) });
  await assert.rejects(() => api.get('/admin/status'), (e) => e instanceof ApiError && e.status === 403 && e.message === 'Invalid admin token');
  assert.equal(seen.length, 1);
});

test('quiet requests do not call onError', async () => {
  const seen = [];
  const api = new Api({ fetchImpl: fakeFetch([{ status: 404, body: { detail: 'no' } }]), onError: (e) => seen.push(e) });
  await assert.rejects(() => api.get('/x', { quiet: true }));
  assert.equal(seen.length, 0);
});

test('network failures become ApiError(0)', async () => {
  const api = new Api({ fetchImpl: fakeFetch([new TypeError('failed')]), onError: () => {} });
  await assert.rejects(() => api.get('/x'), (e) => e.status === 0 && /network error/.test(e.message));
});

test('describeDetail understands FastAPI validation errors and the LiDAR budget body', () => {
  assert.equal(describeDetail({ detail: [{ loc: ['body', 'config', 'channels'], msg: 'Input should be >= 1' }] }, 422), 'config.channels: Input should be >= 1');
  assert.match(describeDetail({ error: 'lidar_budget_exceeded', reason: 'max_sensors', requested_pps: 5, available_pps: 0 }, 409), /budget exceeded \(max_sensors\).*5.*0 available/);
  assert.equal(describeDetail({ error: 'forbidden', message: 'observers cannot create LiDAR sensors' }, 403), 'observers cannot create LiDAR sensors');
  assert.equal(describeDetail(null, 500), 'HTTP 500');
  assert.equal(describeDetail('plain', 500), 'plain');
});

test('empty body responses are null', async () => {
  const api = new Api({ fetchImpl: fakeFetch([{ status: 204 }]) });
  assert.equal(await api.delete('/x'), null);
});
