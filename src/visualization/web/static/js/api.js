// fetch wrapper: credentials as headers, JSON in and out, typed errors.

export class ApiError extends Error {
  constructor(status, detail, body) {
    super(typeof detail === 'string' ? detail : `HTTP ${status}`);
    this.status = status;
    this.detail = detail;
    this.body = body;
  }
}

export class Api {
  /**
   * @param {object} o
   * @param {() => {adminToken?: string, sessionToken?: string}} [o.credentials]
   * @param {typeof fetch} [o.fetchImpl]
   * @param {(e: ApiError) => void} [o.onError] called for every failed request (e.g. to show a toast)
   */
  constructor({ credentials = () => ({}), fetchImpl = globalThis.fetch?.bind(globalThis), onError = () => {}, base = '' } = {}) {
    this.credentials = credentials;
    this.fetchImpl = fetchImpl;
    this.onError = onError;
    this.base = base;
  }

  headers(extra = {}) {
    const c = this.credentials() || {};
    const h = { ...extra };
    if (c.adminToken) h['X-Admin-Token'] = c.adminToken;
    if (c.sessionToken) h['X-Session-Token'] = c.sessionToken;
    return h;
  }

  async request(method, path, body, { quiet = false, headers = {} } = {}) {
    const init = { method, headers: this.headers(headers) };
    if (body !== undefined) {
      init.headers['Content-Type'] = 'application/json';
      init.body = JSON.stringify(body);
    }
    let res;
    try {
      res = await this.fetchImpl(this.base + path, init);
    } catch (e) {
      const err = new ApiError(0, `network error: ${e.message}`, null);
      if (!quiet) this.onError(err);
      throw err;
    }
    let data = null;
    const text = await res.text();
    if (text) {
      try {
        data = JSON.parse(text);
      } catch {
        data = text;
      }
    }
    if (!res.ok) {
      const err = new ApiError(res.status, describeDetail(data, res.status), data);
      if (!quiet) this.onError(err);
      throw err;
    }
    return data;
  }

  get(path, opts) { return this.request('GET', path, undefined, opts); }
  post(path, body, opts) { return this.request('POST', path, body ?? {}, opts); }
  put(path, body, opts) { return this.request('PUT', path, body, opts); }
  delete(path, opts) { return this.request('DELETE', path, undefined, opts); }
}

/** FastAPI errors: {detail: "text"} or {detail: [{loc, msg}]} or our own {error, message}. */
export function describeDetail(data, status) {
  if (data && typeof data === 'object') {
    if (typeof data.detail === 'string') return data.detail;
    if (Array.isArray(data.detail)) return data.detail.map((d) => `${(d.loc || []).slice(1).join('.')}: ${d.msg}`).join('; ');
    if (data.error === 'lidar_budget_exceeded') {
      return `LiDAR budget exceeded (${data.reason}): requested ${data.requested_pps} points/s, ${data.available_pps} available`;
    }
    if (data.message) return data.message;
    if (data.error) return String(data.error);
  }
  if (typeof data === 'string' && data) return data;
  return `HTTP ${status}`;
}
