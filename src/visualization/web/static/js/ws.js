// Client for the multiplexed hub socket (/ws): auth as the first message, subscriptions that survive reconnects,
// exponential backoff with jitter, sequence-gap reporting. Pure of the DOM: the WebSocket class and the timers are
// injected, so the Node tests drive it with a fake socket and a fake clock.

export class HubSocket {
  /**
   * @param {string} url
   * @param {object} o
   * @param {() => ({admin_token?: string, session_token?: string} | null)} [o.credentials]
   * @param {typeof WebSocket} [o.WebSocketImpl]
   * @param {(fn: Function, ms: number) => any} [o.setTimeoutImpl]
   * @param {(h: any) => void} [o.clearTimeoutImpl]
   * @param {() => number} [o.random]
   */
  constructor(url, { credentials = () => null, WebSocketImpl = globalThis.WebSocket,
    // wrapped: calling window.setTimeout as a method of another object throws "Illegal invocation"
    setTimeoutImpl = (fn, ms) => setTimeout(fn, ms), clearTimeoutImpl = (h) => clearTimeout(h), random = () => Math.random(), baseDelayMs = 500, maxDelayMs = 15000, pingEveryMs = 15000 } = {}) {
    this.url = url;
    this.credentials = credentials;
    this.WebSocketImpl = WebSocketImpl;
    this.setTimeoutImpl = setTimeoutImpl;
    this.clearTimeoutImpl = clearTimeoutImpl;
    this.random = random;
    this.baseDelayMs = baseDelayMs;
    this.maxDelayMs = maxDelayMs;
    this.pingEveryMs = pingEveryMs;
    this.subs = new Map();            // topic -> {max_hz, filter, handler}
    this.sock = null;
    this.state = 'closed';
    this.attempt = 0;
    this.retryTimer = null;
    this.pingTimer = null;
    this.wantOpen = false;
    this.role = 'observer';
    this.onState = () => {};
    this.onError = () => {};
    this.onControl = () => {};
    this.lastSeq = new Map();
    this.skipped = new Map();
  }

  _setState(state, detail = '') {
    this.state = state;
    this.onState(state, detail);
  }

  connect() {
    this.wantOpen = true;
    this._open();
  }

  close() {
    this.wantOpen = false;
    this.clearTimeoutImpl(this.retryTimer);
    this.clearTimeoutImpl(this.pingTimer);
    if (this.sock) this.sock.close(1000);
    this.sock = null;
    this._setState('closed');
  }

  _open() {
    this._setState(this.attempt === 0 ? 'connecting' : 'reconnecting', this.attempt ? `attempt ${this.attempt + 1}` : '');
    const sock = new this.WebSocketImpl(this.url);
    this.sock = sock;
    sock.onopen = () => {
      if (this.sock !== sock) return;
      this.attempt = 0;
      const cred = this.credentials();
      if (cred) this._send({ op: 'auth', ...cred });
      for (const [topic, s] of this.subs) this._sendSubscribe(topic, s);
      this._schedulePing();
      this._setState('open');
    };
    sock.onmessage = (ev) => this._onMessage(ev.data);
    sock.onerror = () => {};
    sock.onclose = (ev) => {
      if (this.sock !== sock) return;
      this.sock = null;
      this.clearTimeoutImpl(this.pingTimer);
      if (!this.wantOpen) return;
      if (ev && ev.code === 4401) {                     // wrong credentials: retrying cannot help
        this.wantOpen = false;
        this._setState('closed', 'authentication failed');
        return;
      }
      const delay = this.nextDelay();
      this._setState('reconnecting', `retry in ${(delay / 1000).toFixed(1)} s`);
      this.attempt += 1;
      this.retryTimer = this.setTimeoutImpl(() => this._open(), delay);
    };
  }

  /** Exponential backoff with full jitter in [delay/2, delay]. */
  nextDelay() {
    const cap = Math.min(this.maxDelayMs, this.baseDelayMs * 2 ** this.attempt);
    return Math.round(cap / 2 + (this.random() * cap) / 2);
  }

  _schedulePing() {
    this.clearTimeoutImpl(this.pingTimer);
    this.pingTimer = this.setTimeoutImpl(() => {
      this._send({ op: 'ping' });
      this._schedulePing();
    }, this.pingEveryMs);
  }

  _send(obj) {
    if (this.sock && this.sock.readyState === 1) this.sock.send(JSON.stringify(obj));
  }

  _sendSubscribe(topic, s) {
    const msg = { op: 'subscribe', topic };
    if (s.max_hz !== undefined) msg.max_hz = s.max_hz;
    if (s.filter !== undefined) msg.filter = s.filter;
    this._send(msg);
  }

  /** Subscribe (or replace the subscription of) a topic. The handler gets (data, envelope, skippedSinceLast). */
  subscribe(topic, { max_hz, filter } = {}, handler = () => {}) {
    const s = { max_hz, filter, handler };
    this.subs.set(topic, s);
    this.lastSeq.delete(topic);
    if (this.state === 'open') this._sendSubscribe(topic, s);
  }

  unsubscribe(topic) {
    this.subs.delete(topic);
    this._send({ op: 'unsubscribe', topic });
  }

  _onMessage(raw) {
    let msg;
    try {
      msg = JSON.parse(raw);
    } catch {
      return;
    }
    const topic = msg.topic;
    if (topic === 'error') {
      this.onError(msg);
      return;
    }
    if (topic === 'auth') {
      this.role = msg.data.role;
      this.onControl(msg);
      return;
    }
    if (topic === 'subscribed' || topic === 'unsubscribed' || topic === 'pong') {
      this.onControl(msg);
      return;
    }
    const s = this.subs.get(topic);
    if (!s) return;
    const prev = this.lastSeq.get(topic);
    const gap = typeof prev === 'number' && typeof msg.seq === 'number' && msg.seq > prev + 1 ? msg.seq - prev - 1 : 0;
    if (typeof msg.seq === 'number') this.lastSeq.set(topic, msg.seq);
    if (gap) this.skipped.set(topic, (this.skipped.get(topic) || 0) + gap);
    s.handler(msg.data, msg, gap);
  }
}
