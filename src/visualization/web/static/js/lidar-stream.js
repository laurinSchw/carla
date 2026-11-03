// Socket of one LiDAR sensor (/ws/lidar/{id}): binary protocol-v3 frames and JSON state messages.
// Reconnects after an unexpected close, but not once the sensor is destroyed or in error; the stream profile
// (points / rate / compression) is sent again after every reconnect.

export class LidarStream {
  constructor(url, { WebSocketImpl = globalThis.WebSocket, setTimeoutImpl = (fn, ms) => setTimeout(fn, ms),
    clearTimeoutImpl = (h) => clearTimeout(h), onFrame = () => {}, onMessage = () => {}, onClose = () => {}, onStatus = () => {},
    retryMs = 1000 } = {}) {
    Object.assign(this, { url, WebSocketImpl, setTimeoutImpl, clearTimeoutImpl, onFrame, onMessage, onClose, onStatus, retryMs });
    this.sock = null;
    this.profile = null;
    this.over = false;               // the sensor is gone: do not reconnect
    this.retry = null;
    this.wantOpen = false;
    this.lastState = null;
  }

  connect() {
    this.wantOpen = true;
    this._open();
  }

  _open() {
    const sock = new this.WebSocketImpl(this.url);
    sock.binaryType = 'arraybuffer';
    this.sock = sock;
    sock.onopen = () => {
      if (this.sock !== sock) return;
      this.onStatus('open');
      if (this.profile) this._sendProfile();
    };
    sock.onmessage = (ev) => {
      if (this.sock !== sock) return;
      if (typeof ev.data === 'string') {
        let m;
        try { m = JSON.parse(ev.data); } catch { return; }
        if (m.type === 'state') {
          this.lastState = m.state;
          if (m.state === 'destroyed' || m.state === 'error') this.over = true;
        }
        if (m.type === 'error') this.over = true;
        this.onMessage(m);
      } else {
        this.onFrame(ev.data);
      }
    };
    sock.onclose = () => {
      if (this.sock !== sock) return;
      this.sock = null;
      if (!this.wantOpen || this.over) {
        this.onStatus('closed');
        this.onClose({ reason: this.lastState || 'closed' });
        return;
      }
      this.onStatus('reconnecting');
      this.retry = this.setTimeoutImpl(() => this._open(), this.retryMs);
    };
    sock.onerror = () => {};
  }

  setProfile(profile) {
    this.profile = profile;
    if (this.sock && this.sock.readyState === 1) this._sendProfile();
  }

  _sendProfile() {
    this.sock.send(JSON.stringify({ op: 'profile', ...this.profile }));
  }

  close() {
    this.wantOpen = false;
    this.clearTimeoutImpl(this.retry);
    if (this.sock) { const s = this.sock; this.sock = null; s.close(1000); }
    this.onStatus('closed');
  }
}
