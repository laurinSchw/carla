// Shared page bootstrap: credentials, API client, hub socket wired to the store, top bar, theme.

import { Api } from './api.js';
import { HubSocket } from './ws.js';
import { Store } from './store.js';
import { toastError } from './toasts.js';

const KEY_ADMIN = 'carla.adminToken';
const KEY_SESSION = 'carla.session';

function load(storage, key) {
  try {
    return storage.getItem(key);
  } catch {
    return null;                      // storage blocked: the page still works, it just does not remember
  }
}

function save(storage, key, value) {
  try {
    if (value === null) storage.removeItem(key); else storage.setItem(key, value);
  } catch { /* ignore */ }
}

export const credentials = {
  get adminToken() { return load(sessionStorage, KEY_ADMIN) || ''; },
  set adminToken(v) { save(sessionStorage, KEY_ADMIN, v || null); },
  get session() {
    const s = load(sessionStorage, KEY_SESSION);
    try { return s ? JSON.parse(s) : null; } catch { return null; }
  },
  set session(v) { save(sessionStorage, KEY_SESSION, v ? JSON.stringify(v) : null); },
};

export function wsUrl(path = '/ws') {
  return `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}${path}`;
}

export function createApp({ role = 'observer' } = {}) {
  const api = new Api({
    credentials: () => ({
      adminToken: role === 'admin' ? credentials.adminToken : '',
      sessionToken: role === 'vehicle' ? (credentials.session || {}).token : '',
    }),
    onError: (e) => toastError(e.message),
  });
  const store = new Store();
  const hub = new HubSocket(wsUrl('/ws'), {
    credentials: () => {
      if (role === 'admin' && credentials.adminToken) return { admin_token: credentials.adminToken };
      if (role === 'vehicle' && credentials.session) return { session_token: credentials.session.token };
      return null;
    },
  });
  hub.onState = (state, detail) => store.setConnection(state, detail);
  hub.onError = (m) => {
    store.lastError = m;
    toastError(`${m.code}: ${m.message}`);
  };
  const app = {
    role, api, store, hub,
    /** Subscribe a topic and feed every envelope into the store. */
    watch(topic, opts = {}, extra) {
      hub.subscribe(topic, opts, (data, env, gap) => {
        store.put(env);
        if (extra) extra(data, env, gap);
      });
    },
  };
  return app;
}

const NAV = [['/', 'Status'], ['/observer', 'Observer'], ['/vehicle', 'Vehicle'], ['/admin', 'Admin']];

export function renderTopbar(el, { title, store }) {
  el.className = 'topbar';
  el.textContent = '';
  const logo = document.createElement('img');
  logo.src = '/static/img/logo.png';
  logo.alt = '';
  logo.className = 'logo';
  logo.onerror = () => logo.remove();
  const h = document.createElement('strong');
  h.textContent = title;
  const nav = document.createElement('nav');
  for (const [href, label] of NAV) {
    const a = document.createElement('a');
    a.href = href;
    a.textContent = label;
    if (location.pathname === href) a.setAttribute('aria-current', 'page');
    nav.appendChild(a);
  }
  const conn = document.createElement('span');
  conn.className = 'conn';
  conn.id = 'conn';
  conn.dataset.state = 'closed';
  conn.textContent = 'not connected';
  conn.setAttribute('role', 'status');
  const theme = document.createElement('button');
  theme.textContent = 'Theme';
  theme.title = 'Switch between light and dark';
  theme.addEventListener('click', () => {
    const cur = document.documentElement.dataset.theme
      || (matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light');
    const next = cur === 'dark' ? 'light' : 'dark';
    document.documentElement.dataset.theme = next;
    save(localStorage, 'carla.theme', next);
  });
  el.append(logo, h, nav, conn, theme);
  const saved = load(localStorage, 'carla.theme');
  if (saved) document.documentElement.dataset.theme = saved;
  if (store) {
    store.onConnection(({ state, detail }) => {
      conn.dataset.state = state;
      conn.textContent = detail ? `${state} (${detail})` : state;
    });
  }
}

/** Shows a banner when the server runs on the synthetic world (never mistake it for CARLA). */
export async function showFakeBanner(api) {
  try {
    const s = await api.get('/api/status', { quiet: true });
    if (s && s.fake) {
      const b = document.createElement('div');
      b.className = 'fake-banner';
      b.textContent = 'FAKE WORLD: synthetic data from src/testing/fake_carla, not a simulator';
      document.body.prepend(b);
    }
    return s;
  } catch {
    return null;
  }
}
