// Small transient messages. DOM only (not unit tested; the browser tests check that errors show up).

let host = null;

function ensureHost() {
  if (host && document.body.contains(host)) return host;
  host = document.createElement('div');
  host.className = 'toasts';
  host.setAttribute('role', 'status');
  host.setAttribute('aria-live', 'polite');
  document.body.appendChild(host);
  return host;
}

export function toast(message, kind = 'info', ms = 6000) {
  const el = document.createElement('div');
  el.className = `toast toast-${kind}`;
  el.textContent = message;
  ensureHost().appendChild(el);
  const remove = () => el.remove();
  el.addEventListener('click', remove);
  if (ms > 0) setTimeout(remove, ms);
  return el;
}

export const toastError = (m) => toast(m, 'error', 10000);
