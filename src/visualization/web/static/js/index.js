import { createApp, renderTopbar, showFakeBanner } from './app.js';
import * as u from './units.js';

const app = createApp({ role: 'observer' });
renderTopbar(document.getElementById('top'), { title: 'CARLA V2X Server', store: app.store });
showFakeBanner(app.api);

const el = document.getElementById('status');

function row(k, v) {
  const dt = document.createElement('dt');
  dt.textContent = k;
  const dd = document.createElement('dd');
  dd.textContent = v;
  el.append(dt, dd);
}

async function refresh() {
  try {
    const s = await app.api.get('/api/status', { quiet: true });
    el.textContent = '';
    if (!s.connected) {
      row('CARLA', 'not connected (an admin connects it from the Admin page)');
    } else {
      row('CARLA', 'connected' + (s.fake ? ' (FAKE world)' : ''));
      row('Map', s.map);
      row('Frame', `${s.frame}  ·  sim time ${u.seconds(s.sim_time)}`);
      row('Real-time factor', u.num(s.real_time_factor, 2));
      row('Step', u.seconds(s.fixed_delta_seconds, 3));
      row('V2X', s.v2x_enabled ? 'enabled' : 'disabled');
      row('Sessions', String(s.active_sessions));
      row('Observer access', s.observer_public ? 'public' : 'disabled (token needed)');
      row('Recording', s.run_active ? 'running' : 'no run active');
    }
    document.getElementById('stamp').textContent = new Date().toISOString().slice(11, 19) + ' UTC';
  } catch (e) {
    el.textContent = '';
    row('Server', `unreachable: ${e.message}`);
  }
}

refresh();
setInterval(refresh, 2000);
