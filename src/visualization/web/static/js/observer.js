import { createApp, renderTopbar, showFakeBanner } from './app.js';
import { initObserver } from './observer-core.js';

const app = createApp({ role: 'observer' });
renderTopbar(document.getElementById('top'), { title: 'Observer', store: app.store });
showFakeBanner(app.api);
const page = initObserver(app, { role: 'observer' });
if (new URLSearchParams(location.search).has('debug')) window.carlaDebug = page;
