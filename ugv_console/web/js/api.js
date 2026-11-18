// Talking to the console program on this computer: token, commands, live event stream, clock.

const params = new URLSearchParams((location.hash || '').replace(/^#/, '') || location.search.replace(/^\?/, ''));
let token = params.get('k') || '';
try {
  if (token) sessionStorage.setItem('ugv-console-token', token);
  else token = sessionStorage.getItem('ugv-console-token') || '';
} catch (e) { /* storage blocked: the token from the link still works for this page load */ }
if (params.get('k')) history.replaceState(null, '', location.pathname);   // keep the token out of the address bar

export const TOKEN = token;

export async function post(cmd, args = {}) {
  try {
    const response = await fetch('/api/cmd', {
      method: 'POST', cache: 'no-store',
      headers: {'Content-Type': 'application/json', 'X-Console-Token': token},
      body: JSON.stringify({cmd, args}),
    });
    const type = response.headers.get('Content-Type') || '';
    if (type.includes('application/json')) return await response.json();
    return {ok: false, level: 'error', message: (await response.text()) || ('HTTP ' + response.status)};
  } catch (e) {
    return {ok: false, level: 'error', offline: true, message: 'The console program is not answering.'};
  }
}

// Server (Python monotonic) time <-> this page's clock. The smallest observed difference is the
// offset plus the shortest delivery time; it creeps up slowly so it can follow clock drift.
export const clock = {
  offset: null,
  observe(serverTime) {
    const d = performance.now() / 1000 - serverTime;
    this.offset = this.offset === null ? d : Math.min(this.offset + 0.0004, d);
  },
  now() { return performance.now() / 1000 - (this.offset || 0); },
};

export function openEvents(onData, onAlive) {
  let lastData = 0;
  let source = null;
  const connect = () => {
    source = new EventSource('/api/events?k=' + encodeURIComponent(token));
    source.onmessage = (event) => {
      lastData = performance.now();
      let data;
      try { data = JSON.parse(event.data); } catch (e) { return; }
      if (data.fast && typeof data.fast.t === 'number') clock.observe(data.fast.t);
      onAlive(true);
      onData(data);
    };
    source.onerror = () => { /* EventSource retries by itself; the watchdog below reports a real loss */ };
  };
  connect();
  setInterval(() => {
    if (lastData && performance.now() - lastData > 2500) onAlive(false);
    if (!lastData && performance.now() > 6000) onAlive(false);
  }, 500);
  return () => source && source.close();
}

// The newest camera picture: the robot's ('/api/frame') or the drone's ('/api/uav_frame').
export async function fetchFrame(after, path = '/api/frame') {
  const response = await fetch(path + '?after=' + after, {cache: 'no-store', headers: {'X-Console-Token': token}});
  if (response.status !== 200) return null;
  const meta = JSON.parse(response.headers.get('X-Frame') || '{}');
  const bitmap = await createImageBitmap(await response.blob());
  return {meta, bitmap};
}
