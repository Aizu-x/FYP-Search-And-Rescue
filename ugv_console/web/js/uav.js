// The UAV section: the drone's camera picture, battery and altitude, and flying it with the keyboard.
//
// Keys: W A S D move, arrow up / down change height, arrow left / right turn. The page only reports WHICH keys
// are held (at once when that changes, and several times a second while one is down). The console program turns
// that into stick values, and lets the drone hover as soon as the reports stop. Every key is released when the
// window loses focus, a dialog opens or the section is left.

import {fetchFrame, post} from './api.js';
import {modal, modalOpen, run, showResult, toast} from './ui.js';

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];
const esc = (text) => String(text === null || text === undefined ? '' : text).replace(/[&<>"]/g,
  (c) => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c]));
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const mmss = (s) => `${Math.floor(s / 60)}:${String(Math.round(s) % 60).padStart(2, '0')}`;

// physical keys (the same places on every keyboard layout) -> names the console program knows
const CODES = {KeyW: 'w', KeyA: 'a', KeyS: 's', KeyD: 'd', ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right'};
const WORDS = {w: 'forward', s: 'back', a: 'left', d: 'right', up: 'up', down: 'down', left: 'turn left', right: 'turn right'};
const REPORT_MS = 150;

function typing(target) {
  if (!target || !target.tagName) return false;
  if (target.isContentEditable || target.tagName === 'TEXTAREA' || target.tagName === 'SELECT') return true;
  return target.tagName === 'INPUT' && !['checkbox', 'range', 'radio', 'button'].includes(target.type);
}

export class UavView {
  constructor() {
    this.active = false;                 // the UAV section is on screen
    this.d = null;                       // newest state from the console program
    this.held = new Set(); this.timer = null;
    this.canvas = $('#uav-cam'); this.ctx = this.canvas.getContext('2d');
    this.wantVideo = false; this.running = false; this.last = -1; this.shown = 0;
    this.knownMade = null; this.knownPhotos = null;
    this.bind();
  }

  // ---------------------------------------------------------------- wiring
  bind() {
    $('#btn-uav-connect').addEventListener('click', () => {
      const d = this.d || {};
      run(d.connected || d.state === 'connecting' ? 'uav_disconnect' : 'uav_connect');
    });
    $('#btn-uav-takeoff').addEventListener('click', () => this.takeoff());
    $('#btn-uav-land').addEventListener('click', () => post('uav_land', {}).then((r) => { if (!r.ok) showResult(r); }));
    $('#btn-uav-emergency').addEventListener('click', () => run('uav_emergency'));
    $('#btn-uav-record').addEventListener('click', () => run('uav_record', {on: !(this.d && this.d.rec.active)}));

    window.addEventListener('keydown', (e) => this.key(e, true), true);
    window.addEventListener('keyup', (e) => this.key(e, false), true);
    window.addEventListener('blur', () => this.releaseAll());
    window.addEventListener('pagehide', () => this.releaseAll());
    document.addEventListener('visibilitychange', () => { if (document.hidden) this.releaseAll(); });

    $$('#uav-pad button').forEach((b) => {                         // the on-screen keys: press and hold
      const key = b.dataset.key;
      b.addEventListener('pointerdown', (e) => {
        e.preventDefault();
        try { b.setPointerCapture(e.pointerId); } catch (err) { /* older browsers */ }
        this.held.add(key); this.report();
      });
      for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
        b.addEventListener(type, () => { if (this.held.delete(key)) this.report(); });
      }
      b.addEventListener('contextmenu', (e) => e.preventDefault());
    });
  }

  setActive(on) {
    if (on === this.active) return;
    this.active = on;
    if (!on) this.releaseAll();
    this.video();
  }

  // ---------------------------------------------------------------- flying with the keyboard
  key(e, down) {
    const key = CODES[e.code];
    if (!key || !this.active) return;
    if (!down) { if (this.held.delete(key)) this.report(); return; }      // a release always counts
    if (e.ctrlKey || e.metaKey || e.altKey || typing(e.target) || modalOpen()) return;
    e.preventDefault();                                                  // no scrolling, no moving a focused slider
    if (e.repeat || this.held.has(key)) return;
    this.held.add(key);
    this.report();
  }

  report() {
    const keys = [...this.held];
    post('uav_keys', {keys, t: Math.round(performance.timeOrigin + performance.now())});
    if (keys.length && !this.timer) this.timer = setInterval(() => this.tick(), REPORT_MS);
    if (!keys.length && this.timer) { clearInterval(this.timer); this.timer = null; }
    this.pad();
  }

  tick() {
    if (!this.active || document.hidden || modalOpen()) { this.releaseAll(); return; }
    post('uav_keys', {keys: [...this.held], t: Math.round(performance.timeOrigin + performance.now())});
  }

  releaseAll() {
    if (!this.held.size) return;
    this.held.clear();
    this.report();
  }

  pad() {
    const flying = !!this.d && this.d.state === 'flying';
    $$('#uav-pad button').forEach((b) => b.classList.toggle('on', this.held.has(b.dataset.key)));
    $('#uav-pad').classList.toggle('off', !flying);
    const state = $('#uav-pad-state'), words = [...this.held].map((k) => WORDS[k]);
    state.classList.toggle('on', flying && words.length > 0);
    state.textContent = flying ? (words.length ? words.join(' + ') : 'hovering · hold a key to move')
      : this.d && this.d.connected ? 'the keys steer after take-off' : 'connect the drone first';
  }

  async takeoff() {
    const r = await post('uav_takeoff_prepare', {});
    if (!r.ok) return showResult(r);
    const yes = await modal({title: r.title, text: r.text, confirmLabel: r.confirm_label, kind: 'go', iconName: 'takeoff'});
    if (yes) showResult(await post('uav_takeoff_confirm', {nonce: r.nonce}));
    return null;
  }

  // ---------------------------------------------------------------- the camera picture
  video() {
    const want = this.active && !!this.d && this.d.video.on;
    if (want === this.wantVideo) return;
    this.wantVideo = want;
    if (want && !this.running) this.loop();
    if (!want) { this.shown = 0; this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height); }
  }

  async loop() {
    this.running = true;
    while (this.wantVideo) {
      try {
        const frame = await fetchFrame(this.last, '/api/uav_frame');
        if (!this.wantVideo) { if (frame) frame.bitmap.close(); break; }
        if (frame) {
          const {bitmap, meta} = frame;
          if (this.canvas.width !== bitmap.width || this.canvas.height !== bitmap.height) { this.canvas.width = bitmap.width; this.canvas.height = bitmap.height; }
          this.ctx.drawImage(bitmap, 0, 0);
          bitmap.close();
          this.last = meta.id; this.shown++;
          if (this.shown === 1) this.overlay();
        } else await sleep(100);
      } catch (e) { await sleep(500); }
    }
    this.running = false;
    this.overlay();
  }

  overlay() {
    const d = this.d, empty = $('#uav-empty');
    if (!d) return;
    const live = d.video.live && this.shown > 0 && this.wantVideo;
    empty.hidden = live;
    $('#uav-empty-text').textContent = d.state === 'connecting' ? 'Connecting…'
      : !d.connected ? 'Drone not connected. Choose "Simulated drone" or join the Tello\'s Wi-Fi, then press Connect.'
      : d.video.note ? d.video.note : 'Waiting for the drone\'s camera…';
  }

  // ---------------------------------------------------------------- state from the console program
  update(d) {
    this.d = d;
    const state = d.state, connected = d.connected, busy = state === 'connecting', flying = state === 'flying';
    const chip = (id, cls, text) => { const el = $(id); el.className = 'chip ' + cls; $('span', el).textContent = text; };
    const stateClass = {connecting: 'info', ready: 'ok', taking_off: 'run', flying: 'run', landing: 'run'}[state] || '';
    const label = d.state_text + (d.simulated ? ' · simulated' : '');
    chip('#chip-uav', stateClass, connected || busy ? 'Drone: ' + label.toLowerCase() : 'Drone not connected');
    const bat = d.battery, low = bat !== null && bat < d.low_battery, empty = bat !== null && bat < 20;
    chip('#chip-uav-bat', bat === null ? '' : empty ? 'bad' : low ? 'run' : 'ok', bat === null ? '—' : `${bat} %`);
    const metres = d.height_cm === null ? null : d.height_cm / 100;
    chip('#chip-uav-alt', metres === null ? '' : d.airborne ? 'info' : '', metres === null ? '—' : `${metres.toFixed(1)} m`);

    // gauges on the picture
    const gBat = $('#g-bat'), gAlt = $('#g-alt');
    gBat.className = 'gauge' + (empty ? ' bad' : low ? ' warn' : '');
    $('b', gBat).textContent = bat === null ? '—' : `${bat} %`;
    const bar = $('i', gBat);
    bar.style.width = (bat === null ? 0 : bat) + '%';
    bar.style.background = empty ? 'var(--danger)' : low ? 'var(--warn)' : 'var(--go)';
    $('b', gAlt).textContent = metres === null ? '—' : `${metres.toFixed(1)} m`;
    $('em', gAlt).textContent = d.tof_cm !== null && d.tof_cm < 3000 && d.airborne ? `ground ${d.tof_cm} cm` : '';
    $('#g-time b').textContent = d.flight_s === null ? '—' : mmss(d.flight_s);
    const pill = $('#uav-state');
    pill.className = 'hud uav-state ' + stateClass;
    pill.textContent = label;
    const rec = d.rec, recTag = $('#uav-rec');
    recTag.hidden = !rec.active;
    recTag.textContent = !rec.active ? '' : rec.saving ? 'REC · saving…' : `REC ${mmss(rec.seconds)}`;

    // warnings (the most serious one first)
    const warn = d.warnings.find((w) => w.level === 'bad') || d.warnings[0], banner = $('#uav-banner');
    banner.hidden = !warn;
    if (warn) { banner.className = 'banner ' + (warn.level === 'bad' ? 'bad' : ''); $('#uav-banner-text').textContent = warn.text; }

    // left panel
    const connectBtn = $('#btn-uav-connect');
    connectBtn.classList.toggle('primary', !connected && !busy);
    connectBtn.classList.toggle('danger-soft', connected || busy);
    connectBtn.disabled = d.airborne;
    connectBtn.title = d.airborne ? 'Land the drone first' : '';
    $('span', connectBtn).textContent = busy ? 'Cancel' : connected ? 'Disconnect' : d.sim ? 'Start simulated drone' : 'Connect';
    const tag = (id, text, cls = '') => { const el = $(id); el.textContent = text; el.className = 'tag ' + cls; };
    tag('#uav-conn-tag', connected ? (d.simulated ? 'simulated' : 'connected') : busy ? 'connecting…' : '', connected ? 'ok' : busy ? 'warn' : '');
    $('#uav-ip-row').hidden = d.sim;
    $$('#uav-side input[data-opt="uav_sim"], #uav-side input[data-opt="uav_ip"]').forEach((input) => { input.disabled = connected || busy; });
    const note = $('#uav-conn-note');
    note.className = 'note' + (!connected && !busy && d.note_level === 'warn' && d.note ? ' warn' : '');
    note.textContent = connected ? (d.simulated ? 'A practice drone on this laptop. Same controls and video path as the real one; how it flies is made up.'
      : `Connected to the drone at ${d.ip}.`)
      : busy ? d.note
      : d.note_level === 'warn' && d.note ? d.note
      : d.sim ? 'A practice drone on this laptop: nothing real moves.'
      : 'Join the drone\'s Wi-Fi (TELLO-…) with this laptop first. Nothing moves until you press TAKE OFF and confirm.';
    $('#btn-uav-takeoff').disabled = !(state === 'ready' && !d.takeoff_hint);
    $('#btn-uav-land').disabled = !connected;
    $('#btn-uav-emergency').disabled = !connected;
    tag('#uav-fly-tag', connected ? d.state_text.toLowerCase() : '', flying ? 'warn' : state === 'ready' ? 'ok' : connected ? 'warn' : '');
    const hint = $('#uav-fly-hint');
    hint.className = 'note' + (state === 'ready' && d.takeoff_hint ? ' warn' : '');
    hint.textContent = !connected ? 'Connect the drone first.'
      : state === 'ready' ? (d.takeoff_hint || 'Ready. TAKE OFF asks you to confirm, then the drone rises and hovers.')
      : state === 'taking_off' ? 'Taking off… the keys work as soon as it hovers.'
      : flying ? 'Flying. Hold W A S D / arrow keys to move, let go to hover. LAND or Esc brings it down.'
      : 'Landing…';
    const steps = [connected, flying || state === 'landing', false, false];
    const now = !connected ? 0 : state === 'ready' || state === 'taking_off' ? 1 : flying ? 2 : 3;
    $$('#uav-steps li').forEach((li, k) => { li.className = k === now ? 'now' : steps[k] || k < now ? 'done' : ''; });

    // footage
    const recBtn = $('#btn-uav-record');
    recBtn.disabled = !(rec.active ? !rec.saving : d.video.live);
    recBtn.classList.toggle('recording', rec.active);
    $('span', recBtn).textContent = rec.active ? (rec.saving ? 'Saving…' : 'Stop recording') : 'Record';
    $('#btn-uav-photo').disabled = !d.video.live;
    tag('#uav-rec-tag', rec.active ? (rec.saving ? 'saving…' : `recording ${mmss(rec.seconds)}`) : d.record_on ? 'on' : 'off', rec.active ? 'bad' : d.record_on ? 'ok' : '');
    const info = $('#uav-rec-info');
    info.className = 'note' + (d.last && !rec.active ? ' ok' : '');
    info.textContent = rec.active ? `Recording ${mmss(rec.seconds)} · ${Number(rec.mb).toFixed(1)} MB so far. Saved ${rec.auto ? 'when the drone has landed' : 'when you stop it'}.`
      : d.last ? `Last footage: ${d.last.name} (${mmss(d.last.seconds)}, ${d.last.size_mb} MB) in ${d.folder}`
      : `Footage and photos are saved in ${d.folder}`;
    if (this.knownMade !== null && d.made > this.knownMade && d.last) {
      toast(`Drone footage saved: ${d.last.name} (${mmss(d.last.seconds)}, ${d.last.size_mb} MB). Footage > Open footage folder.`, 'success', 10000);
    }
    this.knownMade = d.made;
    if (this.knownPhotos !== null && d.photos > this.knownPhotos && d.photo) toast(`Photo saved: ${d.photo.name}`, 'success', 5000);
    this.knownPhotos = d.photos;

    // right column
    const tiles = [];
    const tile = (name, value, cls = '', detail = '') => tiles.push(
      `<div class="tile ${cls}" title="${esc(detail)}"><span>${esc(name)}</span><b>${esc(value)}</b>${detail ? `<em>${esc(detail)}</em>` : ''}</div>`);
    tile('Battery', bat === null ? '—' : `${bat} %`, bat === null ? '' : empty ? 'bad' : low ? 'warn' : 'ok', bat !== null && low ? 'land soon' : '');
    tile('Altitude', metres === null ? '—' : `${metres.toFixed(1)} m`, '', 'above the take-off point');
    tile('Distance to ground', d.tof_cm === null ? '—' : `${d.tof_cm} cm`, '', 'range sensor under the drone');
    tile('Flight time', d.flight_s === null ? '—' : mmss(d.flight_s), '', 'motors running');
    tile('Heading', d.yaw === null ? '—' : `${d.yaw}°`, '', 'turned since switch-on');
    tile('Temperature', d.temp_c === null ? '—' : `${d.temp_c} °C`, d.temp_c !== null && d.temp_c > 85 ? 'warn' : '');
    tile('Video', d.video.live ? `${Number(d.video.fps).toFixed(0)} fps` : '—', d.video.on && !d.video.live && connected ? 'warn' : '',
      d.video.live && d.video.w ? `${d.video.w} × ${d.video.h} · ${d.video.kbps > 9999 ? (d.video.kbps / 1000).toFixed(0) + ' Mbit/s' : d.video.kbps + ' kbit/s'}` : '');
    tile('Sticks', d.sticks.join('  '), d.sticks.some((v) => v !== 0) ? 'ok' : '', 'left-right · forward · up · turn');
    $('#uav-tiles').innerHTML = tiles.join('');
    tag('#uav-status-tag', !connected ? '' : d.source === 'status' ? 'live' : d.source === 'asked' ? 'asked' : 'no data', d.source === 'status' ? 'ok' : connected ? 'warn' : '');
    $('#uav-status-note').textContent = !connected ? 'Battery, altitude and the other readings appear when the drone is connected.'
      : d.source === 'asked' ? 'The status messages do not arrive, so battery and height are asked for once a second.' : '';

    if (this.active) {                                             // the line at the bottom of the window
      $('#status-text').textContent = d.note || '';
      $('#status-text').title = d.note || '';
      $('#status-dot').className = 'status-dot ' + ({flying: 'warn', taking_off: 'warn', landing: 'warn', ready: 'success', connecting: 'info'}[state] || '');
    }
    if (!flying && this.held.size && !connected) this.releaseAll();
    this.video();
    this.overlay();
    this.pad();
  }
}
