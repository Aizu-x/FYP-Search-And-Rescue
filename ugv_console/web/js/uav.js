// The UAV section: the drone's camera picture, battery and altitude, flying it with the keyboard, and
// planned flight (a list of steps the drone flies by itself).
//
// Keys: W A S D move, arrow up / down change height, arrow left / right turn. The page only reports WHICH keys
// are held (at once when that changes, and several times a second while one is down). The console program turns
// that into stick values, and lets the drone hover as soon as the reports stop. Every key is released when the
// window loses focus, a dialog opens or the section is left. While a plan is flown a flight key does not steer:
// it stops the plan and gives the operator control.

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

// the steps of a flight plan: label, which number it takes, its unit, and what the "add" row suggests
const STEP = {
  forward: ['Forward', 'cm', 'cm', 50], back: ['Back', 'cm', 'cm', 50], left: ['Left', 'cm', 'cm', 50], right: ['Right', 'cm', 'cm', 50],
  up: ['Up', 'cm', 'cm', 30], down: ['Down', 'cm', 'cm', 30], cw: ['Turn right', 'deg', '°', 90], ccw: ['Turn left', 'deg', '°', 90],
  wait: ['Hover', 's', 's', 2], speed: ['Speed', 'cm_s', 'cm/s', 30], sticks: ['Keys', 's', 's', 1], photo: ['Photo', null, '', null], land: ['Land', null, '', null],
};

function typing(target) {
  if (!target || !target.tagName) return false;
  if (target.isContentEditable || target.tagName === 'TEXTAREA' || target.tagName === 'SELECT') return true;
  return target.tagName === 'INPUT' && !['checkbox', 'range', 'radio', 'button'].includes(target.type);
}

export class UavView {
  constructor() {
    this.active = false;                 // the UAV section is on screen
    this.d = null;                       // newest state from the console program
    this.p = null;                       // the flight plan on the page (and the saved ones)
    this.held = new Set(); this.timer = null;
    this.canvas = $('#uav-cam'); this.ctx = this.canvas.getContext('2d');
    this.wantVideo = false; this.running = false; this.last = -1; this.shown = 0;
    this.knownMade = null; this.knownPhotos = null; this.knownFlights = null;
    this.planSig = ''; this.planWaiting = false; this.runSeen = null;
    this.bind();
    this.bindPlan();
  }

  // ---------------------------------------------------------------- wiring
  bind() {
    $('#btn-uav-connect').addEventListener('click', () => {
      const d = this.d || {};
      run(d.connected || d.state === 'connecting' ? 'uav_disconnect' : 'uav_connect');
    });
    $('#btn-uav-takeoff').addEventListener('click', () => this.confirmed('uav_takeoff', 'takeoff'));
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

  planRunning() { return !!this.d && !!this.d.run && this.d.run.state === 'running'; }

  pad() {
    const d = this.d, flying = !!d && d.state === 'flying', plan = this.planRunning();
    $$('#uav-pad button').forEach((b) => b.classList.toggle('on', this.held.has(b.dataset.key)));
    $('#uav-pad').classList.toggle('off', !flying);
    const state = $('#uav-pad-state'), words = [...this.held].map((k) => WORDS[k]);
    state.classList.toggle('on', plan || (flying && words.length > 0));
    state.textContent = plan ? `plan: ${d.run.index < 0 ? 'taking off' : `step ${d.run.index + 1} of ${d.run.total}`} · any key = you take over`
      : flying ? (words.length ? words.join(' + ') : 'hovering · hold a key to move')
      : d && d.connected ? 'the keys steer after take-off' : 'connect the drone first';
  }

  // A two-step action: the console program describes what will happen, the operator confirms, then it starts.
  async confirmed(name, iconName) {
    const r = await post(name + '_prepare', {});
    if (!r.ok) return showResult(r);
    const yes = await modal({title: r.title, text: r.text, confirmLabel: r.confirm_label, kind: 'go', iconName});
    if (yes) showResult(await post(name + '_confirm', {nonce: r.nonce}));
    return null;
  }

  // ---------------------------------------------------------------- planned flight
  bindPlan() {
    const kind = $('#plan-kind'), value = $('#plan-value');
    const suggest = () => {
      const [, field, unit, example] = STEP[kind.value];
      value.hidden = !field; $('#plan-unit').textContent = unit;
      if (field) value.value = example;
    };
    kind.addEventListener('change', suggest);
    suggest();
    $('#plan-add').addEventListener('click', () => this.addStep());
    value.addEventListener('keydown', (e) => { if (e.key === 'Enter') this.addStep(); });
    $('#plan-speed').addEventListener('change', (e) => this.sendPlan(this.steps(), Number(e.target.value)));
    $('#plan-speed').addEventListener('keydown', (e) => { if (e.key === 'Enter') e.target.blur(); });

    const list = $('#plan-steps');
    list.addEventListener('click', (e) => {
      const b = e.target.closest('button[data-act]');
      if (!b || !this.p) return;
      const steps = this.steps(), i = Number(b.dataset.i), act = b.dataset.act;
      if (act === 'del') steps.splice(i, 1);
      else {
        const j = act === 'up' ? i - 1 : i + 1;
        if (j < 0 || j >= steps.length) return;
        [steps[i], steps[j]] = [steps[j], steps[i]];
      }
      this.sendPlan(steps);
    });
    list.addEventListener('change', (e) => {
      const input = e.target.closest('input[data-i]');
      if (!input || !this.p) return;
      const steps = this.steps(), step = steps[Number(input.dataset.i)];
      step[STEP[step.do][1]] = Number(input.value);
      this.sendPlan(steps);
    });
    list.addEventListener('keydown', (e) => { if (e.key === 'Enter' && e.target.tagName === 'INPUT') e.target.blur(); });
    list.addEventListener('focusout', () => setTimeout(() => { if (this.planWaiting) this.renderPlan(); }, 0));

    $('#plan-select').addEventListener('change', (e) => {
      const name = e.target.value;
      if (name) run('uav_plan_load', {name}).then(() => this.renderPlan(true));
    });
    $('#plan-save').addEventListener('click', () => run('uav_plan_save', {name: $('#plan-name').value.trim() || (this.p ? this.p.name : '')}));
    $('#plan-name').addEventListener('keydown', (e) => { if (e.key === 'Enter') $('#plan-save').click(); });
    $('#plan-new').addEventListener('click', () => run('uav_plan_new'));
    $('#plan-copy').addEventListener('click', () => run('uav_plan_from_flight'));
    $('#plan-delete').addEventListener('click', () => {
      const name = $('#plan-select').value || (this.p ? this.p.name : '');
      if (name) run('uav_plan_delete', {name}); else toast('Choose a saved plan first.', 'warn');
    });
    $('#btn-plan-run').addEventListener('click', () => this.confirmed('uav_plan', 'play'));
    $('#btn-plan-stop').addEventListener('click', () => post('uav_plan_stop', {}).then(showResult));
  }

  steps() {                              // a copy of the plan's steps, as the console program wants them back
    return (this.p ? this.p.steps : []).map((s) => { const copy = {...s}; delete copy.text; return copy; });
  }

  addStep() {
    if (!this.p) return;
    const kind = $('#plan-kind').value, [, field] = STEP[kind];
    const steps = this.steps(), lands = steps.length && steps[steps.length - 1].do === 'land';
    if (kind === 'land' && lands) { toast('The plan already ends with Land.', 'warn'); return; }
    const step = {do: kind};
    if (field) step[field] = Number($('#plan-value').value);
    if (lands) steps.splice(steps.length - 1, 0, step); else steps.push(step);      // "Land" stays the last step
    this.sendPlan(steps);
  }

  sendPlan(steps, speed) {
    if (!this.p) return;
    run('uav_plan_set', {plan: {speed: speed === undefined ? this.p.speed : speed, steps}}).then((r) => { if (!r.ok) this.renderPlan(true); });
  }

  updatePlan(p) {
    const more = this.knownFlights !== null && p.flights > this.knownFlights && p.flight;
    this.p = p;
    if (more) toast(`This flight is remembered (${p.flight.steps} steps). Planned flight > My last flight flies it again.`, 'success', 9000);
    this.knownFlights = p.flights;
    this.renderPlan();
  }

  renderPlan(force = false) {
    const p = this.p, d = this.d || {};
    if (!p) return;
    const flown = d.run || null, flying = this.planRunning();
    const sig = JSON.stringify([p.steps, p.name, p.speed, p.saved, p.dirty, p.has_flight, flown && [flown.state, flown.index, flown.total], flying]);
    if (!force && sig === this.planSig) return;
    const list = $('#plan-steps');
    if (!force && list.contains(document.activeElement) && document.activeElement.tagName === 'INPUT') { this.planWaiting = true; return; }
    this.planSig = sig; this.planWaiting = false;

    const marks = flown && flown.total === p.steps.length;          // the last run was of this very plan
    const mark = (i) => (!marks ? '' : i < flown.index ? 'done' : i > flown.index ? ''
      : flown.state === 'running' ? 'now' : flown.state === 'done' ? 'done' : 'failed');
    const rows = [`<li class="fixed ${marks && flown.index < 0 && flying ? 'now' : ''}"><span class="n"></span><span class="t">Take off <i>by itself, when the drone is on the ground</i></span></li>`];
    p.steps.forEach((s, i) => {
      const [label, field, unit] = STEP[s.do];
      const name = s.do === 'sticks' ? s.text.replace(/ for [\d.]+ s$/, '') : label;
      rows.push(`<li class="${mark(i)}"><span class="n">${i + 1}</span><span class="t" title="${esc(s.text)}">${esc(name)}</span>` +
        (field ? `<input type="number" data-i="${i}" value="${s[field]}" step="${field === 's' ? 0.1 : 1}" ${flying ? 'disabled' : ''} aria-label="${esc(label)}"><span class="u">${unit}</span>` : '') +
        `<button data-act="up" data-i="${i}" title="Earlier" ${flying || i === 0 ? 'disabled' : ''}>&#8593;</button>` +
        `<button data-act="down" data-i="${i}" title="Later" ${flying || i === p.steps.length - 1 ? 'disabled' : ''}>&#8595;</button>` +
        `<button data-act="del" data-i="${i}" title="Remove this step" ${flying ? 'disabled' : ''}>&#10005;</button></li>`);
    });
    if (!p.steps.length) rows.push('<li class="fixed"><span class="n"></span><span class="t">No steps yet. Add some below, for example Up 20 cm, Turn right 360°, Forward 40 cm, Land.</span></li>');
    else if (!p.lands) rows.push('<li class="fixed"><span class="n"></span><span class="t">No Land step: at the end it hovers and you have the keys.</span></li>');
    list.innerHTML = rows.join('');
    const now = $('li.now', list);
    if (now) now.scrollIntoView({block: 'nearest'});

    const select = $('#plan-select');
    select.innerHTML = '<option value="">choose…</option>' + p.saved.map((n) => `<option ${n === p.name ? 'selected' : ''}>${esc(n)}</option>`).join('');
    select.disabled = flying || !p.saved.length;
    for (const [id, value] of [['#plan-name', p.name], ['#plan-speed', p.speed]]) {
      const input = $(id);
      if (document.activeElement !== input && (id !== '#plan-name' || value || !p.dirty)) input.value = value;
      input.disabled = flying && id === '#plan-speed';
    }
    for (const id of ['#plan-add', '#plan-new', '#plan-kind', '#plan-value']) $(id).disabled = flying;
    $('#plan-copy').disabled = flying || !p.has_flight;
    $('#plan-copy').title = p.has_flight ? 'Load the last flight you flew by hand as the plan' : 'Fly once in Manual control first: that flight is remembered';
    $('#plan-delete').disabled = flying || !p.saved.length;
    $('#plan-save').disabled = !p.steps.length;
    const tag = $('#plan-tag');
    tag.textContent = flying ? (flown.index < 0 ? 'taking off' : `step ${flown.index + 1} of ${flown.total}`) : p.steps.length ? `${p.steps.length} step${p.steps.length === 1 ? '' : 's'}${p.dirty ? ' · not saved' : ''}` : 'empty';
    tag.className = 'tag ' + (flying ? 'warn' : p.steps.length && !p.dirty ? 'ok' : '');
    $('#plan-info').textContent = p.steps.length ? `About ${p.seconds < 60 ? p.seconds + ' s' : mmss(p.seconds) + ' min'} with take-off${p.lands ? ' and landing' : ''}. Long distances are flown in pieces of at most 1 m.` : '';
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
    const flown = d.run, plan = !!flown && flown.state === 'running';
    const chip = (id, cls, text) => { const el = $(id); el.className = 'chip ' + cls; $('span', el).textContent = text; };
    const stateClass = {connecting: 'info', ready: 'ok', taking_off: 'run', flying: 'run', landing: 'run'}[state] || '';
    const label = d.state_text + (plan && flying ? ` · plan ${flown.index + 1}/${flown.total}` : '') + (d.simulated ? ' · simulated' : '');
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

    // warnings (the most serious one first); while a plan flies, the step it is on
    const warn = d.warnings.find((w) => w.level === 'bad') || d.warnings[0], banner = $('#uav-banner');
    banner.hidden = !warn && !plan;
    if (warn) { banner.className = 'banner ' + (warn.level === 'bad' ? 'bad' : ''); $('#uav-banner-text').textContent = warn.text; }
    else if (plan) { banner.className = 'banner ok'; $('#uav-banner-text').textContent = `Flying the plan · ${flown.index < 0 ? 'take off' : `step ${flown.index + 1} of ${flown.total}: ${flown.text}`}`; }

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

    // flight mode: manual control, or planned flight
    const planMode = d.mode === 'plan';
    $('#uav-plan-card').hidden = !planMode;
    tag('#uav-mode-tag', plan ? 'plan flying' : planMode ? 'planned' : 'manual', plan ? 'warn' : '');
    $('#uav-mode-note').textContent = planMode
      ? 'The drone flies your plan by itself after you confirm it. The keys still work whenever no plan is flying.'
      : 'You fly with the keys. Each flight is remembered, so Planned flight can fly it again.';
    $$('#uav-side .seg[data-opt="uav_mode"] button').forEach((b) => { b.disabled = plan; });
    $('#btn-plan-run').hidden = plan;
    $('#btn-plan-run').disabled = !!d.plan_hint;
    $('#btn-plan-stop').hidden = !plan;
    const hint = $('#plan-hint');
    hint.className = 'note' + (plan ? '' : d.plan_hint && connected ? ' warn' : flown && flown.state !== 'done' ? ' warn' : flown ? ' ok' : '');
    hint.textContent = plan ? `${flown.index < 0 ? 'Taking off' : `Step ${flown.index + 1} of ${flown.total}: ${flown.text}`} · ${mmss(flown.seconds)}`
      : d.plan_hint ? d.plan_hint
      : flown ? flown.message
      : 'RUN PLAN shows you the whole plan and asks before anything moves.';
    const seen = flown ? `${flown.state}|${flown.message}` : '';
    if (this.runSeen !== null && seen !== this.runSeen && flown && flown.state !== 'running' && flown.message) {
      toast(flown.message, flown.state === 'done' ? 'success' : flown.state === 'stopped' ? 'warn' : 'error', 9000);
    }
    this.runSeen = seen;

    $('#btn-uav-takeoff').disabled = !(state === 'ready' && !d.takeoff_hint);
    $('#btn-uav-land').disabled = !connected;
    $('#btn-uav-emergency').disabled = !connected;
    tag('#uav-fly-tag', connected ? d.state_text.toLowerCase() : '', flying ? 'warn' : state === 'ready' ? 'ok' : connected ? 'warn' : '');
    const flyHint = $('#uav-fly-hint');
    flyHint.className = 'note' + (state === 'ready' && d.takeoff_hint ? ' warn' : '');
    flyHint.textContent = !connected ? 'Connect the drone first.'
      : plan ? 'A plan is flying. LAND or Esc brings the drone down; any flight key stops the plan and gives you control.'
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
    tile(plan ? 'Pilot: the plan' : 'Sticks', d.sticks.join('  '), d.sticks.some((v) => v !== 0) ? 'ok' : '', 'left-right · forward · up · turn');
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
    this.renderPlan();
  }
}
