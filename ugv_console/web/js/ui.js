// Panels, buttons, dialogs: everything around the map. All state comes from the server; this file
// only shows it and sends commands (every command is validated again on the server).

import {post} from './api.js';

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];
const esc = (text) => String(text === null || text === undefined ? '' : text).replace(/[&<>"]/g,
  (c) => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;'}[c]));
const icon = (name) => `<svg class="ic"><use href="#i-${name}"/></svg>`;
const fmt = (v, digits = 2, unit = '') => (v === null || v === undefined || Number.isNaN(v) ? '—' : Number(v).toFixed(digits) + unit);
const CONFIRM_TITLES = {discard_edits: 'Unsaved map edits', revert_map: 'Back to the original map', allow_tight: 'Allow radius below turning circle', ignore_drift: 'Ignore drift alerts',
  clear_survivors: 'Clear survivors'};

// ------------------------------------------------------------------ toasts, dialogs, sound
export function toast(message, level = 'info', ms = 4500) {
  if (!message) return;
  const el = document.createElement('div');
  el.className = 'toast ' + level;
  el.innerHTML = icon({success: 'check', warn: 'alert', error: 'alert'}[level] || 'info') + `<span>${esc(message)}</span>`;
  $('#toasts').appendChild(el);
  while ($('#toasts').children.length > 4) $('#toasts').firstChild.remove();
  setTimeout(() => { el.classList.add('out'); setTimeout(() => el.remove(), 300); }, ms);
}

export function showResult(r) {
  if (!r || r.need_confirm) return r;
  if (!r.ok) toast(r.message || 'Not done.', r.level === 'error' ? 'error' : 'warn', 6500);
  else if (r.message && r.level === 'success') toast(r.message, 'success');
  return r;
}

let modalClose = null;
export function modal({title, text = '', html = '', confirmLabel = 'OK', cancelLabel = 'Cancel', kind = '', wide = false, iconName = 'info'}) {
  return new Promise((resolve) => {
    const back = document.createElement('div');
    back.className = 'modal-back';
    back.innerHTML = `<div class="modal ${kind} ${wide ? 'wide' : ''}" role="dialog" aria-modal="true">
      <h2>${icon(iconName)}<span>${esc(title)}</span></h2>${text ? `<p>${esc(text)}</p>` : ''}${html}
      <div class="actions">${cancelLabel ? `<button class="btn" data-no>${esc(cancelLabel)}</button>` : ''}
      <button class="btn ${kind === 'go' ? 'go' : 'primary'}" data-yes>${esc(confirmLabel)}</button></div></div>`;
    const done = (value) => { back.remove(); modalClose = null; resolve(value); };
    back.addEventListener('pointerdown', (e) => { if (e.target === back) done(false); });
    $('[data-yes]', back).addEventListener('click', () => done(true));
    if (cancelLabel) $('[data-no]', back).addEventListener('click', () => done(false));
    modalClose = () => done(false);
    $('#modal-root').appendChild(back);
    $(cancelLabel ? '[data-no]' : '[data-yes]', back).focus();     // Enter never confirms by accident
  });
}

function ping() {
  try {
    const audio = new (window.AudioContext || window.webkitAudioContext)();
    [880, 1320].forEach((freq, i) => {
      const osc = audio.createOscillator(), gain = audio.createGain(), t0 = audio.currentTime + i * 0.16;
      osc.frequency.value = freq; osc.connect(gain); gain.connect(audio.destination);
      gain.gain.setValueAtTime(0.0001, t0); gain.gain.exponentialRampToValueAtTime(0.18, t0 + 0.02);
      gain.gain.exponentialRampToValueAtTime(0.0001, t0 + 0.35);
      osc.start(t0); osc.stop(t0 + 0.4);
    });
    setTimeout(() => audio.close(), 1200);
  } catch (e) { /* no sound device / not allowed yet */ }
}

// A command that may ask "are you sure?": the server answers need_confirm, we ask, then resend.
export async function run(cmd, args = {}) {
  let r = await post(cmd, args);
  if (!r.ok && r.need_confirm) {
    const yes = await modal({title: CONFIRM_TITLES[r.need_confirm] || 'Please confirm', text: r.message, confirmLabel: 'Yes',
      kind: 'danger', iconName: 'alert'});
    if (!yes) return r;
    r = await post(cmd, {...args, confirm: true});
  }
  return showResult(r);
}

// ------------------------------------------------------------------ the console
export class UI {
  constructor(map, camera) {
    this.map = map; this.camera = camera;
    this.settings = null; this.nav = {connected: false}; this.mapInfo = null; this.mission = null; this.plan = null;
    this.dirty = new Set(); this.timers = {}; this.lastLog = 0; this.knownSurvivors = null; this.theme = null;
    this.logPinned = true;
    this.bind();
  }

  // ---------------------------------------------------------------- wiring
  bind() {
    $$('#tabs button').forEach((b) => b.addEventListener('click', () => this.showTab(b.dataset.tab)));
    $$('#mode-seg button').forEach((b) => b.addEventListener('click', () => run('set_mode', {mode: b.dataset.mode})));
    $('#btn-connect').addEventListener('click', () => run(this.nav.connected ? 'disconnect' : 'connect'));
    $('#btn-stop').addEventListener('click', () => this.stop());
    $('#btn-theme').addEventListener('click', () => this.setOpt('theme', this.settings && this.settings.opts.theme === 'dark' ? 'light' : 'dark'));
    $('#env').addEventListener('change', (e) => run('set_env', {env: e.target.value}).then(() => this.applyOpts(true)));

    // generic: buttons that send one command, inputs bound to one setting
    $$('[data-cmd]').forEach((b) => b.addEventListener('click', () => run(b.dataset.cmd, b.dataset.args ? JSON.parse(b.dataset.args) : {})));
    $$('[data-set-opt]').forEach((b) => b.addEventListener('click', () => this.setOpt(b.dataset.setOpt, Number(b.dataset.value))));
    $$('input[data-opt]').forEach((input) => {
      const key = input.dataset.opt;
      if (input.type === 'checkbox') input.addEventListener('change', () => this.setOpt(key, input.checked));
      else if (input.type === 'range') {
        input.addEventListener('input', () => {
          this.rangeLabels({[key]: Number(input.value)});
          clearTimeout(this.timers[key]);
          this.timers[key] = setTimeout(() => this.setOpt(key, Number(input.value)), 140);
        });
      } else {
        input.addEventListener('change', () => this.setOpt(key, input.type === 'number' ? (input.value === '' ? null : Number(input.value)) : input.value));
        input.addEventListener('keydown', (e) => { if (e.key === 'Enter') input.blur(); });
      }
    });
    $$('.seg[data-opt] button').forEach((b) => b.addEventListener('click', () => {
      const key = b.parentElement.dataset.opt;
      this.setOpt(key, key === 'connectivity' ? Number(b.dataset.value) : b.dataset.value);
    }));

    // tools
    $$('[data-tool]').forEach((b) => b.addEventListener('click', () => this.map.setTool(this.map.tool === b.dataset.tool && b.dataset.tool !== 'pan' ? 'pan' : b.dataset.tool)));
    $('#btn-zoom-in').addEventListener('click', () => this.map.zoomBy(1.3));
    $('#btn-zoom-out').addEventListener('click', () => this.map.zoomBy(1 / 1.3));
    $('#btn-fit').addEventListener('click', () => this.map.fit());
    $('#btn-rotate').addEventListener('click', () => { const deg = this.map.rotate(); toast(deg ? `View turned ${deg}°. Only the picture turns: map x/y, A* and the robot are unchanged.` : 'Normal view: x to the right, y up.'); });
    $('#btn-follow').addEventListener('click', () => { this.map.follow = !this.map.follow; $('#btn-follow').classList.toggle('active', this.map.follow); if (this.map.follow) this.map.view.auto = false; });

    // drive tab
    for (const id of ['sx', 'sy', 'gx', 'gy', 'scale-x', 'scale-y']) {
      $('#' + id).addEventListener('input', () => this.dirty.add(id));
      $('#' + id).addEventListener('keydown', (e) => { if (e.key === 'Enter') (id.startsWith('scale') ? this.applyScale() : this.planRoute()); });
    }
    $('#btn-plan').addEventListener('click', () => this.planRoute());
    $('#btn-authorize').addEventListener('click', () => this.authorize((this.nav.authorize || {}).action || 'drive'));
    $('#btn-search-here').addEventListener('click', () => this.authorize('search_here'));
    $('#banner-btn').addEventListener('click', () => this.bannerAction && this.bannerAction());

    // where is the robot: corner start, find, nudge
    $$('#corners button').forEach((b) => b.addEventListener('click', () => run('corner_start', {corner: b.dataset.corner})));
    $$('#field-corners button').forEach((b) => b.addEventListener('click', () => run('field_sample', {corner: b.dataset.corner})));
    $('#btn-find').addEventListener('click', () => this.findRobot());
    $$('#nudge button').forEach((b) => b.addEventListener('click', () => {
      const [dx, dy, turn] = b.dataset.nudge.split(',').map(Number);
      run('nudge_pose', {dx, dy, turn});
    }));
    $('#found-list').addEventListener('click', (e) => {
      const b = e.target.closest('button[data-choose]');
      if (b) run('find_robot', {choose: Number(b.dataset.choose)});
    });
    for (const id of ['field-x', 'field-y']) {
      $('#' + id).addEventListener('input', () => this.dirty.add(id));
      $('#' + id).addEventListener('keydown', (e) => { if (e.key === 'Enter') this.applyField(); });
    }
    $('#btn-field-apply').addEventListener('click', () => this.applyField());

    // robot + camera tabs
    $('#btn-scale-apply').addEventListener('click', () => this.applyScale());
    $('#btn-cal-finish').addEventListener('click', () => run('cal_finish', {x: Number($('#cal-x').value), y: Number($('#cal-y').value)}));
    $('#btn-sample').addEventListener('click', () => run('camera_sample', {x: Number($('#blk-x').value), y: Number($('#blk-y').value)}));

    // map tab
    $('#btn-picture').addEventListener('click', () => {
      const link = document.createElement('a');
      link.href = this.map.snapshot();
      link.download = `map_${(this.settings ? this.settings.opts.env : 'map')}_${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '-')}.png`;
      link.click();
    });
    $('#btn-batch').addEventListener('click', () => this.batch());

    // Activity log: stay on the newest line unless the OPERATOR scrolled up (wheel or scroll bar).
    // Only real user input may unpin it; scrolling caused by new lines or by the card changing size must not.
    const log = $('#log');
    let userScroll = false;
    log.addEventListener('wheel', () => { userScroll = true; clearTimeout(this.timers.log); this.timers.log = setTimeout(() => { userScroll = false; }, 300); }, {passive: true});
    log.addEventListener('pointerdown', () => { userScroll = true; });
    window.addEventListener('pointerup', () => { setTimeout(() => { userScroll = false; }, 120); });
    log.addEventListener('scroll', () => { if (userScroll) this.logPinned = log.scrollTop + log.clientHeight >= log.scrollHeight - 24; });
    new ResizeObserver(() => { if (this.logPinned) log.scrollTop = log.scrollHeight; }).observe(log);

    window.addEventListener('keydown', (e) => this.key(e), true);
    window.addEventListener('beforeunload', (e) => { if (this.mapInfo && this.mapInfo.unsaved) { e.preventDefault(); e.returnValue = ''; } });
  }

  key(e) {
    if (e.key === 'Escape') {                      // STOP at any time, from anywhere
      e.preventDefault();
      if (modalClose) modalClose();
      this.stop();
      return;
    }
    const typing = ['INPUT', 'SELECT', 'TEXTAREA'].includes((e.target.tagName || '')) || modalClose;
    if (typing || e.ctrlKey || e.metaKey || e.altKey) return;
    const tools = {1: 'pan', 2: 'start', 3: 'goal', 4: 'paint', 5: 'erase', 6: 'block', 7: 'measure'};
    if (tools[e.key]) { if (e.key !== '6' || (this.settings && this.settings.opts.mode === 'sim')) this.map.setTool(tools[e.key]); }
    else if (e.key === 'f' || e.key === 'F') this.map.fit();
    else if (e.key === 'r' || e.key === 'R') $('#btn-rotate').click();
    else if (e.key === '+' || e.key === '=') this.map.zoomBy(1.3);
    else if (e.key === '-') this.map.zoomBy(1 / 1.3);
  }

  stop() {
    post('stop', {}).then((r) => { if (!r.ok) toast(r.message, 'error'); });
    const b = $('#btn-stop');
    b.animate([{transform: 'scale(.94)'}, {transform: 'scale(1)'}], {duration: 180});
  }

  showTab(name) {
    $$('#tabs button').forEach((b) => b.classList.toggle('active', b.dataset.tab === name));
    $$('.tab-panel').forEach((p) => p.classList.toggle('active', p.dataset.panel === name));
  }

  async setOpt(key, value) {
    const r = await run('set_options', {values: {[key]: value}});
    if (!r.ok) this.applyOpts(true);              // put the real value back
    return r;
  }

  async findRobot() {
    const btn = $('#btn-find');
    btn.disabled = true;
    try {
      const r = await post('find_robot', {});
      if (r.ok) toast(r.message, 'success', 7000);
      else toast(r.message || 'Robot not found.', 'warn', 8000);       // several places fit: they are listed in the card
    } finally { btn.disabled = false; }
  }

  applyField() {
    const x = Number($('#field-x').value), y = Number($('#field-y').value);
    this.dirty.delete('field-x'); this.dirty.delete('field-y');
    return run('field_apply', {x, y}).then((r) => { if (!r.ok) this.fieldInputs(true); return r; });
  }

  fieldInputs(force) {
    const f = this.settings && this.settings.field;
    if (!f) return;
    for (const [id, value] of [['field-x', f.measured_w || f.real_w], ['field-y', f.measured_h || f.real_h]]) {
      if (force) this.dirty.delete(id);
      if (!this.dirty.has(id) && document.activeElement !== $('#' + id)) $('#' + id).value = Number(value).toFixed(2);
    }
  }

  planRoute() {
    const v = (id) => { const t = $('#' + id).value.trim(); return t === '' ? null : Number(t); };   // blank goal = not chosen
    for (const id of ['sx', 'sy', 'gx', 'gy']) this.dirty.delete(id);
    return run('apply_coordinates', {sx: v('sx'), sy: v('sy'), gx: v('gx'), gy: v('gy')}).then((r) => { if (!r.ok) this.on_plan(this.plan, true); return r; });
  }

  applyScale() {
    this.dirty.delete('scale-x'); this.dirty.delete('scale-y');
    return run('apply_scale', {x: Number($('#scale-x').value), y: Number($('#scale-y').value)}).then(() => this.applyOpts(true));
  }

  async authorize(action) {
    const r = await post('authorize_prepare', {action});
    if (!r.ok) return showResult(r);
    const yes = await modal({title: r.title, text: r.text, confirmLabel: r.confirm_label, kind: 'go', iconName: 'shield'});
    if (yes) showResult(await post('authorize_confirm', {nonce: r.nonce}));
    return null;
  }

  async batch() {
    toast('Running the three example environments…', 'info', 2500);
    const r = await post('batch_run', {});
    if (!r.ok) return showResult(r);
    const t = r.table;
    const cell = (v) => (typeof v === 'number' ? (Number.isInteger(v) ? v : v.toFixed(3)) : v === null ? '—' : esc(v));
    const html = `<table><tr><th>Metric</th>${t.environments.map((e) => `<th>${esc(e)}</th>`).join('')}</tr>` +
      t.metrics.map((m, i) => `<tr><td>${esc(m.replace(/_/g, ' '))}</td>${t.rows[i].map((v) => `<td>${cell(v)}</td>`).join('')}</tr>`).join('') +
      `</table><p style="margin-top:10px">${t.environments.map((e) => `<b>${esc(e)}</b>: ${esc(t.validation[e])}`).join('<br>')}<br>Files: ${esc(r.folder)}</p>`;
    await modal({title: 'Three-environment results (original maps)', html, confirmLabel: 'Close', cancelLabel: '', wide: true, iconName: 'list'});
    return r;
  }

  // ---------------------------------------------------------------- updates from the server
  update(name, data) {
    const fn = this['on_' + name];
    if (fn) fn.call(this, data);
  }

  on_static(d) {
    this.schema = d.schema;
    $('#env').innerHTML = d.envs.map((e) => `<option>${esc(e)}</option>`).join('');
    $('#ver').textContent = `console v${d.version} · this computer only`;
    $$('input[data-opt][type="number"]').forEach((input) => {
      const s = d.schema[input.dataset.opt];
      if (s && s.lo !== null && !input.min) input.min = s.lo;
      if (s && s.hi !== null && !input.max) input.max = s.hi;
    });
    if (d.config_error) toast('config/navigation.toml could not be loaded: ' + d.config_error + ' (planner and simulator still work)', 'warn', 9000);
    if (this.settings) this.applyOpts(true);
  }

  on_settings(s) {
    const first = !this.settings;
    const modeChanged = first || this.settings.opts.mode !== s.opts.mode || this.settings.opts.lidar_sees !== s.opts.lidar_sees;
    this.settings = s;
    const o = s.opts;
    if (this.theme !== o.theme) {
      this.theme = o.theme;
      document.documentElement.dataset.theme = o.theme;
      this.map.readPalette();
    }
    this.map.setSettings(s);
    $$('#mode-seg button').forEach((b) => b.classList.toggle('active', b.dataset.mode === o.mode));
    $$('[data-show]').forEach((el) => { el.hidden = !el.dataset.show.split(' ').includes(o.mode); });
    if (modeChanged) {
      const active = $('#tabs button.active');
      if (!active || active.hidden) this.showTab('drive');
      if (['block'].includes(this.map.tool) && o.mode !== 'sim') this.map.setTool('pan');
      this.legend();
    }
    const connectBtn = $('#btn-connect');
    connectBtn.classList.toggle('primary', !s.connected);
    connectBtn.classList.toggle('danger-soft', s.connected);
    $('span', connectBtn).textContent = o.mode === 'sim' ? (s.connected ? 'Stop simulator' : 'Start simulator') : (s.connected ? 'Disconnect' : 'Connect');
    $('#conn-title').textContent = o.mode === 'sim' ? 'Simulated robot' : 'Connection';
    $('#conn-note').textContent = s.connected ? '' : (o.mode === 'sim'
      ? 'A simulated MentorPi (motors, odometry, LiDAR, camera) on this laptop, talking through the same link as the real one.'
      : 'Start the bridge on the robot first, then connect. Nothing moves until you authorize.');
    this.applyOpts(first);
    // derived texts
    $('#speed-tag').textContent = `${s.speed.pct} %`;
    $('#speed-info').textContent = `${s.speed.v.toFixed(3)} m/s, turning ${s.speed.w.toFixed(2)} rad/s` + (s.connected ? '' : ' (applies when connected)');
    const info = $('#planner-info');
    if (s.planner_error) { info.textContent = s.planner_error; info.className = 'note bad'; }
    else {
      info.className = 'note' + (s.tight ? ' warn' : '');
      info.innerHTML = s.planning_radius === null ? '' : `A* keeps the robot centre <b>${(s.planning_radius * 100).toFixed(1)} cm</b> from walls. ` +
        `Turning circle of the ${(s.robot.half_length * 200).toFixed(0)} × ${(s.robot.half_width * 200).toFixed(0)} cm body: <b>${(s.robot.turning * 100).toFixed(1)} cm</b>.` +
        (s.tight ? ' The radius is smaller than the turning circle.' : '');
    }
    const cal = $('#cal-tag');
    cal.textContent = s.calibrating ? `${s.calibrating.toUpperCase()} run active` : '';
    cal.className = 'tag' + (s.calibrating ? ' warn' : '');
    // corner start + real field size
    $$('#corners button').forEach((b) => b.classList.toggle('active', b.dataset.corner === s.dock));
    const f = s.field;
    if (f) {
      $$('#field-corners button').forEach((b) => b.classList.toggle('done', f.corners.includes(b.dataset.corner)));
      $('#field-tag').textContent = f.scaled ? `${f.real_w.toFixed(2)} × ${f.real_h.toFixed(2)} m` : 'file size';
      $('#field-tag').className = 'tag ' + (f.scaled ? 'ok' : '');
      $('#field-info').innerHTML = `The map file draws the floor <b>${f.map_w.toFixed(2)} × ${f.map_h.toFixed(2)} m</b> inside the walls` +
        (f.scaled ? `; the console uses your real size <b>${f.real_w.toFixed(2)} × ${f.real_h.toFixed(2)} m</b>.` : '. If your real field is another size, the robot ends up somewhere else than the picture shows.');
      const got = [f.measured_w ? `X ${f.measured_w.toFixed(2)} m` : '', f.measured_h ? `Y ${f.measured_h.toFixed(2)} m` : ''].filter(Boolean);
      const mes = $('#field-measure');
      mes.className = 'note' + (got.length ? ' ok' : '');
      mes.textContent = got.length ? `measured: ${got.join(', ')}. Press Apply.` :
        f.corners.length ? 'one corner measured: now the opposite corner' : 'with the robot: stand it in two corners like for a corner start and click each one';
      this.fieldInputs(false);
    }
    const lid = s.lidar, mount = $('#mount-tag');
    if (lid && o.lidar_sees === 'room' && o.mode === 'robot') {
      mount.textContent = lid.mount_ok ? 'mounting verified' : 'learning mounting';
      mount.className = 'tag ' + (lid.mount_ok ? 'ok' : 'warn');
    } else { mount.textContent = ''; }
    $('#match-label').textContent = o.lidar_sees === 'room' ? 'Room fit' : 'LiDAR fit';
    this.title();
  }

  applyOpts(force) {
    if (!this.settings) return;
    const o = this.settings.opts;
    $$('input[data-opt]').forEach((input) => {
      const value = o[input.dataset.opt];
      if (!force && document.activeElement === input) return;
      if (input.type === 'checkbox') input.checked = !!value;
      else input.value = value === null || value === undefined ? '' : value;
    });
    $$('.seg[data-opt]').forEach((seg) => $$('button', seg).forEach((b) => b.classList.toggle('active', String(o[seg.dataset.opt]) === b.dataset.value)));
    $('#env').value = o.env;
    for (const [id, key] of [['scale-x', 'scale_x'], ['scale-y', 'scale_y']]) {
      if (force) this.dirty.delete(id);
      if (!this.dirty.has(id) && document.activeElement !== $('#' + id)) $('#' + id).value = Number(o[key]).toFixed(3);
    }
    this.rangeLabels(o);
  }

  rangeLabels(o) {
    if (o.brush !== undefined) $('#brush-val').textContent = `${o.brush} cell${Number(o.brush) === 1 ? '' : 's'}`;
    if (o.sim_speed !== undefined) $('#sim-speed-val').textContent = `${Number(o.sim_speed).toFixed(2)} m/s`;
    if (o.robot_speed !== undefined) $('#speed-tag').textContent = `${o.robot_speed} %`;
  }

  title() {
    if (!this.settings) return;
    const m = this.mapInfo, o = this.settings.opts;
    const state = !m ? '' : m.unsaved ? 'unsaved edits' : m.source === 'saved' ? 'your saved map' : 'original map';
    $('#map-title').innerHTML = `${esc(o.env)}<small>${state}</small>`;
    const tag = $('#map-tag');
    tag.textContent = !m ? '' : m.unsaved ? 'Unsaved changes' : m.source === 'saved' ? 'Saved edits' : 'Original';
    tag.className = 'tag ' + (m && m.unsaved ? 'warn' : m && m.source === 'saved' ? 'info' : '');
  }

  legend() {
    const nav = this.settings && this.settings.opts.mode !== 'offline';
    const item = (cls, color, text) => `<span><i class="${cls}" style="background:${color}"></i>${text}</span>`;
    $('#legend').innerHTML = [item('', 'var(--m-wall)', 'Wall'), item('', 'var(--m-inflation)', 'Keep-out'), item('line', 'var(--m-path)', 'Route'),
      item('line', 'var(--m-trail)', nav ? 'Driven' : 'Played'), item('round', 'var(--m-start)', 'Start'), item('round', 'var(--m-goal)', 'Goal')]
      .concat(nav ? (this.settings.opts.lidar_sees === 'room'
        ? [item('round', 'var(--m-scan)', 'LiDAR (sees the room)'), item('', 'color-mix(in srgb, var(--m-text) 35%, transparent)', 'Room outline'),
          item('round', 'var(--m-scan-unknown)', 'Tall object in the arena')]
        : [item('round', 'var(--m-scan)', 'LiDAR on walls'), item('round', 'var(--m-scan-unknown)', 'LiDAR: not on map')]).concat([
        item('', 'var(--m-lidar-cell)', 'Found by LiDAR'), item('', 'var(--m-green)', 'Green block'), item('', 'var(--m-red)', 'Red block')])
        : [item('line', 'var(--m-red)', 'GT violation')]).join('');
  }

  on_map(m) {
    this.mapInfo = m;
    this.map.setMap(m);
    $('#lidar-card').hidden = !m.lidar_count;
    $('#lidar-tag').textContent = `${m.lidar_count} cells`;
    this.title();
  }

  on_plan(p, force) {
    if (!p) return;
    this.plan = p;
    this.map.plan = p;
    const set = (id, v) => { if (force || (!this.dirty.has(id) && document.activeElement !== $('#' + id))) $('#' + id).value = v === null || v === undefined ? '' : Number(v).toFixed(3); };
    if (p.start) { set('sx', p.start[0]); set('sy', p.start[1]); }
    if (p.goal) { set('gx', p.goal[0]); set('gy', p.goal[1]); } else { set('gx', null); set('gy', null); }
    const i = p.info || {}, tag = $('#route-tag'), info = $('#route-info');
    if (p.path && p.path.length > 1) {
      tag.textContent = `${Number(i.length_m).toFixed(2)} m`; tag.className = 'tag info';
      info.className = 'note';
      info.innerHTML = `<b>${i.points}</b> waypoints` + (i.smoothed && i.raw_points ? ` (smoothed from ${i.raw_points})` : i.smoothed ? ' (smoothed)' : '') +
        ` · A* <b>${fmt(i.astar_ms, 1)}</b> ms · ${i.expanded || 0} cells searched`;
    } else if (i.found === false) {
      tag.textContent = 'no route'; tag.className = 'tag bad';
      info.className = 'note bad'; info.textContent = i.message || 'A* found no route to the goal.';
    } else { tag.textContent = ''; info.textContent = ''; }
  }

  on_nav(n) {
    this.nav = n;
    this.map.zone = n.zone || null;
    if (n.max_sigma) this.map.maxSigma = n.max_sigma;
    const chip = (id, cls, text) => { const el = $(id); el.className = 'chip ' + cls; $('span', el).textContent = text; };
    const link = n.link || {};
    const offline = this.settings && this.settings.opts.mode === 'offline';
    const connTag = $('#conn-tag');
    connTag.textContent = !n.connected ? '' : link.ok ? 'connected' : 'connecting…';
    connTag.className = 'tag ' + (n.connected ? (link.ok ? 'ok' : 'warn') : '');
    if (!n.connected) chip('#chip-link', '', 'Not connected');
    else if (!link.ok) chip('#chip-link', 'bad', link.state === 'CONNECTED' ? 'No telemetry' : (link.error ? 'Link: ' + link.error.slice(0, 26) : 'Connecting…'));
    else chip('#chip-link', link.armed ? 'run' : 'ok', `Link ${Math.round(link.rtt_ms || 0)} ms · ${link.armed ? 'ARMED' : 'disarmed'}`);
    if (!offline) {
      const cls = {RUNNING: 'run', ARRIVED: 'ok', STOPPED: 'bad', PLANNED: 'info', IDLE: ''}[n.state] || '';
      chip('#chip-nav', n.connected ? cls : '', n.connected ? ({IDLE: 'Idle', PLANNED: 'Route ready', RUNNING: n.motion === 'spin' ? 'Turning' : 'Driving', ARRIVED: 'Arrived', STOPPED: 'Stopped'}[n.state] || n.state) : 'Robot off');
    }
    if (n.connected && n.loc) {
      const ok = n.loc[0] === 'OK', cm = Math.round((n.sigma || 0) * 100);
      const over = ok && !!n.ignore_drift && (n.sigma || 0) > n.max_sigma;      // past the limit, alert switched off
      chip('#chip-loc', ok ? (over || (n.sigma || 0) > 0.7 * n.max_sigma ? 'run' : 'ok') : 'bad',
        ok ? `±${cm} cm${over ? ' · alerts off' : ''}` : ({UNCALIBRATED: 'Pose not set', STALE: 'No odometry', LOST: 'Position lost', LOW_CONFIDENCE: `Drift ±${cm} cm`}[n.loc[0]] || n.loc[0]));
      const bar = $('#sigma-bar'), frac = Math.min(1, (n.sigma || 0) / (n.max_sigma || 0.15));
      bar.style.width = (ok || n.loc[0] === 'LOW_CONFIDENCE' ? frac * 100 : 0) + '%';
      bar.style.background = over ? 'var(--warn)' : frac > 0.8 ? 'var(--danger)' : frac > 0.55 ? 'var(--warn)' : 'var(--go)';
      $('#sigma-val').textContent = ok || n.loc[0] === 'LOW_CONFIDENCE' ? `${cm} / ${Math.round(n.max_sigma * 100)} cm${over ? ' · ignored' : ''}` : '—';
    } else { chip('#chip-loc', '', '—'); $('#sigma-bar').style.width = '0'; $('#sigma-val').textContent = '—'; }

    // live tracking (LiDAR) + places offered by "Find robot"
    const room = n.connected ? n.room : null, track = $('#track-tag'), tinfo = $('#track-info');
    const tracking = this.settings && this.settings.opts.lidar_correction;
    if (!n.connected || !this.settings || this.settings.opts.lidar_sees !== 'room') { track.textContent = ''; tinfo.textContent = ''; }
    else if (!tracking) { track.textContent = 'tracking off'; track.className = 'tag'; tinfo.className = 'note'; tinfo.textContent = 'Live LiDAR tracking is off (Robot tab): the position comes from odometry only.'; }
    else if (!room) { track.textContent = ''; tinfo.textContent = ''; }
    else {
      const learn = room.learn || {};
      const names = {locked: 'tracking', learning: 'drawing room', searching: 'searching', lost: 'lost', waiting: 'no position yet'};
      const good = room.state === 'locked' && room.calibrated;
      track.textContent = n.searching ? 'searching…' : (room.state === 'locked' && !room.calibrated ? 'watching' : names[room.state] || room.state);
      track.className = 'tag ' + (good ? 'ok' : room.state === 'lost' ? 'bad' : 'warn');
      tinfo.className = 'note' + (room.state === 'lost' ? ' warn' : '');
      tinfo.textContent =
        room.state === 'waiting' ? (room.cells ? 'The room is known: the robot is searched for by itself, or press Find robot.' : 'Place the robot (Corner start) and press Confirm pose: the LiDAR then draws the room around the arena.') :
        room.state === 'lost' || room.state === 'searching' ? `The LiDAR view does not fit here${room.reason ? ' (' + room.reason + ')' : ''}. ${room.state === 'lost' ? 'Searching for the robot…' : ''}` :
        !room.calibrated ? `Watching only: learning how the LiDAR is mounted from the robot's motion (straight ${Math.round((learn.straight || 0) * 100)} %, turns ${Math.round((learn.turn || 0) * 100)} %). Until then the position comes from odometry.` :
        `The position follows the room outline (${room.cells} wall cells)${room.trusted ? ', remembered for next time' : ''}.`;
    }
    const list = $('#found-list'), found = n.connected && n.found ? n.found : [];
    const sig = found.map((c) => `${c.x.toFixed(2)},${c.y.toFixed(2)},${c.heading.toFixed(0)}`).join('|');
    if (sig !== this.foundSig) {
      this.foundSig = sig;
      list.innerHTML = found.length ? `<div class="q">${found.length > 1 ? 'Several places fit the LiDAR view. Where is the robot?' : 'The LiDAR is not sure. Is the robot here? Click to accept.'}</div>` + found.map((c, k) =>
        `<button class="btn" data-choose="${k}"><b>${k + 1}</b>&nbsp; x ${c.x.toFixed(2)}, y ${c.y.toFixed(2)} m, facing ${c.heading.toFixed(0)}° &nbsp;(${Math.round(c.fit * 100)} %)</button>`).join('') : '';
    }
    this.map.found = found;

    // authorize button + hint
    const a = n.authorize || {enabled: false, label: 'AUTHORIZE', hint: ''};
    const btn = $('#btn-authorize');
    btn.disabled = !a.enabled;
    $('span', btn).textContent = a.label;
    btn.classList.toggle('resume', a.action === 'resume');
    $('#authorize-hint').textContent = a.hint || '';

    // steps
    const done = [n.connected && link.ok, n.connected && n.loc && n.loc[0] === 'OK', !!n.has_path, ['RUNNING', 'ARRIVED'].includes(n.state)];
    let current = done.indexOf(false);
    if (n.state === 'PLANNED') current = 3;
    $$('#steps li').forEach((li, k) => { li.className = done[k] && k !== current ? 'done' : k === current ? 'now' : ''; });

    this.banner(n);
    const text = n.connected ? n.message : n.notice;
    $('#status-text').textContent = text || '';
    $('#status-text').title = text || '';
    $('#status-dot').className = 'status-dot ' + (n.connected ? ({RUNNING: 'warn', STOPPED: 'error', ARRIVED: 'success', PLANNED: 'info'}[n.state] || '') : '');
  }

  banner(n) {
    const el = $('#banner'), btn = $('#banner-btn');
    const show = (cls, text, label, action) => {
      el.hidden = false; el.className = 'banner ' + cls; $('#banner-text').textContent = text;
      btn.hidden = !label; btn.textContent = label || ''; this.bannerAction = action || null;
    };
    const m = this.mission, link = n.link || {};
    if (!n.connected) { el.hidden = true; return; }
    if (!link.ok) show('bad', 'No link to the UGV' + (link.error ? ': ' + link.error : ' yet…'));
    else if (n.loc && n.loc[0] === 'LOW_CONFIDENCE') {
      const tip = this.settings && !this.settings.opts.lidar_correction ? ' Tip: "Live LiDAR tracking" in the Robot tab avoids these stops.'
        : n.room && !n.room.calibrated ? ' The LiDAR is still learning how it is mounted; once that is verified these stops go away.' : '';
      const skip = ' Or switch on "Ignore drift alerts" under Robot position.';
      show('', (m && m.active && m.paused ? 'Mission paused: the' : 'The') + ` drift limit was reached (about ${Math.round((n.sigma || 0) * 100)} cm).` + tip + skip,
        'Re-localize with LiDAR', () => run('relocalize'));
    }
    else if (m && m.active && m.paused) show('', 'Mission paused: ' + m.pause_reason, 'Resume', () => this.authorize('resume'));
    else if (n.loc && n.loc[0] === 'LOST') show('bad', `Position lost: ${n.loc[1]}. Place the robot, set Start and Confirm pose.`);
    else if (n.auto && n.auto.pending) show('', 'New route found: continuing by itself in a moment (STOP cancels).');
    else if (n.state === 'STOPPED') show('bad', n.message, n.goal ? 'Plan again' : '', () => this.planRoute());
    else if (n.state === 'ARRIVED' && !(m && m.active)) show('ok', n.message);
    else el.hidden = true;
  }

  on_scan(s) { this.map.scan = s; }
  on_room(r) { this.map.room = r; }

  on_vision(v) {
    this.map.vision = v;
    this.camera.setOn(v.on);
    $('#cam-empty span').textContent = v.on ? 'Waiting for camera frames…' : 'Camera off';
    $('#cam-fps').textContent = v.on ? `${v.fps.toFixed(1)} fps` : '';
    $('#cam-status').textContent = v.on ? `${v.boxes} in view · ${v.pins.length} on the map · ${v.status}` +
      (v.ignored ? ` · ${v.ignored} not block-shaped ignored` : '') : (this.settings && this.settings.opts.camera_on ? 'Starts when you connect.' : 'Turn it on in the Camera tab.');
    const tag = $('#cam-tag'); tag.textContent = v.on ? 'running' : 'off'; tag.className = 'tag ' + (v.on ? 'ok' : '');
    $('#samples-tag').textContent = `${v.samples} sample${v.samples === 1 ? '' : 's'}`;
    const t = v.tilt, info = $('#tilt-info');
    if (v.on && t) {
      info.className = 'note' + (t.confident ? ' ok' : '');
      info.textContent = `Measured from block size: ${t.deg.toFixed(1)}° (spread ${t.spread_deg.toFixed(1)}°, ${t.n} sighting${t.n === 1 ? '' : 's'})` +
        (t.confident ? '' : ' — more sightings needed') + `. In use: ${v.camera_deg.toFixed(1)}°.`;
    } else { info.className = 'note'; info.textContent = 'Show the camera a whole block: the tilt is measured from its known 10 cm height.'; }
    $('#btn-use-tilt').disabled = !(v.on && t && t.confident);
  }

  on_mission(m) {
    this.mission = m;
    this.map.survivors = m.survivors || [];
    const list = $('#survivors'), rows = m.survivors || [];
    const signature = rows.map((s) => `${s.id}:${s.status}:${s.x.toFixed(2)}:${s.y.toFixed(2)}`).join('|');
    if (signature !== this.survSignature) {
      this.survSignature = signature;
      list.innerHTML = rows.length ? rows.map((s) => `<div class="surv ${esc(s.status)}"><div class="badge">S${s.id}</div>
        <div><div class="pos">${s.x.toFixed(2)}, ${s.y.toFixed(2)} m</div><div class="when">found ${esc(s.found_clock)} · after ${Math.round(s.found_s)} s</div></div>
        <span class="st">${esc(s.status)}</span></div>`).join('') : '<div class="empty">No survivors found yet.</div>';
      const secured = rows.filter((s) => s.status === 'secured').length;
      $('#surv-tag').textContent = `${rows.length} found · ${secured} secured`;
      $('#surv-tag').className = 'tag ' + (rows.length ? 'ok' : '');
    }
    if (this.knownSurvivors !== null && rows.length > this.knownSurvivors) {
      const s = rows[rows.length - 1];
      toast(`Survivor S${s.id} found at (${s.x.toFixed(2)}, ${s.y.toFixed(2)}) m`, 'success', 6000);
      ping();
    }
    this.knownSurvivors = rows.length;
    const chip = $('#chip-mission');
    if (m.active) {
      const t = Math.floor(m.elapsed || 0), time = `${String(Math.floor(t / 60)).padStart(2, '0')}:${String(t % 60).padStart(2, '0')}`;
      chip.className = 'chip ' + (m.paused ? 'bad' : 'run');
      $('span', chip).textContent = m.paused ? `Mission paused · ${rows.length} found` : `${m.phase} · ${time} · ${rows.length} found`;
      chip.title = `Search pass ${m.passes} · ${Math.round((m.coverage || 0) * 100)} % of the floor seen by the camera`;
    } else if (m.complete) { chip.className = 'chip ok'; $('span', chip).textContent = `Mission complete · ${rows.filter((s) => s.status === 'secured').length} secured`; }
    else { chip.className = 'chip'; $('span', chip).textContent = rows.length ? `Mission off · ${rows.length} found` : 'Mission off'; }
    if (m.complete && !this.wasComplete) { toast('Mission complete: survivor secured.', 'success', 8000); ping(); }
    this.wasComplete = !!m.complete;
    if (this.nav) this.banner(this.nav);
  }

  on_seen(s) { this.map.setMask('seen', s.mask); }
  on_trail(t) { this.map.trail = t; }

  on_log(l) {
    const box = $('#log'), lines = l.lines || [];
    const fresh = lines.filter((line) => line.id > this.lastLog);
    if (!fresh.length) { if (!lines.length && !box.children.length) box.innerHTML = '<div class="empty">Nothing yet.</div>'; return; }
    if ($('.empty', box)) box.innerHTML = '';
    const stick = this.logPinned;
    for (const line of fresh) {
      const el = document.createElement('div');
      el.className = 'line ' + line.level;
      el.innerHTML = `<time>${esc(line.t)}</time><span>${esc(line.text)}</span>`;
      box.appendChild(el);
      this.lastLog = line.id;
    }
    while (box.children.length > 250) box.firstChild.remove();
    if (stick) { box.scrollTop = box.scrollHeight; this.logPinned = true; }
  }

  on_offline(o) {
    this.map.setMask('search', o.search || '');
    const tag = $('#sim-tag');
    tag.textContent = o.active ? ({ready: 'ready', running: 'playing', paused: 'paused', arrived: 'arrived', collision: 'stopped', idle: 'no route'}[o.sim_state] || o.sim_state) : '';
    tag.className = 'tag ' + ({running: 'info', arrived: 'ok', collision: 'bad'}[o.sim_state] || '');
    if (!o.active) return;
    if (this.settings && this.settings.opts.mode === 'offline') {
      const chip = $('#chip-nav');
      chip.className = 'chip ' + ({running: 'run', arrived: 'ok', collision: 'bad'}[o.sim_state] || 'info');
      $('span', chip).textContent = 'Planner · ' + (tag.textContent || 'idle');
      $('#status-text').textContent = o.dispatch_allowed === false && o.dispatch_message ? o.dispatch_message : (o.message || '');
      $('#status-dot').className = 'status-dot ' + (o.dispatch_allowed ? 'success' : 'warn');
    }
    const r = o.report, verdict = $('#verdict'), kv = $('#report');
    if (!r) { verdict.textContent = 'No plan yet.'; verdict.className = 'verdict'; kv.innerHTML = ''; return; }
    verdict.textContent = r.validation;
    verdict.className = 'verdict ' + (/passed/i.test(r.validation) ? 'ok' : /failed|No path|Invalid/i.test(r.validation) ? 'bad' : 'warn');
    $('#report-tag').textContent = r.path_found ? `${fmt(r.path_length_m, 3)} m` : '';
    const yn = (v) => (v === null || v === undefined ? 'n/a' : v ? 'YES' : 'no');
    const rows = [['Route length', fmt(r.path_length_m, 3, ' m')], ['Planning time', fmt(r.planning_ms, 1, ' ms')], ['Cells A* expanded', r.expanded_nodes],
      ['Direction changes', r.turns === null ? '—' : r.turns], ['Detour ratio', fmt(r.detour_ratio, 3)], ['Min. GT clearance (centre)', fmt(r.minimum_gt_clearance_cm, 2, ' cm')],
      ['GT centre collision', yn(r.gt_center_collision)], ['GT footprint collision', yn(r.gt_footprint_collision)],
      ['Smoothed', r.smoothed ? `yes (was ${fmt(r.raw_path_length_m, 2)} m, ${r.raw_waypoints} corners)` : 'no'],
      ['Clearance weight / target', `${o.weight} / ${fmt(o.extra_cm, 0)} cm`]];
    kv.innerHTML = rows.map(([k, v]) => `<span>${esc(k)}</span><b>${esc(v)}</b>`).join('');
    $('#report-note').textContent = r.explanation + (o.dispatch_allowed ? '' : ' ' + (o.dispatch_message || '')) + (o.edited ? ' Map edits active.' : '') +
      ' GT is used for evaluation only, never for planning.';
  }

  on_diag(d) {
    const link = (this.nav && this.nav.link) || {}, tiles = [];
    const tile = (label, value, cls = '', detail = '') => tiles.push(
      `<div class="tile ${cls}" title="${esc(detail)}"><span>${esc(label)}</span><b>${esc(value)}</b>${detail ? `<em>${esc(detail)}</em>` : ''}</div>`);
    tile('Link round trip', link.ok ? `${Math.round(link.rtt_ms || 0)} ms` : '—', link.ok ? '' : 'bad', link.ok ? (link.armed ? 'UGV armed' : 'UGV disarmed') : 'no link');
    if (d.loop) tile('Control loop', `${d.loop.hz.toFixed(1)} Hz`, d.loop.max_gap_ms > 250 ? 'bad' : d.loop.max_gap_ms > 160 ? 'warn' : 'ok', `worst gap ${Math.round(d.loop.max_gap_ms)} ms`);
    tile('Last A* plan', `${fmt(d.plan_ms, 1)} ms`, '', d.plan && d.plan.expanded ? `${d.plan.expanded} cells searched` : '');
    if (d.yolo_fps !== undefined) tile('YOLO', `${d.yolo_fps.toFixed(1)} fps`, '', 'on this laptop');
    const match = d.match, bar = $('#match-bar'), val = $('#match-val');
    if (match) {
      const pct = Math.round(match.matched * 100);
      bar.style.width = pct + '%'; bar.style.background = pct >= 75 ? 'var(--go)' : pct >= 50 ? 'var(--warn)' : 'var(--danger)';
      val.textContent = `${pct} %`;
    } else { bar.style.width = '0'; val.textContent = '—'; }
    $('#tiles').innerHTML = tiles.join('');
    const fix = d.lidar_fix, info = $('#lidar-fix-info'), o = this.settings ? this.settings.opts : {};
    if (o.lidar_sees === 'room') {
      const lid = this.settings.lidar || {};
      info.className = 'note';
      info.innerHTML = 'The LiDAR looks over the arena, so it sees the room, not the arena walls. After Confirm pose it draws the room outline and follows the robot\'s true movement in it: no drift, also when the robot is carried.' +
        (o.mode === 'robot' ? ` LiDAR mounting learnt from the robot's motion: <b>${lid.mount_cm ? lid.mount_cm[0] : 0} cm</b> ahead, <b>${lid.mount_cm ? lid.mount_cm[1] : 0} cm</b> left, turned <b>${lid.mount_deg || 0}°</b>` +
          (lid.mount_ok ? ' (verified).' : ' (not verified yet: until then the position is only watched, not corrected).') : '') +
        (lid.room_saved ? ' A room outline is saved for this environment.' : '');
    } else if (o.lidar_correction && fix) {
      info.className = 'note ' + (fix.ok ? 'ok' : 'warn');
      info.textContent = fix.ok ? `Last correction: ${fix.shift_cm} cm, ${fix.turn_deg}° · ${Math.round(fix.matched * 100)} % of the scan on walls.` : `No correction just now: ${fix.reason}.`;
    } else {
      info.className = 'note';
      info.textContent = 'The scan is compared with the arena walls on the map: small corrections about once a second while walls are in view.';
    }
  }
}
