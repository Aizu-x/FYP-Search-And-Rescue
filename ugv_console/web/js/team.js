// The team simulator: 2 to 4 simulated robots on one map. Each robot has its own colour, label, start, goal,
// route, state and camera, and the team has an order (the task graph, worked out with Kahn's algorithm).
// All of that is decided by the program (ugv_console/team_console.py); this file draws what the program
// reports and sends the operator's clicks. Nothing here can move a real robot: a team is simulated robots only.

import {post} from './api.js';
import {CameraView} from './camera.js';
import {modal, run, showResult} from './ui.js';

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[c]));
const TAU = Math.PI * 2;
const cssVar = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const wrap = (a) => { while (a > Math.PI) a -= TAU; while (a < -Math.PI) a += TAU; return a; };
const fix = (v) => Number(v).toFixed(2);

const ROBOT_STATE = {OFF: 'not started', IDLE: 'ready', PLANNED: 'route planned', RUNNING: 'driving', ARRIVED: 'at its goal', STOPPED: 'stopped'};
const TASK_STATE = {PENDING: ['waiting', ''], READY: ['ready', 'info'], RUNNING: ['running', 'warn'], SUCCEEDED: ['done', 'ok'],
  FAILED: ['failed', 'bad'], BLOCKED: ['blocked', 'bad'], CANCELLED: ['cancelled', '']};
const PHASE = {setup: ['Not started', ''], ready: ['Ready', 'info'], planned: ['Planned', 'info'], running: ['Driving', 'warn'],
  done: ['Complete', 'ok'], failed: ['Problems', 'bad'], stopped: ['Stopped', 'bad']};

function unpack(b64, n) {
  const out = new Uint8Array(n);
  if (!b64) return out;
  const bin = atob(b64);
  for (let i = 0; i < n; i++) out[i] = (bin.charCodeAt(i >> 3) >> (7 - (i & 7))) & 1;
  return out;
}

function rgba(text) {
  const probe = document.createElement('canvas').getContext('2d');
  probe.fillStyle = '#000'; probe.fillStyle = text;
  probe.fillRect(0, 0, 1, 1);
  return probe.getImageData(0, 0, 1, 1).data;
}

export class TeamView {
  constructor() {
    this.canvas = $('#team-map'); this.ctx = this.canvas.getContext('2d');
    this.active = false;
    this.team = null;             // the program's 'team' report
    this.map = null; this.walls = null;
    this.half = [0.08, 0.07];     // robot half length / half width (m), from the settings
    this.fast = [];               // newest [x, y, yaw, state] of every robot (20 times a second)
    this.drawn = [];              // ... eased towards it, for a smooth picture
    this.tool = null;             // {kind: 'place' | 'goal' | 'block', robot}
    this.drag = null;             // while a start is being placed: {x, y, heading, turned}
    this.hover = null;
    this.cams = [];
    this.sig = {};
    this.view = {s: 100, ox: 0, oy: 0};
    this.bind();
    new ResizeObserver(() => this.resize()).observe(this.canvas.parentElement);
  }

  // ---------------------------------------------------------------- wiring
  bind() {
    $('#team-btn-start').addEventListener('click', () => run(this.team && this.team.running ? 'team_end' : 'team_start'));
    $('#team-btn-plan').addEventListener('click', () => run('team_plan'));
    $('#team-btn-reset').addEventListener('click', () => run('team_reset'));
    $('#team-btn-run').addEventListener('click', () => this.authorize());
    $('#team-tool-block').addEventListener('click', () => this.setTool(this.tool && this.tool.kind === 'block' ? null : {kind: 'block'}));
    $('#team-env').addEventListener('change', (e) => run('set_env', {env: e.target.value}).then(() => this.showEnv()));
    $('#team-robots').addEventListener('click', (e) => {           // the rows are rebuilt, so one listener for all
      const clear = e.target.closest('[data-team-clear]');
      if (clear) { run('team_clear_goals', {robot: Number(clear.dataset.teamClear)}); return; }
      const b = e.target.closest('[data-team-tool]');
      if (!b) return;
      const tool = {kind: b.dataset.teamTool, robot: Number(b.dataset.robot)};
      this.setTool(this.sameTool(tool) ? null : tool);
    });
    const c = this.canvas;
    c.addEventListener('pointerdown', (e) => this.down(e));
    c.addEventListener('pointermove', (e) => this.move(e));
    c.addEventListener('pointerup', (e) => this.up(e));
    c.addEventListener('pointerleave', () => { this.hover = null; $('#team-cursor').textContent = 'x —  ·  y —'; });
    c.addEventListener('contextmenu', (e) => { e.preventDefault(); this.setTool(null); });
  }

  sameTool(t) { return !!this.tool && this.tool.kind === t.kind && this.tool.robot === t.robot; }

  setTool(tool) {
    this.tool = tool; this.drag = null;
    $$('[data-team-tool]').forEach((b) => b.classList.toggle('active',
      !!tool && b.dataset.teamTool === tool.kind && Number(b.dataset.robot) === tool.robot));
    $('#team-tool-block').classList.toggle('active', !!tool && tool.kind === 'block');
    this.canvas.style.cursor = tool ? 'crosshair' : 'default';
    const name = tool && tool.robot ? 'R' + tool.robot : '';
    $('#team-hint').textContent = !tool ? ''
      : tool.kind === 'place' ? `Click where ${name} starts; drag to turn it. Right-click when done.`
        : tool.kind === 'goal' ? `Click where ${name} should go. Right-click when done.`
          : 'Click the map to place a block; click a block to remove it. Right-click when done.';
  }

  async authorize() {                                              // two steps, like the single robot's AUTHORIZE
    const r = await post('team_run_prepare', {});
    if (!r.ok) return showResult(r);
    const yes = await modal({title: r.title, text: r.text, confirmLabel: r.confirm_label, kind: 'go', iconName: 'shield'});
    if (yes) showResult(await post('team_run_confirm', {nonce: r.nonce}));
    return null;
  }

  // ---------------------------------------------------------------- the map: clicks
  world(e) {
    const box = this.canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1, v = this.view;
    const sx = (e.clientX - box.left) * dpr, sy = (e.clientY - box.top) * dpr;
    return [(sx - v.ox) / v.s, (v.oy - sy) / v.s, sx, sy];
  }

  inside(x, y) {
    const m = this.map;
    return !!m && x >= m.origin[0] && y >= m.origin[1] && x <= m.origin[0] + m.w * m.res && y <= m.origin[1] + m.h * m.res;
  }

  down(e) {
    if (e.button !== 0 || !this.tool || !this.map) return;
    const [x, y, sx, sy] = this.world(e);
    if (!this.inside(x, y)) return;
    const tool = this.tool;
    if (tool.kind === 'place') {
      const start = this.team && this.team.starts ? this.team.starts[tool.robot - 1] : null;
      this.drag = {x, y, sx, sy, heading: start ? start[2] * Math.PI / 180 : 0, turned: false};
      this.canvas.setPointerCapture(e.pointerId);
    } else if (tool.kind === 'goal') {
      post('team_goal', {robot: tool.robot, x: +x.toFixed(3), y: +y.toFixed(3)}).then((r) => {
        showResult(r);
        if (r.ok && this.sameTool(tool)) this.nextGoal(tool.robot);
      });
    } else {
      post('team_block', {x: +x.toFixed(3), y: +y.toFixed(3)}).then(showResult);
    }
  }

  move(e) {
    if (!this.map) return;
    const [x, y, sx, sy] = this.world(e);
    this.hover = this.inside(x, y) ? [x, y] : null;
    $('#team-cursor').textContent = this.hover ? `x ${x.toFixed(2)}  ·  y ${y.toFixed(2)} m` : 'x —  ·  y —';
    const d = this.drag;
    if (d && Math.hypot(sx - d.sx, sy - d.sy) > 10 * (window.devicePixelRatio || 1)) {
      const step = (e.shiftKey ? 45 : 5) * Math.PI / 180;
      d.heading = Math.round(Math.atan2(d.sy - sy, sx - d.sx) / step) * step;      // the screen's y points down
      d.turned = true;
    }
  }

  up(e) {
    const d = this.drag, tool = this.tool;
    this.drag = null;
    if (!d || !tool || tool.kind !== 'place') return;
    try { this.canvas.releasePointerCapture(e.pointerId); } catch (err) { /* was not captured */ }
    const args = {robot: tool.robot, x: +d.x.toFixed(3), y: +d.y.toFixed(3)};
    if (d.turned) args.heading = Math.round(((d.heading * 180 / Math.PI) % 360 + 360) % 360);
    post('team_place', args).then(showResult);
  }

  nextGoal(robot) {                                                // after a goal: on to the next robot without one
    const t = this.team;
    if (!t || !t.goals) return;
    for (let k = 1; k <= t.size; k++) {
      const n = (robot + k - 1) % t.size + 1;
      if (n !== robot && !t.goals[n - 1]) return this.setTool({kind: 'goal', robot: n});
    }
    return this.setTool(null);
  }

  // ---------------------------------------------------------------- data from the program
  setSettings(s) {
    const o = s.opts;
    if (s.robot) this.half = [s.robot.half_length, s.robot.half_width];
    const count = o.mode === 'sim' ? o.sim_robots : 1;
    $$('.team-count button').forEach((b) => b.classList.toggle('active', Number(b.dataset.value) === count));
    if (this.theme !== o.theme) { this.theme = o.theme; this.paintWalls(); }
  }

  showEnv() {                                                      // the arena: the same list as in the Drive tab
    const select = $('#team-env'), source = $('#env');
    if (select.options.length !== source.options.length) select.innerHTML = source.innerHTML;
    if (this.team && this.team.env) select.value = this.team.env;         // (as the Drive tab's own list does)
  }

  setMap(m) {
    this.map = m;
    this.cells = unpack(m.cells, m.w * m.h);
    this.paintWalls();
    this.resize();
  }

  paintWalls() {
    const m = this.map;
    if (!m) return;
    const layer = document.createElement('canvas');
    layer.width = m.w; layer.height = m.h;
    const ctx = layer.getContext('2d'), img = ctx.createImageData(m.w, m.h), [r, g, b] = rgba(cssVar('--m-wall'));
    for (let row = 0; row < m.h; row++) {
      for (let col = 0; col < m.w; col++) {
        if (!this.cells[row * m.w + col]) continue;
        const k = ((m.h - 1 - row) * m.w + col) * 4;             // row 0 is the bottom of the map
        img.data[k] = r; img.data[k + 1] = g; img.data[k + 2] = b; img.data[k + 3] = 255;
      }
    }
    ctx.putImageData(img, 0, 0);
    this.walls = layer;
  }

  pushFast(f) {
    this.fast = f.team || [];
  }

  setActive(on) {
    if (on === this.active) return;
    this.active = on;
    if (on) { this.resize(); requestAnimationFrame((t) => this.frame(t)); }
    else this.setTool(null);
    this.cameras();
  }

  update(t) {
    this.team = t;
    if (!t.selected) { this.cameras(); return; }
    const phase = PHASE[t.phase] || [t.phase, ''];
    const start = $('#team-btn-start');
    $('span', start).textContent = t.running ? 'Stop team simulator' : 'Start team simulator';
    start.classList.toggle('primary', !t.running);
    const tag = $('#team-tag');
    tag.textContent = t.running ? `${t.size} robots running` : 'off'; tag.className = 'tag ' + (t.running ? 'ok' : '');
    const ptag = $('#team-phase');
    ptag.textContent = phase[0]; ptag.className = 'tag ' + phase[1];
    $('#team-btn-plan').disabled = !t.running || t.phase === 'running';
    $('#team-btn-reset').disabled = !t.running || t.phase === 'running';
    const go = $('#team-btn-run');
    go.disabled = !t.run.enabled;
    $('#team-run-note').textContent = t.message || t.run.hint;
    $('#team-title').innerHTML = `${esc(this.map ? this.map.env : '')} <small>team of ${t.size} · ${esc(phase[0].toLowerCase())}</small>`;

    const goals = t.goals.filter(Boolean).length, planned = ['planned', 'running', 'done'].includes(t.phase);
    const steps = [t.running, goals > 0, planned, t.phase === 'done'];
    const now = steps.indexOf(false);
    $$('#team-steps li').forEach((li, k) => { li.className = steps[k] ? 'done' : (k === now ? 'now' : ''); });

    this.listRobots(t); this.listTasks(t); this.listFindings(t); this.legend(t); this.cameras(); this.showEnv();
    $('#team-env').disabled = t.phase === 'running';

    const banner = $('#team-banner');
    const show = ['done', 'failed', 'stopped'].includes(t.phase) && t.message;
    banner.hidden = !show;
    if (show) { banner.className = 'banner ' + (t.phase === 'done' ? 'ok' : 'bad'); $('#team-banner-text').textContent = t.message; }
    if (this.active) {
      $('#status-text').textContent = t.running ? `Team simulator: ${t.size} simulated robots · ${phase[0].toLowerCase()}`
        : 'Team simulator: not started';
    }
  }

  listRobots(t) {
    const rows = t.robots.map((r, k) => {
      const start = t.starts[k], goal = t.goals[k];
      const state = r.task || ROBOT_STATE[r.state] || r.state;
      return [r.id, r.colour, state, r.note, r.retries, start.join(), goal ? goal.join() : ''].join('|');
    }).join(';');
    if (rows === this.sig.robots) return;
    this.sig.robots = rows;
    $('#team-robots').innerHTML = t.robots.map((r, k) => {
      const start = t.starts[k], goal = t.goals[k];
      const state = r.task || ROBOT_STATE[r.state] || r.state;
      const cls = r.state === 'RUNNING' ? 'warn' : r.state === 'ARRIVED' ? 'ok' : (r.state === 'STOPPED' || r.task === 'cannot start') ? 'bad' : '';
      const why = r.note ? `<div class="why${r.task.startsWith('at survivor') ? ' mild' : ''}">${esc(r.note)}</div>` : '';
      return `<div class="team-robot" style="--c:${esc(r.colour)}" title="${esc(r.note || r.message || '')}">
        <div class="who"><i class="dot"></i><b>${esc(r.label)}</b><span class="tag ${cls}">${esc(state)}</span></div>
        <div class="where">
          <button class="tool-chip" data-team-tool="place" data-robot="${r.id}" title="Where ${esc(r.label)} starts (facing ${Math.round(start[2])}°)">Start</button>
          <span class="xy" title="facing ${Math.round(start[2])}°">${fix(start[0])}, ${fix(start[1])}</span>
          <button class="tool-chip goal" data-team-tool="goal" data-robot="${r.id}" title="Where ${esc(r.label)} should go">Goal</button>
          <span class="xy">${goal ? `${fix(goal[0])}, ${fix(goal[1])}` : 'not set'}</span>
          ${goal ? `<button class="x" data-team-clear="${r.id}" title="Remove the goal: ${esc(r.label)} stays where it is">&times;</button>` : '<span></span>'}
        </div>${why}</div>`;
    }).join('');
    if (this.tool) this.setTool(this.tool);                        // (keep the chosen button highlighted)
  }

  listTasks(t) {
    const sig = t.tasks.map((k) => k.name + k.state + k.note).join(';') + '|' + t.edges.map((e) => e.why).join(';');
    if (sig === this.sig.tasks) return;
    this.sig.tasks = sig;
    const box = $('#team-tasks');
    if (!t.tasks.length) { box.innerHTML = '<div class="empty">Plan the team routes to see who waits for whom.</div>'; return; }
    const byName = Object.fromEntries(t.tasks.map((k) => [k.name, k]));
    const colour = (id) => (id ? t.colours[id - 1] : 'var(--muted)');
    let html = '';
    t.waves.forEach((wave, n) => {
      html += `<div class="wave"><span>Wave ${n + 1}</span>${n === 0 ? 'starts at once' : 'when what it needs is done'}</div>`;
      for (const name of wave) {
        const k = byName[name];
        if (!k) continue;
        const [text, cls] = TASK_STATE[k.state] || [k.state, ''];
        const needs = k.needs.length ? `needs: ${k.needs.map((x) => (byName[x] ? byName[x].text : x)).join('; ')}` : 'needs nothing';
        html += `<div class="team-task ${cls}" title="${esc(k.note || needs)}"><i class="dot" style="--c:${esc(colour(k.robot))}"></i>
          <span>${esc(k.text)}</span><em class="tag ${cls}">${esc(text)}</em></div>`;
      }
    });
    if (t.edges.length) {
      html += '<div class="wave"><span>Why</span>who waits for whom</div>';
      html += t.edges.map((e) => `<div class="team-why"><b style="color:${esc(colour(e.then))}">R${e.then}</b> waits for
        <b style="color:${esc(colour(e.first))}">R${e.first}</b>: ${esc(e.why)}.</div>`).join('');
    } else {
      html += '<div class="team-why">No robot is in another one\'s way: they all start together.</div>';
    }
    box.innerHTML = html;
  }

  listFindings(t) {
    const sig = t.findings.map((f) => [f.id, f.by.join(), f.status, f.claimed, f.secured_by].join()).join(';');
    if (sig === this.sig.findings) return;
    this.sig.findings = sig;
    const secured = t.findings.filter((f) => f.status === 'secured').length;
    $('#team-found-tag').textContent = `${t.findings.length} found` + (secured ? ` · ${secured} secured` : '');
    $('#team-findings').innerHTML = t.findings.length ? t.findings.map((f) => {
      const done = f.status === 'secured';
      const state = done ? `secured by ${f.secured_by || '?'}` : f.claimed ? `${f.claimed} on its way` : 'found';
      return `<div class="surv ${done ? '' : 'found'}">
      <span class="badge">S${f.id}</span><div><div class="pos">${fix(f.x)}, ${fix(f.y)} m</div>
      <div class="when">seen by ${esc(f.by.join(', '))}</div></div><span class="st">${esc(state)}</span></div>`;
    }).join('') : '<div class="empty">No survivors seen yet.</div>';
  }

  legend(t) {
    const html = t.robots.map((r) => `<span><i style="background:${esc(r.colour)}"></i>${esc(r.label)} ${esc(r.task || ROBOT_STATE[r.state] || r.state)}</span>`).join('');
    if (html !== this.sig.legend) { this.sig.legend = html; $('#team-legend').innerHTML = html; }
  }

  // ---------------------------------------------------------------- cameras: one small picture per robot
  cameras() {
    const t = this.team, box = $('#team-cams');
    const size = t && t.selected ? t.size : 0;
    if (this.cams.length !== size) {
      this.cams.forEach((cam) => cam.setOn(false));
      box.innerHTML = '';
      this.cams = [];
      for (let k = 1; k <= size; k++) {
        const tile = document.createElement('div');
        tile.className = 'team-cam';
        tile.style.setProperty('--c', t.colours[k - 1]);
        tile.innerHTML = `<div class="cam-wrap"><canvas></canvas><div class="cam-empty"><svg class="ic"><use href="#i-camera"/></svg><span>Camera off</span></div></div><b>R${k}</b>`;
        box.appendChild(tile);
        this.cams.push(new CameraView($('canvas', tile), $('.cam-empty', tile), '/api/team_frame?robot=' + k));
      }
    }
    const on = this.active && !!t && !!t.running && !!t.camera;
    this.cams.forEach((cam) => cam.setOn(on));
    $('#team-cam-tag').textContent = on ? `${size} live` : '';
  }

  // ---------------------------------------------------------------- drawing
  resize() {
    const box = this.canvas.parentElement.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.round(box.width * dpr)), h = Math.max(1, Math.round(box.height * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
    const m = this.map;
    if (!m) return;
    const mw = m.w * m.res, mh = m.h * m.res, pad = 34 * dpr;
    const s = Math.max(10, Math.min((w - 2 * pad) / mw, (h - 2 * pad) / mh));
    this.view = {s, ox: (w - mw * s) / 2 - m.origin[0] * s, oy: (h + mh * s) / 2 + m.origin[1] * s};
  }

  frame(now) {
    if (!this.active) return;
    this.draw(now);
    requestAnimationFrame((t) => this.frame(t));
  }

  pose(k) {                                                        // where robot k + 1 is drawn right now
    const t = this.team, target = (t.running && this.fast[k]) || (t.robots[k] ? t.robots[k].pose : null);
    if (!target) return null;
    const p = this.drawn[k];
    if (!p || Math.hypot(target[0] - p[0], target[1] - p[1]) > 0.3) { this.drawn[k] = [target[0], target[1], target[2]]; return this.drawn[k]; }
    p[0] += (target[0] - p[0]) * 0.3; p[1] += (target[1] - p[1]) * 0.3; p[2] += wrap(target[2] - p[2]) * 0.3;
    return p;
  }

  draw(now) {
    const ctx = this.ctx, v = this.view, m = this.map, t = this.team, dpr = window.devicePixelRatio || 1;
    const W = this.canvas.width, H = this.canvas.height;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = cssVar('--m-bg'); ctx.fillRect(0, 0, W, H);
    if (!m || !t || !t.selected) return;
    const X = (x) => v.ox + x * v.s, Y = (y) => v.oy - y * v.s, px = dpr;
    const x0 = m.origin[0], y0 = m.origin[1], mw = m.w * m.res, mh = m.h * m.res;
    ctx.fillStyle = cssVar('--m-floor'); ctx.fillRect(X(x0), Y(y0 + mh), mw * v.s, mh * v.s);
    ctx.strokeStyle = cssVar('--m-grid-major'); ctx.lineWidth = px; ctx.beginPath();           // a line every half metre
    for (let x = Math.ceil(x0 / 0.5) * 0.5; x <= x0 + mw + 1e-6; x += 0.5) { ctx.moveTo(X(x), Y(y0)); ctx.lineTo(X(x), Y(y0 + mh)); }
    for (let y = Math.ceil(y0 / 0.5) * 0.5; y <= y0 + mh + 1e-6; y += 0.5) { ctx.moveTo(X(x0), Y(y)); ctx.lineTo(X(x0 + mw), Y(y)); }
    ctx.stroke();
    if (this.walls) { ctx.imageSmoothingEnabled = false; ctx.drawImage(this.walls, X(x0), Y(y0 + mh), mw * v.s, mh * v.s); }
    ctx.font = `${11 * dpr}px "Segoe UI", system-ui, sans-serif`; ctx.fillStyle = cssVar('--m-label'); ctx.textBaseline = 'top'; ctx.textAlign = 'center';
    for (let x = Math.ceil(x0); x <= x0 + mw + 1e-6; x += 1) ctx.fillText(x.toFixed(0) + ' m', X(x), Y(y0) + 6 * dpr);

    const robots = t.robots, text = cssVar('--m-text'), chip = cssVar('--m-chip');
    const label = (s, x, y, colour, ink = '#06111d') => {                                      // a small rounded label
      ctx.font = `700 ${11 * dpr}px "Segoe UI", system-ui, sans-serif`;
      const w = ctx.measureText(s).width + 10 * dpr, h = 17 * dpr;
      ctx.fillStyle = colour; ctx.beginPath(); ctx.roundRect(x - w / 2, y - h / 2, w, h, 6 * dpr); ctx.fill();
      ctx.fillStyle = ink; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(s, x, y + 0.5 * dpr);
    };

    // routes: dashed while the robot has not started, solid once it drives
    robots.forEach((r) => {
      if (!r.path || r.path.length < 2) return;
      ctx.beginPath();
      r.path.forEach((p, k) => (k ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1]))));
      ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.strokeStyle = r.colour; ctx.globalAlpha = 0.22; ctx.lineWidth = 9 * px; ctx.stroke();
      ctx.globalAlpha = r.state === 'ARRIVED' ? 0.45 : 0.95; ctx.lineWidth = 2.6 * px;
      ctx.setLineDash(r.state === 'RUNNING' || r.state === 'ARRIVED' ? [] : [7 * px, 6 * px]);
      ctx.stroke(); ctx.setLineDash([]); ctx.globalAlpha = 1;
    });

    // where two routes are in each other's way
    (t.edges || []).forEach((e) => {
      const x = X(e.at[0]), y = Y(e.at[1]), q = 7 * px;
      ctx.beginPath(); ctx.moveTo(x, y - q); ctx.lineTo(x + q, y); ctx.lineTo(x, y + q); ctx.lineTo(x - q, y); ctx.closePath();
      ctx.fillStyle = chip; ctx.fill(); ctx.strokeStyle = cssVar('--warn'); ctx.lineWidth = 1.6 * px; ctx.stroke();
    });

    // test blocks (the simulated world) and survivors the cameras have found
    (t.blocks || []).forEach((b) => {
      const size = Math.max(0.05 * v.s, 7 * px);
      ctx.fillStyle = b[2].startsWith('green') ? cssVar('--m-green') : cssVar('--m-red');
      ctx.fillRect(X(b[0]) - size / 2, Y(b[1]) - size / 2, size, size);
      ctx.strokeStyle = 'rgba(255,255,255,.7)'; ctx.lineWidth = px; ctx.strokeRect(X(b[0]) - size / 2, Y(b[1]) - size / 2, size, size);
    });
    (t.findings || []).forEach((f) => {
      const done = f.status === 'secured';
      const x = X(f.x), y = Y(f.y), r = (done ? 12 : 11 + 2 * Math.sin(now / 300)) * px;
      ctx.beginPath(); ctx.arc(x, y, r, 0, TAU); ctx.strokeStyle = cssVar('--m-green'); ctx.lineWidth = (done ? 3.5 : 2) * px; ctx.stroke();
      label('S' + f.id + (done ? ' secured' : ''), x, y - 20 * px, cssVar('--m-green'));
    });

    // starts (a faint outline where each robot was put) and goals (a flag in the robot's colour)
    robots.forEach((r, k) => {
      const start = t.starts[k];
      if (start && t.running) {
        ctx.save(); ctx.translate(X(start[0]), Y(start[1])); ctx.rotate(-start[2] * Math.PI / 180);
        ctx.setLineDash([4 * px, 4 * px]); ctx.strokeStyle = r.colour; ctx.globalAlpha = 0.55; ctx.lineWidth = 1.4 * px;
        ctx.beginPath(); ctx.roundRect(-this.half[0] * v.s, -this.half[1] * v.s, 2 * this.half[0] * v.s, 2 * this.half[1] * v.s, 0.025 * v.s); ctx.stroke();
        ctx.restore();
      }
      const goal = t.goals[k];
      if (goal) {
        const x = X(goal[0]), y = Y(goal[1]), pulse = r.state === 'RUNNING' ? 2 * Math.sin(now / 260) : 0;
        ctx.beginPath(); ctx.arc(x, y, (9 + pulse) * px, 0, TAU); ctx.strokeStyle = r.colour; ctx.lineWidth = 2.2 * px; ctx.stroke();
        ctx.beginPath(); ctx.arc(x, y, 3 * px, 0, TAU); ctx.fillStyle = r.colour; ctx.fill();
        ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x, y - 24 * px); ctx.strokeStyle = r.colour; ctx.lineWidth = 2 * px; ctx.stroke();
        ctx.beginPath(); ctx.moveTo(x, y - 24 * px); ctx.lineTo(x + 15 * px, y - 19 * px); ctx.lineTo(x, y - 14 * px); ctx.closePath(); ctx.fillStyle = r.colour; ctx.fill();
        label(r.label + ' goal', x + 4 * px, y + 20 * px, chip, r.colour);
      }
    });

    // the robots
    robots.forEach((r, k) => {
      const dragging = this.drag && this.tool && this.tool.robot === r.id;
      const p = dragging ? [this.drag.x, this.drag.y, this.drag.heading] : this.pose(k);
      if (!p) return;
      const state = (t.running && this.fast[k] && this.fast[k][3]) || r.state;
      this.robot(ctx, X(p[0]), Y(p[1]), p[2], r, state, now, dragging);
      label(r.label, X(p[0]), Y(p[1]) - (Math.hypot(this.half[0], this.half[1]) * v.s + 15 * px), r.colour);
    });

    // what a click would do
    if (this.tool && this.hover && !this.drag) {
      const colour = this.tool.robot ? t.colours[this.tool.robot - 1] : text;
      const x = X(this.hover[0]), y = Y(this.hover[1]);
      ctx.strokeStyle = colour; ctx.lineWidth = 1.4 * px; ctx.setLineDash([4 * px, 4 * px]);
      ctx.beginPath(); ctx.arc(x, y, (this.tool.kind === 'place' ? Math.hypot(this.half[0], this.half[1]) * v.s : 9 * px), 0, TAU); ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  robot(ctx, x, y, yaw, r, state, now, ghost) {
    const s = this.view.s, px = window.devicePixelRatio || 1, hl = this.half[0] * s, hw = this.half[1] * s;
    ctx.save();
    ctx.translate(x, y);
    const ring = {RUNNING: r.colour, ARRIVED: cssVar('--m-green'), STOPPED: cssVar('--m-red')}[state];
    if (ring && !ghost) {
      const grow = state === 'RUNNING' ? 5 + 3 * Math.sin(now / 240) : 5;
      ctx.beginPath(); ctx.arc(0, 0, Math.hypot(hl, hw) + grow * px, 0, TAU);
      ctx.strokeStyle = ring; ctx.globalAlpha = 0.85; ctx.lineWidth = 2.2 * px; ctx.stroke(); ctx.globalAlpha = 1;
    }
    ctx.rotate(-yaw);                                              // the screen's y points down
    ctx.globalAlpha = ghost ? 0.7 : 1;
    ctx.fillStyle = '#0b1220';                                     // four wheels
    const wl = hl * 0.62, ww = Math.max(3 * px, hw * 0.34);
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) { ctx.beginPath(); ctx.roundRect(sx * (hl - wl / 2 - 1) - wl / 2, sy * (hw + 1) - ww / 2, wl, ww, 2 * px); ctx.fill(); }
    ctx.beginPath(); ctx.roundRect(-hl, -hw, 2 * hl, 2 * hw, Math.min(hl, hw) * 0.3);
    ctx.fillStyle = r.colour; ctx.fill();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.9)'; ctx.lineWidth = 1.4 * px; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(hl * 0.8, 0); ctx.lineTo(hl * 0.25, hw * 0.5); ctx.lineTo(hl * 0.25, -hw * 0.5); ctx.closePath();   // heading
    ctx.fillStyle = 'rgba(255, 255, 255, 0.95)'; ctx.fill();
    ctx.beginPath(); ctx.arc(-hl * 0.3, 0, Math.min(hw * 0.38, 0.03 * s), 0, TAU); ctx.fillStyle = '#0b1220'; ctx.fill();          // LiDAR
    if (r.collided && !ghost) {
      ctx.rotate(yaw); ctx.strokeStyle = cssVar('--m-red'); ctx.lineWidth = 3 * px; ctx.beginPath();
      ctx.moveTo(-hl, -hl); ctx.lineTo(hl, hl); ctx.moveTo(hl, -hl); ctx.lineTo(-hl, hl); ctx.stroke();
    }
    ctx.restore();
  }
}
