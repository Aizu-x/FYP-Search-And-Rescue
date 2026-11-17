// The map: a 60 fps canvas. Server state arrives a few times a second; the robot pose arrives
// ~20 times a second and is INTERPOLATED between samples (rendered ~0.1 s behind), so motion is
// smooth and never ahead of what the robot reported. Everything is drawn in map metres:
// origin bottom-left, x to the right, y up (rotating the view only turns the picture).

import {clock, post} from './api.js';

const TAU = Math.PI * 2;
const wrap = (a) => { while (a > Math.PI) a -= TAU; while (a < -Math.PI) a += TAU; return a; };
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const cssVar = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

function unpack(b64, n) {
  const out = new Uint8Array(n);
  if (!b64) return out;
  const bin = atob(b64);
  for (let i = 0; i < n; i++) out[i] = (bin.charCodeAt(i >> 3) >> (7 - (i & 7))) & 1;
  return out;
}

function parseColor(text) {
  const probe = document.createElement('canvas').getContext('2d');
  probe.fillStyle = '#000'; probe.fillStyle = text;
  probe.fillRect(0, 0, 1, 1);
  const d = probe.getImageData(0, 0, 1, 1).data;
  return [d[0], d[1], d[2], d[3]];
}

function sampleAt(buf, ts) {
  const n = buf.length;
  if (!n) return null;
  if (ts <= buf[0].t) return buf[0];
  if (ts >= buf[n - 1].t) return buf[n - 1];
  for (let i = n - 2; i >= 0; i--) {
    if (buf[i].t <= ts) {
      const a = buf[i], b = buf[i + 1];
      if (Math.hypot(b.x - a.x, b.y - a.y) > 0.25) return b;        // placed somewhere else: jump, do not glide
      const u = (ts - a.t) / Math.max(b.t - a.t, 1e-6);
      return {t: ts, x: a.x + (b.x - a.x) * u, y: a.y + (b.y - a.y) * u, yaw: a.yaw + wrap(b.yaw - a.yaw) * u};
    }
  }
  return buf[n - 1];
}

export class MapView {
  constructor(canvas, hooks) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.hooks = hooks;                 // {cursor(text), hint(text), pick(kind, x, y), toast(result), tip(html, x, y)}
    this.view = {cx: 1.5, cy: 1.5, scale: 200, rot: 0, auto: true};
    this.map = null; this.cells = null; this.layers = {}; this.cellsDirty = false;
    this.plan = {path: [], start: null, goal: null, unsafe: [], segment: null, pending_goal: false};
    this.trail = {est: [], truth: []};
    this.scan = {pts: [], unknown: ''};
    this.room = {pts: []};              // wall cells the LiDAR drew of the room around the arena
    this.found = [];                    // places offered by "Find robot" when several fit
    this.vision = {pins: [], blocks: []};
    this.survivors = [];
    this.opts = {}; this.robot = {half_length: 0.11, half_width: 0.09}; this.camera = null;
    this.mode = 'offline'; this.connected = false; this.navState = ''; this.zone = null; this.maxSigma = 0.15;
    this.samples = []; this.truthSamples = []; this.intervals = []; this.delay = 0.12;
    this.pose = null; this.truth = null; this.cmd = null; this.sigma = 0; this.lastPt = null;
    this.tool = 'pan'; this.prevTool = 'pan'; this.drag = null; this.cursor = null; this.measure = null;
    this.follow = false; this.painting = false; this.pendingMap = null; this.strokeChain = Promise.resolve();
    this.hover = []; this.size = {w: 10, h: 10, dpr: 1};
    this.pad = {l: 100, r: 70, t: 58, b: 86};                   // room for toolbars, title, axis labels, legend
    this.readPalette();
    this.bind();
    new ResizeObserver(() => this.resize()).observe(canvas.parentElement);
    this.resize();
    const loop = (t) => { this.render(t); requestAnimationFrame(loop); };
    requestAnimationFrame(loop);
    this.frames = 0; this.frameGaps = [];                       // for the smoothness self-check (window.__ugv)
  }

  // ---------------------------------------------------------------- palette / theme
  readPalette() {
    const names = ['bg', 'floor', 'floor-2', 'wall', 'wall-edge', 'inflation', 'grid', 'grid-major', 'label', 'path',
      'path-done', 'trail', 'robot', 'robot-2', 'start', 'goal', 'scan', 'scan-unknown', 'lidar-cell', 'gt', 'seen',
      'search', 'green', 'red', 'truth', 'text', 'chip'];
    this.pal = {};
    for (const n of names) this.pal[n] = cssVar('--m-' + n);
    this.font = cssVar('--font') || 'sans-serif';
    this.rebuildLayers();
  }

  // ---------------------------------------------------------------- data from the server
  setMap(m) {
    this.serverMap = m;                                      // what the program really has (see endStroke)
    if (this.painting) { this.pendingMap = m; return; }
    const fresh = !this.map || this.map.w !== m.w || this.map.h !== m.h || this.map.env !== m.env;
    this.map = m;
    const n = m.w * m.h;
    this.cells = unpack(m.cells, n);
    this.masks = {inflated: unpack(m.inflated, n), gt: unpack(m.gt, n), lidar: unpack(m.lidar, n),
      seen: this.masks ? this.masks.seen : null, search: this.masks ? this.masks.search : null};
    if (fresh) { this.masks.seen = null; this.masks.search = null; }
    this.rebuildLayers();
    if (fresh || this.view.auto) this.fit();
  }

  setMask(name, b64) {
    if (!this.map || !this.masks) return;
    this.masks[name] = b64 ? unpack(b64, this.map.w * this.map.h) : null;
    this.layers[name] = this.masks[name] ? this.layerFrom(this.masks[name], this.pal[name]) : null;
  }

  layerFrom(mask, color, hatch = false) {
    const {w, h} = this.map, n = hatch ? 4 : 1;                // hatch: 4 x 4 sub-pixels per cell, diagonal lines
    const c = document.createElement('canvas');
    c.width = w * n; c.height = h * n;
    const ctx = c.getContext('2d');
    const img = ctx.createImageData(w * n, h * n);
    const [r, g, b, a] = parseColor(color);
    for (let row = 0; row < h; row++) {
      const src = row * w, top = (h - 1 - row) * n;             // row 0 is the bottom of the map
      for (let col = 0; col < w; col++) {
        if (!mask[src + col]) continue;
        for (let dy = 0; dy < n; dy++) {
          for (let dx = 0; dx < n; dx++) {
            const k = ((top + dy) * w * n + col * n + dx) * 4;
            const line = hatch && (col * n + dx + top + dy) % 4 === 0;
            img.data[k] = r; img.data[k + 1] = g; img.data[k + 2] = b; img.data[k + 3] = hatch ? (line ? Math.min(255, a * 2.3) : a * 0.6) : a;
          }
        }
      }
    }
    ctx.putImageData(img, 0, 0);
    return c;
  }

  shadowFrom(cellsLayer) {                                   // soft shadow under the walls, blurred ONCE per map change
    const k = 4, c = document.createElement('canvas');
    c.width = cellsLayer.width * k; c.height = cellsLayer.height * k;
    const ctx = c.getContext('2d');
    ctx.filter = 'blur(5px)';
    ctx.drawImage(cellsLayer, 0, 0, c.width, c.height);
    return c;
  }

  rebuildLayers() {
    if (!this.map || !this.cells) return;
    this.layers.cells = this.layerFrom(this.cells, this.pal.wall);
    this.layers.shadow = this.shadowFrom(this.layers.cells);
    for (const name of ['inflated', 'gt', 'lidar', 'seen', 'search']) {
      const mask = this.masks && this.masks[name];
      const color = {inflated: this.pal.inflation, gt: this.pal.gt, lidar: this.pal['lidar-cell'], seen: this.pal.seen,
        search: this.pal.search}[name];
      this.layers[name] = mask ? this.layerFrom(mask, color, name === 'inflated') : null;
    }
    this.cellsDirty = false;
  }

  setSettings(s) {
    this.opts = s.opts; this.robot = s.robot; this.camera = s.camera;
    if (this.mode !== s.opts.mode || this.connected !== s.connected) {
      this.samples = []; this.truthSamples = []; this.pose = null; this.truth = null; this.lastPt = null;
    }
    this.mode = s.opts.mode; this.connected = s.connected;
  }

  pushFast(f) {
    if (f.pose) {
      if (f.pt !== this.lastPt) {
        if (this.lastPt !== null && f.pt > this.lastPt) {
          this.intervals.push(f.pt - this.lastPt);
          if (this.intervals.length > 40) this.intervals.shift();
          const sorted = [...this.intervals].sort((a, b) => a - b);
          this.delay = clamp(1.8 * sorted[sorted.length >> 1] + 0.02, 0.08, 0.3);
        }
        this.lastPt = f.pt;
        this.samples.push({t: f.pt, x: f.pose[0], y: f.pose[1], yaw: f.pose[2]});
        if (this.samples.length > 90) this.samples.shift();
      }
    } else if (this.samples.length) { this.samples = []; this.pose = null; this.lastPt = null; }
    if (f.truth) {
      this.truthSamples.push({t: f.t, x: f.truth[0], y: f.truth[1], yaw: f.truth[2]});
      if (this.truthSamples.length > 90) this.truthSamples.shift();
    } else if (this.truthSamples.length) { this.truthSamples = []; this.truth = null; }
    this.cmd = f.cmd || null;
    this.sigma = f.sigma || 0;
    this.navState = f.state || '';
  }

  // ---------------------------------------------------------------- view
  resize() {
    const box = this.canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    this.size = {w: Math.max(50, box.width), h: Math.max(50, box.height), dpr};
    this.canvas.width = Math.round(this.size.w * dpr);
    this.canvas.height = Math.round(this.size.h * dpr);
    if (this.view.auto) this.fit();
  }

  mapSize() {
    const m = this.map;
    return m ? {ox: m.origin[0], oy: m.origin[1], mw: m.w * m.res, mh: m.h * m.res} : {ox: 0, oy: 0, mw: 3, mh: 3};
  }

  centre() {
    const p = this.pad;
    return {x: p.l + (this.size.w - p.l - p.r) / 2, y: p.t + (this.size.h - p.t - p.b) / 2};
  }

  fit() {
    const {ox, oy, mw, mh} = this.mapSize();
    const turned = this.view.rot % 180 !== 0, p = this.pad;
    this.view.scale = Math.max(20, Math.min((this.size.w - p.l - p.r) / (turned ? mh : mw),
      (this.size.h - p.t - p.b) / (turned ? mw : mh)));
    this.view.cx = ox + mw / 2; this.view.cy = oy + mh / 2;
    this.view.auto = true;
    this.fitScale = this.view.scale;
  }

  zoomBy(factor, sx, sy) {
    const v = this.view;
    const mid = this.centre();
    const before = this.toWorld(sx === undefined ? mid.x : sx, sy === undefined ? mid.y : sy);
    v.scale = clamp(v.scale * factor, (this.fitScale || 100) * 0.5, (this.fitScale || 100) * 14);
    const after = this.toWorld(sx === undefined ? mid.x : sx, sy === undefined ? mid.y : sy);
    v.cx += before.x - after.x; v.cy += before.y - after.y;
    v.auto = false;
  }

  rotate() { this.view.rot = (this.view.rot + 90) % 360; if (this.view.auto) this.fit(); return this.view.rot; }

  toScreen(x, y) {
    const v = this.view, th = v.rot * Math.PI / 180, c = Math.cos(th), s = Math.sin(th);
    const dx = x - v.cx, dy = y - v.cy, mid = this.centre();
    return {x: mid.x + (dx * c - dy * s) * v.scale, y: mid.y - (dx * s + dy * c) * v.scale};
  }

  toWorld(sx, sy) {
    const v = this.view, th = v.rot * Math.PI / 180, c = Math.cos(th), s = Math.sin(th);
    const mid = this.centre();
    const rx = (sx - mid.x) / v.scale, ry = -(sy - mid.y) / v.scale;
    return {x: v.cx + rx * c + ry * s, y: v.cy - rx * s + ry * c};
  }

  worldTransform(ctx) {
    const v = this.view, d = this.size.dpr, th = v.rot * Math.PI / 180, c = Math.cos(th), s = Math.sin(th), k = v.scale * d;
    const mid = this.centre();
    ctx.setTransform(k * c, -k * s, -k * s, -k * c,
      d * mid.x - k * (c * v.cx - s * v.cy), d * mid.y + k * (s * v.cx + c * v.cy));
  }

  // ---------------------------------------------------------------- tools and pointer input
  setTool(tool) {
    if (!tool.startsWith('pick')) this.prevTool = tool;
    this.tool = tool;
    this.measure = tool === 'measure' ? this.measure : null;
    const hints = {start: 'Click where the robot stands. Drag to point where it faces.', goal: 'Click the goal on the map.',
      paint: 'Drag to paint walls.', erase: 'Drag to erase walls.', block: 'Click to place a test block; click a block to remove it.',
      measure: 'Drag to measure a distance.', 'pick-cal': 'Click the robot\'s REAL centre.', 'pick-block': 'Click the block\'s REAL centre.'};
    this.hooks.hint(hints[tool] || '');
    this.canvas.style.cursor = tool === 'pan' ? 'grab' : 'crosshair';
    this.hooks.tool(tool);
  }

  eventPoint(e) {
    const r = this.canvas.getBoundingClientRect();
    const sx = e.clientX - r.left, sy = e.clientY - r.top;
    return {sx, sy, ...this.toWorld(sx, sy)};
  }

  inMap(p) {
    const {ox, oy, mw, mh} = this.mapSize();
    return p.x >= ox && p.x < ox + mw && p.y >= oy && p.y < oy + mh;
  }

  bind() {
    const c = this.canvas;
    c.addEventListener('contextmenu', (e) => e.preventDefault());
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const p = this.eventPoint(e);
      this.zoomBy(Math.exp(-e.deltaY * 0.0015), p.sx, p.sy);
    }, {passive: false});
    c.addEventListener('pointerdown', (e) => {
      c.setPointerCapture(e.pointerId);
      const p = this.eventPoint(e);
      if (e.button !== 0 || this.tool === 'pan') {
        this.drag = {type: 'pan', sx: p.sx, sy: p.sy, cx: this.view.cx, cy: this.view.cy};
        c.style.cursor = 'grabbing';
        return;
      }
      if (!this.inMap(p)) return;
      const round = (v) => Math.round(v * 1000) / 1000;
      switch (this.tool) {
        case 'start': this.drag = {type: 'start', x: round(p.x), y: round(p.y), sx: p.sx, sy: p.sy, heading: null}; break;
        case 'goal': post('set_goal', {x: round(p.x), y: round(p.y)}).then(this.hooks.toast); break;
        case 'block': post('sim_block', {x: round(p.x), y: round(p.y)}).then(this.hooks.toast); break;
        case 'paint': case 'erase': this.beginStroke(p); break;
        case 'measure': this.measure = {a: p, b: p}; this.drag = {type: 'measure'}; break;
        case 'pick-cal': case 'pick-block': this.hooks.pick(this.tool, round(p.x), round(p.y)); this.setTool(this.prevTool); break;
        default: break;
      }
    });
    c.addEventListener('pointermove', (e) => {
      const p = this.eventPoint(e);
      this.cursor = p;
      this.hooks.cursor(this.inMap(p) ? `x ${p.x.toFixed(3)} m  ·  y ${p.y.toFixed(3)} m` : 'x —  ·  y —');
      const d = this.drag;
      if (d && d.type === 'pan') {
        const a = this.toWorld(d.sx, d.sy), b = this.toWorld(p.sx, p.sy);
        this.view.cx += a.x - b.x; this.view.cy += a.y - b.y;
        d.sx = p.sx; d.sy = p.sy;
        this.view.auto = false; this.follow = false; this.hooks.follow(false);
      } else if (d && d.type === 'start') {
        if (Math.hypot(p.sx - d.sx, p.sy - d.sy) > 10) {
          let deg = Math.atan2(p.y - d.y, p.x - d.x) * 180 / Math.PI;
          deg = Math.round(deg / (e.shiftKey ? 45 : 5)) * (e.shiftKey ? 45 : 5);
          d.heading = deg;
          this.hooks.hint(`Heading ${deg}°  (release to set; Shift = 45° steps)`);
        } else d.heading = null;
      } else if (d && d.type === 'stroke') this.extendStroke(p);
      else if (d && d.type === 'measure') this.measure.b = p;
      else this.updateHover(p);
    });
    const finish = (e) => {
      const d = this.drag;
      this.drag = null;
      if (!d) return;
      if (d.type === 'pan') c.style.cursor = this.tool === 'pan' ? 'grab' : 'crosshair';
      if (d.type === 'start') {
        const args = {x: d.x, y: d.y};
        if (d.heading !== null) args.heading = d.heading;
        post('set_start', args).then(this.hooks.toast);
        this.setTool(this.tool);
      }
      if (d.type === 'stroke') this.endStroke();
    };
    c.addEventListener('pointerup', finish);
    c.addEventListener('pointercancel', finish);
    c.addEventListener('pointerleave', () => { this.cursor = null; this.hooks.cursor('x —  ·  y —'); this.hooks.tip(null); });
  }

  // ---------------------------------------------------------------- painting (shown at once, sent in batches)
  beginStroke(p) {
    this.painting = true;
    this.strokeInfo = {value: this.tool === 'paint' ? 1 : 0, brush: Number(this.opts.brush || 0), pending: [], last: null, timer: null};
    this.drag = {type: 'stroke'};
    this.extendStroke(p);
  }

  extendStroke(p) {
    const s = this.strokeInfo, res = this.map.res;
    const from = s.last || p;
    const n = Math.max(1, Math.ceil(Math.hypot(p.x - from.x, p.y - from.y) / (res / 2)));
    for (let i = 1; i <= n; i++) {
      const x = from.x + (p.x - from.x) * i / n, y = from.y + (p.y - from.y) * i / n;
      if (this.paintLocal(x, y, s.brush, s.value)) s.pending.push([Math.round(x * 1000) / 1000, Math.round(y * 1000) / 1000]);
    }
    s.last = p;
    if (!s.timer) s.timer = setTimeout(() => { s.timer = null; this.flushStroke(false); }, 90);
  }

  paintLocal(x, y, brush, value) {
    const m = this.map;
    const c0 = Math.floor((x - m.origin[0]) / m.res), r0 = Math.floor((y - m.origin[1]) / m.res);
    if (c0 < 0 || r0 < 0 || c0 >= m.w || r0 >= m.h) return false;
    for (let dr = -brush; dr <= brush; dr++) {
      for (let dc = -brush; dc <= brush; dc++) {
        const r = r0 + dr, c = c0 + dc;
        if (r >= 0 && c >= 0 && r < m.h && c < m.w && Math.hypot(dr, dc) <= brush + 0.5 && this.cells[r * m.w + c] !== value) {
          this.cells[r * m.w + c] = value;
          this.cellsDirty = true;
        }
      }
    }
    return true;            // always send: the server applies the same brush (keeps both sides identical)
  }

  flushStroke(end) {
    const s = this.strokeInfo;
    if (!s) return;
    const points = s.pending.splice(0);
    if (!points.length && !end) return;
    const args = {points, brush: s.brush, value: s.value, end};
    this.strokeChain = this.strokeChain.then(() => post('paint', args)).then((r) => {
      if (!r.ok) this.strokeFailed = true;                     // e.g. "stop the robot before editing the map"
      if (!r.ok || end) this.hooks.toast(r);
      return r;
    });
  }

  endStroke() {
    const s = this.strokeInfo;
    if (!s) return;
    if (s.timer) { clearTimeout(s.timer); s.timer = null; }
    this.flushStroke(true);
    this.strokeInfo = null;
    this.strokeChain = this.strokeChain.then(() => {
      this.painting = false;
      const m = this.pendingMap || (this.strokeFailed ? this.serverMap : null);   // refused: show the real map again
      this.pendingMap = null; this.strokeFailed = false;
      if (m) this.setMap(m);
    });
  }

  // ---------------------------------------------------------------- hover tooltips
  updateHover(p) {
    let best = null, bestD = 16;
    for (const item of this.hover) {
      const d = Math.hypot(item.sx - p.sx, item.sy - p.sy);
      if (d < Math.max(bestD, item.r || 0)) { best = item; bestD = d; }
    }
    this.hooks.tip(best ? best.html : null, p.sx, p.sy);
  }

  // ---------------------------------------------------------------- drawing
  render(now) {
    this.frames++;
    if (this.lastFrame) { this.frameGaps.push(now - this.lastFrame); if (this.frameGaps.length > 600) this.frameGaps.shift(); }
    this.lastFrame = now;
    const ctx = this.ctx, {w, h, dpr} = this.size, pal = this.pal;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = pal.bg;
    ctx.fillRect(0, 0, w, h);
    if (!this.map) return;
    if (this.cellsDirty) {
      this.layers.cells = this.layerFrom(this.cells, pal.wall);
      if (!this.painting) this.layers.shadow = this.shadowFrom(this.layers.cells);   // (after the stroke: setMap rebuilds it)
      this.cellsDirty = false;
    }
    const ts = clock.now() - this.delay;
    this.pose = sampleAt(this.samples, ts);
    this.truth = sampleAt(this.truthSamples, ts);
    if (this.follow && this.pose) {
      this.view.cx += (this.pose.x - this.view.cx) * 0.12; this.view.cy += (this.pose.y - this.view.cy) * 0.12;
    }
    const px = 1 / this.view.scale;                         // one screen pixel in metres
    const o = this.opts, {ox, oy, mw, mh} = this.mapSize();
    this.hover = [];
    this.worldTransform(ctx);

    // floor + static layers
    ctx.fillStyle = pal.floor;
    ctx.fillRect(ox, oy, mw, mh);
    const image = (layer, smooth) => {
      if (!layer) return;
      ctx.save();
      ctx.transform(1, 0, 0, -1, ox, oy + mh);
      ctx.imageSmoothingEnabled = !!smooth;
      ctx.drawImage(layer, 0, 0, mw, mh);
      ctx.restore();
    };
    if (o.show_seen && this.mode !== 'offline') image(this.layers.seen);
    if (o.show_inflation) image(this.layers.inflated);
    if (this.mode === 'offline' && o.show_search) image(this.layers.search);
    if (o.show_grid) this.drawGrid(ctx, px);
    ctx.globalAlpha = 0.5;                                    // soft shadow under the walls, then crisp cells
    image(this.layers.shadow, true);
    ctx.globalAlpha = 1;
    image(this.layers.cells);
    if (o.show_gt) image(this.layers.gt);
    image(this.layers.lidar);

    this.drawStart(ctx, px);
    if (o.show_path) this.drawPath(ctx, px, now);
    if (o.show_trail) this.drawTrail(ctx, px);
    if (this.mode !== 'offline') {
      if (o.show_room) this.drawRoom(ctx, px);
      if (o.show_scan) this.drawScan(ctx, px);
      this.drawBlocks(ctx, px);
      if (this.pose && o.show_fov && this.camera) this.drawFov(ctx);
      if (this.pose && o.show_zone && this.zone && this.navState === 'RUNNING') this.drawZone(ctx, px);
      if (this.truth && o.show_truth) this.drawBody(ctx, this.truth, px, true);
    }
    if (this.pose) this.drawRobot(ctx, px, now);
    if (this.mode !== 'offline') this.drawFound(ctx, px);
    this.drawToolPreview(ctx, px);

    // screen-space overlays (always upright, constant size)
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    if (o.show_grid) this.drawAxisLabels(ctx);
    this.drawGoal(ctx, now);
    if (this.mode !== 'offline') this.drawPinLabels(ctx);
    this.drawSurvivors(ctx, now);
    if (this.mode !== 'offline') this.drawFoundLabels(ctx);
    if (this.pose) this.drawRobotLabel(ctx);
    this.drawMeasure(ctx);
    this.drawGizmo(ctx);
  }

  drawGrid(ctx, px) {
    const {ox, oy, mw, mh} = this.mapSize(), pal = this.pal;
    const lines = (step, color) => {
      ctx.beginPath();
      for (let x = Math.ceil(ox / step) * step; x <= ox + mw + 1e-9; x += step) { ctx.moveTo(x, oy); ctx.lineTo(x, oy + mh); }
      for (let y = Math.ceil(oy / step) * step; y <= oy + mh + 1e-9; y += step) { ctx.moveTo(ox, y); ctx.lineTo(ox + mw, y); }
      ctx.strokeStyle = color; ctx.lineWidth = px; ctx.stroke();
    };
    if (this.view.scale > 420) lines(0.1, pal.grid);
    lines(0.5, pal['grid-major']);
  }

  drawAxisLabels(ctx) {
    const {ox, oy, mw, mh} = this.mapSize(), th = this.view.rot * Math.PI / 180;
    ctx.font = `11px ${this.font}`; ctx.fillStyle = this.pal.label; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    const out = (nx, ny) => ({x: (nx * Math.cos(th) - ny * Math.sin(th)) * 15, y: -(nx * Math.sin(th) + ny * Math.cos(th)) * 15});
    const below = out(0, -1), left = out(-1, 0);
    for (let x = Math.ceil(ox / 0.5) * 0.5; x <= ox + mw + 1e-9; x += 0.5) {
      const p = this.toScreen(x, oy), last = x + 0.5 > ox + mw + 1e-6;       // the last tick carries the unit
      ctx.fillText(last ? `${x.toFixed(1)} m` : x.toFixed(1), p.x + below.x, p.y + below.y);
    }
    for (let y = Math.ceil(oy / 0.5) * 0.5; y <= oy + mh + 1e-9; y += 0.5) {
      const p = this.toScreen(ox, y);
      ctx.fillText(y.toFixed(1), p.x + left.x * 1.2, p.y + left.y * 1.2);
    }
  }

  bodyPath(ctx, grow = 0) {
    const hl = this.robot.half_length + grow, hw = this.robot.half_width + grow;
    ctx.beginPath();
    ctx.roundRect(-hl, -hw, 2 * hl, 2 * hw, Math.min(0.03 + grow, hw));
  }

  drawBody(ctx, pose, px, ghost) {                           // outline only (true pose in the simulator, start pose)
    ctx.save();
    ctx.translate(pose.x, pose.y); ctx.rotate(pose.yaw);
    this.bodyPath(ctx);
    ctx.setLineDash([5 * px, 4 * px]);
    ctx.strokeStyle = ghost ? this.pal.truth : this.pal.start; ctx.lineWidth = 1.5 * px; ctx.stroke();
    ctx.setLineDash([]);
    ctx.beginPath(); ctx.moveTo(this.robot.half_length, -this.robot.half_width * 0.7); ctx.lineTo(this.robot.half_length, this.robot.half_width * 0.7);
    ctx.lineWidth = 2.5 * px; ctx.stroke();
    ctx.restore();
  }

  drawStart(ctx, px) {
    const d = this.drag && this.drag.type === 'start' ? this.drag : null;
    const s = this.plan.start;
    if (!s && !d) return;
    if (this.mode === 'offline' && !d) {
      ctx.beginPath(); ctx.arc(s[0], s[1], 6 * px, 0, TAU);
      ctx.fillStyle = this.pal.start; ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5 * px; ctx.stroke();
      return;
    }
    const x = d ? d.x : s[0], y = d ? d.y : s[1];
    const deg = d && d.heading !== null ? d.heading : (s ? s[2] : Number(this.opts.heading || 0));
    const yaw = deg * Math.PI / 180;
    if (this.pose && !d && Math.hypot(this.pose.x - x, this.pose.y - y) < 0.02) return;   // robot is standing on it
    const hl = this.robot.half_length;
    ctx.save();
    ctx.translate(x, y); ctx.rotate(yaw);
    this.bodyPath(ctx);
    ctx.fillStyle = this.pal.start; ctx.globalAlpha = 0.14; ctx.fill(); ctx.globalAlpha = 1;
    ctx.setLineDash([5 * px, 4 * px]); ctx.strokeStyle = this.pal.start; ctx.lineWidth = 1.6 * px; ctx.stroke(); ctx.setLineDash([]);
    const tip = hl + Math.max(0.09, 26 * px);
    ctx.beginPath(); ctx.moveTo(hl, 0); ctx.lineTo(tip, 0);
    ctx.moveTo(tip - 7 * px, -5 * px); ctx.lineTo(tip, 0); ctx.lineTo(tip - 7 * px, 5 * px);
    ctx.lineWidth = 2 * px; ctx.lineCap = 'round'; ctx.stroke();
    ctx.restore();
  }

  drawPath(ctx, px, now) {
    const path = this.plan.path;
    if (!path || path.length < 2) return;
    const pal = this.pal;
    let done = null, rest = path;
    const seg = this.plan.segment;
    if (this.pose && seg !== null && seg !== undefined && this.navState === 'RUNNING') {
      const here = [this.pose.x, this.pose.y];
      done = path.slice(0, seg + 1).concat([here]);
      rest = [here].concat(path.slice(seg + 1));
    }
    const stroke = (pts, color, width, alpha = 1) => {
      ctx.beginPath();
      pts.forEach((p, i) => (i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1])));
      ctx.strokeStyle = color; ctx.globalAlpha = alpha; ctx.lineWidth = width * px; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      ctx.stroke(); ctx.globalAlpha = 1;
    };
    if (done) stroke(done, pal['path-done'], 3);
    stroke(rest, pal.path, 11, 0.16);                          // soft glow
    stroke(rest, pal.path, 3);
    ctx.setLineDash([7 * px, 13 * px]);                        // moving dashes show the direction of travel
    ctx.lineDashOffset = -((now / 1000) * 34 * px) % (20 * px);
    stroke(rest, '#ffffff', 1.6, 0.75);
    ctx.setLineDash([]); ctx.lineDashOffset = 0;
    if (path.length <= 60) {
      ctx.fillStyle = pal.path;
      for (let i = 1; i < path.length - 1; i++) { ctx.beginPath(); ctx.arc(path[i][0], path[i][1], 2.6 * px, 0, TAU); ctx.fill(); }
    }
    for (const k of this.plan.unsafe || []) {                 // planner mode: segments that fail the GT check
      if (path[k + 1]) stroke([path[k], path[k + 1]], pal.red, 4.5);
    }
  }

  drawTrail(ctx, px) {
    const draw = (pts, color, width, dash) => {
      if (pts.length < 2) return;
      const chunks = 7, n = pts.length;
      ctx.strokeStyle = color; ctx.lineWidth = width * px; ctx.lineJoin = 'round'; ctx.lineCap = 'round';
      if (dash) ctx.setLineDash(dash.map((v) => v * px));
      for (let c = 0; c < chunks; c++) {                       // older parts fade out
        const a = Math.floor(c * (n - 1) / chunks), b = Math.floor((c + 1) * (n - 1) / chunks);
        if (b <= a) continue;
        ctx.globalAlpha = dash ? 0.7 : 0.25 + 0.75 * (c + 1) / chunks;
        ctx.beginPath(); ctx.moveTo(pts[a][0], pts[a][1]);
        for (let i = a + 1; i <= b; i++) ctx.lineTo(pts[i][0], pts[i][1]);
        ctx.stroke();
      }
      ctx.globalAlpha = 1; ctx.setLineDash([]);
    };
    if (this.mode === 'sim' && this.opts.show_truth) draw(this.trail.truth, this.pal.truth, 1.2, [2, 4]);
    const est = this.pose ? this.trail.est.concat([[this.pose.x, this.pose.y]]) : this.trail.est;
    draw(est, this.pal.trail, 2.6);
  }

  drawRoom(ctx, px) {                                           // the room outline: small soft squares
    const pts = this.room.pts || [];
    if (!pts.length) return;
    const size = Math.max(this.room.res || 0.03, 2 * px), half = size / 2;
    ctx.fillStyle = this.pal.text; ctx.globalAlpha = 0.3;
    ctx.beginPath();
    for (let i = 0; i < pts.length; i++) ctx.rect(pts[i][0] - half, pts[i][1] - half, size, size);
    ctx.fill();
    ctx.globalAlpha = 1;
  }

  drawFound(ctx, px) {                                          // numbered outlines: "is the robot here?"
    for (let k = 0; k < this.found.length; k++) {
      const c = this.found[k], yaw = c.heading * Math.PI / 180;
      ctx.save();
      ctx.translate(c.x, c.y); ctx.rotate(yaw);
      this.bodyPath(ctx);
      ctx.fillStyle = this.pal.goal; ctx.globalAlpha = 0.18; ctx.fill(); ctx.globalAlpha = 1;
      ctx.setLineDash([4 * px, 3 * px]); ctx.strokeStyle = this.pal.goal; ctx.lineWidth = 1.8 * px; ctx.stroke(); ctx.setLineDash([]);
      ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(this.robot.half_length + 0.06, 0); ctx.stroke();
      ctx.restore();
    }
  }

  drawFoundLabels(ctx) {
    for (let k = 0; k < this.found.length; k++) {
      const p = this.toScreen(this.found[k].x, this.found[k].y);
      this.chip(ctx, `${k + 1} ?`, p.x, p.y - 26, this.pal.goal);
    }
  }

  drawScan(ctx, px) {
    const pts = this.scan.pts, flags = this.scan.unknown || '';
    ctx.fillStyle = this.pal.scan; ctx.globalAlpha = 0.85;
    ctx.beginPath();
    for (let i = 0; i < pts.length; i++) if (flags[i] !== '1') { ctx.moveTo(pts[i][0], pts[i][1]); ctx.arc(pts[i][0], pts[i][1], 1.7 * px, 0, TAU); }
    ctx.fill();
    ctx.fillStyle = this.pal['scan-unknown']; ctx.globalAlpha = 1;
    ctx.beginPath();
    for (let i = 0; i < pts.length; i++) if (flags[i] === '1') { ctx.moveTo(pts[i][0], pts[i][1]); ctx.arc(pts[i][0], pts[i][1], 2.6 * px, 0, TAU); }
    ctx.fill();
  }

  drawBlocks(ctx, px) {
    const square = (b, size, fill, stroke, dashed) => {
      ctx.beginPath(); ctx.roundRect(b.x - size / 2, b.y - size / 2, size, size, size * 0.18);
      if (fill) { ctx.fillStyle = fill; ctx.fill(); }
      if (dashed) ctx.setLineDash([3 * px, 3 * px]);
      ctx.strokeStyle = stroke; ctx.lineWidth = (dashed ? 1.3 : 1.6) * px; ctx.stroke(); ctx.setLineDash([]);
    };
    if (this.mode === 'sim' && this.opts.show_truth) {
      for (const b of this.vision.blocks || []) square(b, 0.05, null, b.cls.startsWith('green') ? this.pal.green : this.pal.red, true);
    }
    for (const p of this.vision.pins || []) {
      const green = (p.cls || '').startsWith('green');
      square(p, p.size || 0.05, green ? this.pal.green : this.pal.red, '#ffffff', false);
    }
  }

  drawFov(ctx) {
    const cam = this.camera, pose = this.pose;
    const half = (cam.fov_deg || 60) * Math.PI / 360, range = cam.range || 1.5;
    ctx.save();
    ctx.translate(pose.x, pose.y); ctx.rotate(pose.yaw); ctx.translate(cam.x_offset || 0, 0);
    const g = ctx.createRadialGradient(0, 0, 0, 0, 0, range);
    g.addColorStop(0, 'rgba(94, 234, 212, 0.20)'); g.addColorStop(1, 'rgba(94, 234, 212, 0)');
    ctx.beginPath(); ctx.moveTo(0, 0); ctx.arc(0, 0, range, -half, half); ctx.closePath();
    ctx.fillStyle = g; ctx.fill();
    ctx.restore();
  }

  drawZone(ctx, px) {
    const pose = this.pose, hl = this.robot.half_length, hw = this.robot.half_width;
    ctx.save();
    ctx.translate(pose.x, pose.y); ctx.rotate(pose.yaw);
    ctx.beginPath(); ctx.rect(hl, -hw, this.zone, 2 * hw);
    ctx.fillStyle = 'rgba(255, 182, 72, 0.13)'; ctx.fill();
    ctx.setLineDash([4 * px, 3 * px]); ctx.strokeStyle = 'rgba(255, 182, 72, 0.7)'; ctx.lineWidth = px; ctx.stroke();
    ctx.restore();
  }

  drawRobot(ctx, px, now) {
    const pose = this.pose, pal = this.pal, hl = this.robot.half_length, hw = this.robot.half_width;
    const small = 2 * hl * this.view.scale < 20;             // zoomed far out: fixed-size marker instead
    const running = this.navState === 'RUNNING' || this.navState === 'running';
    const ring = {RUNNING: pal.path, running: pal.path, STOPPED: pal.red, collision: pal.red, ARRIVED: pal.green, arrived: pal.green}[this.navState];
    if (this.sigma >= 0.035 && this.mode !== 'offline') {     // position uncertainty (drift estimate)
      ctx.beginPath(); ctx.arc(pose.x, pose.y, this.shownSigma() + Math.hypot(hl, hw), 0, TAU);
      ctx.setLineDash([4 * px, 4 * px]);
      ctx.strokeStyle = this.sigma > this.maxSigma * 0.8 ? pal.red : pal.truth; ctx.lineWidth = px; ctx.stroke(); ctx.setLineDash([]);
    }
    ctx.save();
    ctx.translate(pose.x, pose.y);
    const glow = ctx.createRadialGradient(0, 0, 0, 0, 0, Math.max(hl * 2.4, 16 * px));
    glow.addColorStop(0, 'rgba(77, 163, 255, 0.30)'); glow.addColorStop(1, 'rgba(77, 163, 255, 0)');
    ctx.fillStyle = glow; ctx.beginPath(); ctx.arc(0, 0, Math.max(hl * 2.4, 16 * px), 0, TAU); ctx.fill();
    ctx.rotate(pose.yaw);
    if (small) {
      const r = 9 * px;
      ctx.beginPath(); ctx.moveTo(r * 1.3, 0); ctx.lineTo(-r * 0.8, r * 0.85); ctx.lineTo(-r * 0.35, 0); ctx.lineTo(-r * 0.8, -r * 0.85); ctx.closePath();
      ctx.fillStyle = pal.robot; ctx.fill(); ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5 * px; ctx.stroke();
      ctx.restore();
      return;
    }
    if (ring) {                                              // state ring (pulses while driving)
      const grow = running ? 0.012 + 0.012 * (0.5 + 0.5 * Math.sin(now / 260)) : 0.012;
      this.bodyPath(ctx, grow + 0.012);
      ctx.strokeStyle = ring; ctx.globalAlpha = running ? 0.85 : 0.7; ctx.lineWidth = 2.2 * px; ctx.stroke(); ctx.globalAlpha = 1;
    }
    ctx.fillStyle = '#0b1220';                                // four wheels
    const wl = Math.min(0.07, hl * 0.6), ww = 0.032;
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
      ctx.beginPath(); ctx.roundRect(sx * (hl - wl / 2 - 0.012) - wl / 2, sy * (hw + 0.004) - ww / 2, wl, ww, 0.008); ctx.fill();
    }
    const body = ctx.createLinearGradient(-hl, 0, hl, 0);
    body.addColorStop(0, pal['robot-2']); body.addColorStop(1, pal.robot);
    this.bodyPath(ctx);
    ctx.fillStyle = body; ctx.fill();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.85)'; ctx.lineWidth = 1.4 * px; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(hl - 0.004, -hw * 0.72); ctx.lineTo(hl - 0.004, hw * 0.72);   // front bumper light
    ctx.strokeStyle = '#ffe066'; ctx.lineWidth = Math.max(0.012, 2.5 * px); ctx.lineCap = 'round'; ctx.stroke();
    ctx.beginPath(); ctx.arc(-hl * 0.18, 0, Math.min(0.036, hw * 0.45), 0, TAU);              // LiDAR
    ctx.fillStyle = '#0b1220'; ctx.fill(); ctx.strokeStyle = 'rgba(255,255,255,.55)'; ctx.lineWidth = px; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(hl * 0.78, 0); ctx.lineTo(hl * 0.36, hw * 0.42); ctx.lineTo(hl * 0.36, -hw * 0.42); ctx.closePath();   // heading
    ctx.fillStyle = 'rgba(255, 255, 255, 0.92)'; ctx.fill();
    ctx.restore();
  }

  shownSigma() {                    // with drift alerts switched off the estimate can grow without limit: draw at most 2x the limit
    return Math.min(this.sigma, this.maxSigma * 2);
  }

  drawRobotLabel(ctx) {
    if (this.mode === 'offline' || this.sigma < 0.06) return;
    const p = this.toScreen(this.pose.x, this.pose.y);
    this.chip(ctx, `±${Math.round(this.sigma * 100)} cm`, p.x, p.y - (this.robot.half_length + this.shownSigma()) * this.view.scale - 16,
      this.sigma > this.maxSigma * 0.8 ? this.pal.red : this.pal.label);
  }

  chip(ctx, text, x, y, color, bold) {
    ctx.font = `${bold ? '700' : '600'} 10.5px ${this.font}`;
    const w = ctx.measureText(text).width + 10;
    ctx.beginPath(); ctx.roundRect(x - w / 2, y - 8, w, 16, 6);
    ctx.fillStyle = this.pal.chip; ctx.fill();
    ctx.fillStyle = color || this.pal.text; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(text, x, y + 0.5);
  }

  drawGoal(ctx, now) {
    const g = this.plan.goal;
    if (!g) return;
    const p = this.toScreen(g[0], g[1]), color = this.pal.goal, pending = this.plan.pending_goal;
    ctx.save();
    ctx.globalAlpha = pending ? 0.45 : 1;
    if (!pending) {
      const phase = (now / 1500) % 1;
      ctx.beginPath(); ctx.arc(p.x, p.y, 6 + 16 * phase, 0, TAU);
      ctx.strokeStyle = color; ctx.globalAlpha = 0.55 * (1 - phase); ctx.lineWidth = 2; ctx.stroke(); ctx.globalAlpha = 1;
    }
    ctx.beginPath(); ctx.arc(p.x, p.y, 5, 0, TAU); ctx.fillStyle = color; ctx.fill();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(p.x, p.y - 4); ctx.lineTo(p.x, p.y - 28);
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.lineCap = 'round'; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(p.x + 1, p.y - 28); ctx.lineTo(p.x + 17, p.y - 22.5); ctx.lineTo(p.x + 1, p.y - 17); ctx.closePath();
    ctx.fillStyle = color; ctx.fill();
    ctx.restore();
    this.hover.push({sx: p.x, sy: p.y - 12, r: 18, html: `<b>Goal${pending ? ' (not planned yet)' : ''}</b><span>x ${g[0].toFixed(3)} m · y ${g[1].toFixed(3)} m</span>`});
  }

  drawPinLabels(ctx) {
    for (const pin of this.vision.pins || []) {
      const p = this.toScreen(pin.x, pin.y), green = (pin.cls || '').startsWith('green');
      const name = green ? 'green' : 'red';
      const isSurvivor = green && this.survivors.some((s) => Math.hypot(s.x - pin.x, s.y - pin.y) < 0.1);   // has its own S marker
      if (this.view.scale > 150 && !isSurvivor) this.chip(ctx, pin.conf ? `${name} ${Number(pin.conf).toFixed(2)}` : name, p.x + 26, p.y - 12, green ? this.pal.green : this.pal.red);
      this.hover.push({sx: p.x, sy: p.y, r: 12, html: `<b>${green ? 'Green block (survivor)' : 'Red block (obstacle)'}</b><span>x ${pin.x.toFixed(3)} m · y ${pin.y.toFixed(3)} m` +
        (pin.conf ? ` · confidence ${Number(pin.conf).toFixed(2)}` : '') + '</span>'});
    }
  }

  drawSurvivors(ctx, now) {
    for (const s of this.survivors) {
      const p = this.toScreen(s.x, s.y);
      const color = {found: this.pal.goal, secured: this.pal.green, unreachable: '#8a98a9'}[s.status] || this.pal.green;
      const cy = p.y - 24;
      if (s.status === 'found') {
        const phase = (now / 1100) % 1;
        ctx.beginPath(); ctx.arc(p.x, cy, 12 + 12 * phase, 0, TAU);
        ctx.strokeStyle = color; ctx.globalAlpha = 0.6 * (1 - phase); ctx.lineWidth = 2; ctx.stroke(); ctx.globalAlpha = 1;
      }
      ctx.beginPath(); ctx.moveTo(p.x, p.y - 2); ctx.lineTo(p.x - 7, cy + 8); ctx.lineTo(p.x + 7, cy + 8); ctx.closePath();   // pin tail
      ctx.fillStyle = color; ctx.fill();
      ctx.beginPath(); ctx.arc(p.x, cy, 12, 0, TAU); ctx.fillStyle = color; ctx.fill();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
      ctx.fillStyle = '#04261a'; ctx.font = `800 10.5px ${this.font}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('S' + s.id, p.x, cy + 0.5);
      this.hover.push({sx: p.x, sy: cy, r: 16, html: `<b>Survivor S${s.id} · ${s.status}</b><span>x ${s.x.toFixed(3)} m · y ${s.y.toFixed(3)} m · found ${s.found_clock}</span>`});
    }
  }

  drawToolPreview(ctx, px) {
    const c = this.cursor;
    if (!c || !this.map || this.drag) return;
    if ((this.tool === 'paint' || this.tool === 'erase') && this.inMap(c)) {
      const m = this.map, brush = Number(this.opts.brush || 0);
      const x = m.origin[0] + (Math.floor((c.x - m.origin[0]) / m.res) + 0.5) * m.res;
      const y = m.origin[1] + (Math.floor((c.y - m.origin[1]) / m.res) + 0.5) * m.res;
      ctx.beginPath(); ctx.arc(x, y, (brush + 0.5) * m.res, 0, TAU);
      ctx.fillStyle = this.tool === 'paint' ? 'rgba(127, 147, 173, 0.35)' : 'rgba(255, 89, 100, 0.22)'; ctx.fill();
      ctx.strokeStyle = this.tool === 'paint' ? this.pal['wall-edge'] : this.pal.red; ctx.lineWidth = 1.2 * px; ctx.stroke();
    }
  }

  drawMeasure(ctx) {
    const m = this.measure;
    if (!m) return;
    const a = this.toScreen(m.a.x, m.a.y), b = this.toScreen(m.b.x, m.b.y);
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y);
    ctx.strokeStyle = this.pal.text; ctx.lineWidth = 1.5; ctx.setLineDash([6, 4]); ctx.stroke(); ctx.setLineDash([]);
    for (const p of [a, b]) { ctx.beginPath(); ctx.arc(p.x, p.y, 3.5, 0, TAU); ctx.fillStyle = this.pal.text; ctx.fill(); }
    const d = Math.hypot(m.b.x - m.a.x, m.b.y - m.a.y);
    if (d > 0.005) this.chip(ctx, `${d.toFixed(3)} m   (Δx ${(m.b.x - m.a.x).toFixed(3)}, Δy ${(m.b.y - m.a.y).toFixed(3)})`, (a.x + b.x) / 2, (a.y + b.y) / 2 - 14, this.pal.text, true);
  }

  drawGizmo(ctx) {
    const th = this.view.rot * Math.PI / 180, x0 = 30, y0 = this.size.h - 62, len = 24;
    const arrow = (dx, dy, color, label) => {
      const ex = x0 + (dx * Math.cos(th) - dy * Math.sin(th)) * len, ey = y0 - (dx * Math.sin(th) + dy * Math.cos(th)) * len;
      ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(ex, ey); ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.lineCap = 'round'; ctx.stroke();
      ctx.fillStyle = color; ctx.font = `700 10.5px ${this.font}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(label, ex + (ex - x0) * 0.42, ey + (ey - y0) * 0.42);
    };
    arrow(1, 0, '#ff6b76', 'x'); arrow(0, 1, '#34d399', 'y');
    if (this.view.rot) this.chip(ctx, `view turned ${this.view.rot}°`, x0 + 52, y0 - 34, this.pal.goal);
  }

  snapshot() { return this.canvas.toDataURL('image/png'); }

  stats() {                                                  // smoothness self-check
    const g = [...this.frameGaps].sort((a, b) => a - b);
    return {frames: this.frames, median: g[g.length >> 1] || 0, p95: g[Math.floor(g.length * 0.95)] || 0, max: g[g.length - 1] || 0,
      delay: this.delay, samples: this.samples.length};
  }
}
