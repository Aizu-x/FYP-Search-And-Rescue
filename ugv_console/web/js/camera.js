// Live camera: the UGV's own JPEG frames with the YOLO boxes of exactly that frame drawn on top.

import {fetchFrame} from './api.js';

const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

export class CameraView {
  constructor(canvas, empty, path = '/api/frame') {              // path: which camera (a team robot has its own)
    this.canvas = canvas; this.ctx = canvas.getContext('2d'); this.empty = empty; this.path = path;
    this.on = false; this.running = false; this.last = -1; this.shown = 0;
  }

  setOn(on) {
    if (on === this.on) return;
    this.on = on;
    this.empty.hidden = on && this.shown > 0;
    if (!on) { this.shown = 0; this.last = -1; this.empty.hidden = false; this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height); }
    if (on && !this.running) this.loop();
  }

  async loop() {
    this.running = true;
    while (this.on) {
      try {
        const frame = await fetchFrame(this.last, this.path);
        if (!this.on) { if (frame) frame.bitmap.close(); break; }
        if (frame) { this.draw(frame.bitmap, frame.meta); frame.bitmap.close(); this.last = frame.meta.id; }
        else await sleep(120);
      } catch (e) { await sleep(600); }
    }
    this.running = false;
  }

  draw(bitmap, meta) {
    const box = this.canvas.parentElement.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const w = Math.round(box.width * dpr), h = Math.round(box.width * dpr * meta.h / meta.w);
    if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
    this.canvas.parentElement.style.aspectRatio = `${meta.w} / ${meta.h}`;
    const ctx = this.ctx, k = w / meta.w;
    ctx.drawImage(bitmap, 0, 0, w, h);
    ctx.lineJoin = 'round';
    for (const b of meta.boxes || []) {
      const [x1, y1, x2, y2] = b.b.map((v) => v * k);
      const green = b.c.startsWith('green');
      const color = green ? '#2fd486' : '#ff5964';
      const corner = Math.min(14 * dpr, (x2 - x1) / 3, (y2 - y1) / 3);
      ctx.globalAlpha = b.u ? 1 : 0.75;
      ctx.fillStyle = green ? 'rgba(47, 212, 134, .13)' : 'rgba(255, 89, 100, .13)';
      ctx.fillRect(x1, y1, x2 - x1, y2 - y1);
      ctx.strokeStyle = color; ctx.lineWidth = 2.2 * dpr;
      if (!b.u) ctx.setLineDash([5 * dpr, 4 * dpr]);
      ctx.beginPath();                                           // corner brackets + thin outline
      for (const [cx, cy, sx, sy] of [[x1, y1, 1, 1], [x2, y1, -1, 1], [x1, y2, 1, -1], [x2, y2, -1, -1]]) {
        ctx.moveTo(cx + sx * corner, cy); ctx.lineTo(cx, cy); ctx.lineTo(cx, cy + sy * corner);
      }
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.lineWidth = 1 * dpr; ctx.globalAlpha *= 0.55; ctx.strokeRect(x1, y1, x2 - x1, y2 - y1); ctx.globalAlpha = b.u ? 1 : 0.75;
      let label = `${green ? 'green' : 'red'} ${b.p.toFixed(2)}`;
      if (b.r !== null && b.r !== undefined) label += ` · ${b.r.toFixed(2)} m`;
      if (b.n && !b.u) label += ` · ${b.n}`;
      ctx.font = `600 ${11 * dpr}px "Segoe UI", system-ui, sans-serif`;
      const tw = ctx.measureText(label).width + 10 * dpr, th = 17 * dpr;
      const lx = Math.min(Math.max(x1, 0), w - tw), ly = y1 - th - 3 * dpr < 0 ? y2 + 3 * dpr : y1 - th - 3 * dpr;
      ctx.fillStyle = color; ctx.beginPath(); ctx.roundRect(lx, ly, tw, th, 5 * dpr); ctx.fill();
      ctx.fillStyle = '#06130d'; ctx.textBaseline = 'middle'; ctx.textAlign = 'left';
      ctx.fillText(label, lx + 5 * dpr, ly + th / 2 + 0.5);
      ctx.globalAlpha = 1;
    }
    this.shown++;
    this.empty.hidden = true;
  }
}
