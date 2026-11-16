// Start-up: connect the live state stream to the map and the panels.

import {TOKEN, openEvents} from './api.js';
import {MapView} from './map.js';
import {CameraView} from './camera.js';
import {UI, showResult, toast} from './ui.js';

const $ = (sel) => document.querySelector(sel);
const tip = $('#tip');

const hooks = {
  cursor: (text) => { $('#cursor').textContent = text; },
  hint: (text) => { $('#hint').textContent = text; },
  toast: showResult,
  tool: (tool) => document.querySelectorAll('[data-tool]').forEach((b) => b.classList.toggle('active', b.dataset.tool === tool)),
  follow: (on) => $('#btn-follow').classList.toggle('active', on),
  pick: (kind, x, y) => {
    const [ix, iy] = kind === 'pick-cal' ? ['#cal-x', '#cal-y'] : ['#blk-x', '#blk-y'];
    $(ix).value = x.toFixed(3); $(iy).value = y.toFixed(3);
    toast(kind === 'pick-cal' ? `Real end position (${x.toFixed(3)}, ${y.toFixed(3)}) m: adjust if needed, then Finish run.`
      : `Block centre (${x.toFixed(3)}, ${y.toFixed(3)}) m: adjust if needed, then Add sample.`);
  },
  tip: (html, x, y) => {
    if (!html) { tip.hidden = true; return; }
    tip.innerHTML = html; tip.hidden = false;
    const stage = $('#stage').getBoundingClientRect();
    const left = Math.min(x + 16, stage.width - tip.offsetWidth - 8), top = Math.max(8, y - tip.offsetHeight - 10);
    tip.style.left = left + 'px'; tip.style.top = top + 'px';
  },
};

const map = new MapView($('#map'), hooks);
const camera = new CameraView($('#cam'), $('#cam-empty'));
const ui = new UI(map, camera);
map.setTool('pan');
window.__ugv = {map, ui, camera};                 // handy for checks from the browser console

if (!TOKEN) {
  const lost = $('#lost');
  lost.querySelector('h2').textContent = 'Open the console with its link';
  lost.querySelector('p').innerHTML = 'This page needs the link that <code>nav_main.py gui</code> prints in the terminal (it carries a one-time key).';
  lost.hidden = false;
} else {
  openEvents((data) => {
    for (const [name, value] of Object.entries(data)) {
      if (name === 'fast') map.pushFast(value);
      else ui.update(name, value);
    }
  }, (alive) => { $('#lost').hidden = alive; });
}
