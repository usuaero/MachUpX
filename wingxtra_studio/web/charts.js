// Small dependency-free SVG line charts: one y-axis, recessive grid, hover crosshair and tooltip.

const NS = 'http://www.w3.org/2000/svg';
const COLORS = ['var(--s1)', 'var(--s2)', 'var(--s3)', 'var(--s4)'];

function el(name, attrs = {}, parent) {
  const node = document.createElementNS(NS, name);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  if (parent) parent.appendChild(node);
  return node;
}

function niceTicks(min, max, count = 5) {
  if (min === max) { min -= 1; max += 1; }
  const raw = (max - min) / count;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw);
  const lo = Math.floor(min / step) * step;
  const hi = Math.ceil(max / step) * step;
  const ticks = [];
  for (let v = lo; v <= hi + step * 1e-6; v += step) ticks.push(Math.abs(v) < step * 1e-9 ? 0 : v);
  return { lo, hi, ticks, step };
}

export function fmt(v, digits) {
  if (v === null || v === undefined || Number.isNaN(v)) return '–';
  if (digits === undefined) {
    const a = Math.abs(v);
    digits = a >= 100 ? 0 : a >= 10 ? 1 : a >= 1 ? 2 : 3;
  }
  return v.toFixed(digits).replace('-', '−');
}

function tickLabel(v, step) {
  const digits = step >= 1 ? 0 : Math.min(4, Math.ceil(-Math.log10(step)) + (String(step).includes('25') ? 1 : 0));
  return v.toFixed(digits).replace('-', '−');
}

/**
 * series: [{name, points: [[x, y], ...], dashed}]   markers: [{x, y, label}]
 */
export function lineChart(container, { title, sub, xLabel, yLabel, series, markers = [], height = 250 }) {
  container.classList.add('chart');
  container.innerHTML = '';
  if (title) container.insertAdjacentHTML('beforeend', `<h4>${title}</h4>`);
  if (sub) container.insertAdjacentHTML('beforeend', `<div class="sub">${sub}</div>`);
  if (series.length > 1) {
    const legend = series.map((s, i) => `<span><i style="background:${COLORS[i]}"></i>${s.name}</span>`).join('');
    container.insertAdjacentHTML('beforeend', `<div class="legend">${legend}</div>`);
  }

  // Drawn at the container's real pixel width so text is never scaled.
  const W = Math.max(300, Math.round(container.clientWidth) || 520), H = height, m = { l: 52, r: 16, t: 10, b: 38 };
  const all = series.flatMap((s) => s.points);
  const xs = all.map((p) => p[0]), ys = all.map((p) => p[1]).concat(markers.map((k) => k.y));
  const X = niceTicks(Math.min(...xs), Math.max(...xs), W < 420 ? 4 : 6);
  const Y = niceTicks(Math.min(...ys), Math.max(...ys), 5);
  const px = (x) => m.l + ((x - X.lo) / (X.hi - X.lo)) * (W - m.l - m.r);
  const py = (y) => H - m.b - ((y - Y.lo) / (Y.hi - Y.lo)) * (H - m.t - m.b);

  const svg = el('svg', { viewBox: `0 0 ${W} ${H}`, role: 'img', 'aria-label': title || '' }, container);
  for (const t of Y.ticks) {
    el('line', { x1: m.l, x2: W - m.r, y1: py(t), y2: py(t), stroke: t === 0 ? 'var(--axis)' : 'var(--grid)', 'stroke-width': 1 }, svg);
    el('text', { x: m.l - 8, y: py(t) + 4, 'text-anchor': 'end', class: 'tick' }, svg).textContent = tickLabel(t, Y.step);
  }
  for (const t of X.ticks) {
    el('text', { x: px(t), y: H - m.b + 16, 'text-anchor': 'middle', class: 'tick' }, svg).textContent = tickLabel(t, X.step);
  }
  el('line', { x1: m.l, x2: W - m.r, y1: H - m.b, y2: H - m.b, stroke: 'var(--axis)', 'stroke-width': 1 }, svg);
  el('text', { x: (m.l + W - m.r) / 2, y: H - 4, 'text-anchor': 'middle', class: 'axis-label' }, svg).textContent = xLabel;
  el('text', { x: 12, y: (m.t + H - m.b) / 2, 'text-anchor': 'middle', class: 'axis-label',
    transform: `rotate(-90 12 ${(m.t + H - m.b) / 2})` }, svg).textContent = yLabel;

  series.forEach((s, i) => {
    const d = s.points.map((p, k) => `${k ? 'L' : 'M'}${px(p[0]).toFixed(1)},${py(p[1]).toFixed(1)}`).join('');
    el('path', { d, fill: 'none', stroke: COLORS[i], 'stroke-width': 2, 'stroke-linejoin': 'round', 'stroke-linecap': 'round',
      ...(s.dashed ? { 'stroke-dasharray': '5 4' } : {}) }, svg);
  });
  for (const k of markers) {
    el('circle', { cx: px(k.x), cy: py(k.y), r: 4.5, fill: 'var(--ink)', stroke: 'var(--surface)', 'stroke-width': 2 }, svg);
    const right = px(k.x) < W - 110;
    el('text', { x: px(k.x) + (right ? 8 : -8), y: py(k.y) - 8, 'text-anchor': right ? 'start' : 'end', class: 'tick',
      style: 'fill: var(--ink-2)' }, svg).textContent = k.label;
  }

  // Hover layer: snap to the nearest x of the first series.
  const cross = el('line', { y1: m.t, y2: H - m.b, stroke: 'var(--axis)', 'stroke-width': 1, visibility: 'hidden' }, svg);
  const dots = series.map((_, i) => el('circle', { r: 4, fill: COLORS[i], stroke: 'var(--surface)', 'stroke-width': 2, visibility: 'hidden' }, svg));
  const tip = document.createElement('div');
  tip.className = 'tip';
  tip.hidden = true;
  container.appendChild(tip);
  const hit = el('rect', { x: m.l, y: m.t, width: W - m.l - m.r, height: H - m.t - m.b, fill: 'transparent' }, svg);

  const nearest = (pts, x) => pts.reduce((best, p) => (Math.abs(p[0] - x) < Math.abs(best[0] - x) ? p : best), pts[0]);
  hit.addEventListener('pointermove', (ev) => {
    const box = svg.getBoundingClientRect();
    const sx = ((ev.clientX - box.left) / box.width) * W;
    const x = X.lo + ((sx - m.l) / (W - m.l - m.r)) * (X.hi - X.lo);
    const anchor = nearest(series[0].points, x);
    cross.setAttribute('x1', px(anchor[0])); cross.setAttribute('x2', px(anchor[0]));
    cross.setAttribute('visibility', 'visible');
    let rows = '';
    series.forEach((s, i) => {
      const p = nearest(s.points, anchor[0]);
      dots[i].setAttribute('cx', px(p[0])); dots[i].setAttribute('cy', py(p[1])); dots[i].setAttribute('visibility', 'visible');
      rows += `<div><i style="background:${COLORS[i]}"></i>${series.length > 1 ? s.name + ': ' : ''}<b>${fmt(p[1])}</b></div>`;
    });
    tip.innerHTML = `<div class="x">${xLabel}: ${fmt(anchor[0])}</div>${rows}`;
    tip.hidden = false;
    const cbox = container.getBoundingClientRect();
    const left = (px(anchor[0]) / W) * box.width + (box.left - cbox.left);
    const flip = left > cbox.width - 160;
    tip.style.left = `${flip ? left - tip.offsetWidth - 12 : left + 12}px`;
    tip.style.top = `${ev.clientY - cbox.top - 10}px`;
  });
  hit.addEventListener('pointerleave', () => {
    cross.setAttribute('visibility', 'hidden');
    dots.forEach((d) => d.setAttribute('visibility', 'hidden'));
    tip.hidden = true;
  });
}
