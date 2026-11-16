import { Viewer } from '/viewer.js';
import { lineChart, fmt } from '/charts.js';

const $ = (sel) => document.querySelector(sel);
const state = { meta: null, req: null, sheet: null, sheetResult: null, mode: 'req', design: null, analysis: null, mesh: null, tab: 'review', seq: 0 };
const viewer = new Viewer($('#viewer'));
const STORE = 'wingxtra-studio-v1';

// ------------------------------------------------------------------ field definitions

const CONFIGS = [['conventional', 'Fixed-wing'], ['flying_wing', 'Flying wing'], ['vtol_quadplane', 'VTOL']];

const TAILS = [['conventional', 'Conventional'], ['v_tail', 'V-tail']];
const isVTail = (src) => src.tail === 'v_tail';

const REQ_FIELDS = [
  { group: 'Mission' },
  ['payload_kg', 'Payload', 'kg', 0.1],
  ['cruise_speed_ms', 'Cruise speed', 'm/s', 1],
  ['stall_speed_ms', 'Stall speed', 'm/s', 0.5],
  ['endurance_min', 'Endurance', 'min', 5],
  ['cruise_altitude_m', 'Altitude', 'm', 100],
  ['hover_time_min', 'Hover time', 'min', 0.5, 'vtol_quadplane'],
  { group: 'Wing' },
  ['aspect_ratio', 'Aspect ratio', '', 0.5],
  ['taper_ratio', 'Taper ratio', '', 0.05],
  ['wing_airfoil', 'Airfoil', 'airfoil'],
  ['static_margin', 'Static margin', 'MAC', 0.01],
  ['tail', 'Tail', TAILS, null, (src) => src.config !== 'flying_wing'],
  { group: 'Technology assumptions', collapsed: true },
  ['battery_wh_per_kg', 'Battery energy', 'Wh/kg', 10],
  ['battery_usable_fraction', 'Usable capacity', '', 0.05],
  ['structure_fraction', 'Structure fraction', '', 0.01],
  ['avionics_kg', 'Avionics mass', 'kg', 0.05],
  ['avionics_power_w', 'Avionics power', 'W', 1],
  ['prop_efficiency', 'Propeller efficiency', '', 0.01],
  ['drive_efficiency', 'Motor and ESC efficiency', '', 0.01],
  ['climb_rate_ms', 'Climb rate', 'm/s', 0.5],
  ['rotor_disk_loading_nm2', 'Rotor disk loading', 'N/m²', 10, 'vtol_quadplane'],
];

const SHEET_FIELDS = [
  { group: 'Mission requirements' },
  ['endurance_min', 'Endurance', 'min', 5],
  ['range_km', 'Range', 'km', 5],
  ['stall_margin_ms', 'Stall margin', 'm/s', 0.5],
  ['min_airspeed_ms', 'Min airspeed', 'm/s', 0.5],
  ['cruise_airspeed_ms', 'Cruise airspeed', 'm/s', 0.5],
  { group: 'Payload' },
  ['payload_kg', 'Payload weight', 'kg', 0.1],
  ['payload_power_w', 'Payload power', 'W', 1],
  { group: 'Aircraft' },
  ['airframe_kg', 'Airframe weight', 'kg', 0.1],
  ['avionics_power_w', 'Avionics power', 'W', 1],
  ['fuselage_cd', 'Fuselage drag coeff.', '', 0.001],
  ['zero_lift_cd', 'Zero-lift drag coeff.', '', 0.001],
  ['air_density', 'Air density', 'kg/m³', 0.005],
  { group: 'Propulsion / batteries' },
  ['battery_capacity_mah', 'Battery pack capacity', 'mAh', 500],
  ['energy_density_wh_kg', 'Energy density', 'Wh/kg', 5],
  ['cell_voltage', 'Cell voltage', 'V', 0.1],
  ['cells_in_series', 'Cells in series', 's', 1],
  ['battery_count', 'Number of batteries', '', 1],
  ['capacity_used_pct', 'Total capacity used', '%', 5],
  ['overall_efficiency_pct', 'Overall efficiency', '%', 1],
  { group: 'Main wing' },
  ['wing_loading_g_dm2', 'Wing loading', 'g/dm²', 1],
  ['aspect_ratio', 'Aspect ratio', '', 0.1],
  ['taper_ratio', 'Taper ratio', '', 0.05],
  ['oswald_efficiency', 'Oswald efficiency', '', 0.01],
  ['max_lift_coefficient', 'Max lift coefficient', '', 0.01],
  ['wing_airfoil', 'Airfoil', 'airfoil'],
  { group: 'Horizontal stabilizer' },
  ['h_volume_ratio', 'Volume ratio (0.3–0.6)', '', 0.01],
  ['h_moment_arm_m', 'Moment arm', 'm', 0.05],
  ['h_aspect_ratio', 'Aspect ratio', '', 0.1],
  ['h_taper_ratio', 'Taper ratio', '', 0.05],
  { group: 'Vertical stabilizer' },
  ['v_volume_ratio', 'Volume ratio (0.02–0.05)', '', 0.001],
  ['v_moment_arm_m', 'Moment arm', 'm', 0.05],
  ['v_semispan_m', 'Semispan', 'm', 0.01],
  ['v_taper_ratio', 'Taper ratio', '', 0.01],
  { group: 'Tail and balance' },
  ['tail', 'Tail', TAILS],
  ['vtail_span_m', 'V-tail span', 'm', 0.01, isVTail, 'auto'],
  ['vtail_average_chord_m', 'V-tail average chord', 'm', 0.005, isVTail, 'auto'],
  ['vtail_taper_ratio', 'V-tail taper ratio', '', 0.05, isVTail],
  ['static_margin', 'Static margin', 'MAC', 0.01],
];

const surfaceFields = (key, name, vertical) => [
  { group: name, key },
  [`${key}.span`, vertical ? 'Height' : 'Span', 'm', 0.01],
  [`${key}.root_chord`, 'Root chord', 'm', 0.005],
  [`${key}.tip_chord`, 'Tip chord', 'm', 0.005],
  [`${key}.sweep_deg`, 'Sweep', 'deg', 1],
  ...(vertical ? [] : [[`${key}.dihedral_deg`, 'Dihedral', 'deg', 0.5], [`${key}.incidence_deg`, 'Incidence', 'deg', 0.25],
    [`${key}.washout_deg`, 'Tip twist', 'deg', 0.5]]),
  [`${key}.airfoil`, 'Airfoil (NACA)', 'text'],
  ...(key === 'wing' ? [] : [[`${key}.x`, 'Position x', 'm', 0.01]]),
  [`${key}.control_chord_fraction`, 'Control chord', '', 0.05],
];

const get = (obj, path) => path.split('.').reduce((o, k) => (o == null ? o : o[k]), obj);
const set = (obj, path, value) => {
  const keys = path.split('.');
  keys.slice(0, -1).reduce((o, k) => o[k], obj)[keys.at(-1)] = value;
};
const round = (v, step) => (typeof v === 'number' ? +v.toFixed(step && step < 0.01 ? 4 : 3) : v);

// ------------------------------------------------------------------ server

async function post(url, body) {
  const res = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) });
  if (!res.ok) {
    let detail = `Request failed (${res.status})`;
    try {
      const j = await res.json();
      detail = typeof j.detail === 'string' ? j.detail
        : j.detail.map((d) => `${d.loc.slice(1).join(' ')}: ${d.msg}`).join('; ');
    } catch { /* keep the generic message */ }
    throw new Error(detail);
  }
  return res;
}

function busy(text) { const s = $('#status'); s.textContent = text; s.classList.toggle('busy', !!text); }

async function run(url, body, label) {
  const seq = ++state.seq;
  busy(label);
  try {
    const data = await (await post(url, body)).json();
    if (seq !== state.seq) return; // a newer request superseded this one
    Object.assign(state, { design: data.design, analysis: data.analysis, mesh: data.mesh, sheetResult: data.sheet || null });
    $('#error').hidden = true;
    save();
    renderAll();
  } catch (err) {
    if (seq !== state.seq) return;
    $('#error').textContent = err.message;
    $('#error').hidden = false;
  } finally {
    if (seq === state.seq) busy('');
  }
}

const sizeAircraft = () => { state.mode = 'req'; return run('/api/size', state.req, 'Sizing and analysing'); };
const buildSheet = () => { state.mode = 'sheet'; return run('/api/sheet', state.sheet, 'Building and analysing'); };
const analyseDesign = () => run('/api/analyze', state.design, 'Analysing');
const debounce = (fn, ms) => { let t; return () => { clearTimeout(t); t = setTimeout(fn, ms); }; };
const sizeSoon = debounce(sizeAircraft, 500);
const analyseSoon = debounce(analyseDesign, 500);
const sheetSoon = debounce(buildSheet, 500);

function save() {
  try { localStorage.setItem(STORE, JSON.stringify({ req: state.req, sheet: state.sheet, mode: state.mode })); } catch { /* storage unavailable */ }
}

// ------------------------------------------------------------------ forms

function fieldHtml([path, label, unit, step, , placeholder], value, prefix) {
  const id = `${prefix}-${path.replace(/\./g, '-')}`;
  let control;
  if (Array.isArray(unit)) {
    const opts = unit.map(([v, text]) => `<option value="${v}" ${v === value ? 'selected' : ''}>${text}</option>`).join('');
    control = `<select id="${id}" data-path="${path}" data-rerender="1">${opts}</select>`;
  } else if (unit === 'airfoil') {
    const opts = state.meta.airfoils.map((a) => `<option value="${a.code}" ${a.code === value ? 'selected' : ''}>${a.name}</option>`).join('');
    control = `<select id="${id}" data-path="${path}">${opts}</select>`;
  } else if (unit === 'text') {
    control = `<input id="${id}" data-path="${path}" type="text" value="${value}" maxlength="4" inputmode="numeric">`;
  } else {
    control = `<input id="${id}" data-path="${path}" type="number" step="${step}" value="${value == null ? '' : round(value, step)}"` +
      `${placeholder ? ` placeholder="${placeholder}" data-optional="1"` : ''}>` +
      (unit ? `<span class="unit">${unit}</span>` : '');
  }
  return `<div class="field"><label for="${id}">${label}</label><div class="control">${control}</div></div>`;
}

function buildForm(fields, source, prefix) {
  let html = '', open = false;
  for (const f of fields) {
    if (!Array.isArray(f)) {
      if (open) { html += '</details>'; open = false; }
      if (f.collapsed) { html += `<details><summary>${f.group}</summary>`; open = true; } else html += `<h3>${f.group}</h3>`;
      continue;
    }
    if (f[4] && !(typeof f[4] === 'function' ? f[4](source) : f[4] === source.config)) continue;
    html += fieldHtml(f, get(source, f[0]), prefix);
  }
  return html + (open ? '</details>' : '');
}

function renderReqForm() {
  const seg = CONFIGS.map(([id, label]) => `<button type="button" data-config="${id}" aria-pressed="${state.req.config === id}">${label}</button>`).join('');
  const wasOpen = $('#req-form details')?.open;
  $('#req-form').innerHTML = `<h3>Configuration</h3><div class="segmented">${seg}</div>` + buildForm(REQ_FIELDS, state.req, 'r');
  if (wasOpen) $('#req-form details').open = true;
  $('#design-name').value = state.req.name;
}

function renderSheetForm() {
  $('#sheet-form').innerHTML = buildForm(SHEET_FIELDS, state.sheet, 's');
}

function renderGeoForm() {
  const d = state.design;
  if (!d) return;
  const fields = [...surfaceFields('wing', 'Wing', false)];
  if (d.htail) fields.push(...surfaceFields('htail', d.tail_type === 'v_tail' ? 'V-tail' : 'Horizontal tail', false));
  if (d.vtail) fields.push(...surfaceFields('vtail', d.htail ? 'Fin' : 'Winglets (each)', true));
  if (document.activeElement?.closest('#geo-form')) return; // do not rebuild under the user's cursor
  $('#geo-form').innerHTML = buildForm(fields, d, 'g');
}

function readInput(input) {
  if (input.type === 'number') {
    if (input.dataset.optional && input.value.trim() === '') return undefined; // blank means "work it out"
    const v = parseFloat(input.value); return Number.isFinite(v) ? v : null;
  }
  return input.value.trim();
}

$('#req-form').addEventListener('input', (ev) => {
  const v = readInput(ev.target);
  if (v === null || v === '') return;
  state.req[ev.target.dataset.path] = v;
  if (ev.target.dataset.rerender) { renderReqForm(); sizeAircraft(); } else sizeSoon();
});
$('#sheet-form').addEventListener('input', (ev) => {
  const v = readInput(ev.target);
  if (v === null || v === '') return;
  state.sheet[ev.target.dataset.path] = v === undefined ? null : v;
  if (ev.target.dataset.rerender) { renderSheetForm(); buildSheet(); } else sheetSoon();
});
$('#req-form').addEventListener('click', (ev) => {
  const cfg = ev.target.dataset?.config;
  if (!cfg) return;
  state.req.config = cfg;
  renderReqForm();
  sizeAircraft();
});
$('#geo-form').addEventListener('input', (ev) => {
  const v = readInput(ev.target);
  if (v === null || v === '' || (ev.target.type === 'text' && v.length !== 4)) return;
  set(state.design, ev.target.dataset.path, v);
  analyseSoon();
});
$('#design-name').addEventListener('change', (ev) => {
  const name = ev.target.value || 'My UAV';
  if (state.mode === 'sheet') state.sheet.name = name; else state.req.name = name;
  if (state.design) {
    state.design.requirements.name = name;
    if (state.design.sheet_inputs) state.design.sheet_inputs.name = name;
  }
  save();
});

function tabs(root, onChange) {
  root.addEventListener('click', (ev) => {
    const btn = ev.target.closest('button[data-pane]');
    if (!btn) return;
    root.querySelectorAll('button').forEach((b) => b.setAttribute('aria-selected', b === btn));
    onChange(btn.dataset.pane);
  });
}
tabs($('#input-tabs'), (pane) => {
  for (const id of ['req', 'sheet', 'geo']) $(`#pane-${id}`).hidden = pane !== id;
  // Opening a pane makes it the source of the design, so the inputs on screen match the aircraft shown.
  if (pane === 'sheet' && state.mode !== 'sheet') buildSheet();
  if (pane === 'req' && state.mode !== 'req') sizeAircraft();
});
tabs($('#result-tabs'), (pane) => { state.tab = pane; renderResult(); });

document.querySelector('.view-buttons').addEventListener('click', (ev) => {
  if (ev.target.dataset.view) viewer.view(ev.target.dataset.view);
});

// ------------------------------------------------------------------ export / open

function renderExportMenu() {
  const hints = { step: 'SolidWorks, Fusion, Onshape, FreeCAD, CATIA', stl: '3D printing, CFD meshing',
    machupx: 'Run the same model in MachUpX', json: 'Reopen later in Wingxtra Studio' };
  $('#export-menu').innerHTML = state.meta.exports.map((e) =>
    `<button type="button" data-format="${e.id}">${e.label} (.${e.ext})<small>${hints[e.id] || ''}</small></button>`).join('');
}
$('#export-btn').addEventListener('click', () => {
  const menu = $('#export-menu');
  menu.hidden = !menu.hidden;
  $('#export-btn').setAttribute('aria-expanded', !menu.hidden);
});
document.addEventListener('click', (ev) => { if (!ev.target.closest('.menu')) $('#export-menu').hidden = true; });
$('#export-menu').addEventListener('click', async (ev) => {
  const btn = ev.target.closest('button[data-format]');
  if (!btn || !state.design) return;
  $('#export-menu').hidden = true;
  busy('Exporting');
  try {
    const res = await post(`/api/export/${btn.dataset.format}`, state.design);
    const name = /filename="([^"]+)"/.exec(res.headers.get('Content-Disposition') || '')?.[1] || 'aircraft';
    const a = Object.assign(document.createElement('a'), { href: URL.createObjectURL(await res.blob()), download: name });
    a.click();
    URL.revokeObjectURL(a.href);
  } catch (err) {
    $('#error').textContent = err.message; $('#error').hidden = false;
  } finally { busy(''); }
});
$('#open-btn').addEventListener('click', () => $('#open-file').click());
$('#open-file').addEventListener('change', async (ev) => {
  const file = ev.target.files[0];
  ev.target.value = '';
  if (!file) return;
  try {
    const design = JSON.parse(await file.text());
    if (!design.requirements || !design.wing) throw new Error('not a design file');
    state.req = design.requirements;
    state.design = design;
    if (design.source === 'spreadsheet' && design.sheet_inputs) {
      state.sheet = design.sheet_inputs; state.mode = 'sheet'; renderSheetForm();
    } else state.mode = 'req';
    renderReqForm();
    analyseDesign();
  } catch {
    $('#error').textContent = 'That file is not a Wingxtra Studio design.'; $('#error').hidden = false;
  }
});

// ------------------------------------------------------------------ results

function tile(k, v, unit, s = '') {
  return `<div class="tile"><div class="k">${k}</div><div class="v">${v}<small>${unit}</small></div><div class="s">${s}</div></div>`;
}

function renderTiles() {
  const a = state.analysis, d = state.design, p = a.performance, g = a.geometry;
  const hover = p.hover ? tile('Hover power', fmt(p.hover.power_w, 0), 'W', `${fmt(p.hover.full_battery_hover_min, 1)} min on full battery`) : '';
  $('#tiles').innerHTML = [
    tile('Take-off mass', fmt(g.mass_kg, 2), 'kg', `${fmt(d.requirements.payload_kg, 2)} kg payload`),
    tile('Wingspan', fmt(g.span, 2), 'm', `${fmt(g.wing_area, 3)} m² wing, AR ${fmt(g.aspect_ratio, 1)}`),
    tile('Battery', fmt(g.battery_wh, 0), 'Wh', `${fmt(p.cruise.electrical_power_w, 0)} W in cruise`),
    tile('Endurance', fmt(p.cruise.endurance_min, 0), 'min', `${fmt(p.cruise.range_km, 0)} km at cruise`),
    tile('Stall speed', fmt(p.stall_speed_ms, 1), 'm/s', `CL max ${fmt(a.polar.CL_max, 2)}`),
    tile('Cruise L/D', fmt(p.cruise.LD, 1), '', `best ${fmt(p.glide_ratio, 1)}`),
    tile('Static margin', fmt(a.trim.static_margin * 100, 1), '%', `elevator ${fmt(a.trim.elevator, 1)}° to trim`),
    hover,
  ].join('');
}

const LABEL = { pass: 'Pass', warn: 'Review', fail: 'Fail' };

function reviewPane(a) {
  const s = a.summary;
  const headline = s.fail ? 'This design has problems that need fixing before it is built.'
    : s.warn ? 'This design should fly, with points worth reviewing.'
      : 'This design is predicted to fly well and meet every requirement.';
  const order = { fail: 0, warn: 1, pass: 2 };
  const rows = [...a.checks].sort((x, y) => order[x.status] - order[y.status]).map((c) => `
    <li><span class="pill ${c.status}">${LABEL[c.status]}</span><span class="title">${c.title}</span>
      <span class="value">${c.value}</span><span class="target">Target: ${c.target}</span>
      ${c.advice ? `<span class="advice">${c.advice}</span>` : ''}</li>`).join('');
  const notes = state.design.notes.map((n) => `<p class="note">${n}</p>`).join('');
  return `<div class="card"><div class="verdict"><span class="headline">${headline}</span>
      <span class="pill pass">${s.pass} pass</span>${s.warn ? `<span class="pill warn">${s.warn} to review</span>` : ''}
      ${s.fail ? `<span class="pill fail">${s.fail} failing</span>` : ''}</div>
    ${notes}<ul class="checks">${rows}</ul></div>`;
}

function sheetPane() {
  const r = state.sheetResult;
  if (!r) return { html: '<div class="empty">Open the Spreadsheet tab on the left to build a design from your sheet.</div>', draw() {} };
  const e = r.estimates;
  const rows = r.comparison.map((c) => [c.quantity, `${fmt(c.sheet)} ${c.unit}`, `${fmt(c.solver)} ${c.unit}`,
    c.difference_pct == null ? '–' : `${c.difference_pct > 0 ? '+' : ''}${fmt(c.difference_pct, 0)}%`]);
  const geo = [
    ['Total mass', `${fmt(e.total_mass_kg, 2)} kg`], ['Batteries weight', `${fmt(e.battery_kg, 2)} kg`],
    ['Payload and battery mass fraction', `${fmt(e.mass_fraction_pct, 1)}%`], ['Voltage', `${fmt(e.voltage, 1)} V`],
    ['Wing area', `${fmt(e.wing_area, 3)} m²`], ['Wing semispan', `${fmt(e.semispan, 3)} m`],
    ['Wing root / tip chord', `${fmt(e.root_chord, 3)} / ${fmt(e.tip_chord, 3)} m`],
    ['Horizontal stabilizer area', `${fmt(e.h_area, 4)} m²`], ['Horizontal stabilizer semispan', `${fmt(e.h_semispan, 3)} m`],
    ['Vertical stabilizer area', `${fmt(e.v_area, 4)} m²`],
    ['Vertical root / tip chord', `${fmt(e.v_root_chord, 3)} / ${fmt(e.v_tip_chord, 3)} m`],
    ['Required V-tail area', `${fmt(e.vtail_required_area, 4)} m²`], ['V-tail dihedral from formula', `${fmt(e.vtail_dihedral_deg, 1)}°`],
    ['Min-power / min-drag airspeed', `${fmt(e.min_power_airspeed, 1)} / ${fmt(e.min_drag_airspeed, 1)} m/s`],
    ['Cruise current draw', `${fmt(e.cruise_current_a, 2)} A`],
  ];
  return {
    html: `<p class="note">Left: what the sheet's formulas predict. Right: what the lifting-line solver predicts for the
        same aircraft. The solver computes wing drag and maximum lift itself; it uses your fuselage drag coefficient
        as entered. Geometry edits change the solver column only.</p>
      <div class="grid2"><div class="card"><h4>Sheet estimate against solver prediction</h4><div class="sub">At cruise airspeed</div>
        ${table(['Quantity', 'Spreadsheet', 'Solver', 'Difference'], rows)}</div>
      <div class="card"><h4>Spreadsheet results</h4><div class="sub">The sheet's calculated cells</div>
        ${table(['Quantity', 'Value'], geo)}</div></div>`,
    draw() {},
  };
}

function table(head, rows, numericFrom = 1, numericTo = Infinity) {
  const cls = (i) => (i >= numericFrom && i <= numericTo ? ' class="num"' : '');
  return `<div class="table-wrap"><table><thead><tr>${head.map((h, i) => `<th${cls(i)}>${h}</th>`).join('')}</tr></thead>
    <tbody>${rows.map((r) => `<tr>${r.map((c, i) => `<td${cls(i)}>${c}</td>`).join('')}</tr>`).join('')}</tbody></table></div>`;
}

const card = (id) => `<div class="card" id="${id}"></div>`;

function aeroPane(a) {
  const drag = a.drag_buildup.map((i) => [i.name, fmt(i.cda * 1e4, 1), fmt(i.cd, 4)]);
  return {
    html: `<p class="note">Lift, induced drag and moments come from the MachUpX numerical lifting-line solver
        (${a.trim.solver} solution). The polar ends where the most loaded wing section reaches its maximum lift.</p>
      <div class="grid2">${card('c-cl')}${card('c-polar')}${card('c-ld')}${card('c-load')}</div>
      <div class="card" style="margin-top:12px"><h4>Drag of non-lifting parts</h4>
        <div class="sub">Added to the lifting-line result; referenced to wing area</div>
        ${table(['Component', 'Drag area (cm²)', 'CD'], drag)}</div>`,
    draw() {
      const pts = a.polar.points, stall = pts.at(-1);
      lineChart($('#c-cl'), { title: 'Lift curve', sub: 'Untrimmed, controls neutral', xLabel: 'Angle of attack (deg)', yLabel: 'CL',
        series: [{ name: 'CL', points: pts.map((p) => [p.alpha, p.CL]) }],
        markers: [{ x: stall.alpha, y: stall.CL, label: `Stall ${fmt(stall.alpha, 1)}°` }, { x: a.trim.alpha, y: a.trim.CL, label: 'Cruise' }] });
      lineChart($('#c-polar'), { title: 'Drag polar', sub: 'Whole aircraft', xLabel: 'CD', yLabel: 'CL',
        series: [{ name: 'CL', points: pts.map((p) => [p.CD, p.CL]) }],
        markers: [{ x: a.trim.CD, y: a.trim.CL, label: 'Cruise' }] });
      lineChart($('#c-ld'), { title: 'Lift-to-drag ratio', sub: `Best ${fmt(a.polar.LD_max, 1)} at CL ${fmt(a.polar.CL_best_LD, 2)}`,
        xLabel: 'CL', yLabel: 'L/D', series: [{ name: 'L/D', points: pts.filter((p) => p.CL > 0).map((p) => [p.CL, p.LD]) }],
        markers: [{ x: a.trim.CL, y: a.trim.LD, label: 'Cruise' }] });
      const ld = a.trim.lift_distribution;
      lineChart($('#c-load'), { title: 'Spanwise lift at cruise', sub: 'Local lift coefficient x chord, against the ideal elliptical loading',
        xLabel: 'Span position (m)', yLabel: 'cl x chord (m)',
        series: [{ name: 'This wing', points: ld.y.map((y, i) => [y, ld.load[i]]) },
          { name: 'Elliptical', points: ld.y.map((y, i) => [y, ld.elliptical[i]]), dashed: true }] });
    },
  };
}

function stabilityPane(a) {
  const t = a.trim, s = t.stability, d = t.damping, c = t.control, modes = a.dynamics.modes;
  const names = { short_period: 'Short period', phugoid: 'Phugoid', dutch_roll: 'Dutch roll', roll: 'Roll subsidence', spiral: 'Spiral' };
  const modeRows = Object.entries(modes).map(([k, m]) => [names[k],
    m.oscillatory ? `${fmt(m.period_s, 2)} s` : '–',
    m.oscillatory ? fmt(m.damping_ratio, 3) : '–',
    m.time_to_half_or_double_s ? `${fmt(m.time_to_half_or_double_s, 2)} s to ${m.stable ? 'halve' : 'double'}` : '–',
    `<span class="pill ${m.stable ? 'pass' : 'warn'}">${m.stable ? 'Stable' : 'Unstable'}</span>`]);
  const row = (label, sym, v, meaning) => [label, sym, fmt(v, 3), meaning];
  const derivs = [
    row('Lift slope', 'CL,α', s['CL,a'], 'per rad'),
    row('Pitch stiffness', 'Cm,α', s['Cm,a'], 'negative is stable'),
    row('Pitch damping', 'Cm,q', d['Cm,qbar'], 'negative damps'),
    row('Dihedral effect', 'Cl,β', s['Cl,b'], 'negative is stable'),
    row('Weathercock stability', 'Cn,β', s['Cn,b'], 'positive is stable'),
    row('Side force', 'CY,β', s['Cy,b'], ''),
    row('Roll damping', 'Cl,p', d['Cl,pbar'], 'negative damps'),
    row('Yaw damping', 'Cn,r', d['Cn,rbar'], 'negative damps'),
    row('Elevator power', 'Cm,δe', c['Cm,delevator'], 'per rad'),
    row('Aileron power', 'Cl,δa', c['Cl,daileron'], 'per rad'),
    ...(c['Cn,drudder'] !== undefined ? [row('Rudder power', 'Cn,δr', c['Cn,drudder'], 'per rad')] : []),
  ];
  return {
    html: `<div class="grid2">
      <div class="card"><h4>Trim at cruise</h4><div class="sub">Level flight at ${fmt(state.design.requirements.cruise_speed_ms, 1)} m/s</div>
        ${table(['Quantity', 'Value'], [
          ['Angle of attack', `${fmt(t.alpha, 2)}°`], ['Elevator', `${fmt(t.elevator, 2)}°`],
          ['Lift coefficient', fmt(t.CL, 3)], ['Static margin', `${fmt(t.static_margin * 100, 1)}% MAC`],
          ['Neutral point', `${fmt(-t.neutral_point_x * 1000, 0)} mm aft of wing root quarter-chord`],
          ['Centre of gravity', `${fmt(-state.design.cg_x * 1000, 0)} mm aft of wing root quarter-chord`],
          ...(t.near_stall ? [['Elevator near stall', `${fmt(t.near_stall.elevator, 1)}°`]] : [])])}</div>
      <div class="card"><h4>Natural modes</h4><div class="sub">Linearised about cruise, controls fixed</div>
        ${table(['Mode', 'Period', 'Damping ratio', 'Rate', ''], modeRows, 1, 2)}</div>
      </div>
      <div class="card" style="margin-top:12px"><h4>Stability and control derivatives</h4>
        <div class="sub">Body axes, about the centre of gravity, rates non-dimensionalised by b/2V and c/2V</div>
        ${table(['Derivative', 'Symbol', 'Value', ''], derivs, 2, 2)}</div>`,
    draw() {},
  };
}

function performancePane(a) {
  const p = a.performance, c = p.curve, d = state.design;
  const avail = d.propulsion.motor_power_w;
  const rows = [
    ['Stall speed', `${fmt(p.stall_speed_ms, 1)} m/s`], ['Maximum level speed', p.max_speed_ms ? `${fmt(p.max_speed_ms, 1)} m/s` : 'cannot sustain level flight'],
    ['Best-endurance speed', `${fmt(p.best_endurance.speed, 1)} m/s for ${fmt(p.best_endurance.endurance_min, 0)} min`],
    ['Best-range speed', `${fmt(p.best_range.speed, 1)} m/s for ${fmt(p.best_range.range_km, 0)} km`],
    ['Maximum climb rate', `${fmt(p.max_climb_rate_ms, 1)} m/s at ${fmt(p.best_climb_speed_ms, 1)} m/s`],
    ['Minimum sink, power off', `${fmt(p.min_sink_ms, 2)} m/s`], ['Best glide ratio', fmt(p.glide_ratio, 1)],
    ['Wing loading', `${fmt(p.wing_loading_kg_m2, 1)} kg/m²`], ['Cruise motor', `${fmt(avail, 0)} W shaft, ${fmt(d.propulsion.prop_diameter * 39.37, 0)} in propeller`],
    ['Gust sensitivity', `${fmt(p.gust_load_factor_per_ms, 2)} g per m/s of vertical gust`],
    ...(p.hover ? [['Hover power', `${fmt(p.hover.power_w, 0)} W`], ['Lift rotors', `4 x ${fmt(d.lift_rotors.diameter * 39.37, 0)} in, ${fmt(p.hover.thrust_per_rotor_n / 9.81, 2)} kg peak thrust each`],
      ['Hover energy reserved', `${fmt(p.hover.energy_wh, 0)} Wh`]] : []),
  ];
  return {
    html: `<div class="grid2">${card('c-power')}${card('c-endurance')}${card('c-climb')}
      <div class="card"><h4>Performance summary</h4><div class="sub">At ${fmt(d.requirements.cruise_altitude_m, 0)} m, standard atmosphere</div>${table(['Quantity', 'Value'], rows)}</div></div>`,
    draw() {
      lineChart($('#c-power'), { title: 'Power required and available', sub: 'Shaft power in level flight', xLabel: 'Airspeed (m/s)', yLabel: 'Power (W)',
        series: [{ name: 'Required', points: c.map((q) => [q.speed, q.shaft_power_w]) },
          { name: 'Available', points: c.map((q) => [q.speed, avail]), dashed: true }] });
      lineChart($('#c-endurance'), { title: 'Endurance', sub: p.hover ? 'After the planned hover time' : 'On the usable battery capacity',
        xLabel: 'Airspeed (m/s)', yLabel: 'Endurance (min)', series: [{ name: 'Endurance', points: c.map((q) => [q.speed, q.endurance_min]) }],
        markers: [{ x: p.cruise.speed, y: p.cruise.endurance_min, label: 'Cruise' }] });
      lineChart($('#c-climb'), { title: 'Climb rate', sub: 'At full cruise-motor power', xLabel: 'Airspeed (m/s)', yLabel: 'Climb (m/s)',
        series: [{ name: 'Climb', points: c.map((q) => [q.speed, q.climb_rate_ms]) }] });
    },
  };
}

function simPane(a) {
  const r = a.dynamics.responses;
  return {
    html: `<p class="note">How the aircraft responds with no autopilot: the linearised equations of motion, built from the
        predicted derivatives and estimated inertia, flown through two standard test inputs.</p>
      <h3>Elevator doublet: 2° up for 0.5 s, then 2° down</h3>
      <div class="grid2">${card('s-pitch')}${card('s-alt')}${card('s-speed')}</div>
      <h3>Aileron pulse: 2° held for 0.5 s</h3>
      <div class="grid2">${card('s-bank')}${card('s-rates')}</div>`,
    draw() {
      const p = r.pitch_doublet, q = r.roll_pulse, zip = (t, y) => t.map((v, i) => [v, y[i]]);
      const early = (t, y, end) => zip(t, y).filter((pt) => pt[0] <= end);
      lineChart($('#s-pitch'), { title: 'Pitch response', sub: 'First 8 seconds: the short-period mode', xLabel: 'Time (s)', yLabel: 'Angle (deg)',
        series: [{ name: 'Pitch attitude', points: early(p.t, p.pitch_deg, 8) }, { name: 'Angle of attack', points: early(p.t, p.alpha_deg, 8) },
          { name: 'Elevator', points: early(p.t, p.elevator_deg, 8), dashed: true }] });
      lineChart($('#s-alt'), { title: 'Altitude', sub: 'The slow phugoid exchange of height and speed', xLabel: 'Time (s)', yLabel: 'Altitude change (m)',
        series: [{ name: 'Altitude', points: zip(p.t, p.altitude_m) }] });
      lineChart($('#s-speed'), { title: 'Airspeed', sub: 'Phugoid', xLabel: 'Time (s)', yLabel: 'Airspeed (m/s)',
        series: [{ name: 'Airspeed', points: zip(p.t, p.speed_ms) }] });
      lineChart($('#s-bank'), { title: 'Bank and sideslip', sub: 'Stick released at 1.5 s; any slow drift afterwards is the spiral mode', xLabel: 'Time (s)', yLabel: 'Angle (deg)',
        series: [{ name: 'Bank angle', points: zip(q.t, q.bank_deg) }, { name: 'Sideslip', points: zip(q.t, q.sideslip_deg) }] });
      lineChart($('#s-rates'), { title: 'Roll and yaw rate', sub: 'Dutch roll shows as the decaying ripple', xLabel: 'Time (s)', yLabel: 'Rate (deg/s)',
        series: [{ name: 'Roll rate', points: zip(q.t, q.roll_rate_dps) }, { name: 'Yaw rate', points: zip(q.t, q.yaw_rate_dps) }] });
    },
  };
}

function massPane() {
  const d = state.design, total = d.mass_kg, nose = d.fuselage.nose_x;
  const rows = d.masses.map((m) => [m.name, fmt(m.mass_kg, 3), `${fmt((100 * m.mass_kg) / total, 0)}%`, fmt((nose - m.x) * 1000, 0), m.note]);
  rows.push(['<b>Total</b>', `<b>${fmt(total, 3)}</b>`, '100%', `<b>${fmt((nose - d.cg_x) * 1000, 0)}</b>`, 'Centre of gravity']);
  const I = d.inertia;
  return {
    html: `<div class="card"><h4>Mass and balance</h4><div class="sub">Stations measured aft from the nose</div>
        ${table(['Item', 'Mass (kg)', 'Share', 'Station (mm)', ''], rows, 1, 3)}</div>
      <div class="card" style="margin-top:12px"><h4>Estimated moments of inertia</h4>
        <div class="sub">From the mass layout; replace with CAD values when available</div>
        ${table(['Axis', 'kg m²'], [['Roll, Ixx', fmt(I.Ixx, 3)], ['Pitch, Iyy', fmt(I.Iyy, 3)], ['Yaw, Izz', fmt(I.Izz, 3)]])}</div>`,
    draw() {},
  };
}

function renderResult() {
  const a = state.analysis;
  if (!a) { $('#result').innerHTML = '<div class="empty">Sizing your aircraft…</div>'; return; }
  const pane = { review: () => ({ html: reviewPane(a), draw() {} }), sheet: sheetPane, aero: () => aeroPane(a), stability: () => stabilityPane(a),
    performance: () => performancePane(a), sim: () => simPane(a), mass: massPane }[state.tab]();
  $('#result').innerHTML = pane.html;
  pane.draw();
}

function renderAll() {
  $('#tab-sheet').hidden = !state.sheetResult;
  if (!state.sheetResult && state.tab === 'sheet') {
    state.tab = 'review';
    document.querySelectorAll('#result-tabs button').forEach((b) => b.setAttribute('aria-selected', b.dataset.pane === 'review'));
  }
  $('#design-name').value = state.design.requirements.name;
  viewer.setAircraft(state.mesh, state.analysis.trim.neutral_point_x);
  renderTiles();
  renderGeoForm();
  renderResult();
}

// ------------------------------------------------------------------ start

async function start() {
  state.meta = await (await fetch('/api/meta')).json();
  let saved = null;
  try { saved = JSON.parse(localStorage.getItem(STORE) || 'null'); } catch { /* ignore */ }
  state.req = { ...state.meta.defaults, ...(saved?.req || {}) };
  state.sheet = { ...state.meta.sheet_defaults, ...(saved?.sheet || {}) };
  $('#presets').innerHTML = state.meta.presets.map((p) => `<button type="button" data-preset="${p.id}"><b>${p.label}</b><span>${p.blurb}</span></button>`).join('');
  $('#presets').addEventListener('click', (ev) => {
    const btn = ev.target.closest('button[data-preset]');
    if (!btn) return;
    state.req = { ...state.meta.presets.find((p) => p.id === btn.dataset.preset).requirements };
    renderReqForm();
    sizeAircraft();
  });
  renderExportMenu();
  renderReqForm();
  renderSheetForm();
  renderResult();
  sizeAircraft();
}

start();
