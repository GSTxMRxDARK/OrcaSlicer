// Smart Print assistant page. Talks to the app through window.wx.postMessage (requests) and
// HandleStudio (replies); the engine behind it is Slic3r::SmartPrint::Session.

const $ = (id) => document.getElementById(id);
const el = (tag, cls, text) => {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
};
const fmt = (n, digits = 0) => Number(n).toLocaleString(undefined, { minimumFractionDigits: digits, maximumFractionDigits: digits });
const sfText = (sf) => (sf >= 10 ? fmt(sf, 0) : fmt(sf, sf >= 3 ? 1 : 2));

function b64ToBuffer(b64) {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; ++i) bytes[i] = bin.charCodeAt(i);
  return bytes.buffer;
}

function minutesText(min) {
  min = Math.max(1, Math.round(min));
  return min >= 60 ? t("tier_hours", Math.floor(min / 60), min % 60) : t("tier_minutes", min);
}

// ---------------------------------------------------------------------------------------------
// Bridge to the app (or to the development server when opened over http).

const Bridge = {
  seq: 0,
  pending: new Map(),
  post(message) {
    if (window.wx && typeof window.wx.postMessage === "function") {
      window.wx.postMessage(JSON.stringify(message));
    } else if (location.protocol.startsWith("http")) {
      fetch("api", { method: "POST", body: JSON.stringify(message) })
        .then((r) => r.json())
        .then((replies) => replies.forEach((r) => HandleStudio(r)))
        .catch((e) => setStatus(String(e), true));
    }
  },
  request(req) {
    const id = ++this.seq;
    req.request_id = id;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject });
      this.post({ command: "smartprint_request", request: req });
    });
  },
  settle(result) {
    const p = this.pending.get(result.request_id);
    if (!p) return;
    this.pending.delete(result.request_id);
    if (result.error) p.reject(new Error(result.error));
    else p.resolve(result);
  }
};

function HandleStudio(value) {
  const msg = typeof value === "string" ? JSON.parse(value) : value;
  if (!msg || typeof msg !== "object") return;
  switch (msg.command) {
    case "init": onInit(msg); break;
    case "init_failed": setStatus(t("st_error", msg.error || ""), true); break;
    case "smartprint_result": Bridge.settle(msg.result); break;
    case "smartprint_progress": setProgress(msg.value); break;
    case "applied":
      if (msg.ok) setStatus(t("st_applied", msg.object || ""));
      else setStatus(t("st_apply_failed", msg.error || ""), true);
      break;
    case "theme_changed": SafetyScale.refresh(); refreshColors(); break;
  }
}

// ---------------------------------------------------------------------------------------------
// State

const S = {
  libs: null,
  context: {},
  viewer: null,
  useCase: null,
  environment: "indoor",
  impact: 1,
  ranking: [],
  filament: null,
  baseStructure: { walls: 2, infill: 0.15, pattern: "gyroid", top: 4, bottom: 4, layer_height: 0.2, line_width: 0.45 },
  structure: null,          // settings being simulated (selected option or the object's own)
  quality: "fast",
  drone: { motor: null, prop: null, battery: null, motor_count: 4, payload_g: 250, measured_thrust_g: 0, style: "freestyle", layout: null },
  selectedMotor: -1,
  custom: { supports: [], forces: [] },
  tool: null,
  tiers: null,
  tierId: null,
  tiersStale: false,
  analysis: null,
  caseId: null,
  power: null,
  analysisDirty: false,
  analysisBusy: false
};

const useCaseInfo = (id) => (S.libs.use_cases.use_cases || []).find((u) => u.id === id);
const analysisType = () => (S.useCase ? useCaseInfo(S.useCase).analysis : "none");

// ---------------------------------------------------------------------------------------------
// Status line

function setStatus(text, isError) {
  const s = $("status");
  s.textContent = text || "";
  s.classList.toggle("is-error", !!isError);
}
function setProgress(value) {
  const bar = $("progress");
  bar.hidden = value === null || value === undefined || value >= 1;
  $("progressFill").style.width = Math.round((value || 0) * 100) + "%";
}

// ---------------------------------------------------------------------------------------------
// Start-up

function onInit(msg) {
  S.libs = msg.libraries;
  S.context = msg.libraries.context || {};
  if (S.context.structure) Object.assign(S.baseStructure, S.context.structure);
  const printer = S.context.printer || {};
  $("hardened").checked = !!printer.hardened_nozzle;
  $("enclosure").checked = !!printer.enclosure;
  $("objectName").textContent = msg.object_name || (msg.mesh && msg.mesh.name) || "";

  if (msg.mesh && msg.mesh.num_faces > 0) {
    S.viewer.setMesh(new Float32Array(b64ToBuffer(msg.mesh.vertices)), new Uint32Array(b64ToBuffer(msg.mesh.indices)));
  } else {
    setStatus(t("st_no_mesh"), true);
  }
  renderUseCases();
  fillPowerSelects();
  setStatus("");
}

function renderUseCases() {
  const box = $("useCases");
  box.textContent = "";
  for (const uc of S.libs.use_cases.use_cases || []) {
    const b = el("button", "use");
    b.type = "button";
    b.setAttribute("role", "radio");
    b.setAttribute("aria-checked", String(uc.id === S.useCase));
    b.append(el("span", "name", t("uc_" + uc.id)), el("span", "desc", t("uc_" + uc.id + "_desc")));
    b.addEventListener("click", () => selectUseCase(uc.id));
    box.append(b);
  }
}

function renderEnvironments() {
  const box = $("environments");
  box.textContent = "";
  for (const env of S.libs.use_cases.environments || []) {
    const b = el("button", "chip", t("env_" + env.id));
    b.type = "button";
    b.setAttribute("role", "radio");
    b.setAttribute("aria-checked", String(env.id === S.environment));
    b.addEventListener("click", () => {
      S.environment = env.id;
      renderEnvironments();
      rankMaterials();
    });
    box.append(b);
  }
}

function selectUseCase(id) {
  S.useCase = id;
  const uc = useCaseInfo(id);
  S.environment = uc.default_environment || "indoor";
  S.impact = id === "drone_frame" ? 2 : 1;
  $("impact").value = String(S.impact);
  S.tiers = null;
  S.tierId = null;
  S.analysis = null;
  S.structure = null;
  S.filament = null; // re-pick the best match for the new use
  $("findBtn").textContent = t("sp_find");
  renderUseCases();
  renderEnvironments();
  $("viewerEmpty").hidden = true;
  $("stepUse").classList.add("is-done");
  $("stepWhere").hidden = false;
  $("stepMaterial").hidden = false;
  const type = analysisType();
  $("stepPower").hidden = type !== "drone";
  $("stepLoads").hidden = type !== "custom";
  $("stepOptions").hidden = false;
  $("optionsNum").textContent = type === "none" ? "4" : "5";
  renderTiers();
  renderExpectations();
  renderCases();
  S.viewer.setSafety(null);
  rankMaterials();
  if (type === "drone") {
    if (!S.drone.layout) detectLayout();
    else drawMarkers();
    updatePower();
  } else {
    drawMarkers();
  }
}

// ---------------------------------------------------------------------------------------------
// Materials

function printerPayload() {
  const p = S.context.printer || {};
  return { name: p.name || "", hardened_nozzle: $("hardened").checked, enclosure: $("enclosure").checked, max_nozzle_temp: p.max_nozzle_temp || 300 };
}

async function rankMaterials() {
  if (!S.useCase) return;
  setStatus(t("st_ranking"));
  try {
    const r = await Bridge.request({ cmd: "rank", use_case: S.useCase, environment: S.environment, impact: S.impact, printer: printerPayload() });
    S.ranking = r.materials;
    const keep = S.filament && S.ranking.find((m) => m.preset === S.filament.preset && m.usable);
    S.filament = keep || S.ranking.find((m) => m.usable) || null;
    renderMaterials();
    setStatus("");
    inputsChanged(true);
  } catch (e) {
    setStatus(t("st_error", e.message), true);
  }
}

function renderMaterials() {
  const box = $("materials");
  box.textContent = "";
  if (!S.ranking.length) {
    box.append(el("p", "hint", t("mat_none")));
    return;
  }
  const best = S.ranking.find((m) => m.usable);
  for (const m of S.ranking) {
    const b = el("button", "mat" + (m.usable ? "" : " is-unusable"));
    b.type = "button";
    b.setAttribute("role", "radio");
    b.setAttribute("aria-checked", String(S.filament && S.filament.preset === m.preset));
    b.append(el("span", "preset", m.preset));
    b.append(m === best ? el("span", "best-tag", t("mat_best")) : el("span", "family", m.material.name));
    const notes = el("span", "notes");
    const parts = [];
    for (const s of m.strengths.slice(0, 3)) parts.push(["good", t("mat_" + s)]);
    for (const i of m.issues) parts.push([m.usable && !["needs_enclosure", "too_hot"].includes(i) ? "" : "bad", t("mat_" + i)]);
    parts.forEach(([cls, text], i) => {
      if (i) notes.append(document.createTextNode(", "));
      notes.append(el("span", cls, text));
    });
    b.append(notes);
    b.addEventListener("click", () => {
      S.filament = m;
      renderMaterials();
      $("stepMaterial").classList.add("is-done");
      markTiersStale();
      inputsChanged(true);
    });
    box.append(b);
  }
}

function filamentPayload() {
  const f = S.filament;
  return f ? { preset: f.preset, type: f.type, vendor: f.vendor, density: f.density, max_volumetric_speed: f.max_volumetric_speed } : null;
}

// ---------------------------------------------------------------------------------------------
// Drone power system

function optionList(select, items, label, selected, groupOf) {
  select.textContent = "";
  const groups = new Map();
  for (const item of items) {
    const g = groupOf ? groupOf(item) : "";
    if (!groups.has(g)) groups.set(g, []);
    groups.get(g).push(item);
  }
  for (const [g, list] of groups) {
    const parent = g ? Object.assign(document.createElement("optgroup"), { label: g }) : select;
    for (const item of list) {
      const o = document.createElement("option");
      o.value = item.id;
      o.textContent = label(item);
      o.selected = item.id === selected;
      parent.append(o);
    }
    if (parent !== select) select.append(parent);
  }
}

function motorGroup(m) {
  if (m.source === "user") return t("group_yours");
  if (m.brand === "Typical") return t("group_typical");
  return m.brand;
}

function fillPowerSelects() {
  const L = S.libs;
  const pickDefault = (list, id) => (list.find((x) => x.id === id) || list[0] || {}).id;
  S.drone.motor = S.drone.motor || pickDefault(L.motors, "class-2207-1750");
  S.drone.prop = S.drone.prop || pickDefault(L.props, "p-51466-3");
  S.drone.battery = S.drone.battery || pickDefault(L.batteries, "b-6s-1300");
  optionList($("motor"), L.motors, (m) => (m.brand && m.brand !== "Typical" ? m.brand + " " : "") + m.name, S.drone.motor, motorGroup);
  optionList($("prop"), L.props, (p) => p.name, S.drone.prop);
  optionList($("battery"), L.batteries, (b) => b.name, S.drone.battery);
}

function currentMotor() { return S.libs.motors.find((m) => m.id === S.drone.motor) || {}; }

function mountRadius() {
  // Mirrors mount_radius_for() in Advisor.cpp: the motor base is a bit wider than the stator.
  const st = String(currentMotor().stator || "");
  const d = parseInt(st.slice(0, 2), 10);
  return st.length >= 4 && d > 0 ? Math.min(Math.max(d * 0.5 + 2, 4), 40) : 12;
}

function systemPayload() {
  return { motor: S.drone.motor, prop: S.drone.prop, battery: S.drone.battery, motor_count: S.drone.motor_count,
           payload_g: S.drone.payload_g, measured_thrust_g: S.drone.measured_thrust_g || 0 };
}

async function detectLayout() {
  try {
    const r = await Bridge.request({ cmd: "layout", motor_count: S.drone.motor_count, mount_radius: mountRadius() });
    S.drone.layout = r.layout;
    drawMarkers();
    inputsChanged(false);
  } catch (e) {
    setStatus(t("st_error", e.message), true);
  }
}

async function updatePower() {
  const frame = S.analysis ? S.analysis.mass_g : 0;
  try {
    const r = await Bridge.request({ cmd: "power", system: systemPayload(), frame_g: frame });
    S.power = r.report;
    renderPower();
  } catch (e) {
    setStatus(t("st_error", e.message), true);
  }
}

function renderPower() {
  const box = $("power");
  box.textContent = "";
  const r = S.power;
  if (!r) return;
  const grid = el("div", "power-grid");
  const cell = (value, label) => {
    const d = el("div");
    d.append(el("b", "", value), el("span", "", label));
    grid.append(d);
  };
  cell(t("pw_grams", fmt(r.full.thrust_g)), t("pw_thrust"));
  cell(t("pw_grams", fmt(r.all_up_weight_g)), t("pw_auw"));
  cell(t("pw_ratio", fmt(r.thrust_to_weight, 1)), t("pw_twr"));
  cell(r.can_hover ? t("pw_percent", fmt(r.hover.throttle * 100)) : "—", t("pw_hover"));
  cell(t("pw_amps", fmt(r.full.current_a)), t("pw_current"));
  cell(r.can_hover ? t("pw_minutes", fmt(r.flight_time_min, 1)) : "—", t("pw_time"));
  box.append(grid);
  if (r.warnings.length) {
    const ul = el("ul", "warnings");
    for (const w of r.warnings) {
      let v = w.value;
      if (w.code === "high_hover_throttle") v = fmt(v * 100);
      else if (w.code === "low_thrust_to_weight" || w.code === "prop_size_mismatch") v = fmt(v, 1);
      else v = fmt(v);
      ul.append(el("li", "", t("warn_" + w.code, v)));
    }
    box.append(ul);
  }
}

function bindPowerInputs() {
  const changed = (relayout) => {
    if (relayout) detectLayout();
    updatePower();
    markTiersStale();
    inputsChanged(false);
  };
  $("motor").addEventListener("change", (e) => {
    S.drone.motor = e.target.value;
    if (S.drone.layout) S.drone.layout.mount_radius = mountRadius();
    drawMarkers();
    changed(false);
  });
  $("prop").addEventListener("change", (e) => { S.drone.prop = e.target.value; changed(false); });
  $("battery").addEventListener("change", (e) => { S.drone.battery = e.target.value; changed(false); });
  $("motorCount").addEventListener("change", (e) => { S.drone.motor_count = Number(e.target.value); changed(true); });
  $("payload").addEventListener("change", (e) => { S.drone.payload_g = Math.max(0, Number(e.target.value) || 0); changed(false); });
  $("measured").addEventListener("change", (e) => { S.drone.measured_thrust_g = Math.max(0, Number(e.target.value) || 0); changed(false); });
  $("style").addEventListener("change", (e) => { S.drone.style = e.target.value; changed(false); });
  document.querySelectorAll("[data-add]").forEach((b) => b.addEventListener("click", () => openOwnForm(b.dataset.add)));
}

// Form for the user's own motor / propeller / battery.
const OWN_FIELDS = {
  motors: [["name", "own_name", "text", "", true], ["stator", "own_stator", "text", "2207"], ["kv", "own_kv", "number", 1750],
           ["weight_g", "own_weight", "number", 33], ["resistance", "own_resistance", "number", 0.07], ["idle_current", "own_idle", "number", 1.0],
           ["max_current", "own_max_current", "number", 40], ["min_cells", "own_cells_min", "number", 4], ["max_cells", "own_cells_max", "number", 6]],
  props: [["name", "own_name", "text", "", true], ["diameter_in", "own_diameter", "number", 5], ["pitch_in", "own_pitch", "number", 4],
          ["blades", "own_blades", "number", 3], ["weight_g", "own_weight", "number", 4.5]],
  batteries: [["name", "own_name", "text", "", true], ["cells", "own_cells", "number", 6], ["capacity_mah", "own_capacity", "number", 1300],
              ["weight_g", "own_weight", "number", 210]]
};

function openOwnForm(kind) {
  const form = $("ownForm");
  form.textContent = "";
  form.hidden = false;
  for (const [key, label, type, value, full] of OWN_FIELDS[kind]) {
    const l = el("label", full ? "full" : "");
    const input = el("input");
    input.type = type;
    input.name = key;
    input.value = value;
    input.required = key === "name";
    if (type === "number") input.step = "any";
    l.append(el("span", "", t(label)), input);
    form.append(l);
  }
  const actions = el("div", "form-actions");
  const cancel = el("button", "ButtonStyleRegular", t("own_cancel"));
  cancel.type = "button";
  cancel.addEventListener("click", () => { form.hidden = true; });
  const save = el("button", "primary", t("own_save"));
  save.type = "submit";
  actions.append(cancel, save);
  form.append(actions);
  form.onsubmit = async (e) => {
    e.preventDefault();
    const data = Object.fromEntries(new FormData(form).entries());
    const item = { id: "user-" + kind + "-" + Date.now(), source: "user" };
    for (const [k, v] of Object.entries(data)) item[k] = k === "name" || k === "stator" ? v : Number(v);
    if (kind === "motors") {
      item.brand = "";
      item.cells = [item.min_cells || 3, item.max_cells || 6];
      item.max_power = (item.max_current || 30) * 3.7 * item.cells[1] * 0.9;
      delete item.min_cells;
      delete item.max_cells;
    }
    if (kind === "batteries") { item.chemistry = "lipo"; item.resistance_per_cell = 0.005; }
    try {
      await Bridge.request({ cmd: "save_user_item", kind, item });
      S.libs[kind].unshift(item);
      if (kind === "motors") S.drone.motor = item.id;
      if (kind === "props") S.drone.prop = item.id;
      if (kind === "batteries") S.drone.battery = item.id;
      fillPowerSelects();
      form.hidden = true;
      setStatus(t("st_saved_item", item.name));
      updatePower();
      inputsChanged(false);
    } catch (err) {
      setStatus(t("st_error", err.message), true);
    }
  };
}

// ---------------------------------------------------------------------------------------------
// Markers and custom loads

function drawMarkers() {
  const v = S.viewer;
  v.clearMarkers();
  const type = analysisType();
  if (type === "drone" && S.drone.layout) {
    const L = S.drone.layout;
    const up = new THREE.Vector3(...L.up);
    v.addSupport(new THREE.Vector3(...L.center), L.center_radius);
    L.motors.forEach((m, i) => v.addDisc(new THREE.Vector3(...m), L.mount_radius, up, v.accent(), 0.25, String(i + 1), i));
    v.selectLabel(S.selectedMotor);
  } else if (type === "custom") {
    for (const s of S.custom.supports) v.addSupport(new THREE.Vector3(...s.center), s.radius);
    for (const f of S.custom.forces) v.addForce(new THREE.Vector3(...f.center), new THREE.Vector3(...forceVector(f)));
  }
  updatePicking();
}

const DIRS = { down: [0, 0, -1], up: [0, 0, 1], xp: [1, 0, 0], xn: [-1, 0, 0], yp: [0, 1, 0], yn: [0, -1, 0] };
const forceVector = (f) => DIRS[f.dir].map((c) => c * f.newtons);

function defaultRadius() {
  const b = S.viewer.bounds;
  const diag = b ? b.getSize(new THREE.Vector3()).length() : 100;
  return Math.round(Math.min(Math.max(diag * 0.04, 3), 15));
}

function updatePicking() {
  const picking = (analysisType() === "custom" && S.tool) || (analysisType() === "drone" && S.selectedMotor >= 0);
  $("viewer").classList.toggle("is-picking", !!picking);
  if (S.tool) setStatus(t(S.tool === "support" ? "ld_click_support" : "ld_click_force"));
}

function onPick(point) {
  const p = [point.x, point.y, point.z];
  const type = analysisType();
  if (type === "drone" && S.selectedMotor >= 0 && S.drone.layout) {
    S.drone.layout.motors[S.selectedMotor] = p;
    S.selectedMotor = -1;
    drawMarkers();
    markTiersStale();
    inputsChanged(false);
  } else if (type === "custom" && S.tool) {
    if (S.tool === "support") S.custom.supports.push({ center: p, radius: defaultRadius() });
    else S.custom.forces.push({ center: p, radius: defaultRadius(), newtons: 50, dir: "down" });
    S.tool = null;
    document.querySelectorAll(".tool").forEach((b) => b.classList.remove("is-on"));
    setStatus("");
    renderLoads();
    drawMarkers();
    markTiersStale();
    inputsChanged(false);
  }
}

function renderLoads() {
  const list = $("loadList");
  list.textContent = "";
  const all = [...S.custom.supports.map((s) => ["support", s]), ...S.custom.forces.map((f) => ["force", f])];
  if (!all.length) {
    list.append(el("li", "hint", t("ld_empty")));
    return;
  }
  for (const [kind, item] of all) {
    const li = el("li");
    const dot = el("span", "dot");
    dot.style.background = kind === "support" ? "#3b6fd8" : "#d6402f";
    const edit = el("div", "edit");
    edit.append(el("b", "", t(kind === "support" ? "ld_support" : "ld_force")));
    const changed = () => { drawMarkers(); markTiersStale(); inputsChanged(false); };
    if (kind === "force") {
      const n = el("input");
      n.type = "number"; n.min = "0"; n.step = "5"; n.value = item.newtons; n.title = t("ld_kg_hint");
      n.addEventListener("change", () => { item.newtons = Math.max(0, Number(n.value) || 0); changed(); });
      const unit = el("span", "unit", t("ld_newtons"));
      const dir = el("select");
      for (const d of Object.keys(DIRS)) {
        const o = el("option", "", t("ld_dir_" + d));
        o.value = d;
        o.selected = d === item.dir;
        dir.append(o);
      }
      dir.addEventListener("change", () => { item.dir = dir.value; changed(); });
      edit.append(n, unit, dir);
    }
    const r = el("input");
    r.type = "number"; r.min = "1"; r.step = "1"; r.value = item.radius; r.title = t("ld_radius");
    r.addEventListener("change", () => { item.radius = Math.max(1, Number(r.value) || 1); changed(); });
    edit.append(r, el("span", "unit", "mm"));
    const del = el("button", "icon-btn", "×");
    del.type = "button";
    del.title = t("ld_remove");
    del.setAttribute("aria-label", t("ld_remove"));
    del.addEventListener("click", () => {
      const arr = kind === "support" ? S.custom.supports : S.custom.forces;
      arr.splice(arr.indexOf(item), 1);
      renderLoads();
      changed();
    });
    li.append(dot, edit, del);
    list.append(li);
  }
}

// ---------------------------------------------------------------------------------------------
// Analysis

function canAnalyse() {
  const type = analysisType();
  if (!S.filament || type === "none") return false;
  if (type === "drone") return !!S.drone.layout;
  return S.custom.supports.length > 0 && S.custom.forces.length > 0;
}

function problemPayload() {
  const req = { use_case: S.useCase, environment: S.environment, impact: S.impact, printer: printerPayload(),
                filament: filamentPayload(), quality: S.quality };
  if (analysisType() === "drone") req.drone = { system: systemPayload(), style: S.drone.style, layout: S.drone.layout };
  if (analysisType() === "custom") {
    req.custom = {
      supports: S.custom.supports.map((s) => ({ center: s.center, radius: s.radius })),
      forces: S.custom.forces.map((f) => ({ center: f.center, radius: f.radius, force: forceVector(f) }))
    };
  }
  return req;
}

let analysisTimer = 0;
function inputsChanged(immediate) {
  $("applyBtn").disabled = !S.filament || (!S.structure && !S.tiers);
  if (!canAnalyse()) return;
  clearTimeout(analysisTimer);
  analysisTimer = setTimeout(runAnalysis, immediate ? 0 : 350);
}

async function runAnalysis() {
  S.analysisDirty = true;
  if (S.analysisBusy) return;
  S.analysisBusy = true;
  while (S.analysisDirty && canAnalyse()) {
    S.analysisDirty = false;
    const req = Object.assign({ cmd: "analyze", structure: S.structure || S.baseStructure }, problemPayload());
    setStatus(t("st_analyzing", fmt(0)));
    setProgress(0.01);
    try {
      const r = await Bridge.request(req);
      if (r.cancelled) continue;
      S.analysis = r;
      if (r.power) { S.power = r.power; renderPower(); }
      if (r.layout && S.drone.layout) {
        S.drone.layout.center = r.layout.center;
        S.drone.layout.center_radius = r.layout.center_radius;
      }
      setStatus(t("st_analyzed", fmt(r.elements), fmt(r.seconds, 1)));
      renderCases();
      renderExpectations();
    } catch (e) {
      setStatus(t("st_error", e.message), true);
    }
    setProgress(null);
  }
  S.analysisBusy = false;
}

function renderCases() {
  const tabs = $("caseTabs");
  tabs.textContent = "";
  const a = S.analysis;
  $("legend").hidden = !a;
  if (!a) {
    S.viewer.setCallout($("callout"), null);
    return;
  }
  const okCases = a.cases.filter((c) => c.ok);
  if (!okCases.find((c) => c.id === S.caseId)) S.caseId = okCases.length ? okCases[0].id : null;
  for (const c of okCases) {
    const b = el("button", "case-tab");
    b.type = "button";
    b.setAttribute("role", "tab");
    b.setAttribute("aria-selected", String(c.id === S.caseId));
    const pill = el("span", "sf-pill", sfText(c.min_safety) + "×");
    pill.style.background = SafetyScale.cssColor(c.min_safety);
    b.append(el("span", "", t("case_" + c.id)), pill);
    b.addEventListener("click", () => { S.caseId = c.id; renderCases(); });
    tabs.append(b);
  }
  refreshColors();
}

function refreshColors() {
  const a = S.analysis;
  const c = a && a.cases.find((x) => x.id === S.caseId);
  if (!c) {
    S.viewer.setSafety(null);
    return;
  }
  S.viewer.setSafety(new Float32Array(b64ToBuffer(c.vertex_safety)));
  const callout = $("callout");
  callout.textContent = t("callout_weakest", sfText(c.min_safety));
  S.viewer.setCallout(callout, new THREE.Vector3(...c.weakest));
}

// ---------------------------------------------------------------------------------------------
// Options (Light / Balanced / Strong)

function markTiersStale() {
  if (S.tiers) {
    S.tiersStale = true;
    $("findBtn").textContent = t("sp_find_again");
  }
}

async function findSettings() {
  if (!S.filament) {
    setStatus(t("st_need_material"), true);
    return;
  }
  const req = Object.assign({ cmd: "optimize", structure: { layer_height: S.baseStructure.layer_height, line_width: S.baseStructure.line_width,
                                                           top: S.baseStructure.top, bottom: S.baseStructure.bottom } }, problemPayload());
  if (analysisType() === "custom" && !canAnalyse()) {
    setStatus(t("ld_click_support"), true);
    return;
  }
  $("findBtn").disabled = true;
  setStatus(t("st_optimizing"));
  setProgress(0.01);
  try {
    const r = await Bridge.request(req);
    if (!r.cancelled) {
      S.tiers = r.tiers;
      S.tiersStale = false;
      $("findBtn").textContent = t("sp_find_again");
      const pick = r.tiers.find((x) => x.id === "balanced" && x.reached) || r.tiers.find((x) => x.reached) || r.tiers[r.tiers.length - 1];
      selectTier(pick.id);
      setStatus(t("st_optimized", fmt(r.seconds, 1)));
      $("stepOptions").classList.add("is-done");
    }
  } catch (e) {
    setStatus(t("st_error", e.message), true);
  }
  setProgress(null);
  $("findBtn").disabled = false;
}

function selectTier(id) {
  S.tierId = id;
  const tier = S.tiers.find((x) => x.id === id);
  S.structure = tier.settings;
  renderTiers();
  renderExpectations();
  $("applyBtn").disabled = !S.filament;
  if (canAnalyse()) inputsChanged(true);
}

function renderTiers() {
  const box = $("tiers");
  box.textContent = "";
  if (!S.tiers) return;
  for (const tier of S.tiers) {
    const b = el("button", "tier");
    b.type = "button";
    b.setAttribute("role", "radio");
    b.setAttribute("aria-checked", String(tier.id === S.tierId));
    b.append(el("span", "tier-name", t("tier_" + tier.id)));
    const big = el("span", "big", fmt(tier.mass_g));
    big.append(el("small", "", " g"));
    b.append(big);
    const facts = el("div", "facts");
    facts.append(el("span", "", t("tier_walls_n", fmt(tier.settings.walls))));
    facts.append(el("span", "", t("tier_infill_n", fmt(tier.settings.infill * 100))));
    facts.append(el("span", "muted", t("tier_shells_n", tier.settings.top, tier.settings.bottom)));
    facts.append(el("span", "muted", minutesText(tier.print_minutes)));
    if (tier.thrust_to_weight) facts.append(el("span", "muted", t("pw_ratio", fmt(tier.thrust_to_weight, 1))));
    b.append(facts);
    if (tier.target > 0) {
      const pill = el("span", "sf-pill", sfText(tier.safety) + "×");
      pill.style.background = SafetyScale.cssColor(tier.safety);
      pill.title = t("tier_safety");
      const wrap = el("span");
      wrap.append(pill);
      b.append(wrap);
    }
    if (tier.target > 0 && !tier.reached) b.append(el("span", "miss", t("tier_miss")));
    b.addEventListener("click", () => selectTier(tier.id));
    box.append(b);
  }
}

function patternLabel(id) {
  const p = (S.libs.patterns || []).find((x) => x.id === id);
  return p ? p.label : id;
}

function renderExpectations() {
  const box = $("expect");
  box.textContent = "";
  const tier = S.tiers && S.tiers.find((x) => x.id === S.tierId);
  if (!tier) return;
  const name = t("tier_" + tier.id);
  const head = el("p");
  head.append(el("b", "", t("ex_title", name)));
  box.append(head);
  const ul = el("ul");
  const li = (text) => ul.append(el("li", "", text));
  const s = tier.settings;
  li(t("ex_rule", fmt(s.walls), fmt(s.infill * 100), patternLabel(s.pattern), fmt(tier.mass_g), minutesText(tier.print_minutes)));

  const cases = tier.cases || {};
  const a = S.analysis;
  const type = analysisType();
  if (type === "drone") {
    const style = (useCaseInfo("drone_frame").styles || []).find((x) => x.id === S.drone.style) || {};
    if (cases.punch) {
      const sf = cases.punch.safety;
      li(t(sf >= 2.5 ? "ex_punch_great" : sf >= 1.5 ? "ex_punch_ok" : sf >= 1 ? "ex_punch_thin" : "ex_punch_fail", sfText(sf)));
      if (cases.punch.displacement > 1) li(t("ex_flex", fmt(cases.punch.displacement, 1)));
    }
    if (cases.landing) li(t(cases.landing.safety >= 1 ? "ex_landing_ok" : "ex_landing_fail", fmt(style.landing_g || 8)));
    if (cases.arm_hit) li(t(cases.arm_hit.safety >= 1 ? "ex_crash_ok" : "ex_crash_fail"));
    if (a && a.vibration && S.tierId && S.structure === tier.settings) {
      const modes = a.vibration.modes_in_range || [];
      const hover = modes.find((m) => m.near_hover);
      const z = modes.find((m) => m.axis === "z") || modes[0];
      if (hover) li(t("ex_vibration_hover", fmt(hover.frequency)));
      else if (z && z.throttle > 0.15) li(t("ex_vibration", fmt(z.frequency), fmt(z.throttle * 100)));
    }
  } else if (type === "custom" && cases.load) {
    const sf = cases.load.safety;
    li(t(sf >= 3 ? "ex_load_great" : sf >= 2 ? "ex_load_ok" : sf >= 1 ? "ex_load_thin" : "ex_load_fail", sfText(sf)));
    if (cases.load.displacement > 0.05) li(t("ex_load_flex", fmt(cases.load.displacement, cases.load.displacement < 1 ? 2 : 1)));
  }
  if (Object.values(cases).some((c) => c.interlayer && c.safety < 2)) li(t("ex_layers"));
  if (a && a.ignored_elements > 0) li(t("ex_ignored"));
  box.append(ul);
  const tips = (useCaseInfo(S.useCase).tips || []).map((id) => t("tip_" + id));
  if (tips.length) {
    const tipHead = el("p");
    tipHead.append(el("b", "", t("ex_tips")));
    const tipList = el("ul");
    tips.forEach((tip) => tipList.append(el("li", "", tip)));
    box.append(tipHead, tipList);
  }
  if (tier.thickness_factor_needed) {
    box.append(el("p", "advice", t("ex_unreached", sfText(tier.target), fmt((tier.thickness_factor_needed - 1) * 100))));
  }
}

// ---------------------------------------------------------------------------------------------
// Apply

function applySettings() {
  const s = S.structure || (S.tiers && S.tiers[1] && S.tiers[1].settings);
  if (!s || !S.filament) return;
  Bridge.post({ command: "apply", settings: s, filament: S.filament.preset, use_case: S.useCase });
}

// ---------------------------------------------------------------------------------------------
// Wiring

window.addEventListener("DOMContentLoaded", () => {
  if (typeof TranslatePage === "function") TranslatePage();
  S.viewer = new PartViewer($("viewer"));
  S.viewer.onPick = onPick;
  S.viewer.onMarkerClick = (id) => {
    S.selectedMotor = S.selectedMotor === id ? -1 : id;
    S.viewer.selectLabel(S.selectedMotor);
    updatePicking();
  };
  document.querySelectorAll(".seg").forEach((b) => b.addEventListener("click", () => {
    S.quality = b.dataset.quality;
    document.querySelectorAll(".seg").forEach((x) => {
      x.classList.toggle("is-on", x === b);
      x.setAttribute("aria-checked", String(x === b));
    });
    inputsChanged(true);
  }));
  $("impact").addEventListener("change", (e) => { S.impact = Number(e.target.value); rankMaterials(); });
  $("hardened").addEventListener("change", rankMaterials);
  $("enclosure").addEventListener("change", rankMaterials);
  document.querySelectorAll(".tool").forEach((b) => b.addEventListener("click", () => {
    S.tool = S.tool === b.dataset.tool ? null : b.dataset.tool;
    document.querySelectorAll(".tool").forEach((x) => x.classList.toggle("is-on", x.dataset.tool === S.tool));
    updatePicking();
  }));
  $("findBtn").addEventListener("click", findSettings);
  $("applyBtn").addEventListener("click", applySettings);
  $("closeBtn").addEventListener("click", () => Bridge.post({ command: "close_page" }));
  bindPowerInputs();
  renderLoads();
  setStatus(t("st_loading"));
  Bridge.post({ command: "ready" });
});
