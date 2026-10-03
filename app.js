"use strict";

// Velocidrone BetaFlight420 rate math.
// Betaflight: aojmhfnnhld. Actual: jmdjfndobkk.
// Stick s is in [-1, 1] (rc deflection / 500).

const RC_RATE_INCREMENTAL = 14.54;
const SETPOINT_LIMIT = 1998;
const STORE_KEY = "rate-curves-v1";

const AXES = [
  { id: "roll", name: "Roll", color: "#ff5d6c" },
  { id: "pitch", name: "Pitch", color: "#3dce86" },
  { id: "yaw", name: "Yaw", color: "#5aa2ff" },
];

function clamp(v, lo, hi) {
  return Math.min(hi, Math.max(lo, v));
}

function adjustedRcRate(rcRate) {
  const r = Math.max(0, rcRate);
  if (r > 2) return r + RC_RATE_INCREMENTAL * (r - 2);
  return r;
}

function kToRcRate(K) {
  const rp = K / 200;
  if (rp <= 2) return rp;
  return (rp + RC_RATE_INCREMENTAL * 2) / (1 + RC_RATE_INCREMENTAL);
}

function bfOmega(s, rcRate, rate, expo) {
  const abs = Math.abs(s);
  const e = clamp(expo, 0, 1);
  const sup = clamp(rate, 0, 1);
  let shaped = s;
  if (e > 0) shaped = s * abs * abs * abs * e + s * (1 - e);
  const K = 200 * adjustedRcRate(rcRate);
  let w = K * shaped;
  if (sup !== 0) w /= clamp(1 - abs * sup, 0.01, 1);
  return w;
}

function actualOmega(s, center, max, expo) {
  const abs = Math.abs(s);
  const e = clamp(expo, 0, 1);
  const c = Math.max(0, center);
  const m = Math.max(0, max);
  const fifth = s * s * s * s * s;
  const g = abs * (fifth * e + s * (1 - e));
  return s * c + Math.max(0, m - c) * g;
}

function centerBf(rcRate, expo) {
  return 200 * adjustedRcRate(rcRate) * (1 - clamp(expo, 0, 1));
}

function maxBf(rcRate, rate) {
  const K = 200 * adjustedRcRate(rcRate);
  const sup = clamp(rate, 0, 1);
  if (sup === 0) return K;
  return K / clamp(1 - sup, 0.01, 1);
}

function omega(mode, params, s) {
  if (mode === "bf") return bfOmega(s, params.rcRate, params.rate, params.expo);
  return actualOmega(s, params.center, params.max, params.expo);
}

function measure(pred, target) {
  let acc = 0;
  let worst = 0;
  const n = 200;
  for (let i = 1; i <= n; i++) {
    const s = i / n;
    const d = Math.abs(pred(s) - target(s));
    acc += d * d;
    if (d > worst) worst = d;
  }
  return { rms: Math.sqrt(acc / n), max: worst };
}

function round2(n) {
  return clamp(Math.round(n * 100) / 100, 0, 1);
}

function roundInt(n) {
  return Math.max(0, Math.round(n));
}

function fitActualExpo(center, max, target) {
  if (!(max > center + 1e-6)) return 0;
  let num = 0;
  let den = 0;
  for (let i = 1; i <= 200; i++) {
    const s = i / 200;
    const s2 = s * s;
    const s6 = s2 * s2 * s2;
    const A = s6 - s2;
    const b = (target(s) - center * s) / (max - center) - s2;
    num += A * b;
    den += A * A;
  }
  if (!(den > 0)) return 0;
  return clamp(num / den, 0, 1);
}

function analyzeBf(rcRate, rate, expo) {
  const center = centerBf(rcRate, expo);
  const max = maxBf(rcRate, rate);
  const target = (s) => bfOmega(s, rcRate, rate, expo);
  const hidden = fitActualExpo(center, max, target);
  const snap = {
    center: roundInt(center),
    max: roundInt(max),
    expo: round2(hidden),
  };
  const err = measure(
    (s) => actualOmega(s, snap.center, snap.max, snap.expo),
    target
  );
  return {
    center,
    max,
    hiddenExpo: snap.expo,
    snap,
    err,
    sticks: [0.25, 0.5, 0.75, 1].map(target),
  };
}

function searchBf(center, maxRate, target) {
  if (!(maxRate > 1e-6)) {
    return { rcRate: 0, rate: 0, expo: 0, err: { rms: 0, max: 0 } };
  }
  let kLo = Math.max(center, maxRate * 0.01);
  const kHi = Math.max(maxRate, kLo);
  if (kLo > kHi) kLo = kHi;

  let best = null;
  const steps = 480;
  for (let i = 0; i <= steps; i++) {
    const K = kLo + ((kHi - kLo) * i) / steps;
    const cand = candidateFromK(K, center, maxRate, target);
    if (!best || cand.err.rms < best.err.rms) best = cand;
  }

  const snapped = snapBfNeighborhood(best, target);
  return snapped;
}

function candidateFromK(K, center, maxRate, target) {
  const safeK = Math.max(K, 1e-9);
  const expo = clamp(1 - center / safeK, 0, 1);
  const rate = clamp(1 - safeK / maxRate, 0, 0.99);
  const rcRate = Math.max(0, kToRcRate(safeK));
  const err = measure((s) => bfOmega(s, rcRate, rate, expo), target);
  return { rcRate, rate, expo, err };
}

function stepNeighbors(v, step, lo, hi) {
  const q = v / step;
  const values = [
    Math.round(q) * step,
    Math.floor(q + 1e-9) * step,
    Math.ceil(q - 1e-9) * step,
  ];
  const out = [];
  for (const n of values) {
    const c = clamp(Math.round(n / step) * step, lo, hi);
    const rounded = Math.round(c * 1000) / 1000;
    if (!out.includes(rounded)) out.push(rounded);
  }
  return out;
}

function snapBfNeighborhood(best, target) {
  let winner = null;
  const rcRates = stepNeighbors(best.rcRate, 0.01, 0, 4);
  const rates = stepNeighbors(best.rate, 0.01, 0, 1);
  const expos = stepNeighbors(best.expo, 0.01, 0, 1);
  for (const rcRate of rcRates) {
    for (const rate of rates) {
      for (const expo of expos) {
        const err = measure((s) => bfOmega(s, rcRate, rate, expo), target);
        if (
          !winner ||
          err.rms < winner.err.rms - 1e-9 ||
          (Math.abs(err.rms - winner.err.rms) < 1e-9 && err.max < winner.err.max)
        ) {
          winner = { rcRate, rate, expo, err };
        }
      }
    }
  }
  return winner || best;
}

function analyzeActual(center, max, expo) {
  const e = clamp(expo, 0, 1);
  const realizedMax = actualOmega(1, center, max, e);
  const realizedCenter = Math.max(0, center);
  const target = (s) => actualOmega(s, center, max, e);
  const bf = searchBf(realizedCenter, realizedMax, target);
  return {
    center: realizedCenter,
    max: realizedMax,
    enteredMax: Math.max(0, max),
    expo: e,
    bf,
    sticks: [0.25, 0.5, 0.75, 1].map(target),
  };
}

function sampleCurve(fn) {
  const n = 400;
  const pts = new Array(n + 1);
  for (let i = 0; i <= n; i++) pts[i] = fn(-1 + (2 * i) / n);
  return pts;
}

function curveRms(a, b) {
  let acc = 0;
  const n = Math.min(a.length, b.length);
  for (let i = 0; i < n; i++) {
    const d = a[i] - b[i];
    acc += d * d;
  }
  return Math.sqrt(acc / n);
}

function defaultState() {
  return {
    mode: "bf",
    showSlope: false,
    visible: { roll: true, pitch: true, yaw: true },
    bf: {
      roll: { rcRate: 1, rate: 0.7, expo: 0 },
      pitch: { rcRate: 1, rate: 0.64, expo: 0 },
      yaw: { rcRate: 1, rate: 0.55, expo: 0 },
    },
    actual: {
      roll: { center: 200, max: 670, expo: 0.54 },
      pitch: { center: 200, max: 560, expo: 0.4 },
      yaw: { center: 180, max: 450, expo: 0.2 },
    },
    ghost: null,
    hover: null,
  };
}

function loadState() {
  const state = defaultState();
  try {
    const raw = localStorage.getItem(STORE_KEY);
    if (!raw) return state;
    const saved = JSON.parse(raw);
    if (saved.mode === "bf" || saved.mode === "actual") state.mode = saved.mode;
    if (typeof saved.showSlope === "boolean") state.showSlope = saved.showSlope;
    for (const axis of AXES) {
      const id = axis.id;
      if (saved.visible && typeof saved.visible[id] === "boolean") state.visible[id] = saved.visible[id];
      copyParams(state.bf[id], saved.bf && saved.bf[id], ["rcRate", "rate", "expo"]);
      copyParams(state.actual[id], saved.actual && saved.actual[id], ["center", "max", "expo"]);
    }
  } catch {
    return defaultState();
  }
  return state;
}

function copyParams(dest, src, keys) {
  if (!src) return;
  for (const key of keys) {
    const n = Number(src[key]);
    if (Number.isFinite(n)) dest[key] = n;
  }
}

const state = typeof document === "undefined" ? defaultState() : loadState();
let view = null;

function persist() {
  try {
    localStorage.setItem(
      STORE_KEY,
      JSON.stringify({
        mode: state.mode,
        showSlope: state.showSlope,
        visible: state.visible,
        bf: state.bf,
        actual: state.actual,
      })
    );
  } catch {
    /* private mode / file restrictions */
  }
}

function formatDeg(n) {
  if (!Number.isFinite(n)) return "—";
  const rounded = Math.round(n * 10) / 10;
  if (Math.abs(rounded - Math.round(rounded)) < 1e-6) return String(Math.round(rounded));
  return rounded.toFixed(1);
}

function formatSignedStick(pct) {
  if (pct < 0) return `−${Math.abs(pct)}%`;
  if (pct > 0) return `+${pct}%`;
  return "0%";
}

function percentOfMax(value, max) {
  if (!(max > 0)) return "—";
  return `${Math.round((100 * value) / max)}%`;
}

function gainText(max, center) {
  if (!(center >= 1)) return "";
  return `Full stick is ×${(max / center).toFixed(2)} the center slope.`;
}

function fitPhrase(err, other) {
  const word = err.rms < 4 ? "Very close to" : err.rms < 15 ? "Close to" : "Rough fit of";
  return `${word} the ${other} curve. ${formatDeg(err.rms)} deg/s RMS, ${formatDeg(err.max)} deg/s at the worst stick.`;
}

function fieldSpec(mode) {
  if (mode === "bf") {
    return [
      { key: "rcRate", label: "Rc rate", min: 0, max: 4, step: "0.01", digits: 2 },
      { key: "rate", label: "Rate", min: 0, max: 1, step: "0.01", digits: 2, hint: true },
      { key: "expo", label: "Rc expo", min: 0, max: 1, step: "0.01", digits: 2 },
    ];
  }
  return [
    { key: "center", label: "Center", min: 0, max: 1000000, step: "1", digits: 0, suffix: "deg/s" },
    { key: "max", label: "Max rate", min: 0, max: 1000000, step: "1", digits: 0, suffix: "deg/s" },
    { key: "expo", label: "Expo", min: 0, max: 1, step: "0.01", digits: 2 },
  ];
}

function renderMode() {
  const bf = state.mode === "bf";
  document.getElementById("mode-bf").setAttribute("aria-pressed", String(bf));
  document.getElementById("mode-actual").setAttribute("aria-pressed", String(!bf));
  document.getElementById("help").textContent = bf
    ? "Rc rate sets the center slope. Rate bends the curve even with expo at 0. Switching mode does not convert."
    : "Center is the slope at mid-stick. Max rate is full stick. Expo holds the middle down. Switching mode does not convert.";
  document.getElementById("convert").textContent = bf ? "Convert to Actual" : "Convert to Betaflight";
}

function renderFields() {
  const root = document.getElementById("fields");
  const specs = fieldSpec(state.mode);
  root.innerHTML = AXES.map((axis) => {
    const params = state[state.mode][axis.id];
    const fields = specs
      .map((spec) => {
        const value = Number(params[spec.key]).toFixed(spec.digits);
        const hint = spec.hint ? `<span class="hint" id="hint-${axis.id}-rate"></span>` : "";
        const suffix = spec.suffix ? `<span class="suffix">${spec.suffix}</span>` : "";
        return `<label class="field">
          <span class="field-label">${spec.label}</span>
          <span class="field-box">
            <input type="number" inputmode="decimal" data-axis="${axis.id}" data-key="${spec.key}"
              min="${spec.min}" max="${spec.max}" step="${spec.step}" value="${value}" autocomplete="off">
            ${suffix}
          </span>
          ${hint}
        </label>`;
      })
      .join("");
    return `<section class="axis-block" style="--axis:${axis.color}"><h2>${axis.name}</h2>${fields}</section>`;
  }).join("");

  root.querySelectorAll("input").forEach((input) => {
    input.addEventListener("wheel", () => input.blur(), { passive: true });
  });
  updateHints();
}

function updateHints() {
  if (state.mode !== "bf") return;
  for (const axis of AXES) {
    const el = document.getElementById(`hint-${axis.id}-rate`);
    if (el) el.textContent = `CLI ${Math.round(state.bf[axis.id].rate * 100)}`;
  }
}

function stickTable(sticks, max) {
  const head = ["25%", "50%", "75%", "100%"]
    .map((label) => `<th>${label}</th>`)
    .join("");
  const deg = sticks.map((v) => `<td>${formatDeg(v)}</td>`).join("");
  const pct = sticks.map((v) => `<td>${percentOfMax(v, max)}</td>`).join("");
  return `<table class="sticks">
    <thead><tr><th></th>${head}</tr></thead>
    <tbody>
      <tr><td>deg/s</td>${deg}</tr>
      <tr><td>% of max</td>${pct}</tr>
    </tbody>
  </table>`;
}

function renderDetails() {
  const root = document.getElementById("cards");
  root.innerHTML = AXES.map((axis) => {
    const params = state[state.mode][axis.id];
    const off = state.visible[axis.id] ? "" : " is-off";
    if (state.mode === "bf") {
      const info = analyzeBf(params.rcRate, params.rate, params.expo);
      const gain = gainText(info.max, info.center);
      return `<article class="card${off}" style="--axis:${axis.color}" data-axis="${axis.id}">
        <button type="button" class="card-toggle" data-toggle="${axis.id}">
          <span class="dot"></span><h2>${axis.name}</h2>
        </button>
        <div class="stats">
          <div class="stat"><b>${formatDeg(info.center)}</b><span>Center deg/s</span></div>
          <div class="stat"><b>${formatDeg(info.max)}</b><span>Max deg/s</span></div>
          <div class="stat"><b>${info.hiddenExpo.toFixed(2)}</b><span>Hidden expo</span></div>
        </div>
        <p class="sub">${fitPhrase(info.err, "Actual")}</p>
        ${gain ? `<p class="sub">${gain}</p>` : ""}
        ${stickTable(info.sticks, info.max)}
      </article>`;
    }
    const info = analyzeActual(params.center, params.max, params.expo);
    const bf = info.bf;
    const gain = gainText(info.max, info.center);
    const ignored = params.max < params.center ? `<p class="sub">Max is below center, so the curve stays on the center slope and ends at ${formatDeg(info.max)} deg/s.</p>` : "";
    return `<article class="card${off}" style="--axis:${axis.color}" data-axis="${axis.id}">
      <button type="button" class="card-toggle" data-toggle="${axis.id}">
        <span class="dot"></span><h2>${axis.name}</h2>
      </button>
      <div class="stats">
        <div class="stat"><b>${formatDeg(info.center)}</b><span>Center deg/s</span></div>
        <div class="stat"><b>${formatDeg(info.max)}</b><span>Max deg/s</span></div>
        <div class="stat"><b>${info.expo.toFixed(2)}</b><span>Expo</span></div>
      </div>
      <p class="sub">Closest Betaflight: Rc rate ${bf.rcRate.toFixed(2)} · Rate ${bf.rate.toFixed(2)} · Rc expo ${bf.expo.toFixed(2)}.</p>
      <p class="sub">${fitPhrase(bf.err, "Betaflight")}</p>
      ${gain ? `<p class="sub">${gain}</p>` : ""}
      ${ignored}
      ${stickTable(info.sticks, info.max)}
    </article>`;
  }).join("");
  syncChips();
}

function renderNotes() {
  const notes = [];
  let over = false;
  let rateFloor = false;
  let maxIgnored = false;
  for (const axis of AXES) {
    if (state.mode === "bf") {
      const p = state.bf[axis.id];
      if (p.rate > 0.99) rateFloor = true;
      if (maxBf(p.rcRate, p.rate) > SETPOINT_LIMIT) over = true;
    } else {
      const p = state.actual[axis.id];
      if (p.max < p.center) maxIgnored = true;
      if (actualOmega(1, p.center, p.max, p.expo) > SETPOINT_LIMIT) over = true;
    }
  }
  if (rateFloor) {
    notes.push("Rate above 0.99 hits the formula floor: the divisor never goes under 0.01, so full-stick rate stops climbing.");
  }
  if (maxIgnored) {
    notes.push("When max rate is below center, expo does nothing and full stick equals the center slope.");
  }
  if (over) {
    notes.push("A curve crosses ±1998 deg/s. The flight code clamps the setpoint there. The graph keeps the unclamped shape and marks the line.");
  }
  const el = document.getElementById("notes");
  el.innerHTML = notes.map((n) => `<p>${n}</p>`).join("");
  el.hidden = notes.length === 0;
}

function renderBanner() {
  const el = document.getElementById("banner");
  if (!state.ghost) {
    el.hidden = true;
    return;
  }
  const bits = AXES.map((axis) => {
    const now = sampleCurve((s) => omega(state.mode, state[state.mode][axis.id], s));
    const rms = curveRms(now, state.ghost.samples[axis.id]);
    return `${axis.name} ${formatDeg(rms)}`;
  });
  el.hidden = false;
  el.querySelector(".banner-text").textContent =
    `Dashed lines are the ${state.ghost.label} curve before conversion. Difference now: ${bits.join(" · ")} deg/s RMS.`;
}

function syncChips() {
  for (const axis of AXES) {
    const on = state.visible[axis.id];
    document.querySelectorAll(`.chip[data-axis="${axis.id}"]`).forEach((el) => {
      el.setAttribute("aria-pressed", String(on));
    });
  }
}

function refresh() {
  updateHints();
  renderDetails();
  renderNotes();
  renderBanner();
  draw();
  persist();
}

function parseDraft(raw, fallback) {
  const t = raw.trim();
  if (t === "" || t === "-" || t === "." || t === "-." || t.endsWith(".")) return fallback;
  const n = Number(t);
  return Number.isFinite(n) ? n : fallback;
}

function readInput(input, commit) {
  const axis = input.dataset.axis;
  const key = input.dataset.key;
  const prev = state[state.mode][axis][key];
  let n = parseDraft(input.value, prev);
  const lo = Number(input.min);
  const hi = Number(input.max);
  if (commit) n = clamp(n, lo, hi);
  else if (Number.isFinite(Number(input.value))) n = clamp(n, lo, hi);
  state[state.mode][axis][key] = n;
  if (commit && input.value.trim() !== "" && Number(input.value) !== n) {
    const digits = input.step.includes(".") ? 2 : 0;
    input.value = n.toFixed(digits);
  }
}

function niceStep(max) {
  const rough = Math.max(max / 4, 1);
  const pow = 10 ** Math.floor(Math.log10(rough));
  const n = rough / pow;
  const mult = n <= 1 ? 1 : n <= 2 ? 2 : n <= 2.5 ? 2.5 : n <= 5 ? 5 : 10;
  return mult * pow;
}

function formatTick(n, step) {
  if (step < 1 || Math.abs(n - Math.round(n)) > 1e-6) {
    return String(Math.round(n * 10) / 10);
  }
  return String(Math.round(n));
}

function draw() {
  const canvas = document.getElementById("graph");
  const plotEl = document.getElementById("plot");
  if (!canvas || !plotEl) return;
  const cssW = plotEl.clientWidth;
  const cssH = plotEl.clientHeight;
  if (cssW < 10 || cssH < 10) return;

  const dpr = Math.min(window.devicePixelRatio || 1, 2.5);
  canvas.width = Math.round(cssW * dpr);
  canvas.height = Math.round(cssH * dpr);
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cssW, cssH);
  ctx.fillStyle = "#0c0f14";
  ctx.fillRect(0, 0, cssW, cssH);
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.font = '12px "IBM Plex Mono", ui-monospace, monospace';

  let peak = 100;
  const series = AXES.filter((axis) => state.visible[axis.id]).map((axis) => {
    const fn = (s) => omega(state.mode, state[state.mode][axis.id], s);
    peak = Math.max(peak, Math.abs(fn(1)));
    return { axis, fn };
  });
  if (state.ghost) {
    for (const axis of AXES) {
      if (!state.visible[axis.id]) continue;
      const samples = state.ghost.samples[axis.id];
      for (let i = 0; i < samples.length; i++) peak = Math.max(peak, Math.abs(samples[i]));
    }
  }

  const step = niceStep(peak);
  const yMax = Math.max(step, Math.ceil((peak * 1.08) / step) * step);
  const showClamp = peak > SETPOINT_LIMIT && yMax > SETPOINT_LIMIT;

  const leftProbe = ctx.measureText(formatTick(yMax, step)).width;
  const rightProbe = ctx.measureText("0000 deg/s").width;
  const m = {
    l: Math.max(68, leftProbe + 28),
    r: Math.max(92, rightProbe + 18),
    t: 18,
    b: 28,
  };
  const plot = { x: m.l, y: m.t, w: cssW - m.l - m.r, h: cssH - m.t - m.b };
  if (plot.w < 20 || plot.h < 20) return;

  const sToX = (s) => plot.x + ((s + 1) / 2) * plot.w;
  const yToPx = (v) => plot.y + plot.h / 2 - (v / yMax) * (plot.h / 2);

  view = { plot, yMax };

  ctx.save();
  ctx.beginPath();
  ctx.rect(plot.x, plot.y, plot.w, plot.h);
  ctx.clip();

  ctx.strokeStyle = "rgba(255,255,255,0.045)";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 8; i++) {
    const x = plot.x + (plot.w * i) / 8;
    ctx.beginPath();
    ctx.moveTo(x, plot.y);
    ctx.lineTo(x, plot.y + plot.h);
    ctx.stroke();
  }
  const minor = step / 2;
  for (let v = -yMax; v <= yMax + 1e-6; v += minor) {
    const y = yToPx(v);
    ctx.beginPath();
    ctx.moveTo(plot.x, y);
    ctx.lineTo(plot.x + plot.w, y);
    ctx.stroke();
  }

  ctx.strokeStyle = "rgba(255,255,255,0.1)";
  for (let v = -yMax; v <= yMax + 1e-6; v += step) {
    const y = yToPx(v);
    ctx.beginPath();
    ctx.moveTo(plot.x, y);
    ctx.lineTo(plot.x + plot.w, y);
    ctx.stroke();
  }

  ctx.strokeStyle = "rgba(255,255,255,0.38)";
  ctx.lineWidth = 1.25;
  ctx.beginPath();
  ctx.moveTo(plot.x, yToPx(0));
  ctx.lineTo(plot.x + plot.w, yToPx(0));
  ctx.moveTo(sToX(0), plot.y);
  ctx.lineTo(sToX(0), plot.y + plot.h);
  ctx.stroke();

  if (showClamp) {
    ctx.save();
    ctx.setLineDash([3, 4]);
    ctx.strokeStyle = "rgba(226, 177, 90, 0.85)";
    ctx.lineWidth = 1;
    for (const sign of [1, -1]) {
      const y = yToPx(sign * SETPOINT_LIMIT);
      ctx.beginPath();
      ctx.moveTo(plot.x, y);
      ctx.lineTo(plot.x + plot.w, y);
      ctx.stroke();
    }
    ctx.restore();
    ctx.fillStyle = "#e2b15a";
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.fillText("1998", plot.x + 6, yToPx(SETPOINT_LIMIT) - 3);
  }

  if (state.showSlope) {
    ctx.save();
    ctx.setLineDash([2, 6]);
    ctx.lineWidth = 1.25;
    for (const { axis } of series) {
      const params = state[state.mode][axis.id];
      const slope = state.mode === "bf" ? centerBf(params.rcRate, params.expo) : Math.max(0, params.center);
      ctx.strokeStyle = axis.color;
      ctx.globalAlpha = 0.45;
      ctx.beginPath();
      ctx.moveTo(sToX(-1), yToPx(-slope));
      ctx.lineTo(sToX(1), yToPx(slope));
      ctx.stroke();
    }
    ctx.restore();
  }

  if (state.ghost) {
    ctx.save();
    ctx.setLineDash([4, 5]);
    ctx.lineWidth = 1.5;
    ctx.globalAlpha = 0.45;
    for (const axis of AXES) {
      if (!state.visible[axis.id]) continue;
      const samples = state.ghost.samples[axis.id];
      ctx.beginPath();
      for (let i = 0; i < samples.length; i++) {
        const s = -1 + (2 * i) / (samples.length - 1);
        const x = sToX(s);
        const y = yToPx(samples[i]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.strokeStyle = axis.color;
      ctx.stroke();
    }
    ctx.restore();
  }

  for (const { axis, fn } of series) {
    ctx.beginPath();
    for (let i = 0; i <= 400; i++) {
      const s = -1 + (2 * i) / 400;
      const x = sToX(s);
      const y = yToPx(fn(s));
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.strokeStyle = axis.color;
    ctx.globalAlpha = 0.28;
    ctx.lineWidth = 7;
    ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.lineWidth = 2.25;
    ctx.stroke();
  }

  if (state.hover && series.length) {
    const s = state.hover.pct / 100;
    const x = sToX(s);
    ctx.strokeStyle = "rgba(255,255,255,0.28)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x, plot.y);
    ctx.lineTo(x, plot.y + plot.h);
    ctx.stroke();
    for (const { axis, fn } of series) {
      ctx.beginPath();
      ctx.arc(x, yToPx(fn(s)), 3.5, 0, Math.PI * 2);
      ctx.fillStyle = axis.color;
      ctx.fill();
    }
  }

  ctx.beginPath();
  ctx.arc(sToX(0), yToPx(0), 5, 0, Math.PI * 2);
  ctx.fillStyle = "#d7e6ff";
  ctx.fill();
  ctx.lineWidth = 2;
  ctx.strokeStyle = "#1b2433";
  ctx.stroke();
  ctx.restore();

  ctx.fillStyle = "#8e97a8";
  ctx.textBaseline = "middle";
  ctx.textAlign = "right";
  for (let v = -yMax; v <= yMax + 1e-6; v += step) {
    if (Math.abs(v) < 1e-6) continue;
    ctx.fillText(formatTick(v, step), plot.x - 8, yToPx(v));
  }
  ctx.save();
  ctx.translate(15, plot.y + plot.h / 2);
  ctx.rotate(-Math.PI / 2);
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillStyle = "#6a7384";
  ctx.fillText("deg/s", 0, 0);
  ctx.restore();

  ctx.fillStyle = "#8e97a8";
  ctx.textBaseline = "top";
  for (let i = -4; i <= 4; i++) {
    const s = i / 4;
    const pct = Math.round(s * 100);
    const label = i === -4 || i === 4 ? `${pct}%` : `${pct}`;
    const x = sToX(s);
    if (i === -4) ctx.textAlign = "left";
    else if (i === 4) ctx.textAlign = "right";
    else ctx.textAlign = "center";
    ctx.fillText(label, x, plot.y + plot.h + 8);
  }

  const labels = series.map(({ axis, fn }) => ({
    axis,
    y: yToPx(fn(1)),
    text: `${formatDeg(fn(1))} deg/s`,
  }));
  labels.sort((a, b) => a.y - b.y);
  let last = plot.y;
  for (const label of labels) {
    label.labelY = Math.max(label.y, last + 16);
    last = label.labelY;
  }
  if (labels.length) {
    const overflow = labels[labels.length - 1].labelY - (plot.y + plot.h - 8);
    if (overflow > 0) {
      for (const label of labels) label.labelY -= overflow;
    }
  }
  ctx.textAlign = "left";
  ctx.textBaseline = "middle";
  ctx.font = '12px "IBM Plex Mono", ui-monospace, monospace';
  for (const label of labels) {
    const tx = plot.x + plot.w + 12;
    if (Math.abs(label.labelY - label.y) > 8) {
      ctx.strokeStyle = label.axis.color;
      ctx.globalAlpha = 0.5;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(plot.x + plot.w, label.y);
      ctx.lineTo(tx - 4, label.labelY);
      ctx.stroke();
      ctx.globalAlpha = 1;
    }
    const tw = ctx.measureText(label.text).width;
    const bx = tx - 4;
    const by = label.labelY - 8;
    ctx.fillStyle = "rgba(12, 15, 20, 0.92)";
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(bx, by, tw + 8, 16, 4);
    else ctx.rect(bx, by, tw + 8, 16);
    ctx.fill();
    ctx.fillStyle = label.axis.color;
    ctx.fillText(label.text, tx, label.labelY);
  }
}

function positionTip() {
  const tip = document.getElementById("tip");
  const plotEl = document.getElementById("plot");
  if (!state.hover) {
    tip.hidden = true;
    return;
  }
  const s = state.hover.pct / 100;
  const rows = AXES.filter((axis) => state.visible[axis.id])
    .map((axis) => {
      const w = omega(state.mode, state[state.mode][axis.id], s);
      return `<div class="tip-row"><i style="background:${axis.color}"></i><span>${axis.name}</span><b>${formatDeg(w)}</b></div>`;
    })
    .join("");
  tip.hidden = false;
  tip.innerHTML = `<div class="tip-stick">${formatSignedStick(state.hover.pct)}</div>${rows}`;
  const pad = 8;
  let left = state.hover.x + 14;
  let top = state.hover.y + 14;
  const tw = tip.offsetWidth;
  const th = tip.offsetHeight;
  if (left + tw > plotEl.clientWidth - pad) left = state.hover.x - tw - 14;
  if (top + th > plotEl.clientHeight - pad) top = state.hover.y - th - 14;
  tip.style.left = `${Math.max(pad, left)}px`;
  tip.style.top = `${Math.max(pad, top)}px`;
}

function clearHover() {
  if (!state.hover) return;
  state.hover = null;
  document.getElementById("tip").hidden = true;
  draw();
}

function onHover(x, y) {
  if (!view) return;
  const { plot } = view;
  if (x < plot.x || x > plot.x + plot.w || y < plot.y || y > plot.y + plot.h) {
    clearHover();
    return;
  }
  const s = clamp(((x - plot.x) / plot.w) * 2 - 1, -1, 1);
  const pct = clamp(Math.round(s * 100), -100, 100);
  state.hover = { pct, x, y };
  draw();
  positionTip();
}

function setVisible(id) {
  state.visible[id] = !state.visible[id];
  refresh();
}

function convert() {
  const from = state.mode;
  const samples = {};
  for (const axis of AXES) {
    const params = state[from][axis.id];
    samples[axis.id] = sampleCurve((s) => omega(from, params, s));
  }
  if (from === "bf") {
    state.mode = "actual";
    for (const axis of AXES) {
      const info = analyzeBf(state.bf[axis.id].rcRate, state.bf[axis.id].rate, state.bf[axis.id].expo);
      state.actual[axis.id] = { ...info.snap };
    }
  } else {
    state.mode = "bf";
    for (const axis of AXES) {
      const p = state.actual[axis.id];
      const info = analyzeActual(p.center, p.max, p.expo);
      state.bf[axis.id] = {
        rcRate: info.bf.rcRate,
        rate: info.bf.rate,
        expo: info.bf.expo,
      };
    }
  }
  state.ghost = {
    label: from === "bf" ? "Betaflight" : "Actual",
    samples,
  };
  state.hover = null;
  renderMode();
  renderFields();
  refresh();
}

function copyRoll() {
  const src = state[state.mode].roll;
  for (const id of ["pitch", "yaw"]) {
    state[state.mode][id] = { ...src };
  }
  renderFields();
  refresh();
}

function resetAll() {
  const fresh = defaultState();
  state.mode = fresh.mode;
  state.showSlope = fresh.showSlope;
  state.visible = fresh.visible;
  state.bf = fresh.bf;
  state.actual = fresh.actual;
  state.ghost = null;
  state.hover = null;
  document.getElementById("slope").checked = false;
  try {
    localStorage.removeItem(STORE_KEY);
  } catch {
    /* ignore */
  }
  renderMode();
  renderFields();
  syncChips();
  refresh();
}

function init() {
  document.getElementById("slope").checked = state.showSlope;
  renderMode();
  renderFields();
  syncChips();
  refresh();

  document.getElementById("mode-bf").addEventListener("click", () => {
    if (state.mode === "bf") return;
    state.mode = "bf";
    state.hover = null;
    renderMode();
    renderFields();
    refresh();
  });
  document.getElementById("mode-actual").addEventListener("click", () => {
    if (state.mode === "actual") return;
    state.mode = "actual";
    state.hover = null;
    renderMode();
    renderFields();
    refresh();
  });

  document.getElementById("fields").addEventListener("input", (event) => {
    const input = event.target;
    if (!(input instanceof HTMLInputElement)) return;
    readInput(input, false);
    refresh();
  });
  document.getElementById("fields").addEventListener("focusout", (event) => {
    const input = event.target;
    if (!(input instanceof HTMLInputElement)) return;
    readInput(input, true);
    refresh();
  });

  document.getElementById("chips").addEventListener("click", (event) => {
    const btn = event.target.closest("[data-axis]");
    if (!btn) return;
    setVisible(btn.dataset.axis);
  });
  document.getElementById("cards").addEventListener("click", (event) => {
    const btn = event.target.closest("[data-toggle]");
    if (!btn) return;
    setVisible(btn.dataset.toggle);
  });
  document.getElementById("slope").addEventListener("change", (event) => {
    state.showSlope = event.target.checked;
    refresh();
  });
  document.getElementById("convert").addEventListener("click", convert);
  document.getElementById("copy").addEventListener("click", copyRoll);
  document.getElementById("reset").addEventListener("click", resetAll);
  document.getElementById("dismiss").addEventListener("click", () => {
    state.ghost = null;
    renderBanner();
    draw();
  });

  const plot = document.getElementById("plot");
  plot.addEventListener("pointermove", (event) => {
    const rect = document.getElementById("graph").getBoundingClientRect();
    onHover(event.clientX - rect.left, event.clientY - rect.top);
  });
  plot.addEventListener("pointerleave", clearHover);

  const observer = new ResizeObserver(() => draw());
  observer.observe(plot);
  if (document.fonts && document.fonts.ready) document.fonts.ready.then(() => draw());
}

if (typeof document !== "undefined") {
  document.addEventListener("DOMContentLoaded", init);
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = {
    bfOmega,
    actualOmega,
    centerBf,
    maxBf,
    analyzeBf,
    analyzeActual,
    adjustedRcRate,
    kToRcRate,
  };
}
