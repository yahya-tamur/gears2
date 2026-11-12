// The page: builds the options form, asks the worker for meshes whenever an
// option changes, shows them with three.js, and writes a binary STL.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { stlBlob } from "./stl.js";

// ------------------------------------------------------------------ options
//
// Every option, in the order shown. `kinds` limits a field to some gear
// types; `optional` fields may be left blank for the default shown as the
// placeholder; `whole` fields take whole numbers.

const SECTIONS = [
  {
    title: "Gear set",
    help: "Spur and ring gears made with the same gear set always mesh. Bevel gears only mesh with the partner made with them; for them, these lengths are at the outer end of the teeth.",
    fields: [
      { name: "module", label: "Module", unit: "mm", value: "2",
        help: () => "Tooth size. Two meshing gears behave like two wheels rolling on each other; each wheel's edge is called the gear's pitch circle, " +
          "which lies about halfway up the teeth. A gear's pitch circle is teeth × module across, and neighbouring teeth are " +
          `π × module apart along it${now(Math.PI * num("module", NaN), "now ")}. Gears only mesh with the same module.` },
      { name: "pressure_angle", label: "Pressure angle", unit: "°", value: "20",
        help: "Slope of the tooth sides at the pitch circle (see Module). 20° is standard. Larger angles give stronger but more pointed teeth, which push the gears apart harder." },
      { name: "helix_angle", label: "Helix angle", unit: "°", value: "30",
        help: () => {
          const pa = num("pressure_angle", NaN) * Math.PI / 180, ha = num("helix_angle", NaN) * Math.PI / 180;
          const deg = (a) => Number.isFinite(a) ? `${+(a * 180 / Math.PI).toFixed(1)}°` : "–";
          // The flanks' normals all lean out of the plane of rotation by the
          // base helix angle, which is also how far they overhang.
          const baseHelix = Math.atan(Math.tan(ha) * Math.cos(pa));
          // Backlash is set in the plane of rotation; the faces' true distance
          // is backlash * cos(pressure angle) * cos(base helix angle).
          const backlash = inputs.kind.value === "bevel" ? NaN : num("backlash", NaN);
          const gap = backlash * Math.cos(pa) * Math.cos(baseHelix);
          return [
            "Slant of the V-shaped (herringbone) teeth; 0° is straight.",
            `Current normal pressure angle (measured square to the slanted teeth): ${deg(Math.atan(Math.tan(pa) * Math.cos(ha)))}`,
            ...(Number.isFinite(gap) && gap >= 0 ? [`Current distance between tooth faces (with the other side touching): ${+gap.toFixed(3)} mm`] : []),
            `Current overhang angle (how far the downward-facing tooth sides lean from vertical): ${deg(baseHelix)}`,
          ].join("\n");
        } },
      { name: "height", label: "Tooth height", unit: "mm", optional: "auto",
        help: () => `How far each tooth reaches into its mate: half of it above the pitch circle (see Module), half below. The gaps go a root gap deeper. Leave blank for the standard 2 × module${now(toothHeight())}.` },
      { name: "backlash", label: "Backlash", unit: "mm", value: "0.2", kinds: ["spur", "ring", "planetary"],
        help: "Play between meshing teeth: hold one gear, and the other can rock back and forth by this much, measured along its pitch circle (see Module). Leaves room for printing tolerances; 0.1–0.3 mm is typical." },
      { name: "clearance", label: "Root gap", unit: "mm", optional: "auto",
        help: () => `Space between a tooth tip and the bottom of the gap it enters, so tips never touch bottom. Leave blank for the standard module ÷ 4${now(rootGap())}.` },
      { name: "fillet_radius", label: "Root rounding", unit: "mm", optional: "auto",
        help: () => `Radius of the rounded corners at the bottom of each gap, which make the teeth stronger. Leave blank for the largest that fits${now(rootGap() / (1 - Math.sin(num("pressure_angle", NaN) * Math.PI / 180)), "up to ")}; each gear uses less where its gaps are too narrow. 0 for sharp corners.` },
    ],
  },
  {
    title: "Gear",
    fields: [
      { name: "kind", label: "Type", select: [["spur", "Spur gear"], ["ring", "Ring gear"], ["planetary", "Planetary set"], ["bevel", "Bevel pair"]], value: "planetary",
        helps: {
          spur: "A spur gear: a wheel with teeth around the outside.",
          ring: "A ring gear: a ring with teeth around the inside, for spur gears from this gear set to roll in.",
          planetary: "A planetary set: a sun gear in the middle, planets around it and a ring gear outside, all meshing together and positioned ready to assemble.",
          bevel: "A bevel pair: two gears on cones whose shafts meet at an angle. A bevel gear only meshes with the partner it was made with, so both are made together.",
        } },
      { name: "teeth", label: "Teeth", whole: true, value: "21", kinds: ["spur", "bevel"], labels: { bevel: "Gear 1 teeth" },
        helps: { bevel: "Teeth on gear 1 (blue in the preview)." },
        help: "Number of teeth. The gear set sets the smallest number allowed (shown below)." },
      { name: "ring_teeth", label: "Teeth", whole: true, value: "63", kinds: ["ring"],
        help: "Number of teeth. The gear set sets the smallest number allowed (shown below)." },
      { name: "teeth2", label: "Gear 2 teeth", whole: true, value: "30", kinds: ["bevel"],
        help: "Teeth on gear 2 (green in the preview). Together with the shaft angle, the two tooth counts set how steeply each gear's teeth slope." },
      { name: "shaft_angle", label: "Shaft angle", unit: "°", value: "90", kinds: ["bevel"],
        help: "Angle between the two gears' shafts, which meet at a point. 90° is a right angle." },
      { name: "sun_teeth", label: "Sun teeth", whole: true, value: "21", kinds: ["planetary"],
        help: "Teeth on the sun gear in the middle. The ring gets sun + 2 × planet teeth." },
      { name: "planet_teeth", label: "Planet teeth", whole: true, value: "21", kinds: ["planetary"],
        help: "Teeth on each planet gear." },
      { name: "planets", label: "Planets", whole: true, value: "4", kinds: ["planetary"],
        help: "Number of planet gears, spaced evenly. Sun teeth + ring teeth must be divisible by this, and the planets must fit side by side." },
      { name: "face_width", label: "Face width", unit: "mm", value: "12", kinds: ["bevel"],
        help: () => {
          const toe = bevelToe(), w = num("face_width", NaN);
          const reach = w / (1 - toe); // outer end of the teeth to where the shafts meet
          return "Length of the teeth, measured along their slope from the outer edge inward. The teeth get smaller " +
            "toward the inner end, all the way to the point where the two shafts meet. A typical face width is at most " +
            `a third of the way from the outer edge to that point${now(reach / 3, "here: ")}.`;
        } },
      { name: "width", label: "Face width", unit: "mm", value: "30", kinds: ["spur", "ring", "planetary"],
        help: "Height of the part with full teeth, along the axis. The chamfers are added below and above it." },
      { name: "bore", label: "Bore", unit: "mm", value: "10", kinds: ["spur", "planetary", "bevel"], labels: { planetary: "Sun bore", bevel: "Gear 1 bore" },
        help: "Diameter of the hole through the middle for a shaft. 0 for none.",
        helps: { bevel: "Diameter of the hole through gear 1 for its shaft. 0 for none." } },
      { name: "bore2", label: "Gear 2 bore", unit: "mm", value: "8", kinds: ["bevel"],
        help: "Diameter of the hole through gear 2 for its shaft. 0 for none." },
      { name: "backlash_inner", label: "Backlash at inner end", unit: "mm", value: "0.2", optional: "auto", kinds: ["bevel"],
        help: () => "Play between meshing teeth (hold one gear, and the other can rock back and forth by this much), measured along the pitch circle (see Module) at the inner end of the teeth, where it is smallest; it grows toward the outer end. Set this or the outer end's, and leave the other blank" +
          (blank("backlash_inner") ? `${now(num("backlash_outer", NaN) * bevelToe())}.` : ".") },
      { name: "backlash_outer", label: "Backlash at outer end", unit: "mm", optional: "auto", kinds: ["bevel"],
        help: () => "Play between meshing teeth, measured along the pitch circle (see Module) at the outer end of the teeth, where it is largest. Set this or the inner end's, and leave the other blank" +
          (blank("backlash_outer") ? `${now(num("backlash_inner", NaN) / bevelToe())}.` : ".") },
      { name: "planet_bore", label: "Planet bore", unit: "mm", value: "8", kinds: ["planetary"],
        help: "Diameter of the hole through each planet. 0 for none." },
      { name: "rim", label: "Ring outer diameter", unit: "mm", value: "140", kinds: ["ring", "planetary"],
        help: "Diameter of the ring gear's smooth outside. It must leave a wall around the bottoms of the tooth gaps." },
      { name: "hand", label: "Twist direction", select: [["1", "Counterclockwise"], ["-1", "Clockwise"]], value: "1",
        help: "Which way the lower half of the herringbone teeth (see Helix angle) turns going up, seen from above.",
        helps: {
          spur: "Which way the lower half of the herringbone teeth (see Helix angle) turns going up, seen from above. Two spur gears that mesh need opposite directions; a spur gear inside a ring gear needs the ring's direction.",
          ring: "Which way the lower half of the herringbone teeth (see Helix angle) turns going up, seen from above. The spur gears inside it need the same direction.",
          planetary: "Which way the lower half of the sun's herringbone teeth (see Helix angle) turns going up, seen from above. The planets and the ring get the matching directions automatically.",
          bevel: "With gear 1 standing on its flat back: which way the outer half of its herringbone teeth (see Helix angle) turns going up toward the inner end, seen from above. Gear 2 gets the matching direction automatically.",
        } },
      { name: "end_bottom", label: "Bottom chamfer", unit: "mm", optional: "auto", kinds: ["spur", "ring", "planetary"],
        help: () => chamferHelp("below") },
      { name: "end_top", label: "Top chamfer", unit: "mm", optional: "auto", kinds: ["spur", "ring", "planetary"],
        help: () => chamferHelp("above") },
      { name: "layout", label: "STL layout", select: [["assembled", "Assembled"], ["print", "Ready to print"]], value: "assembled", kinds: ["bevel"],
        help: "Assembled: the two gears in place, meshing. Ready to print: each gear standing on its flat back, side by side." },
    ],
  },
  {
    title: "STL smoothness",
    help: "How closely the flat triangles of the STL follow the true curved surfaces. Finer is smoother but makes larger files.",
    fields: [
      { name: "max_deviation", label: "Surface deviation", unit: "mm", value: "0.01",
        help: "The mesh may deviate from the exact shape by up to this much, to reduce triangles. Distances between parts can only grow, never shrink." },
      { name: "ds", label: "Minimum point spacing", unit: "mm", value: "0.05",
        help: "Minimum distance between points." },
      { name: "fillet_deviation", label: "Root rounding deviation", unit: "mm", optional: "auto",
        help: "Surface deviation for the rounded corners at the bottom of the gaps between teeth (see Root rounding). A larger value here doesn't affect how the gear works, and significantly simplifies the mesh." },
    ],
  },
  {
    title: "STEP smoothness",
    help: "How closely the STEP file's curved surfaces follow the true shape. Its flat faces, bores, chamfer cones and spherical ends are always exact.",
    fields: [
      { name: "step_deviation", label: "Surface deviation", unit: "mm", value: "0.0001",
        help: "The STEP file's curved tooth surfaces may deviate from the exact shape by up to this much, either way. Smaller makes larger files; 0.0001 mm is far finer than any printer or machine can make." },
    ],
  },
];

// Current values, for the hover texts of values left on auto.
function num(name, fallback) {
  const text = (inputs[name]?.value ?? "").trim().replace(",", ".");
  return text === "" ? fallback : Number(text);
}
const blank = (name) => (inputs[name]?.value ?? "").trim() === "";
// " (now 0.5 mm)", or nothing while the inputs it depends on aren't numbers.
function now(value, prefix = "") {
  return Number.isFinite(value) && value >= 0 ? ` (${prefix ? prefix : "now "}${+value.toFixed(3)} mm)` : "";
}
const rootGap = () => num("clearance", num("module", NaN) / 4);
const toothHeight = () => num("height", 2 * num("module", NaN));
// Bevel pair: inner end / outer end, as cone distances (R - face width) / R.
function bevelToe() {
  const [z1, z2, shaft] = [num("teeth", NaN), num("teeth2", NaN), num("shaft_angle", NaN) * Math.PI / 180];
  const d1 = Math.atan2(Math.sin(shaft), z2 / z1 + Math.cos(shaft));
  const r = num("module", NaN) * z1 / (2 * Math.sin(d1));
  return (r - num("face_width", NaN)) / r;
}

// The chamfer cone runs from the tooth tips to module / 4 inside the gap
// bottoms, so its slope is 45° when its height equals that radial run.
function chamferHelp(where) {
  const module = num("module", NaN);
  return `Extra height ${where} the face width where the teeth taper off, so no edge is sharper than a right angle. ` +
    `Leave blank for a 45° slope: tooth height + root gap + module ÷ 4${now(toothHeight() + rootGap() + module / 4)}. 0 for a flat end.`;
}

// Slider ranges [min, max, step] for every number field. Click a slider's
// end to change them; changes are remembered in this browser.
// Default slider [min, max, step]: at most about 20 steps, so each step can
// be felt, covering the usual values. Typing past an end extends it.
const RANGES = {
  module: [0.5, 5, 0.25], pressure_angle: [10, 30, 1], helix_angle: [0, 50, 2.5], height: [0.5, 10, 0.5],
  backlash: [0, 1, 0.05], clearance: [0, 2, 0.1], fillet_radius: [0, 2, 0.1],
  teeth: [8, 28, 1], ring_teeth: [50, 70, 1], teeth2: [16, 36, 1], shaft_angle: [40, 140, 5],
  sun_teeth: [8, 28, 1], planet_teeth: [8, 28, 1], planets: [1, 12, 1],
  face_width: [4, 24, 1], width: [5, 55, 2.5], bore: [0, 20, 1], bore2: [0, 20, 1], planet_bore: [0, 20, 1],
  backlash_inner: [0, 1, 0.05], backlash_outer: [0, 1, 0.05], rim: [40, 240, 10],
  end_bottom: [0, 10, 0.5], end_top: [0, 10, 0.5],
  max_deviation: [0.002, 0.04, 0.002], ds: [0.01, 0.2, 0.01], fillet_deviation: [0.01, 0.2, 0.01],
  step_deviation: [0.00005, 0.001, 0.00005],
};
// Typing a value past a slider's end moves that end out to it (as in
// Desmos), but never past what the value can be at all: a pressure angle is
// at most 45°, angles and lengths can't be negative. A value beyond that
// stays in the box (and shows its error); the slider stops at the limit.
const LIMITS = { pressure_angle: [0, 45], helix_angle: [0, 89] };
function extendRange(name) {
  const s = sliders[name];
  const v = num(name, NaN);
  if (!s || !Number.isFinite(v)) return;
  const [lo, hi] = LIMITS[name] || [0, Infinity];
  let [min, max, step] = s.r;
  if (v > max) max = Math.max(max, Math.min(v, hi));
  if (v < min) min = Math.min(min, Math.max(v, lo));
  if (min !== s.r[0] || max !== s.r[1]) s.apply([min, max, step], true);
}

// What a blank auto field currently means (its slider shows it, faded).
const chamfer45 = () => toothHeight() + rootGap() + num("module", NaN) / 4;
const AUTO = {
  height: () => toothHeight(),
  clearance: () => rootGap(),
  fillet_radius: () => rootGap() / (1 - Math.sin(num("pressure_angle", NaN) * Math.PI / 180)),
  backlash_inner: () => num("backlash_outer", NaN) * bevelToe(),
  backlash_outer: () => num("backlash_inner", NaN) / bevelToe(),
  end_bottom: chamfer45,
  end_top: chamfer45,
  fillet_deviation: () => Math.max(0.05, num("max_deviation", NaN)),
};
const sliders = {};

function savedRange(name) {
  try {
    const r = JSON.parse(localStorage.getItem("range:" + name));
    if (Array.isArray(r) && r.length === 3 && r.every(Number.isFinite)) return r;
  } catch {}
  return RANGES[name];
}

function decimals(step) {
  const text = String(step);
  return text.includes("e-") ? Number(text.split("e-")[1]) : (text.split(".")[1] || "").length;
}

function buildSlider(f, input, row) {
  const wrap = document.createElement("div");
  wrap.className = "slider";
  const lo = document.createElement("button"), hi = document.createElement("button");
  lo.type = hi.type = "button";
  lo.className = "lim lo";
  hi.className = "lim hi";
  lo.title = hi.title = "Change the slider's range and step";
  const track = document.createElement("span");
  track.className = "track";
  const ticks = document.createElement("span");
  ticks.className = "ticks";
  const range = document.createElement("input");
  range.type = "range";
  range.setAttribute("aria-label", f.label + " slider");
  track.append(ticks, range);
  wrap.append(lo, track, hi);

  // the editor: [min] ≤ label ≤ [max]  step [step]
  const editor = document.createElement("div");
  editor.className = "limits";
  editor.hidden = true;
  const box = (title) => {
    const b = document.createElement("input");
    b.type = "text";
    b.inputMode = "decimal";
    b.setAttribute("aria-label", title);
    return b;
  };
  const [eMin, eMax, eStep] = [box("Slider minimum"), box("Slider maximum"), box("Slider step")];
  const reset = document.createElement("button");
  reset.type = "button";
  reset.textContent = "Reset";
  const text = (t) => Object.assign(document.createElement("span"), { textContent: t });
  editor.append(eMin, text("≤ " + f.label + " ≤"), eMax, text("step"), eStep, reset);

  const s = { range, wrap, lo, hi, ticks, editor, eMin, eMax, eStep, field: f, input };
  sliders[f.name] = s;

  const apply = (r, save) => {
    const [min, max, step] = r;
    s.r = r;
    range.min = min;
    range.max = max;
    range.step = step;
    lo.textContent = String(min);
    hi.textContent = String(max);
    eMin.value = min;
    eMax.value = max;
    eStep.value = step;
    const n = (max - min) / step;
    ticks.style.background = n <= 40
      ? `radial-gradient(circle, var(--muted) 1.2px, transparent 1.6px) 0 50% / calc(100% / ${n}) 100% repeat-x`
      : "none";
    ticks.style.width = "";
    if (save) {
      try { localStorage.setItem("range:" + f.name, JSON.stringify(r)); } catch {}
    }
    syncSlider(f.name);
  };
  s.apply = apply;
  apply(savedRange(f.name), false);

  const open = (focus) => {
    editor.hidden = false;
    focus.focus();
    focus.select();
  };
  lo.addEventListener("click", () => open(eMin));
  hi.addEventListener("click", () => open(eMax));
  const fromEditor = () => {
    const r = [eMin, eMax, eStep].map((b) => Number(b.value.trim().replace(",", ".")));
    if (r.every(Number.isFinite) && r[0] < r[1] && r[2] > 0) apply(r, true);
  };
  for (const b of [eMin, eMax, eStep]) {
    b.addEventListener("input", (e) => { e.stopPropagation(); fromEditor(); });
    b.addEventListener("keydown", (e) => { if (e.key === "Enter") { e.preventDefault(); editor.hidden = true; } });
  }
  reset.addEventListener("click", () => {
    try { localStorage.removeItem("range:" + f.name); } catch {}
    apply(RANGES[f.name], false);
    // A value outside the default range moves to its nearer end.
    const v = num(f.name, NaN);
    const [min, max] = RANGES[f.name];
    if (Number.isFinite(v) && (v < min || v > max)) {
      input.value = String(Math.min(max, Math.max(min, v)));
      input.dispatchEvent(new Event("input", { bubbles: true }));
    }
  });
  editor.addEventListener("focusout", (e) => {
    if (!editor.contains(e.relatedTarget) && e.relatedTarget !== lo && e.relatedTarget !== hi) editor.hidden = true;
  });

  range.addEventListener("input", (e) => {
    e.stopPropagation();
    input.value = String(+Number(range.value).toFixed(decimals(s.r[2])));
    input.dispatchEvent(new Event("input", { bubbles: true }));
  });
  return { wrap, editor };
}

// Put a slider's thumb at its field's value (or its auto value, faded).
function syncSlider(name) {
  const s = sliders[name];
  if (!s) return;
  const isBlank = s.input.value.trim() === "";
  const value = isBlank ? (AUTO[name] ? AUTO[name]() : NaN) : num(name, NaN);
  s.wrap.classList.toggle("auto", isBlank);
  s.range.disabled = isBlank && !!s.field.optional;
  if (s.field.optional) s.input.placeholder = Number.isFinite(value) ? String(+value.toFixed(3)) : "";
  if (Number.isFinite(value)) s.range.value = String(Math.min(s.r[1], Math.max(s.r[0], value)));
}

// Put a field's unit right after its number (or after the placeholder);
// hide it when the number fills the box.
const measure = document.createElement("canvas").getContext("2d");
function placeUnit(name) {
  const entry = rows[name];
  const input = inputs[name];
  const unit = entry && entry.unit;
  if (!unit) return;
  const style = getComputedStyle(input);
  measure.font = style.font;
  const text = input.value || input.placeholder || "";
  const left = parseFloat(style.paddingLeft) + parseFloat(style.borderLeftWidth) + measure.measureText(text).width + 4;
  // the unit's own width, measured from its text (a hidden unit's box is 0 wide)
  measure.font = getComputedStyle(unit).font;
  const width = measure.measureText(unit.textContent).width;
  unit.style.left = left + "px";
  unit.hidden = left + width > input.clientWidth;
}

// Bevel backlash: one end is set, the other auto.
const AUTO_PARTNER = { backlash_inner: "backlash_outer", backlash_outer: "backlash_inner" };

// Turn a field's auto checkbox on (blank: the automatic value) or off (the
// current automatic value filled in, ready to change).
function setAuto(name, on, fire = true) {
  const input = inputs[name], s = sliders[name];
  const partner = AUTO_PARTNER[name];
  // Exactly one bevel backlash end is auto: switching one on switches the
  // other off, with the value it has now (worked out before this one clears).
  if (on && partner && rows[partner].auto.checked) {
    const other = inputs[partner];
    const v = AUTO[partner]();
    other.value = Number.isFinite(v) ? String(+v.toFixed(decimals(sliders[partner].r[2]))) : "";
    rows[partner].auto.checked = false;
    other.disabled = false;
  }
  rows[name].auto.checked = on;
  input.disabled = on;
  if (on) {
    input.value = "";
  } else if (input.value.trim() === "") {
    const v = AUTO[name] ? AUTO[name]() : NaN;
    input.value = Number.isFinite(v) ? String(+v.toFixed(s ? decimals(s.r[2]) : 3)) : "";
  }
  if (!on && partner && !rows[partner].auto.checked) setAuto(partner, true, false);
  if (fire) input.dispatchEvent(new Event("input", { bubbles: true }));
}

const form = document.getElementById("options");
const generate = document.getElementById("generate");
const download = document.getElementById("download");
const downloadStep = document.getElementById("download-step");
let buildingParams = null; // the settings of the build in progress (kept for a STEP file)
let makingStep = false;
const statusLine = document.getElementById("status");
const errorBox = document.getElementById("error");
const restoreButton = document.getElementById("restore");
const canvas = document.getElementById("canvas");
const inputs = {};
const rows = {};

function buildForm() {
  const holder = document.getElementById("fields");
  for (const section of SECTIONS) {
    const set = document.createElement("fieldset");
    const legend = document.createElement("legend");
    legend.textContent = section.title;
    if (section.help) {
      legend.append(helpMark());
    }
    set.append(legend);
    if (section.help) {
      const tip = document.createElement("span");
      tip.className = "tip legend-tip";
      tip.textContent = section.help;
      set.append(tip);
    }
    for (const f of section.fields) set.append(buildField(f));
    holder.append(set);
  }
}

function helpMark() {
  const mark = document.createElement("span");
  mark.className = "help-mark";
  mark.textContent = "?";
  mark.setAttribute("aria-hidden", "true");
  return mark;
}

// One line per option: name, slider, value, "auto" (optional values) and
// "set" (numbers; shown after an error, for Search).
function buildField(f) {
  const row = document.createElement("div");
  row.className = "field";
  const name = document.createElement("span");
  name.className = "name";
  name.textContent = f.label;
  name.append(helpMark());
  const box = document.createElement("span");
  box.className = "input";
  let input;
  if (f.select) {
    input = document.createElement("select");
    for (const [value, text] of f.select) input.append(new Option(text, value, false, value === f.value));
    row.classList.add("select-row");
  } else {
    input = document.createElement("input");
    input.type = "text";
    input.inputMode = f.whole ? "numeric" : "decimal";
    input.value = f.value ?? "";
    if (f.unit) {
      const unit = document.createElement("span");
      unit.className = "unit";
      unit.textContent = f.unit;
      box.append(unit);
      box.unitSpan = unit;
    } else {
      input.classList.add("unitless");
    }
  }
  input.name = f.name;
  input.setAttribute("aria-label", f.label);
  box.prepend(input);
  const tip = document.createElement("span");
  tip.className = "tip";
  tip.textContent = typeof f.help === "function" ? "" : f.help;
  inputs[f.name] = input;
  const entry = { row, name, tip, field: f, unit: box.unitSpan };
  rows[f.name] = entry;

  // A small toggle button ("auto", "set") with a checked property and a
  // change event, like a checkbox.
  const check = (cls, text, title) => {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "toggle " + cls;
    b.textContent = text;
    b.title = title;
    b.setAttribute("aria-pressed", "false");
    Object.defineProperty(b, "checked", {
      get: () => b.getAttribute("aria-pressed") === "true",
      set: (v) => b.setAttribute("aria-pressed", v ? "true" : "false"),
    });
    b.addEventListener("click", () => {
      b.checked = !b.checked;
      b.dispatchEvent(new Event("change"));
    });
    return [b, b];
  };
  const parts = [name];
  let editor = null;
  if (!f.select && RANGES[f.name]) {
    const sl = buildSlider(f, input, row);
    parts.push(sl.wrap);
    editor = sl.editor;
  }
  parts.push(box);
  if (f.optional) {
    const [label, c] = check("auto-box", "auto", "Use the automatic value (see the ? for what it is now)");
    entry.auto = c;
    c.checked = input.value.trim() === "";
    input.disabled = c.checked;
    c.addEventListener("change", (e) => { e.stopPropagation(); setAuto(f.name, c.checked); });
    parts.push(label);
  } else {
    parts.push(Object.assign(document.createElement("span"), { className: "auto-cell" }));
  }
  if (!f.select && RANGES[f.name]) {
    const [label, c] = check("set-box", "set", "Keep this value when finding values that work");
    entry.set = c;
    c.checked = true; // everything starts set; clear the ones Search may change
    c.addEventListener("change", (e) => e.stopPropagation());
    parts.push(label);
  } else {
    parts.push(Object.assign(document.createElement("span"), { className: "set-cell" }));
  }
  row.append(...parts, tip);
  if (editor) row.append(editor);
  return row;
}

buildForm();

function showFieldsFor(kind) {
  for (const name in sliders) syncSlider(name);
  requestAnimationFrame(() => { for (const name in rows) placeUnit(name); });
  for (const { row, name, tip, field } of Object.values(rows)) {
    if (field.helps) tip.textContent = field.helps[kind] ?? field.help;
    if (typeof field.help === "function") tip.textContent = field.help();
    row.hidden = field.kinds ? !field.kinds.includes(kind) : false;
    const label = (field.labels && field.labels[kind]) || field.label;
    name.firstChild.textContent = label;
  }
}

// Reads every visible field. Returns { params } or { errors: [message] },
// marking bad fields.
function readOptions() {
  const kind = inputs.kind.value;
  const params = { kind, hand: Number(inputs.hand.value) };
  const errors = [];
  for (const section of SECTIONS) {
    for (const f of section.fields) {
      if (f.select) {
        if (f.name !== "kind" && f.name !== "hand") params[f.name] = inputs[f.name].value;
        continue;
      }
      const input = inputs[f.name];
      input.classList.remove("invalid");
      const used = !f.kinds || f.kinds.includes(kind);
      const text = input.value.trim().replace(",", ".");
      if (text === "" && f.optional) {
        params[f.name] = null;
        continue;
      }
      const value = Number(text);
      let problem = null;
      if (text === "" || !Number.isFinite(value)) problem = "needs a number";
      else if (value < 0) problem = f.name === "helix_angle" ? "can't be negative: change the twist direction instead" : "can't be negative";
      else if (f.whole && !Number.isInteger(value)) problem = "needs a whole number";
      if (problem) {
        if (used) {
          input.classList.add("invalid");
          errors.push(`${(f.labels && f.labels[kind]) || f.label} ${problem}.`);
        }
        params[f.name] = 0;
      } else {
        params[f.name] = value;
      }
    }
  }
  return errors.length ? { errors } : { params };
}

// Errors go in their own box above the buttons (always in view); a
// successful build clears it and shows its summary below them.
function setStatus(text, isError = false) {
  errorBox.hidden = !isError;
  // Only download what the current settings make (the preview may still
  // show the last values that worked).
  download.disabled = isError || !current;
  updateStepButton(isError);
  form.classList.toggle("show-set", isError);
  if (isError) {
    errorBox.textContent = text;
    statusLine.textContent = searching ? "" : "Click Search to look for values that work. Values marked \"set\" stay as they are.";
  } else {
    statusLine.textContent = text;
  }
}

// ------------------------------------------------------------------- viewer

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.localClippingEnabled = true; // for the section sliders
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x26292e);
const scene = new THREE.Scene();
// Two cameras: perspective for a field of view above 0°, orthographic (as
// if from infinitely far away) at 0°. Changing the field of view moves the
// camera so the model (its bounding sphere) stays about the same size, and
// the camera stays outside it even at wide angles.
const perspective = new THREE.PerspectiveCamera(35, 1, 0.1, 10000);
const orthographic = new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 10000);
perspective.up.set(0, 0, 1);
orthographic.up.set(0, 0, 1);
let camera = perspective;
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
let sceneSize = 100; // the shown model's size, for the clipping planes
let aspect = 1;

// How far out the view is: 1 when the model's bounding sphere just fits the
// view's height, larger when zoomed out.
function viewScale() {
  const radius = sceneSize / 2;
  if (camera.isOrthographicCamera) return (camera.top - camera.bottom) / camera.zoom / (2 * radius);
  return camera.position.distanceTo(controls.target) * Math.sin(camera.fov * Math.PI / 360) / radius;
}
// Point the camera from `direction` (toward it from the target), with the
// field of view `fov`, zoomed out by `scale` (see viewScale).
function placeCamera(fov, scale, direction) {
  const next = fov > 0 ? perspective : orthographic;
  const radius = sceneSize / 2;
  const height = 2 * radius * scale; // for the orthographic camera
  let distance;
  if (fov > 0) {
    perspective.fov = fov;
    distance = scale * radius / Math.sin(fov * Math.PI / 360);
    perspective.near = distance / 100;
    perspective.far = distance + sceneSize * 20;
  } else {
    // far enough to see the whole model in front of it
    distance = sceneSize * 5;
    orthographic.zoom = 1;
    orthographic.top = height / 2;
    orthographic.bottom = -height / 2;
    orthographic.near = 0;
    orthographic.far = distance + sceneSize * 20;
  }
  next.position.copy(controls.target).addScaledVector(direction.clone().normalize(), distance);
  if (next !== camera) {
    camera = next;
    controls.object = camera;
  }
  updateAspect();
  controls.update();
}
function updateAspect() {
  perspective.aspect = aspect;
  perspective.updateProjectionMatrix();
  const half = (orthographic.top - orthographic.bottom) / 2;
  orthographic.left = -half * aspect;
  orthographic.right = half * aspect;
  orthographic.updateProjectionMatrix();
}

const fovInput = document.getElementById("fov");
const fovValue = document.getElementById("fov-value");
function setFov(fov) {
  fovInput.value = fov;
  fovValue.textContent = fov + "°";
  placeCamera(fov, viewScale(), camera.position.clone().sub(controls.target));
}
fovInput.addEventListener("input", () => {
  setFov(Number(fovInput.value));
  try { localStorage.setItem("fov", fovInput.value); } catch {}
});

scene.add(new THREE.HemisphereLight(0xffffff, 0x444455, 1.6));
const light = new THREE.DirectionalLight(0xffffff, 1.8);
light.position.set(1, -1.5, 2);
scene.add(light);

const COLORS = { sun: 0x6f93d6, ring: 0xd9a066, planet: 0x7fbf7f };
let group = new THREE.Group();
scene.add(group);
let framedKind = null;
let reframe = false; // set by loading a settings file: frame the next model anew

// Section: cut away everything above a plane square to a gear's axis, to see
// inside (only in this view). 100% is no cut; 0% hides everything. A
// bevel pair is cut with one plane per gear, both through the same
// point of the line where their pitch cones touch, so the teeth meshing there
// show at the top of both cuts.
const sectionInput = document.getElementById("section");
const sectionValue = document.getElementById("section-value");
let sections = []; // per gear (or one for all): { axis, plane, meshes, height(t) }
function updateSections() {
  const t = Number(sectionInput.value) / 100;
  sectionValue.textContent = t >= 1 ? "off" : Math.round(t * 100) + "%";
  for (const sec of sections) {
    // keep the side where -axis . p + h >= 0, that is below height h
    sec.plane.normal.copy(sec.axis).negate();
    sec.plane.constant = t >= 1 ? 1e9 : sec.height(t);
    // at 0% everything is cut away: show nothing
    for (const mesh of sec.meshes) mesh.visible = t > 0;
    outlineCut(sec, t >= 1 || t <= 0 ? null : sec.plane.constant);
  }
}

// The outline of a cut: where each triangle crosses the plane, a segment,
// drawn as a flat strip in the plane a tenth of a module wide (so it scales
// with the gear, not the view), a hair above the plane so the surfaces along
// the cut don't hide it. It also covers the seam between the cut face and
// the sides.
const outlineMaterial = new THREE.MeshBasicMaterial({ color: 0x101114, side: THREE.DoubleSide });
function outlineCut(sec, h) {
  if (sec.outline) {
    sec.outline.geometry.dispose();
    group.remove(sec.outline);
    sec.outline = null;
  }
  if (h === null) return;
  const half = 0.05 * (num("module", 2) || 2);
  const lift = sceneSize * 2e-4;
  const axis = sec.axis;
  const out = [];
  const p = new THREE.Vector3(), q = new THREE.Vector3(), d = new THREE.Vector3(), n = new THREE.Vector3();
  const corner = (base, sd, sn) => out.push(base.x + d.x * sd + n.x * sn, base.y + d.y * sd + n.y * sn, base.z + d.z * sd + n.z * sn);
  for (const { positions: P, indices: I, heights: H } of sec.parts) {
    for (let t = 0; t < I.length; t += 3) {
      const a = I[t], b = I[t + 1], c = I[t + 2];
      const da = H[a] - h, db = H[b] - h, dc = H[c] - h;
      if ((da > 0 && db > 0 && dc > 0) || (da <= 0 && db <= 0 && dc <= 0)) continue;
      // the two edges whose ends are on opposite sides
      const ends = [];
      for (const [i, j, di, dj] of [[a, b, da, db], [b, c, db, dc], [c, a, dc, da]]) {
        if ((di > 0) === (dj > 0)) continue;
        const f = di / (di - dj);
        ends.push(new THREE.Vector3(
          P[3 * i] + f * (P[3 * j] - P[3 * i]),
          P[3 * i + 1] + f * (P[3 * j + 1] - P[3 * i + 1]),
          P[3 * i + 2] + f * (P[3 * j + 2] - P[3 * i + 2])).addScaledVector(axis, lift));
      }
      if (ends.length !== 2) continue;
      p.copy(ends[0]);
      q.copy(ends[1]);
      d.subVectors(q, p);
      if (d.lengthSq() === 0) continue;
      d.normalize();
      n.crossVectors(axis, d); // across the segment, in the plane
      // a rectangle, run past both ends by half the width to close the joints
      corner(p, -half, half); corner(p, -half, -half); corner(q, half, -half);
      corner(p, -half, half); corner(q, half, -half); corner(q, half, half);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(out), 3));
  sec.outline = new THREE.Mesh(geometry, outlineMaterial);
  group.add(sec.outline);
}
sectionInput.addEventListener("input", updateSections);

function resize() {
  const { clientWidth: w, clientHeight: h } = canvas.parentElement;
  renderer.setSize(w, h, false);
  aspect = w / h;
  updateAspect();
  // Resizing clears the canvas: draw again now, not on the next frame, so
  // the model doesn't blink while the column is dragged.
  renderer.render(scene, camera);
}
new ResizeObserver(resize).observe(canvas.parentElement);
// Boxes change width with the column: re-place their units.
new ResizeObserver(() => { for (const name in rows) placeUnit(name); }).observe(document.getElementById("fields"));
resize();
try {
  const saved = Number(localStorage.getItem("fov"));
  // the camera takes it when the first model is framed
  if (localStorage.getItem("fov") !== null && saved >= 0 && saved <= 179) {
    fovInput.value = saved;
    fovValue.textContent = saved + "°";
  }
} catch {}

(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();

function show(parts, kind, info) {
  scene.remove(group);
  group.traverse((o) => o.geometry && o.geometry.dispose());
  group = new THREE.Group();
  // One cut for the whole model, along z; or for a bevel pair one plane per
  // gear, square to its axis (frames from the build: apex, axis toward the
  // apex, pitch cone angle).
  const frames = kind === "bevel" && info && info.bevel_frames;
  const bevel = !!frames && frames.length === parts.length;
  sections = (bevel ? frames : [null]).map((f) => ({
    axis: f ? new THREE.Vector3(...f.axis) : new THREE.Vector3(0, 0, 1),
    apex: f ? new THREE.Vector3(...f.apex) : null,
    cone: f ? f.pitch_cone : 0,
    min: Infinity, max: -Infinity, plane: new THREE.Plane(), meshes: [], parts: [], outline: null,
  }));
  parts.forEach((part, n) => {
    const sec = sections[bevel ? n : 0];
    const P = part.positions;
    const heights = new Float32Array(P.length / 3); // along the axis, for the outline
    for (let i = 0; i < P.length; i += 3) {
      const h = sec.axis.x * P[i] + sec.axis.y * P[i + 1] + sec.axis.z * P[i + 2];
      heights[i / 3] = h;
      if (h < sec.min) sec.min = h;
      if (h > sec.max) sec.max = h;
    }
    sec.parts.push({ positions: P, indices: part.indices, heights });
  });
  if (bevel) {
    // A point rho from the apex along the line where the pitch cones touch
    // is rho cos(cone) below the apex along each gear's axis. One rho sets
    // both planes, from above both gears (t = 1) to below both (t = 0).
    const top = sections.map((sec) => sec.axis.dot(sec.apex));
    let lo = Infinity, hi = -Infinity;
    sections.forEach((sec, i) => {
      lo = Math.min(lo, (top[i] - sec.max) / Math.cos(sec.cone));
      hi = Math.max(hi, (top[i] - sec.min) / Math.cos(sec.cone));
    });
    sections.forEach((sec, i) => {
      sec.height = (t) => top[i] - (hi + t * (lo - hi)) * Math.cos(sec.cone);
    });
  } else {
    const sec = sections[0];
    sec.height = (t) => sec.min + t * (sec.max - sec.min);
  }
  parts.forEach((part, n) => {
    const sec = sections[bevel ? n : 0];
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(part.positions, 3));
    geometry.setIndex(new THREE.BufferAttribute(part.indices, 1));
    geometry.computeVertexNormals();
    let color = COLORS.sun;
    if (kind === "ring" || (kind === "planetary" && n === parts.length - 1)) color = COLORS.ring;
    else if ((kind === "planetary" || kind === "bevel") && n > 0) color = COLORS.planet;
    const clippingPlanes = [sec.plane];
    const material = new THREE.MeshStandardMaterial({ color, metalness: 0.1, roughness: 0.6, flatShading: true, clippingPlanes });
    // The cut face. Through a cut, a closed solid shows the inside of its far
    // side exactly where the cut face would be; drawing those back faces in
    // one flat, unlit, lighter color makes them look like a flat cut face.
    const inside = new THREE.MeshBasicMaterial({ color: new THREE.Color(color).lerp(new THREE.Color(0xffffff), 0.45), side: THREE.BackSide, clippingPlanes });
    const meshes = [new THREE.Mesh(geometry, material), new THREE.Mesh(geometry, inside)];
    group.add(...meshes);
    sec.meshes.push(...meshes);
  });
  updateSections();
  scene.add(group);
  // Re-frame only when the type or layout changes, so the view stays put while editing.
  const key = kind + "/" + inputs.layout.value;
  // A new type or layout, or a loaded file: frame it anew, uncut.
  if (key !== framedKind || reframe) {
    sectionInput.value = 100;
    updateSections();
    frame();
    framedKind = key;
    reframe = false;
  }
}

function frame() {
  const box = new THREE.Box3().setFromObject(group);
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length();
  sceneSize = size;
  controls.target.copy(center);
  // the view a 35° camera gets from 1.49 sizes away, at any field of view
  placeCamera(Number(fovInput.value), 0.9, new THREE.Vector3(0.6, -1.1, 0.8));
}

// ------------------------------------------------------------- generating
//
// Every change starts a new build (after a short pause in typing). If a
// build takes longer than AUTO_LIMIT seconds, automatic building stops and
// the button comes back, until a build is quick again.

const AUTO_LIMIT = 2.0;
const worker = new Worker("worker.js", { type: "module" });
let ready = false;
let auto = true;
let busy = false;
let pending = false; // a change arrived while building
let requestId = 0;
let current = null; // { parts, kind }
let startedAt = 0;

function setButton(state) {
  // After a gear error the button offers to find values that work (see search.js).
  if (state === "failed" && !searching) state = "find";
  generate.dataset.state = state;
  generate.classList.toggle("state", !["manual", "find", "searching"].includes(state));
  generate.classList.toggle("done", state === "done");
  generate.classList.toggle("failed", state === "failed" || state === "stale");
  generate.disabled = !["manual", "find", "searching"].includes(state);
  generate.textContent = {
    loading: "Loading…", building: "Generating…", done: "✓ Generated", failed: "✗ Not generated",
    stale: "✗ Not generated", manual: "Generate", find: "Search", searching: "Searching… (stop)",
  }[state];
}

function requestBuild() {
  if (!ready) return;
  const read = readOptions();
  if (read.errors) {
    setStatus(read.errors.join("\n"), true);
    setButton(auto ? "stale" : "manual");
    return;
  }
  if (busy) {
    pending = true;
    return;
  }
  busy = true;
  requestId += 1;
  startedAt = performance.now();
  setButton("building");
  buildingParams = read.params;
  worker.postMessage({ id: requestId, params: read.params });
}

function finished(seconds, failed = false) {
  busy = false;
  auto = seconds <= AUTO_LIMIT;
  if (pending) {
    pending = false;
    if (auto) {
      requestBuild();
      return;
    }
  }
  setButton(failed ? "failed" : "done");
}

let typingTimer = null;
form.addEventListener("input", (event) => {
  if (!restoring) recordSoon();
  restoreButton.hidden = true;
  if (event.isTrusted && sliders[event.target.name] && event.target.type === "text") extendRange(event.target.name);
  if (event.target.classList && event.target.classList.contains("changed")) {
    event.target.classList.remove("changed");
    event.target.title = "";
  }
  if (searching) stopSearch();
  showFieldsFor(inputs.kind.value); // labels and help depend on the values
  if (!auto) {
    setButton("manual");
    return;
  }
  clearTimeout(typingTimer);
  typingTimer = setTimeout(requestBuild, event.target.tagName === "SELECT" ? 0 : 300);
});

// ---------------------------------------------------------------- history
//
// Ctrl+Z / Ctrl+Shift+Z (or Ctrl+Y) step through earlier settings: every
// value, auto toggle and choice. Typing is grouped into one step after a
// short pause. Inside a text box, Ctrl+Z first undoes typing in that box
// (the browser's own undo), then, once there's nothing left to undo there,
// the settings.

const history = [];
let historyAt = -1;
let historyTimer = null;
let restoring = false;
let beforeSearch = -1; // history entry from before the last search, for the Restore button

function snapshot() {
  const state = {};
  for (const [name, input] of Object.entries(inputs)) {
    state[name] = [input.value, rows[name].auto ? rows[name].auto.checked : null];
  }
  return state;
}

function record() {
  clearTimeout(historyTimer);
  historyTimer = null;
  const now = snapshot();
  if (historyAt >= 0 && JSON.stringify(history[historyAt]) === JSON.stringify(now)) return;
  history.splice(historyAt + 1);
  history.push(now);
  if (history.length > 200) history.shift();
  historyAt = history.length - 1;
  save();
}

// The browser's memory of the page: every value, choice and toggle, as
//   { "gears2": 1, "values": { "kind": "planetary", "module": 2, "height": "auto", ... },
//     "not_set": ["teeth", ...] }
// ("not_set" lists the values Search may change). Settings files are
// different: see "Settings files" below.
function settingsObject() {
  const values = {}, notSet = [];
  for (const [name, input] of Object.entries(inputs)) {
    const entry = rows[name];
    if (entry.auto && entry.auto.checked) values[name] = "auto";
    else {
      const text = input.value.trim();
      values[name] = input.tagName !== "SELECT" && text !== "" && Number.isFinite(Number(text)) ? Number(text) : input.value;
    }
    if (entry.set && !entry.set.checked) notSet.push(name);
  }
  return { gears2: 1, values, not_set: notSet };
}
// Applies what it can; returns false if it isn't a settings object at all.
function applySettings(obj) {
  if (!obj || typeof obj !== "object" || !obj.values || typeof obj.values !== "object") return false;
  const state = {};
  for (const [name, v] of Object.entries(obj.values)) {
    const input = inputs[name];
    if (!input || !["string", "number"].includes(typeof v)) continue;
    const auto = rows[name].auto;
    const value = v === "auto" && auto ? "" : String(v);
    if (input.tagName === "SELECT" && ![...input.options].some((o) => o.value === value)) continue;
    state[name] = [value, auto ? v === "auto" : null];
  }
  restore(state);
  const notSet = Array.isArray(obj.not_set) ? obj.not_set : [];
  for (const [name, entry] of Object.entries(rows)) if (entry.set) entry.set.checked = !notSet.includes(name);
  return true;
}

// The latest settings are kept in the browser, so closing the tab and
// opening the page again brings back the same gear.
function save() {
  try { localStorage.setItem("settings", JSON.stringify(settingsObject())); } catch {}
}
function load() {
  try { applySettings(JSON.parse(localStorage.getItem("settings"))); } catch {}
}
addEventListener("pagehide", () => { if (historyTimer) record(); else save(); });

function recordSoon() {
  clearTimeout(historyTimer);
  historyTimer = setTimeout(record, 600);
}

function restore(state) {
  restoreButton.hidden = true;
  restoring = true;
  for (const [name, [value, auto]] of Object.entries(state)) {
    const input = inputs[name];
    input.value = value;
    if (auto !== null) {
      rows[name].auto.checked = auto;
      input.disabled = auto;
    }
    input.classList.remove("changed", "invalid");
    input.title = "";
  }
  restoring = false;
  if (searching) stopSearch();
  showFieldsFor(inputs.kind.value);
  requestBuild();
}

function step(by) {
  if (historyTimer) record(); // keep the typing that hasn't been recorded yet
  const to = historyAt + by;
  if (to < 0 || to >= history.length) return;
  historyAt = to;
  restore(history[to]);
}

restoreButton.addEventListener("click", () => {
  if (historyTimer) record();
  if (beforeSearch < 0 || beforeSearch >= history.length) return;
  historyAt = beforeSearch;
  restore(history[beforeSearch]);
});

document.addEventListener("keydown", (event) => {
  if (!(event.ctrlKey || event.metaKey) || event.altKey) return;
  const key = event.key.toLowerCase();
  const redo = (key === "z" && event.shiftKey) || key === "y";
  if (key !== "z" && !redo) return;
  // in a text box with its own typing to undo, let the browser do that
  const t = event.target;
  if (t && t.tagName === "INPUT" && t.type === "text" && !redo && historyTimer) return;
  event.preventDefault();
  step(redo ? 1 : -1);
});

form.addEventListener("submit", (event) => {
  event.preventDefault();
  if (generate.dataset.state === "find") startSearch();
  else if (generate.dataset.state === "searching") stopSearch("Stopped searching.");
  else requestBuild();
});

// ----------------------------------------------------------------- search
//
// The numbers not marked "set" (and shown for this gear type) may change,
// within their sliders' ranges and steps; search.js finds the nearest
// combination that builds, and the page fills it in.

var searching = false; // var: setButton reads it before this line runs
let searchWorker = null;
let searchId = 0;

function startSearch() {
  restoreButton.hidden = true;
  for (const input of form.querySelectorAll("input.changed")) {
    input.classList.remove("changed");
    input.title = "";
  }
  const read = readOptions();
  if (read.errors) return;
  const kind = inputs.kind.value;
  const free = [];
  for (const [name, s] of Object.entries(sliders)) {
    const entry = rows[name];
    if (entry.row.hidden || entry.set.checked) continue;
    const blank = inputs[name].value.trim() === "";
    const value = blank ? (AUTO[name] ? AUTO[name]() : NaN) : num(name, NaN);
    if (!Number.isFinite(value)) continue;
    // the bevel backlash end left on auto stays auto
    if (AUTO_PARTNER[name] && blank) continue;
    free.push({ name, value, min: s.r[0], max: s.r[1], step: s.r[2], whole: !!entry.field.whole, label: (entry.field.labels && entry.field.labels[kind]) || entry.field.label, blank });
  }
  if (!free.length) {
    statusLine.textContent = "Every value is set: turn \"set\" off on the values Search may change, then click Search.";
    return;
  }
  searching = true;
  searchId += 1;
  if (!searchWorker) searchWorker = new Worker("search.js", { type: "module" });
  searchWorker.onmessage = (event) => onSearch(event.data, free);
  searchWorker.postMessage({ id: searchId, params: read.params, free: free.map(({ name, value, min, max, step, whole }) => ({ name, value, min, max, step, whole })), log: searchLogging() });
  setButton("searching");
}

function stopSearch(message) {
  if (searchWorker) searchWorker.terminate();
  searchWorker = null;
  searching = false;
  setButton("failed");
  if (message) statusLine.textContent = message;
}

// For testing Search, not for users: in the browser console,
// searchLog(true) keeps every combination the next searches score;
// downloadSearchLog() saves the last search's as a CSV file (one row per
// combination: the free values as scored, then the score, 0 if it works;
// for the exhaustive search a code: 0 works, 1 fails a check, 2 no gear set).
function searchLogging() {
  try { return localStorage.getItem("search-log") === "on"; } catch { return false; }
}
window.searchLog = (on = true) => {
  try { if (on) localStorage.setItem("search-log", "on"); else localStorage.removeItem("search-log"); } catch {}
  return on ? "Search logging on: run a search, then downloadSearchLog()." : "Search logging off.";
};
let lastSearchLog = null;
window.downloadSearchLog = () => {
  if (!lastSearchLog) return "No search log yet: searchLog(true), then run a search.";
  const { names, rows } = lastSearchLog;
  const width = names.length + 1;
  const lines = [[...names, "score"].join(",")];
  for (let i = 0; i < rows.length; i += width) lines.push(Array.from(rows.subarray(i, i + width)).join(","));
  const url = URL.createObjectURL(new Blob([lines.join("\n") + "\n"], { type: "text/csv" }));
  const a = document.createElement("a");
  a.href = url;
  a.download = "search_log.csv";
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
  return `${rows.length / width} combinations saved.`;
};

function onSearch(msg, free) {
  if (msg.id !== searchId) return;
  if (msg.type === "progress") {
    generate.textContent = `Searching… ${msg.evals.toLocaleString()} tried (stop)`;
    return;
  }
  searching = false;
  if (msg.log) {
    lastSearchLog = msg.log;
    window.lastSearchLog = msg.log;
    console.log(`Search log: ${msg.log.rows.length / (msg.log.names.length + 1)} combinations (downloadSearchLog() saves them).`);
  }
  if (!msg.found) {
    setButton("failed");
    statusLine.textContent = `No working values found by changing the values that aren't set (${msg.evals.toLocaleString()} tried, ` +
      "within the sliders' ranges). Clear more \"set\" boxes, or widen a slider's range (click its ends).";
    return;
  }
  record(); // so Ctrl+Z comes back to the values before the search
  beforeSearch = historyAt;
  let changed = 0;
  msg.values.forEach((v, i) => {
    const f = free[i];
    if (Math.abs(v - f.value) < 1e-9) return;
    if (rows[f.name].auto && rows[f.name].auto.checked) setAuto(f.name, false, false);
    const input = inputs[f.name];
    input.value = String(v);
    input.classList.add("changed");
    input.title = `Changed by Search: was ${f.blank ? "auto (" + +f.value.toFixed(3) + ")" : +f.value.toFixed(3)} (Ctrl+Z to undo)`;
    changed++;
  });
  record();
  restoreButton.hidden = !changed;
  const first = form.querySelector("input.changed");
  if (first) first.scrollIntoView({ block: "nearest" });
  showFieldsFor(inputs.kind.value);
  requestBuild();
}

const defaultSettings = settingsObject(); // what a settings file leaves out
load();
showFieldsFor(inputs.kind.value);
record(); // the starting settings, for Ctrl+Z
setButton("loading");
setStatus("Loading…");

worker.onmessage = (event) => {
  const msg = event.data;
  if (msg.type === "ready") {
    ready = true;
    requestBuild();
    return;
  }
  if (msg.type === "fatal") {
    setStatus("Couldn't load the gear generator: " + msg.message, true);
    return;
  }
  if (msg.type === "step") {
    stepDone(msg);
    return;
  }
  if (msg.id !== requestId) return;
  const seconds = (performance.now() - startedAt) / 1000;
  if (msg.type === "error") {
    setStatus(msg.message, true);
    finished(seconds, true);
    return;
  }
  const kind = inputs.kind.value;
  current = { parts: msg.parts, kind, params: buildingParams };
  show(msg.parts, kind, msg.info);
  download.disabled = false;
  const i = msg.info;
  setStatus(
    `${i.triangles.toLocaleString()} triangles, built in ${seconds.toFixed(2)} s.` +
    (kind === "bevel" ? "" : `\nFewest teeth this gear set allows:\n${i.min_spur_teeth} on a spur gear, ${i.min_ring_teeth} on a ring gear`) +
    (seconds > AUTO_LIMIT ? "\nThat took a while, so changes now wait for Generate." : "")
  );
  finished(seconds);
};

function saveFile(blob, name) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

download.addEventListener("click", () => {
  if (current) saveFile(stlBlob(current.parts), `${current.kind}_gear.stl`);
});

// STEP files hold the exact solids (see src/step.rs): made by the worker on
// request, from the settings of the gears on view.
function updateStepButton(isError = false) {
  downloadStep.disabled = isError || !current || makingStep;
  downloadStep.title = "The exact solids, for CAD programs";
  downloadStep.textContent = makingStep ? "Making STEP…" : "Download STEP";
}
downloadStep.addEventListener("click", () => {
  if (!current || makingStep) return;
  makingStep = true;
  updateStepButton();
  worker.postMessage({ id: 0, params: current.params, step: true, kind: current.kind });
});
function stepDone(msg) {
  makingStep = false;
  updateStepButton(!errorBox.hidden);
  if (msg.text) saveFile(new Blob([msg.text], { type: "model/step" }), `${current.kind}_gear.step`);
  else statusLine.textContent = msg.message;
}

// ---------------------------------------------------------- settings files
//
// A settings file describes one gear, as plain text, one setting per line:
//
//   gears2 version: 0.1.0
//   kind: planetary
//   module: 2
//   height: auto
//   hand: counterclockwise
//
// Only the settings this kind of gear uses are written. When reading, a
// setting that's left out takes its default value, blank lines and anything
// after "#" are ignored, and quotes around a value are optional. A file with
// problems (bad lines, another version or none) isn't loaded, but "Load
// valid settings" takes its good lines; a bad line's setting takes its default.

const VERSION = "0.1.0"; // keep equal to the version in Cargo.toml
const FIELDS = SECTIONS.flatMap((section) => section.fields);

// A choice is written by its value, or by its name when the value is a number.
const choiceText = (input) => {
  const option = input.options[input.selectedIndex];
  return /^-?\d/.test(option.value) ? option.textContent.toLowerCase() : option.value;
};

function settingsText() {
  const kind = inputs.kind.value;
  const lines = [`gears2 version: ${VERSION}`];
  // the kind first, since it decides which settings follow
  for (const f of [FIELDS.find((f) => f.name === "kind"), ...FIELDS.filter((f) => f.name !== "kind")]) {
    if (f.kinds && !f.kinds.includes(kind)) continue;
    const input = inputs[f.name];
    const entry = rows[f.name];
    let value;
    if (input.tagName === "SELECT") value = choiceText(input);
    else if (entry.auto && entry.auto.checked) value = "auto";
    else value = input.value.trim();
    lines.push(`${f.name}: ${value}`);
  }
  return lines.join("\n") + "\n";
}

// The same checks as readOptions, plus the angle limits. Returns the number,
// or a problem as text.
function checkNumber(f, text) {
  const value = Number(text.replace(",", "."));
  if (text === "" || !Number.isFinite(value)) return `must be a number${rows[f.name].auto ? ' or "auto"' : ""}`;
  if (value < 0) return f.name === "helix_angle" ? "can't be negative (change the twist direction instead)" : "can't be negative";
  if (f.whole && !Number.isInteger(value)) return "must be a whole number";
  const limit = LIMITS[f.name];
  if (limit && value > limit[1]) return `must be at most ${limit[1]}`;
  return value;
}

// Returns { values, names, errors, invalid }: values in the browser-memory
// form, with defaults for whatever the file leaves out or gets wrong; names,
// the settings it set; errors, one line per problem; invalid, whether any
// setting was bad (rather than just the version).
function parseSettings(text) {
  const errors = [];
  const values = { ...defaultSettings.values };
  const names = [];
  let version = null;
  text.split(/\r?\n/).forEach((raw, i) => {
    const line = raw.replace(/#.*/, "").trim();
    if (!line) return;
    const at = `Line ${i + 1}`;
    const colon = line.indexOf(":");
    if (colon < 0) { errors.push(`${at} needs a name and a value, like "module: 2".`); return; }
    const name = line.slice(0, colon).trim();
    const value = line.slice(colon + 1).trim().replace(/^(["'])(.*)\1$/, "$2");
    if (name === "gears2 version") { version = value; return; }
    const input = inputs[name];
    const f = FIELDS.find((f) => f.name === name);
    if (!input || !f) { errors.push(`${at}: there's no setting called "${name}".`); return; }
    if (input.tagName === "SELECT") {
      const option = [...input.options].find((o) =>
        o.value === value || o.textContent.toLowerCase() === value.toLowerCase());
      if (!option) {
        const choices = [...input.options].map((o) => /^-?\d/.test(o.value) ? o.textContent.toLowerCase() : o.value);
        errors.push(`${at}: ${name} must be one of ${choices.join(", ")}.`);
        return;
      }
      values[name] = option.value;
    } else if (value === "auto") {
      if (!rows[name].auto) { errors.push(`${at}: ${name} can't be auto.`); return; }
      values[name] = "auto";
    } else {
      const checked = checkNumber(f, value);
      if (typeof checked === "string") { errors.push(`${at}: ${name} ${checked}.`); return; }
      values[name] = checked;
    }
    names.push(name);
  });
  const invalid = errors.length > 0; // problems with the settings themselves
  if (version === null) errors.unshift('It has no "gears2 version" line, so it may not be a gear settings file.');
  else if (version !== VERSION) errors.unshift(`It was saved by gears2 version ${version}; this is version ${VERSION}.`);
  return { values, names, errors, invalid };
}

const settingsFile = document.getElementById("settings-file");
const importError = document.getElementById("import-error");
const importChoices = document.getElementById("import-choices");
const fileButtons = document.getElementById("file-buttons");
let pendingImport = null; // a settings file with problems, for "Load valid settings"
document.getElementById("save-settings").addEventListener("click", () => {
  const url = URL.createObjectURL(new Blob([settingsText()], { type: "text/plain" }));
  const a = document.createElement("a");
  a.href = url;
  a.download = `${inputs.kind.value}_gear_settings.txt`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
document.getElementById("import-settings").addEventListener("click", () => settingsFile.click());
function importSettings({ values, names }) {
  if (historyTimer) record();
  reframe = true;
  // the "set" toggles are about this page, not the gear: keep them
  const notSet = Object.entries(rows).filter(([, e]) => e.set && !e.set.checked).map(([n]) => n);
  applySettings({ values, not_set: notSet });
  for (const name of names) if (sliders[name]) extendRange(name);
  record(); // Ctrl+Z goes back to the settings from before the import
}
// Choosing a file also fires "input", which would reach the form and rebuild
// (clearing the error box): it isn't a setting.
settingsFile.addEventListener("input", (e) => e.stopPropagation());
settingsFile.addEventListener("change", async () => {
  const file = settingsFile.files[0];
  settingsFile.value = ""; // so choosing the same file again still loads it
  if (!file) return;
  const parsed = parseSettings(await file.text());
  if (!parsed.errors.length) {
    importSettings(parsed);
    return;
  }
  importError.textContent = (parsed.invalid ? `Some settings in "${file.name}" are invalid:\n` : `Didn't load "${file.name}":\n`) +
    parsed.errors.join("\n");
  pendingImport = parsed;
  showImportChoices(true);
  document.getElementById("import-anyway").disabled = !parsed.names.length;
  // in view above the buttons docked at the bottom
  importChoices.style.scrollMarginBottom = document.querySelector(".dock").offsetHeight + 8 + "px";
  importChoices.scrollIntoView({ block: "nearest" });
});
// The message and choices take the place of the save and load buttons.
function showImportChoices(on) {
  importError.hidden = !on;
  importChoices.hidden = !on;
  fileButtons.hidden = on;
}
// "Load valid settings": the settings that passed; the rest take their defaults.
document.getElementById("import-anyway").addEventListener("click", () => {
  showImportChoices(false);
  if (pendingImport) importSettings(pendingImport);
  pendingImport = null;
});
document.getElementById("import-cancel").addEventListener("click", () => {
  showImportChoices(false);
  pendingImport = null;
});

// ------------------------------------------------------------ column resize
//
// Drag the divider between the options and the viewer; the width is
// remembered in this browser. On narrow screens the viewer sits above the
// options and the divider sets its height instead.

const layout = document.querySelector(".layout");
const resizer = document.getElementById("resizer");
const MIN_OPTIONS = 300; // narrow columns switch options to two lines

function setOptionsWidth(width) {
  const max = Math.max(MIN_OPTIONS, window.innerWidth - 240);
  width = Math.min(max, Math.max(MIN_OPTIONS, width));
  layout.style.setProperty("--options-width", `${width}px`);
  return width;
}

const stacked = window.matchMedia("(max-width: 1000px)");
function setViewerHeight(height) {
  height = Math.min(window.innerHeight - 200, Math.max(120, height));
  layout.style.setProperty("--viewer-height", `${height}px`);
  return height;
}

try {
  const saved = Number(localStorage.getItem("options-width"));
  if (saved) setOptionsWidth(saved);
  const savedHeight = Number(localStorage.getItem("viewer-height"));
  if (savedHeight) setViewerHeight(savedHeight);
} catch {}

resizer.addEventListener("pointerdown", (event) => {
  event.preventDefault();
  resizer.setPointerCapture(event.pointerId);
  resizer.classList.add("dragging");
  document.body.classList.add("resizing");
  const vertical = stacked.matches;
  const startX = event.clientX, startY = event.clientY;
  const start = vertical ? canvas.parentElement.getBoundingClientRect().height : form.getBoundingClientRect().width;
  let size = start;
  const move = (e) => {
    size = vertical ? setViewerHeight(start + e.clientY - startY) : setOptionsWidth(start + e.clientX - startX);
  };
  const up = () => {
    resizer.removeEventListener("pointermove", move);
    resizer.removeEventListener("pointerup", up);
    resizer.removeEventListener("pointercancel", up);
    resizer.classList.remove("dragging");
    document.body.classList.remove("resizing");
    try { localStorage.setItem(vertical ? "viewer-height" : "options-width", String(Math.round(size))); } catch {}
  };
  resizer.addEventListener("pointermove", move);
  resizer.addEventListener("pointerup", up);
  resizer.addEventListener("pointercancel", up);
});
