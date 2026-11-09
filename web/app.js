// The page: builds the options form, asks the worker for meshes whenever an
// option changes, shows them with three.js, and writes a binary STL.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

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
    title: "Smoothness",
    help: "How closely the flat triangles of the STL follow the true curved surfaces. Finer is smoother but makes larger files.",
    fields: [
      { name: "max_deviation", label: "Surface accuracy", unit: "mm", value: "0.01",
        help: "The flat triangles stay within this distance of the exact shape, always cutting into the material, never out of it. So the play between meshing gears can grow by up to twice this, never shrink." },
      { name: "ds", label: "Smallest point spacing", unit: "mm", value: "0.05",
        help: "No two points of the mesh closer than this, which keeps tiny sliver triangles out of the file." },
      { name: "fillet_deviation", label: "Root rounding accuracy", unit: "mm", optional: "auto",
        help: () => `Surface accuracy for the rounded corners at the bottom of each gap between teeth (see Root rounding). Their exact shape doesn't affect how the gears mesh, so they can be coarser. Leave blank for automatic${now(Math.max(0.05, num("max_deviation", NaN)))}; it follows the surface accuracy when that is coarser.` },
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
const RANGES = {
  module: [0.5, 5, 0.25], pressure_angle: [10, 35, 0.5], helix_angle: [0, 45, 1], height: [0.5, 10, 0.1],
  backlash: [0, 1, 0.05], clearance: [0, 2, 0.05], fillet_radius: [0, 3, 0.05],
  teeth: [6, 100, 1], ring_teeth: [20, 200, 1], teeth2: [6, 100, 1], shaft_angle: [10, 170, 1],
  sun_teeth: [6, 60, 1], planet_teeth: [6, 60, 1], planets: [1, 12, 1],
  face_width: [2, 50, 0.5], width: [2, 100, 1], bore: [0, 50, 0.5], bore2: [0, 50, 0.5], planet_bore: [0, 30, 0.5],
  backlash_inner: [0, 1, 0.05], backlash_outer: [0, 1, 0.05], rim: [20, 300, 1],
  end_bottom: [0, 15, 0.25], end_top: [0, 15, 0.25],
  max_deviation: [0.001, 0.1, 0.001], ds: [0.01, 0.5, 0.01], fillet_deviation: [0.005, 0.2, 0.005],
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
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x26292e);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(35, 1, 0.1, 10000);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x444455, 1.6));
const light = new THREE.DirectionalLight(0xffffff, 1.8);
light.position.set(1, -1.5, 2);
scene.add(light);

const COLORS = { sun: 0x6f93d6, ring: 0xd9a066, planet: 0x7fbf7f };
let group = new THREE.Group();
scene.add(group);
let framedKind = null;

function resize() {
  const { clientWidth: w, clientHeight: h } = canvas.parentElement;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  // Resizing clears the canvas: draw again now, not on the next frame, so
  // the model doesn't blink while the column is dragged.
  renderer.render(scene, camera);
}
new ResizeObserver(resize).observe(canvas.parentElement);
// Boxes change width with the column: re-place their units.
new ResizeObserver(() => { for (const name in rows) placeUnit(name); }).observe(document.getElementById("fields"));
resize();

(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();

function show(parts, kind) {
  scene.remove(group);
  group.traverse((o) => o.geometry && o.geometry.dispose());
  group = new THREE.Group();
  parts.forEach((part, n) => {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(part.positions, 3));
    geometry.setIndex(new THREE.BufferAttribute(part.indices, 1));
    geometry.computeVertexNormals();
    let color = COLORS.sun;
    if (kind === "ring" || (kind === "planetary" && n === parts.length - 1)) color = COLORS.ring;
    else if ((kind === "planetary" || kind === "bevel") && n > 0) color = COLORS.planet;
    const material = new THREE.MeshStandardMaterial({ color, metalness: 0.1, roughness: 0.6, flatShading: true });
    group.add(new THREE.Mesh(geometry, material));
  });
  scene.add(group);
  // Re-frame only when the type or layout changes, so the view stays put while editing.
  const key = kind + "/" + inputs.layout.value;
  if (key !== framedKind) {
    frame();
    framedKind = key;
  }
}

function frame() {
  const box = new THREE.Box3().setFromObject(group);
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3()).length();
  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(0.6, -1.1, 0.8).multiplyScalar(size));
  camera.near = size / 100;
  camera.far = size * 20;
  camera.updateProjectionMatrix();
}

// ---------------------------------------------------------------------- STL

function stlBlob(parts) {
  const count = parts.reduce((n, p) => n + p.indices.length / 3, 0);
  const buffer = new ArrayBuffer(84 + 50 * count);
  const view = new DataView(buffer);
  view.setUint32(80, count, true);
  let offset = 84;
  for (const { positions: P, indices: I } of parts) {
    for (let t = 0; t < I.length; t += 3) {
      const a = I[t] * 3, b = I[t + 1] * 3, c = I[t + 2] * 3;
      const ux = P[b] - P[a], uy = P[b + 1] - P[a + 1], uz = P[b + 2] - P[a + 2];
      const vx = P[c] - P[a], vy = P[c + 1] - P[a + 1], vz = P[c + 2] - P[a + 2];
      const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
      const len = Math.hypot(nx, ny, nz) || 1;
      for (const value of [nx / len, ny / len, nz / len]) { view.setFloat32(offset, value, true); offset += 4; }
      for (const i of [a, b, c]) {
        for (let k = 0; k < 3; k++) { view.setFloat32(offset, P[i + k], true); offset += 4; }
      }
      offset += 2;
    }
  }
  return new Blob([buffer], { type: "model/stl" });
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
}

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
    free.push({ name, value, min: s.r[0], max: s.r[1], step: s.r[2], label: (entry.field.labels && entry.field.labels[kind]) || entry.field.label, blank });
  }
  if (!free.length) {
    statusLine.textContent = "Every value is set: turn \"set\" off on the values Search may change, then click Search.";
    return;
  }
  searching = true;
  searchId += 1;
  if (!searchWorker) searchWorker = new Worker("search.js", { type: "module" });
  searchWorker.onmessage = (event) => onSearch(event.data, free);
  searchWorker.postMessage({ id: searchId, params: read.params, free: free.map(({ name, value, min, max, step }) => ({ name, value, min, max, step })) });
  setButton("searching");
}

function stopSearch(message) {
  if (searchWorker) searchWorker.terminate();
  searchWorker = null;
  searching = false;
  setButton("failed");
  if (message) statusLine.textContent = message;
}

function onSearch(msg, free) {
  if (msg.id !== searchId) return;
  if (msg.type === "progress") {
    generate.textContent = `Searching… ${msg.evals.toLocaleString()} tried (stop)`;
    return;
  }
  searching = false;
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
  if (msg.id !== requestId) return;
  const seconds = (performance.now() - startedAt) / 1000;
  if (msg.type === "error") {
    setStatus(msg.message, true);
    finished(seconds, true);
    return;
  }
  const kind = inputs.kind.value;
  current = { parts: msg.parts, kind };
  show(msg.parts, kind);
  download.disabled = false;
  const i = msg.info;
  setStatus(
    `${i.triangles.toLocaleString()} triangles, built in ${seconds.toFixed(2)} s.` +
    (kind === "bevel" ? "" : `\nFewest teeth this gear set allows:\n${i.min_spur_teeth} on a spur gear, ${i.min_ring_teeth} on a ring gear`) +
    (seconds > AUTO_LIMIT ? "\nThat took a while, so changes now wait for Generate." : "")
  );
  finished(seconds);
};

download.addEventListener("click", () => {
  if (!current) return;
  const url = URL.createObjectURL(stlBlob(current.parts));
  const a = document.createElement("a");
  a.href = url;
  a.download = `${current.kind}_gear.stl`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});

// ------------------------------------------------------------ column resize
//
// Drag the divider between the options and the viewer; the width is
// remembered in this browser.

const layout = document.querySelector(".layout");
const resizer = document.getElementById("resizer");
const MIN_OPTIONS = 300; // narrow columns switch options to two lines

function setOptionsWidth(width) {
  const max = Math.max(MIN_OPTIONS, window.innerWidth - 240);
  width = Math.min(max, Math.max(MIN_OPTIONS, width));
  layout.style.setProperty("--options-width", `${width}px`);
  return width;
}

try {
  const saved = Number(localStorage.getItem("options-width"));
  if (saved) setOptionsWidth(saved);
} catch {}

resizer.addEventListener("pointerdown", (event) => {
  event.preventDefault();
  resizer.setPointerCapture(event.pointerId);
  resizer.classList.add("dragging");
  document.body.classList.add("resizing");
  const startX = event.clientX;
  const startWidth = form.getBoundingClientRect().width;
  let width = startWidth;
  const move = (e) => { width = setOptionsWidth(startWidth + e.clientX - startX); };
  const up = () => {
    resizer.removeEventListener("pointermove", move);
    resizer.removeEventListener("pointerup", up);
    resizer.removeEventListener("pointercancel", up);
    resizer.classList.remove("dragging");
    document.body.classList.remove("resizing");
    try { localStorage.setItem("options-width", String(Math.round(width))); } catch {}
  };
  resizer.addEventListener("pointermove", move);
  resizer.addEventListener("pointerup", up);
  resizer.addEventListener("pointercancel", up);
});
