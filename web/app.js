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
        help: "The teeth are herringbone: V-shaped, slanting one way in one half of the gear and back in the other. This is the angle of that slant from straight across. 0° gives straight teeth. Slanted teeth run smoother and quieter, and the V cancels the sideways push a single slant would make." },
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

const form = document.getElementById("options");
const generate = document.getElementById("generate");
const download = document.getElementById("download");
const statusLine = document.getElementById("status");
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

function buildField(f) {
  const row = document.createElement("label");
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
  } else {
    input = document.createElement("input");
    input.type = "text";
    input.inputMode = f.whole ? "numeric" : "decimal";
    input.value = f.value ?? "";
    if (f.optional) input.placeholder = f.optional;
    if (f.unit) {
      const unit = document.createElement("span");
      unit.className = "unit";
      unit.textContent = f.unit;
      box.append(unit);
    } else {
      input.classList.add("unitless");
    }
  }
  input.name = f.name;
  input.setAttribute("aria-label", f.label);
  box.prepend(input);
  const tip = document.createElement("span");
  tip.className = "tip";
  tip.textContent = f.help;
  row.append(name, box, tip);
  inputs[f.name] = input;
  rows[f.name] = { row, name, tip, field: f };
  if (typeof f.help === "function") tip.textContent = "";
  return row;
}

buildForm();

function showFieldsFor(kind) {
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
      else if (value < 0) problem = "can't be negative";
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

function setStatus(text, isError = false) {
  statusLine.textContent = text;
  statusLine.classList.toggle("error", isError);
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
}
new ResizeObserver(resize).observe(canvas.parentElement);
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
  generate.classList.toggle("state", state !== "manual");
  generate.classList.toggle("done", state === "done");
  generate.classList.toggle("failed", state === "failed" || state === "stale");
  generate.disabled = state !== "manual";
  generate.textContent = { loading: "Loading…", building: "Generating…", done: "✓ Generated", failed: "✗ Not generated", stale: "✗ Not generated", manual: "Generate" }[state];
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
  showFieldsFor(inputs.kind.value); // labels and help depend on the values
  if (!auto) {
    setButton("manual");
    return;
  }
  clearTimeout(typingTimer);
  typingTimer = setTimeout(requestBuild, event.target.tagName === "SELECT" ? 0 : 300);
});

form.addEventListener("submit", (event) => {
  event.preventDefault();
  requestBuild();
});

showFieldsFor(inputs.kind.value);
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
    (kind === "bevel" ? "" : `\nFewest teeth this gear set allows: ${i.min_spur_teeth} on a spur gear, sun or planet; ` +
    `${i.min_ring_teeth} on a ring gear.`) +
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
const MIN_OPTIONS = 260;

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
