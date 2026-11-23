// The gear generator as a JavaScript module, for programs: the same Rust code
// the page runs (compiled to WebAssembly), without the page. Works in
// browsers, Deno and Node (18+). See API.md for the full description.
//
//   import * as gears from "https://<site>/api.js";
//   const stl = await gears.stl({ kind: "spur", teeth: 30, bore: 8 });
//
// Every function takes an options object; anything left out takes the page's
// default (defaults.js). Gear problems (too few teeth, a bore too large, ...)
// throw a GearError with the same message the page shows.
//
// The work runs on the calling thread and can take a second or more for
// fine settings: in a page, call it from a Worker to keep the page responsive.

import init, { build, build_step, limits as wasmLimits, assess } from "./pkg/gears2.js";
import { DEFAULTS, OPTIONAL } from "./defaults.js";
import { stlBytes } from "./stl.js";

export { DEFAULTS };
export const VERSION = "0.1.0"; // keep equal to the version in Cargo.toml

export class GearError extends Error {
  constructor(message) {
    super(message);
    this.name = "GearError";
  }
}

let loading = null;

// Loads the WebAssembly module; the other functions call it themselves, so
// it's only needed to load from somewhere else: `wasm` is a URL, a Response,
// bytes (ArrayBuffer or typed array) or a WebAssembly.Module. By default it's
// gears2_bg.wasm next to pkg/gears2.js (read from disk under Node).
export function load(wasm) {
  if (!loading) {
    loading = (async () => {
      let source = wasm;
      if (source === undefined && typeof process !== "undefined" && process.versions && process.versions.node) {
        const url = new URL("./pkg/gears2_bg.wasm", import.meta.url);
        // Node's fetch can't read file: URLs
        if (url.protocol === "file:") source = await (await import("node:fs/promises")).readFile(url);
      }
      await init(source === undefined ? undefined : { module_or_path: source });
    })();
    loading.catch(() => { loading = null; }); // a failed load can be retried
  }
  return loading;
}

const CHOICES = {
  kind: ["spur", "ring", "planetary", "bevel"],
  layout: ["assembled", "print"],
};
const HANDS = { counterclockwise: 1, clockwise: -1, "1": 1, "-1": -1 };

// The complete options the Rust code takes: `options` with the defaults
// filled in and checked. Throws a GearError for unknown names or values of
// the wrong type. "auto" is accepted for null.
export function options(given = {}) {
  if (!given || typeof given !== "object") throw new GearError("Options must be an object.");
  const o = { ...DEFAULTS };
  for (const [name, raw] of Object.entries(given)) {
    if (!(name in DEFAULTS)) throw new GearError(`There's no option called "${name}".`);
    let v = raw === "auto" || raw === undefined ? null : raw;
    if (name === "hand") {
      v = HANDS[String(v).toLowerCase()];
      if (!v) throw new GearError('hand must be 1 or "counterclockwise", or -1 or "clockwise".');
    } else if (CHOICES[name]) {
      if (!CHOICES[name].includes(v)) throw new GearError(`${name} must be one of ${CHOICES[name].join(", ")}.`);
    } else if (v === null) {
      if (!OPTIONAL.has(name)) throw new GearError(`${name} can't be auto.`);
    } else {
      if (typeof v === "string" && v.trim() !== "") v = Number(v);
      if (typeof v !== "number" || !Number.isFinite(v)) throw new GearError(`${name} must be a number.`);
      if (v < 0) throw new GearError(`${name} can't be negative.`);
    }
    o[name] = v;
  }
  // A bevel pair's backlash is set at one end only: setting the outer end
  // alone leaves the inner end (set by default) on auto.
  if (given.backlash_outer != null && given.backlash_outer !== "auto" && !("backlash_inner" in given)) o.backlash_inner = null;
  return o;
}

function fail(result) {
  if (result && result.error) throw new GearError(result.error);
  return result;
}

// The gears as triangle meshes: { parts, info }.
// - parts: one per gear (planetary: sun, planets, ring; bevel: gear 1, gear 2),
//   each { positions: Float32Array of x, y, z triples (mm), indices:
//   Uint32Array of vertex triples (counterclockwise seen from outside) }.
// - info: { triangles, min_spur_teeth, min_ring_teeth, spin: [{ role, teeth,
//   center, axis }] per part, and bevel_frames for a bevel pair }.
export async function generate(given) {
  const o = options(given);
  await load();
  return fail(build(o));
}

// A binary STL file of all the parts, as a Uint8Array.
export async function stl(given) {
  return stlBytes((await generate(given)).parts);
}

// A STEP file (exact solids, an assembly of the parts) as text.
export async function step(given) {
  const o = options(given);
  await load();
  return fail(build_step(o)).text;
}

// The limits on values that follow from the others: { name: [min, max] },
// null where there's no limit, e.g. { teeth: [17, null], bore: [null, 31.2] }.
export async function limits(given) {
  const o = options(given);
  await load();
  return wasmLimits(o);
}

// Whether the options build, without building: { ok, score }, score 0 when
// they do, otherwise larger the further they are from building. Much faster
// than generate().
export async function check(given) {
  const o = options(given);
  await load();
  const score = assess(o);
  return { ok: score === 0, score };
}
