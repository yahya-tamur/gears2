// Tests web/api.js under Node. Needs the WebAssembly package built first:
//   wasm-pack build --release --target web --out-dir web/pkg --no-typescript
//   node tests/api.mjs

import assert from "node:assert/strict";
import * as gears from "../web/api.js";

const quick = { max_deviation: 0.05, ds: 0.1 };

// meshes
const spur = await gears.generate({ kind: "spur", teeth: 30, ...quick });
assert.equal(spur.parts.length, 1);
assert.ok(spur.parts[0].positions instanceof Float32Array);
assert.ok(spur.parts[0].indices instanceof Uint32Array);
assert.equal(spur.info.triangles, spur.parts[0].indices.length / 3);

const planetary = await gears.generate({ ...quick, width: 10 }); // the default kind
assert.deepEqual(planetary.info.spin.map((s) => s.role), ["sun", "planet", "planet", "planet", "planet", "ring"]);

// STL: header, count, 50 bytes a triangle
const stl = await gears.stl({ kind: "spur", teeth: 30, ...quick });
assert.ok(stl instanceof Uint8Array);
assert.equal(new DataView(stl.buffer).getUint32(80, true), spur.info.triangles);
assert.equal(stl.length, 84 + 50 * spur.info.triangles);

// STEP
const step = await gears.step({ kind: "ring", ring_teeth: 70, rim: 160, hand: "clockwise", width: 10 });
assert.ok(step.startsWith("ISO-10303-21;"));

// limits and check
const limits = await gears.limits({ kind: "spur", teeth: 20 });
assert.ok(limits.teeth[0] > 0 && limits.teeth[1] === null);
assert.deepEqual(await gears.check({ kind: "spur" }), { ok: true, score: 0 });
assert.equal((await gears.check({ kind: "spur", teeth: 5 })).ok, false);

// a bevel pair with the backlash at the outer end only
const bevel = await gears.generate({ kind: "bevel", backlash_outer: 0.3, ...quick });
assert.equal(bevel.parts.length, 2);

// options: defaults, "auto", hand names
assert.equal(gears.options({}).module, gears.DEFAULTS.module);
assert.equal(gears.options({ height: "auto" }).height, null);
assert.equal(gears.options({ hand: "clockwise" }).hand, -1);

// errors
const fails = async (options, pattern) => {
  await assert.rejects(gears.generate(options), (e) => e instanceof gears.GearError && pattern.test(e.message));
};
await fails({ kind: "spur", teeth: 5 }, /at least \d+ teeth/);
await fails({ colour: 1 }, /no option called "colour"/);
await fails({ teeth: "x" }, /must be a number/);
await fails({ module: null }, /can't be auto/);
await fails({ kind: "worm" }, /must be one of/);
await fails({ bore: -1 }, /can't be negative/);

console.log("api.js: all tests passed");
