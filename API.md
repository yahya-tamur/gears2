# Programming interface

The gear generator is a static site with no server: all the work runs in
WebAssembly in the browser. Programs can use it in two ways:

- **`api.js`**, a JavaScript module that makes gears without the page. Use it
  from a script in Node or Deno, or from your own web page.
- **`window.gears`** on the page itself, for driving the live site from
  browser automation (Playwright, Puppeteer, Selenium) or the console.

Both take the same **options**, described below.

## Options

These are plain objects, with lengths in mm and angles in degrees. Anything
left out takes the page's default (see [`web/defaults.js`](web/defaults.js)).
`null` or `"auto"` means auto, for the options the page has an "auto" toggle
for.

| Option | Default | For | Meaning |
|---|---|---|---|
| `kind` | `"planetary"` | | `"spur"`, `"ring"`, `"planetary"` or `"bevel"` |
| `module` | 2 | all | tooth size: pitch diameter ÷ teeth |
| `pressure_angle` | 20 | all | |
| `helix_angle` | 30 | all | herringbone slant, 0 for straight teeth |
| `height` | auto | all | tooth height (auto: 2 × module) |
| `backlash` | 0.2 | spur, ring, planetary | |
| `clearance` | auto | all | root gap (auto: module ÷ 4) |
| `fillet_radius` | auto | all | root rounding (auto: the largest that fits) |
| `teeth` | 21 | spur, bevel | spur gear teeth, or bevel gear 1's teeth |
| `ring_teeth` | 63 | ring | |
| `teeth2` | 30 | bevel | gear 2 teeth |
| `shaft_angle` | 90 | bevel | |
| `sun_teeth`, `planet_teeth`, `planets` | 21, 21, 4 | planetary | |
| `face_width` | 12 | bevel | tooth length along the cone |
| `width` | 30 | spur, ring, planetary | face width along the axis |
| `bore` | 10 | spur, planetary, bevel | shaft hole (planetary: the sun's; bevel: gear 1's), 0 for none |
| `bore2` | 8 | bevel | gear 2's shaft hole |
| `planet_bore` | 8 | planetary | |
| `rim` | 140 | ring, planetary | ring gear outer diameter |
| `backlash_inner`, `backlash_outer` | 0.2, auto | bevel | set exactly one; giving only `backlash_outer` makes `backlash_inner` auto |
| `hand` | 1 | all | twist direction: `1` / `"counterclockwise"` or `-1` / `"clockwise"` |
| `end_bottom`, `end_top` | auto | spur, ring, planetary | chamfer heights (auto: a 45° slope) |
| `layout` | `"assembled"` | bevel | `"assembled"` or `"print"` |
| `max_deviation` | 0.01 | all | STL surface deviation |
| `ds` | 0.05 | all | STL minimum point spacing |
| `fillet_deviation` | auto | all | STL deviation at the root rounding |
| `step_deviation` | 0.0001 | all | STEP surface deviation |

The page's "?" hover texts describe each option in more detail.

## `api.js`

```js
import * as gears from "https://<site>/api.js";   // or a local path to web/api.js

const stl  = await gears.stl({ kind: "spur", teeth: 30, bore: 8 });  // Uint8Array
const step = await gears.step({ kind: "spur", teeth: 30, bore: 8 }); // string
```

Every function is async and loads the WebAssembly on first use.

| Function | Returns |
|---|---|
| `generate(options)` | `{ parts, info }`: one part per gear (planetary: sun, planets, ring; bevel: gear 1, gear 2), each `{ positions: Float32Array` (x, y, z triples) `, indices: Uint32Array` (triangles, counterclockwise from outside) `}`. `info` holds `triangles`, `min_spur_teeth`, `min_ring_teeth`, `spin` (per part: `role`, `teeth`, `center`, `axis`) and `bevel_frames` for a bevel pair. |
| `stl(options)` | a binary STL of all the parts, as a `Uint8Array` |
| `step(options)` | a STEP file (exact solids, an assembly of the parts) as a string |
| `limits(options)` | the limits other values set, e.g. `{ teeth: [21, null], bore: [null, 33.8] }` (`null`: no limit) |
| `check(options)` | `{ ok, score }`: a fast check without building, `score` 0 when every check passes, otherwise larger the further off. `generate()` has the final word. |
| `options(options)` | the complete options with the defaults filled in |
| `load(wasm?)` | loads the WebAssembly from somewhere else (a URL, bytes or a `WebAssembly.Module`) |
| `DEFAULTS`, `VERSION` | the default options, and the generator's version |

Gear problems (too few teeth, a bore too large, an unknown option, ...)
reject with a `GearError`, whose message is the one the page shows:

```js
try {
  await gears.generate({ kind: "spur", teeth: 5 });
} catch (e) {
  if (e instanceof gears.GearError) console.log(e.message);
  // Spur gears in this gear set need at least 21 teeth (asked for 5).
}
```

The work runs on the calling thread. In a web page, call it from a Worker
so the page stays responsive.

### Node

```js
// save_gear.mjs: run inside a checkout, with web/pkg built (see below)
import { writeFile } from "node:fs/promises";
import * as gears from "./web/api.js";

await writeFile("sun.stl", await gears.stl({ kind: "spur", teeth: 24, bore: 5 }));
```

Node can't import modules over https, so use a checkout. Build the
WebAssembly once with:

```bash
wasm-pack build --release --target web --out-dir web/pkg --no-typescript
```

Deno and browsers can import `api.js` straight from the site.

## `window.gears` (the page)

On the page, `window.gears` changes the settings just as the controls do. The
preview, undo history and saved settings all follow. Each change waits for
its build.

| Member | Does |
|---|---|
| `get()` | the page's current options (`null` for auto) |
| `set(options, { reset })` | changes the given options (with `reset: true`, the rest go back to their defaults), builds, and resolves to the outcome, as `result()` |
| `result()` | resolves, once nothing is building, to `{ ok: true, info }` or `{ ok: false, error }` |
| `stl()` | the current gears' STL, as a `Uint8Array` |
| `step()` | the current gears' STEP file, as a string |
| `defaults`, `version` | as in `api.js` |

```js
// Playwright
await page.goto("https://<site>/");
const outcome = await page.evaluate(() => gears.set({ kind: "spur", teeth: 40 }));
// { ok: true, info: { triangles: ..., spin: [...], ... } }
const bytes = await page.evaluate(async () => Array.from(await gears.stl()));
```

`set()` rejects with a `GearError` for unknown options or values of the wrong
type, without changing anything. If the values are valid but make no gear, it
resolves to `{ ok: false, error }`, and the page shows that error, as it would
for typed values.
