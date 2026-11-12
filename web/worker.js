// Background thread: builds the gears with the Rust code compiled to
// WebAssembly (web/pkg, made by `wasm-pack build --release --target web
// --out-dir web/pkg --no-typescript`), so the page stays responsive.

import init, { build, build_step } from "./pkg/gears2.js";

const ready = init()
  .then(() => self.postMessage({ type: "ready" }))
  .catch((e) => self.postMessage({ type: "fatal", message: e && e.message ? e.message : String(e) }));

self.onmessage = async (event) => {
  const { id, params, step } = event.data;
  await ready;
  if (step) {
    // a STEP file of the same settings: { text } or { error }
    try {
      const result = build_step(params);
      self.postMessage({ type: "step", id, text: result.text, message: result.error });
    } catch (e) {
      self.postMessage({ type: "step", id, message: "Internal error: " + String(e) });
    }
    return;
  }
  try {
    const start = performance.now();
    const result = build(params);
    if (result.error) {
      self.postMessage({ type: "error", id, message: result.error });
      return;
    }
    const info = { ...result.info, seconds: ((performance.now() - start) / 1000).toFixed(2) };
    const transfer = result.parts.flatMap((p) => [p.positions.buffer, p.indices.buffer]);
    self.postMessage({ type: "result", id, parts: result.parts, info }, transfer);
  } catch (e) {
    self.postMessage({ type: "error", id, message: "Internal error: " + String(e) });
  }
};
