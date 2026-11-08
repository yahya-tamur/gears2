// Background thread: builds the gears with the Rust code compiled to
// WebAssembly (web/pkg, made by `wasm-pack build --release --target web
// --out-dir web/pkg --no-typescript`), so the page stays responsive.

import init, { build } from "./pkg/gears2.js";

const ready = init()
  .then(() => self.postMessage({ type: "ready" }))
  .catch((e) => self.postMessage({ type: "fatal", message: e && e.message ? e.message : String(e) }));

self.onmessage = async (event) => {
  const { id, params } = event.data;
  await ready;
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
