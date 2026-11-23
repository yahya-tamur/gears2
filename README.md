# gears2

Generates herringbone gears (spur, ring, planetary sets and bevel pairs) as
STL files for 3D printing or STEP files for CAD.

- **In the browser:** use the page at
  <https://yahya-tamur.github.io/gears2/>. Set the options, see the gears in
  3D, and download the files.
- **From JavaScript (browsers, Deno):**
  ```js
  import * as gears from "https://yahya-tamur.github.io/gears2/api.js";
  const stl = await gears.stl({ kind: "planetary", sun_teeth: 21, planet_teeth: 21 });
  ```
- **From Node:**
  ```bash
  npm install https://yahya-tamur.github.io/gears2/gears2-api.tgz
  ```
  ```js
  import * as gears from "gears2";
  ```

[API.md](API.md) lists every option and function.

Everything runs locally in WebAssembly, compiled from the Rust code in
`src/`. To build it yourself: `cargo test`, then
`wasm-pack build --release --target web --out-dir web/pkg --no-typescript`,
and serve `web/`.
