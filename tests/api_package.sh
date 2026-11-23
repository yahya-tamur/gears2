#!/usr/bin/env bash
# Tests web/gears2-api.tgz (see tools/pack_api.sh) the way users get it:
# installed with npm into an empty project, then imported by name.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
dir="$(mktemp -d)"
trap 'rm -rf "$dir"' EXIT
cd "$dir"
npm init -y >/dev/null
npm install --silent --no-audit --no-fund "$root/web/gears2-api.tgz"
cat > check.mjs <<'EOF'
import * as gears from "gears2";
const stl = await gears.stl({ kind: "spur", teeth: 30, max_deviation: 0.05, ds: 0.1 });
const count = new DataView(stl.buffer).getUint32(80, true);
if (!(count > 0 && stl.length === 84 + 50 * count)) throw new Error("bad STL");
console.log(`gears2-api.tgz: installed and made a ${count}-triangle STL`);
EOF
node check.mjs
