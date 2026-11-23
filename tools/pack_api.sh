#!/usr/bin/env bash
# Packs the JavaScript API (web/api.js and what it needs, with the built
# WebAssembly) as web/gears2-api.tgz, a package npm can install from its URL:
#
#   npm install https://yahya-tamur.github.io/gears2/gears2-api.tgz
#
# Needs web/pkg built first (wasm-pack build --release --target web
# --out-dir web/pkg --no-typescript). Run from anywhere.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
version="$(grep -m1 '^version' "$root/Cargo.toml" | sed -E 's/.*"(.*)".*/\1/')"
stage="$(mktemp -d)"
trap 'rm -rf "$stage"' EXIT

# Copied file by file: web/pkg/.gitignore ignores everything, which npm would
# honor too.
mkdir -p "$stage/package/pkg"
cp "$root"/web/{api.js,defaults.js,stl.js} "$stage/package/"
cp "$root"/web/pkg/{gears2.js,gears2_bg.wasm} "$stage/package/pkg/"
cp "$root/API.md" "$stage/package/README.md"
cat > "$stage/package/package.json" <<EOF
{
  "name": "gears2",
  "version": "$version",
  "description": "Herringbone spur, ring, planetary and bevel gears as STL or STEP files.",
  "type": "module",
  "exports": "./api.js",
  "homepage": "https://yahya-tamur.github.io/gears2/",
  "repository": "github:yahya-tamur/gears2",
  "engines": { "node": ">=18" }
}
EOF

(cd "$stage/package" && npm pack --silent --pack-destination "$stage" >/dev/null)
mv "$stage/gears2-$version.tgz" "$root/web/gears2-api.tgz"
echo "web/gears2-api.tgz (gears2 $version)"
