// STL files: the meshes the worker built, as one binary STL.
//
// Binary STL: an 80-byte header (unused, zeros), the triangle count, then
// per triangle its unit normal and three corners as 32-bit floats and a
// 2-byte attribute (0). parts: [{ positions: Float32Array, indices }].

export function stlBlob(parts) {
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
