//! STL files: a closed triangle mesh as binary STL.
//!
//! Binary STL: an 80-byte header (unused, zeros), the triangle count, then
//! per triangle its unit normal and three corners as 32-bit floats and a
//! 2-byte attribute (0). Several bodies go in one file as one mesh (see
//! mesh::combined).

use crate::mesh::IndexedMesh;

impl IndexedMesh {
    /// The mesh as a binary STL file.
    pub fn stl_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(84 + 50 * self.triangles.len());
        out.extend_from_slice(&[0u8; 80]);
        out.extend_from_slice(&(self.triangles.len() as u32).to_le_bytes());
        for &t in &self.triangles {
            let n = self.normal(t);
            let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            let length = if length == 0.0 { 1.0 } else { length };
            for c in n {
                out.extend_from_slice(&((c / length) as f32).to_le_bytes());
            }
            for i in t {
                for c in self.vertices[i] {
                    out.extend_from_slice(&(c as f32).to_le_bytes());
                }
            }
            out.extend_from_slice(&[0u8; 2]);
        }
        out
    }

    /// Write a binary STL.
    pub fn save_stl(&self, path: impl AsRef<std::path::Path>) -> std::io::Result<()> {
        std::fs::write(path, self.stl_bytes())
    }
}
