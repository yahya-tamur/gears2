//! Herringbone spur, ring and planetary gears as closed triangle meshes.
//!
//! - `gears`: the tooth system and 2D outlines (GearSystem).
//! - `mesh`: indexed meshes, constrained Delaunay triangulation with
//!   Ruppert refinement, and gear solids.
//! - `web`: the WebAssembly interface used by the web page.

// Parameter checks are written `!(x > 0.0)` on purpose: unlike `x <= 0.0`,
// that also rejects NaN.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

pub mod gears;
pub mod mesh;
pub mod web;
