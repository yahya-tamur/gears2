//! Herringbone spur, ring and planetary gears as closed triangle meshes.
//!
//! - `bevel`: bevel gear pairs (spherical involutes) and their meshes.
//! - `gears`: the tooth system and 2D outlines (GearSystem).
//! - `mesh`: indexed meshes, constrained Delaunay triangulation with
//!   Ruppert refinement, and gear solids.
//! - `stl`: binary STL files.
//! - `step`: STEP files (exact solids); `gear_step` and `bevel_step`: the
//!   gears as exact solids.
//! - `search`: exhaustive search for working values (for the page's Search).
//! - `web`: the WebAssembly interface used by the web page.

// Parameter checks are written `!(x > 0.0)` on purpose: unlike `x <= 0.0`,
// that also rejects NaN.
#![allow(clippy::neg_cmp_op_on_partial_ord)]

pub mod bevel;
pub mod search;
pub mod stl;
pub mod step;
pub mod gear_step;
pub mod bevel_step;
pub mod gears;
pub mod mesh;
pub mod web;
