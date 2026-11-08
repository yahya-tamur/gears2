//! The WebAssembly interface used by the web page.
//!
//! `build(options)` takes the page's options as a plain JavaScript object
//! (lengths in mm, angles in degrees) and returns either `{ error: message }`
//! for a parameter problem, or `{ parts: [{ positions, indices }, ...], info }`
//! with positions as a Float32Array of x, y, z triples and indices as a
//! Uint32Array of vertex triples.

use crate::bevel::bevel_pair_mesh;
use crate::gears::{GearError, GearSystem, Params};
use crate::mesh::{planetary_gear_mesh, ring_gear_mesh, spur_gear_mesh, IndexedMesh};
use serde::Deserialize;
use std::f64::consts::PI;
use wasm_bindgen::prelude::*;

/// The page's options (see web/index.html).
#[derive(Deserialize)]
pub struct Options {
    pub kind: String,
    pub module: f64,
    pub pressure_angle: f64,
    pub helix_angle: f64,
    pub height: Option<f64>,
    pub backlash: f64,
    pub clearance: Option<f64>,
    pub fillet_radius: Option<f64>,
    pub fillet_deviation: Option<f64>,
    pub max_deviation: f64,
    pub ds: f64,
    pub hand: i32,
    pub end_bottom: Option<f64>,
    pub end_top: Option<f64>,
    pub width: f64,
    pub bore: f64,
    pub planet_bore: f64,
    pub rim: f64,
    pub teeth: f64,
    pub ring_teeth: f64,
    pub sun_teeth: f64,
    pub planet_teeth: f64,
    pub planets: f64,
    // bevel pair
    #[serde(default)]
    pub teeth2: f64,
    #[serde(default)]
    pub shaft_angle: f64,
    #[serde(default)]
    pub face_width: f64,
    #[serde(default)]
    pub bore2: f64,
    #[serde(default)]
    pub backlash_inner: Option<f64>,
    #[serde(default)]
    pub backlash_outer: Option<f64>,
    #[serde(default)]
    pub layout: Option<String>,
}

/// Summary shown under the options.
pub struct Info {
    pub min_spur_teeth: usize,
    pub min_ring_teeth: usize,
    pub triangles: usize,
}

fn whole(value: f64, name: &str) -> Result<usize, GearError> {
    if value.fract() != 0.0 || value < 0.0 {
        return Err(GearError(format!("{} must be a whole number.", name)));
    }
    Ok(value as usize)
}

/// Build the meshes for the page's options (shared by the WebAssembly entry
/// point and the tests).
pub fn build_parts(o: &Options) -> Result<(Vec<IndexedMesh>, Info), GearError> {
    let degrees = PI / 180.0;
    // A bevel pair's backlash is given at one end of the teeth, the other left blank.
    let (backlash, at_toe) = if o.kind == "bevel" {
        match (o.backlash_inner, o.backlash_outer) {
            (Some(b), None) => (b, true),
            (None, Some(b)) => (b, false),
            (Some(_), Some(_)) => {
                return Err(GearError("Set the backlash at one end of the teeth only; leave the other blank (auto).".into()))
            }
            (None, None) => return Err(GearError("Set the backlash at one end of the teeth.".into())),
        }
    } else {
        (o.backlash, false)
    };
    // Tooth height left blank: the standard 2 * module.
    let height = o.height.unwrap_or(2.0 * o.module);
    let mut params = Params::new(o.module * PI, o.pressure_angle * degrees, height, backlash, o.max_deviation, o.ds);
    params.helix_angle = o.helix_angle * degrees;
    params.clearance = o.clearance;
    params.fillet_radius = o.fillet_radius;
    params.fillet_deviation = o.fillet_deviation;
    let s = GearSystem::new(&params)?;
    // Chamfers left blank: a 45 degree slope, as tall as the cone's radial run
    // from the tip circle to the fold circle (see mesh::EndZones).
    let slope_45 = s.height + s.clearance + s.module() / 4.0;
    let (end_bottom, end_top) = (o.end_bottom.unwrap_or(slope_45), o.end_top.unwrap_or(slope_45));
    let parts = match o.kind.as_str() {
        "spur" => vec![spur_gear_mesh(&s, whole(o.teeth, "Teeth")?, o.width, o.bore, o.hand, end_bottom, end_top)?],
        "ring" => vec![ring_gear_mesh(&s, whole(o.ring_teeth, "Teeth")?, o.width, o.rim, o.hand, end_bottom, end_top)?],
        "planetary" => planetary_gear_mesh(
            &s,
            whole(o.sun_teeth, "Sun teeth")?,
            whole(o.planet_teeth, "Planet teeth")?,
            whole(o.planets, "Planets")?,
            o.width,
            o.rim,
            o.bore,
            o.planet_bore,
            o.hand,
            end_bottom,
            end_top,
        )?,
        "bevel" => bevel_pair_mesh(
            &s,
            [whole(o.teeth, "Gear 1 teeth")?, whole(o.teeth2, "Gear 2 teeth")?],
            o.shaft_angle * degrees,
            o.face_width,
            [o.bore, o.bore2],
            o.hand,
            at_toe,
            o.layout.as_deref() != Some("print"),
        )?,
        other => return Err(GearError(format!("Unknown gear type {:?}.", other))),
    };
    let info = Info {
        min_spur_teeth: s.min_spur_teeth,
        min_ring_teeth: s.min_ring_teeth,
        triangles: parts.iter().map(|m| m.triangles.len()).sum(),
    };
    Ok((parts, info))
}

fn set(target: &js_sys::Object, name: &str, value: &JsValue) {
    js_sys::Reflect::set(target, &JsValue::from_str(name), value).unwrap();
}

/// Entry point for the page: see the module documentation.
#[wasm_bindgen]
pub fn build(options: JsValue) -> JsValue {
    console_error_panic_hook::set_once();
    let result = js_sys::Object::new();
    let options: Options = match serde_wasm_bindgen::from_value(options) {
        Ok(o) => o,
        Err(e) => {
            set(&result, "error", &JsValue::from_str(&format!("Bad options: {}", e)));
            return result.into();
        }
    };
    match build_parts(&options) {
        Err(e) => set(&result, "error", &JsValue::from_str(&e.0)),
        Ok((parts, info)) => {
            let list = js_sys::Array::new();
            for m in &parts {
                let positions: Vec<f32> = m.vertices.iter().flat_map(|v| v.iter().map(|&c| c as f32)).collect();
                let indices: Vec<u32> = m.triangles.iter().flat_map(|t| t.iter().map(|&i| i as u32)).collect();
                let part = js_sys::Object::new();
                set(&part, "positions", &js_sys::Float32Array::from(positions.as_slice()).into());
                set(&part, "indices", &js_sys::Uint32Array::from(indices.as_slice()).into());
                list.push(&part);
            }
            set(&result, "parts", &list);
            let summary = js_sys::Object::new();
            set(&summary, "min_spur_teeth", &JsValue::from(info.min_spur_teeth as u32));
            set(&summary, "min_ring_teeth", &JsValue::from(info.min_ring_teeth as u32));
            set(&summary, "triangles", &JsValue::from(info.triangles as u32));
            set(&result, "info", &summary);
        }
    }
    result.into()
}
