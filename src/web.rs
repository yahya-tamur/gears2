//! The WebAssembly interface used by the web page.
//!
//! `build(options)` takes the page's options as a plain JavaScript object
//! (lengths in mm, angles in degrees) and returns either `{ error: message }`
//! for a parameter problem, or `{ parts: [{ positions, indices }, ...], info }`
//! with positions as a Float32Array of x, y, z triples and indices as a
//! Uint32Array of vertex triples.

use crate::bevel::{bevel_pair_mesh, BevelPair};
use crate::gears::{GearError, GearSystem, Params};
use crate::mesh::{planetary_gear_mesh, ring_gear_mesh, spur_gear_mesh, EndZones, IndexedMesh};
use serde::Deserialize;
use std::f64::consts::PI;
use wasm_bindgen::prelude::*;

/// The page's options (see web/index.html).
#[derive(Deserialize, Clone)]
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

/// The gear set for the page's options, whether a bevel pair's backlash is
/// at the toe, and the chamfer lengths (auto values filled in).
fn gear_set(o: &Options, quick: bool) -> Result<(GearSystem, bool, f64, f64), GearError> {
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
    let s = if quick { GearSystem::new_quick(&params, 400)? } else { GearSystem::new(&params)? };
    // Chamfers left blank: a 45 degree slope, as tall as the cone's radial run
    // from the tip circle to the fold circle (see mesh::EndZones).
    let slope_45 = s.height + s.clearance + s.module() / 4.0;
    Ok((s, at_toe, o.end_bottom.unwrap_or(slope_45), o.end_top.unwrap_or(slope_45)))
}

/// How far the options are from building: 0 if they pass every check that
/// building makes (without building any meshes), otherwise a positive score
/// that shrinks as they get closer (mm of bore too wide, teeth short of the
/// minimum times the module, and so on; 1000 or more for problems that have
/// no measure). Used to search for working values. (A gear set whose
/// smallest gear would need more than 400 teeth counts as not building.)
pub fn badness(o: &Options) -> f64 {
    const HARD: f64 = 1000.0;
    let degrees = PI / 180.0;
    let whole_or = |v: f64| if v.fract() == 0.0 && v >= 0.0 { Some(v as usize) } else { None };
    // The gear set's own measurable problems first.
    let m = o.module;
    let (pitch, alpha) = (m * PI, o.pressure_angle * degrees);
    let height = o.height.unwrap_or(2.0 * m);
    let mut v = 0.0;
    let backlash = if o.kind == "bevel" { o.backlash_inner.or(o.backlash_outer).unwrap_or(0.0) } else { o.backlash };
    if m > 0.0 && backlash >= pitch / 2.0 {
        v += backlash - pitch / 2.0 + 0.01;
    }
    if m > 0.0 && alpha > 0.0 && height > 0.0 && height / alpha.sin() < pitch * alpha.cos() {
        v += pitch * alpha.cos() - height / alpha.sin();
    }
    if v > 0.0 {
        return 100.0 + v;
    }
    let Ok((s, at_toe, eb, et)) = gear_set(o, true) else { return HARD };
    if !(o.width > 0.0) && o.kind != "bevel" || eb < 0.0 || et < 0.0 || (o.hand != 1 && o.hand != -1) {
        return HARD;
    }
    let ends = eb > 0.0 || et > 0.0;
    let short = |teeth: usize, min: usize| (min.saturating_sub(teeth)) as f64 * m;
    // bore (radius) against the flat face a spur gear's ends leave
    let bore_over = |teeth: usize, bore: f64| {
        let z = EndZones::new(&s, teeth, o.width, eb, et, o.hand, false);
        let limit = if ends { z.r_fold } else { z.r_root };
        (bore / 2.0 - (limit - 2.0 * s.ds)).max(0.0)
    };
    let rim_under = |teeth: usize| {
        let z = EndZones::new(&s, teeth, o.width, eb, et, o.hand, true);
        let limit = if ends { z.r_fold } else { z.r_root };
        ((limit + 2.0 * s.ds) - o.rim / 2.0).max(0.0)
    };
    match o.kind.as_str() {
        "spur" => {
            let Some(z) = whole_or(o.teeth) else { return HARD };
            let z = z.max(3);
            short(z, s.min_spur_teeth) + bore_over(z, o.bore)
        }
        "ring" => {
            let Some(z) = whole_or(o.ring_teeth) else { return HARD };
            short(z, s.min_ring_teeth) + rim_under(z.max(3))
        }
        "planetary" => {
            let (Some(zs), Some(zp), Some(n)) = (whole_or(o.sun_teeth), whole_or(o.planet_teeth), whole_or(o.planets)) else {
                return HARD;
            };
            if n < 1 {
                return HARD;
            }
            let (zs, zp) = (zs.max(3), zp.max(3));
            let zr = zs + 2 * zp;
            let mut v = short(zs, s.min_spur_teeth) + short(zp, s.min_spur_teeth) + short(zr, s.min_ring_teeth);
            let off = (zs + zr) % n;
            v += off.min(n - off) as f64 * m;
            if n > 1 {
                let a = s.pitch_radius(zs) + s.pitch_radius(zp);
                let gap = 2.0 * a * (PI / n as f64).sin() - 2.0 * (s.pitch_radius(zp) + s.height / 2.0);
                v += (s.clearance - gap).max(0.0);
            }
            v + bore_over(zs, o.bore) + bore_over(zp, o.planet_bore) + rim_under(zr)
        }
        "bevel" => {
            let (Some(z1), Some(z2)) = (whole_or(o.teeth), whole_or(o.teeth2)) else { return HARD };
            let Ok(pair) = BevelPair::checked(&s, [z1, z2], o.shaft_angle * degrees, o.face_width, at_toe) else {
                return HARD;
            };
            let toe = pair.r_cone - pair.face;
            let mut v = 0.0;
            for (i, bore) in [o.bore, o.bore2].into_iter().enumerate() {
                let g = &pair.gears[i];
                let fold = g.root - 0.25 * s.module() / pair.r_cone;
                v += (bore / 2.0 - (toe * fold.sin() - 2.0 * s.ds)).max(0.0) * (bore > 0.0) as i32 as f64;
            }
            v
        }
        _ => HARD,
    }
}

/// badness() for the page (see web/worker.js's search).
#[wasm_bindgen]
pub fn assess(options: JsValue) -> f64 {
    match serde_wasm_bindgen::from_value::<Options>(options) {
        Ok(o) => badness(&o),
        Err(_) => 1e9,
    }
}

/// Build the meshes for the page's options (shared by the WebAssembly entry
/// point and the tests).
pub fn build_parts(o: &Options) -> Result<(Vec<IndexedMesh>, Info), GearError> {
    let degrees = PI / 180.0;
    let (s, at_toe, end_bottom, end_top) = gear_set(o, false)?;
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
