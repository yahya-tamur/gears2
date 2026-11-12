//! The WebAssembly interface used by the web page.
//!
//! `build(options)` takes the page's options as a plain JavaScript object
//! (lengths in mm, angles in degrees) and returns either `{ error: message }`
//! for a parameter problem, or `{ parts: [{ positions, indices }, ...], info }`
//! with positions as a Float32Array of x, y, z triples and indices as a
//! Uint32Array of vertex triples.

use crate::bevel::{bevel_pair_layout, BevelFrame, BevelPair};
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
    /// Each bevel gear's apex, axis and pitch cone, for the section view.
    pub bevel_frames: Option<[BevelFrame; 2]>,
}

fn whole(value: f64, name: &str) -> Result<usize, GearError> {
    if value.fract() != 0.0 || value < 0.0 {
        return Err(GearError(format!("{} must be a whole number.", name)));
    }
    Ok(value as usize)
}

/// The gear set for the page's options, whether a bevel pair's backlash is
/// at the toe, and the chamfer lengths (auto values filled in).
/// The gear set's parameters, and whether a bevel pair's backlash is given
/// at the toe.
fn gear_params(o: &Options) -> Result<(Params, bool), GearError> {
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
    Ok((params, at_toe))
}

pub(crate) fn gear_set(o: &Options, quick: bool) -> Result<(GearSystem, bool, f64, f64), GearError> {
    let (params, at_toe) = gear_params(o)?;
    let s = if quick { quick_gear_set(&params)? } else { GearSystem::new(&params)? };
    // Chamfers left blank: a 45 degree slope, as tall as the cone's radial run
    // from the tip circle to the fold circle (see mesh::EndZones).
    let slope_45 = s.height + s.clearance + s.module() / 4.0;
    Ok((s, at_toe, o.end_bottom.unwrap_or(slope_45), o.end_top.unwrap_or(slope_45)))
}

thread_local! {
    /// Gear sets already worked out by quick_gear_set (Search scores many
    /// combinations that share one).
    static GEAR_SETS: std::cell::RefCell<std::collections::HashMap<Vec<u64>, Result<GearSystem, GearError>>> =
        std::cell::RefCell::new(std::collections::HashMap::new());
}

/// GearSystem::new_quick(params, 400), remembered.
fn quick_gear_set(p: &Params) -> Result<GearSystem, GearError> {
    let opt = |v: Option<f64>| v.map_or(u64::MAX, f64::to_bits);
    let key = vec![
        p.pitch.to_bits(), p.pressure_angle.to_bits(), p.height.to_bits(), p.backlash.to_bits(),
        p.max_deviation.to_bits(), p.ds.to_bits(), p.helix_angle.to_bits(),
        opt(p.clearance), opt(p.fillet_radius), opt(p.fillet_deviation),
    ];
    GEAR_SETS.with(|cache| {
        if let Some(r) = cache.borrow().get(&key) {
            return r.clone();
        }
        let r = GearSystem::new_quick(p, 400);
        let mut cache = cache.borrow_mut();
        if cache.len() > 5000 {
            cache.clear(); // keep memory bounded
        }
        cache.insert(key, r.clone());
        r
    })
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
    fit_badness(o, &s, at_toe, eb, et)
}

/// badness() once the gear set is made: the checks on the tooth counts,
/// planets, bores and ring with this gear set (`at_toe`, `eb`, `et` as
/// gear_set returns them).
pub(crate) fn fit_badness(o: &Options, s: &GearSystem, at_toe: bool, eb: f64, et: f64) -> f64 {
    const HARD: f64 = 1000.0;
    let degrees = PI / 180.0;
    let whole_or = |v: f64| if v.fract() == 0.0 && v >= 0.0 { Some(v as usize) } else { None };
    let m = o.module;
    if !(o.width > 0.0) && o.kind != "bevel" || eb < 0.0 || et < 0.0 || (o.hand != 1 && o.hand != -1) {
        return HARD;
    }
    let short = |teeth: usize, min: usize| (min.saturating_sub(teeth)) as f64 * m;
    let bore_over = |teeth: usize, bore: f64| ((bore - largest_bore(o, s, eb, et, teeth)) / 2.0).max(0.0);
    let rim_under = |teeth: usize| ((smallest_rim(o, s, eb, et, teeth) - o.rim) / 2.0).max(0.0);
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
            let Ok(pair) = BevelPair::checked(s, [z1, z2], o.shaft_angle * degrees, o.face_width, at_toe) else {
                return HARD;
            };
            let mut v = 0.0;
            for (i, bore) in [o.bore, o.bore2].into_iter().enumerate() {
                v += ((bore - largest_bevel_bore(s, &pair, i)) / 2.0).max(0.0) * (bore > 0.0) as i32 as f64;
            }
            v
        }
        _ => HARD,
    }
}

/// Largest bore (diameter) a spur gear (or the sun or a planet) with this
/// many teeth can have: the flat face its ends leave, less a margin.
pub(crate) fn largest_bore(o: &Options, s: &GearSystem, eb: f64, et: f64, teeth: usize) -> f64 {
    let z = EndZones::new(s, teeth, o.width, eb, et, o.hand, false);
    let limit = if eb > 0.0 || et > 0.0 { z.r_fold } else { z.r_root };
    2.0 * (limit - 2.0 * s.ds)
}

/// Smallest outer diameter a ring gear with this many teeth can have.
pub(crate) fn smallest_rim(o: &Options, s: &GearSystem, eb: f64, et: f64, teeth: usize) -> f64 {
    let z = EndZones::new(s, teeth, o.width, eb, et, o.hand, true);
    let limit = if eb > 0.0 || et > 0.0 { z.r_fold } else { z.r_root };
    2.0 * (limit + 2.0 * s.ds)
}

/// Largest bore (diameter) bevel gear `which` can have: the flat face at the
/// toe, less a margin.
fn largest_bevel_bore(s: &GearSystem, pair: &BevelPair, which: usize) -> f64 {
    let toe = pair.r_cone - pair.face;
    let fold = pair.gears[which].root - 0.25 * s.module() / pair.r_cone;
    2.0 * (toe * fold.sin() - 2.0 * s.ds)
}

/// Limits that follow directly from the other values, as (name, min, max):
/// one value's check that badness() makes, solved for that value with the
/// rest as they are. Search uses them to skip values that can't work.
/// - backlash: below half the tooth spacing; tooth height: tall enough for
///   the involute to start below the tips.
/// - tooth counts: at least the gear set's minimums (a planet also needs a
///   ring of at least the ring minimum).
/// - bores: at most the flat face the gear's ends leave; ring outer diameter:
///   at least the ring's root circle and ends, plus a margin.
///
/// Only limits that can be worked out are listed (none that depend on the
/// gear set when it doesn't build, none for a tooth count that isn't whole).
pub fn limits(o: &Options) -> Vec<(&'static str, Option<f64>, Option<f64>)> {
    let degrees = PI / 180.0;
    let whole_or = |v: f64| if v.fract() == 0.0 && v >= 3.0 { Some(v as usize) } else { None };
    let mut out = Vec::new();
    let m = o.module;
    if !(m > 0.0) {
        return out;
    }
    let (pitch, alpha) = (m * PI, o.pressure_angle * degrees);
    if o.kind != "bevel" {
        out.push(("backlash", None, Some(pitch / 2.0 - 0.01)));
    }
    if alpha > 0.0 && o.height.is_some() {
        out.push(("height", Some(pitch * alpha.cos() * alpha.sin()), None));
    }
    // and the same check solved for the pressure angle: sin(2 alpha) <= 2 h / pitch
    let height = o.height.unwrap_or(2.0 * m);
    let alpha_max = if 2.0 * height < pitch { 0.5 * (2.0 * height / pitch).asin() } else { PI / 4.0 };
    if 2.0 * height < pitch {
        out.push(("pressure_angle", None, Some(alpha_max / degrees)));
    }
    let Ok((s, at_toe, eb, et)) = gear_set(o, true) else { return out };
    let (min_spur, min_ring) = (s.min_spur_teeth as f64, s.min_ring_teeth as f64);
    match o.kind.as_str() {
        "spur" => {
            out.push(("teeth", Some(min_spur), None));
            if let Some(z) = whole_or(o.teeth) {
                out.push(("bore", None, Some(largest_bore(o, &s, eb, et, z))));
            }
        }
        "ring" => {
            out.push(("ring_teeth", Some(min_ring), None));
            if let Some(z) = whole_or(o.ring_teeth) {
                out.push(("rim", Some(smallest_rim(o, &s, eb, et, z)), None));
            }
        }
        "planetary" => {
            out.push(("sun_teeth", Some(min_spur), None));
            let mut planet_min = min_spur;
            if let Some(zs) = whole_or(o.sun_teeth) {
                planet_min = planet_min.max(((min_ring - zs as f64) / 2.0).ceil());
                out.push(("bore", None, Some(largest_bore(o, &s, eb, et, zs))));
            }
            out.push(("planet_teeth", Some(planet_min), None));
            if let Some(zp) = whole_or(o.planet_teeth) {
                out.push(("planet_bore", None, Some(largest_bore(o, &s, eb, et, zp))));
                if let Some(zs) = whole_or(o.sun_teeth) {
                    out.push(("rim", Some(smallest_rim(o, &s, eb, et, zs + 2 * zp)), None));
                }
            }
        }
        "bevel" => {
            if let (Some(z1), Some(z2)) = (whole_or(o.teeth), whole_or(o.teeth2)) {
                if let Ok(pair) = BevelPair::checked(&s, [z1, z2], o.shaft_angle * degrees, o.face_width, at_toe) {
                    out.push(("bore", None, Some(largest_bevel_bore(&s, &pair, 0))));
                    out.push(("bore2", None, Some(largest_bevel_bore(&s, &pair, 1))));
                }
            }
        }
        _ => {}
    }
    out
}

/// The nearest tooth height, and separately the nearest pressure angle, on
/// its slider's steps (`height`, `pressure`: [min, max, step], or None to
/// leave that one alone), at which the gear set builds and these tooth counts
/// reach its minimums, with everything else as it is: (name, min, max), a
/// min when that value is above the current one, a max when below. Empty
/// when the tooth counts fit already. They're alternatives (either fixes the
/// tooth counts).
///
/// Neither works in one direction only (too short a tooth needs more teeth,
/// as does too small a pressure angle; too large a one makes pointed tips),
/// so each is found by trying slider steps outward from the current value,
/// nearest first. Search moves along the same steps, so the gear sets tried
/// here are mostly ones already worked out (see quick_gear_set).
pub fn tooth_fit_limits(o: &Options, height: Option<[f64; 3]>, pressure: Option<[f64; 3]>) -> Vec<(&'static str, Option<f64>, Option<f64>)> {
    let whole = |v: f64| v.fract() == 0.0 && v >= 3.0;
    let needs: Vec<(f64, bool)> = match o.kind.as_str() {
        "spur" => vec![(o.teeth, false)],
        "ring" => vec![(o.ring_teeth, true)],
        "planetary" => vec![(o.sun_teeth, false), (o.planet_teeth, false), (o.sun_teeth + 2.0 * o.planet_teeth, true)],
        _ => return vec![],
    };
    if !(o.module > 0.0) || !needs.iter().all(|&(z, _)| whole(z)) {
        return vec![];
    }
    // the smallest spur count, and the ring's (see GearSystem::teeth_fit)
    let spur = needs.iter().filter(|n| !n.1).map(|n| n.0 as usize).min();
    let ring = needs.iter().find(|n| n.1).map(|n| n.0 as usize);
    let fits = |o: &Options| match gear_params(o) {
        Ok((p, _)) => GearSystem::teeth_fit(&p, spur.unwrap_or(ring.unwrap_or(9) / 3), ring, 400),
        Err(_) => false,
    };
    if fits(o) {
        return vec![];
    }
    let nearest = |name: &'static str, cur: f64, [lo, hi, step]: [f64; 3], set: &dyn Fn(f64) -> Options| {
        if !(step > 0.0) {
            return None;
        }
        let on_grid = |k: f64| ((lo + k * step) * 1e9).round() / 1e9;
        let here = ((cur - lo) / step).round();
        let steps = ((hi - lo) / step + 1e-9).floor();
        for d in 1..=steps as i64 {
            for k in [here + d as f64, here - d as f64] {
                if k < 0.0 || k > steps {
                    continue;
                }
                let v = on_grid(k);
                if fits(&set(v)) {
                    return Some(if v > cur { (name, Some(v), None) } else { (name, None, Some(v)) });
                }
            }
        }
        None
    };
    let mut out = Vec::new();
    if let (Some(h), Some(grid)) = (o.height, height) {
        out.extend(nearest("height", h, grid, &|v| Options { height: Some(v), ..o.clone() }));
    }
    if let Some(grid) = pressure {
        out.extend(nearest("pressure_angle", o.pressure_angle, grid, &|v| Options { pressure_angle: v, ..o.clone() }));
    }
    out
}

/// limits() for the page: { name: [min or null, max or null] }.
#[wasm_bindgen(js_name = limits)]
pub fn limits_js(options: JsValue) -> JsValue {
    let result = js_sys::Object::new();
    if let Ok(o) = serde_wasm_bindgen::from_value::<Options>(options) {
        for (name, lo, hi) in limits(&o) {
            let pair = js_sys::Array::new();
            pair.push(&lo.map_or(JsValue::NULL, JsValue::from));
            pair.push(&hi.map_or(JsValue::NULL, JsValue::from));
            set(&result, name, &pair);
        }
    }
    result.into()
}

impl Options {
    /// A number field by its name (a blank optional one as None).
    pub fn get(&self, name: &str) -> Option<f64> {
        Some(match name {
            "module" => self.module,
            "pressure_angle" => self.pressure_angle,
            "helix_angle" => self.helix_angle,
            "height" => return self.height,
            "backlash" => self.backlash,
            "clearance" => return self.clearance,
            "fillet_radius" => return self.fillet_radius,
            "fillet_deviation" => return self.fillet_deviation,
            "max_deviation" => self.max_deviation,
            "ds" => self.ds,
            "end_bottom" => return self.end_bottom,
            "end_top" => return self.end_top,
            "width" => self.width,
            "bore" => self.bore,
            "planet_bore" => self.planet_bore,
            "rim" => self.rim,
            "teeth" => self.teeth,
            "ring_teeth" => self.ring_teeth,
            "sun_teeth" => self.sun_teeth,
            "planet_teeth" => self.planet_teeth,
            "planets" => self.planets,
            "teeth2" => self.teeth2,
            "shaft_angle" => self.shaft_angle,
            "face_width" => self.face_width,
            "bore2" => self.bore2,
            "backlash_inner" => return self.backlash_inner,
            "backlash_outer" => return self.backlash_outer,
            _ => return None,
        })
    }

    /// Sets a number field by its name; false if there's no such field.
    pub fn set(&mut self, name: &str, v: f64) -> bool {
        match name {
            "module" => self.module = v,
            "pressure_angle" => self.pressure_angle = v,
            "helix_angle" => self.helix_angle = v,
            "height" => self.height = Some(v),
            "backlash" => self.backlash = v,
            "clearance" => self.clearance = Some(v),
            "fillet_radius" => self.fillet_radius = Some(v),
            "fillet_deviation" => self.fillet_deviation = Some(v),
            "max_deviation" => self.max_deviation = v,
            "ds" => self.ds = v,
            "end_bottom" => self.end_bottom = Some(v),
            "end_top" => self.end_top = Some(v),
            "width" => self.width = v,
            "bore" => self.bore = v,
            "planet_bore" => self.planet_bore = v,
            "rim" => self.rim = v,
            "teeth" => self.teeth = v,
            "ring_teeth" => self.ring_teeth = v,
            "sun_teeth" => self.sun_teeth = v,
            "planet_teeth" => self.planet_teeth = v,
            "planets" => self.planets = v,
            "teeth2" => self.teeth2 = v,
            "shaft_angle" => self.shaft_angle = v,
            "face_width" => self.face_width = v,
            "bore2" => self.bore2 = v,
            "backlash_inner" => self.backlash_inner = Some(v),
            "backlash_outer" => self.backlash_outer = Some(v),
            _ => return false,
        }
        true
    }
}

/// A value Search may change: its slider's range and step, and the value it
/// starts from.
#[derive(Clone, Debug)]
pub struct Free {
    pub name: String,
    pub min: f64,
    pub max: f64,
    pub step: f64,
    pub value: f64,
}

/// Values only their own check depends on: Search moves them to their limit
/// instead of searching them.
pub const SOLVED: [&str; 4] = ["bore", "bore2", "planet_bore", "rim"];
/// Values with a limit that also matter elsewhere: searched, and moved to
/// the limit when past it.
pub const LIMITED: [&str; 7] = ["teeth", "ring_teeth", "sun_teeth", "planet_teeth", "height", "pressure_angle", "backlash"];

/// Scores one combination for Search: `values` (one per free value) put into
/// `base`, then every free value with a limit (see limits()) that's past it
/// moved to the nearest slider step inside it, tooth counts and the like
/// first (a planet's minimum depends on the sun), then bores and the ring.
/// Returns badness() and the values as moved.
pub fn settle(base: &Options, free: &[Free], values: &[f64]) -> (f64, Vec<f64>) {
    let mut o = base.clone();
    for (f, &v) in free.iter().zip(values) {
        o.set(&f.name, v);
    }
    let snap = |f: &Free, v: f64, up: bool| {
        let k = (v - f.min) / f.step;
        let k = if up { (k - 1e-9).ceil() } else { (k + 1e-9).floor() };
        ((f.min + k * f.step) * 1e9).round() / 1e9
    };
    // limits(), and tooth_fit_limits() as alternatives: either of those fixes
    // the tooth counts, so only the smaller move (as a part of its slider's
    // range) is made.
    let clamp = |o: &mut Options, names: &[&str]| {
        let mut moves = Vec::new(); // (name, new value, part of the range, an alternative)
        let direct = limits(o).into_iter().map(|l| (l, false));
        let wanted = free.iter().any(|f| f.name == "height" || f.name == "pressure_angle");
        let grid = |n: &str| free.iter().find(|f| f.name == n).map(|f| [f.min, f.max, f.step]);
        let either = if names.contains(&"height") && wanted {
            tooth_fit_limits(o, grid("height"), grid("pressure_angle"))
        } else {
            vec![]
        };
        for ((name, lo, hi), alternative) in direct.chain(either.into_iter().map(|l| (l, true))) {
            if !names.contains(&name) {
                continue;
            }
            let Some(f) = free.iter().find(|f| f.name == name) else { continue };
            let Some(v) = o.get(name) else { continue };
            let mut w = v;
            if let Some(lo) = lo {
                if w < lo {
                    w = snap(f, lo, true).min(f.max);
                }
            }
            if let Some(hi) = hi {
                if w > hi {
                    w = snap(f, hi, false).max(f.min);
                }
            }
            if w != v {
                moves.push((name, w, (w - v).abs() / (f.max - f.min).max(1e-12), alternative));
            }
        }
        let best = moves.iter().filter(|m| m.3).min_by(|a, b| a.2.total_cmp(&b.2)).map(|m| m.0);
        let mut changed = false;
        for (name, w, _, alt) in moves {
            if !alt || Some(name) == best {
                o.set(name, w);
                changed = true;
            }
        }
        changed
    };
    if free.iter().any(|f| LIMITED.contains(&f.name.as_str())) {
        for _ in 0..3 {
            if !clamp(&mut o, &LIMITED) {
                break;
            }
        }
    }
    if free.iter().any(|f| SOLVED.contains(&f.name.as_str())) {
        clamp(&mut o, &SOLVED);
    }
    let settled = free.iter().map(|f| o.get(&f.name).unwrap_or(f64::NAN)).collect();
    (badness(&o), settled)
}

thread_local! {
    /// The search in progress: the options and the free values (see
    /// search_begin).
    static SEARCH: std::cell::RefCell<Option<(Options, Vec<Free>)>> = const { std::cell::RefCell::new(None) };
}

/// Starts a search: the options, and for each free value its name and
/// [slider min, max, step, starting value] (in `grids`, four numbers each).
#[wasm_bindgen]
pub fn search_begin(options: JsValue, names: Vec<String>, grids: Vec<f64>) -> bool {
    let Ok(o) = serde_wasm_bindgen::from_value::<Options>(options) else { return false };
    let free = names
        .into_iter()
        .enumerate()
        .map(|(i, name)| Free { name, min: grids[4 * i], max: grids[4 * i + 1], step: grids[4 * i + 2], value: grids[4 * i + 3] })
        .collect();
    SEARCH.with(|s| *s.borrow_mut() = Some((o, free)));
    true
}

thread_local! {
    /// The last exhaustive search's log (see search_exhaustive).
    static EXHAUSTIVE_LOG: std::cell::RefCell<Vec<f64>> = const { std::cell::RefCell::new(Vec::new()) };
}

/// search::exhaustive() for the search in progress (steps `fine` times
/// finer): [applies (0 or 1), found, searched every free value,
/// combinations checked, then the free values if found]. With `log`, keeps
/// every combination checked for search_take_log().
#[wasm_bindgen]
pub fn search_exhaustive(fine: f64, log: bool) -> Vec<f64> {
    SEARCH.with(|s| {
        let Some((o, free)) = &*s.borrow() else { return vec![0.0] };
        let mut rows = Vec::new();
        let out = crate::search::exhaustive(o, free, fine, if log { Some(&mut rows) } else { None });
        EXHAUSTIVE_LOG.with(|l| *l.borrow_mut() = rows);
        match out {
            None => vec![0.0],
            Some(r) => {
                let mut v = vec![1.0, r.values.is_some() as i32 as f64, r.complete as i32 as f64, r.evals as f64];
                v.extend(r.values.unwrap_or_default());
                v
            }
        }
    })
}

/// The last exhaustive search's log: per combination checked, the free
/// values, then 0 if it works, 1 if not. Empties it.
#[wasm_bindgen]
pub fn search_take_log() -> Vec<f64> {
    EXHAUSTIVE_LOG.with(|l| std::mem::take(&mut *l.borrow_mut()))
}

/// settle() for the search in progress: [score, the free values as moved].
#[wasm_bindgen]
pub fn search_score(values: Vec<f64>) -> Vec<f64> {
    SEARCH.with(|s| match &*s.borrow() {
        Some((o, free)) => {
            let (score, settled) = settle(o, free, &values);
            std::iter::once(score).chain(settled).collect()
        }
        None => vec![1e9],
    })
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
    let mut bevel_frames = None;
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
        "bevel" => {
            let (parts, frames) = bevel_pair_layout(
                &s,
                [whole(o.teeth, "Gear 1 teeth")?, whole(o.teeth2, "Gear 2 teeth")?],
                o.shaft_angle * degrees,
                o.face_width,
                [o.bore, o.bore2],
                o.hand,
                at_toe,
                o.layout.as_deref() != Some("print"),
            )?;
            bevel_frames = Some(frames);
            parts
        }
        other => return Err(GearError(format!("Unknown gear type {:?}.", other))),
    };
    let info = Info {
        min_spur_teeth: s.min_spur_teeth,
        min_ring_teeth: s.min_ring_teeth,
        triangles: parts.iter().map(|m| m.triangles.len()).sum(),
        bevel_frames,
    };
    Ok((parts, info))
}

fn set(target: &js_sys::Object, name: &str, value: &JsValue) {
    js_sys::Reflect::set(target, &JsValue::from_str(name), value).unwrap();
}

/// The exact solids for a STEP file, as an assembly whose instances are in
/// the order and places of build_parts()'s meshes.
pub fn build_assembly(o: &Options) -> Result<crate::gear_step::Assembly, GearError> {
    use crate::gear_step::{planetary_assembly, ring_gear_solid, spur_gear_solid, Assembly};
    let (s, at_toe, end_bottom, end_top) = gear_set(o, false)?;
    Ok(match o.kind.as_str() {
        "spur" => Assembly::single("spur gear", spur_gear_solid(&s, whole(o.teeth, "Teeth")?, o.width, o.bore, o.hand, end_bottom, end_top)?),
        "ring" => Assembly::single("ring gear", ring_gear_solid(&s, whole(o.ring_teeth, "Teeth")?, o.width, o.rim, o.hand, end_bottom, end_top)?),
        "planetary" => planetary_assembly(
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
        "bevel" => crate::bevel_step::bevel_pair_assembly(
            &s,
            [whole(o.teeth, "Gear 1 teeth")?, whole(o.teeth2, "Gear 2 teeth")?],
            o.shaft_angle * PI / 180.0,
            o.face_width,
            [o.bore, o.bore2],
            o.hand,
            at_toe,
            o.layout.as_deref() != Some("print"),
        )?,
        other => return Err(GearError(format!("Unknown gear type {:?}.", other))),
    })
}

/// A STEP file for the page: { text } or { error }.
#[wasm_bindgen]
pub fn build_step(options: JsValue) -> JsValue {
    console_error_panic_hook::set_once();
    let result = js_sys::Object::new();
    match serde_wasm_bindgen::from_value::<Options>(options) {
        Err(e) => set(&result, "error", &JsValue::from_str(&format!("Bad options: {}", e))),
        Ok(o) => match build_assembly(&o) {
            Err(e) => set(&result, "error", &JsValue::from_str(&e.0)),
            Ok(a) => set(&result, "text", &JsValue::from_str(&a.step(&format!("{}_gear", o.kind)))),
        },
    }
    result.into()
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
            if let Some(frames) = info.bevel_frames {
                let list = js_sys::Array::new();
                for f in frames {
                    let frame = js_sys::Object::new();
                    set(&frame, "apex", &js_sys::Float64Array::from(&f.apex[..]).into());
                    set(&frame, "axis", &js_sys::Float64Array::from(&f.axis[..]).into());
                    set(&frame, "pitch_cone", &JsValue::from(f.pitch_cone));
                    list.push(&frame);
                }
                set(&summary, "bevel_frames", &list);
            }
            set(&result, "info", &summary);
        }
    }
    result.into()
}
