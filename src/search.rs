//! Exhaustive search for working values (spur, ring and planetary).
//!
//! Only the gear-set values are hard to solve together: module, pressure
//! angle, tooth height, backlash, root gap and root rounding decide the
//! fewest teeth allowed (a numerical check, with windows rather than one
//! direction). Given a gear set, the rest is easy: tooth counts start at
//! the minimums, the planet count is a small whole number, and bores and
//! the ring's outer diameter are solved directly (see web::limits).
//!
//! So: every combination of the free gear-set values on their sliders'
//! steps, in order of distance from the starting values (distance: how far
//! each value moved, as a part of its slider's range, added up). For each,
//! the gear set is made, and the tooth counts and planet count are tried
//! nearest first, each from the minimum the gear set allows; bores and the
//! ring are moved to their limits when past them. The first combination
//! whose distance can't be beaten ends the search: the result is the
//! working combination nearest the starting values, or certainly none on
//! the sliders' steps.
//!
//! With `fine` above 1, the gear-set values move in steps that many times
//! smaller than their sliders' (tooth counts stay whole), for working
//! ranges narrower than a slider step.
//!
//! Free values it doesn't search (helix angle, accuracy, widths, chamfers)
//! stay as they are; Search falls back to its general search if this finds
//! nothing.

use crate::web::{fit_badness, gear_set, largest_bore, smallest_rim, Free, Options};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};

/// The gear-set values searched together.
const CORE: [&str; 6] = ["module", "pressure_angle", "height", "backlash", "clearance", "fillet_radius"];
/// Tooth counts, the planet count, and the values solved directly.
const COUNTS: [&str; 5] = ["teeth", "ring_teeth", "sun_teeth", "planet_teeth", "planets"];
const SOLVED: [&str; 3] = ["bore", "planet_bore", "rim"];
/// The most gear-set combinations it will go through.
pub const MAX_CORE: usize = 400_000;
/// The most combinations it logs.
const MAX_LOG: usize = 1_000_000;

pub struct Outcome {
    /// The working values, one per free value (None if none work).
    pub values: Option<Vec<f64>>,
    /// Gear sets made and combinations checked.
    pub evals: usize,
    /// Whether it searched every free value (if not, a "none" is only for
    /// the ones it searched).
    pub complete: bool,
}

/// A free value's slider steps (divided by `fine`), nearest the starting
/// value first, with their distances (as a part of the range).
fn steps(f: &Free, fine: f64) -> Vec<(f64, f64)> {
    let range = (f.max - f.min).max(1e-12);
    let step = f.step / fine;
    let n = ((f.max - f.min) / step + 1e-9).floor().max(0.0) as usize;
    let mut out: Vec<(f64, f64)> = (0..=n)
        .map(|k| {
            let v = ((f.min + k as f64 * step) * 1e9).round() / 1e9;
            (v, (v - f.value).abs() / range)
        })
        .collect();
    out.sort_by(|a, b| a.1.total_cmp(&b.1));
    out
}

#[derive(PartialEq)]
struct Node(f64, Vec<usize>);
impl Eq for Node {}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.0.total_cmp(&self.0) // smallest distance first
    }
}

/// The search (see the module documentation). `log`, if given, gets every
/// combination checked: the free values, then 0 if it works, 1 if not.
/// None if this kind or these free values aren't for it.
pub fn exhaustive(base: &Options, free: &[Free], fine: f64, log: Option<&mut Vec<f64>>) -> Option<Outcome> {
    if !["spur", "ring", "planetary"].contains(&base.kind.as_str()) {
        return None;
    }
    let idx = |name: &str| free.iter().position(|f| f.name == name);
    let core: Vec<usize> = CORE.iter().filter_map(|n| idx(n)).collect();
    let complete = free.iter().all(|f| [&CORE[..], &COUNTS[..], &SOLVED[..]].concat().contains(&f.name.as_str()));
    let grids: Vec<Vec<(f64, f64)>> = core.iter().map(|&i| steps(&free[i], fine)).collect();
    if grids.iter().any(|g| g.is_empty()) || grids.iter().map(|g| g.len()).product::<usize>() > MAX_CORE {
        return None;
    }
    // the free values as they start
    let mut o = base.clone();
    for f in free {
        o.set(&f.name, f.value);
    }
    let range = |i: usize| (free[i].max - free[i].min).max(1e-12);
    let dist_of = |i: usize, v: f64| (v - free[i].value).abs() / range(i);
    let snap = |i: usize, v: f64, up: bool| {
        let f = &free[i];
        let k = (v - f.min) / f.step;
        let k = if up { (k - 1e-9).ceil() } else { (k + 1e-9).floor() };
        ((f.min + k * f.step) * 1e9).round() / 1e9
    };
    // tooth or planet counts for a free count from `min`, nearest first;
    // a fixed count, if it's at least `min`
    let counts = |name: &str, o: &Options, min: f64| -> Vec<(f64, f64)> {
        match idx(name) {
            Some(i) => steps(&free[i], 1.0).into_iter().filter(|&(v, _)| v >= min && v.fract() == 0.0).collect(),
            None => {
                let v = o.get(name).unwrap_or(0.0);
                if v >= min { vec![(v, 0.0)] } else { vec![] }
            }
        }
    };

    // Logs a combination tried: 0 it works, 1 it fails a check, 2 no gear
    // set at all.
    let log = std::cell::RefCell::new(log);
    let note = |o: &Options, code: f64| {
        if let Some(l) = log.borrow_mut().as_deref_mut() {
            if l.len() < MAX_LOG * (free.len() + 1) {
                l.extend(free.iter().map(|f| o.get(&f.name).unwrap_or(f64::NAN)));
                l.push(code);
            }
        }
    };
    let mut best: Option<(f64, Vec<f64>)> = None;
    let mut evals = 0;
    let mut heap = BinaryHeap::new();
    let mut seen = HashSet::new();
    let start = vec![0; core.len()];
    heap.push(Node(0.0, start.clone()));
    seen.insert(start);
    while let Some(Node(d_core, at)) = heap.pop() {
        if best.as_ref().is_some_and(|b| d_core >= b.0) {
            break;
        }
        for k in 0..at.len() {
            if at[k] + 1 < grids[k].len() {
                let mut next = at.clone();
                next[k] += 1;
                if seen.insert(next.clone()) {
                    let d = next.iter().enumerate().map(|(j, &n)| grids[j][n].1).sum();
                    heap.push(Node(d, next));
                }
            }
        }
        let mut o = o.clone();
        for (k, &n) in at.iter().enumerate() {
            o.set(&free[core[k]].name, grids[k][n].0);
        }
        evals += 1;
        let Ok((s, at_toe, eb, et)) = gear_set(&o, true) else {
            note(&o, 2.0);
            continue;
        };
        let (min_spur, min_ring) = (s.min_spur_teeth as f64, s.min_ring_teeth as f64);
        let bound = |best: &Option<(f64, Vec<f64>)>| best.as_ref().map_or(f64::INFINITY, |b| b.0);
        // Moves a free bore or ring to its limit; false if it can't get there.
        let solve = |o: &mut Options, name: &str, limit: f64, up: bool| -> Option<f64> {
            let Some(i) = idx(name) else { return Some(0.0) };
            let v = o.get(name).unwrap_or(0.0);
            let past = if up { v < limit } else { v > limit };
            if past {
                let w = snap(i, limit, up);
                if w < free[i].min - 1e-9 || w > free[i].max + 1e-9 {
                    return None;
                }
                o.set(name, w);
            }
            Some(dist_of(i, o.get(name).unwrap_or(0.0)))
        };
        let check = |o: &Options, d: f64, best: &mut Option<(f64, Vec<f64>)>, evals: &mut usize| {
            *evals += 1;
            let works = fit_badness(o, &s, at_toe, eb, et) == 0.0;
            note(o, if works { 0.0 } else { 1.0 });
            if works && d < bound(best) {
                *best = Some((d, free.iter().map(|f| o.get(&f.name).unwrap_or(f64::NAN)).collect()));
            }
        };
        match o.kind.as_str() {
            "spur" => {
                let list = counts("teeth", &o, min_spur);
                if list.is_empty() {
                    note(&o, 1.0); // no tooth count on the slider reaches the minimum
                }
                for (t, dt) in list {
                    if d_core + dt >= bound(&best) {
                        break;
                    }
                    let mut o = o.clone();
                    o.teeth = t;
                    let limit = largest_bore(&o, &s, eb, et, t as usize);
                    let Some(db) = solve(&mut o, "bore", limit, false) else {
                        note(&o, 1.0);
                        continue;
                    };
                    check(&o, d_core + dt + db, &mut best, &mut evals);
                }
            }
            "ring" => {
                let list = counts("ring_teeth", &o, min_ring);
                if list.is_empty() {
                    note(&o, 1.0);
                }
                for (t, dt) in list {
                    if d_core + dt >= bound(&best) {
                        break;
                    }
                    let mut o = o.clone();
                    o.ring_teeth = t;
                    let limit = smallest_rim(&o, &s, eb, et, t as usize);
                    let Some(dr) = solve(&mut o, "rim", limit, true) else {
                        note(&o, 1.0);
                        continue;
                    };
                    check(&o, d_core + dt + dr, &mut best, &mut evals);
                }
            }
            _ => {
                // With the ring's outer diameter set: the most ring teeth it
                // has room for (its smallest diameter grows with the teeth).
                let ring_max = if idx("rim").is_none() {
                    let fits = |zr: usize| smallest_rim(&o, &s, eb, et, zr) <= o.rim;
                    if !fits(3) {
                        note(&o, 1.0);
                        continue;
                    }
                    let (mut good, mut bad) = (3, 4096);
                    while bad - good > 1 {
                        let mid = (good + bad) / 2;
                        if fits(mid) { good = mid } else { bad = mid }
                    }
                    good as f64
                } else {
                    f64::INFINITY
                };
                let suns = counts("sun_teeth", &o, min_spur);
                if suns.is_empty() {
                    note(&o, 1.0);
                }
                for (zs, ds) in suns {
                    if d_core + ds >= bound(&best) {
                        break;
                    }
                    let mut o = o.clone();
                    o.sun_teeth = zs;
                    let limit = largest_bore(&o, &s, eb, et, zs as usize);
                    let Some(db) = solve(&mut o, "bore", limit, false) else {
                        note(&o, 1.0);
                        continue;
                    };
                    let planet_min = min_spur.max(((min_ring - zs) / 2.0).ceil());
                    let planets = counts("planet_teeth", &o, planet_min);
                    if planets.is_empty() {
                        note(&o, 1.0);
                    }
                    for (zp, dp) in planets {
                        if d_core + ds + dp >= bound(&best) {
                            break;
                        }
                        let mut o = o.clone();
                        o.planet_teeth = zp;
                        if zs + 2.0 * zp > ring_max {
                            note(&o, 1.0);
                            continue;
                        }
                        let zr = (zs + 2.0 * zp) as usize;
                        // the ring and the planet bore: the same for every planet count
                        let limit = smallest_rim(&o, &s, eb, et, zr);
                        let Some(dr) = solve(&mut o, "rim", limit, true) else {
                            note(&o, 1.0);
                            continue;
                        };
                        let limit = largest_bore(&o, &s, eb, et, zp as usize);
                        let Some(dq) = solve(&mut o, "planet_bore", limit, false) else {
                            note(&o, 1.0);
                            continue;
                        };
                        let d = d_core + ds + dp + dr + db + dq;
                        let a = s.pitch_radius(zs as usize) + s.pitch_radius(zp as usize);
                        for (n, dn) in counts("planets", &o, 1.0) {
                            if d + dn >= bound(&best) {
                                break;
                            }
                            let mut o = o.clone();
                            o.planets = n;
                            // the two planet checks (see web::fit_badness) first, cheaply
                            let k = n as usize;
                            let gap = 2.0 * a * (std::f64::consts::PI / n).sin() - 2.0 * (s.pitch_radius(zp as usize) + s.height / 2.0);
                            if !(zs as usize + zr).is_multiple_of(k) || (k > 1 && gap < s.clearance) {
                                note(&o, 1.0);
                                continue;
                            }
                            check(&o, d + dn, &mut best, &mut evals);
                        }
                    }
                }
            }
        }
    }
    Some(Outcome { values: best.map(|b| b.1), evals, complete })
}
