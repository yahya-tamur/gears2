//! The exhaustive search finds the working combination nearest the starting
//! values: the same distance as trying every combination on the sliders'
//! steps one by one.

use gears2::search::exhaustive;
use gears2::web::{badness, Free, Options};

fn options(kind: &str) -> Options {
    Options {
        kind: kind.into(), module: 2.0, pressure_angle: 20.0, helix_angle: 30.0, height: None, backlash: 0.2,
        clearance: None, fillet_radius: None, fillet_deviation: Some(0.1), max_deviation: 0.05, ds: 0.1, hand: 1,
        end_bottom: None, end_top: None, width: 10.0, bore: 8.0, planet_bore: 6.0, rim: 140.0, teeth: 21.0,
        ring_teeth: 63.0, sun_teeth: 21.0, planet_teeth: 21.0, planets: 4.0, teeth2: 30.0, shaft_angle: 90.0,
        face_width: 12.0, bore2: 8.0, backlash_inner: Some(0.2), backlash_outer: None, layout: None,
    }
}

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn pick(&mut self, lo: f64, hi: f64, step: f64) -> f64 {
        lo + ((hi - lo) / step * self.next()).round() * step
    }
}

fn free(name: &str, min: f64, max: f64, step: f64, value: f64) -> Free {
    Free { name: name.into(), min, max, step, value }
}

/// The smallest distance of any working combination, trying them all.
fn brute_force(base: &Options, free: &[Free]) -> Option<f64> {
    let grids: Vec<Vec<f64>> = free
        .iter()
        .map(|f| {
            let n = ((f.max - f.min) / f.step + 1e-9).floor() as usize;
            (0..=n).map(|k| ((f.min + k as f64 * f.step) * 1e9).round() / 1e9).collect()
        })
        .collect();
    let mut best: Option<f64> = None;
    let mut at = vec![0; free.len()];
    loop {
        let mut o = base.clone();
        let mut d = 0.0;
        for (i, f) in free.iter().enumerate() {
            let v = grids[i][at[i]];
            o.set(&f.name, v);
            d += (v - f.value).abs() / (f.max - f.min);
        }
        if best.is_none_or(|b| d < b) && badness(&o) == 0.0 {
            best = Some(d);
        }
        // next combination
        let mut k = 0;
        loop {
            if k == at.len() {
                return best;
            }
            at[k] += 1;
            if at[k] < grids[k].len() {
                break;
            }
            at[k] = 0;
            k += 1;
        }
    }
}

#[test]
fn exhaustive_finds_the_nearest_working_combination() {
    let mut rng = Lcg(21);
    let (mut found, mut none) = (0, 0);
    for k in 0..36 {
        let kind = ["planetary", "spur", "ring"][k % 3];
        let mut o = options(kind);
        o.backlash = rng.pick(0.1, 0.8, 0.1);
        o.rim = rng.pick(40.0, 120.0, 10.0);
        let m = rng.pick(1.0, 2.0, 0.25);
        let pa = rng.pick(16.0, 28.0, 2.0);
        let fr: Vec<Free> = match kind {
            "planetary" => vec![
                free("module", 1.0, 2.0, 0.25, m),
                free("pressure_angle", 16.0, 30.0, 2.0, pa),
                free("sun_teeth", 8.0, 20.0, 2.0, rng.pick(8.0, 20.0, 2.0)),
                free("planet_teeth", 8.0, 20.0, 2.0, rng.pick(8.0, 20.0, 2.0)),
                free("planets", 2.0, 5.0, 1.0, 4.0),
                free("bore", 0.0, 12.0, 3.0, 9.0),
                // every sixth with no room: certainly none
                if k % 6 == 0 { free("rim", 20.0, 30.0, 10.0, 30.0) } else { free("rim", 40.0, 120.0, 10.0, o.rim) },
            ],
            "spur" => vec![
                free("module", 1.0, 2.0, 0.25, m),
                free("pressure_angle", 16.0, 30.0, 2.0, pa),
                free("height", 1.0, 5.0, 0.5, 2.0 * m),
                free("teeth", 6.0, 30.0, 1.0, rng.pick(6.0, 30.0, 1.0)),
                free("bore", 0.0, 20.0, 2.0, 10.0),
            ],
            _ => vec![
                free("module", 1.0, 2.0, 0.25, m),
                free("pressure_angle", 16.0, 30.0, 2.0, pa),
                free("ring_teeth", 20.0, 70.0, 2.0, rng.pick(20.0, 70.0, 2.0)),
                if k % 6 == 2 { free("rim", 20.0, 30.0, 10.0, 30.0) } else { free("rim", 40.0, 160.0, 10.0, o.rim) },
            ],
        };
        let out = exhaustive(&o, &fr, 1.0, None).expect("applies");
        let mut o2 = o.clone();
        for f in &fr {
            o2.set(&f.name, f.value);
        }
        let expected = brute_force(&o2, &fr);
        match (&out.values, expected) {
            (Some(v), Some(d)) => {
                let mut w = o2.clone();
                let mut dist = 0.0;
                for (f, &x) in fr.iter().zip(v) {
                    w.set(&f.name, x);
                    dist += (x - f.value).abs() / (f.max - f.min);
                }
                assert_eq!(badness(&w), 0.0, "{kind}: the values found don't work");
                assert!((dist - d).abs() < 1e-9, "{kind}: found at distance {dist}, nearest is {d}");
                found += 1;
            }
            (None, None) => none += 1,
            (a, b) => panic!("{kind}: exhaustive {a:?}, brute force {b:?}"),
        }
    }
    assert!(found > 10 && none > 0, "{found} found, {none} with none: the sample should have both");
}
