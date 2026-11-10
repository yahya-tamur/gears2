//! The page's fast check (badness) agrees with actually building.

use gears2::web::{badness, build_parts, limits, tooth_fit_limits, Options};

fn options(kind: &str) -> Options {
    Options {
        kind: kind.into(),
        module: 2.0,
        pressure_angle: 20.0,
        helix_angle: 30.0,
        height: None,
        backlash: 0.2,
        clearance: None,
        fillet_radius: None,
        fillet_deviation: Some(0.1),
        max_deviation: 0.05,
        ds: 0.1,
        hand: 1,
        end_bottom: None,
        end_top: None,
        width: 10.0,
        bore: 8.0,
        planet_bore: 6.0,
        rim: 140.0,
        teeth: 21.0,
        ring_teeth: 63.0,
        sun_teeth: 21.0,
        planet_teeth: 21.0,
        planets: 4.0,
        teeth2: 30.0,
        shaft_angle: 90.0,
        face_width: 12.0,
        bore2: 8.0,
        backlash_inner: Some(0.2),
        backlash_outer: None,
        layout: None,
    }
}

/// A small deterministic random generator.
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

fn random_options(rng: &mut Lcg, kind: &str) -> Options {
    let mut o = options(kind);
    o.module = rng.pick(1.0, 3.0, 0.25);
    o.pressure_angle = rng.pick(15.0, 32.0, 0.5);
    o.height = if rng.next() < 0.5 { None } else { Some(rng.pick(1.5, 6.0, 0.1)) };
    o.backlash = rng.pick(0.0, 1.0, 0.05);
    o.backlash_inner = Some(o.backlash);
    o.teeth = rng.pick(8.0, 40.0, 1.0);
    o.ring_teeth = rng.pick(24.0, 90.0, 1.0);
    o.sun_teeth = rng.pick(8.0, 30.0, 1.0);
    o.planet_teeth = rng.pick(8.0, 30.0, 1.0);
    o.planets = rng.pick(2.0, 6.0, 1.0);
    o.teeth2 = rng.pick(8.0, 40.0, 1.0);
    o.shaft_angle = rng.pick(40.0, 140.0, 5.0);
    o.face_width = rng.pick(4.0, 20.0, 1.0);
    o.bore = rng.pick(0.0, 20.0, 1.0);
    o.bore2 = rng.pick(0.0, 20.0, 1.0);
    o.planet_bore = rng.pick(0.0, 12.0, 1.0);
    o.rim = rng.pick(40.0, 200.0, 5.0);
    o
}

#[test]
fn badness_is_zero_exactly_when_the_build_works() {
    let mut rng = Lcg(7);
    let (mut agree, mut valid) = (0, 0);
    for k in 0..240 {
        let kind = ["spur", "ring", "planetary", "bevel"][k % 4];
        let o = random_options(&mut rng, kind);
        let b = badness(&o);
        let built = build_parts(&o);
        assert_eq!(b == 0.0, built.is_ok(), "{} options: badness {} but build {:?}", kind, b, built.as_ref().err());
        agree += 1;
        valid += built.is_ok() as usize;
    }
    assert!(valid > 20 && valid < agree - 20, "{} of {} valid: the sample should have both", valid, agree);
}

/// Each limit is exact for its own check: at the limit the value passes it,
/// a little beyond it fails it, and nothing else about the score changes.
#[test]
fn limits_are_where_their_checks_start_failing() {
    let mut rng = Lcg(11);
    let mut checked = 0;
    for k in 0..400 {
        let kind = ["spur", "ring", "planetary", "bevel"][k % 4];
        let mut o = random_options(&mut rng, kind);
        // tooth counts at their minimums, so the gear-size limits apply
        for (name, lo, _) in limits(&o) {
            if let Some(lo) = lo {
                match name {
                    "teeth" => o.teeth = o.teeth.max(lo),
                    "ring_teeth" => o.ring_teeth = o.ring_teeth.max(lo),
                    "sun_teeth" => o.sun_teeth = o.sun_teeth.max(lo),
                    _ => {}
                }
            }
        }
        for (name, lo, _) in limits(&o) {
            if let ("planet_teeth", Some(lo)) = (name, lo) {
                o.planet_teeth = o.planet_teeth.max(lo);
            }
        }
        for (name, lo, hi) in limits(&o) {
            let set = |o: &mut Options, v: f64| match name {
                "bore" => o.bore = v,
                "bore2" => o.bore2 = v,
                "planet_bore" => o.planet_bore = v,
                "rim" => o.rim = v,
                _ => {}
            };
            let (at, beyond, fine) = match (name, lo, hi) {
                ("bore" | "bore2" | "planet_bore", _, Some(hi)) if hi > 0.1 => (hi - 1e-9, hi + 0.1, 0.0),
                ("rim", Some(lo), _) => (lo + 1e-9, lo - 0.1, 1e4),
                _ => continue,
            };
            let mut a = o.clone();
            set(&mut a, at);
            let mut b = o.clone();
            set(&mut b, beyond);
            let mut c = o.clone();
            set(&mut c, fine);
            let (ba, bb, bc) = (badness(&a), badness(&b), badness(&c));
            if bc >= 1000.0 {
                continue;
            }
            assert!((ba - bc).abs() < 1e-9, "{kind} {name} at its limit {at}: {ba}, well inside: {bc}");
            assert!(bb > bc, "{kind} {name} past its limit ({beyond}): {bb}, inside: {bc}");
            checked += 1;
        }
        // tooth counts: a spur gear without a bore works at its minimum, not below
        if kind == "spur" {
            if let Some((_, Some(lo), _)) = limits(&o).into_iter().find(|l| l.0 == "teeth") {
                let mut a = o.clone();
                a.bore = 0.0;
                a.teeth = lo;
                if badness(&a) < 1000.0 {
                    assert_eq!(badness(&a), 0.0, "spur at its minimum teeth {lo}");
                    a.teeth = lo - 1.0;
                    if lo - 1.0 >= 3.0 {
                        assert!(badness(&a) > 0.0, "spur below its minimum teeth {lo}");
                    }
                    checked += 1;
                }
            }
        }
    }
    assert!(checked > 150, "only {checked} limits checked");
}

/// tooth_fit_limits: at the tooth height (or pressure angle) it finds, the
/// tooth counts reach the gear set's minimums; one step short of it, toward
/// the current value, they don't.
#[test]
fn tooth_height_and_pressure_limits_are_where_the_tooth_counts_start_fitting() {
    let mut rng = Lcg(5);
    let (mut checked, mut sharp) = (0, 0);
    let short = |o: &Options| {
        // badness of the gear set and tooth counts alone: no bores, wide ring
        let mut o = o.clone();
        o.bore = 0.0;
        o.planet_bore = 0.0;
        o.rim = 1e4;
        o.planets = 1.0;
        badness(&o)
    };
    for k in 0..300 {
        let kind = ["spur", "ring", "planetary"][k % 3];
        let mut o = random_options(&mut rng, kind);
        o.height = Some(rng.pick(1.0, 8.0, 0.1));
        o.teeth = rng.pick(5.0, 16.0, 1.0);
        o.sun_teeth = rng.pick(5.0, 14.0, 1.0);
        o.planet_teeth = rng.pick(5.0, 14.0, 1.0);
        o.ring_teeth = rng.pick(15.0, 40.0, 1.0);
        for (name, lo, hi) in tooth_fit_limits(&o, Some([0.5, 10.0, 0.05]), Some([5.0, 45.0, 0.1])) {
            let cur = if name == "height" { o.height.unwrap() } else { o.pressure_angle };
            let edge = lo.or(hi).unwrap();
            let before = edge - (if name == "height" { 0.05 } else { 0.1 }) * (edge - cur).signum();
            let with = |v: f64| {
                let mut a = o.clone();
                if name == "height" { a.height = Some(v) } else { a.pressure_angle = v }
                a
            };
            assert!(short(&with(edge)) < 1000.0 && short(&with(edge)) <= short(&o), "{kind} {name} at {edge}: {}", short(&with(edge)));
            checked += 1;
            sharp += (short(&with(before)) > 0.0 || short(&with(edge)) > 0.0) as usize;
        }
    }
    assert!(checked > 40, "only {checked} limits checked");
    assert_eq!(sharp, checked, "{sharp} of {checked} limits are edges");
}

/// GearSystem::teeth_fit agrees with the minimums new_quick() finds.
#[test]
fn teeth_fit_agrees_with_the_minimums() {
    use gears2::gears::{GearSystem, Params};
    let mut rng = Lcg(3);
    let mut compared = 0;
    for _ in 0..400 {
        let m = rng.pick(0.5, 3.0, 0.25);
        let mut p = Params::new(m * std::f64::consts::PI, rng.pick(12.0, 40.0, 1.0).to_radians(), rng.pick(0.8, 3.0, 0.1) * m,
            rng.pick(0.0, 0.8, 0.05), 0.01, 0.05);
        p.helix_angle = rng.pick(0.0, 45.0, 5.0).to_radians();
        p.clearance = if rng.next() < 0.5 { None } else { Some(rng.pick(0.0, 1.0, 0.1)) };
        let full = GearSystem::new_quick(&p, 400);
        for _ in 0..5 {
            let spur = rng.pick(3.0, 40.0, 1.0) as usize;
            let ring = rng.pick(9.0, 120.0, 1.0) as usize;
            let quick = GearSystem::teeth_fit(&p, spur, Some(ring), 400);
            let slow = full.as_ref().is_ok_and(|s| spur >= s.min_spur_teeth && ring >= s.min_ring_teeth);
            assert_eq!(quick, slow, "{p:?}: spur {spur}, ring {ring}");
            assert_eq!(GearSystem::teeth_fit(&p, spur, None, 400), full.as_ref().is_ok_and(|s| spur >= s.min_spur_teeth), "{p:?}: spur {spur}");
            compared += 1;
        }
    }
    assert!(compared == 2000);
}
