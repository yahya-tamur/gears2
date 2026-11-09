//! The page's fast check (badness) agrees with actually building.

use gears2::web::{badness, build_parts, Options};

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

#[test]
fn badness_is_zero_exactly_when_the_build_works() {
    let mut rng = Lcg(7);
    let (mut agree, mut valid) = (0, 0);
    for k in 0..240 {
        let kind = ["spur", "ring", "planetary", "bevel"][k % 4];
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
        let b = badness(&o);
        let built = build_parts(&o);
        assert_eq!(b == 0.0, built.is_ok(), "{} options: badness {} but build {:?}", kind, b, built.as_ref().err());
        agree += 1;
        valid += built.is_ok() as usize;
    }
    assert!(valid > 20 && valid < agree - 20, "{} of {} valid: the sample should have both", valid, agree);
}
