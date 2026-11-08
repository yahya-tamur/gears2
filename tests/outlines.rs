//! 2D outline tests: point spacing, shape, fillets, minimum tooth counts and
//! error messages.

mod common;

use common::*;
use gears2::gears::{arc_points, arc_steps, GearSystem, Params, P, MIN_LAND};
use std::f64::consts::PI;

fn base() -> Params {
    params(20.0, 4.0, 0.01, 0.05)
}

fn with(f: impl FnOnce(&mut Params)) -> GearSystem {
    let mut p = base();
    f(&mut p);
    system(p)
}

fn outline(s: &GearSystem, kind: &str, teeth: usize) -> Vec<P> {
    if kind == "spur" {
        s.spur_gear(teeth).unwrap()
    } else {
        s.ring_gear(teeth).unwrap()
    }
}

fn error_of(p: Params) -> String {
    GearSystem::new(&p).expect_err("should be rejected").0
}

// ---------------------------------------------------------------- arc points

#[test]
fn arc_steps_limits() {
    // Sagitta R (1 - cos(t / 2)) <= max_deviation sets the fewest steps...
    let k = arc_steps(10.0, PI, 0.01, 1e-6) as f64;
    assert!(10.0 * (1.0 - (PI / k / 2.0).cos()) <= 0.01 && 0.01 < 10.0 * (1.0 - (PI / (k - 1.0) / 2.0).cos()));
    // ...and chords of at least ds cap them.
    let k = arc_steps(10.0, PI, 1e-9, 1.0) as f64;
    assert!(2.0 * 10.0 * (PI / k / 2.0).sin() >= 1.0 && 1.0 > 2.0 * 10.0 * (PI / (k + 1.0) / 2.0).sin());
    assert_eq!(arc_steps(10.0, 0.0, 0.01, 0.1), 1);
}

#[test]
fn arc_points_are_even_short_way_and_within_tolerance() {
    for sweep in [0.3, -0.3, 1.5, -1.5, 3.0] {
        let (center, radius, dev, ds) = ((1.0, -2.0), 0.7, 0.001, 0.01);
        let start = (center.0 + radius, center.1);
        let end = (center.0 + radius * f64::cos(sweep), center.1 + radius * f64::sin(sweep));
        let points = arc_points(center, start, end, dev, ds, false);
        assert_eq!(*points.last().unwrap(), end); // exactly the end point
        assert_eq!(points.len(), arc_steps(radius, sweep, dev, ds)); // start left out
        for p in &points {
            assert_close((p.0 - center.0).hypot(p.1 - center.1), radius, 1e-12, "on the circle");
        }
        let mut chain = vec![start];
        chain.extend(&points);
        let lengths: Vec<f64> = chain.windows(2).map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1)).collect();
        assert!(max_of(lengths.iter().copied()) - min_of(lengths.iter().copied()) < 1e-12); // evenly spaced
        assert!(min_of(lengths.iter().copied()) >= ds);
        assert!(radius * (1.0 - (sweep.abs() / points.len() as f64 / 2.0).cos()) <= dev); // sagitta
        // goes the short way, in the direction of the sweep
        let angles: Vec<f64> = chain.iter().map(|p| (p.1 - center.1).atan2(p.0 - center.0)).collect();
        assert!(angles.windows(2).all(|w| (w[1] - w[0]) * sweep > 0.0));
    }
}

// ------------------------------------------------------------------ outlines

/// None = default; 1.5 and 3 are too big for these gaps.
const FILLETS: [Option<f64>; 5] = [Some(0.0), Some(0.3), None, Some(1.5), Some(3.0)];

fn check_outline(s: &GearSystem, points: &[P], label: &str) {
    let shortest = min_of(segment_lengths(points));
    assert!(shortest >= s.ds * (1.0 - 1e-9), "{}: points {} apart (ds {})", label, shortest, s.ds);
    assert_eq!(backward_steps(points), 0, "{}: not star-shaped", label);
}

#[test]
fn spur_outline_spacing_and_shape() {
    for fillet in FILLETS {
        let s = with(|p| p.fillet_radius = fillet);
        for teeth in [21, 22, 24, 30, 45, 100] {
            check_outline(&s, &s.spur_gear(teeth).unwrap(), &format!("spur {} fillet {:?}", teeth, fillet));
        }
    }
}

#[test]
fn ring_outline_spacing_and_shape() {
    for fillet in FILLETS {
        let s = with(|p| p.fillet_radius = fillet);
        for teeth in [63, 64, 80, 150] {
            check_outline(&s, &s.ring_gear(teeth).unwrap(), &format!("ring {} fillet {:?}", teeth, fillet));
        }
    }
}

const CASES: [(&str, usize, Option<f64>); 5] =
    [("spur", 21, None), ("spur", 21, Some(0.0)), ("spur", 60, None), ("ring", 63, None), ("ring", 63, Some(0.0))];

/// Every point of a very finely sampled outline (on the true curves) is
/// within max_deviation of the outline, and within fillet_deviation on the
/// fillets.
#[test]
fn outline_deviation() {
    for dev in [0.01, 0.002] {
        for (kind, teeth, fillet) in CASES {
            let s = with(|p| {
                p.max_deviation = dev;
                p.ds = dev;
                p.fillet_radius = fillet;
            });
            let fine = with(|p| {
                p.max_deviation = 1e-5;
                p.ds = 1e-4;
                p.fillet_radius = fillet;
                p.fillet_deviation = Some(1e-5);
            });
            let coarse = outline(&s, kind, teeth);
            let reference = outline(&fine, kind, teeth);
            let d = distances(&reference, &coarse);
            // Fillet points are those strictly between the root circle and the
            // fillet's top P (inside for a spur gear, outside for a ring).
            let g = if kind == "spur" { fine.spur_profile(teeth) } else { fine.ring_profile(teeth) }.unwrap();
            let (r1, r2) = (g.r_root, g.p.0.hypot(g.p.1));
            let (lo, hi) = (r1.min(r2), r1.max(r2));
            for (p, d) in reference.iter().zip(d) {
                let r = p.0.hypot(p.1);
                let on_fillet = g.rho > 0.0 && r > lo + 1e-9 && r < hi - 1e-9;
                let allowed = if on_fillet { s.fillet_deviation } else { dev };
                assert!(d <= allowed, "{} {} fillet {:?} dev {}: {} from the outline", kind, teeth, fillet, dev, d);
            }
        }
    }
}

/// A looser fillet tolerance takes points off the fillets only.
#[test]
fn fillet_deviation_sets_fillet_points() {
    let tight = with(|p| p.fillet_deviation = Some(0.01));
    let loose = with(|p| p.fillet_deviation = Some(0.05));
    let sharp = with(|p| p.fillet_radius = Some(0.0));
    let per_tooth = |s: &GearSystem| s.spur_gear(21).unwrap().len() / 21;
    assert!(per_tooth(&sharp) < per_tooth(&loose) && per_tooth(&loose) < per_tooth(&tight));
    assert_eq!(loose.fillet_deviation, 0.05);
    assert_eq!(with(|_| ()).fillet_deviation, 0.05);
    let coarse = with(|p| {
        p.max_deviation = 0.1;
        p.ds = 0.1;
    });
    assert_eq!(coarse.fillet_deviation, 0.1); // never tighter
}

/// The finely sampled reference outlines are this close to the true curves.
const REFERENCE_DEV: f64 = 1e-5;

/// The outline never extends past the true shape: every point of the true
/// outline is on the outline or on its air side. (A spur gear's material is
/// inside its outline, a ring gear's outside.) Points within the reference's
/// own sampling error of the outline count as on it.
#[test]
fn outline_stays_in_material() {
    for dev in [0.01, 0.002] {
        for (kind, teeth, fillet) in [("spur", 21, None), ("spur", 21, Some(0.0)), ("ring", 63, None), ("ring", 63, Some(0.0))] {
            let s = with(|p| {
                p.max_deviation = dev;
                p.ds = dev;
                p.fillet_radius = fillet;
            });
            let fine = with(|p| {
                p.max_deviation = REFERENCE_DEV;
                p.ds = 1e-4;
                p.fillet_radius = fillet;
            });
            let out = outline(&s, kind, teeth);
            let truth = outline(&fine, kind, teeth);
            let in_outline = inside(&truth, &out);
            let d = distances(&truth, &out);
            for i in 0..truth.len() {
                let on_material_side = if kind == "spur" { in_outline[i] } else { !in_outline[i] };
                assert!(!(on_material_side && d[i] > 2.0 * REFERENCE_DEV), "{} {} {:?} {}: point {:?} is past the outline", kind, teeth, fillet, dev, truth[i]);
            }
        }
    }
}

#[test]
fn circle() {
    let s = with(|_| ());
    let reference = with(|p| {
        p.max_deviation = REFERENCE_DEV;
        p.ds = 1e-4;
    })
    .circle(5.0, false)
    .unwrap();
    for hole in [false, true] {
        let points = s.circle(5.0, hole).unwrap();
        assert!(max_deviation(&reference, &points) <= s.max_deviation);
        assert!(min_of(segment_lengths(&points)) >= s.ds);
    }
    // A rim's polygon stays inside the circle; a hole's contains it.
    let rim = s.circle(5.0, false).unwrap();
    assert!(rim.iter().all(|p| p.0.hypot(p.1) <= 5.0 + 1e-12));
    let hole = s.circle(5.0, true).unwrap();
    let ins = inside(&reference, &hole);
    let d = distances(&reference, &hole);
    assert!((0..reference.len()).all(|i| ins[i] || d[i] <= 2.0 * REFERENCE_DEV));
}

#[test]
fn outlines_at_minimum_tooth_counts() {
    for degrees in [14.5, 20.0, 25.0, 30.0, 45.0] {
        for height in [3.0, 4.0, 5.0, 6.0] {
            let mut p = base();
            p.pressure_angle = deg(degrees);
            p.height = height;
            match GearSystem::new(&p) {
                // Only for a reason that rules out every tooth count.
                Err(e) => assert!(e.0.contains("can't make") || e.0.contains("too short"), "{}", e.0),
                Ok(s) => {
                    let label = format!("{} deg, height {}", degrees, height);
                    check_outline(&s, &s.spur_gear(s.min_spur_teeth).unwrap(), &label);
                    check_outline(&s, &s.ring_gear(s.min_ring_teeth).unwrap(), &label);
                }
            }
        }
    }
}

// ------------------------------------------------------------------- fillets

#[test]
fn default_fillet_comes_from_clearance() {
    let s = with(|_| ());
    assert_close(s.clearance, 0.25 * s.module(), 1e-12, "clearance");
    assert_close(s.fillet_radius, s.clearance / (1.0 - 0.3420201433), 1e-9, "fillet radius");
}

#[test]
fn oversized_fillet_is_reduced() {
    let s = with(|p| p.fillet_radius = Some(3.0));
    for g in [s.spur_profile(24).unwrap(), s.ring_profile(72).unwrap()] {
        assert!(g.rho < 3.0);
        // The fillet never reaches where a mating tooth touches.
        let side = if g.r_tip > g.r_root { 1.0 } else { -1.0 };
        assert!(side * (g.p.0.hypot(g.p.1) - g.r_contact) <= 1e-9);
    }
}

#[test]
fn fitting_fillet_is_kept() {
    let s = with(|p| p.fillet_radius = Some(0.3));
    let g = s.spur_profile(24).unwrap();
    assert!(g.rho == 0.3 && !g.full_round);
}

#[test]
fn full_round_root() {
    // Short, wide gaps: the requested fillet meets in the middle of the gap
    // before it reaches the contact zone.
    let s = with(|p| {
        p.height = 3.0;
        p.clearance = Some(1.2);
        p.fillet_radius = Some(3.0);
    });
    let g = s.spur_profile(s.min_spur_teeth).unwrap();
    assert!(g.full_round);
    check_outline(&s, &s.spur_gear(s.min_spur_teeth).unwrap(), "full round");
}

// ------------------------------------------------------------------ minimums

#[test]
fn minimums_for_standard_system() {
    let s = with(|_| ()); // 20 degrees, height = 2 * module
    // A rack needs 17.1 teeth; a planet in a ring needs 6/5 of that.
    assert_eq!(s.min_spur_teeth, 21);
    assert_eq!(s.min_ring_teeth, 63);
}

#[test]
fn below_minimum_is_an_error() {
    let s = with(|_| ());
    assert!(s.spur_gear(16).unwrap_err().0.contains("at least 21 teeth"));
    assert!(s.ring_gear(48).unwrap_err().0.contains("at least 63 teeth"));
}

// -------------------------------------------------------------------- errors

#[test]
fn system_that_cant_make_gears() {
    let mut p = base();
    p.height = 12.0; // even a rack's teeth would be pointed
    assert!(error_of(p).contains("can't make spur gears"));
}

#[test]
fn teeth_too_short_to_stay_in_contact() {
    let mut p = base();
    p.height = 1.5;
    assert!(error_of(p).contains("too short"));
}

#[test]
fn too_much_backlash_is_an_error() {
    let mut p = base();
    p.backlash = 4.0;
    assert!(error_of(p).contains("backlash"));
}

#[test]
fn tiny_fillet_is_an_error() {
    let mut p = base();
    p.fillet_radius = Some(0.01);
    assert!(error_of(p).contains("too small"));
}

#[test]
fn min_land_follows_ds() {
    let s = with(|p| p.ds = 0.05);
    assert_eq!(s.min_land, MIN_LAND.max(0.1));
}

/// A point spacing too large for the tooth tips is reported as such, with
/// the largest spacing that works (which then does work).
#[test]
fn too_large_point_spacing_is_named() {
    let mut p = base();
    p.ds = 1.05;
    let e = error_of(p);
    assert!(e.contains("smallest point spacing") && e.contains("Use at most"), "{}", e);
    let most: f64 = e.rsplit("at most ").next().unwrap().trim_end_matches(" mm.").parse().unwrap();
    assert!(most > 0.05 && most < 1.05);
    p.ds = most;
    GearSystem::new(&p).unwrap();
}
