//! Rotating mesh tests.
//!
//! Gear A drives gear B through one tooth of A, in the correct ratio. B
//! starts from the centered position (spur_mesh_rotation / ring_mesh_rotation,
//! how gears are assembled) and is turned back until the play on one side is
//! gone, as when one gear drives the other: the drive-side flanks are
//! DRIVE_GAP apart (just short of touching) and the coast side has all the
//! backlash. At every step we check (see common::problems):
//!
//! - Drive side: along its line of action there is at least one pair of
//!   facing flanks, and every facing pair is DRIVE_GAP apart: the gears stay
//!   in contact, through every hand-over from one tooth pair to the next.
//! - Pressure angle: at those contacts, both flanks' normals point along the
//!   line of action, at alpha to the common tangent (within the error of
//!   measuring a curve with straight outline segments).
//! - Coast side: every facing pair along the other line of action is the
//!   whole backlash apart, backlash * cos(alpha) - DRIVE_GAP. (The backlash j
//!   is an arc on the pitch circle; along the line of action, the flanks'
//!   common normal, it is j cos(alpha).)
//! - Everywhere else: nothing that isn't on a drive-side flank comes closer
//!   to the other gear than that, and nothing overlaps.
//!
//! GearSystem promises this for every pair it accepts (see the gears module),
//! so the tests run every system at its minimum tooth counts, where it is
//! hardest to keep. (The gears are symmetric, so driving the other way is the
//! mirror image.)

mod common;

use common::*;
use gears2::gears::{GearSystem, Params};
use std::f64::consts::PI;

const STEPS: usize = 30;
const SHRINK_TOL: f64 = 1e-6;

fn named(name: &str, dev: f64) -> GearSystem {
    let mut p: Params = params(20.0, 4.0, dev, 0.01);
    match name {
        "20deg" => {}
        "25deg" => p.pressure_angle = deg(25.0),
        "14.5deg" => p.pressure_angle = deg(14.5),
        "20deg-short" => p.height = 3.0,
        "20deg-sharp" => p.fillet_radius = Some(0.0),
        "20deg-big-fillet" => p.fillet_radius = Some(2.0),
        _ => panic!("unknown system {}", name),
    }
    system(p)
}

fn centered_gap(s: &GearSystem) -> f64 {
    s.backlash * s.pressure_angle.cos() / 2.0
}

/// Spur pairs at the minimum tooth count, and its partners.
fn external(name: &str, dev: f64, partners: &[usize]) {
    let s = named(name, dev);
    let m = s.min_spur_teeth;
    for &zb in partners {
        let zb = if zb == 0 { m } else { zb };
        assert_clean(&s, &sweep_spur_pair(&s, m, zb, STEPS), &format!("{} dev {}: spur {}/{}", name, dev, m, zb));
    }
}

/// Planets in the smallest rings the guarantee covers (ring - planet =
/// 2 * min_spur_teeth), small and large.
fn internal(name: &str, dev: f64, pairs: &[(usize, usize)]) {
    let s = named(name, dev);
    let m = s.min_spur_teeth;
    for &(p, r) in pairs {
        let (zp, zr) = (p * m, if r > 20 { r } else { r * m });
        assert_clean(&s, &sweep_ring_pair(&s, zp, zr, STEPS), &format!("{} dev {}: planet {} in ring {}", name, dev, zp, zr));
    }
}

macro_rules! system_tests {
    ($($test:ident: $name:expr),* $(,)?) => {$(
        mod $test {
            use super::*;

            #[test]
            fn external_mesh() {
                let m = named($name, 0.001).min_spur_teeth;
                external($name, 0.001, &[0, m + 1, 3 * m, 150]);
            }

            #[test]
            fn internal_mesh() {
                internal($name, 0.001, &[(1, 3), (2, 4), (5, 7), (1, 150)]);
            }

            #[test]
            fn below_minimum_is_rejected() {
                let s = named($name, 0.001);
                let e = s.spur_gear(s.min_spur_teeth - 1).unwrap_err().0;
                assert!(e.contains(&format!("at least {} teeth", s.min_spur_teeth)), "{}", e);
                let e = s.ring_gear(s.min_ring_teeth - 1).unwrap_err().0;
                assert!(e.contains(&format!("at least {} teeth", s.min_ring_teeth)), "{}", e);
            }
        }
    )*};
}

system_tests! {
    deg20: "20deg",
    deg25: "25deg",
    deg14_5: "14.5deg",
    deg20_short: "20deg-short",
    deg20_sharp: "20deg-sharp",
    deg20_big_fillet: "20deg-big-fillet",
}

/// Also run coarser deviations, on a few systems.
macro_rules! coarse_tests {
    ($($test:ident: $name:expr, $dev:expr),* $(,)?) => {$(
        #[test]
        fn $test() {
            let m = named($name, $dev).min_spur_teeth;
            external($name, $dev, &[0, 150]);
            internal($name, $dev, &[(1, 3), (5, 7)]);
            let _ = m;
        }
    )*};
}

coarse_tests! {
    coarse_20deg_003: "20deg", 0.003,
    coarse_20deg_01: "20deg", 0.01,
    coarse_big_fillet_003: "20deg-big-fillet", 0.003,
    coarse_big_fillet_01: "20deg-big-fillet", 0.01,
}

#[test]
fn planetary_meshes_every_planet() {
    let s = named("20deg", 0.001);
    let (teeth_sun, teeth_planet, planets) = (21, 21, 3);
    let outlines = s.planetary_gear(teeth_sun, teeth_planet, planets, 0.0).unwrap();
    assert_eq!(outlines.len(), planets + 2);
    let (sun, ring) = (&outlines[0], &outlines[planets + 1]);
    let teeth_ring = teeth_sun + 2 * teeth_planet;
    let (r_sun, r_ring) = (s.pitch_radius(teeth_sun), s.pitch_radius(teeth_ring));
    let window = 2.0 * (s.height + s.clearance);
    for k in 0..planets {
        let planet = &outlines[1 + k];
        let direction = 2.0 * PI * k as f64 / planets as f64;
        for (other, pitch_r, is_ring) in [(sun, r_sun, false), (ring, r_ring, true)] {
            let pitch_point = (pitch_r * direction.cos(), pitch_r * direction.sin());
            let (_, _, gap, overlapping) = check_mesh(planet, other, (0.0, (0.0, 0.0)), (0.0, (0.0, 0.0)), pitch_point, window, is_ring);
            assert!(!overlapping);
            assert!(centered_gap(&s) - SHRINK_TOL <= gap && gap <= centered_gap(&s) + 2.0 * s.max_deviation, "gap {}", gap);
        }
    }
}

/// Each planet implies the same ring rotation, up to a whole ring tooth.
#[test]
fn planetary_ring_agrees_with_every_planet() {
    let (teeth_sun, teeth_planet, planets) = (24, 21, 3); // 24 + 66 = 90, divisible by 3
    let teeth_ring = teeth_sun + 2 * teeth_planet;
    let step = 2.0 * PI / teeth_ring as f64;
    let rotations: Vec<f64> = (0..planets)
        .map(|k| {
            let direction = 2.0 * PI * k as f64 / planets as f64;
            let rot_p = GearSystem::spur_mesh_rotation(teeth_sun, 0.0, teeth_planet, direction);
            GearSystem::ring_mesh_rotation(teeth_planet, rot_p, teeth_ring, direction)
        })
        .collect();
    for r in &rotations {
        let off = (r - rotations[0]).rem_euclid(step);
        assert!(off.min(step - off) < 1e-9);
    }
}

#[test]
fn planets_must_divide_evenly() {
    let e = named("20deg", 0.001).planetary_gear(21, 21, 5, 0.0).unwrap_err().0; // 21 + 63 = 84
    assert!(e.contains("spaced evenly"), "{}", e);
}

#[test]
fn planets_must_not_collide() {
    let e = named("20deg", 0.001).planetary_gear(21, 42, 6, 0.0).unwrap_err().0; // 21 + 105 = 126
    assert!(e.contains("don't fit"), "{}", e);
}
