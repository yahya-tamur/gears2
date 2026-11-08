//! Bevel pairs: the rotating mesh test on the sphere (see common::sphere),
//! the outlines, the meshes, and parameter errors.

mod common;

use common::sphere::{problems, Sweep};
use common::*;
use gears2::bevel::*;
use gears2::gears::{GearSystem, Params};
use std::f64::consts::PI;

const STEPS: usize = 30;

fn gear_set(pressure_deg: f64, dev: f64, f: impl Fn(&mut Params)) -> GearSystem {
    let mut p = params(pressure_deg, 4.0, dev, 0.01);
    p.helix_angle = deg(30.0);
    f(&mut p);
    system(p)
}

fn pair(s: &GearSystem, z1: usize, z2: usize, shaft_deg: f64) -> BevelPair {
    let r = s.module() * z1 as f64 / (2.0 * (deg(shaft_deg).sin()).atan2(z2 as f64 / z1 as f64 + deg(shaft_deg).cos()).sin());
    BevelPair::new(s, [z1, z2], deg(shaft_deg), r / 3.0, true).expect("a valid pair")
}

fn assert_meshes(s: &GearSystem, z1: usize, z2: usize, shaft_deg: f64) {
    let p = pair(s, z1, z2, shaft_deg);
    let found = problems(&p, &Sweep::new(&p).run(STEPS));
    assert!(found.is_empty(), "{}/{} at {} deg: {}", z1, z2, shaft_deg, found.iter().take(5).cloned().collect::<Vec<_>>().join("; "));
}

macro_rules! meshing {
    ($($name:ident: $pa:expr, $dev:expr, $z1:expr, $z2:expr, $shaft:expr, $tweak:expr;)*) => {$(
        #[test]
        fn $name() {
            assert_meshes(&gear_set($pa, $dev, $tweak), $z1, $z2, $shaft);
        }
    )*};
}

meshing! {
    meshes_20_30_square: 20.0, 0.001, 20, 30, 90.0, |_| {};
    meshes_equal_square: 20.0, 0.001, 24, 24, 90.0, |_| {};
    meshes_small_pinion: 20.0, 0.001, 18, 54, 90.0, |_| {};
    meshes_narrow_shaft: 20.0, 0.001, 20, 30, 50.0, |_| {};
    meshes_wide_shaft: 20.0, 0.001, 30, 40, 120.0, |_| {};
    meshes_25_degrees: 25.0, 0.001, 16, 30, 90.0, |_| {};
    meshes_14_5_degrees: 14.5, 0.001, 30, 45, 90.0, |_| {};
    meshes_sharp_roots: 20.0, 0.001, 20, 30, 90.0, |p: &mut Params| p.fillet_radius = Some(0.0);
    meshes_big_fillets: 20.0, 0.001, 20, 30, 90.0, |p: &mut Params| p.fillet_radius = Some(2.0);
    meshes_coarse: 20.0, 0.01, 20, 30, 90.0, |_| {};
    meshes_coarse_small_pinion: 20.0, 0.01, 18, 54, 90.0, |_| {};
    meshes_nearly_flat_gear: 20.0, 0.001, 21, 30, 130.0, |_| {};
}

#[test]
fn sections_turn_in_step() {
    // Every sphere shows the same mesh: gear 2's section turns by -z1/z2 of gear 1's.
    let s = gear_set(20.0, 0.01, |_| {});
    let p = pair(&s, 20, 30, 90.0);
    for k in 0..=10 {
        let rho = p.r_cone - p.face * k as f64 / 10.0;
        for hand in [1, -1] {
            let (t1, t2) = (p.twist(0, hand, rho), p.twist(1, hand, rho));
            assert!((t2 + t1 * 20.0 / 30.0).abs() < 1e-12, "rho {}: {} vs {}", rho, t1, t2);
        }
    }
    assert_eq!(p.twist(0, 1, p.r_cone), 0.0);
    assert!(p.twist(0, 1, p.r_cone - p.face).abs() < 1e-12);
}

#[test]
fn outline_is_spaced_and_ordered() {
    let s = gear_set(20.0, 0.01, |_| {});
    let p = pair(&s, 20, 30, 90.0);
    for which in 0..2 {
        let o = p.outline(which);
        let n = o.len();
        for i in 0..n {
            let (a, b) = (o[i], o[(i + 1) % n]);
            assert!(angle(a, b) >= p.ds * (1.0 - 1e-9), "gear {} points {} apart (at least {})", which + 1, angle(a, b), p.ds);
            let step = (azimuth(b) - azimuth(a)).rem_euclid(2.0 * PI);
            assert!(!(PI < step && step < 2.0 * PI - 1e-9), "gear {}: azimuth goes backwards at {}", which + 1, i);
        }
    }
}

#[test]
fn outline_follows_the_involute() {
    // Every point of the true upper flank (above the fillet) is within the
    // accuracy of the outline's arcs, and on the air side of none of them.
    let s = gear_set(20.0, 0.01, |_| {});
    let p = pair(&s, 20, 30, 90.0);
    for which in 0..2 {
        let g = &p.gears[which];
        let o = p.outline(which);
        let n = o.len();
        for k in 0..=200 {
            let sv = g.s_p + (g.s_tip - g.s_p) * k as f64 / 200.0;
            let q = g.flank.at(sv);
            let d = (0..n).map(|i| common::sphere::arc_distance(q, o[i], o[(i + 1) % n])).fold(f64::INFINITY, f64::min);
            assert!(d <= p.dev * (1.0 + 1e-6), "gear {}: flank point {} off the outline by {} (accuracy {})", which + 1, k, d, p.dev);
        }
    }
}

fn mesh_ok(m: &gears2::mesh::IndexedMesh, holes: i64) {
    let found = m.problems();
    assert!(found.is_empty(), "{:?}", found.iter().take(3).collect::<Vec<_>>());
    assert_eq!(m.euler_characteristic(), 2 - 2 * holes);
    assert!(m.volume() > 0.0);
}

#[test]
fn meshes_are_closed() {
    for helix in [0.0, 30.0] {
        let s = gear_set(20.0, 0.01, |p: &mut Params| p.helix_angle = deg(helix));
        for (bores, holes) in [([0.0, 0.0], 0), ([5.0, 8.0], 1)] {
            for hand in [1, -1] {
                for assembled in [true, false] {
                    let parts = bevel_pair_mesh(&s, [20, 30], PI / 2.0, 12.0, bores, hand, true, assembled).unwrap();
                    for m in &parts {
                        mesh_ok(m, holes);
                    }
                }
            }
        }
    }
}

#[test]
fn nearly_flat_gear_meshes_are_closed() {
    // Gear 2's tip cone is 89.5 degrees: its tooth ends face almost along the axis.
    let s = gear_set(20.0, 0.01, |_| {});
    for shaft in [120.0, 130.0] {
        let parts = bevel_pair_mesh(&s, [21, 30], deg(shaft), 12.0, [10.0, 8.0], 1, true, true).unwrap();
        for m in &parts {
            mesh_ok(m, 1);
            assert!(m.triangles.len() < 60_000, "{} triangles", m.triangles.len());
        }
    }
}

#[test]
fn assembled_pair_sits_on_gear_1s_back_face() {
    let s = gear_set(20.0, 0.01, |_| {});
    for shaft in [60.0, 90.0, 120.0] {
        let parts = bevel_pair_mesh(&s, [21, 30], deg(shaft), 12.0, [10.0, 8.0], 1, true, true).unwrap();
        let g1 = &parts[0];
        let lowest = g1.vertices.iter().map(|v| v[2]).fold(f64::INFINITY, f64::min);
        assert!(lowest.abs() < 1e-9, "shaft {}: gear 1's back face at z = {}", shaft, lowest);
        let (sx, sy) = g1.vertices.iter().fold((0.0, 0.0), |a, v| (a.0 + v[0], a.1 + v[1]));
        let n = g1.vertices.len() as f64;
        assert!((sx / n).abs() < 0.5 && (sy / n).abs() < 0.5, "gear 1 not on the z axis");
    }
}

#[test]
fn print_layout_stands_on_the_heel() {
    let s = gear_set(20.0, 0.01, |_| {});
    let parts = bevel_pair_mesh(&s, [20, 30], PI / 2.0, 12.0, [5.0, 8.0], 1, true, false).unwrap();
    for m in &parts {
        let lowest = m.vertices.iter().map(|v| v[2]).fold(f64::INFINITY, f64::min);
        assert!(lowest.abs() < 1e-9);
        // the whole back face is flat on z = 0 out to the fold circle
        assert!(m.vertices.iter().filter(|v| v[2].abs() < 1e-9).count() > 20);
    }
}

#[test]
fn tooth_ends_stay_near_the_spheres() {
    // Every end vertex outside the flat face is on its sphere, and every end
    // triangle there strays from the sphere by at most the accuracy.
    let s = gear_set(20.0, 0.01, |_| {});
    let p = pair(&s, 20, 30, 90.0);
    let m = bevel_gear_mesh(&p, 0, 5.0, 1).unwrap();
    let (r, toe) = (p.r_cone, p.r_cone - p.face);
    let on = |v: [f64; 3], rho: f64| (norm(v) - rho).abs() < 1e-9;
    let mut checked = 0;
    for t in &m.triangles {
        let v: Vec<[f64; 3]> = t.iter().map(|&i| m.vertices[i]).collect();
        for rho in [r, toe] {
            if v.iter().all(|&x| on(x, rho)) {
                let c = scale(add(add(v[0], v[1]), v[2]), 1.0 / 3.0);
                assert!(rho - norm(c) <= s.max_deviation, "end triangle {} off its sphere", rho - norm(c));
                checked += 1;
            }
        }
    }
    assert!(checked > 100);
}

#[test]
fn errors_name_the_problem() {
    let s = gear_set(20.0, 0.01, |_| {});
    let e = BevelPair::new(&s, [8, 60], deg(150.0), 5.0, true).unwrap_err().0;
    assert!(e.contains("flat disc"), "{}", e);
    let e = BevelPair::new(&s, [20, 30], deg(90.0), 40.0, true).unwrap_err().0;
    assert!(e.contains("shafts meet"), "{}", e);
    let e = BevelPair::new(&s, [20, 30], deg(180.0), 10.0, true).unwrap_err().0;
    assert!(e.contains("shaft angle"), "{}", e);
    let e = BevelPair::new(&s, [5, 30], deg(90.0), 5.0, true).unwrap_err().0;
    assert!(e.contains("Gear 1") || e.contains("gear 1"), "{}", e);
    let e = BevelPair::new(&s, [14, 42], deg(90.0), 10.0, true).unwrap_err().0;
    assert!(e.contains("too deep"), "{}", e);
    let e = bevel_pair_mesh(&s, [20, 30], PI / 2.0, 12.0, [30.0, 8.0], 1, true, true).unwrap_err().0;
    assert!(e.contains("bore"), "{}", e);
}

#[test]
fn backlash_at_the_toe_is_larger_at_the_heel() {
    let s = gear_set(20.0, 0.01, |_| {});
    let (at_toe, at_heel) = (BevelPair::new(&s, [20, 30], PI / 2.0, 12.0, true).unwrap(), BevelPair::new(&s, [20, 30], PI / 2.0, 12.0, false).unwrap());
    assert!((at_heel.backlash - 0.2).abs() < 1e-12);
    assert!((at_toe.backlash - 0.2 * at_toe.r_cone / (at_toe.r_cone - 12.0)).abs() < 1e-12);
}

#[test]
fn the_sweep_catches_a_misplaced_gear() {
    // Turning gear 2 by a whole backlash out of its place must show up.
    let s = gear_set(20.0, 0.001, |_| {});
    let p = pair(&s, 20, 30, 90.0);
    let mut sweep = Sweep::new(&p);
    let turn = p.backlash / (p.r_cone * p.gears[1].delta.sin());
    sweep.local[1] = sweep.local[1].iter().map(|&v| rot_z(v, turn)).collect();
    let found = problems(&p, &sweep.run(STEPS));
    assert!(found.iter().any(|m| m.contains("overlap") || m.contains("gap")), "{:?}", found.first());
}

/// Not a check: writes the rotating test's frames as JSON for drawing
/// (cargo test --release --test bevel -- --ignored, with BEVEL_FRAMES=dir).
#[test]
#[ignore]
fn write_sweep_frames() {
    let Ok(dir) = std::env::var("BEVEL_FRAMES") else { return };
    let s = gear_set(20.0, 0.001, |_| {});
    for (z1, z2, shaft) in [(20, 30, 90.0), (30, 40, 120.0)] {
        let p = pair(&s, z1, z2, shaft);
        let sweep = Sweep::new(&p);
        let steps = sweep.run(60);
        let pitch = sph(p.gears[0].delta, 0.0);
        let e = [p.gears[0].delta.cos(), 0.0, p.gears[0].delta.sin()];
        // gnomonic view centered on the pitch point, in mm at the heel
        let view = |x: V| -> Option<(f64, f64)> {
            if angle(x, pitch) > 1.6 * sweep.window {
                return None;
            }
            let q = scale(x, 1.0 / dot(x, pitch));
            Some((q[1] * p.r_cone, dot(q, e) * p.r_cone))
        };
        let poly = |pts: &[V]| -> String {
            let mut runs: Vec<Vec<String>> = vec![vec![]];
            for &x in pts.iter().chain(pts.iter().take(1)) {
                match view(x) {
                    Some((a, b)) => runs.last_mut().unwrap().push(format!("[{:.5},{:.5}]", a, b)),
                    None => {
                        if !runs.last().unwrap().is_empty() {
                            runs.push(vec![]);
                        }
                    }
                }
            }
            format!("[{}]", runs.iter().filter(|r| r.len() > 1).map(|r| format!("[{}]", r.join(","))).collect::<Vec<_>>().join(","))
        };
        let mut frames = Vec::new();
        for st in &steps {
            let contacts: Vec<String> = st.contacts.iter().filter_map(|&x| view(x)).map(|(a, b)| format!("[{:.5},{:.5}]", a, b)).collect();
            let drive = st.drive.iter().map(|q| q.gap).fold(f64::NAN, f64::min);
            let coast = st.coast.iter().map(|q| q.gap).fold(f64::NAN, f64::min);
            frames.push(format!(
                "{{\"a\":{},\"b\":{},\"contacts\":[{}],\"drive\":{:.6},\"coast\":{:.6},\"elsewhere\":{:.6},\"overlap\":{}}}",
                poly(&st.outlines[0]),
                poly(&st.outlines[1]),
                contacts.join(","),
                if drive.is_nan() { -1.0 } else { drive },
                if coast.is_nan() { -1.0 } else { coast },
                st.elsewhere,
                st.overlap
            ));
        }
        let json = format!(
            "{{\"teeth\":[{},{}],\"shaft\":{},\"alpha\":{},\"window\":{},\"backlash\":{},\"problems\":{},\"frames\":[{}]}}",
            z1,
            z2,
            shaft,
            p.pressure_angle,
            sweep.window * p.r_cone,
            p.backlash,
            problems(&p, &steps).len(),
            frames.join(",")
        );
        std::fs::write(format!("{}/bevel_sweep_{}_{}_{}.json", dir, z1, z2, shaft as i32), json).unwrap();
    }
}
