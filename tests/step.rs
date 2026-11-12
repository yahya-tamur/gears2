//! STEP files: the exact outline pieces, the solids built from them, and
//! the file text.
//!
//! The solids are checked for a closed, consistently oriented shell, and
//! every B-spline curve and surface against the exact shape: turned back by
//! the twist at its height, it must lie on the gear's outline (for a bevel
//! gear, turned back by the twist at its cone distance and scaled to the
//! unit sphere, on the outline there). With
//! GEARS2_OCC_PYTHON set to a Python that has OCP (pip install
//! cadquery-ocp), the last test also has OpenCascade read the files (see
//! tools/check_step.py), check that every solid is valid and closed, and
//! compare a volume with the exact one.

use gears2::bevel::{azimuth, bevel_pair_layout, rot_z, BevelPair};
use gears2::bevel_step::{bevel_gear_solid, bevel_pair_assembly};
use gears2::gear_step::{planetary_assembly, planetary_solids, ring_gear_solid, spur_gear_solid, TOLERANCE};
use gears2::gears::{GearSystem, Params, Piece};
use gears2::mesh::EndZones;
use gears2::step::{norm, scale, step_file, sub, unit, Curve, Solid, Surface, V3};
use std::f64::consts::PI;

/// Module 2, the given helix and pressure angles (the default 20 degrees
/// needs at least 21 teeth; 30 degrees allows small gears).
fn system_with(helix: f64, pressure: f64) -> GearSystem {
    let mut p = Params::new(2.0 * PI, pressure.to_radians(), 4.0, 0.2, 0.01, 0.05);
    p.helix_angle = helix.to_radians();
    GearSystem::new(&p).unwrap()
}

fn system(helix: f64) -> GearSystem {
    system_with(helix, 20.0)
}

/// The 45 degree chamfer length the page uses by default.
fn chamfer(s: &GearSystem) -> f64 {
    s.height + s.clearance + s.module() / 4.0
}

/// Distance from q to the outline of `teeth` teeth: over the pieces of the
/// nearest teeth, the nearest of 24 samples per piece, refined by Newton's
/// method on the piece's parameter.
fn distance_to_outline(pieces: &[Piece], teeth: usize, q: (f64, f64)) -> f64 {
    let per = pieces.len() / teeth;
    let tooth = (q.1.atan2(q.0) / (2.0 * PI / teeth as f64)).round() as i64;
    let near: Vec<&Piece> = (tooth - 1..=tooth + 1)
        .flat_map(|t| {
            let t = t.rem_euclid(teeth as i64) as usize;
            pieces[t * per..(t + 1) * per].iter()
        })
        .collect();
    let mut best = f64::INFINITY;
    for p in near {
        let d2 = |t: f64| {
            let (a, _) = p.at(t);
            (a.0 - q.0).powi(2) + (a.1 - q.1).powi(2)
        };
        let mut t = (0..=24).map(|i| i as f64 / 24.0).min_by(|&a, &b| d2(a).total_cmp(&d2(b))).unwrap();
        for _ in 0..12 {
            // minimize |p(t) - q|^2: step by -(p - q).p' / |p'|^2
            let ((x, y), (dx, dy)) = p.at(t);
            let den = dx * dx + dy * dy;
            if den == 0.0 {
                break;
            }
            t = (t - ((x - q.0) * dx + (y - q.1) * dy) / den).clamp(0.0, 1.0);
        }
        best = best.min(d2(t).sqrt());
    }
    best
}

/// The largest distance from the outline of B-spline curve and surface
/// points, each turned back by the twist at its height (and moved back by
/// `offset`, turned back by `rotation`, for a placed part).
fn largest_error(solid: &Solid, pieces: &[Piece], teeth: usize, zones: &EndZones, rotation: f64, offset: V3) -> f64 {
    let back = |p: V3| {
        let (x, y) = (p[0] - offset[0], p[1] - offset[1]);
        let a = -rotation - zones.twist(p[2]);
        (x * a.cos() - y * a.sin(), x * a.sin() + y * a.cos())
    };
    let mut worst: f64 = 0.0;
    for e in &solid.edges {
        if let Curve::BSpline(b) = &e.curve {
            let (a, c) = b.range();
            for i in 0..=4 {
                worst = worst.max(distance_to_outline(pieces, teeth, back(b.at(a + (c - a) * i as f64 / 4.0))));
            }
        }
    }
    for f in &solid.faces {
        let (us, vs) = match &f.surface {
            Surface::BSpline { knots_u, knots_v, .. } => ((knots_u[3], knots_u[knots_u.len() - 4]), (knots_v[3], knots_v[knots_v.len() - 4])),
            _ => continue,
        };
        for i in 1..=3 {
            for j in 1..=3 {
                let (u, v) = (us.0 + (us.1 - us.0) * i as f64 / 4.0, vs.0 + (vs.1 - vs.0) * j as f64 / 4.0);
                worst = worst.max(distance_to_outline(pieces, teeth, back(f.surface.at(u, v).unwrap())));
            }
        }
    }
    worst
}

#[test]
fn pieces_join_up_and_follow_the_outline() {
    let s = system(30.0);
    for ring in [false, true] {
        let teeth = if ring { 63 } else { 21 };
        let (pieces, points) = if ring {
            (s.ring_gear_pieces(teeth).unwrap(), s.ring_gear(teeth).unwrap())
        } else {
            (s.spur_gear_pieces(teeth).unwrap(), s.spur_gear(teeth).unwrap())
        };
        for i in 0..pieces.len() {
            let (end, _) = pieces[i].at(1.0);
            let (start, _) = pieces[(i + 1) % pieces.len()].at(0.0);
            assert!((end.0 - start.0).hypot(end.1 - start.1) < 1e-9, "piece {i} ends at {end:?}, the next starts at {start:?}");
        }
        // The outline's points are on the pieces, or (tangent polygons) within
        // the deviation allowed: the root roundings' is the larger.
        let allowed = s.max_deviation.max(s.fillet_deviation) + 1e-9;
        for &q in &points {
            let d = distance_to_outline(&pieces, teeth, q);
            assert!(d <= allowed, "ring {ring}: outline point {q:?} is {d} from the pieces");
        }
    }
}

#[test]
fn piece_derivatives_match_their_points() {
    let s = system_with(30.0, 30.0);
    for p in s.spur_gear_pieces(12).unwrap().into_iter().chain(s.ring_gear_pieces(40).unwrap()) {
        for i in 1..10 {
            let t = i as f64 / 10.0;
            let h = 1e-6;
            let ((x0, y0), _) = p.at(t - h);
            let ((x1, y1), _) = p.at(t + h);
            let (_, (dx, dy)) = p.at(t);
            let (nx, ny) = ((x1 - x0) / (2.0 * h), (y1 - y0) / (2.0 * h));
            assert!((nx - dx).hypot(ny - dy) < 1e-4 * (1.0 + dx.hypot(dy)), "{p:?} at {t}: derivative {dx},{dy}, numerically {nx},{ny}");
        }
    }
}

#[test]
fn spur_and_ring_solids_are_closed_and_exact() {
    for helix in [0.0, 30.0] {
        let s = system_with(helix, 30.0);
        let c = chamfer(&s);
        for (eb, et) in [(0.0, 0.0), (c, c), (c, 0.0), (0.0, 2.0)] {
            for hand in if eb == et { vec![1, -1] } else { vec![1] } {
                for (ring, teeth, circle) in [(false, 21, 8.0), (false, 12, 0.0), (true, 63, 150.0)] {
                    let solid = if ring {
                        ring_gear_solid(&s, teeth, 10.0, circle, hand, eb, et).unwrap()
                    } else {
                        spur_gear_solid(&s, teeth, 10.0, circle, hand, eb, et).unwrap()
                    };
                    let what = format!("helix {helix}, ends {eb},{et}, hand {hand}, ring {ring}, {teeth} teeth");
                    let problems = solid.problems(1e-9);
                    assert!(problems.is_empty(), "{what}: {:?}", &problems[..problems.len().min(5)]);
                    let pieces = if ring { s.ring_gear_pieces(teeth).unwrap() } else { s.spur_gear_pieces(teeth).unwrap() };
                    let zones = EndZones::new(&s, teeth, 10.0, eb, et, hand, ring);
                    let err = largest_error(&solid, &pieces, teeth, &zones, 0.0, [0.0; 3]);
                    assert!(err < 2.0 * TOLERANCE, "{what}: a curve or surface is {err} mm from the exact shape");
                }
            }
        }
    }
}

#[test]
fn planetary_solids_are_closed_and_placed_like_the_meshes() {
    let s = system(30.0);
    let c = chamfer(&s);
    let solids = planetary_solids(&s, 21, 21, 4, 10.0, 150.0, 10.0, 8.0, 1, c, c).unwrap();
    let meshes = gears2::mesh::planetary_gear_mesh(&s, 21, 21, 4, 10.0, 150.0, 10.0, 8.0, 1, c, c).unwrap();
    assert_eq!(solids.len(), meshes.len());
    for (k, (solid, mesh)) in solids.iter().zip(&meshes).enumerate() {
        assert!(solid.problems(1e-9).is_empty());
        // the same place: the center of the vertices (a gear's are spread
        // evenly round its axis) and the same height range
        let center = |pts: &[[f64; 3]]| {
            let n = pts.len() as f64;
            (pts.iter().map(|p| p[0]).sum::<f64>() / n, pts.iter().map(|p| p[1]).sum::<f64>() / n)
        };
        let (a, b) = (center(&solid.vertices), center(&mesh.vertices));
        assert!((a.0 - b.0).hypot(a.1 - b.1) < 0.5, "part {k}: solid centered at {a:?}, mesh at {b:?}");
        let top = |pts: &[[f64; 3]]| pts.iter().map(|p| p[2]).fold(0.0, f64::max);
        assert!((top(&solid.vertices) - top(&mesh.vertices)).abs() < 1e-9);
    }
}

#[test]
fn step_text_is_well_formed() {
    let s = system_with(30.0, 30.0);
    let c = chamfer(&s);
    let solids = vec![spur_gear_solid(&s, 13, 6.0, 4.0, 1, c, 0.0).unwrap(), ring_gear_solid(&s, 40, 6.0, 100.0, -1, 0.0, c).unwrap()];
    let text = step_file(&solids, "test", TOLERANCE);
    assert!(text.starts_with("ISO-10303-21;\nHEADER;"));
    assert!(text.ends_with("ENDSEC;\nEND-ISO-10303-21;\n"));
    let data = &text[text.find("DATA;\n").unwrap() + 6..text.rfind("ENDSEC;").unwrap()];
    let mut defined = std::collections::HashSet::new();
    for (k, line) in data.lines().enumerate() {
        assert!(line.ends_with(';'), "{line}");
        let (id, body) = line.split_once('=').unwrap();
        let id: usize = id.trim_start_matches('#').parse().unwrap();
        assert_eq!(id, k + 1, "ids in order");
        // every reference is to an entity already defined
        for part in body.split('#').skip(1) {
            let r: usize = part.chars().take_while(|c| c.is_ascii_digit()).collect::<String>().parse().unwrap();
            assert!(defined.contains(&r), "#{id} refers to #{r}, not defined before it");
        }
        // reals always have a decimal point (no 1e-5 or bare integers in lists of reals)
        assert!(!body.contains("e-") && !body.contains("e+") && !body.contains("NaN") && !body.contains("inf"), "{line}");
        defined.insert(id);
    }
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 2);
    assert_eq!(text.matches("ADVANCED_FACE(").count(), solids.iter().map(|s| s.faces.len()).sum::<usize>());
}

#[test]
fn planetary_assemblies_share_parts_and_the_turned_over_sun_is_exact() {
    let s = system(30.0);
    let c = chamfer(&s);
    // the sun has the planets' teeth and bore, and the ends match: one part
    let a = planetary_assembly(&s, 21, 21, 4, 10.0, 150.0, 8.0, 8.0, 1, c, c).unwrap();
    assert_eq!(a.parts.len(), 2, "planet (also the sun) and ring");
    assert_eq!(a.instances.len(), 6);
    let sun = &a.solids()[0];
    assert!(sun.problems(1e-9).is_empty());
    let zones = EndZones::new(&s, 21, 10.0, c, c, 1, false);
    let err = largest_error(sun, &s.spur_gear_pieces(21).unwrap(), 21, &zones, 0.0, [0.0; 3]);
    assert!(err < 2.0 * TOLERANCE, "the turned-over planet is {err} mm from the sun's exact shape");
    // a different bore, or different ends: the sun is a part of its own
    assert_eq!(planetary_assembly(&s, 21, 21, 4, 10.0, 150.0, 10.0, 8.0, 1, c, c).unwrap().parts.len(), 3);
    assert_eq!(planetary_assembly(&s, 21, 21, 4, 10.0, 150.0, 8.0, 8.0, 1, c, 0.0).unwrap().parts.len(), 3);
    // the file holds each part's solid once
    let text = a.step("planetary");
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 2);
    assert_eq!(text.matches("NEXT_ASSEMBLY_USAGE_OCCURRENCE(").count(), 6);
}

/// Distance (on the unit sphere) from the unit vector q to a bevel gear's
/// outline: the nearest teeth's pieces, 24 samples each, then Newton.
fn distance_on_sphere(pieces: &[gears2::bevel::SphPiece], teeth: usize, q: V3) -> f64 {
    let per = pieces.len() / teeth;
    let tooth = (azimuth(q) / (2.0 * PI / teeth as f64)).round() as i64;
    let mut best = f64::INFINITY;
    for t in tooth - 1..=tooth + 1 {
        let t = t.rem_euclid(teeth as i64) as usize;
        for p in &pieces[t * per..(t + 1) * per] {
            let d = |t: f64| norm(sub(p.at(t), q));
            let mut t = (0..=24).map(|i| i as f64 / 24.0).min_by(|&a, &b| d(a).total_cmp(&d(b))).unwrap();
            for _ in 0..12 {
                let h = 1e-6;
                let (x, dx) = (p.at(t), scale(sub(p.at(t + h), p.at(t - h)), 0.5 / h));
                let den = gears2::step::dot(dx, dx);
                if den == 0.0 {
                    break;
                }
                t = (t - gears2::step::dot(sub(x, q), dx) / den).clamp(0.0, 1.0);
            }
            best = best.min(d(t));
        }
    }
    best
}

/// The largest distance (mm) of a bevel solid's B-spline points from the
/// exact twisted outline.
fn bevel_error(solid: &Solid, pair: &BevelPair, which: usize, hand: i32) -> f64 {
    let pieces = pair.outline_pieces(which);
    let teeth = pair.gears[which].teeth;
    let check = |p: V3| {
        let rho = norm(p);
        rho * distance_on_sphere(&pieces, teeth, unit(rot_z(p, -pair.twist(which, hand, rho))))
    };
    let mut worst: f64 = 0.0;
    for e in &solid.edges {
        if let Curve::BSpline(b) = &e.curve {
            let (a, c) = b.range();
            for i in 0..=4 {
                worst = worst.max(check(b.at(a + (c - a) * i as f64 / 4.0)));
            }
        }
    }
    for f in &solid.faces {
        if let Surface::BSpline { knots_u, knots_v, .. } = &f.surface {
            let (us, vs) = ((knots_u[3], knots_u[knots_u.len() - 4]), (knots_v[3], knots_v[knots_v.len() - 4]));
            for i in 1..=3 {
                for j in 1..=3 {
                    worst = worst.max(check(f.surface.at(us.0 + (us.1 - us.0) * i as f64 / 4.0, vs.0 + (vs.1 - vs.0) * j as f64 / 4.0).unwrap()));
                }
            }
        }
    }
    worst
}

#[test]
fn bevel_solids_are_closed_and_exact() {
    let s = system(30.0);
    for (z1, z2, shaft) in [(21, 30, 90.0), (15, 40, 60.0), (20, 20, 130.0)] {
        let pair = BevelPair::new(&s, [z1, z2], f64::to_radians(shaft), 12.0, true).unwrap();
        for which in 0..2 {
            for (hand, bore) in [(1, 8.0), (-1, 0.0)] {
                let solid = bevel_gear_solid(&pair, which, bore, hand).unwrap();
                let what = format!("{z1}/{z2} at {shaft}, gear {}, hand {hand}, bore {bore}", which + 1);
                let problems = solid.problems(1e-9);
                assert!(problems.is_empty(), "{what}: {:?}", &problems[..problems.len().min(5)]);
                let err = bevel_error(&solid, &pair, which, hand);
                assert!(err < 2.0 * TOLERANCE, "{what}: a curve or surface is {err} mm from the exact shape");
            }
        }
    }
}

#[test]
fn bevel_pairs_are_placed_like_the_meshes() {
    let s = system(30.0);
    for assembled in [true, false] {
        let a = bevel_pair_assembly(&s, [21, 30], PI / 2.0, 12.0, [8.0, 8.0], 1, true, assembled).unwrap();
        let (_, frames) = bevel_pair_layout(&s, [21, 30], PI / 2.0, 12.0, [8.0, 8.0], 1, true, assembled).unwrap();
        for ((_, place), frame) in a.instances.iter().zip(&frames) {
            // apex and axis (the frames' axis points from the gear to the apex: local +z)
            let (apex, axis) = (place.point([0.0; 3]), place.direction([0.0, 0.0, 1.0]));
            // laid out for printing, the gaps follow the widest point, found slightly differently
            let tol = if assembled { 1e-9 } else { 0.05 };
            assert!(norm(sub(apex, frame.apex)) < tol, "assembled {assembled}: apex {apex:?}, the mesh's {:?}", frame.apex);
            assert!(norm(sub(axis, frame.axis)) < 1e-9, "assembled {assembled}: axis {axis:?}, the mesh's {:?}", frame.axis);
        }
    }
}

/// Area inside a closed outline of pieces (Green's theorem, Gauss-Legendre
/// on each piece).
fn outline_area(pieces: &[Piece]) -> f64 {
    let (x, w) = ([-0.906179845938664, -0.538469310105683, 0.0, 0.538469310105683, 0.906179845938664],
        [0.236926885056189, 0.478628670499366, 0.568888888888889, 0.478628670499366, 0.236926885056189]);
    let mut area = 0.0;
    for p in pieces {
        for k in 0..64 {
            for (xi, wi) in x.iter().zip(&w) {
                let t = (k as f64 + 0.5 + xi / 2.0) / 64.0;
                let ((px, py), (dx, dy)) = p.at(t);
                area += 0.5 * (px * dy - py * dx) * wi / 2.0 / 64.0;
            }
        }
    }
    area
}

#[test]
fn opencascade_reads_the_files() {
    let Ok(python) = std::env::var("GEARS2_OCC_PYTHON") else {
        eprintln!("skipped: set GEARS2_OCC_PYTHON to a Python with OCP to run it");
        return;
    };
    let s = system(30.0);
    let c = chamfer(&s);
    let dir = std::env::temp_dir().join("gears2_step_test");
    std::fs::create_dir_all(&dir).unwrap();
    // A plain spur gear: its volume is exactly its cross-section's area times
    // the width (the twist doesn't change the area), less the bore.
    let (teeth, width, bore) = (23, 8.0, 6.0);
    let exact = (outline_area(&s.spur_gear_pieces(teeth).unwrap()) - PI * (bore / 2.0) * (bore / 2.0)) * width;
    let cases: Vec<(&str, Vec<Solid>, Vec<f64>)> = vec![
        ("plain", vec![spur_gear_solid(&s, teeth, width, bore, 1, 0.0, 0.0).unwrap()], vec![exact]),
        ("chamfered", vec![spur_gear_solid(&s, 21, 10.0, 8.0, -1, c, c).unwrap()], vec![]),
        ("ring", vec![ring_gear_solid(&s, 66, 6.0, 160.0, 1, c, 0.0).unwrap()], vec![]),
        ("planetary", planetary_solids(&s, 21, 21, 3, 5.0, 150.0, 4.0, 3.0, 1, c, c).unwrap(), vec![]),
    ];
    // Assemblies: written with each part once; read back, every instance.
    let assemblies = vec![
        ("planetary_assembly", planetary_assembly(&s, 21, 21, 3, 5.0, 150.0, 3.0, 3.0, 1, c, c).unwrap()),
        ("bevel_assembled", bevel_pair_assembly(&s, [21, 30], PI / 2.0, 12.0, [8.0, 8.0], 1, true, true).unwrap()),
        ("bevel_print", bevel_pair_assembly(&s, [20, 20], f64::to_radians(130.0), 8.0, [6.0, 0.0], -1, true, false).unwrap()),
    ];
    let mut files: Vec<(String, String, usize, Vec<f64>)> = cases
        .into_iter()
        .map(|(name, solids, volumes)| (name.to_string(), step_file(&solids, name, TOLERANCE), solids.len(), volumes))
        .collect();
    files.extend(assemblies.into_iter().map(|(name, a)| (name.to_string(), a.step(name), a.instances.len(), vec![])));
    for (name, text, count, volumes) in files {
        let path = dir.join(format!("{name}.step"));
        std::fs::write(&path, text).unwrap();
        let mut args = vec![std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tools/check_step.py").display().to_string(), path.display().to_string()];
        args.extend(volumes.iter().map(|v| v.to_string()));
        let out = std::process::Command::new(&python).args(&args).output().expect("python runs");
        let report = String::from_utf8_lossy(&out.stdout);
        assert!(out.status.success(), "{name}: OpenCascade found problems:\n{report}{}", String::from_utf8_lossy(&out.stderr));
        assert_eq!(report.lines().filter(|l| l.starts_with("solid")).count(), count, "{name}: {report}");
        // the volume of the plain gear, to within 1e-5 (relative)
        for (line, v) in report.lines().zip(&volumes) {
            let got: f64 = line.split("volume ").nth(1).unwrap().split_whitespace().next().unwrap().parse().unwrap();
            assert!((got - v).abs() < 1e-5 * v, "{name}: volume {got}, exactly {v}");
        }
    }
}
