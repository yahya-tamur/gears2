//! Exact solids of spur, ring and planetary gears, for STEP files: the same
//! shapes as the meshes in `mesh` (see spur_gear_mesh), from the outlines'
//! exact pieces (GearSystem::spur_gear_pieces) instead of their points.
//!
//! Faces of a gear, for each outline piece (fillet, flank, tip land, ...):
//! - the working width: two walls, below and above the middle, each the
//!   piece swept up while turning at the helix rate (one way, then back): a
//!   bicubic B-spline surface from the exact sweep and its derivatives;
//! - each end zone (if any): the piece extruded straight (no twist), up to
//!   where the chamfer cone cuts it. The tip land has no wall there (the cone
//!   meets it exactly at the working edge).
//!
//! Then, at each end: with an end zone, the cone (from the cut walls down to
//! the fold circle) and the flat face inside (spur) or outside (ring) the
//! fold circle; without one, a flat face bounded by the outline. The bore
//! (spur) or rim (ring) is a cylinder.
//!
//! Neighbouring faces share their edges, built from the same data, so they
//! match exactly: the outline at a height and the vertical curves where
//! pieces meet are the boundary curves of the B-spline walls (see
//! step::hermite_surface). Curves and surfaces stay within TOLERANCE (1e-4
//! mm, a hundredth of the meshes' default surface deviation) of the exact
//! shape.

use crate::gears::{GearSystem, Piece, Result};
use crate::mesh::{check_bore, check_common, check_rim, EndZones};
use crate::step::{add, cross, dot, hermite_breaks, hermite_curve, hermite_surface, scale, step_assembly, step_file, BSpline, Curve, Face, Placement, Solid, Surface, V3};

/// How closely curves and surfaces follow the exact shape, in mm.
pub const TOLERANCE: f64 = 1e-4;

const Z: V3 = [0.0, 0.0, 1.0];

fn turn(p: (f64, f64), a: f64) -> V3 {
    let (c, s) = (a.cos(), a.sin());
    [p.0 * c - p.1 * s, p.0 * s + p.1 * c, 0.0]
}

/// Turned 90 degrees counterclockwise about z.
fn perp(v: V3) -> V3 {
    [-v[1], v[0], 0.0]
}

/// The loop reversed: the edges in the other order, each the other way.
fn reversed(lp: Vec<(usize, bool)>) -> Vec<(usize, bool)> {
    lp.into_iter().rev().map(|(e, fw)| (e, !fw)).collect()
}

/// Signed area of a loop's projection on the xy plane.
fn xy_area(solid: &Solid, lp: &[(usize, bool)]) -> f64 {
    let pts = solid.loop_points(lp, 24);
    (0..pts.len()).map(|i| {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        p[0] * q[1] - q[0] * p[1]
    }).sum::<f64>() / 2.0
}

/// A face that projects one to one onto the xy plane (an end face or
/// cone), its normal up (`up`) or down: the largest loop runs
/// counterclockwise seen from outside, the others the other way.
fn flat_face(solid: &mut Solid, surface: Surface, same_sense: bool, loops: Vec<Vec<(usize, bool)>>, up: bool) {
    let sign = if up { 1.0 } else { -1.0 };
    let areas: Vec<f64> = loops.iter().map(|lp| xy_area(solid, lp)).collect();
    let outer = (0..loops.len()).max_by(|&a, &b| areas[a].abs().total_cmp(&areas[b].abs())).unwrap();
    let mut ordered = Vec::new();
    for (i, lp) in loops.into_iter().enumerate() {
        let want = if i == outer { sign } else { -sign };
        let lp = if areas[i] * want < 0.0 { reversed(lp) } else { lp };
        if i == outer { ordered.insert(0, lp) } else { ordered.push(lp) }
    }
    solid.faces.push(Face { surface, same_sense, loops: ordered });
}

/// An exact herringbone gear: `pieces` the outline (counterclockwise), the
/// material inside it (spur) or outside it (ring), and a plain circle of
/// radius `circle` (the bore or rim; 0 for none).
fn gear_solid(pieces: &[Piece], zones: &EndZones, ring: bool, circle: f64) -> Solid {
    let mut solid = Solid::default();
    let n = pieces.len();
    let (z_lo, z_hi, total) = (zones.z_lo, zones.z_hi, zones.total);
    let z_mid = 0.5 * (z_lo + z_hi);
    let k = zones.k;
    let levels = [z_lo, z_mid, z_hi];
    // the turn at height z and its rate, in the lower and upper halves
    let theta = |z: f64| zones.twist(z);
    let rate = |half: usize| if half == 0 { k } else { -k };
    // outward: right of the outline's direction for a spur gear, left for a ring
    let side = if ring { -1.0 } else { 1.0 };

    // Each piece's breakpoints: where its Hermite fit stays within tolerance.
    let piece_fn = |i: usize| {
        let p = pieces[i];
        move |t: f64| -> (V3, V3) {
            let (q, d) = p.at(t);
            ([q.0, q.1, 0.0], [d.0, d.1, 0.0])
        }
    };
    let ts: Vec<Vec<f64>> = (0..n).map(|i| hermite_breaks(&piece_fn(i), 0.0, 1.0, &[], TOLERANCE)).collect();
    // Heights: where a helix at the outermost radius stays within tolerance.
    let r_max = pieces.iter().map(|p| (0..=8).map(|j| {
        let (q, _) = p.at(j as f64 / 8.0);
        q.0.hypot(q.1)
    }).fold(0.0, f64::max)).fold(0.0, f64::max);
    let zs: Vec<Vec<f64>> = (0..2).map(|half| {
        let (a, b) = (levels[half], levels[half + 1]);
        let w = rate(half);
        let helix = move |z: f64| {
            let p = turn((r_max, 0.0), theta(z));
            (add(p, scale(Z, z)), add(scale(perp(p), w), Z))
        };
        hermite_breaks(&helix, a, b, &[], TOLERANCE)
    }).collect();

    // the outline piece i at height z, turned by the twist there
    let outline = |i: usize, z: f64| -> BSpline {
        let (p, a) = (pieces[i], theta(z));
        hermite_curve(&move |t: f64| {
            let (q, d) = p.at(t);
            (add(turn(q, a), scale(Z, z)), turn(d, a))
        }, &ts[i])
    };
    let junction = |j: usize| pieces[j % n].at(0.0).0;

    // Vertices and edges of the working width.
    let w_vert: Vec<Vec<usize>> = levels.iter().map(|&z| (0..n).map(|j| solid.add_vertex(add(turn(junction(j), theta(z)), scale(Z, z)))).collect()).collect();
    let mut h_edge = vec![vec![0; n]; 3];
    for (l, &z) in levels.iter().enumerate() {
        for i in 0..n {
            h_edge[l][i] = solid.add_edge(w_vert[l][i], w_vert[l][(i + 1) % n], Curve::BSpline(outline(i, z)));
        }
    }
    let mut v_edge = vec![vec![0; n]; 2];
    for half in 0..2 {
        let w = rate(half);
        for j in 0..n {
            let q = junction(j);
            let helix = hermite_curve(&move |z: f64| {
                let p = turn(q, theta(z));
                (add(p, scale(Z, z)), add(scale(perp(p), w), Z))
            }, &zs[half]);
            v_edge[half][j] = solid.add_edge(w_vert[half][j], w_vert[half + 1][j], Curve::BSpline(helix));
        }
    }
    // Walls of the working width.
    for half in 0..2 {
        let w = rate(half);
        for i in 0..n {
            let p = pieces[i];
            let sweep = move |t: f64, z: f64| {
                let (a, (q, d)) = (theta(z), p.at(t));
                let (q, d) = (turn(q, a), turn(d, a));
                (add(q, scale(Z, z)), d, add(scale(perp(q), w), Z), scale(perp(d), w))
            };
            let surface = hermite_surface(&sweep, &ts[i], &zs[half]);
            let (_, su, sv, _) = sweep(0.5, 0.5 * (levels[half] + levels[half + 1]));
            let outward = scale([su[1], -su[0], 0.0], side);
            let same = dot(cross(su, sv), outward) > 0.0;
            let lp = vec![
                (h_edge[half][i], true),
                (v_edge[half][(i + 1) % n], true),
                (h_edge[half + 1][i], false),
                (v_edge[half][i], false),
            ];
            solid.faces.push(Face { surface, same_sense: same, loops: vec![if same { lp } else { reversed(lp) }] });
        }
    }

    // The plain circle: a cylinder from bottom to top, with a seam at angle 0.
    let mut circle_edges = None;
    if circle > 0.0 {
        let (b, t) = (solid.add_vertex([circle, 0.0, 0.0]), solid.add_vertex([circle, 0.0, total]));
        let ring_at = |z: f64| Curve::Circle { center: [0.0, 0.0, z], axis: Z, x_dir: [1.0, 0.0, 0.0], radius: circle };
        let (cb, ct) = (solid.add_edge(b, b, ring_at(0.0)), solid.add_edge(t, t, ring_at(total)));
        let seam = solid.add_edge(b, t, Curve::Line { from: [circle, 0.0, 0.0], to: [circle, 0.0, total] });
        let lp = vec![(cb, true), (seam, true), (ct, false), (seam, false)];
        // its normal points away from the axis: out of a ring's rim, into a bore
        let same = ring;
        let surface = Surface::Cylinder { origin: [0.0; 3], axis: Z, x_dir: [1.0, 0.0, 0.0], radius: circle };
        solid.faces.push(Face { surface, same_sense: same, loops: vec![if same { lp } else { reversed(lp) }] });
        circle_edges = Some((cb, ct));
    }

    // The ends.
    for bottom in [true, false] {
        let (z_face, z_edge, level, length) = if bottom { (0.0, z_lo, 0, zones.ends.0) } else { (total, z_hi, 2, zones.ends.1) };
        let plain = circle_edges.map(|(cb, ct)| vec![(if bottom { cb } else { ct }, true)]);
        if length <= 0.0 {
            // a flat end bounded by the outline (and the plain circle)
            let mut loops = vec![(0..n).map(|i| (h_edge[level][i], true)).collect::<Vec<_>>()];
            loops.extend(plain);
            let normal = if bottom { [0.0, 0.0, -1.0] } else { Z };
            flat_face(&mut solid, Surface::Plane { origin: [0.0, 0.0, z_face], normal, x_dir: [1.0, 0.0, 0.0] }, true, loops, !bottom);
            continue;
        }
        // The cone: height above (below) the end face at radius r.
        let (r_tip, r_fold) = (zones.r_tip, zones.r_fold);
        let slope = length / (r_tip - r_fold); // dh/dr
        let h = move |r: f64| length * (r - r_fold) / (r_tip - r_fold);
        let z_at = move |r: f64| if bottom { h(r) } else { total - h(r) };
        let tip_land = |i: usize| matches!(pieces[i], Piece::Arc { center, radius, .. }
            if center.0.hypot(center.1) < 1e-9 && (radius - r_tip).abs() < 1e-9);
        // Where each junction is cut (merged with the working edge's vertex
        // where the cone meets it there: at the tip corners).
        let mut cut_vert = vec![0; n];
        let mut riser = vec![None; n];
        for j in 0..n {
            let q = junction(j);
            let z = z_at(q.0.hypot(q.1));
            if (z - z_edge).abs() < 1e-9 {
                cut_vert[j] = w_vert[level][j];
            } else {
                cut_vert[j] = solid.add_vertex([q.0, q.1, z]);
                let (lo, hi) = if bottom { (cut_vert[j], w_vert[level][j]) } else { (w_vert[level][j], cut_vert[j]) };
                riser[j] = Some(solid.add_edge(lo, hi, Curve::Line { from: solid.vertices[lo], to: solid.vertices[hi] }));
            }
        }
        // Each piece's cut curve: its outline, lifted onto the cone (the same
        // xy control points as the outline, so it lies on the straight wall).
        let mut cut_edge = vec![None; n];
        for i in 0..n {
            if tip_land(i) {
                continue;
            }
            let p = pieces[i];
            let curve = hermite_curve(&move |t: f64| {
                let (q, d) = p.at(t);
                let r = q.0.hypot(q.1);
                let dz = slope * (q.0 * d.0 + q.1 * d.1) / r * if bottom { 1.0 } else { -1.0 };
                ([q.0, q.1, z_at(r)], [d.0, d.1, dz])
            }, &ts[i]);
            cut_edge[i] = Some(solid.add_edge(cut_vert[i], cut_vert[(i + 1) % n], Curve::BSpline(curve)));
        }
        // Straight walls of the end zone.
        for i in 0..n {
            let Some(cut) = cut_edge[i] else { continue };
            let base = outline(i, z_edge);
            let (_, d) = pieces[i].at(0.5);
            let su = [d.0, d.1, 0.0];
            let same = dot(cross(su, Z), scale([su[1], -su[0], 0.0], side)) > 0.0;
            let (lower, upper) = if bottom { (cut, h_edge[level][i]) } else { (h_edge[level][i], cut) };
            let mut lp = vec![(lower, true)];
            if let Some(e) = riser[(i + 1) % n] {
                lp.push((e, true));
            }
            lp.push((upper, false));
            if let Some(e) = riser[i] {
                lp.push((e, false));
            }
            solid.faces.push(Face { surface: Surface::Extrusion { curve: base, dir: Z }, same_sense: same, loops: vec![if same { lp } else { reversed(lp) }] });
        }
        // The cone face: the cut curves (and tip lands) all round, a seam down
        // to the fold circle at a junction where the cone cuts, the fold circle.
        let j0 = (0..n).find(|&j| riser[j].is_some()).expect("a cut junction");
        let q0 = junction(j0);
        let dir0 = [q0.0 / q0.0.hypot(q0.1), q0.1 / q0.0.hypot(q0.1), 0.0];
        let fold_vert = solid.add_vertex(add(scale(dir0, r_fold), scale(Z, z_face)));
        let fold = solid.add_edge(fold_vert, fold_vert, Curve::Circle { center: [0.0, 0.0, z_face], axis: Z, x_dir: dir0, radius: r_fold });
        let seam = solid.add_edge(fold_vert, cut_vert[j0], Curve::Line { from: solid.vertices[fold_vert], to: solid.vertices[cut_vert[j0]] });
        let mut lp: Vec<(usize, bool)> = (0..n).map(|m| {
            let i = (j0 + m) % n;
            (cut_edge[i].unwrap_or(h_edge[level][i]), true)
        }).collect();
        lp.extend([(seam, false), (fold, false), (seam, true)]);
        // radius r_fold at the end face, growing toward the working width
        let growing_up = (r_tip > r_fold) == bottom;
        let axis = if growing_up { Z } else { [0.0, 0.0, -1.0] };
        let semi_angle = ((r_tip - r_fold).abs() / length).atan();
        let cone = Surface::Cone { origin: [0.0, 0.0, z_face], axis, x_dir: dir0, radius: r_fold, semi_angle };
        // same sense: the cone's normal (away from its axis) against outward
        let normal = add(scale([1.0, 0.0, 0.0], semi_angle.cos()), scale(axis, -semi_angle.sin()));
        let outward = [slope, 0.0, if bottom { -1.0 } else { 1.0 }];
        flat_face(&mut solid, cone, dot(normal, outward) > 0.0, vec![lp], !bottom);
        // The flat face: inside the fold circle (spur) or outside it (ring).
        let mut loops = vec![vec![(fold, true)]];
        loops.extend(plain);
        let normal = if bottom { [0.0, 0.0, -1.0] } else { Z };
        flat_face(&mut solid, Surface::Plane { origin: [0.0, 0.0, z_face], normal, x_dir: [1.0, 0.0, 0.0] }, true, loops, !bottom);
    }
    solid
}

/// The exact solid of spur_gear_mesh() (same arguments).
pub fn spur_gear_solid(s: &GearSystem, teeth: usize, width: f64, bore: f64, hand: i32, end_bottom: f64, end_top: f64) -> Result<Solid> {
    check_common(width, hand, end_bottom, end_top)?;
    let zones = EndZones::new(s, teeth, width, end_bottom, end_top, hand, false);
    check_bore(s, &zones, bore, end_bottom > 0.0 || end_top > 0.0)?;
    Ok(gear_solid(&s.spur_gear_pieces(teeth)?, &zones, false, bore / 2.0))
}

/// The exact solid of ring_gear_mesh() (same arguments).
pub fn ring_gear_solid(s: &GearSystem, teeth: usize, width: f64, rim: f64, hand: i32, end_bottom: f64, end_top: f64) -> Result<Solid> {
    check_common(width, hand, end_bottom, end_top)?;
    let zones = EndZones::new(s, teeth, width, end_bottom, end_top, hand, true);
    check_rim(s, &zones, rim, end_bottom > 0.0 || end_top > 0.0)?;
    Ok(gear_solid(&s.ring_gear_pieces(teeth)?, &zones, true, rim / 2.0))
}

/// Parts written once each and placed one or more times (a STEP assembly).
#[derive(Debug, Clone)]
pub struct Assembly {
    pub parts: Vec<(String, Solid)>,
    /// (part, where it goes), in the order of the meshes.
    pub instances: Vec<(usize, Placement)>,
}

impl Assembly {
    /// One part, in place.
    pub fn single(name: &str, solid: Solid) -> Assembly {
        Assembly { parts: vec![(name.to_string(), solid)], instances: vec![(0, Placement::IDENTITY)] }
    }

    /// Every instance as a solid of its own, in place.
    pub fn solids(&self) -> Vec<Solid> {
        self.instances.iter().map(|(part, place)| self.parts[*part].1.transformed(place)).collect()
    }

    /// The STEP file: a single part as a plain part file, several as an
    /// assembly.
    pub fn step(&self, name: &str) -> String {
        if self.parts.len() == 1 && self.instances.len() == 1 {
            return step_file(&self.solids(), name, TOLERANCE);
        }
        let parts: Vec<(&str, &Solid)> = self.parts.iter().map(|(n, s)| (n.as_str(), s)).collect();
        step_assembly(&parts, &self.instances, name, TOLERANCE)
    }
}

/// planetary_solids() as an assembly: the planet written once and placed
/// at each planet. The sun is the planet turned upside down when it has the
/// planet's teeth and bore and the ends match (turning it over swaps the
/// ends and reverses the twist, as the sun needs); otherwise its own part.
/// (Mirroring would also reverse the twist, but STEP places parts by
/// rotations only.)
#[allow(clippy::too_many_arguments)]
pub fn planetary_assembly(
    s: &GearSystem,
    teeth_sun: usize,
    teeth_planet: usize,
    planets: usize,
    width: f64,
    rim: f64,
    bore: f64,
    planet_bore: f64,
    hand: i32,
    end_bottom: f64,
    end_top: f64,
) -> Result<Assembly> {
    let (teeth_ring, layout, ring_rotation) = s.planetary_layout(teeth_sun, teeth_planet, planets, 0.0)?;
    let planet = spur_gear_solid(s, teeth_planet, width, planet_bore, -hand, end_bottom, end_top)?;
    let ring = ring_gear_solid(s, teeth_ring, width, rim, -hand, end_bottom, end_top)?;
    let mut parts = vec![("planet".to_string(), planet)];
    let total = end_bottom + width + end_top;
    let sun = if teeth_sun == teeth_planet && bore == planet_bore && end_bottom == end_top {
        // upside down: x stays, y and z flip; the bottom face goes to the top
        (0, Placement { origin: [0.0, 0.0, total], axis: [0.0, 0.0, -1.0], x_dir: [1.0, 0.0, 0.0] })
    } else {
        parts.push(("sun".to_string(), spur_gear_solid(s, teeth_sun, width, bore, hand, end_bottom, end_top)?));
        (parts.len() - 1, Placement::IDENTITY)
    };
    parts.push(("ring".to_string(), ring));
    let mut instances = vec![sun];
    for (center, rotation) in layout {
        instances.push((0, Placement::turned(rotation, [center.0, center.1, 0.0])));
    }
    instances.push((parts.len() - 1, Placement::turned(ring_rotation, [0.0; 3])));
    Ok(Assembly { parts, instances })
}

/// The exact solids of planetary_gear_mesh() (same arguments and order: sun,
/// planets, ring), placed the same way.
#[allow(clippy::too_many_arguments)]
pub fn planetary_solids(
    s: &GearSystem,
    teeth_sun: usize,
    teeth_planet: usize,
    planets: usize,
    width: f64,
    rim: f64,
    bore: f64,
    planet_bore: f64,
    hand: i32,
    end_bottom: f64,
    end_top: f64,
) -> Result<Vec<Solid>> {
    Ok(planetary_assembly(s, teeth_sun, teeth_planet, planets, width, rim, bore, planet_bore, hand, end_bottom, end_top)?.solids())
}
