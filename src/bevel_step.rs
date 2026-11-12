//! Exact solids of bevel gears, for STEP files: the same shapes as
//! bevel::bevel_gear_mesh, from the outline's exact pieces on the unit
//! sphere (BevelPair::outline_pieces).
//!
//! In each gear's own frame (apex at the origin, axis -z):
//! - walls: for each outline piece, the piece scaled by the cone distance
//!   rho and turned by the twist there, from the toe (R - face) to the middle
//!   (sqrt(R (R - face))) and from there to the heel (R): a bicubic B-spline
//!   surface each, from the exact sweep and its derivatives;
//! - the heel and toe: a zone of the sphere of radius rho about the apex,
//!   between the tooth outline (where the twist is 0) and the fold circle,
//!   and inside the fold circle a flat face square to the axis;
//! - the bore, a cylinder from one flat face to the other.
//!
//! As for spur gears (see gear_step), neighbouring faces share their edges,
//! built from the same data.

use crate::bevel::{sph, BevelPair, SphPiece};
use crate::gear_step::{check_tolerance, Assembly};
use crate::gears::{GearError, GearSystem, Result};
use crate::step::{add, cross, dot, hermite_breaks, hermite_curve, hermite_surface, norm, scale, sub, unit, Curve, Face, Placement, Solid, Surface, V3};

const Z: V3 = [0.0, 0.0, 1.0];

/// Turned about z.
fn turn(v: V3, a: f64) -> V3 {
    let (c, s) = (a.cos(), a.sin());
    [v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]]
}

/// Turned 90 degrees counterclockwise about z (d/da of turn).
fn perp(v: V3) -> V3 {
    [-v[1], v[0], 0.0]
}

fn reversed(lp: Vec<(usize, bool)>) -> Vec<(usize, bool)> {
    lp.into_iter().rev().map(|(e, fw)| (e, !fw)).collect()
}

/// A piece's point and derivative at t (the derivative by central
/// differences: the spherical involute's is messy, and only ever used to
/// build Hermite data, the same way for a surface and its edges).
fn piece_at(p: &SphPiece, t: f64) -> (V3, V3) {
    let h = 1e-6;
    (p.at(t), scale(sub(p.at(t + h), p.at(t - h)), 1.0 / (2.0 * h)))
}

/// The exact solid of bevel_gear_mesh() (same arguments), B-splines within
/// `tol` mm.
pub fn bevel_gear_solid(pair: &BevelPair, which: usize, bore: f64, hand: i32, tol: f64) -> Result<Solid> {
    if hand != 1 && hand != -1 {
        return Err(GearError("The twist direction must be 1 or -1.".into()));
    }
    check_tolerance(tol)?;
    pair.check_bore(which, bore)?;
    let g = &pair.gears[which];
    let (r, f) = (pair.r_cone, pair.face);
    let toe = r - f;
    let mid = (r * toe).sqrt();
    let levels = [toe, mid, r];
    let fold = pair.fold(which);
    let pieces = pair.outline_pieces(which);
    let n = pieces.len();
    let tw = |rho: f64| pair.twist(which, hand, rho);
    // rho times the twist's rate (constant in each half: the twist is linear in ln rho)
    let sign = if which == 0 { hand } else { -hand } as f64;
    let k = sign * pair.helix_angle.tan() / g.delta.sin();
    let rate = |half: usize| if half == 0 { k } else { -k };

    let mut solid = Solid::default();
    // Breakpoints: each piece at the heel (where it's largest), and the
    // cone distances along the tip's path at the heel.
    let ts: Vec<Vec<f64>> = pieces.iter().map(|p| {
        let p = *p;
        hermite_breaks(&move |t: f64| {
            let (q, d) = piece_at(&p, t);
            (scale(q, r), scale(d, r))
        }, 0.0, 1.0, &[], tol)
    }).collect();
    let q_tip = sph(g.tip, 0.0);
    let rhos: Vec<Vec<f64>> = (0..2).map(|half| {
        let w = rate(half);
        let path = move |rho: f64| {
            let q = turn(q_tip, tw(rho));
            (scale(q, rho), add(q, scale(perp(q), w)))
        };
        hermite_breaks(&path, levels[half], levels[half + 1], &[], tol)
    }).collect();

    let junction = |j: usize| pieces[j % n].at(0.0);
    let w_vert: Vec<Vec<usize>> = levels.iter().map(|&rho| (0..n).map(|j| solid.add_vertex(scale(turn(junction(j), tw(rho)), rho))).collect()).collect();
    let mut h_edge = vec![vec![0; n]; 3];
    for (l, &rho) in levels.iter().enumerate() {
        let a = tw(rho);
        for i in 0..n {
            let p = pieces[i];
            let curve = hermite_curve(&move |t: f64| {
                let (q, d) = piece_at(&p, t);
                (scale(turn(q, a), rho), scale(turn(d, a), rho))
            }, &ts[i]);
            h_edge[l][i] = solid.add_edge(w_vert[l][i], w_vert[l][(i + 1) % n], Curve::BSpline(curve));
        }
    }
    let mut v_edge = vec![vec![0; n]; 2];
    for half in 0..2 {
        let w = rate(half);
        for j in 0..n {
            let q0 = junction(j);
            let curve = hermite_curve(&move |rho: f64| {
                let q = turn(q0, tw(rho));
                (scale(q, rho), add(q, scale(perp(q), w)))
            }, &rhos[half]);
            v_edge[half][j] = solid.add_edge(w_vert[half][j], w_vert[half + 1][j], Curve::BSpline(curve));
        }
    }
    // Walls.
    for half in 0..2 {
        let w = rate(half);
        for i in 0..n {
            let p = pieces[i];
            let sweep = move |t: f64, rho: f64| {
                let a = tw(rho);
                let (q, d) = piece_at(&p, t);
                let (q, d) = (turn(q, a), turn(d, a));
                (scale(q, rho), scale(d, rho), add(q, scale(perp(q), w)), add(d, scale(perp(d), w)))
            };
            let surface = hermite_surface(&sweep, &ts[i], &rhos[half]);
            let (s0, su, sv, _) = sweep(0.5, 0.5 * (levels[half] + levels[half + 1]));
            // outward: away from the tooth, square to the outline on the sphere
            let outward = cross(unit(s0), su);
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

    // The bore: a cylinder between the flat faces (heel lower, toe higher).
    let (z_heel, z_toe) = (-r * fold.cos(), -toe * fold.cos());
    let rb = bore / 2.0;
    let mut bore_edges = None;
    if rb > 0.0 {
        let (b, t) = (solid.add_vertex([rb, 0.0, z_heel]), solid.add_vertex([rb, 0.0, z_toe]));
        let ring = |z: f64| Curve::Circle { center: [0.0, 0.0, z], axis: Z, x_dir: [1.0, 0.0, 0.0], radius: rb };
        let (cb, ct) = (solid.add_edge(b, b, ring(z_heel)), solid.add_edge(t, t, ring(z_toe)));
        let seam = solid.add_edge(b, t, Curve::Line { from: [rb, 0.0, z_heel], to: [rb, 0.0, z_toe] });
        // its normal points away from the axis, into the bore: reversed
        let lp = vec![(cb, true), (seam, true), (ct, false), (seam, false)];
        let surface = Surface::Cylinder { origin: [0.0, 0.0, z_heel], axis: Z, x_dir: [1.0, 0.0, 0.0], radius: rb };
        solid.faces.push(Face { surface, same_sense: false, loops: vec![reversed(lp)] });
        bore_edges = Some((cb, ct));
    }

    // The ends.
    for heel in [true, false] {
        let (rho, level) = if heel { (r, 2) } else { (toe, 0) };
        let z = -rho * fold.cos();
        let q0 = junction(0);
        let dir0 = unit([q0[0], q0[1], 0.0]);
        let fold_vert = solid.add_vertex(scale(sph(fold, dir0[1].atan2(dir0[0])), rho));
        let fold_circle = solid.add_edge(fold_vert, fold_vert, Curve::Circle { center: [0.0, 0.0, z], axis: Z, x_dir: dir0, radius: rho * fold.sin() });
        // the seam: the meridian arc from the fold circle up to junction 0
        let (fp, bp) = (solid.vertices[fold_vert], solid.vertices[w_vert[level][0]]);
        let axis = unit(cross(fp, bp));
        let sweep = norm(cross(fp, bp)).atan2(dot(fp, bp));
        let seam = solid.add_edge(fold_vert, w_vert[level][0], Curve::Arc { center: [0.0; 3], axis, x_dir: unit(fp), radius: rho, sweep });
        // The sphere zone, seen from outside: at the heel (outside is away
        // from the apex) the outline runs clockwise about z and the fold
        // circle counterclockwise; at the toe the other way.
        let chain: Vec<(usize, bool)> = (0..n).map(|i| (h_edge[level][i], true)).collect();
        let mut lp = if heel { reversed(chain) } else { chain };
        lp.extend([(seam, false), (fold_circle, heel), (seam, true)]);
        let sphere = Surface::Sphere { center: [0.0; 3], axis: [0.0, 0.0, -1.0], x_dir: dir0, radius: rho };
        solid.faces.push(Face { surface: sphere, same_sense: heel, loops: vec![lp] });
        // The flat face inside the fold circle, normal down at the heel, up at the toe.
        let mut loops = vec![vec![(fold_circle, !heel)]];
        if let Some((cb, ct)) = bore_edges {
            loops.push(vec![if heel { (cb, true) } else { (ct, false) }]);
        }
        let normal = if heel { [0.0, 0.0, -1.0] } else { Z };
        solid.faces.push(Face { surface: Surface::Plane { origin: [0.0, 0.0, z], normal, x_dir: [1.0, 0.0, 0.0] }, same_sense: true, loops });
    }
    Ok(solid)
}

/// Both gears as exact solids, placed as bevel::bevel_pair_layout places
/// the meshes: assembled (gear 2 turned into place, gear 1's heel face
/// centered at the origin), or for printing (each on its heel face at z = 0,
/// side by side along x, 5 mm apart).
#[allow(clippy::too_many_arguments)]
pub fn bevel_pair_assembly(
    s: &GearSystem,
    teeth: [usize; 2],
    shaft_angle: f64,
    face: f64,
    bores: [f64; 2],
    hand: i32,
    backlash_at_toe: bool,
    assembled: bool,
    tol: f64,
) -> Result<Assembly> {
    let pair = BevelPair::new(s, teeth, shaft_angle, face, backlash_at_toe)?;
    let r = pair.r_cone;
    let parts = vec![
        ("gear 1".to_string(), bevel_gear_solid(&pair, 0, bores[0], hand, tol)?),
        ("gear 2".to_string(), bevel_gear_solid(&pair, 1, bores[1], hand, tol)?),
    ];
    // Each gear's lowest point in its own frame: the center of its heel face.
    let low = |which: usize| -r * pair.fold(which).cos();
    let instances = if assembled {
        let rot = pair.mesh_rotation(0.0);
        let lift = [0.0, 0.0, -low(0)];
        let gear2 = Placement {
            origin: lift,
            axis: pair.to_world(1, rot, Z),
            x_dir: pair.to_world(1, rot, [1.0, 0.0, 0.0]),
        };
        vec![(0, Placement { origin: lift, ..Placement::IDENTITY }), (1, gear2)]
    } else {
        // reach: the widest point of the teeth (at the heel)
        let reach = |which: usize| {
            pair.outline_pieces(which).iter().flat_map(|p| (0..=8).map(move |k| p.at(k as f64 / 8.0))).map(|q| r * q[0].hypot(q[1])).fold(0.0, f64::max)
        };
        let mut x = 0.0;
        let mut out = Vec::new();
        for which in 0..2 {
            if which == 1 {
                x += reach(1);
            }
            out.push((which, Placement { origin: [x, 0.0, -low(which)], ..Placement::IDENTITY }));
            x += reach(which) + 5.0;
        }
        out
    };
    Ok(Assembly { parts, instances, tolerance: tol })
}
