//! Tests for the mesh module: the indexed mesh and its checks, the refined
//! constrained Delaunay triangulation, and gear solids.
//!
//! Also: refining the faces must not change what the rotating tests
//! measured. Ruppert only adds interior points and midpoints of existing
//! boundary segments, so the face boundary is still exactly the gear's
//! outline (same straight segments, a few extra vertices on them). Two tests
//! check that directly, and one reruns a driven rotation sweep on the refined
//! boundary.

mod common;

use common::*;
use gears2::gears::{GearSystem, Params, P};
use gears2::mesh::{
    combined, incircle, layer_heights, min_angle_of, orient, planetary_gear_mesh, ring_gear_mesh, spur_gear_mesh,
    triangulate, EndZones, IndexedMesh, Triangulated,
};
use rustc_hash::{FxHashMap, FxHashSet};
use std::f64::consts::PI;

fn min_angle() -> f64 {
    deg(25.0)
}

fn base() -> Params {
    let mut p = params(20.0, 4.0, 0.01, 0.05);
    p.helix_angle = deg(30.0);
    p
}

fn with(f: impl FnOnce(&mut Params)) -> GearSystem {
    let mut p = base();
    f(&mut p);
    system(p)
}

fn polygon_area(lp: &[P]) -> f64 {
    let n = lp.len();
    (0..n).map(|i| lp[(i + n - 1) % n].0 * lp[i].1 - lp[i].0 * lp[(i + n - 1) % n].1).sum::<f64>() / 2.0
}

fn hole(s: &GearSystem, radius: f64) -> Vec<P> {
    let mut h = s.circle(radius, true).unwrap();
    h.reverse();
    h
}

fn check_triangulation(loops: &[Vec<P>], t: &Triangulated, angle: f64) {
    let pts = &t.points;
    // Every triangle counterclockwise, the areas add up to the polygon's.
    assert!(t.triangles.iter().all(|&[a, b, c]| orient(pts[a], pts[b], pts[c]) > 0.0));
    let total: f64 = t.triangles.iter().map(|&[a, b, c]| orient(pts[a], pts[b], pts[c]) / 2.0).sum();
    assert_close(total, loops.iter().map(|l| polygon_area(l)).sum(), 1e-9, "area");
    // Every refined boundary segment is a triangle edge, traversed in loop
    // direction (so the material is on its left).
    let edges: FxHashSet<(usize, usize)> = t.triangles.iter().flat_map(|&[a, b, c]| [(a, b), (b, c), (c, a)]).collect();
    for lp in &t.boundary {
        for i in 0..lp.len() {
            assert!(edges.contains(&(lp[i], lp[(i + 1) % lp.len()])));
        }
    }
    // Constrained Delaunay: across every edge that isn't a boundary segment,
    // the opposite vertex is not inside the triangle's circumcircle.
    let mut owner: FxHashMap<(usize, usize), usize> = FxHashMap::default();
    for &[a, b, c] in &t.triangles {
        for (u, v, w) in [(a, b, c), (b, c, a), (c, a, b)] {
            owner.insert((u, v), w);
        }
    }
    for (&(u, v), &w) in &owner {
        let Some(&x) = owner.get(&(v, u)) else { continue };
        let (a, b, c, d) = (pts[u], pts[v], pts[w], pts[x]);
        let scale = max_of([b, c, d].iter().map(|p| (p.0 - a.0).hypot(p.1 - a.1))).powi(4);
        assert!(incircle(a, b, c, d) <= 1e-9 * scale, "not Delaunay across edge {}-{}", u, v);
    }
    // Quality.
    assert!(min_angle_of(pts, &t.triangles) >= angle - 1e-9);
}

/// The refined loop holds the original points in order, and every point added
/// lies on the original segment it was added to.
fn check_boundary_unchanged(lp: &[P], pts: &[P], refined: &[usize]) {
    let on_loop: FxHashSet<(u64, u64)> = lp.iter().map(|p| (p.0.to_bits(), p.1.to_bits())).collect();
    let originals: Vec<usize> = (0..refined.len()).filter(|&i| on_loop.contains(&(pts[refined[i]].0.to_bits(), pts[refined[i]].1.to_bits()))).collect();
    let in_order: Vec<P> = originals.iter().map(|&i| pts[refined[i]]).collect();
    assert_eq!(in_order, lp); // same points, same order
    for k in 0..originals.len() {
        let (start, end) = (originals[k], originals[(k + 1) % originals.len()]);
        let (a, b) = (pts[refined[start]], pts[refined[end]]);
        let mut idx = start + 1;
        while idx % refined.len() != end {
            let p = pts[refined[idx % refined.len()]];
            // on the segment ab: collinear, between the ends
            assert!(orient(a, b, p).abs() <= 1e-9 * (b.0 - a.0).hypot(b.1 - a.1).powi(2));
            assert!((p.0 - a.0) * (b.0 - a.0) + (p.1 - a.1) * (b.1 - a.1) > 0.0);
            assert!((p.0 - b.0) * (a.0 - b.0) + (p.1 - b.1) * (a.1 - b.1) > 0.0);
            idx += 1;
        }
    }
}

// ------------------------------------------------------------- triangulation

#[test]
fn square_with_hole() {
    let loops = vec![vec![(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)], vec![(1.0, 1.0), (1.0, 3.0), (3.0, 3.0), (3.0, 1.0)]];
    let t = triangulate(&loops, min_angle(), 0.0, &[]).unwrap();
    check_triangulation(&loops, &t, min_angle());
}

#[test]
fn skinny_polygon_gets_refined() {
    // A long thin rectangle: its boundary points alone only allow slivers.
    let loops = vec![vec![(0.0, 0.0), (10.0, 0.0), (10.0, 0.5), (0.0, 0.5)]];
    let t = triangulate(&loops, min_angle(), 0.0, &[]).unwrap();
    check_triangulation(&loops, &t, min_angle());
    assert!(t.points.len() > 4);
}

#[test]
fn guaranteed_and_practical_angles() {
    let s = with(|_| ());
    let loops = vec![s.spur_gear(21).unwrap(), hole(&s, 5.0)];
    for d in [20.7, 25.0, 30.0] {
        let t = triangulate(&loops, deg(d), s.ds, &[]).unwrap();
        check_triangulation(&loops, &t, deg(d));
    }
}

#[test]
fn gear_face() {
    let s = with(|_| ());
    for (teeth, bore) in [(21, 0.0), (21, 10.0), (40, 0.0), (40, 30.0)] {
        let mut loops = vec![s.spur_gear(teeth).unwrap()];
        if bore > 0.0 {
            loops.push(hole(&s, bore / 2.0));
        }
        let t = triangulate(&loops, min_angle(), s.ds, &[]).unwrap();
        check_triangulation(&loops, &t, min_angle());
        for (lp, refined) in loops.iter().zip(&t.boundary) {
            check_boundary_unchanged(lp, &t.points, refined);
        }
    }
}

#[test]
fn no_points_closer_than_ds() {
    let s = with(|_| ());
    let t = triangulate(&[s.spur_gear(21).unwrap()], min_angle(), s.ds, &[]).unwrap();
    let shortest = min_of(t.triangles.iter().flat_map(|&[a, b, c]| [(a, b), (b, c), (c, a)]).map(|(u, v)| {
        let (p, q) = (t.points[u], t.points[v]);
        (p.0 - q.0).hypot(p.1 - q.1)
    }));
    assert!(shortest >= s.ds * (1.0 - 1e-9));
}

/// The driven rotation sweep, with gear A's outline replaced by the boundary
/// of its refined face triangulation.
#[test]
fn refined_boundary_passes_the_rotating_test() {
    let s = system(params(20.0, 4.0, 0.001, 0.01));
    let teeth = s.min_spur_teeth;
    let t = triangulate(&[s.spur_gear(teeth).unwrap()], min_angle(), s.ds, &[]).unwrap();
    let refined: Vec<P> = t.boundary[0].iter().map(|&i| t.points[i]).collect();
    assert!(refined.len() >= s.spur_gear(teeth).unwrap().len());
    let results = sweep_spur_outlines(&s, &refined, teeth, teeth + 5, 20);
    assert_clean(&s, &results, "refined boundary");
}

// ------------------------------------------------------------------ the mesh

#[test]
fn mesh_problems_are_found() {
    let mut m = IndexedMesh::new();
    for p in [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
        m.add_vertex(p);
    }
    let faces = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]]; // a closed tetrahedron
    for f in faces {
        m.add_triangle(f[0], f[1], f[2]);
    }
    assert!(m.problems().is_empty());
    assert_eq!(m.euler_characteristic(), 2);
    assert_close(m.volume(), 1.0 / 6.0, 1e-12, "volume");

    let mut flipped = m.clone();
    flipped.triangles = vec![faces[0], faces[1], faces[2], [0, 2, 3]];
    assert!(flipped.problems().iter().any(|p| p.contains("same direction") || p.contains("no matching")));

    let mut open = m.clone();
    open.triangles = faces[..3].to_vec();
    assert!(open.problems().iter().any(|p| p.contains("no matching")));

    let mut inside_out = m.clone();
    inside_out.triangles = faces.iter().map(|&[a, b, c]| [a, c, b]).collect();
    assert!(inside_out.problems().iter().any(|p| p.contains("volume")));
}

#[test]
fn stl_file() {
    let m = spur_gear_mesh(&with(|_| ()), 21, 10.0, 0.0, 1, 0.0, 0.0).unwrap();
    let path = std::env::temp_dir().join(format!("gears2_test_{}.stl", std::process::id()));
    m.save_stl(&path).unwrap();
    let data = std::fs::read(&path).unwrap();
    std::fs::remove_file(&path).unwrap();
    let count = u32::from_le_bytes(data[80..84].try_into().unwrap()) as usize;
    assert_eq!(count, m.triangles.len());
    assert_eq!(data.len(), 84 + 50 * count);
}

// --------------------------------------------------------------- gear solids

fn perimeter(lp: &[P]) -> f64 {
    segment_lengths(lp).iter().sum()
}

fn min_z(m: &IndexedMesh) -> f64 {
    min_of(m.vertices.iter().map(|v| v[2]))
}

fn max_z(m: &IndexedMesh) -> f64 {
    max_of(m.vertices.iter().map(|v| v[2]))
}

#[test]
fn gear_solid() {
    for bore in [0.0, 10.0] {
        for hand in [1, -1] {
            for helix in [0.0, 30.0] {
                let s = with(|p| p.helix_angle = deg(helix));
                let width = 20.0;
                let m = spur_gear_mesh(&s, 21, width, bore, hand, 0.0, 0.0).unwrap();
                assert!(m.problems().is_empty(), "{:?}", &m.problems()[..1]);
                assert_eq!(m.euler_characteristic(), if bore > 0.0 { 0 } else { 2 });
                // Twisting doesn't change the volume: face area * width, up to the
                // walls' deviation from the true surface (perimeter * width * max_deviation).
                let outline = s.spur_gear(21).unwrap();
                let area = polygon_area(&outline) - if bore > 0.0 { polygon_area(&s.circle(bore / 2.0, true).unwrap()) } else { 0.0 };
                assert!((m.volume() - area * width).abs() <= perimeter(&outline) * width * s.max_deviation);
            }
        }
    }
}

fn round_key(x: f64, y: f64) -> (i64, i64) {
    ((x * 1e9).round() as i64, (y * 1e9).round() as i64)
}

/// The bore doesn't twist: its vertices are only on the two faces, at the bore
/// polygon's own positions.
#[test]
fn bore_wall_is_one_straight_layer() {
    let s = with(|_| ());
    let width = 20.0;
    let m = spur_gear_mesh(&s, 21, width, 10.0, 1, 0.0, 0.0).unwrap();
    assert!(m.problems().is_empty());
    let bore_pts = s.circle(5.0, true).unwrap();
    let bore: FxHashSet<(i64, i64)> = bore_pts.iter().map(|p| round_key(p.0, p.1)).collect();
    let on_bore: Vec<&[f64; 3]> = m.vertices.iter().filter(|v| bore.contains(&round_key(v[0], v[1]))).collect();
    let mut zs: Vec<f64> = on_bore.iter().map(|v| v[2]).collect();
    zs.sort_by(|a, b| a.partial_cmp(b).unwrap());
    zs.dedup();
    assert_eq!(zs, vec![0.0, width]);
    assert_eq!(on_bore.len(), 2 * bore.len());
    // Nothing else lies on the bore's circle between the faces.
    let r_bore = 5.0 / (PI / bore.len() as f64).cos();
    assert!(!m.vertices.iter().any(|v| 0.0 < v[2] && v[2] < width && v[0].hypot(v[1]) < r_bore + 1e-6));
}

#[test]
fn straight_gear_needs_no_layers() {
    let m = spur_gear_mesh(&with(|p| p.helix_angle = 0.0), 21, 20.0, 0.0, 1, 0.0, 0.0).unwrap();
    let mut zs: Vec<i64> = m.vertices.iter().map(|v| (v[2] * 1e9).round() as i64).collect();
    zs.sort();
    zs.dedup();
    assert_eq!(zs, vec![0, 20_000_000_000]);
}

fn point_triangle_distance(p: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    // Sample the triangle densely; fine for a test.
    let mut best = f64::INFINITY;
    for i in 0..=40 {
        let u = i as f64 / 40.0;
        let steps = ((41.0 * (1.0 - u)) as usize).max(2);
        for j in 0..steps {
            let v = (1.0 - u) * j as f64 / (steps - 1) as f64;
            let q = [0, 1, 2].map(|k| a[k] + u * (b[k] - a[k]) + v * (c[k] - a[k]));
            best = best.min(((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt());
        }
    }
    best
}

/// Between layers the walls are flat. Check, at the middle of every layer of
/// one half of a herringbone (a helix over half the width), how far the true
/// twisted surface is from the mesh:
/// - at the outermost boundary point (the helix it follows),
/// - at the middle of the longest boundary segment (the twisted quad).
#[test]
fn layers_follow_the_twist_within_max_deviation() {
    let s = with(|p| p.helix_angle = deg(35.0));
    let (teeth, width) = (21, 20.0);
    let t = triangulate(&[s.spur_gear(teeth).unwrap()], min_angle(), s.ds, &[]).unwrap();
    let pts = &t.points;
    let lp = &t.boundary[0];
    let length = |u: usize, v: usize| (pts[v].0 - pts[u].0).hypot(pts[v].1 - pts[u].1);
    let (mut longest, mut best) = ((lp[0], lp[1]), 0.0);
    for i in 0..lp.len() {
        let (u, v) = (lp[i], lp[(i + 1) % lp.len()]);
        if length(u, v) > best {
            best = length(u, v);
            longest = (u, v);
        }
    }
    let outer = *lp.iter().max_by(|&&a, &&b| pts[a].0.hypot(pts[a].1).partial_cmp(&pts[b].0.hypot(pts[b].1)).unwrap()).unwrap();
    let zs = layer_heights(&s, teeth, width / 2.0, best, None);
    let k = s.helix_angle.tan() / s.pitch_radius(teeth);
    let at = |p: P, z: f64| [p.0 * (k * z).cos() - p.1 * (k * z).sin(), p.0 * (k * z).sin() + p.1 * (k * z).cos(), z];
    for w in zs.windows(2) {
        let (z0, z1) = (w[0], w[1]);
        let zm = (z0 + z1) / 2.0;
        // Helix of the outermost point against the straight edge between layers.
        let (p0, p1, truth) = (at(pts[outer], z0), at(pts[outer], z1), at(pts[outer], zm));
        let mid = [0, 1, 2].map(|i| (p0[i] + p1[i]) / 2.0);
        assert!(((truth[0] - mid[0]).powi(2) + (truth[1] - mid[1]).powi(2) + (truth[2] - mid[2]).powi(2)).sqrt() <= s.max_deviation);
        // Twisted quad of the longest segment against its two triangles.
        let (u, v) = longest;
        let (a, b, c, d) = (at(pts[u], z0), at(pts[v], z0), at(pts[v], z1), at(pts[u], z1));
        let truth = at(((pts[u].0 + pts[v].0) / 2.0, (pts[u].1 + pts[v].1) / 2.0), zm);
        let dist = |p: [f64; 3], q: [f64; 3]| ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt();
        let tris = if dist(a, c) <= dist(b, d) { [(a, b, c), (a, c, d)] } else { [(a, b, d), (b, c, d)] };
        assert!(tris.iter().map(|&(x, y, z)| point_triangle_distance(truth, x, y, z)).fold(f64::INFINITY, f64::min) <= s.max_deviation);
    }
}

/// Every layer is the bottom outline turned by hand * twist(z), where the
/// twist rises at tan(helix_angle) / r_pitch per mm up to mid-height and falls
/// back: with hand = 1 the lower half turns counterclockwise going up, with
/// hand = -1 clockwise.
#[test]
fn hand() {
    let s = with(|_| ());
    let (teeth, width) = (21, 20.0);
    let k = s.helix_angle.tan() / s.pitch_radius(teeth);
    let key = |x: f64, y: f64, z: f64| ((x * 1e6).round() as i64, (y * 1e6).round() as i64, (z * 1e6).round() as i64);
    for hand in [1, -1] {
        let m = spur_gear_mesh(&s, teeth, width, 0.0, hand, 0.0, 0.0).unwrap();
        let bottom = s.spur_gear(teeth).unwrap(); // the outline's points are on every layer
        let mut levels: Vec<f64> = m.vertices.iter().map(|v| v[2]).collect();
        levels.sort_by(|a, b| a.partial_cmp(b).unwrap());
        levels.dedup();
        assert!(levels[0] == 0.0 && *levels.last().unwrap() == width);
        let all: FxHashSet<(i64, i64, i64)> = m.vertices.iter().map(|v| key(v[0], v[1], v[2])).collect();
        for &z in &levels {
            let turn = hand as f64 * k * z.min(width - z);
            for &(x, y) in &bottom {
                assert!(all.contains(&key(x * turn.cos() - y * turn.sin(), x * turn.sin() + y * turn.cos(), z)));
            }
        }
    }
}

#[test]
fn bad_hand() {
    assert!(spur_gear_mesh(&with(|_| ()), 21, 10.0, 0.0, 2, 0.0, 0.0).unwrap_err().0.contains("twist direction"));
}

#[test]
fn bore_must_fit() {
    assert!(spur_gear_mesh(&with(|_| ()), 21, 10.0, 40.0, 1, 0.0, 0.0).unwrap_err().0.contains("bore"));
}

// ---------------------------------------------------------- creases and ends

#[test]
fn crease_is_kept_with_triangles_on_both_sides() {
    let square = vec![(0.0, 0.0), (6.0, 0.0), (6.0, 6.0), (0.0, 6.0)];
    let crease = vec![(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)];
    let t = triangulate(&[square], min_angle(), 0.0, &[crease]).unwrap();
    assert_eq!(t.boundary.len(), 2);
    let total: f64 = t.triangles.iter().map(|&[a, b, c]| orient(t.points[a], t.points[b], t.points[c]) / 2.0).sum();
    assert_close(total, 36.0, 1e-9, "area"); // the inside of the crease is filled too
    let edges: FxHashSet<(usize, usize)> = t.triangles.iter().flat_map(|&[a, b, c]| [(a, b), (b, c), (c, a)]).collect();
    let lp = &t.boundary[1];
    for n in 0..lp.len() {
        let (u, v) = (lp[n], lp[(n + 1) % lp.len()]);
        assert!(edges.contains(&(u, v)) && edges.contains(&(v, u))); // triangles on both sides
    }
    assert!(min_angle_of(&t.points, &t.triangles) >= min_angle() - 1e-9);
}

/// Angle inside the material (degrees) at every edge shared by two triangles:
/// 180 - (angle between the normals) where the edge is convex, 180 + it where
/// concave.
fn interior_dihedrals(m: &IndexedMesh) -> Vec<f64> {
    let v = &m.vertices;
    let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let normals: Vec<[f64; 3]> = m
        .triangles
        .iter()
        .map(|&[a, b, c]| {
            let (u, w) = (sub(v[b], v[a]), sub(v[c], v[a]));
            let n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
            let l = dot(n, n).sqrt();
            [n[0] / l, n[1] / l, n[2] / l]
        })
        .collect();
    let mut owners: FxHashMap<(usize, usize), Vec<(usize, usize)>> = FxHashMap::default();
    for (t, &[a, b, c]) in m.triangles.iter().enumerate() {
        for (x, y, w) in [(a, b, c), (b, c, a), (c, a, b)] {
            owners.entry((x.min(y), x.max(y))).or_default().push((t, w));
        }
    }
    owners
        .iter()
        .map(|(&(u, _), pair)| {
            let ((t1, _), (t2, w2)) = (pair[0], pair[1]);
            let phi = dot(normals[t1], normals[t2]).clamp(-1.0, 1.0).acos().to_degrees();
            let convex = dot(sub(v[w2], v[u]), normals[t1]) < 0.0;
            if convex {
                180.0 - phi
            } else {
                180.0 + phi
            }
        })
        .collect()
}

#[test]
fn no_acute_edges_with_chamfered_ends() {
    for (teeth, helix, ends) in [(21, 30.0, (4.5, 4.5)), (21, 45.0, (2.0, 12.0)), (60, 30.0, (4.5, 4.5)), (60, 45.0, (12.0, 2.0))] {
        let s = with(|p| p.helix_angle = deg(helix));
        let m = spur_gear_mesh(&s, teeth, 20.0, 10.0, 1, ends.0, ends.1).unwrap();
        assert!(m.problems().is_empty());
        let sharpest = min_of(interior_dihedrals(&m));
        assert!(sharpest >= 89.0, "{} teeth, {} deg, ends {:?}: {}", teeth, helix, ends, sharpest);
    }
}

/// The check above can fail: without end zones a helical flank meets the end
/// face at 90 degrees minus about the helix angle.
#[test]
fn flat_ends_do_have_acute_edges() {
    let m = spur_gear_mesh(&with(|p| p.helix_angle = deg(30.0)), 21, 20.0, 0.0, 1, 0.0, 0.0).unwrap();
    assert!(min_of(interior_dihedrals(&m)) < 75.0);
}

#[test]
fn chamfered_gear_solid() {
    let s = with(|_| ());
    let (teeth, width) = (21, 20.0);
    for ends in [(4.5, 4.5), (0.0, 4.5), (2.0, 12.0)] {
        for bore in [0.0, 10.0] {
            for hand in [1, -1] {
                let m = spur_gear_mesh(&s, teeth, width, bore, hand, ends.0, ends.1).unwrap();
                assert!(m.problems().is_empty());
                assert_eq!(m.euler_characteristic(), if bore > 0.0 { 0 } else { 2 });
                assert!(min_z(&m) == 0.0);
                assert_close(max_z(&m), ends.0 + ends.1 + width, 1e-12, "height");
                // No edge shorter than ds.
                let shortest = min_of(m.triangles.iter().flat_map(|&[a, b, c]| [(a, b), (b, c), (c, a)]).map(|(u, v)| {
                    let (p, q) = (m.vertices[u], m.vertices[v]);
                    ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
                }));
                assert!(shortest >= s.ds * (1.0 - 1e-9));
                // Volume: the working zone, plus end zones somewhere between the
                // flat disk inside the fold circle and the whole outline, extruded.
                let outline = s.spur_gear(teeth).unwrap();
                let hole_area = if bore > 0.0 { polygon_area(&s.circle(bore / 2.0, true).unwrap()) } else { 0.0 };
                let area = polygon_area(&outline) - hole_area;
                let r_fold = s.pitch_radius(teeth) - s.height / 2.0 - s.clearance - s.module() / 4.0;
                let disk = PI * r_fold * r_fold - hole_area;
                let slack = perimeter(&outline) * width * s.max_deviation;
                let total_ends = ends.0 + ends.1;
                let vol = m.volume();
                assert!(area * width + disk * total_ends - slack <= vol && vol <= area * (width + total_ends) + slack);
            }
        }
    }
}

/// In the end zones nothing is turned (every vertex sits at a point of the
/// face triangulation), the outer faces are flat inside the fold circle, and
/// each outline column ends on the cone.
#[test]
fn end_zones_are_straight_and_cut_by_the_cone() {
    let s = with(|_| ());
    let (teeth, width, l) = (21, 20.0, 4.5);
    let m = spur_gear_mesh(&s, teeth, width, 0.0, 1, l, l).unwrap();
    let r_pitch = s.pitch_radius(teeth);
    let r_tip = r_pitch + s.height / 2.0;
    let r_fold = r_pitch - s.height / 2.0 - s.clearance - s.module() / 4.0;
    let t = triangulate(&[s.spur_gear(teeth).unwrap()], min_angle(), s.ds, &[s.circle(r_fold, false).unwrap()]).unwrap();
    let flat: FxHashSet<(i64, i64)> = t.points.iter().map(|p| round_key(p.0, p.1)).collect();
    assert!(m.vertices.iter().filter(|v| v[2] <= l + 1e-9 || v[2] >= l + width - 1e-9).all(|v| flat.contains(&round_key(v[0], v[1]))));
    assert!(m.vertices.iter().filter(|v| v[2] == 0.0).all(|v| v[0].hypot(v[1]) <= r_fold + 1e-9));
    let mut by_xy: FxHashMap<(i64, i64), f64> = FxHashMap::default();
    for v in &m.vertices {
        let e = by_xy.entry(round_key(v[0], v[1])).or_insert(f64::INFINITY);
        *e = e.min(v[2]);
    }
    for &i in &t.boundary[0] {
        let (x, y) = t.points[i];
        let r = x.hypot(y);
        let mut expected = if r <= r_fold { 0.0 } else { l.min(l * (r - r_fold) / (r_tip - r_fold)) };
        if l - expected < s.ds {
            expected = l; // snapped to the working edge
        }
        assert_close(by_xy[&round_key(x, y)], expected, 1e-9, "cone height");
    }
}

#[test]
fn bore_must_fit_inside_the_flat_end_face() {
    let s = with(|_| ());
    let r_root = s.pitch_radius(21) - s.height / 2.0 - s.clearance;
    let bore = 2.0 * (r_root - 0.2); // fits the root circle, not the fold circle
    spur_gear_mesh(&s, 21, 10.0, bore, 1, 0.0, 0.0).unwrap();
    assert!(spur_gear_mesh(&s, 21, 10.0, bore, 1, 0.0, 4.5).unwrap_err().0.contains("flat face"));
}

// --------------------------------------------------------------- ring gears

#[test]
fn ring_gear_solid() {
    let s = with(|_| ());
    let (teeth, width, rim) = (63, 20.0, 140.0);
    for ends in [(0.0, 0.0), (4.5, 4.5), (0.0, 4.5), (2.0, 12.0)] {
        for hand in [1, -1] {
            let m = ring_gear_mesh(&s, teeth, width, rim, hand, ends.0, ends.1).unwrap();
            assert!(m.problems().is_empty());
            assert_eq!(m.euler_characteristic(), 0); // an annulus
            assert!(min_z(&m) == 0.0);
            assert_close(max_z(&m), ends.0 + ends.1 + width, 1e-12, "height");
            // Volume: rim disk minus the toothed hole over the working width; in
            // the end zones between the flat annulus outside the fold and the full ring.
            let outline = s.ring_gear(teeth).unwrap();
            let disk = polygon_area(&s.circle(rim / 2.0, false).unwrap());
            let area = disk - polygon_area(&outline);
            let r_fold = s.pitch_radius(teeth) + s.height / 2.0 + s.clearance + s.module() / 4.0;
            let annulus = disk - PI * r_fold * r_fold;
            let slack = perimeter(&outline) * width * s.max_deviation;
            let total_ends = ends.0 + ends.1;
            let vol = m.volume();
            assert!(area * width + annulus * total_ends - slack <= vol && vol <= area * (width + total_ends) + slack);
        }
    }
}

#[test]
fn ring_has_no_acute_edges_with_chamfered_ends() {
    for (helix, ends) in [(30.0, (4.5, 4.5)), (45.0, (2.0, 12.0))] {
        let m = ring_gear_mesh(&with(|p| p.helix_angle = deg(helix)), 63, 20.0, 140.0, 1, ends.0, ends.1).unwrap();
        assert!(m.problems().is_empty());
        assert!(min_of(interior_dihedrals(&m)) >= 89.0);
    }
}

#[test]
fn flat_ring_has_acute_edges() {
    let m = ring_gear_mesh(&with(|p| p.helix_angle = deg(30.0)), 63, 20.0, 140.0, 1, 0.0, 0.0).unwrap();
    assert!(min_of(interior_dihedrals(&m)) < 75.0);
}

#[test]
fn rim_must_clear_the_teeth() {
    let s = with(|_| ());
    let r_root = s.pitch_radius(63) + s.height / 2.0 + s.clearance;
    let rim = 2.0 * (r_root + 0.3); // outside the root circle, inside the end zones' fold
    ring_gear_mesh(&s, 63, 10.0, rim, 1, 0.0, 0.0).unwrap();
    assert!(ring_gear_mesh(&s, 63, 10.0, rim, 1, 0.0, 4.5).unwrap_err().0.contains("flat face"));
    assert!(ring_gear_mesh(&s, 63, 10.0, 2.0 * r_root, 1, 0.0, 0.0).unwrap_err().0.contains("root circle"));
}

// ---------------------------------------------------------------- planetary

#[test]
fn planetary_parts_are_valid_and_combine() {
    let s = with(|_| ());
    let parts = planetary_gear_mesh(&s, 21, 21, 4, 20.0, 140.0, 10.0, 8.0, 1, 4.5, 4.5).unwrap();
    assert_eq!(parts.len(), 1 + 4 + 1);
    for m in &parts {
        assert!(m.problems().is_empty());
    }
    let whole = combined(&parts);
    assert!(whole.problems().is_empty());
    assert_eq!(whole.triangles.len(), parts.iter().map(|m| m.triangles.len()).sum::<usize>());
    assert_close(whole.volume(), parts.iter().map(|m| m.volume()).sum(), 1e-9, "volume");
}

/// At heights through the working width and the end zones, each part's cross
/// section is its outline turned by its own twist (planets and ring with the
/// opposite hand to the sun). Every mesh in every slice must be centered:
/// nothing overlaps, and the gaps along the line of action are the centered
/// gap, up to the outlines' sampling (at most 2 * max_deviation more).
#[test]
fn planetary_slices_mesh() {
    let s = with(|_| ());
    let (teeth_sun, teeth_planet, planets, width, l) = (21, 21, 4, 20.0, 4.5);
    let (teeth_ring, layout, ring_rotation) = s.planetary_layout(teeth_sun, teeth_planet, planets, 0.0).unwrap();
    let (sun_out, planet_out, ring_out) = (s.spur_gear(teeth_sun).unwrap(), s.spur_gear(teeth_planet).unwrap(), s.ring_gear(teeth_ring).unwrap());
    let (r_sun, r_ring) = (s.pitch_radius(teeth_sun), s.pitch_radius(teeth_ring));
    let window = 2.0 * (s.height + s.clearance);
    let centered = s.backlash * s.pressure_angle.cos() / 2.0;
    for hand in [1, -1] {
        let sun = EndZones::new(&s, teeth_sun, width, l, l, hand, false);
        let planet = EndZones::new(&s, teeth_planet, width, l, l, -hand, false);
        let ring = EndZones::new(&s, teeth_ring, width, l, l, -hand, true);
        for z in [0.5 * l, l, l + width / 4.0, l + width / 2.0, l + 0.9 * width, l + width + 0.5 * l] {
            for (k, &(center, rotation)) in layout.iter().enumerate() {
                let direction = 2.0 * PI * k as f64 / planets as f64;
                let planet_turn = rotation + planet.twist(z);
                for (other, place_other, pitch_r, is_ring) in [
                    (&sun_out, (sun.twist(z), (0.0, 0.0)), r_sun, false),
                    (&ring_out, (ring_rotation + ring.twist(z), (0.0, 0.0)), r_ring, true),
                ] {
                    let pitch_point = (pitch_r * direction.cos(), pitch_r * direction.sin());
                    let (_, _, gap, overlapping) = check_mesh(&planet_out, other, (planet_turn, center), place_other, pitch_point, window, is_ring);
                    assert!(!overlapping, "overlap at z={}", z);
                    assert!(centered - 1e-6 <= gap && gap <= centered + 2.0 * s.max_deviation, "gap {} at z={}", gap, z);
                }
            }
        }
    }
}
