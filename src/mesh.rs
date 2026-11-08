//! Meshes: closed triangle meshes of gears, written as STL.
//!
//! All lengths are in mm and all angles in radians.
//!
//! - IndexedMesh: one shared vertex list plus triangles as triples of vertex
//!   indices. Neighbouring triangles share vertices by index, so edges match
//!   exactly. `problems()` checks that the mesh is closed and consistently
//!   oriented, with no degenerate triangles and positive volume.
//! - triangulate(): a constrained Delaunay triangulation of a polygon (with
//!   holes), built by Bowyer-Watson insertion and refined with Ruppert's
//!   algorithm so that every angle is at least `min_angle`.
//! - spur_gear_mesh(), ring_gear_mesh(), planetary_gear_mesh(): herringbone
//!   gears from a GearSystem, optionally with chamfered end zones.
//!
//! # Triangulation
//!
//! Bowyer-Watson inserts points one at a time: the triangles whose
//! circumcircles contain the new point form a hole (the cavity), which is
//! removed and filled by connecting the point to the hole's boundary. The
//! cavity never grows across a constrained (boundary) edge, so those stay in
//! place.
//!
//! 1. Insert the boundary points into a large "super triangle".
//! 2. Recover the boundary: a boundary segment that is not yet an edge is
//!    split at its midpoint (and the halves checked again) until every piece
//!    is an edge. Then mark them as constrained.
//! 3. Ruppert's refinement, until nothing changes:
//!    - a boundary segment is encroached when a vertex lies inside its
//!      diametral circle (the circle with the segment as diameter); split it
//!      at its midpoint;
//!    - a triangle with an angle below min_angle is bad: insert its
//!      circumcenter, unless that would encroach a segment, in which case
//!      split the segment instead.
//!
//! Ruppert proved this terminates with every angle at least
//! arcsin(1 / (2 sqrt 2)) ~ 20.7 degrees when no two boundary segments meet at
//! less than 60 degrees (gear outlines, bores and rims qualify); in practice it
//! reaches 25-30 degrees. Splitting a boundary segment puts a new vertex on
//! the existing straight edge, so the boundary's shape never changes.

use crate::gears::{GearError, GearSystem, Result, P};
use rustc_hash::{FxHashMap, FxHashSet};
use std::f64::consts::PI;

/// The angle every face triangle is refined to.
pub const DEFAULT_MIN_ANGLE: f64 = 25.0 * PI / 180.0;

fn internal<T>(message: &str) -> Result<T> {
    Err(GearError(format!("Internal error: {}.", message)))
}

// ------------------------------------------------------------- indexed mesh

/// Vertices (x, y, z) and triangles (i, j, k) counterclockwise seen from
/// outside, so normals point out.
#[derive(Debug, Clone, Default)]
pub struct IndexedMesh {
    pub vertices: Vec<[f64; 3]>,
    pub triangles: Vec<[usize; 3]>,
}

impl IndexedMesh {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn add_vertex(&mut self, p: [f64; 3]) -> usize {
        self.vertices.push(p);
        self.vertices.len() - 1
    }

    pub fn add_triangle(&mut self, i: usize, j: usize, k: usize) {
        self.triangles.push([i, j, k]);
    }

    fn normal(&self, t: [usize; 3]) -> [f64; 3] {
        let (a, b, c) = (self.vertices[t[0]], self.vertices[t[1]], self.vertices[t[2]]);
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
    }

    /// Enclosed volume (divergence theorem); positive when normals point out.
    pub fn volume(&self) -> f64 {
        let mut total = 0.0;
        for t in &self.triangles {
            let (a, b, c) = (self.vertices[t[0]], self.vertices[t[1]], self.vertices[t[2]]);
            total += a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]);
        }
        total / 6.0
    }

    pub fn euler_characteristic(&self) -> i64 {
        let mut edges = FxHashSet::default();
        let mut used = FxHashSet::default();
        for t in &self.triangles {
            for k in 0..3 {
                let (u, v) = (t[k], t[(k + 1) % 3]);
                edges.insert((u.min(v), u.max(v)));
                used.insert(u);
            }
        }
        used.len() as i64 - edges.len() as i64 + self.triangles.len() as i64
    }

    /// Readable list of defects (empty for a good mesh):
    /// - every directed edge must appear exactly once, and its reverse exactly
    ///   once (closed, manifold, consistently oriented);
    /// - no triangle may have (nearly) zero area or repeat a vertex;
    /// - the volume must be positive (normals point outward).
    pub fn problems(&self) -> Vec<String> {
        let min_area = 1e-12;
        let mut out = Vec::new();
        let mut directed: FxHashMap<(usize, usize), usize> = FxHashMap::default();
        let mut order = Vec::new();
        for &t in &self.triangles {
            let [i, j, k] = t;
            if i == j || j == k || i == k {
                out.push(format!("triangle {:?} repeats a vertex", t));
                continue;
            }
            let n = self.normal(t);
            if (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt() / 2.0 < min_area {
                out.push(format!("triangle {:?} has (nearly) zero area", t));
            }
            for e in [(i, j), (j, k), (k, i)] {
                let count = directed.entry(e).or_insert(0);
                if *count == 0 {
                    order.push(e);
                }
                *count += 1;
            }
        }
        for (i, j) in order {
            let count = directed[&(i, j)];
            if count > 1 {
                out.push(format!("edge {}->{} is used by {} triangles in the same direction", i, j, count));
            }
            if !directed.contains_key(&(j, i)) {
                out.push(format!("edge {}->{} has no matching edge {}->{} (open or flipped)", i, j, j, i));
            }
        }
        if out.is_empty() && self.volume() <= 0.0 {
            out.push("the volume is not positive (normals point inward)".to_string());
        }
        out
    }

    /// The mesh as a binary STL file.
    pub fn stl_bytes(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(84 + 50 * self.triangles.len());
        out.extend_from_slice(&[0u8; 80]);
        out.extend_from_slice(&(self.triangles.len() as u32).to_le_bytes());
        for &t in &self.triangles {
            let n = self.normal(t);
            let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            let length = if length == 0.0 { 1.0 } else { length };
            for c in n {
                out.extend_from_slice(&((c / length) as f32).to_le_bytes());
            }
            for i in t {
                for c in self.vertices[i] {
                    out.extend_from_slice(&(c as f32).to_le_bytes());
                }
            }
            out.extend_from_slice(&[0u8; 2]);
        }
        out
    }

    /// Write a binary STL.
    pub fn save_stl(&self, path: impl AsRef<std::path::Path>) -> std::io::Result<()> {
        std::fs::write(path, self.stl_bytes())
    }
}

// ------------------------------------------------------------ triangulation

/// Twice the signed area of abc: positive when counterclockwise.
pub fn orient(a: P, b: P, c: P) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

/// Positive when d is inside the circumcircle of the counterclockwise
/// triangle abc.
pub fn incircle(a: P, b: P, c: P, d: P) -> f64 {
    let (adx, ady) = (a.0 - d.0, a.1 - d.1);
    let (bdx, bdy) = (b.0 - d.0, b.1 - d.1);
    let (cdx, cdy) = (c.0 - d.0, c.1 - d.1);
    (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy) - (bdx * bdx + bdy * bdy) * (adx * cdy - cdx * ady)
        + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady)
}

fn circumcenter(a: P, b: P, c: P) -> P {
    let (bx, by) = (b.0 - a.0, b.1 - a.1);
    let (cx, cy) = (c.0 - a.0, c.1 - a.1);
    let d = 2.0 * (bx * cy - by * cx);
    let ux = (cy * (bx * bx + by * by) - by * (cx * cx + cy * cy)) / d;
    let uy = (bx * (cx * cx + cy * cy) - cx * (bx * bx + by * by)) / d;
    (a.0 + ux, a.1 + uy)
}

/// Even-odd test against all loops (outer boundary and holes).
fn inside_loops(p: P, loops: &[Vec<P>]) -> bool {
    let mut inside = false;
    for lp in loops {
        let n = lp.len();
        for i in 0..n {
            let ((x1, y1), (x2, y2)) = (lp[i], lp[(i + 1) % n]);
            if (y1 > p.1) != (y2 > p.1) && p.0 < x1 + (p.1 - y1) * (x2 - x1) / (y2 - y1) {
                inside = !inside;
            }
        }
    }
    inside
}

/// Smallest angle (radians) of any triangle.
pub fn min_angle_of(points: &[P], triangles: &[[usize; 3]]) -> f64 {
    let mut best = PI;
    for t in triangles {
        let p = [points[t[0]], points[t[1]], points[t[2]]];
        let mut sides: Vec<f64> = (0..3).map(|k| (p[(k + 1) % 3].0 - p[k].0).hypot(p[(k + 1) % 3].1 - p[k].1)).collect();
        sides.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let area2 = orient(p[0], p[1], p[2]).abs();
        // sin(angle opposite the shortest side) = 2 * area / (the other two sides)
        best = best.min((area2 / (sides[1] * sides[2])).min(1.0).asin());
    }
    best
}

fn key(a: usize, b: usize) -> (usize, usize) {
    (a.min(b), a.max(b))
}

/// Mutable constrained triangulation used by triangulate(). Triangles live in
/// a vector (None once removed), so iterating them is in creation order and
/// fully deterministic.
struct Triangulation {
    points: Vec<P>,
    tris: Vec<Option<[usize; 3]>>,
    interior: Vec<bool>,
    alive: usize,
    /// Directed edge (a, b) -> the triangle that has it.
    edge: FxHashMap<(usize, usize), usize>,
    constrained: FxHashSet<(usize, usize)>,
    last: usize,
}

type Cavity = (Vec<usize>, Vec<(usize, usize, usize)>);

impl Triangulation {
    fn new(all: &[P]) -> Self {
        let (mut x0, mut x1, mut y0, mut y1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for p in all {
            x0 = x0.min(p.0);
            x1 = x1.max(p.0);
            y0 = y0.min(p.1);
            y1 = y1.max(p.1);
        }
        let (cx, cy) = ((x0 + x1) / 2.0, (y0 + y1) / 2.0);
        let size = (x1 - x0).max(y1 - y0).max(1.0);
        let big = 50.0 * size;
        let mut t = Triangulation {
            points: vec![(cx - big, cy - big), (cx + big, cy - big), (cx, cy + big)],
            tris: Vec::new(),
            interior: Vec::new(),
            alive: 0,
            edge: FxHashMap::default(),
            constrained: FxHashSet::default(),
            last: 0,
        };
        t.last = t.add_tri(0, 1, 2, false);
        t
    }

    fn add_tri(&mut self, a: usize, b: usize, c: usize, interior: bool) -> usize {
        let t = self.tris.len();
        self.tris.push(Some([a, b, c]));
        self.interior.push(interior);
        self.alive += 1;
        self.edge.insert((a, b), t);
        self.edge.insert((b, c), t);
        self.edge.insert((c, a), t);
        t
    }

    fn remove_tri(&mut self, t: usize) {
        let [a, b, c] = self.tris[t].take().expect("removing a removed triangle");
        self.alive -= 1;
        for e in [(a, b), (b, c), (c, a)] {
            if self.edge.get(&e) == Some(&t) {
                self.edge.remove(&e);
            }
        }
    }

    fn tri(&self, t: usize) -> [usize; 3] {
        self.tris[t].expect("a removed triangle")
    }

    fn is_constrained(&self, a: usize, b: usize) -> bool {
        self.constrained.contains(&key(a, b))
    }

    /// Triangle containing p, found by walking from `start`. Returns
    /// Ok((Some(triangle), None)), or Ok((None, Some((a, b)))) if the walk
    /// would have to cross the constrained edge ab (only when
    /// cross_constraints is false).
    fn locate(&self, p: P, start: Option<usize>, cross_constraints: bool) -> Result<(Option<usize>, Option<(usize, usize)>)> {
        let mut t = match start {
            Some(s) if self.tris[s].is_some() => s,
            _ => self.last,
        };
        if self.tris[t].is_none() {
            t = (0..self.tris.len()).find(|&i| self.tris[i].is_some()).unwrap();
        }
        let pts = &self.points;
        for _ in 0..4 * self.alive + 10 {
            let [a, b, c] = self.tri(t);
            let mut moved = false;
            for (u, v) in [(a, b), (b, c), (c, a)] {
                if orient(pts[u], pts[v], p) < 0.0 {
                    if !cross_constraints && self.is_constrained(u, v) {
                        return Ok((None, Some((u, v))));
                    }
                    match self.edge.get(&(v, u)) {
                        Some(&n) => {
                            t = n;
                            moved = true;
                        }
                        None => return internal("point outside the triangulation"),
                    }
                    break;
                }
            }
            if !moved {
                return Ok((Some(t), None));
            }
        }
        // The walk can cycle on degenerate input; fall back to a search.
        for (i, tri) in self.tris.iter().enumerate() {
            if let Some([a, b, c]) = *tri {
                if orient(pts[a], pts[b], p).min(orient(pts[b], pts[c], p)).min(orient(pts[c], pts[a], p)) >= 0.0 {
                    return Ok((Some(i), None));
                }
            }
        }
        internal("couldn't locate a point")
    }

    /// Triangles whose circumcircles contain p, grown from `seeds` without
    /// crossing constrained edges (except `opened`, a segment being split).
    /// Returns (triangles, boundary edges [(u, v, owner)]) with the cavity made
    /// star-shaped from p, or None if it can't be.
    fn cavity(&self, p: P, seeds: &[usize], opened: Option<(usize, usize)>) -> Option<Cavity> {
        let pts = &self.points;
        let mut in_cav: FxHashSet<usize> = seeds.iter().copied().collect();
        let mut cav: Vec<usize> = seeds.to_vec();
        let mut stack: Vec<usize> = seeds.to_vec();
        while let Some(t) = stack.pop() {
            let [a, b, c] = self.tri(t);
            for (u, v) in [(a, b), (b, c), (c, a)] {
                if self.is_constrained(u, v) && Some(key(u, v)) != opened {
                    continue;
                }
                let Some(&n) = self.edge.get(&(v, u)) else { continue };
                if in_cav.contains(&n) {
                    continue;
                }
                let [x, y, z] = self.tri(n);
                if incircle(pts[x], pts[y], pts[z], p) > 0.0 {
                    in_cav.insert(n);
                    cav.push(n);
                    stack.push(n);
                }
            }
        }
        // Every boundary edge must face p, or the new triangles would overlap.
        // Rounding error can break that; drop the offending triangles.
        loop {
            let mut boundary = Vec::new();
            for &t in &cav {
                let [a, b, c] = self.tri(t);
                for (u, v) in [(a, b), (b, c), (c, a)] {
                    let inside = self.edge.get(&(v, u)).is_some_and(|n| in_cav.contains(n));
                    if !inside {
                        boundary.push((u, v, t));
                    }
                }
            }
            let bad: FxHashSet<usize> = boundary
                .iter()
                .filter(|&&(u, v, _)| orient(pts[u], pts[v], p) <= 0.0)
                .map(|&(_, _, t)| t)
                .collect();
            if bad.is_empty() {
                return Some((cav, boundary));
            }
            if seeds.iter().any(|s| bad.contains(s)) {
                return None;
            }
            cav.retain(|t| !bad.contains(t));
            for t in &bad {
                in_cav.remove(t);
            }
        }
    }

    /// Replace the cavity by triangles fanning from the new point p.
    fn insert(&mut self, p: P, cavity: Cavity) -> (usize, Vec<usize>) {
        let (cav, boundary) = cavity;
        self.points.push(p);
        let i = self.points.len() - 1;
        let labels: FxHashMap<usize, bool> = cav.iter().map(|&t| (t, self.interior[t])).collect();
        for &t in &cav {
            self.remove_tri(t);
        }
        let new: Vec<usize> = boundary.iter().map(|&(u, v, t)| self.add_tri(u, v, i, labels[&t])).collect();
        self.last = new[0];
        (i, new)
    }
}

/// Output of triangulate(): points, counterclockwise triangles, and for each
/// input loop and then each crease, the indices of its points in order,
/// including the points added on it.
pub struct Triangulated {
    pub points: Vec<P>,
    pub triangles: Vec<[usize; 3]>,
    pub boundary: Vec<Vec<usize>>,
}

const NONE: usize = usize::MAX;

/// Constrained Delaunay triangulation of a polygon with holes, refined with
/// Ruppert's algorithm.
///
/// - loops: closed polylines, the outer boundary counterclockwise, holes
///   clockwise.
/// - min_angle: refine until every angle is at least this (the guarantee is
///   ~20.7 degrees; up to ~30 usually works).
/// - ds: boundary segments shorter than 2 * ds are never split, so no new
///   point lands closer than ds to another along the boundary.
/// - creases: closed polylines strictly inside the polygon whose segments
///   must also be triangle edges (fold lines), with triangles on both sides.
///   They are fixed edges like the boundary (recovered, split when
///   encroached) but don't bound the polygon.
pub fn triangulate(loops: &[Vec<P>], min_angle: f64, ds: f64, creases: &[Vec<P>]) -> Result<Triangulated> {
    triangulate_with_limit(loops, min_angle, ds, creases, 200_000)
}

pub fn triangulate_with_limit(loops: &[Vec<P>], min_angle: f64, ds: f64, creases: &[Vec<P>], max_points: usize) -> Result<Triangulated> {
    triangulate_sized(loops, min_angle, ds, creases, max_points, &|_| f64::INFINITY)
}

/// triangulate() that also refines every triangle whose circumradius is above
/// max_radius(centroid) (for faces that are later bent onto a curved surface,
/// where a triangle's size sets how far it strays from the surface).
pub fn triangulate_sized(
    loops: &[Vec<P>],
    min_angle: f64,
    ds: f64,
    creases: &[Vec<P>],
    max_points: usize,
    max_radius: &dyn Fn(P) -> f64,
) -> Result<Triangulated> {
    let fixed: Vec<&Vec<P>> = loops.iter().chain(creases.iter()).collect();
    let all: Vec<P> = fixed.iter().flat_map(|l| l.iter().copied()).collect();
    let mut tr = Triangulation::new(&all);
    // succ[v]: the next fixed vertex along its loop or crease (NONE if none).
    let mut succ: Vec<usize> = vec![NONE; 3];
    let mut starts = Vec::new();
    for lp in &fixed {
        let mut ids = Vec::new();
        for &p in lp.iter() {
            let (t, _) = tr.locate(p, None, true)?;
            let Some(cav) = tr.cavity(p, &[t.unwrap()], None) else {
                return internal("couldn't insert a boundary point");
            };
            let (i, _) = tr.insert(p, cav);
            succ.resize(tr.points.len(), NONE);
            ids.push(i);
        }
        starts.push(ids[0]);
        for k in 0..ids.len() {
            succ[ids[k]] = ids[(k + 1) % ids.len()];
        }
    }
    let segments = |succ: &Vec<usize>| -> Vec<(usize, usize)> {
        (0..succ.len()).filter(|&a| succ[a] != NONE).map(|a| (a, succ[a])).collect()
    };
    let split_point = |pts: &Vec<P>, a: usize, b: usize| ((pts[a].0 + pts[b].0) / 2.0, (pts[a].1 + pts[b].1) / 2.0);
    let length = |pts: &Vec<P>, a: usize, b: usize| (pts[b].0 - pts[a].0).hypot(pts[b].1 - pts[a].1);

    // Recover the boundary: split missing segments until every piece is an edge.
    let mut pending = segments(&succ);
    while let Some((a, b)) = pending.pop() {
        if tr.edge.contains_key(&(a, b)) || tr.edge.contains_key(&(b, a)) {
            continue;
        }
        if length(&tr.points, a, b) < 2.0 * ds {
            return internal("a boundary segment is missing and too short to split");
        }
        let m = split_point(&tr.points, a, b);
        let (t, _) = tr.locate(m, None, true)?;
        let Some(cav) = tr.cavity(m, &[t.unwrap()], None) else {
            return internal("couldn't split a boundary segment");
        };
        let (i, _) = tr.insert(m, cav);
        succ.resize(tr.points.len(), NONE);
        succ[a] = i;
        succ[i] = b;
        pending.push((a, i));
        pending.push((i, b));
    }
    for (a, b) in segments(&succ) {
        tr.constrained.insert(key(a, b));
    }

    // Inside or outside the polygon: each triangle's centroid against the loops.
    for t in 0..tr.tris.len() {
        if let Some([a, b, c]) = tr.tris[t] {
            let pts = &tr.points;
            let centroid = ((pts[a].0 + pts[b].0 + pts[c].0) / 3.0, (pts[a].1 + pts[b].1 + pts[c].1) / 3.0);
            tr.interior[t] = a.min(b).min(c) > 2 && inside_loops(centroid, loops);
        }
    }

    // ------------------------------------------------------------- Ruppert
    let ratio = 1.0 / (2.0 * min_angle.sin()); // bad: circumradius / shortest edge > ratio

    let segment_of = |succ: &Vec<usize>, a: usize, b: usize| -> Option<(usize, usize)> {
        if a < succ.len() && succ[a] == b {
            Some((a, b))
        } else if b < succ.len() && succ[b] == a {
            Some((b, a))
        } else {
            None
        }
    };
    // Is an interior-side apex inside the segment's diametral circle?
    let encroached = |tr: &Triangulation, (a, b): (usize, usize)| -> bool {
        for (u, v) in [(a, b), (b, a)] {
            let Some(&t) = tr.edge.get(&(u, v)) else { continue };
            if !tr.interior[t] {
                continue;
            }
            let c = tr.tri(t).into_iter().find(|&x| x != u && x != v).unwrap();
            let (pa, pb, pc) = (tr.points[a], tr.points[b], tr.points[c]);
            if (pc.0 - pa.0) * (pc.0 - pb.0) + (pc.1 - pa.1) * (pc.1 - pb.1) < 0.0 {
                return true;
            }
        }
        false
    };
    let bad = |tr: &Triangulation, t: usize| -> bool {
        let Some([a, b, c]) = tr.tris[t] else { return false };
        if !tr.interior[t] {
            return false;
        }
        let (a, b, c) = (tr.points[a], tr.points[b], tr.points[c]);
        let shortest = (b.0 - a.0).hypot(b.1 - a.1).min((c.0 - b.0).hypot(c.1 - b.1)).min((a.0 - c.0).hypot(a.1 - c.1));
        let cc = circumcenter(a, b, c);
        let radius = (cc.0 - a.0).hypot(cc.1 - a.1);
        radius > ratio * shortest || radius > max_radius(((a.0 + b.0 + c.0) / 3.0, (a.1 + b.1 + c.1) / 3.0))
    };

    let mut unsplittable: FxHashSet<(usize, usize)> = FxHashSet::default();
    let mut enc_queue: Vec<(usize, usize)> = segments(&succ).into_iter().filter(|&s| encroached(&tr, s)).collect();
    let mut bad_queue: Vec<usize> = (0..tr.tris.len()).filter(|&t| bad(&tr, t)).collect();

    let after_insert = |tr: &Triangulation, succ: &Vec<usize>, new: &[usize], enc_queue: &mut Vec<(usize, usize)>, bad_queue: &mut Vec<usize>| {
        for &t in new {
            let [a, b, c] = tr.tri(t);
            for (u, v) in [(a, b), (b, c), (c, a)] {
                if let Some(seg) = segment_of(succ, u, v) {
                    if encroached(tr, seg) {
                        enc_queue.push(seg);
                    }
                }
            }
            if bad(tr, t) {
                bad_queue.push(t);
            }
        }
    };

    while !enc_queue.is_empty() || !bad_queue.is_empty() {
        if tr.points.len() > max_points {
            return internal("refinement didn't finish (too many points)");
        }
        if let Some((a, b)) = enc_queue.pop() {
            // Split the segment ab at its midpoint (unless already split, or too short).
            if succ[a] != b || unsplittable.contains(&(a, b)) {
                continue;
            }
            if length(&tr.points, a, b) < 2.0 * ds {
                unsplittable.insert((a, b));
                continue;
            }
            let m = split_point(&tr.points, a, b);
            let seeds: Vec<usize> = [tr.edge.get(&(a, b)), tr.edge.get(&(b, a))].into_iter().flatten().copied().collect();
            let Some(cav) = tr.cavity(m, &seeds, Some(key(a, b))) else {
                return internal("couldn't split a boundary segment");
            };
            tr.constrained.remove(&key(a, b));
            let (i, new) = tr.insert(m, cav);
            succ.resize(tr.points.len(), NONE);
            succ[a] = i;
            succ[i] = b;
            tr.constrained.insert(key(a, i));
            tr.constrained.insert(key(i, b));
            after_insert(&tr, &succ, &new, &mut enc_queue, &mut bad_queue);
            continue;
        }
        let t = bad_queue.pop().unwrap();
        if !bad(&tr, t) {
            continue;
        }
        let [a, b, c] = tr.tri(t);
        let cc = circumcenter(tr.points[a], tr.points[b], tr.points[c]);
        let (place, blocked) = tr.locate(cc, Some(t), false)?;
        if let Some((u, v)) = blocked {
            // The circumcenter lies beyond a boundary segment: that segment is
            // encroached by it. Split it and try the triangle again.
            if let Some(seg) = segment_of(&succ, u, v) {
                if !unsplittable.contains(&seg) {
                    enc_queue.push(seg);
                    bad_queue.push(t);
                }
            }
            continue;
        }
        let place = place.unwrap();
        let Some((cav, boundary)) = tr.cavity(cc, &[place], None) else { continue };
        let mut hit = Vec::new();
        for &(u, v, _) in &boundary {
            if let Some(seg) = segment_of(&succ, u, v) {
                let (pu, pv) = (tr.points[u], tr.points[v]);
                if (cc.0 - pu.0) * (cc.0 - pv.0) + (cc.1 - pu.1) * (cc.1 - pv.1) < 0.0 {
                    hit.push(seg);
                }
            }
        }
        if !hit.is_empty() {
            let splittable: Vec<(usize, usize)> = hit
                .into_iter()
                .filter(|s| !unsplittable.contains(s) && length(&tr.points, s.0, s.1) >= 2.0 * ds)
                .collect();
            if !splittable.is_empty() {
                enc_queue.extend(splittable);
                bad_queue.push(t);
            }
            continue; // never insert a point that encroaches a segment
        }
        if !tr.interior[place] {
            continue;
        }
        let (_, new) = tr.insert(cc, (cav, boundary));
        after_insert(&tr, &succ, &new, &mut enc_queue, &mut bad_queue);
    }

    // --------------------------------------------------------------- output
    let mut used = vec![false; tr.points.len()];
    for (t, tri) in tr.tris.iter().enumerate() {
        if let Some(tri) = tri {
            if tr.interior[t] {
                for &i in tri {
                    used[i] = true;
                }
            }
        }
    }
    let mut remap = vec![NONE; tr.points.len()];
    let mut points = Vec::new();
    for (i, &u) in used.iter().enumerate() {
        if u {
            remap[i] = points.len();
            points.push(tr.points[i]);
        }
    }
    let triangles = tr
        .tris
        .iter()
        .enumerate()
        .filter_map(|(t, tri)| tri.filter(|_| tr.interior[t]).map(|[a, b, c]| [remap[a], remap[b], remap[c]]))
        .collect();
    let mut boundary = Vec::new();
    for &start in &starts {
        let mut lp = Vec::new();
        let mut v = start;
        loop {
            lp.push(remap[v]);
            v = succ[v];
            if v == start {
                break;
            }
        }
        boundary.push(lp);
    }
    Ok(Triangulated { points, triangles, boundary })
}

// -------------------------------------------------------------------- gears

/// Heights (from 0 to span) of the layers a twisted wall needs.
///
/// A boundary point at radius R on a gear twisting at rate k (radians per mm;
/// k = tan(helix_angle) / r_pitch, so the tooth slants at the helix angle at
/// the pitch circle) moves along a helix of curvature R k^2 / (1 + R^2 k^2).
/// Between layers the wall is straight, so:
/// - each vertical edge strays from its helix by about L^2 * curvature / 8,
///   for an edge of length L = dz * sqrt(1 + R^2 k^2);
/// - each wall quad is twisted: its top edge is its bottom edge turned by
///   k dz, so splitting it into two triangles strays from the twisted surface
///   by about segment_length * k * dz / 4.
///
/// Both must stay within max_deviation (using r_max, the outermost radius of
/// the twisted outline, by default a spur gear's tip radius, and the longest
/// boundary segment). Layers are also at least ds apart.
pub fn layer_heights(system: &GearSystem, teeth: usize, span: f64, max_segment: f64, r_max: Option<f64>) -> Vec<f64> {
    let k = system.helix_angle.tan() / system.pitch_radius(teeth);
    if k == 0.0 {
        return vec![0.0, span];
    }
    let r = r_max.unwrap_or(system.pitch_radius(teeth) + system.height / 2.0);
    let eps = system.max_deviation;
    let curvature = r * k * k / (1.0 + r * r * k * k);
    let dz_helix = (8.0 * eps / curvature).sqrt() / (1.0 + r * r * k * k).sqrt();
    let dz_twist = 4.0 * eps / (max_segment * k);
    let dz = dz_helix.min(dz_twist);
    let n = ((span / dz - 1e-9).ceil() as i64).max(1);
    let n = n.min((span / system.ds + 1e-9).floor() as i64).max(1) as usize;
    (0..=n).map(|i| span * i as f64 / n as f64).collect()
}

/// Geometry of a herringbone gear along z, with optional chamfered end zones.
///
/// z runs from 0 to total = end_bottom + width + end_top. The working zone
/// [end_bottom, end_bottom + width] is a plain herringbone: the twist (as an
/// arc length on the pitch circle) is 0 at its edges and rises at
/// tan(helix_angle) toward its middle.
///
/// An end zone of length L is the outline at the working edge extruded
/// straight (no twist), cut by a cone from the tip circle at the working edge
/// to the fold circle at the outer face, beyond which the outer face is flat:
/// - spur gear: fold r_root - module / 4 (just inside the root circle); the
///   material is inside the cone;
/// - ring gear: fold r_root + module / 4 (just outside its root circle); the
///   material is outside the cone.
///
/// In both cases the cone reaches radius R at height L d from the outer face,
/// d = (R - r_fold) / (r_tip - r_fold), for 0 < d < 1.
///
/// With no twist where the cone cuts, a flank meets the cone at
/// 90 degrees + asin(sin(a) sin(cone angle)) >= 90 degrees (a the pressure
/// angle there), for any zone length. Every gear's twist is 0 in its end
/// zones, so meshing gears stay matched slice by slice; they must use the
/// same width and end zones. At the working edge the flank bends by the local
/// helix angle (an obtuse crease).
#[derive(Debug, Clone)]
pub struct EndZones {
    pub ds: f64,
    pub width: f64,
    pub ends: (f64, f64),
    pub z_lo: f64,
    pub z_hi: f64,
    pub total: f64,
    pub r_pitch: f64,
    pub r_tip: f64,
    pub r_root: f64,
    pub r_fold: f64,
    /// Twist rate in the working zone, radians per mm (signed by hand).
    pub k: f64,
}

impl EndZones {
    pub fn new(s: &GearSystem, teeth: usize, width: f64, end_bottom: f64, end_top: f64, hand: i32, ring: bool) -> Self {
        let r_pitch = s.pitch_radius(teeth);
        let (r_tip, r_root, r_fold) = if ring {
            let r_root = r_pitch + s.height / 2.0 + s.clearance;
            (r_pitch - s.height / 2.0, r_root, r_root + s.module() / 4.0)
        } else {
            let r_root = r_pitch - s.height / 2.0 - s.clearance;
            (r_pitch + s.height / 2.0, r_root, r_root - s.module() / 4.0)
        };
        EndZones {
            ds: s.ds,
            width,
            ends: (end_bottom, end_top),
            z_lo: end_bottom,
            z_hi: end_bottom + width,
            total: end_bottom + width + end_top,
            r_pitch,
            r_tip,
            r_root,
            r_fold,
            k: hand as f64 * s.helix_angle.tan() / r_pitch,
        }
    }

    /// Turn (radians) of the outline at height z.
    pub fn twist(&self, z: f64) -> f64 {
        if z <= self.z_lo || z >= self.z_hi {
            return 0.0;
        }
        self.k * (z - self.z_lo).min(self.z_hi - z)
    }

    /// Height of the cone above the outer face at radius r, or None where the
    /// face is flat.
    fn cut(&self, r: f64, l: f64) -> Option<f64> {
        if l == 0.0 {
            return None;
        }
        let d = (r - self.r_fold) / (self.r_tip - self.r_fold);
        if d <= 0.0 {
            None
        } else {
            Some(l * d.min(1.0))
        }
    }

    /// Lowest z with material at radius r. Within ds of the working edge it
    /// snaps to the edge, so no wall gets a sliver: the cone is a design
    /// surface (like the fillets), not a working one.
    pub fn z_bottom(&self, r: f64) -> f64 {
        match self.cut(r, self.ends.0) {
            None => 0.0,
            Some(h) if self.z_lo - h < self.ds => self.z_lo,
            Some(h) => h,
        }
    }

    pub fn z_top(&self, r: f64) -> f64 {
        match self.cut(r, self.ends.1) {
            None => self.total,
            Some(h) => {
                let z = self.total - h;
                if z - self.z_hi < self.ds {
                    self.z_hi
                } else {
                    z
                }
            }
        }
    }

    /// Heights of the layers: layer_heights() on each half of the working zone
    /// (the end zones are straight and need none).
    pub fn levels(&self, s: &GearSystem, teeth: usize, longest: f64, r_max: f64) -> Vec<f64> {
        if self.k == 0.0 {
            return vec![self.z_lo, self.z_hi];
        }
        let half = layer_heights(s, teeth, self.width / 2.0, longest, Some(r_max));
        let mut out: Vec<f64> = half.iter().map(|z| self.z_lo + z).collect();
        out.extend(half[..half.len() - 1].iter().rev().map(|z| self.z_hi - z));
        out
    }
}

fn check_common(width: f64, hand: i32, end_bottom: f64, end_top: f64) -> Result<()> {
    if !(width > 0.0) {
        return Err(GearError("The face width must be positive.".into()));
    }
    if hand != 1 && hand != -1 {
        return Err(GearError("The twist direction must be 1 or -1.".into()));
    }
    if end_bottom < 0.0 || end_top < 0.0 {
        return Err(GearError("The chamfer heights can't be negative.".into()));
    }
    Ok(())
}

/// Closed mesh of a herringbone spur gear centered on the z axis, from z = 0
/// to end_bottom + width + end_top: the outline from system.spur_gear(teeth),
/// twisting at the helix angle up to the middle of the working width and
/// back. bore is a diameter (0 for none).
///
/// hand (+1 or -1) is the direction of the twist: with +1 the lower half
/// turns counterclockwise going up (seen from above). Two external gears that
/// mesh need opposite hands; a planet and its ring the same hand.
///
/// end_bottom, end_top: lengths of the end zones below and above the working
/// width (0 for none: a flat end). An end zone extrudes the outline straight
/// (no twist) and a cone chamfers the teeth down to just inside the root
/// circle, so every flank meets the end surface at 90 degrees or more (see
/// EndZones). The working width keeps full teeth.
///
/// Faces: one triangulation (triangulate()), with the fold circle as a crease
/// when there are end zones, lifted onto each end surface. Walls: a column of
/// vertices per outline point (each ending where the cone cuts it),
/// neighbouring columns stitched together. The bore is a straight prism (its
/// circle doesn't need to turn).
#[allow(clippy::too_many_arguments)]
pub fn spur_gear_mesh(s: &GearSystem, teeth: usize, width: f64, bore: f64, hand: i32, end_bottom: f64, end_top: f64) -> Result<IndexedMesh> {
    check_common(width, hand, end_bottom, end_top)?;
    let zones = EndZones::new(s, teeth, width, end_bottom, end_top, hand, false);
    let has_ends = end_bottom > 0.0 || end_top > 0.0;
    let mut loops = vec![s.spur_gear(teeth)?];
    if bore > 0.0 {
        let limit = if has_ends { zones.r_fold } else { zones.r_root };
        if bore / 2.0 > limit - 2.0 * s.ds {
            return Err(GearError(if has_ends {
                format!("The bore ({} mm) must be smaller than the flat face the end chamfers leave ({:.3} mm).", bore, 2.0 * zones.r_fold)
            } else {
                format!("The bore ({} mm) must be smaller than the bottom of the tooth gaps ({:.3} mm across).", bore, 2.0 * zones.r_root)
            }));
        }
        let mut hole = s.circle(bore / 2.0, true)?;
        hole.reverse(); // holes go clockwise
        loops.push(hole);
    }
    let r_max = zones.r_tip;
    gear_solid(s, teeth, &zones, loops, 0, has_ends, r_max)
}

/// Closed mesh of a herringbone ring gear centered on the z axis: the toothed
/// edge from system.ring_gear(teeth) inside a rim of diameter `rim`, from
/// z = 0 to end_bottom + width + end_top. hand, end_bottom and end_top as for
/// spur_gear_mesh(); a planet and its ring use the same hand. In the end zones
/// the ring's teeth are chamfered out to just past its root circle, leaving a
/// flat annulus out to the rim.
#[allow(clippy::too_many_arguments)]
pub fn ring_gear_mesh(s: &GearSystem, teeth: usize, width: f64, rim: f64, hand: i32, end_bottom: f64, end_top: f64) -> Result<IndexedMesh> {
    check_common(width, hand, end_bottom, end_top)?;
    let zones = EndZones::new(s, teeth, width, end_bottom, end_top, hand, true);
    let has_ends = end_bottom > 0.0 || end_top > 0.0;
    let limit = if has_ends { zones.r_fold } else { zones.r_root };
    if rim / 2.0 < limit + 2.0 * s.ds {
        return Err(GearError(if has_ends {
            format!("The ring's outer diameter ({} mm) must be larger than the flat face the end chamfers leave ({:.3} mm).", rim, 2.0 * zones.r_fold)
        } else {
            format!("The ring's outer diameter ({} mm) must be larger than the bottom of its tooth gaps (the root circle, {:.3} mm across).", rim, 2.0 * zones.r_root)
        }));
    }
    // The rim's material is inside its circle: points on the circle, chords inside.
    let mut toothed = s.ring_gear(teeth)?;
    toothed.reverse();
    let loops = vec![s.circle(rim / 2.0, false)?, toothed];
    let r_max = zones.r_root;
    gear_solid(s, teeth, &zones, loops, 1, has_ends, r_max)
}

/// Shared by spur_gear_mesh() and ring_gear_mesh(). loops: the face's
/// boundary, outer counterclockwise and hole clockwise; loops[toothed] is the
/// toothed outline (it twists and is cut by the cones), the other, if any, a
/// plain circle (bore or rim, a straight prism).
fn gear_solid(s: &GearSystem, teeth: usize, zones: &EndZones, loops: Vec<Vec<P>>, toothed: usize, has_ends: bool, r_max: f64) -> Result<IndexedMesh> {
    let creases = if has_ends { vec![s.circle(zones.r_fold, false)?] } else { vec![] }; // fold between cone and flat face
    let tri = triangulate(&loops, DEFAULT_MIN_ANGLE, s.ds, &creases)?;
    let points = &tri.points;
    let teeth_loop = &tri.boundary[toothed];
    let empty = Vec::new();
    let plain_loop = if loops.len() == 2 { &tri.boundary[1 - toothed] } else { &empty };

    let seg_length = |lp: &Vec<usize>, n: usize| {
        let (p, q) = (points[lp[n]], points[lp[(n + 1) % lp.len()]]);
        (q.0 - p.0).hypot(q.1 - p.1)
    };
    let longest = (0..teeth_loop.len()).map(|n| seg_length(teeth_loop, n)).fold(0.0, f64::max);
    let levels = zones.levels(s, teeth, longest, r_max);
    let mut mesh = IndexedMesh::new();

    let placed = |i: usize, z: f64, angle: f64| -> [f64; 3] {
        let (x, y) = points[i];
        let (c, sn) = (angle.cos(), angle.sin());
        [x * c - y * sn, x * sn + y * c, z]
    };

    // Toothed columns: from where the bottom cone cuts to where the top cone
    // cuts, through every level in between. A column on a cone ends at its own
    // height; z_bottom / z_top keep that at least ds from the working edges, so
    // it can't fall right next to a level.
    let mut columns: FxHashMap<usize, Vec<(f64, usize)>> = FxHashMap::default();
    for &i in teeth_loop {
        let r = points[i].0.hypot(points[i].1);
        let (zb, zt) = (zones.z_bottom(r), zones.z_top(r));
        let mut zs = vec![zb];
        zs.extend(levels.iter().copied().filter(|&z| zb < z && z < zt));
        zs.push(zt);
        let col = zs.into_iter().map(|z| (z, mesh.add_vertex(placed(i, z, zones.twist(z))))).collect();
        columns.insert(i, col);
    }

    // Plain circle (bore or rim): a straight prism. Its rings sit at the end
    // faces' turns (the flat parts of the end surfaces turn rigidly); if those
    // differed, the turn would be spread linearly over enough layers to keep
    // the wall's twisted quads within max_deviation.
    if !plain_loop.is_empty() {
        let (turn0, turn1) = (zones.twist(0.0), zones.twist(zones.total));
        let longest = (0..plain_loop.len()).map(|n| seg_length(plain_loop, n)).fold(0.0, f64::max);
        let n_plain = ((longest * (turn1 - turn0).abs() / (4.0 * s.max_deviation) - 1e-9).ceil() as i64).max(1) as usize;
        for &i in plain_loop {
            let col = (0..=n_plain)
                .map(|m| {
                    let z = zones.total * m as f64 / n_plain as f64;
                    (z, mesh.add_vertex(placed(i, z, turn0 + (turn1 - turn0) * z / zones.total)))
                })
                .collect();
            columns.insert(i, col);
        }
    }

    // End surfaces: every face vertex lifted to where the cone (or the flat
    // face) is at its radius, turned by the twist there. Boundary vertices are
    // the columns' ends.
    for bottom in [true, false] {
        let mut ids: FxHashMap<usize, usize> =
            columns.iter().map(|(&i, col)| (i, if bottom { col[0].1 } else { col[col.len() - 1].1 })).collect();
        let mut vid = |i: usize, mesh: &mut IndexedMesh| -> usize {
            if let Some(&v) = ids.get(&i) {
                return v;
            }
            let r = points[i].0.hypot(points[i].1);
            let z = if bottom { zones.z_bottom(r) } else { zones.z_top(r) };
            let v = mesh.add_vertex(placed(i, z, zones.twist(z)));
            ids.insert(i, v);
            v
        };
        for &[a, b, c] in &tri.triangles {
            let (va, vb, vc) = (vid(a, &mut mesh), vid(b, &mut mesh), vid(c, &mut mesh));
            if bottom {
                mesh.add_triangle(va, vc, vb);
            } else {
                mesh.add_triangle(va, vb, vc);
            }
        }
    }

    // Walls. Going along a counterclockwise outer loop (or a clockwise hole),
    // triangles (u_i, v_j, v_j+1) and (u_i, v_j, u_i+1) face away from the material.
    for lp in [teeth_loop, plain_loop] {
        for n in 0..lp.len() {
            let (cu, cv) = (&columns[&lp[n]], &columns[&lp[(n + 1) % lp.len()]]);
            stitch(&mut mesh, cu, cv);
        }
    }
    Ok(mesh)
}

/// Wall strip between two vertical chains [(z, vertex)] of neighbouring
/// boundary points u -> v: always advance the chain whose next vertex is
/// lower. When both are at the same height (a quad), split it along its
/// shorter diagonal.
pub(crate) fn stitch(mesh: &mut IndexedMesh, cu: &[(f64, usize)], cv: &[(f64, usize)]) {
    let (mut i, mut j) = (0, 0);
    while i < cu.len() - 1 || j < cv.len() - 1 {
        let advance_v = if i == cu.len() - 1 {
            true
        } else if j == cv.len() - 1 {
            false
        } else if (cv[j + 1].0 - cu[i + 1].0).abs() < 1e-12 {
            // diagonal u_i - v_j+1 if v goes first, v_j - u_i+1 otherwise
            let v = &mesh.vertices;
            dist3(v[cu[i].1], v[cv[j + 1].1]) <= dist3(v[cv[j].1], v[cu[i + 1].1])
        } else {
            cv[j + 1].0 < cu[i + 1].0
        };
        if advance_v {
            mesh.add_triangle(cu[i].1, cv[j].1, cv[j + 1].1);
            j += 1;
        } else {
            mesh.add_triangle(cu[i].1, cv[j].1, cu[i + 1].1);
            i += 1;
        }
    }
}

fn dist3(p: [f64; 3], q: [f64; 3]) -> f64 {
    ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
}

/// A copy of the mesh turned about the z axis, then moved in x and y.
pub fn placed_mesh(mesh: &IndexedMesh, rotation: f64, center: P) -> IndexedMesh {
    let (c, sn) = (rotation.cos(), rotation.sin());
    IndexedMesh {
        vertices: mesh.vertices.iter().map(|&[x, y, z]| [x * c - y * sn + center.0, x * sn + y * c + center.1, z]).collect(),
        triangles: mesh.triangles.clone(),
    }
}

/// One mesh holding several separate closed bodies.
pub fn combined(meshes: &[IndexedMesh]) -> IndexedMesh {
    let mut out = IndexedMesh::new();
    for m in meshes {
        let base = out.vertices.len();
        out.vertices.extend_from_slice(&m.vertices);
        out.triangles.extend(m.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
    }
    out
}

/// Meshes of a planetary set, placed like GearSystem::planetary_gear():
/// [sun, planet_1, ..., planet_n, ring], every mesh centered.
///
/// The sun has `hand`; the planets and the ring the opposite hand (each
/// external pair opposite, a planet and its ring the same). All parts share
/// the width and end zones, so their twists match slice by slice. bore is the
/// sun's, planet_bore the planets', rim the ring's outer diameter. Use
/// combined() for a single STL.
#[allow(clippy::too_many_arguments)]
pub fn planetary_gear_mesh(
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
) -> Result<Vec<IndexedMesh>> {
    let (teeth_ring, layout, ring_rotation) = s.planetary_layout(teeth_sun, teeth_planet, planets, 0.0)?;
    let sun = spur_gear_mesh(s, teeth_sun, width, bore, hand, end_bottom, end_top)?;
    let planet = spur_gear_mesh(s, teeth_planet, width, planet_bore, -hand, end_bottom, end_top)?;
    let ring = ring_gear_mesh(s, teeth_ring, width, rim, -hand, end_bottom, end_top)?;
    let mut parts = vec![sun];
    for (center, rotation) in layout {
        parts.push(placed_mesh(&planet, rotation, center));
    }
    parts.push(placed_mesh(&ring, ring_rotation, (0.0, 0.0)));
    Ok(parts)
}
