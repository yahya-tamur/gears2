//! STEP files: exact solids (boundary representations) as ISO 10303-21
//! text, schema AP214 (automotive_design), lengths in mm.
//!
//! A Solid is a closed shell of faces. Each face lies on one surface and is
//! bounded by loops of edges; each edge lies on one curve and runs between
//! two vertices. Neighbouring faces share their edges (each edge is used
//! once in each direction), which is what makes the shell closed.
//!
//! Orientation, as STEP defines it: a face's normal is its surface's normal
//! (flipped when `same_sense` is false), and it points out of the solid. Each
//! loop runs counterclockwise seen from outside (the face on the left); holes
//! run the other way.
//!
//! Curves are lines, circles (whole or arcs) and cubic B-splines; surfaces
//! are planes, cylinders, cones, spheres, linear extrusions of a curve, and
//! bicubic B-spline surfaces. B-splines here are built from piecewise cubic Hermite data
//! (points and exact derivatives at the ends of each piece), which joins
//! with matching first derivatives, so every interior knot has multiplicity
//! 2 (see `hermite_curve` and `hermite_surface`).

use std::fmt::Write;

pub type V3 = [f64; 3];

pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn scale(a: V3, k: f64) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}
pub fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}
pub fn unit(a: V3) -> V3 {
    scale(a, 1.0 / norm(a))
}

/// A cubic B-spline: control points and the full knot vector (each knot
/// repeated by its multiplicity; 4 at each end).
#[derive(Debug, Clone)]
pub struct BSpline {
    pub points: Vec<V3>,
    pub knots: Vec<f64>,
}

#[derive(Debug, Clone)]
pub enum Curve {
    Line { from: V3, to: V3 },
    /// Counterclockwise about `axis` from `x_dir` (angle 0).
    Circle { center: V3, axis: V3, x_dir: V3, radius: f64 },
    /// The part of that circle from angle 0 (x_dir) to `sweep` (> 0): written
    /// as the circle, the edge's vertices bounding it.
    Arc { center: V3, axis: V3, x_dir: V3, radius: f64, sweep: f64 },
    BSpline(BSpline),
}

#[derive(Debug, Clone)]
pub enum Surface {
    /// Normal `normal`.
    Plane { origin: V3, normal: V3, x_dir: V3 },
    /// Normal pointing away from the axis.
    Cylinder { origin: V3, axis: V3, x_dir: V3, radius: f64 },
    /// Radius `radius` at `origin`, growing along `axis` at `semi_angle`;
    /// normal pointing away from the axis.
    Cone { origin: V3, axis: V3, x_dir: V3, radius: f64, semi_angle: f64 },
    /// Normal pointing away from the center.
    Sphere { center: V3, axis: V3, x_dir: V3, radius: f64 },
    /// curve(u) + v * dir; normal curve'(u) x dir.
    Extrusion { curve: BSpline, dir: V3 },
    /// Control points [u][v]; normal dS/du x dS/dv.
    BSpline { points: Vec<Vec<V3>>, knots_u: Vec<f64>, knots_v: Vec<f64> },
}

#[derive(Debug, Clone)]
pub struct Edge {
    pub start: usize,
    pub end: usize,
    /// Runs from start to end as its parameter grows.
    pub curve: Curve,
}

#[derive(Debug, Clone)]
pub struct Face {
    pub surface: Surface,
    pub same_sense: bool,
    /// Loops of (edge, forward); the first is the outer boundary.
    pub loops: Vec<Vec<(usize, bool)>>,
}

#[derive(Debug, Clone, Default)]
pub struct Solid {
    pub vertices: Vec<V3>,
    pub edges: Vec<Edge>,
    pub faces: Vec<Face>,
}

// ------------------------------------------------------------ placements

/// A rigid placement (a STEP AXIS2_PLACEMENT_3D): local points (x, y, z) go
/// to origin + x x_dir + y (axis x x_dir) + z axis. axis and x_dir are unit
/// and square to each other, so it never mirrors.
#[derive(Debug, Clone, Copy)]
pub struct Placement {
    pub origin: V3,
    pub axis: V3,
    pub x_dir: V3,
}

impl Placement {
    pub const IDENTITY: Placement = Placement { origin: [0.0; 3], axis: [0.0, 0.0, 1.0], x_dir: [1.0, 0.0, 0.0] };

    /// Turned about the z axis by `rotation`, then moved by `offset`.
    pub fn turned(rotation: f64, offset: V3) -> Placement {
        Placement { origin: offset, axis: [0.0, 0.0, 1.0], x_dir: [rotation.cos(), rotation.sin(), 0.0] }
    }

    /// The local x, y and z directions.
    pub fn frame(&self) -> [V3; 3] {
        [self.x_dir, cross(self.axis, self.x_dir), self.axis]
    }

    pub fn direction(&self, v: V3) -> V3 {
        let [x, y, z] = self.frame();
        add(add(scale(x, v[0]), scale(y, v[1])), scale(z, v[2]))
    }

    pub fn point(&self, v: V3) -> V3 {
        add(self.origin, self.direction(v))
    }

    /// This placement after `inner`: points placed by inner, then by self.
    pub fn then(&self, inner: &Placement) -> Placement {
        Placement { origin: self.point(inner.origin), axis: self.direction(inner.axis), x_dir: self.direction(inner.x_dir) }
    }
}

// ------------------------------------------------------------ evaluation

impl BSpline {
    /// Parameter range.
    pub fn range(&self) -> (f64, f64) {
        (self.knots[3], self.knots[self.knots.len() - 4])
    }

    /// The point at t (de Boor).
    pub fn at(&self, t: f64) -> V3 {
        let k = &self.knots;
        let n = self.points.len();
        // the span: k[i] <= t < k[i + 1], 3 <= i < n
        let mut i = 3;
        while i + 1 < n && t >= k[i + 1] {
            i += 1;
        }
        let mut d: Vec<V3> = (0..4).map(|j| self.points[i - 3 + j]).collect();
        for r in 1..4 {
            for j in (r..4).rev() {
                let ii = i - 3 + j;
                let den = k[ii + 4 - r] - k[ii];
                let a = if den == 0.0 { 0.0 } else { (t - k[ii]) / den };
                d[j] = add(scale(d[j - 1], 1.0 - a), scale(d[j], a));
            }
        }
        d[3]
    }
}

impl Curve {
    pub fn range(&self) -> (f64, f64) {
        match self {
            Curve::Line { .. } => (0.0, 1.0),
            Curve::Circle { .. } => (0.0, 2.0 * std::f64::consts::PI),
            Curve::Arc { sweep, .. } => (0.0, *sweep),
            Curve::BSpline(b) => b.range(),
        }
    }

    pub fn at(&self, t: f64) -> V3 {
        match self {
            Curve::Line { from, to } => add(*from, scale(sub(*to, *from), t)),
            Curve::Circle { center, axis, x_dir, radius } | Curve::Arc { center, axis, x_dir, radius, .. } => {
                let y = cross(*axis, *x_dir);
                add(*center, add(scale(*x_dir, radius * t.cos()), scale(y, radius * t.sin())))
            }
            Curve::BSpline(b) => b.at(t),
        }
    }
}

impl Surface {
    /// The point at (u, v), for the parametric surfaces (B-spline and
    /// extrusion); None for the others.
    pub fn at(&self, u: f64, v: f64) -> Option<V3> {
        match self {
            Surface::BSpline { points, knots_u, knots_v } => {
                let column: Vec<V3> = points.iter().map(|row| BSpline { points: row.clone(), knots: knots_v.clone() }.at(v)).collect();
                Some(BSpline { points: column, knots: knots_u.clone() }.at(u))
            }
            Surface::Extrusion { curve, dir } => Some(add(curve.at(u), scale(*dir, v))),
            _ => None,
        }
    }
}

impl Solid {
    pub fn add_vertex(&mut self, p: V3) -> usize {
        self.vertices.push(p);
        self.vertices.len() - 1
    }

    /// An edge from vertex start to vertex end. A B-spline's end points are
    /// set to the vertices exactly (they're its first and last control
    /// points), so rounding in the curve's data can't separate them.
    pub fn add_edge(&mut self, start: usize, end: usize, curve: Curve) -> usize {
        let curve = match curve {
            Curve::BSpline(mut b) => {
                let last = b.points.len() - 1;
                b.points[0] = self.vertices[start];
                b.points[last] = self.vertices[end];
                Curve::BSpline(b)
            }
            other => other,
        };
        self.edges.push(Edge { start, end, curve });
        self.edges.len() - 1
    }

    /// Points along a loop, in its direction (n per edge).
    pub fn loop_points(&self, lp: &[(usize, bool)], n: usize) -> Vec<V3> {
        let mut out = Vec::new();
        for &(e, forward) in lp {
            let c = &self.edges[e].curve;
            let (a, b) = c.range();
            for i in 0..n {
                let f = i as f64 / n as f64;
                let t = if forward { a + (b - a) * f } else { b - (b - a) * f };
                out.push(c.at(t));
            }
        }
        out
    }

    /// The solid turned about the z axis by `rotation`, then moved by
    /// `offset`.
    pub fn placed(&self, rotation: f64, offset: V3) -> Solid {
        self.transformed(&Placement::turned(rotation, offset))
    }

    /// The solid moved into `place` (its x, y, z axes becoming the
    /// placement's).
    pub fn transformed(&self, place: &Placement) -> Solid {
        let dir = |v: V3| place.direction(v);
        let pt = |v: V3| place.point(v);
        let spline = |b: &BSpline| BSpline { points: b.points.iter().map(|&p| pt(p)).collect(), knots: b.knots.clone() };
        let curve = |cv: &Curve| match cv {
            Curve::Line { from, to } => Curve::Line { from: pt(*from), to: pt(*to) },
            Curve::Circle { center, axis, x_dir, radius } => Curve::Circle { center: pt(*center), axis: dir(*axis), x_dir: dir(*x_dir), radius: *radius },
            Curve::Arc { center, axis, x_dir, radius, sweep } => {
                Curve::Arc { center: pt(*center), axis: dir(*axis), x_dir: dir(*x_dir), radius: *radius, sweep: *sweep }
            }
            Curve::BSpline(b) => Curve::BSpline(spline(b)),
        };
        let surface = |sf: &Surface| match sf {
            Surface::Plane { origin, normal, x_dir } => Surface::Plane { origin: pt(*origin), normal: dir(*normal), x_dir: dir(*x_dir) },
            Surface::Cylinder { origin, axis, x_dir, radius } => {
                Surface::Cylinder { origin: pt(*origin), axis: dir(*axis), x_dir: dir(*x_dir), radius: *radius }
            }
            Surface::Cone { origin, axis, x_dir, radius, semi_angle } => Surface::Cone {
                origin: pt(*origin),
                axis: dir(*axis),
                x_dir: dir(*x_dir),
                radius: *radius,
                semi_angle: *semi_angle,
            },
            Surface::Sphere { center, axis, x_dir, radius } => Surface::Sphere { center: pt(*center), axis: dir(*axis), x_dir: dir(*x_dir), radius: *radius },
            Surface::Extrusion { curve, dir: d } => Surface::Extrusion { curve: spline(curve), dir: dir(*d) },
            Surface::BSpline { points, knots_u, knots_v } => Surface::BSpline {
                points: points.iter().map(|row| row.iter().map(|&p| pt(p)).collect()).collect(),
                knots_u: knots_u.clone(),
                knots_v: knots_v.clone(),
            },
        };
        Solid {
            vertices: self.vertices.iter().map(|&p| pt(p)).collect(),
            edges: self.edges.iter().map(|e| Edge { start: e.start, end: e.end, curve: curve(&e.curve) }).collect(),
            faces: self.faces.iter().map(|f| Face { surface: surface(&f.surface), same_sense: f.same_sense, loops: f.loops.clone() }).collect(),
        }
    }

    /// Problems with the shell's structure: an edge not used exactly once in
    /// each direction, a loop whose edges don't join end to end, an edge whose
    /// curve doesn't start and end at its vertices (within `tol`). Empty if
    /// none.
    pub fn problems(&self, tol: f64) -> Vec<String> {
        let mut out = Vec::new();
        let mut uses = vec![(0, 0); self.edges.len()];
        for (fi, f) in self.faces.iter().enumerate() {
            for lp in &f.loops {
                for k in 0..lp.len() {
                    let (e, fw) = lp[k];
                    if fw { uses[e].0 += 1 } else { uses[e].1 += 1 }
                    let end = if fw { self.edges[e].end } else { self.edges[e].start };
                    let (n, nfw) = lp[(k + 1) % lp.len()];
                    let next_start = if nfw { self.edges[n].start } else { self.edges[n].end };
                    if end != next_start {
                        out.push(format!("face {fi}: edge {e} ends at vertex {end}, the next edge {n} starts at {next_start}"));
                    }
                }
            }
        }
        for (e, &(f, b)) in uses.iter().enumerate() {
            if (f, b) != (1, 1) {
                out.push(format!("edge {e} used {f} times forward and {b} times backward"));
            }
        }
        for (i, e) in self.edges.iter().enumerate() {
            let (a, b) = e.curve.range();
            let (p, q) = (e.curve.at(a), e.curve.at(b));
            if norm(sub(p, self.vertices[e.start])) > tol || norm(sub(q, self.vertices[e.end])) > tol {
                out.push(format!("edge {i}: its curve runs {p:?} to {q:?}, its vertices are {:?} to {:?}", self.vertices[e.start], self.vertices[e.end]));
            }
        }
        out
    }
}

// --------------------------------------------------- Hermite to B-spline

/// Breakpoints in [a, b] where cubic Hermite interpolation of `f` (point and
/// derivative) stays within `tol`: pieces are halved until the cubic matches
/// f at 3 points inside each. `forced` breakpoints are always kept.
pub fn hermite_breaks(f: &dyn Fn(f64) -> (V3, V3), a: f64, b: f64, forced: &[f64], tol: f64) -> Vec<f64> {
    let mut ts = vec![a];
    let mut bounds: Vec<f64> = forced.iter().copied().filter(|&t| a < t && t < b).collect();
    bounds.push(b);
    let mut lo = a;
    for hi in bounds {
        split(f, lo, hi, tol, &mut ts, 0);
        lo = hi;
    }
    ts
}

fn split(f: &dyn Fn(f64) -> (V3, V3), a: f64, b: f64, tol: f64, out: &mut Vec<f64>, depth: usize) {
    let (fa, fb) = (f(a), f(b));
    let ok = [0.25, 0.5, 0.75].iter().all(|&s| {
        let p = hermite(fa, fb, b - a, s);
        norm(sub(p, f(a + (b - a) * s).0)) <= tol
    });
    if ok || depth > 24 {
        out.push(b);
    } else {
        let m = 0.5 * (a + b);
        split(f, a, m, tol, out, depth + 1);
        split(f, m, b, tol, out, depth + 1);
    }
}

/// The cubic Hermite interpolant at s in [0, 1] of a piece of length h.
fn hermite((p0, d0): (V3, V3), (p1, d1): (V3, V3), h: f64, s: f64) -> V3 {
    let (h00, h10, h01, h11) = (
        2.0 * s * s * s - 3.0 * s * s + 1.0,
        s * s * s - 2.0 * s * s + s,
        -2.0 * s * s * s + 3.0 * s * s,
        s * s * s - s * s,
    );
    add(add(scale(p0, h00), scale(d0, h10 * h)), add(scale(p1, h01), scale(d1, h11 * h)))
}

/// Knot vector for breakpoints ts: 4 at each end, 2 at each inside.
fn knots(ts: &[f64]) -> Vec<f64> {
    let mut k = vec![ts[0]; 4];
    for &t in &ts[1..ts.len() - 1] {
        k.push(t);
        k.push(t);
    }
    k.extend([ts[ts.len() - 1]; 4]);
    k
}

/// The cubic B-spline through f's points and derivatives at ts, parametrized
/// like f. Per piece the Bezier points are p0, p0 + d0 h/3, p1 - d1 h/3, p1;
/// the shared end points between pieces are left out (knots of multiplicity
/// 2: the curve is C1, so they're implied).
pub fn hermite_curve(f: &dyn Fn(f64) -> (V3, V3), ts: &[f64]) -> BSpline {
    let data: Vec<(V3, V3)> = ts.iter().map(|&t| f(t)).collect();
    let mut points = vec![data[0].0];
    for i in 0..ts.len() - 1 {
        let h = ts[i + 1] - ts[i];
        points.push(add(data[i].0, scale(data[i].1, h / 3.0)));
        points.push(sub(data[i + 1].0, scale(data[i + 1].1, h / 3.0)));
    }
    points.push(data[data.len() - 1].0);
    BSpline { points, knots: knots(ts) }
}

/// A surface's point and derivatives at (u, v): (S, Su, Sv, Suv).
pub type SurfaceData = (V3, V3, V3, V3);

/// The bicubic B-spline surface through f's data at us x vs (patch by patch
/// the Hermite-to-Bezier net, shared rows and columns between patches left
/// out as for hermite_curve). Its boundary curves equal hermite_curve() of
/// the same data, so edges built that way lie exactly on it.
pub fn hermite_surface(f: &dyn Fn(f64, f64) -> SurfaceData, us: &[f64], vs: &[f64]) -> Surface {
    // Control values along one direction from point/derivative pairs.
    fn line(vals: &[(V3, V3)], ts: &[f64]) -> Vec<V3> {
        let mut out = vec![vals[0].0];
        for i in 0..ts.len() - 1 {
            let h = ts[i + 1] - ts[i];
            out.push(add(vals[i].0, scale(vals[i].1, h / 3.0)));
            out.push(sub(vals[i + 1].0, scale(vals[i + 1].1, h / 3.0)));
        }
        out.push(vals[vals.len() - 1].0);
        out
    }
    let data: Vec<Vec<SurfaceData>> = us.iter().map(|&u| vs.iter().map(|&v| f(u, v)).collect()).collect();
    // Along v for each u node: rows of (S, Sv) and of (Su, Suv).
    let pos: Vec<Vec<V3>> = data.iter().map(|col| line(&col.iter().map(|d| (d.0, d.2)).collect::<Vec<_>>(), vs)).collect();
    let der: Vec<Vec<V3>> = data.iter().map(|col| line(&col.iter().map(|d| (d.1, d.3)).collect::<Vec<_>>(), vs)).collect();
    // Then along u for each v control index.
    let nv = pos[0].len();
    let mut columns: Vec<Vec<V3>> = Vec::new();
    for j in 0..nv {
        let vals: Vec<(V3, V3)> = (0..us.len()).map(|i| (pos[i][j], der[i][j])).collect();
        columns.push(line(&vals, us));
    }
    let nu = columns[0].len();
    let points = (0..nu).map(|i| (0..nv).map(|j| columns[j][i]).collect()).collect();
    Surface::BSpline { points, knots_u: knots(us), knots_v: knots(vs) }
}

// ------------------------------------------------------------- writing

/// A STEP real: always with a decimal point. Lengths are mm and angles
/// radians, so 10 decimals (1e-10 mm) is far below the accuracy the solids
/// are drawn to; every copy of a number rounds the same way, so shared
/// points stay shared.
fn real(x: f64) -> String {
    let s = format!("{x:.10}");
    let s = s.trim_end_matches('0'); // keeps the decimal point
    if s == "-0." { "0.".to_string() } else { s.to_string() }
}

struct Writer {
    out: String,
    next: usize,
}

impl Writer {
    fn add(&mut self, entity: String) -> usize {
        let id = self.next;
        self.next += 1;
        writeln!(self.out, "#{id}={entity};").unwrap();
        id
    }
    fn point(&mut self, p: V3) -> usize {
        self.add(format!("CARTESIAN_POINT('',({},{},{}))", real(p[0]), real(p[1]), real(p[2])))
    }
    fn direction(&mut self, d: V3) -> usize {
        let d = unit(d);
        self.add(format!("DIRECTION('',({},{},{}))", real(d[0]), real(d[1]), real(d[2])))
    }
    fn placement(&mut self, origin: V3, axis: V3, x_dir: V3) -> usize {
        let (o, a, x) = (self.point(origin), self.direction(axis), self.direction(x_dir));
        self.add(format!("AXIS2_PLACEMENT_3D('',#{o},#{a},#{x})"))
    }
    fn list(ids: &[usize]) -> String {
        ids.iter().map(|i| format!("#{i}")).collect::<Vec<_>>().join(",")
    }
    /// Distinct knots and their multiplicities.
    fn knot_lists(knots: &[f64]) -> (String, String) {
        let mut vals: Vec<f64> = Vec::new();
        let mut mults: Vec<usize> = Vec::new();
        for &k in knots {
            if vals.last() == Some(&k) {
                *mults.last_mut().unwrap() += 1;
            } else {
                vals.push(k);
                mults.push(1);
            }
        }
        (mults.iter().map(|m| m.to_string()).collect::<Vec<_>>().join(","), vals.iter().map(|&v| real(v)).collect::<Vec<_>>().join(","))
    }
    fn spline(&mut self, b: &BSpline) -> usize {
        let pts: Vec<usize> = b.points.iter().map(|&p| self.point(p)).collect();
        let (m, k) = Self::knot_lists(&b.knots);
        self.add(format!("B_SPLINE_CURVE_WITH_KNOTS('',3,({}),.UNSPECIFIED.,.F.,.F.,({m}),({k}),.UNSPECIFIED.)", Self::list(&pts)))
    }
    fn curve(&mut self, c: &Curve) -> usize {
        match c {
            Curve::Line { from, to } => {
                let p = self.point(*from);
                let d = self.direction(sub(*to, *from));
                let v = self.add(format!("VECTOR('',#{d},{})", real(norm(sub(*to, *from)))));
                self.add(format!("LINE('',#{p},#{v})"))
            }
            Curve::Circle { center, axis, x_dir, radius } | Curve::Arc { center, axis, x_dir, radius, .. } => {
                let a = self.placement(*center, *axis, *x_dir);
                self.add(format!("CIRCLE('',#{a},{})", real(*radius)))
            }
            Curve::BSpline(b) => self.spline(b),
        }
    }
    fn surface(&mut self, s: &Surface) -> usize {
        match s {
            Surface::Plane { origin, normal, x_dir } => {
                let a = self.placement(*origin, *normal, *x_dir);
                self.add(format!("PLANE('',#{a})"))
            }
            Surface::Cylinder { origin, axis, x_dir, radius } => {
                let a = self.placement(*origin, *axis, *x_dir);
                self.add(format!("CYLINDRICAL_SURFACE('',#{a},{})", real(*radius)))
            }
            Surface::Cone { origin, axis, x_dir, radius, semi_angle } => {
                let a = self.placement(*origin, *axis, *x_dir);
                self.add(format!("CONICAL_SURFACE('',#{a},{},{})", real(*radius), real(*semi_angle)))
            }
            Surface::Sphere { center, axis, x_dir, radius } => {
                let a = self.placement(*center, *axis, *x_dir);
                self.add(format!("SPHERICAL_SURFACE('',#{a},{})", real(*radius)))
            }
            Surface::Extrusion { curve, dir } => {
                let c = self.spline(curve);
                let d = self.direction(*dir);
                let v = self.add(format!("VECTOR('',#{d},{})", real(norm(*dir))));
                self.add(format!("SURFACE_OF_LINEAR_EXTRUSION('',#{c},#{v})"))
            }
            Surface::BSpline { points, knots_u, knots_v } => {
                let rows: Vec<String> = points.iter().map(|row| format!("({})", Self::list(&row.iter().map(|&p| self.point(p)).collect::<Vec<_>>()))).collect();
                let (mu, ku) = Self::knot_lists(knots_u);
                let (mv, kv) = Self::knot_lists(knots_v);
                self.add(format!(
                    "B_SPLINE_SURFACE_WITH_KNOTS('',3,3,({}),.UNSPECIFIED.,.F.,.F.,.F.,({mu}),({mv}),({ku}),({kv}),.UNSPECIFIED.)",
                    rows.join(",")
                ))
            }
        }
    }
    fn solid(&mut self, s: &Solid) -> usize {
        let verts: Vec<usize> = s.vertices.iter().map(|&p| {
            let q = self.point(p);
            self.add(format!("VERTEX_POINT('',#{q})"))
        }).collect();
        let edges: Vec<usize> = s.edges.iter().map(|e| {
            let c = self.curve(&e.curve);
            self.add(format!("EDGE_CURVE('',#{},#{},#{c},.T.)", verts[e.start], verts[e.end]))
        }).collect();
        let mut faces = Vec::new();
        for f in &s.faces {
            let mut bounds = Vec::new();
            for (k, lp) in f.loops.iter().enumerate() {
                let oriented: Vec<usize> = lp
                    .iter()
                    .map(|&(e, fw)| self.add(format!("ORIENTED_EDGE('',*,*,#{},{})", edges[e], if fw { ".T." } else { ".F." })))
                    .collect();
                let l = self.add(format!("EDGE_LOOP('',({}))", Self::list(&oriented)));
                let kind = if k == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
                bounds.push(self.add(format!("{kind}('',#{l},.T.)")));
            }
            let surf = self.surface(&f.surface);
            faces.push(self.add(format!(
                "ADVANCED_FACE('',({}),#{surf},{})",
                Self::list(&bounds),
                if f.same_sense { ".T." } else { ".F." }
            )));
        }
        let shell = self.add(format!("CLOSED_SHELL('',({}))", Self::list(&faces)));
        self.add(format!("MANIFOLD_SOLID_BREP('',#{shell})"))
    }
}

/// The boilerplate every file needs: the application context and the
/// geometric context (mm, radians, `tolerance`). Returns (app, ctx).
fn contexts(w: &mut Writer, tolerance: f64) -> (usize, usize) {
    let app = w.add("APPLICATION_CONTEXT('automotive_design')".into());
    w.add(format!("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{app})"));
    let mm = w.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))".into());
    let rad = w.add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))".into());
    let sr = w.add("(NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT())".into());
    let unc = w.add(format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{mm},'distance_accuracy_value','confusion accuracy')", real(tolerance)));
    let ctx = w.add(format!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{unc})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{mm},#{rad},#{sr})) REPRESENTATION_CONTEXT('Context3D','3D Context'))"
    ));
    (app, ctx)
}

/// A product (part or assembly) named `name`: returns its product
/// definition.
fn product(w: &mut Writer, app: usize, name: &str) -> usize {
    let pc = w.add(format!("PRODUCT_CONTEXT('',#{app},'mechanical')"));
    let product = w.add(format!("PRODUCT('{name}','{name}','',(#{pc}))"));
    w.add(format!("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{product}))"));
    let pdf = w.add(format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
    let pdc = w.add(format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{app},'design')"));
    w.add(format!("PRODUCT_DEFINITION('design','',#{pdf},#{pdc})"))
}

fn file_text(w: Writer, name: &str) -> String {
    format!(
        "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('gears2 gear'),'2;1');\nFILE_NAME('{name}.step','',(''),(''),'gears2','gears2','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));\nENDSEC;\nDATA;\n{}ENDSEC;\nEND-ISO-10303-21;\n",
        w.out
    )
}

/// A STEP file holding `solids` (one part named `name`), drawn to within
/// `tolerance` mm (written as the file's distance accuracy).
pub fn step_file(solids: &[Solid], name: &str, tolerance: f64) -> String {
    let name = name.replace('\'', "");
    let mut w = Writer { out: String::new(), next: 1 };
    let (app, ctx) = contexts(&mut w, tolerance);
    let pd = product(&mut w, app, &name);
    let pds = w.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{pd})"));
    let origin = w.placement([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let mut items = Vec::new();
    for s in solids {
        items.push(w.solid(s));
    }
    items.push(origin);
    let rep = w.add(format!("ADVANCED_BREP_SHAPE_REPRESENTATION('{name}',({}),#{ctx})", Writer::list(&items)));
    w.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})"));
    file_text(w, &name)
}

/// A STEP assembly named `name`: each part (name, solid) written once, and
/// placed once per instance (part index, placement). Identical gears (the
/// planets of a planetary set) cost one copy of their geometry.
///
/// Structure, as CAD programs write it: a product per part with its solid
/// (an ADVANCED_BREP_SHAPE_REPRESENTATION, with the part's origin), an
/// assembly product whose SHAPE_REPRESENTATION holds a placement per
/// instance, and per instance a NEXT_ASSEMBLY_USAGE_OCCURRENCE with the
/// transformation from the part's origin to that placement.
pub fn step_assembly(parts: &[(&str, &Solid)], instances: &[(usize, Placement)], name: &str, tolerance: f64) -> String {
    let clean = |s: &str| s.replace('\'', "");
    let name = clean(name);
    let mut w = Writer { out: String::new(), next: 1 };
    let (app, ctx) = contexts(&mut w, tolerance);
    let assembly = product(&mut w, app, &name);
    let assembly_pds = w.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{assembly})"));
    let mut part_defs = Vec::new();
    for (part_name, solid) in parts {
        let part_name = clean(part_name);
        let pd = product(&mut w, app, &part_name);
        let pds = w.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{pd})"));
        let origin = w.placement([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
        let brep = w.solid(solid);
        let rep = w.add(format!("ADVANCED_BREP_SHAPE_REPRESENTATION('{part_name}',(#{brep},#{origin}),#{ctx})"));
        w.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})"));
        part_defs.push((pd, rep, origin));
    }
    let origin = w.placement([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let places: Vec<usize> = instances.iter().map(|(_, p)| w.placement(p.origin, p.axis, p.x_dir)).collect();
    let mut items = vec![origin];
    items.extend(&places);
    let assembly_rep = w.add(format!("SHAPE_REPRESENTATION('{name}',({}),#{ctx})", Writer::list(&items)));
    w.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{assembly_pds},#{assembly_rep})"));
    for (k, (&(part, _), &place)) in instances.iter().zip(&places).enumerate() {
        let (pd, rep, part_origin) = part_defs[part];
        let label = format!("{}:{}", clean(parts[part].0), k + 1);
        let nauo = w.add(format!("NEXT_ASSEMBLY_USAGE_OCCURRENCE('{label}','{label}','',#{assembly},#{pd},$)"));
        let pds = w.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{nauo})"));
        let idt = w.add(format!("ITEM_DEFINED_TRANSFORMATION('','',#{part_origin},#{place})"));
        let rel = w.add(format!(
            "(REPRESENTATION_RELATIONSHIP('','',#{rep},#{assembly_rep})REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{idt})SHAPE_REPRESENTATION_RELATIONSHIP())"
        ));
        w.add(format!("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{rel},#{pds})"));
    }
    file_text(w, &name)
}
