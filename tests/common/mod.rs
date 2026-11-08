//! Helpers for checking 2D gear outlines and pairs of meshing gears.
#![allow(dead_code)]

use gears2::gears::{GearSystem, Params, P};
use std::f64::consts::PI;

pub fn deg(d: f64) -> f64 {
    d * PI / 180.0
}

/// A system with pitch 2 pi (module 2), backlash 0.2, and the given values.
pub fn params(pressure_angle_deg: f64, height: f64, max_deviation: f64, ds: f64) -> Params {
    Params::new(2.0 * PI, deg(pressure_angle_deg), height, 0.2, max_deviation, ds)
}

pub fn system(p: Params) -> GearSystem {
    GearSystem::new(&p).expect("a valid system")
}

pub fn assert_close(a: f64, b: f64, rel: f64, what: &str) {
    assert!((a - b).abs() <= rel * a.abs().max(b.abs()).max(1e-300), "{}: {} vs {}", what, a, b);
}

pub fn segment_lengths(points: &[P]) -> Vec<f64> {
    let n = points.len();
    (0..n).map(|i| (points[(i + 1) % n].0 - points[i].0).hypot(points[(i + 1) % n].1 - points[i].1)).collect()
}

pub fn min_of(values: impl IntoIterator<Item = f64>) -> f64 {
    values.into_iter().fold(f64::INFINITY, f64::min)
}

pub fn max_of(values: impl IntoIterator<Item = f64>) -> f64 {
    values.into_iter().fold(f64::NEG_INFINITY, f64::max)
}

fn modulo(a: f64, m: f64) -> f64 {
    let r = a % m;
    if r < 0.0 {
        r + m
    } else {
        r
    }
}

/// Number of steps where the polar angle goes backwards (should be 0 for a
/// star-shaped outline traversed counterclockwise).
pub fn backward_steps(points: &[P]) -> usize {
    let n = points.len();
    let angles: Vec<f64> = points.iter().map(|p| p.1.atan2(p.0)).collect();
    (0..n)
        .filter(|&i| {
            let step = modulo(angles[(i + 1) % n] - angles[i], 2.0 * PI);
            PI < step && step < 2.0 * PI - 1e-9
        })
        .count()
}

/// Rotate an outline about the origin, then move it to `center`.
pub fn place(points: &[P], rotation: f64, center: P) -> Vec<P> {
    let (c, s) = (rotation.cos(), rotation.sin());
    points.iter().map(|&(x, y)| (x * c - y * s + center.0, x * s + y * c + center.1)).collect()
}

/// Which points lie inside the closed polygon (ray casting).
pub fn inside(points: &[P], polygon: &[P]) -> Vec<bool> {
    let n = polygon.len();
    points
        .iter()
        .map(|&(x, y)| {
            let mut count = 0;
            for i in 0..n {
                let ((x1, y1), (x2, y2)) = (polygon[i], polygon[(i + 1) % n]);
                if (y1 > y) != (y2 > y) && x < x1 + (y - y1) * (x2 - x1) / (y2 - y1) {
                    count += 1;
                }
            }
            count % 2 == 1
        })
        .collect()
}

fn segment_distance(p: P, a: P, b: P) -> f64 {
    let (abx, aby) = (b.0 - a.0, b.1 - a.1);
    let t = (((p.0 - a.0) * abx + (p.1 - a.1) * aby) / (abx * abx + aby * aby)).clamp(0.0, 1.0);
    (p.0 - a.0 - t * abx).hypot(p.1 - a.1 - t * aby)
}

/// Distance from each point to the closed polyline.
pub fn distances(points: &[P], polygon: &[P]) -> Vec<f64> {
    let n = polygon.len();
    points
        .iter()
        .map(|&p| min_of((0..n).map(|i| segment_distance(p, polygon[i], polygon[(i + 1) % n]))))
        .collect()
}

/// Largest distance from a point of `fine` (on the true curve) to the closed
/// polyline `coarse`.
pub fn max_deviation(fine: &[P], coarse: &[P]) -> f64 {
    max_of(distances(fine, coarse))
}

/// Smallest distance from any of `points` to any edge of the closed polygon.
/// With a center and radius, only edges starting within radius of center count.
pub fn min_distance(points: &[P], polygon: &[P], within: Option<(P, f64)>) -> f64 {
    let n = polygon.len();
    let edges: Vec<(P, P)> = (0..n)
        .filter(|&i| match within {
            Some((c, r)) => (polygon[i].0 - c.0).hypot(polygon[i].1 - c.1) < r,
            None => true,
        })
        .map(|i| (polygon[i], polygon[(i + 1) % n]))
        .collect();
    min_of(points.iter().flat_map(|&p| edges.iter().map(move |&(a, b)| segment_distance(p, a, b))))
}

pub fn near(points: &[P], center: P, radius: f64) -> Vec<P> {
    points.iter().copied().filter(|p| (p.0 - center.0).hypot(p.1 - center.1) < radius).collect()
}

/// Where the line origin + t * direction crosses a polygon edge.
#[derive(Debug, Clone, Copy)]
pub struct Crossing {
    pub t: f64,
    pub point: P,
    pub edge: P,
    pub length: f64,
}

/// Crossings of the line origin + t * direction with the polygon's edges,
/// for |t| <= reach.
pub fn line_crossings(polygon: &[P], origin: P, direction: P, reach: f64) -> Vec<Crossing> {
    let n = polygon.len();
    let (dx, dy) = direction;
    let mut out = Vec::new();
    for i in 0..n {
        let a = polygon[i];
        let e = (polygon[(i + 1) % n].0 - a.0, polygon[(i + 1) % n].1 - a.1);
        let w = (a.0 - origin.0, a.1 - origin.1);
        let denom = dx * e.1 - dy * e.0;
        if denom.abs() <= 1e-15 {
            continue;
        }
        let t = (w.0 * e.1 - w.1 * e.0) / denom;
        let s = (w.0 * dy - w.1 * dx) / denom;
        if (0.0..1.0).contains(&s) && t.abs() <= reach {
            let length = e.0.hypot(e.1);
            out.push(Crossing { t, point: (origin.0 + t * dx, origin.1 + t * dy), edge: (e.0 / length, e.1 / length), length });
        }
    }
    out
}

/// The involute part of a gear's flanks: radii strictly between r_low and
/// r_high from the gear's center.
#[derive(Debug, Clone, Copy)]
pub struct Zone {
    pub center: P,
    pub r_low: f64,
    pub r_high: f64,
    pub r_base: f64,
}

impl Zone {
    pub fn radius(&self, p: P) -> f64 {
        (p.0 - self.center.0).hypot(p.1 - self.center.1)
    }

    pub fn contains(&self, p: P) -> bool {
        let r = self.radius(p);
        self.r_low + 1e-6 < r && r < self.r_high - 1e-6
    }
}

/// A facing flank pair along a line of action: the gap along the line, and
/// for each gear the pressure angle measured there with its allowed error.
#[derive(Debug, Clone)]
pub struct Pair {
    pub gap: f64,
    pub angles: [(f64, f64); 2],
}

fn sorted_crossings(a: &[P], b: &[P], origin: P, d: P, reach: f64) -> Vec<(Crossing, usize)> {
    let mut all: Vec<(Crossing, usize)> = line_crossings(a, origin, d, reach).into_iter().map(|c| (c, 0)).collect();
    all.extend(line_crossings(b, origin, d, reach).into_iter().map(|c| (c, 1)));
    all.sort_by(|x, y| x.0.t.partial_cmp(&y.0.t).unwrap().then(x.1.cmp(&y.1)));
    all
}

/// Gaps between the flanks that should be in contact, along each line of
/// action.
///
/// Both lines of action pass through the pitch point at angle alpha to the
/// common tangent (the y direction, since the centers are on the x-axis).
/// Every involute flank of the matching side crosses its line of action at a
/// right angle, and along it neighbouring flanks of the two gears are exactly
/// backlash * cos(alpha) / 2 apart (centered mesh). Crossings of the other
/// flanks cross at about 2 * alpha from square and are skipped (the cut-off,
/// 15 degrees from square, is loose so it doesn't decide the measured angle).
/// The zones say whether a crossing point lies on the involute part of that
/// gear's flank.
///
/// For each facing pair it also measures the pressure angle on both flanks:
/// the flank's normal at the crossing (perpendicular to the outline edge
/// there) should make the angle alpha with the common tangent, i.e. point
/// along the line of action. The outline edge is a chord of the involute, so
/// its direction can differ from the true tangent at the crossing by up to
/// (edge length) / (curvature radius). An involute's curvature radius at
/// radius R is sqrt(R^2 - r_base^2), the unwound string length; that bound is
/// returned as the allowed error.
pub fn contact_gaps(a: &[P], b: &[P], pitch_point: P, alpha: f64, zones: [Zone; 2], reach: f64) -> [Vec<Pair>; 2] {
    let mut result: [Vec<Pair>; 2] = [Vec::new(), Vec::new()];
    for (line, sign) in [(0, 1.0), (1, -1.0)] {
        let d = (sign * alpha.sin(), alpha.cos());
        let crossings = sorted_crossings(a, b, pitch_point, d, reach);
        let facing = |c: &(Crossing, usize)| {
            (c.0.edge.0 * d.0 + c.0.edge.1 * d.1).abs() < deg(15.0).sin() && zones[c.1].contains(c.0.point)
        };
        let angle_and_bound = |c: &(Crossing, usize)| {
            let zone = zones[c.1];
            // normal (-e_y, e_x); its angle from the y direction
            let angle = c.0.edge.1.abs().atan2(c.0.edge.0.abs());
            let curvature = (zone.radius(c.0.point).powi(2) - zone.r_base.powi(2)).max(1e-12).sqrt();
            (angle, c.0.length / curvature)
        };
        for w in crossings.windows(2) {
            let (c1, c2) = (&w[0], &w[1]);
            if c1.1 != c2.1 && facing(c1) && facing(c2) {
                let (ca, cb) = if c1.1 == 0 { (c1, c2) } else { (c2, c1) };
                result[line].push(Pair { gap: c2.0.t - c1.0.t, angles: [angle_and_bound(ca), angle_and_bound(cb)] });
            }
        }
    }
    result
}

/// Gap and overlap between two placed outlines near the pitch point: the
/// placed outlines, the smallest distance between them within `window` of the
/// pitch point, and whether any vertex of one lies in the other's material.
/// Gear A is always a spur gear. If B is a ring, its material is outside its
/// outline, so A's vertices must stay inside it.
pub fn check_mesh(
    outline_a: &[P],
    outline_b: &[P],
    place_a: (f64, P),
    place_b: (f64, P),
    pitch_point: P,
    window: f64,
    b_is_ring: bool,
) -> (Vec<P>, Vec<P>, f64, bool) {
    let a = place(outline_a, place_a.0, place_a.1);
    let b = place(outline_b, place_b.0, place_b.1);
    let (a_near, b_near) = (near(&a, pitch_point, window), near(&b, pitch_point, window));
    let overlapping = overlap(&a_near, &b_near, &a, &b, b_is_ring);
    // Edges slightly beyond the window still count, so nothing near its edge is missed.
    let reach = window + 1.0;
    let gap = min_distance(&a_near, &b, Some((pitch_point, reach))).min(min_distance(&b_near, &a, Some((pitch_point, reach))));
    (a, b, gap, overlapping)
}

fn overlap(a_near: &[P], b_near: &[P], a: &[P], b: &[P], b_is_ring: bool) -> bool {
    let a_in_b = inside(a_near, b);
    let b_in_a = inside(b_near, a).into_iter().any(|x| x);
    if b_is_ring {
        a_in_b.into_iter().any(|x| !x) || b_in_a
    } else {
        a_in_b.into_iter().any(|x| x) || b_in_a
    }
}

/// Involute part of a spur gear's flank: above the base circle and the top of
/// the fillet, below the tip circle.
pub fn spur_zone(s: &GearSystem, teeth: usize, center: P) -> Zone {
    let g = s.spur_profile(teeth).unwrap();
    Zone { center, r_low: g.r_base.max(g.p.0.hypot(g.p.1)), r_high: g.r_tip, r_base: g.r_base }
}

/// Involute part of a ring gear's flank: outside its tip and base circles,
/// inside the top of its fillet.
pub fn ring_zone(s: &GearSystem, teeth: usize) -> Zone {
    let g = s.ring_profile(teeth).unwrap();
    Zone { center: (0.0, 0.0), r_low: g.r_tip.max(g.r_base), r_high: g.p.0.hypot(g.p.1), r_base: g.r_base }
}

/// In the driven test the drive-side flanks are left this far apart instead of
/// exactly touching, so rounding can't make them read as overlapping. (The
/// outlines never extend past the true shapes, so nothing else can.)
pub const DRIVE_GAP: f64 = 1e-4;

/// +1 or -1 for each point (0 exactly on a center line): which side of the
/// nearest tooth center line (spur) or gap center line (ring) it is on. Tooth /
/// gap 0 of an unrotated outline is centered on +x, so the center lines are at
/// rotation + 2 pi k / teeth.
pub fn side_of(points: &[P], center: P, rotation: f64, teeth: usize) -> Vec<f64> {
    let step = 2.0 * PI / teeth as f64;
    points
        .iter()
        .map(|p| {
            let theta = (p.1 - center.1).atan2(p.0 - center.0) - rotation;
            let v = modulo(theta + step / 2.0, step) - step / 2.0;
            if v > 0.0 {
                1.0
            } else if v < 0.0 {
                -1.0
            } else {
                0.0
            }
        })
        .collect()
}

/// One step of a driven sweep.
#[derive(Debug, Clone)]
pub struct Step {
    pub overlap: bool,
    pub drive: Vec<Pair>,
    pub coast: Vec<Pair>,
    pub elsewhere: f64,
}

/// Driven rotating test, shared by the spur/spur and spur/ring cases.
///
/// rotations(rot_a) gives B's centered rotation for A's rotation. B is then
/// turned back by just enough to close the play on one side (the drive side)
/// down to DRIVE_GAP: the centered mesh leaves backlash/2 of arc on the pitch
/// circle on each side, which is backlash cos(alpha)/2 along the line of
/// action.
#[allow(clippy::too_many_arguments)]
fn sweep(
    s: &GearSystem,
    outlines: (&[P], &[P]),
    teeth: (usize, usize),
    centers: (P, P),
    rotations: &dyn Fn(f64) -> f64,
    pitch_point: P,
    zones: [Zone; 2],
    b_is_ring: bool,
    steps: usize,
) -> Vec<Step> {
    let alpha = s.pressure_angle;
    let (a_out, b_out) = outlines;
    let (za, zb) = teeth;
    let window = 2.0 * (s.height + s.clearance);
    let shift = (s.backlash / 2.0 - DRIVE_GAP / alpha.cos()) / s.pitch_radius(zb);

    let frame = |rot_a: f64, sign: f64| {
        let rot_b = rotations(rot_a) + sign * shift;
        let a = place(a_out, rot_a, centers.0);
        let b = place(b_out, rot_b, centers.1);
        let lines = contact_gaps(&a, &b, pitch_point, alpha, zones, window);
        (a, b, rot_b, lines)
    };

    // With B turned back by sign * shift: the drive line of action (the one
    // whose facing flanks come together) and the drive flank of each gear (+1 /
    // -1: the side of its tooth or gap center line that the drive-side contacts
    // are on), or None if neither line closes up. Contacts are the neighbouring
    // crossings of the drive line, one from each gear, that are nearly
    // touching. (Other crossings won't do: on short teeth a coast flank near
    // its base circle can also cross the line nearly square.)
    let drive_setup = |sign: f64| -> Option<(usize, [f64; 2])> {
        let (a, b, rot_b, lines) = frame(0.0, sign);
        let smallest = |k: usize| min_of(lines[k].iter().map(|p| p.gap)).min(1e9);
        let drive_line = if smallest(1) < smallest(0) { 1 } else { 0 };
        let d = (if drive_line == 0 { alpha.sin() } else { -alpha.sin() }, alpha.cos());
        let crossings = sorted_crossings(&a, &b, pitch_point, d, window);
        let mut contact: [Vec<P>; 2] = [Vec::new(), Vec::new()];
        for w in crossings.windows(2) {
            if w[0].1 != w[1].1 && w[1].0.t - w[0].0.t < s.backlash * alpha.cos() / 2.0 {
                contact[w[0].1].push(w[0].0.point);
                contact[w[1].1].push(w[1].0.point);
            }
        }
        if contact[0].is_empty() {
            return None;
        }
        let mut drive_side = [0.0; 2];
        for (owner, rotation, center, z) in [(0, 0.0, centers.0, za), (1, rot_b, centers.1, zb)] {
            let sides = side_of(&contact[owner], center, rotation, z);
            assert!(sides.iter().all(|&x| x == sides[0]), "drive flank not found");
            drive_side[owner] = sides[0];
        }
        Some((drive_line, drive_side))
    };

    // A turns counterclockwise and drives B, so A's leading flank (its upper
    // flank, on the +theta side of each tooth) is the drive flank. Turn B back
    // whichever way puts the contact there.
    let (sign, drive_line, drive_side) = [1.0, -1.0]
        .into_iter()
        .find_map(|sign| match drive_setup(sign) {
            Some((line, sides)) if sides[0] == 1.0 => Some((sign, line, sides)),
            _ => None,
        })
        .expect("couldn't set up A driving B");

    (0..steps)
        .map(|i| {
            let rot_a = 2.0 * PI / za as f64 * i as f64 / steps as f64;
            let (a, b, rot_b, lines) = frame(rot_a, sign);
            let (a_near, b_near) = (near(&a, pitch_point, window), near(&b, pitch_point, window));
            let overlapping = overlap(&a_near, &b_near, &a, &b, b_is_ring);
            // Everywhere else: every point that isn't on a drive-side flank (or
            // the drive-side half of a land), against the whole other outline.
            let a_sides = side_of(&a_near, centers.0, rot_a, za);
            let b_sides = side_of(&b_near, centers.1, rot_b, zb);
            let a_rest: Vec<P> = a_near.iter().zip(a_sides).filter(|(_, s)| *s != drive_side[0]).map(|(p, _)| *p).collect();
            let b_rest: Vec<P> = b_near.iter().zip(b_sides).filter(|(_, s)| *s != drive_side[1]).map(|(p, _)| *p).collect();
            let reach = window + 1.0;
            let elsewhere = min_distance(&a_rest, &b, Some((pitch_point, reach))).min(min_distance(&b_rest, &a, Some((pitch_point, reach))));
            let [line0, line1] = lines;
            let (drive, coast) = if drive_line == 0 { (line0, line1) } else { (line1, line0) };
            Step { overlap: overlapping, drive, coast, elsewhere }
        })
        .collect()
}

/// Gear A drives gear B through one tooth of A.
pub fn sweep_spur_pair(s: &GearSystem, teeth_a: usize, teeth_b: usize, steps: usize) -> Vec<Step> {
    sweep_spur_outlines(s, &s.spur_gear(teeth_a).unwrap(), teeth_a, teeth_b, steps)
}

/// The same with gear A's outline given (for example a refined face boundary).
pub fn sweep_spur_outlines(s: &GearSystem, outline_a: &[P], teeth_a: usize, teeth_b: usize, steps: usize) -> Vec<Step> {
    let r_a = s.pitch_radius(teeth_a);
    let d = r_a + s.pitch_radius(teeth_b);
    let b = s.spur_gear(teeth_b).unwrap();
    sweep(
        s,
        (outline_a, &b),
        (teeth_a, teeth_b),
        ((0.0, 0.0), (d, 0.0)),
        &|rot_a| GearSystem::spur_mesh_rotation(teeth_a, rot_a, teeth_b, 0.0),
        (r_a, 0.0),
        [spur_zone(s, teeth_a, (0.0, 0.0)), spur_zone(s, teeth_b, (d, 0.0))],
        false,
        steps,
    )
}

/// The spur gear drives the ring gear through one tooth of the spur gear. The
/// ring is at the origin, the spur gear's center on the +x axis.
pub fn sweep_ring_pair(s: &GearSystem, teeth_planet: usize, teeth_ring: usize, steps: usize) -> Vec<Step> {
    let r_ring = s.pitch_radius(teeth_ring);
    let a = r_ring - s.pitch_radius(teeth_planet);
    let (p, r) = (s.spur_gear(teeth_planet).unwrap(), s.ring_gear(teeth_ring).unwrap());
    sweep(
        s,
        (&p, &r),
        (teeth_planet, teeth_ring),
        ((a, 0.0), (0.0, 0.0)),
        &|rot_p| GearSystem::ring_mesh_rotation(teeth_planet, rot_p, teeth_ring, 0.0),
        (r_ring, 0.0),
        [spur_zone(s, teeth_planet, (a, 0.0)), ring_zone(s, teeth_ring)],
        true,
        steps,
    )
}

/// What's wrong with a driven sweep, as a list of messages (empty if clean).
///
/// - Drive side: along its line of action there is at least one pair of
///   facing flanks, and every facing pair is DRIVE_GAP apart.
/// - Pressure angle: at those contacts, both flanks' normals point along the
///   line of action (within the error of measuring a curve with straight
///   outline segments).
/// - Coast side: every facing pair along the other line of action is the
///   whole backlash apart, backlash * cos(alpha) - DRIVE_GAP.
/// - Everywhere else: nothing that isn't on a drive-side flank comes closer
///   than that, and nothing overlaps.
///
/// The outlines never extend past the true shapes and stray from them by at
/// most max_deviation, so a gap can't shrink, and can grow by at most
/// 2 * max_deviation.
pub fn problems(s: &GearSystem, results: &[Step]) -> Vec<String> {
    const SHRINK_TOL: f64 = 1e-6; // rounding only
    const ANGLE_TOL: f64 = 1e-4; // radians, beyond the outline's own chord error
    let alpha = s.pressure_angle;
    let coast = s.backlash * alpha.cos() - DRIVE_GAP;
    let grow = 2.0 * s.max_deviation + SHRINK_TOL;
    let mut out = Vec::new();
    for (i, r) in results.iter().enumerate() {
        if r.overlap {
            out.push(format!("step {}: outlines overlap", i));
        }
        if r.drive.is_empty() {
            out.push(format!("step {}: no flanks in contact on the drive side", i));
        }
        for pair in &r.drive {
            if !(DRIVE_GAP - SHRINK_TOL <= pair.gap && pair.gap <= DRIVE_GAP + grow) {
                out.push(format!("step {}: drive-side gap {:.4} (should be {})", i, pair.gap, DRIVE_GAP));
            }
            for (gear, (angle, bound)) in ["A", "B"].iter().zip(pair.angles) {
                if (angle - alpha).abs() > bound + ANGLE_TOL {
                    out.push(format!(
                        "step {}: pressure angle on {}'s flank {:.3} deg (expected {:.3}, outline allows +-{:.3})",
                        i,
                        gear,
                        angle.to_degrees(),
                        alpha.to_degrees(),
                        bound.to_degrees()
                    ));
                }
            }
        }
        for pair in &r.coast {
            if !(coast - SHRINK_TOL <= pair.gap && pair.gap <= coast + grow) {
                out.push(format!("step {}: coast-side gap {:.4} (should be {:.4})", i, pair.gap, coast));
            }
        }
        if r.elsewhere < coast - SHRINK_TOL {
            out.push(format!("step {}: gap elsewhere {:.4} (at least {:.4})", i, r.elsewhere, coast));
        }
    }
    out
}

pub fn assert_clean(s: &GearSystem, results: &[Step], label: &str) {
    let found = problems(s, results);
    assert!(found.is_empty(), "{}: {}", label, found.iter().take(5).cloned().collect::<Vec<_>>().join("; "));
}
