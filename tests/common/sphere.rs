//! The rotating mesh test for bevel pairs, on the sphere.
//!
//! The same checks as the plane test (see super::sweep and super::problems),
//! made on the unit sphere about the apex, in gear 1's frame:
//! - the mesh's walls are made of planes through the apex, so on the sphere
//!   the outlines are polygons of great circle arcs;
//! - the lines of action are great circles through the pitch point;
//! - gaps along them are angles along the great circle, and distances are
//!   angles between points and arcs, all exact on the sphere, reported in mm
//!   at the heel (times the cone distance R);
//! - "is this point inside that gear" is decided in the gear's own frame:
//!   its outline goes around the axis with growing azimuth, so the edge at
//!   the point's azimuth is found, and the point is in the material when its
//!   polar angle is smaller than the edge's there (exact for the great circle
//!   arc, at any angle from the axis).
//!
//! Every sphere about the apex shows the same pair (scaled, and turned by the
//! twist in step), so one sphere covers the whole face width.
#![allow(dead_code)]

use super::{min_of, DRIVE_GAP};
use gears2::bevel::*;
use std::f64::consts::PI;

pub struct BPair {
    pub gap: f64,
    /// For each gear: how far its flank is from crossing the line of action
    /// at a right angle (radians), and the error allowed for its chord.
    pub squareness: [(f64, f64); 2],
}

pub struct BStep {
    pub overlap: bool,
    pub drive: Vec<BPair>,
    pub coast: Vec<BPair>,
    pub elsewhere: f64,
    /// For drawing: the two outlines (world frame) and the contacts.
    pub outlines: [Vec<V>; 2],
    pub contacts: Vec<V>,
}

struct Cross {
    t: f64,
    point: V,
    owner: usize,
    tangent: V,
    length: f64,
}

fn modulo(a: f64, m: f64) -> f64 {
    let r = a % m;
    if r < 0.0 {
        r + m
    } else {
        r
    }
}

/// Angular distance from p to the great circle arc a-b.
pub fn arc_distance(p: V, a: V, b: V) -> f64 {
    let n = cross(a, b);
    if norm(n) < 1e-15 {
        return angle(p, a);
    }
    let n = unit(n);
    let q = sub(p, scale(n, dot(p, n)));
    if norm(q) > 1e-15 {
        let q = unit(q);
        if dot(cross(a, q), n) >= 0.0 && dot(cross(q, b), n) >= 0.0 {
            return dot(p, n).abs().min(1.0).asin();
        }
    }
    angle(p, a).min(angle(p, b))
}

/// Is p (in the gear's own frame) in the material of the outline (in order
/// of growing azimuth)? See the module documentation.
fn inside_outline(p: V, outline: &[V]) -> bool {
    let phi = azimuth(p);
    let n = outline.len();
    for i in 0..n {
        let (a, b) = (outline[i], outline[(i + 1) % n]);
        let (pa, step) = (azimuth(a), modulo(azimuth(b) - azimuth(a), 2.0 * PI));
        let off = modulo(phi - pa, 2.0 * PI);
        if step <= 0.0 || step >= PI || off >= step {
            continue;
        }
        // where the edge's great circle crosses the meridian at phi
        let m = [-phi.sin(), phi.cos(), 0.0];
        let mut x = unit(cross(cross(a, b), m));
        if x[0] * phi.cos() + x[1] * phi.sin() < 0.0 {
            x = scale(x, -1.0);
        }
        return polar(p) < polar(x);
    }
    false
}

pub struct Sweep<'a> {
    pub pair: &'a BevelPair,
    pub local: [Vec<V>; 2],
    pub window: f64,
}

impl<'a> Sweep<'a> {
    pub fn new(pair: &'a BevelPair) -> Self {
        let local = [pair.outline(0), pair.outline(1)];
        let g = &pair.gears[0];
        Sweep { pair, local, window: 2.0 * (g.tip - g.root) }
    }

    fn world(&self, which: usize, rotation: f64) -> Vec<V> {
        self.local[which].iter().map(|&v| self.pair.to_world(which, rotation, v)).collect()
    }

    fn zone_contains(&self, which: usize, x: V) -> bool {
        let g = &self.pair.gears[which];
        let theta = dot(x, self.pair.axis(which)).clamp(-1.0, 1.0).acos();
        let low = g.base.max(polar(g.p));
        low + 1e-9 < theta && theta < g.tip - 1e-9
    }

    fn crossings(&self, outlines: &[Vec<V>; 2], sign: f64, reach: f64) -> Vec<Cross> {
        let (pitch, d) = self.pair.line_of_action(sign);
        let nl = unit(cross(pitch, d));
        let mut out = Vec::new();
        for (owner, outline) in outlines.iter().enumerate() {
            let n = outline.len();
            for i in 0..n {
                let (a, b) = (outline[i], outline[(i + 1) % n]);
                if angle(a, pitch) > reach + 0.05 {
                    continue;
                }
                let ne = cross(a, b);
                if norm(ne) < 1e-15 {
                    continue;
                }
                let ne = unit(ne);
                let c = cross(nl, ne);
                if norm(c) < 1e-15 {
                    continue;
                }
                let mut x = unit(c);
                if dot(x, pitch) < 0.0 {
                    x = scale(x, -1.0);
                }
                if !(dot(cross(a, x), ne) >= 0.0 && dot(cross(x, b), ne) > 0.0) {
                    continue;
                }
                let t = dot(x, d).atan2(dot(x, pitch));
                if t.abs() <= reach {
                    out.push(Cross { t, point: x, owner, tangent: unit(cross(ne, x)), length: angle(a, b) });
                }
            }
        }
        out.sort_by(|p, q| p.t.partial_cmp(&q.t).unwrap().then(p.owner.cmp(&q.owner)));
        out
    }

    /// Facing flank pairs along both lines of action: [line +, line -].
    fn contact_gaps(&self, outlines: &[Vec<V>; 2]) -> [Vec<BPair>; 2] {
        let mut result: [Vec<BPair>; 2] = [Vec::new(), Vec::new()];
        for (line, sign) in [(0, 1.0), (1, -1.0)] {
            let (pitch, d) = self.pair.line_of_action(sign);
            let nl = unit(cross(pitch, d));
            let cr = self.crossings(outlines, sign, self.window);
            let tl = |x: V| unit(cross(nl, x));
            let facing = |c: &Cross| dot(c.tangent, tl(c.point)).abs() < (15f64).to_radians().sin() && self.zone_contains(c.owner, c.point);
            let square = |c: &Cross| {
                let g = &self.pair.gears[c.owner];
                let theta = dot(c.point, self.pair.axis(c.owner)).clamp(-1.0, 1.0).acos();
                let tau = g.flank.tau_at(theta).max(1e-12);
                (dot(c.tangent, tl(c.point)).abs().min(1.0).asin(), c.length / tau.tan())
            };
            for w in cr.windows(2) {
                let (c1, c2) = (&w[0], &w[1]);
                if c1.owner != c2.owner && facing(c1) && facing(c2) {
                    let (ca, cb) = if c1.owner == 0 { (c1, c2) } else { (c2, c1) };
                    result[line].push(BPair { gap: (c2.t - c1.t) * self.pair.r_cone, squareness: [square(ca), square(cb)] });
                }
            }
        }
        result
    }

    fn side_of(&self, which: usize, rotation: f64, x: V) -> f64 {
        let step = 2.0 * PI / self.pair.gears[which].teeth as f64;
        let v = modulo(azimuth(self.pair.from_world(which, rotation, x)) + step / 2.0, step) - step / 2.0;
        if v > 0.0 {
            1.0
        } else if v < 0.0 {
            -1.0
        } else {
            0.0
        }
    }

    fn near(&self, pts: &[V]) -> Vec<V> {
        let pitch = sph(self.pair.gears[0].delta, 0.0);
        pts.iter().copied().filter(|&p| angle(p, pitch) < self.window).collect()
    }

    fn inside(&self, which: usize, rotation: f64, x: V) -> bool {
        inside_outline(self.pair.from_world(which, rotation, x), &self.local[which])
    }

    fn min_distance(&self, pts: &[V], outline: &[V]) -> f64 {
        let pitch = sph(self.pair.gears[0].delta, 0.0);
        let n = outline.len();
        let edges: Vec<(V, V)> = (0..n)
            .filter(|&i| angle(outline[i], pitch) < self.window + 0.05)
            .map(|i| (outline[i], outline[(i + 1) % n]))
            .collect();
        min_of(pts.iter().flat_map(|&p| edges.iter().map(move |&(a, b)| arc_distance(p, a, b)))) * self.pair.r_cone
    }

    /// Gear 1 drives gear 2 through one tooth of gear 1 (see super::sweep).
    pub fn run(&self, steps: usize) -> Vec<BStep> {
        let pair = self.pair;
        let alpha = pair.pressure_angle;
        let r = pair.r_cone;
        let shift = (pair.backlash / 2.0 - DRIVE_GAP / alpha.cos()) / (r * pair.gears[1].delta.sin());
        let frame = |rot1: f64, sign: f64| {
            let rot2 = pair.mesh_rotation(rot1) + sign * shift;
            let outlines = [self.world(0, rot1), self.world(1, rot2)];
            let lines = self.contact_gaps(&outlines);
            (outlines, rot2, lines)
        };
        let drive_setup = |sign: f64| -> Option<(usize, [f64; 2])> {
            let (outlines, rot2, lines) = frame(0.0, sign);
            let smallest = |k: usize| min_of(lines[k].iter().map(|p| p.gap)).min(1e9);
            let drive_line = if smallest(1) < smallest(0) { 1 } else { 0 };
            let cr = self.crossings(&outlines, if drive_line == 0 { 1.0 } else { -1.0 }, self.window);
            let mut contact: [Vec<V>; 2] = [Vec::new(), Vec::new()];
            for w in cr.windows(2) {
                if w[0].owner != w[1].owner && (w[1].t - w[0].t) * r < pair.backlash * alpha.cos() / 2.0 {
                    contact[w[0].owner].push(w[0].point);
                    contact[w[1].owner].push(w[1].point);
                }
            }
            if contact[0].is_empty() {
                return None;
            }
            let mut sides = [0.0; 2];
            for (owner, rotation) in [(0, 0.0), (1, rot2)] {
                let s: Vec<f64> = contact[owner].iter().map(|&x| self.side_of(owner, rotation, x)).collect();
                assert!(s.iter().all(|&x| x == s[0]), "drive flank not found");
                sides[owner] = s[0];
            }
            Some((drive_line, sides))
        };
        let (sign, drive_line, drive_side) = [1.0, -1.0]
            .into_iter()
            .find_map(|sign| match drive_setup(sign) {
                Some((line, sides)) if sides[0] == 1.0 => Some((sign, line, sides)),
                _ => None,
            })
            .expect("couldn't set up gear 1 driving gear 2");

        (0..steps)
            .map(|i| {
                let rot1 = 2.0 * PI / pair.gears[0].teeth as f64 * i as f64 / steps as f64;
                let (outlines, rot2, lines) = frame(rot1, sign);
                let rots = [rot1, rot2];
                let near = [self.near(&outlines[0]), self.near(&outlines[1])];
                let overlap = near[0].iter().any(|&x| self.inside(1, rot2, x)) || near[1].iter().any(|&x| self.inside(0, rot1, x));
                let rest = |k: usize| -> Vec<V> {
                    near[k].iter().copied().filter(|&x| self.side_of(k, rots[k], x) != drive_side[k]).collect()
                };
                let elsewhere = self.min_distance(&rest(0), &outlines[1]).min(self.min_distance(&rest(1), &outlines[0]));
                let cr = self.crossings(&outlines, if drive_line == 0 { 1.0 } else { -1.0 }, self.window);
                let contacts = cr
                    .windows(2)
                    .filter(|w| w[0].owner != w[1].owner && (w[1].t - w[0].t) * r < pair.backlash * alpha.cos() / 4.0)
                    .map(|w| w[0].point)
                    .collect();
                let [l0, l1] = lines;
                let (drive, coast) = if drive_line == 0 { (l0, l1) } else { (l1, l0) };
                BStep { overlap, drive, coast, elsewhere, outlines, contacts }
            })
            .collect()
    }
}

/// What's wrong with a bevel sweep (see super::problems; gaps in mm at the
/// heel).
pub fn problems(pair: &BevelPair, results: &[BStep]) -> Vec<String> {
    const SHRINK_TOL: f64 = 1e-6;
    const ANGLE_TOL: f64 = 1e-4;
    let alpha = pair.pressure_angle;
    let coast = pair.backlash * alpha.cos() - DRIVE_GAP;
    let grow = 2.0 * pair.dev * pair.r_cone + SHRINK_TOL;
    let mut out = Vec::new();
    for (i, r) in results.iter().enumerate() {
        if r.overlap {
            out.push(format!("step {}: outlines overlap", i));
        }
        if r.drive.is_empty() {
            out.push(format!("step {}: no flanks in contact on the drive side", i));
        }
        for p in &r.drive {
            if !(DRIVE_GAP - SHRINK_TOL <= p.gap && p.gap <= DRIVE_GAP + grow) {
                out.push(format!("step {}: drive-side gap {:.5} (should be {})", i, p.gap, DRIVE_GAP));
            }
            for (gear, (off, bound)) in ["1", "2"].iter().zip(p.squareness) {
                if off > bound + ANGLE_TOL {
                    out.push(format!(
                        "step {}: gear {}'s flank crosses the line of action {:.3} deg from square (outline allows {:.3})",
                        i,
                        gear,
                        off.to_degrees(),
                        bound.to_degrees()
                    ));
                }
            }
        }
        for p in &r.coast {
            if !(coast - SHRINK_TOL <= p.gap && p.gap <= coast + grow) {
                out.push(format!("step {}: coast-side gap {:.5} (should be {:.5})", i, p.gap, coast));
            }
        }
        if r.elsewhere < coast - SHRINK_TOL {
            out.push(format!("step {}: gap elsewhere {:.5} (at least {:.5})", i, r.elsewhere, coast));
        }
    }
    out
}
