//! Bevel gears: a meshing pair of herringbone gears on cones whose apexes meet.
//!
//! All lengths are in mm and all angles in radians.
//!
//! # Geometry
//!
//! Each gear has its own frame: the apex at the origin and the axis along -z,
//! so the gear lies below the apex. A point on the sphere of radius rho about
//! the apex is given by its polar angle theta from the axis and its azimuth
//! phi (see `sph`). Gear 1's frame is the world frame; gear 2's axis is
//! turned by the shaft angle toward +x (see `BevelPair::to_world`), so the
//! pitch cones touch along the line at polar angle delta_1, azimuth 0.
//!
//! With teeth z_1, z_2 and shaft angle sigma, the pitch cones roll on each
//! other when z_1 / sin(delta_1) = z_2 / sin(delta_2) and delta_1 + delta_2 =
//! sigma, which gives tan(delta_1) = sin(sigma) / (z_2 / z_1 + cos(sigma)).
//! Tooth sizes shrink toward the apex in proportion to rho: everything is an
//! angle on the sphere. The gear set's module, tooth height, root gap and
//! root rounding are taken at the heel (the outer end, cone distance R =
//! module z / (2 sin(delta))), as is the surface accuracy, where errors are
//! largest; the point spacing and the shortest land must hold at the toe
//! (cone distance R - face width), where everything is smallest.
//!
//! # Teeth
//!
//! The flanks are spherical involutes: the curve traced on the sphere by a
//! great circle unrolling from the base cone, sin(delta_b) = sin(delta)
//! cos(alpha). Two spherical involutes drawn from the same great circle (the
//! line of action, through the pitch point at alpha to the pitch circles)
//! are conjugate, exactly as plane involutes are: turning a gear by dphi moves
//! its flank along the line of action by sin(delta_b) dphi. The azimuth of
//! the involute at polar angle theta is
//!
//! ```text
//! ev(theta) = acos(cos(theta) / cos(delta_b)) / sin(delta_b) - acos(tan(delta_b) / tan(theta)),
//! ```
//!
//! the spherical counterpart of inv(a) = tan(a) - a. Below the base cone the
//! flank continues along a meridian (a great circle through the axis).
//! Fillets are small circles tangent to the flank and the root circle, chosen
//! against the paths swept by the mate's tooth-tip corners, as for spur gears
//! (see gears.rs); tip and root lands are circles about the axis.
//!
//! Every piece of the outline is drawn so that, on the sphere, it stays on
//! the material side of the true curve: the straight edges between outline
//! points lie in planes through the apex, so on every sphere they are great
//! circle arcs. Great circle arcs between points of a convex piece (flank,
//! land) cut into the material; a concave piece (fillet) is a tangent
//! polygon of great circles.
//!
//! The herringbone twist keeps a constant spiral angle beta on the pitch cone:
//! a section at cone distance rho is turned by tan(beta) ln(rho / rho_0) /
//! sin(delta), rising from both ends of the face to the middle (in ln rho).
//! Both gears turn their sections in step (z_1 / sin(delta_1) = z_2 /
//! sin(delta_2)), so every sphere about the apex shows the same, correctly
//! meshing pair: the rotating tests check one sphere.

use crate::gears::{GearError, GearSystem, Result, MIN_LAND};
use crate::mesh::{stitch, triangulate_sized, IndexedMesh, DEFAULT_MIN_ANGLE};
use std::f64::consts::PI;

pub type V = [f64; 3];

fn err<T>(message: impl Into<String>) -> Result<T> {
    Err(GearError(message.into()))
}

// ------------------------------------------------------------------ vectors

pub fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn scale(a: V, k: f64) -> V {
    [a[0] * k, a[1] * k, a[2] * k]
}
pub fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub fn norm(a: V) -> f64 {
    dot(a, a).sqrt()
}
pub fn unit(a: V) -> V {
    scale(a, 1.0 / norm(a))
}
/// Angle between two directions.
pub fn angle(a: V, b: V) -> f64 {
    norm(cross(a, b)).atan2(dot(a, b))
}

/// Unit vector at polar angle theta (from the axis, -z) and azimuth phi.
pub fn sph(theta: f64, phi: f64) -> V {
    [theta.sin() * phi.cos(), theta.sin() * phi.sin(), -theta.cos()]
}
/// Polar angle from the axis (-z).
pub fn polar(v: V) -> f64 {
    (v[0].hypot(v[1])).atan2(-v[2])
}
pub fn azimuth(v: V) -> f64 {
    v[1].atan2(v[0])
}
pub fn rot_z(v: V, a: f64) -> V {
    let (c, s) = (a.cos(), a.sin());
    [v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]]
}
/// Turn about the y axis.
pub fn rot_y(v: V, a: f64) -> V {
    let (c, s) = (a.cos(), a.sin());
    [v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c]
}
fn mirror(v: V) -> V {
    [v[0], -v[1], v[2]]
}
/// Rodrigues: turn v about the unit axis k by angle a.
fn rotate_about(v: V, k: V, a: f64) -> V {
    let (c, s) = (a.cos(), a.sin());
    add(add(scale(v, c), scale(cross(k, v), s)), scale(k, dot(k, v) * (1.0 - c)))
}
/// Angular distance from q to the great circle through a and b.
pub fn off_great_circle(q: V, a: V, b: V) -> f64 {
    dot(q, unit(cross(a, b))).abs().min(1.0).asin()
}

// ------------------------------------------------------------- the involute

/// The upper flank (azimuth > 0) of a tooth centered at azimuth 0, on the
/// unit sphere: azimuth half_angle(theta) = constant - ev(theta).
///
/// Parametrized like the plane flank: s < 0 on the meridian below the base
/// cone (theta = delta_b + s); s >= 0 on the involute, s = tau^2 / 2 for the
/// unrolled angle tau, cos(theta) = cos(delta_b) cos(tau).
#[derive(Debug, Clone, Copy)]
pub struct SphFlank {
    pub base: f64,
    pub constant: f64,
}

impl SphFlank {
    pub fn ev(&self, theta: f64) -> f64 {
        let b = self.base;
        if theta <= b {
            return 0.0;
        }
        (theta.cos() / b.cos()).min(1.0).acos() / b.sin() - (b.tan() / theta.tan()).min(1.0).acos()
    }

    pub fn half_angle(&self, theta: f64) -> f64 {
        self.constant - self.ev(theta)
    }

    pub fn s_at(&self, theta: f64) -> f64 {
        if theta <= self.base {
            return theta - self.base;
        }
        let tau = (theta.cos() / self.base.cos()).min(1.0).acos();
        tau * tau / 2.0
    }

    pub fn theta_at(&self, s: f64) -> f64 {
        if s <= 0.0 {
            self.base + s
        } else {
            (self.base.cos() * (2.0 * s).sqrt().cos()).acos()
        }
    }

    /// The unrolled angle at polar angle theta (0 at and below the base cone).
    /// The flank's geodesic curvature there is cot(tau).
    pub fn tau_at(&self, theta: f64) -> f64 {
        if theta <= self.base {
            0.0
        } else {
            (theta.cos() / self.base.cos()).min(1.0).acos()
        }
    }

    pub fn at(&self, s: f64) -> V {
        let theta = self.theta_at(s);
        sph(theta, self.half_angle(theta))
    }

    /// Unit normal at s, tangent to the sphere, pointing into the air (toward
    /// larger azimuth, away from the tooth).
    pub fn normal(&self, s: f64) -> V {
        let h = 1e-7;
        let p = self.at(s);
        let t = sub(self.at(s + h), self.at(s - h));
        let mut n = unit(cross(p, t));
        let e_phi = [-azimuth(p).sin(), azimuth(p).cos(), 0.0];
        if dot(n, e_phi) < 0.0 {
            n = scale(n, -1.0);
        }
        n
    }

    /// Points from s_start to s_end (leaving out s_start), joined by great
    /// circle arcs that stay within dev of the flank (angles, unit sphere)
    /// and at least ds apart. The flank is convex seen from the tooth, so the
    /// arcs cut into the tooth.
    ///
    /// The meridian is a great circle: no points inside it. On the involute
    /// the points are spaced evenly in s^(3/4) (as for the plane involute,
    /// whose curvature radius grows like sqrt(s)); the fewest steps that keep
    /// every arc within dev are found by checking each arc at a few points,
    /// then reduced while any arc is shorter than ds.
    pub fn points(&self, s_start: f64, s_end: f64, dev: f64, ds: f64) -> Vec<V> {
        let (lo, hi) = (s_start.min(s_end), s_start.max(s_end));
        let mut values = vec![lo];
        if lo < 0.0 && 0.0 < hi && -lo >= ds {
            values.push(0.0);
        }
        if hi > 0.0 {
            let a = lo.max(0.0);
            let (u0, u1) = (a.powf(0.75), hi.powf(0.75));
            let s_of = |u: f64| u.powf(4.0 / 3.0);
            let within = |n: usize| -> bool {
                (0..n).all(|i| {
                    let (ua, ub) = (u0 + (u1 - u0) * i as f64 / n as f64, u0 + (u1 - u0) * (i + 1) as f64 / n as f64);
                    let (pa, pb) = (self.at(s_of(ua)), self.at(s_of(ub)));
                    (1..8).all(|k| off_great_circle(self.at(s_of(ua + (ub - ua) * k as f64 / 8.0)), pa, pb) <= dev)
                })
            };
            let mut n = 1;
            while !within(n) {
                n *= 2;
            }
            let (mut good, mut bad) = (n, n / 2);
            while good - bad > 1 {
                let mid = (good + bad) / 2;
                if within(mid) {
                    good = mid;
                } else {
                    bad = mid;
                }
            }
            let mut n = good;
            loop {
                let mut candidate = values.clone();
                for i in 1..=n {
                    candidate.push(s_of(u0 + (u1 - u0) * i as f64 / n as f64));
                }
                *candidate.last_mut().unwrap() = hi;
                let pts: Vec<V> = candidate.iter().map(|&s| self.at(s)).collect();
                let shortest = pts.windows(2).map(|w| angle(w[0], w[1])).fold(f64::INFINITY, f64::min);
                if shortest >= ds || n == 1 {
                    values = candidate;
                    break;
                }
                n -= 1;
            }
        } else if *values.last().unwrap() != hi {
            values.push(hi);
        }
        let mut pts: Vec<V> = values.iter().map(|&s| self.at(s)).collect();
        if s_start > s_end {
            pts.reverse();
        }
        pts.remove(0);
        pts
    }
}

/// Points along the small circle about `center` (unit) from `start` to `end`
/// (both on it), the short way; leaves out start, ends exactly at end.
///
/// inside: points on the circle joined by great circle arcs, which bend
/// toward the center (for material inside the circle). A great circle
/// through two points of a circle of angular radius r, t apart (seen from the
/// center), passes the center at angular distance atan(tan(r) cos(t / 2)).
///
/// outside: a tangent polygon of great circles (for material outside the
/// circle). Tangents at two points t apart meet at angular distance
/// atan(tan(r) / cos(t / 2)) from the center; the end segments, from a point
/// of the circle to the first corner, are atan(sin(r) tan(t / 2)) long.
pub fn sph_arc(center: V, start: V, end: V, dev: f64, ds: f64, outside: bool) -> Vec<V> {
    let r = angle(center, start);
    let (u, v) = (unit(sub(start, scale(center, dot(start, center)))), unit(sub(end, scale(center, dot(end, center)))));
    let sweep = dot(cross(u, v), center).atan2(dot(u, v));
    let mut points = Vec::new();
    if sweep.abs() < 1e-15 || r < 1e-15 {
        points.push(end);
        return points;
    }
    let tooth = |k: usize| -> (f64, f64) {
        let t = sweep.abs() / k as f64;
        if outside {
            (((r.tan() / (t / 2.0).cos()).atan() - r), (r.sin() * (t / 2.0).tan()).atan())
        } else {
            (r - (r.tan() * (t / 2.0).cos()).atan(), 2.0 * (r.sin() * (t / 2.0).sin()).asin())
        }
    };
    let mut k = 1;
    while tooth(k).0 > dev {
        k += 1;
    }
    while k > 1 && tooth(k).1 < ds {
        k -= 1;
    }
    let step = sweep / k as f64;
    if outside {
        let lift = (r.tan() / (step / 2.0).cos()).atan();
        for i in 0..k {
            let dir = rotate_about(start, center, step / 2.0 + step * i as f64);
            // same azimuth about the center, angular distance `lift`
            let w = unit(sub(dir, scale(center, dot(dir, center))));
            points.push(add(scale(center, lift.cos()), scale(w, lift.sin())));
        }
    } else {
        for i in 1..k {
            points.push(rotate_about(start, center, step * i as f64));
        }
    }
    points.push(end);
    points
}

// ------------------------------------------------------------------ the pair

/// One gear's tooth geometry (unit sphere; see BevelPair).
#[derive(Debug, Clone)]
pub struct BevelProfile {
    pub teeth: usize,
    /// Pitch, base, tip and root cone angles (polar angles).
    pub delta: f64,
    pub base: f64,
    pub tip: f64,
    pub root: f64,
    pub flank: SphFlank,
    pub s_tip: f64,
    pub s_p: f64,
    /// Top of the fillet on the upper flank, its center and its foot on the
    /// root circle (all three the same point for a sharp corner).
    pub p: V,
    pub c: Option<V>,
    pub q: V,
    /// Fillet radius as an angle.
    pub rho: f64,
    pub full_round: bool,
    /// Lowest polar angle the mate's tips touch on the flank.
    pub contact: f64,
}

/// A meshing pair of bevel gears (see the module documentation).
#[derive(Debug, Clone)]
pub struct BevelPair {
    pub shaft_angle: f64,
    /// Cone distance at the heel, and the face width.
    pub r_cone: f64,
    pub face: f64,
    /// Backlash as an arc on the pitch circle at the heel.
    pub backlash: f64,
    pub pressure_angle: f64,
    pub helix_angle: f64,
    /// Accuracy, fillet accuracy, point spacing and shortest land, as angles
    /// on the sphere (accuracy at the heel; spacing and land at the toe).
    pub dev: f64,
    pub fillet_dev: f64,
    pub ds: f64,
    pub min_land: f64,
    pub gears: [BevelProfile; 2],
}

impl BevelPair {
    /// The pair from a gear set. backlash_at_toe: the gear set's backlash is
    /// the play at the toe (the smallest) instead of at the heel.
    pub fn new(s: &GearSystem, teeth: [usize; 2], shaft_angle: f64, face: f64, backlash_at_toe: bool) -> Result<Self> {
        let mut pair = Self::checked(s, teeth, shaft_angle, face, backlash_at_toe)?;
        for i in 0..2 {
            pair.choose_fillet(i, s);
        }
        Ok(pair)
    }

    /// Everything new() checks, without choosing the fillets (fast; for
    /// searching parameters). The fillets are left sharp.
    pub fn checked(s: &GearSystem, teeth: [usize; 2], shaft_angle: f64, face: f64, backlash_at_toe: bool) -> Result<Self> {
        if teeth[0] < 3 || teeth[1] < 3 {
            return err("Each bevel gear needs at least 3 teeth.");
        }
        if !(0.0 < shaft_angle && shaft_angle < PI) {
            return err("The shaft angle must be between 0 and 180 degrees.");
        }
        let (z1, z2) = (teeth[0] as f64, teeth[1] as f64);
        let d1 = shaft_angle.sin().atan2(z2 / z1 + shaft_angle.cos());
        let d2 = shaft_angle - d1;
        let m = s.module();
        let r_cone = m * z1 / (2.0 * d1.sin());
        if !(face > 0.0) {
            return err("The face width must be positive.");
        }
        if face >= r_cone {
            return err(format!(
                "The face width must be less than the distance from the outer edge of the teeth to the point \
                 where the two shafts meet ({:.3} mm, measured along the teeth); a third of that is typical.",
                r_cone
            ));
        }
        let toe = (r_cone - face) / r_cone;
        let backlash = if backlash_at_toe { s.backlash / toe } else { s.backlash };
        if backlash >= s.pitch / 2.0 {
            return err(format!(
                "The backlash at the outer end ({:.3} mm) must be less than half the tooth spacing there \
                 (π × module / 2 = {:.3} mm).",
                backlash,
                s.pitch / 2.0
            ));
        }
        let alpha = s.pressure_angle;
        let make = |z: usize, delta: f64| -> Result<BevelProfile> {
            let base = (delta.sin() * alpha.cos()).asin();
            let tip = delta + s.height / 2.0 / r_cone;
            let root = delta - (s.height / 2.0 + s.clearance) / r_cone;
            // Tooth thickness at the heel's pitch circle, (pitch - backlash) / 2,
            // as an azimuth; half of it on each side of the center line.
            let half = (s.pitch - backlash) / 4.0 / (r_cone * delta.sin());
            let flank = SphFlank { base, constant: 0.0 };
            let flank = SphFlank { base, constant: half + flank.ev(delta) };
            Ok(BevelProfile {
                teeth: z,
                delta,
                base,
                tip,
                root,
                flank,
                s_tip: flank.s_at(tip),
                s_p: 0.0,
                p: [0.0; 3],
                c: None,
                q: [0.0; 3],
                rho: 0.0,
                full_round: false,
                contact: 0.0,
            })
        };
        let mut pair = BevelPair {
            shaft_angle,
            r_cone,
            face,
            backlash,
            pressure_angle: alpha,
            helix_angle: s.helix_angle,
            dev: s.max_deviation / r_cone,
            fillet_dev: s.fillet_deviation / r_cone,
            ds: s.ds / (r_cone - face),
            min_land: MIN_LAND.max(2.0 * s.ds) / (r_cone - face),
            gears: [make(teeth[0], d1)?, make(teeth[1], d2)?],
        };
        pair.check()?;
        for i in 0..2 {
            pair.gears[i].contact = pair.contact(i)?;
        }
        Ok(pair)
    }

    fn name(i: usize) -> &'static str {
        ["Gear 1", "Gear 2"][i]
    }

    /// Everything that doesn't depend on the mate: cones, lands.
    fn check(&self) -> Result<()> {
        for (i, g) in self.gears.iter().enumerate() {
            let name = Self::name(i);
            if g.tip >= PI / 2.0 - 1e-9 {
                return err(format!(
                    "{} would be flatter than a flat disc: its tooth tips would point {:.1}° away from its axis \
                     (at most 90°). Use a smaller shaft angle, or fewer teeth on {} compared to {}.",
                    name,
                    g.tip.to_degrees(),
                    ["gear 1", "gear 2"][i],
                    ["gear 2", "gear 1"][i]
                ));
            }
            if g.root <= 0.0 {
                return err(format!("{}'s tooth gaps would reach past its axis: use more teeth on it.", name));
            }
            let at_toe = (self.r_cone - self.face) / self.r_cone;
            if 2.0 * g.flank.half_angle(g.tip) * g.tip.sin() < self.min_land {
                return err(format!(
                    "{}'s teeth would come to a point before reaching their tips at the inner end ({:.3} mm \
                     wide there). Use more teeth, a smaller tooth height or less backlash.",
                    name,
                    2.0 * g.flank.half_angle(g.tip) * g.tip.sin() * self.r_cone * at_toe
                ));
            }
            if (2.0 * PI / g.teeth as f64 - 2.0 * g.flank.half_angle(g.root)) * g.root.sin() < self.min_land {
                return err(format!(
                    "{}'s neighbouring teeth would meet before reaching the bottom of the gaps. Use more teeth \
                     or a smaller root gap.",
                    name
                ));
            }
        }
        Ok(())
    }

    // --------------------------------------------------------------- frames

    /// A point of gear `which`'s frame, turned by `rotation` about its axis,
    /// in the world (gear 1's) frame. Gear 2's axis is gear 1's turned by the
    /// shaft angle toward +x: (sin sigma, 0, -cos sigma).
    pub fn to_world(&self, which: usize, rotation: f64, v: V) -> V {
        let v = rot_z(v, rotation);
        if which == 0 {
            v
        } else {
            rot_y(v, -self.shaft_angle)
        }
    }

    pub fn from_world(&self, which: usize, rotation: f64, w: V) -> V {
        let w = if which == 0 { w } else { rot_y(w, self.shaft_angle) };
        rot_z(w, -rotation)
    }

    /// Gear 2's rotation for gear 1's rotation, mesh centered: gear 1's tooth
    /// 0 points at the pitch line (azimuth 0) when unturned, and gear 2's
    /// pitch line is at its azimuth pi, where a gap must then be centered.
    /// Gear 1 turning by +d moves its pitch point toward +y; gear 2 turning by
    /// +d moves its pitch point toward -y, so they roll together when gear 2
    /// turns by -d z_1 / z_2.
    pub fn mesh_rotation(&self, rotation_1: f64) -> f64 {
        let (z1, z2) = (self.gears[0].teeth as f64, self.gears[1].teeth as f64);
        PI + PI / z2 - rotation_1 * z1 / z2
    }

    pub fn mesh_rotation_inverse(&self, rotation_2: f64) -> f64 {
        let (z1, z2) = (self.gears[0].teeth as f64, self.gears[1].teeth as f64);
        (PI + PI / z2 - rotation_2) * z2 / z1
    }

    /// The pitch point (unit, world frame) and a line of action through it:
    /// L(t) = cos(t) pitch + sin(t) direction, at alpha to the pitch circles'
    /// common tangent (+y). sign picks one of the two lines. Going toward
    /// negative t, the line approaches gear 1's base cone (touching it where
    /// gear 1's polar angle is smallest); toward positive t, gear 2's.
    pub fn line_of_action(&self, sign: f64) -> (V, V) {
        let d1 = self.gears[0].delta;
        let pitch = sph(d1, 0.0);
        let e = [d1.cos(), 0.0, d1.sin()]; // toward gear 2's axis
        let a = self.pressure_angle;
        (pitch, add(scale([0.0, 1.0, 0.0], a.cos()), scale(e, sign * a.sin())))
    }

    pub fn axis(&self, which: usize) -> V {
        self.to_world(which, 0.0, [0.0, 0.0, -1.0])
    }

    /// Polar angle about gear `which`'s axis at L(t).
    fn polar_on_line(&self, which: usize, t: f64) -> f64 {
        let (p, d) = self.line_of_action(1.0);
        let l = add(scale(p, t.cos()), scale(d, t.sin()));
        dot(l, self.axis(which)).clamp(-1.0, 1.0).acos()
    }

    /// Lowest polar angle where the mate's tips touch gear `which`'s flank,
    /// checking that it is on the involute and that the teeth stay in
    /// contact from one tooth to the next.
    fn contact(&self, which: usize) -> Result<f64> {
        let mate = 1 - which;
        let (g, gm) = (&self.gears[which], &self.gears[mate]);
        // Where the line touches this gear's base cone: L(t) . axis largest.
        let (p, d) = self.line_of_action(1.0);
        let ax = self.axis(which);
        let mut t_base = dot(d, ax).atan2(dot(p, ax));
        if which == 0 {
            t_base = t_base.min(0.0);
        } else {
            t_base = t_base.max(0.0);
        }
        // The mate's tip circle crosses the line between the pitch point and there?
        let f = |t: f64| self.polar_on_line(mate, t) - gm.tip;
        if f(t_base) < 0.0 {
            return err(format!(
                "{}'s tooth tips would reach too deep into {}'s gaps, below where its tooth sides can be shaped \
                 to mesh. Use more teeth, a smaller tooth height or a larger pressure angle.",
                Self::name(mate),
                Self::name(which).to_lowercase()
            ));
        }
        let (mut lo, mut hi) = (t_base, 0.0);
        for _ in 0..80 {
            let mid = (lo + hi) / 2.0;
            if f(mid) > 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let t_end = (lo + hi) / 2.0;
        if which == 0 {
            // contact ratio, once: the line between both tip circles over the base pitch
            let other = {
                let ax2 = self.axis(1);
                let t_b2 = dot(d, ax2).atan2(dot(p, ax2)).max(0.0);
                let f2 = |t: f64| self.polar_on_line(0, t) - g.tip;
                if f2(t_b2) < 0.0 {
                    return err(
                        "Gear 1's tooth tips would reach too deep into gear 2's gaps, below where its tooth sides \
                         can be shaped to mesh. Use more teeth, a smaller tooth height or a larger pressure angle.",
                    );
                }
                let (mut lo, mut hi) = (0.0, t_b2);
                for _ in 0..80 {
                    let mid = (lo + hi) / 2.0;
                    if f2(mid) < 0.0 {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                (lo + hi) / 2.0
            };
            let base_pitch = 2.0 * PI * g.base.sin() / g.teeth as f64;
            if other - t_end < base_pitch {
                return err(
                    "The teeth are too short for this pair: they would lose contact between one tooth and the \
                     next. Use more teeth, a taller tooth height or a larger pressure angle.",
                );
            }
        }
        Ok(self.polar_on_line(which, t_end))
    }

    // ----------------------------------------------------------- fillets

    /// Points swept by the mate's tooth-tip corners in gear `which`'s frame
    /// while the mesh turns through three of its teeth around tooth 0, with
    /// the mate moved by `shift` (an arc on its pitch circle at the heel)
    /// away from this gear's upper flank (see gears.rs, corner_paths).
    fn corner_paths(&self, which: usize, shift: f64) -> Vec<V> {
        let mate = 1 - which;
        let (g, gm) = (&self.gears[which], &self.gears[mate]);
        let half = gm.flank.half_angle(gm.tip);
        let n = 400;
        let span = 1.5 * 2.0 * PI / g.teeth as f64;
        let start = if which == 0 { 0.0 } else { PI };
        let pitch_world = sph(self.gears[0].delta, 0.0);
        let mut out = Vec::new();
        for i in 0..=n {
            let own = start + span * (2.0 * i as f64 / n as f64 - 1.0);
            // Moving gear 2 away from gear 1's upper flank turns it by -; moving
            // gear 1 away from gear 2's upper flank turns it by + (see mesh_rotation).
            let mate_rot = if which == 0 {
                self.mesh_rotation(own) - shift / (self.r_cone * gm.delta.sin())
            } else {
                self.mesh_rotation_inverse(own) + shift / (self.r_cone * gm.delta.sin())
            };
            for k in 0..gm.teeth {
                for sgn in [1.0, -1.0] {
                    let corner = sph(gm.tip, sgn * half + 2.0 * PI * k as f64 / gm.teeth as f64);
                    let w = self.to_world(mate, mate_rot, corner);
                    if angle(w, pitch_world) > 6.0 * 2.0 * PI * gm.delta.sin() / gm.teeth as f64 {
                        continue;
                    }
                    out.push(self.from_world(which, own, w));
                }
            }
        }
        out
    }

    /// Like GearSystem::fillet, on the sphere (angles throughout).
    fn choose_fillet(&mut self, which: usize, s: &GearSystem) {
        let g = self.gears[which].clone();
        let flank = g.flank;
        let (s_corner, s_far) = (flank.s_at(g.root), g.s_tip);
        let corner = flank.at(s_corner);
        let rho_max = s.fillet_radius / self.r_cone;
        let j = self.backlash;
        let obstacles = [
            (self.corner_paths(which, j / 2.0), j * self.pressure_angle.cos() / self.r_cone),
            (self.corner_paths(which, -j / 2.0), 0.0),
        ];
        let reach = 3.0 * rho_max + 1.0 / self.r_cone;
        let gap = 2.0 * PI / g.teeth as f64;
        let nearby: Vec<(Vec<V>, f64)> = obstacles
            .iter()
            .map(|(pts, req)| {
                let kept = pts
                    .iter()
                    .copied()
                    .filter(|&o| {
                        let a = azimuth(o);
                        angle(o, corner) < reach && 0.0 < a && a < gap
                    })
                    .collect();
                (kept, *req)
            })
            .collect();
        let land_center = PI / g.teeth as f64;

        let solve_fillet = |rho: f64| -> Option<(f64, V, V, V)> {
            let center = |s: f64| -> (V, V) {
                let p = flank.at(s);
                let n = flank.normal(s);
                (p, add(scale(p, rho.cos()), scale(n, rho.sin())))
            };
            let target = g.root + rho;
            let f = |s: f64| polar(center(s).1) - target;
            let f_corner = f(s_corner);
            if f_corner * f(s_far) > 0.0 {
                return None;
            }
            let (mut lo, mut hi) = (s_corner, s_far);
            for _ in 0..80 {
                let mid = (lo + hi) / 2.0;
                if f(mid) * f_corner > 0.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let sv = (lo + hi) / 2.0;
            let (p, c) = center(sv);
            Some((sv, p, c, sph(g.root, azimuth(c))))
        };

        let clear = |p: V, c: V, q: V, rho: f64| -> bool {
            let tangent = |x: V| sub(x, scale(c, dot(x, c)));
            let (u, v) = (tangent(p), tangent(q));
            let sweep = dot(cross(u, v), c).atan2(dot(u, v));
            for (pts, req) in &nearby {
                for &o in pts {
                    let w = tangent(o);
                    let t = dot(cross(u, w), c).atan2(dot(u, w));
                    let d = if 0.0 <= t * sweep && t.abs() <= sweep.abs() {
                        rho - angle(c, o)
                    } else {
                        angle(o, p).min(angle(o, q))
                    };
                    if d < *req {
                        return false;
                    }
                }
            }
            true
        };

        let solve = |rho: f64| -> Option<(f64, (f64, V, V, V))> {
            let sol = solve_fillet(rho)?;
            let remaining = land_center - azimuth(sol.3);
            if polar(sol.1) > g.contact {
                return None;
            }
            if !clear(sol.1, sol.2, sol.3, rho) {
                return None;
            }
            Some((remaining, sol))
        };
        let sharp = (0.0, s_corner, corner, None, corner, false);
        let ds = self.ds;
        let keep = |choice: (f64, f64, V, Option<V>, V, bool)| {
            let (rho, _, p, c, q, _) = choice;
            let c = c.unwrap();
            if rho * angle(sub(p, scale(c, dot(p, c))), sub(q, scale(c, dot(q, c)))) >= 2.0 * ds {
                choice
            } else {
                sharp
            }
        };
        let root_sin = g.root.sin();
        let min_land = self.min_land;
        let choice = (|| {
            if rho_max == 0.0 {
                return sharp;
            }
            if let Some((rem, sol)) = solve(rho_max) {
                if 2.0 * rem * root_sin >= min_land {
                    return keep((rho_max, sol.0, sol.1, Some(sol.2), sol.3, false));
                }
            }
            let (mut lo, mut hi) = (0.0, rho_max);
            let mut best = None;
            for _ in 0..60 {
                let mid = (lo + hi) / 2.0;
                match solve(mid) {
                    Some((rem, sol)) if rem >= 0.0 => {
                        lo = mid;
                        best = Some((rem, sol));
                    }
                    _ => hi = mid,
                }
            }
            let Some((rem, sol)) = best else { return sharp };
            if 2.0 * rem * root_sin >= min_land {
                return keep((lo, sol.0, sol.1, Some(sol.2), sol.3, false));
            }
            keep((lo, sol.0, sol.1, Some(sol.2), sph(g.root, land_center), true))
        })();
        let gm = &mut self.gears[which];
        (gm.rho, gm.s_p, gm.p, gm.c, gm.q, gm.full_round) = choice;
    }

    // ------------------------------------------------------------- outline

    /// Closed outline of gear `which` on the unit sphere in its own frame, in
    /// order of increasing azimuth, tooth 0 centered at azimuth 0, with no
    /// repeated end point.
    pub fn outline(&self, which: usize) -> Vec<V> {
        let g = &self.gears[which];
        let (dev, fdev, ds) = (self.dev, self.fillet_dev, self.ds);
        let step = 2.0 * PI / g.teeth as f64;
        let pole = [0.0, 0.0, -1.0];
        let tip = g.flank.at(g.s_tip);
        let (p_lo, q_lo) = (mirror(g.p), mirror(g.q));
        let mut period = Vec::new();
        if g.rho > 0.0 {
            period.extend(sph_arc(mirror(g.c.unwrap()), q_lo, p_lo, fdev, ds, true));
        }
        period.extend(g.flank.points(g.s_p, g.s_tip, dev, ds).into_iter().map(mirror));
        period.extend(sph_arc(pole, mirror(tip), tip, dev, ds, false));
        period.extend(g.flank.points(g.s_tip, g.s_p, dev, ds));
        if g.rho > 0.0 {
            period.extend(sph_arc(g.c.unwrap(), g.p, g.q, fdev, ds, true));
        }
        if !g.full_round {
            period.extend(sph_arc(pole, g.q, rot_z(q_lo, step), dev, ds, false));
        }
        (0..g.teeth).flat_map(|i| period.iter().map(move |&p| rot_z(p, i as f64 * step))).collect()
    }

    /// Turn of the section at cone distance rho, for gear `which` with the
    /// given hand (gear 1's hand; gear 2 gets the opposite in its own frame,
    /// which meshes: see mesh_rotation).
    pub fn twist(&self, which: usize, hand: i32, rho: f64) -> f64 {
        let (r, f) = (self.r_cone, self.face);
        let rho = rho.clamp(r - f, r);
        let h = if which == 0 { hand } else { -hand } as f64;
        h * self.helix_angle.tan() / self.gears[which].delta.sin() * (rho / (r - f)).ln().min((r / rho).ln())
    }

    /// Cone distances of the layers a twisted wall needs, from the toe to the
    /// heel. In each half of the face the layers are evenly spaced in ln rho,
    /// so each step turns the section by the same angle k dl (k =
    /// tan(beta) / sin(delta)). Between layers the wall is straight:
    /// - each edge between layers is a chord of a point's path, which is
    ///   checked directly at the heel's tip, where it strays most;
    /// - each quad is twisted by k dl, so splitting it strays by about
    ///   (segment length) k dl / 4.
    ///
    /// Both stay within the accuracy; layers are at least ds apart at the toe.
    pub fn layers(&self, which: usize, longest_segment: f64) -> Vec<f64> {
        let (r, f) = (self.r_cone, self.face);
        let k = self.helix_angle.tan() / self.gears[which].delta.sin();
        if k == 0.0 {
            return vec![r - f, r];
        }
        let dev = self.dev * r;
        let tip = self.gears[which].tip;
        let mid = (r * (r - f)).sqrt();
        let half = (r / mid).ln();
        let path = |t: f64| scale(sph(tip, k * t), r * t.exp());
        let strays = |dl: f64| -> f64 {
            let (a, b) = (path(-dl), path(0.0));
            let m = path(-dl / 2.0);
            let ab = sub(b, a);
            let t = (dot(sub(m, a), ab) / dot(ab, ab)).clamp(0.0, 1.0);
            norm(sub(m, add(a, scale(ab, t))))
        };
        let mut n = 1usize;
        loop {
            let dl = half / n as f64;
            if strays(dl) <= dev && longest_segment * k * dl / 4.0 <= dev {
                break;
            }
            n += 1;
        }
        while n > 1 && (r - f) * ((half / n as f64).exp() - 1.0) < self.ds * (r - f) {
            n -= 1;
        }
        let dl = half / n as f64;
        let mut out: Vec<f64> = (0..=n).map(|i| (r - f) * (dl * i as f64).exp()).collect();
        out.extend((1..=n).map(|i| mid * (dl * i as f64).exp()));
        *out.last_mut().unwrap() = r;
        out
    }
}

// --------------------------------------------------------------------- meshes

/// Closed mesh of one gear of the pair in its own frame (apex at the origin,
/// axis -z), herringbone with gear 1's `hand` (gear 2 the opposite). bore: a
/// diameter, 0 for none.
///
/// Ends: the teeth end on the spheres at the heel (cone distance R) and toe
/// (R - face width). Each end is one triangulation, made in a plane and bent
/// onto the end. The fold circle, at module / 4 inside the root circle on the
/// sphere, splits it: inside, the end is the flat plane through the fold
/// circle (the gear's body face); outside, the sphere (the tooth ends). The
/// fold circle is on both, and it is a fixed edge of the triangulation, so
/// the flat face and the curved tooth ends meet exactly along it.
///
/// The plane: the flat part is itself (radius r from the axis); a point of
/// the sphere beyond the fold circle, at polar angle theta, is at radius
/// r_fold + R (theta - theta_fold), its distance along the sphere from the
/// fold circle (the same azimuth). This stays even all the way to a flat
/// disc (theta = 90 degrees), and lengths in the plane are never shorter than
/// on the sphere, so triangles kept small in the plane (circumradius at most
/// sqrt(2 R accuracy)) stay within the accuracy of the sphere. Points the
/// triangulation adds on an outline edge are put on the great circle arc
/// between the edge's ends (the wall there is a plane through the apex), at
/// the same fraction along it. The toe's end is the same triangulation on the
/// toe's sphere and flat face (the flat part shrunk toward the bore, which
/// keeps its size).
///
/// Walls: a column per boundary point through every layer, turned by the
/// twist (0 at both ends); neighbouring columns stitched together. The bore
/// is a straight prism.
pub fn bevel_gear_mesh(pair: &BevelPair, which: usize, bore: f64, hand: i32) -> Result<IndexedMesh> {
    if hand != 1 && hand != -1 {
        return err("The twist direction must be 1 or -1.");
    }
    let g = &pair.gears[which];
    let (r, f) = (pair.r_cone, pair.face);
    let toe = r - f;
    // Fold circle: module / 4 inside the root circle at the heel (an angle).
    let module = 2.0 * r * g.delta.sin() / g.teeth as f64;
    let fold = g.root - 0.25 * module / r;
    let depth = r * fold.cos(); // the heel's flat face, below the apex
    let r_fold = r * fold.sin(); // fold circle radius in the projection plane (= on the heel)
    let r_fold_toe = toe * fold.sin();
    let rb = bore / 2.0;
    let ds = pair.ds * r; // point spacing, scaled to the heel
    if bore < 0.0 {
        return err("The bore can't be negative.");
    }
    if rb > 0.0 && rb > r_fold_toe - 2.0 * pair.ds * toe {
        return err(format!(
            "{}'s bore ({} mm) must be smaller than its flat face at the inner end ({:.3} mm across).",
            BevelPair::name(which),
            bore,
            2.0 * r_fold_toe
        ));
    }
    let to_plane = |v: V| -> (f64, f64) {
        let (rr, a) = (r_fold + r * (polar(v) - fold), azimuth(v));
        (rr * a.cos(), rr * a.sin())
    };
    let outline_dirs = pair.outline(which);
    let outline: Vec<(f64, f64)> = outline_dirs.iter().map(|&v| to_plane(v)).collect();
    let dev_mm = pair.dev * r;
    let circle = |radius: f64, hole: bool| -> Vec<(f64, f64)> {
        use crate::gears::{arc_steps, tangent_arc_steps};
        if hole {
            let k = tangent_arc_steps(radius, 2.0 * PI, dev_mm, ds / 2.0).max(3);
            let rr = radius / (PI / k as f64).cos();
            (0..k).rev().map(|i| (rr * (2.0 * PI * (i as f64 + 0.5) / k as f64).cos(), rr * (2.0 * PI * (i as f64 + 0.5) / k as f64).sin())).collect()
        } else {
            let k = arc_steps(radius, 2.0 * PI, dev_mm, ds).max(3);
            (0..k).map(|i| (radius * (2.0 * PI * i as f64 / k as f64).cos(), radius * (2.0 * PI * i as f64 / k as f64).sin())).collect()
        }
    };
    let mut loops = vec![outline.clone()];
    if rb > 0.0 {
        loops.push(circle(rb, true));
    }
    let creases = vec![circle(r_fold, false)];
    let max_radius = (2.0 * toe.min(r) * dev_mm).sqrt() * 0.9;
    let limit = move |p: (f64, f64)| if p.0.hypot(p.1) > r_fold { max_radius } else { f64::INFINITY };
    let tri = triangulate_sized(&loops, DEFAULT_MIN_ANGLE, ds, &creases, 400_000, &limit)?;
    let pts = &tri.points;

    // Directions (unit) of the outline loop's points: the outline's own, and
    // added points on the great circle arc of their edge.
    let mut dirs: std::collections::HashMap<usize, V> = std::collections::HashMap::new();
    {
        let lp = &tri.boundary[0];
        let original: std::collections::HashMap<(u64, u64), V> =
            outline.iter().zip(&outline_dirs).map(|(p, &v)| ((p.0.to_bits(), p.1.to_bits()), v)).collect();
        let starts: Vec<usize> = (0..lp.len()).filter(|&k| original.contains_key(&(pts[lp[k]].0.to_bits(), pts[lp[k]].1.to_bits()))).collect();
        for (n, &k) in starts.iter().enumerate() {
            let k_end = if n + 1 < starts.len() { starts[n + 1] } else { starts[0] + lp.len() };
            let (pa, pb) = (pts[lp[k]], pts[lp[k_end % lp.len()]]);
            let (va, vb) = (original[&(pa.0.to_bits(), pa.1.to_bits())], original[&(pb.0.to_bits(), pb.1.to_bits())]);
            let (len, theta) = ((pb.0 - pa.0).hypot(pb.1 - pa.1), angle(va, vb));
            let axis = unit(cross(va, vb));
            dirs.insert(lp[k], va);
            for m in k + 1..k_end {
                let p = pts[lp[m % lp.len()]];
                let t = (p.0 - pa.0).hypot(p.1 - pa.1) / len;
                dirs.insert(lp[m % lp.len()], rotate_about(va, axis, t * theta));
            }
        }
    }

    // A face point on the end at cone distance rho (heel: r, toe: toe).
    let lift = |i: usize, rho: f64| -> V {
        let (x, y) = pts[i];
        let rr = x.hypot(y);
        if rr <= r_fold * (1.0 + 1e-12) {
            let k = rho / r;
            if rb > 0.0 && rr > 0.0 {
                let rf = r_fold * k;
                let new = rb + (rr - rb) * (rf - rb) / (r_fold - rb);
                [x / rr * new, y / rr * new, -depth * k]
            } else {
                [x * k, y * k, -depth * k]
            }
        } else if let Some(&v) = dirs.get(&i) {
            scale(v, rho)
        } else {
            scale(sph(fold + (rr - r_fold) / r, y.atan2(x)), rho)
        }
    };
    let longest = {
        let lp = &tri.boundary[0];
        (0..lp.len())
            .map(|n| {
                let (a, b) = (lift(lp[n], r), lift(lp[(n + 1) % lp.len()], r));
                norm(sub(a, b))
            })
            .fold(0.0, f64::max)
    };
    let layers = pair.layers(which, longest);
    let mut mesh = IndexedMesh::new();
    let mut columns: std::collections::HashMap<usize, Vec<(f64, usize)>> = std::collections::HashMap::new();
    for &i in &tri.boundary[0] {
        let dir = dirs[&i];
        let col = layers
            .iter()
            .rev()
            .map(|&rho| (r - rho, mesh.add_vertex(scale(rot_z(dir, pair.twist(which, hand, rho)), rho))))
            .collect();
        columns.insert(i, col);
    }
    if rb > 0.0 {
        for &i in &tri.boundary[1] {
            let col = vec![(0.0, mesh.add_vertex(lift(i, r))), (f, mesh.add_vertex(lift(i, toe)))];
            columns.insert(i, col);
        }
    }
    // Ends: the heel faces away from the apex (like a spur gear's bottom).
    for heel in [true, false] {
        let rho = if heel { r } else { toe };
        let mut ids: std::collections::HashMap<usize, usize> =
            columns.iter().map(|(&i, col)| (i, if heel { col[0].1 } else { col[col.len() - 1].1 })).collect();
        for &[a, b, c] in &tri.triangles {
            let mut vid = |i: usize| -> usize {
                *ids.entry(i).or_insert_with(|| mesh.add_vertex(lift(i, rho)))
            };
            let (va, vb, vc) = (vid(a), vid(b), vid(c));
            if heel {
                mesh.add_triangle(va, vc, vb);
            } else {
                mesh.add_triangle(va, vb, vc);
            }
        }
    }
    for lp in tri.boundary.iter().take(if rb > 0.0 { 2 } else { 1 }) {
        for n in 0..lp.len() {
            stitch(&mut mesh, &columns[&lp[n]], &columns[&lp[(n + 1) % lp.len()]]);
        }
    }
    Ok(mesh)
}

/// Both gears' meshes. assembled: in mesh, gear 2 turned into place about
/// gear 1, gear 1 on the z axis with the center of its flat back face (the
/// heel) at the origin, so the pair doesn't move when the shaft angle changes. Otherwise laid out for
/// printing: each standing on its flat back face (the heel) at z = 0, side
/// by side along x.
#[allow(clippy::too_many_arguments)]
pub fn bevel_pair_mesh(
    s: &GearSystem,
    teeth: [usize; 2],
    shaft_angle: f64,
    face: f64,
    bores: [f64; 2],
    hand: i32,
    backlash_at_toe: bool,
    assembled: bool,
) -> Result<Vec<IndexedMesh>> {
    Ok(bevel_pair_layout(s, teeth, shaft_angle, face, bores, hand, backlash_at_toe, assembled)?.0)
}

/// Where a gear of a laid-out pair is: its apex, its axis (a unit vector
/// from the gear toward the apex) and its pitch cone angle. For the page's
/// section view.
#[derive(Clone, Copy, Debug)]
pub struct BevelFrame {
    pub apex: V,
    pub axis: V,
    pub pitch_cone: f64,
}

/// bevel_pair_mesh(), and each gear's frame in the layout.
#[allow(clippy::too_many_arguments)]
pub fn bevel_pair_layout(
    s: &GearSystem,
    teeth: [usize; 2],
    shaft_angle: f64,
    face: f64,
    bores: [f64; 2],
    hand: i32,
    backlash_at_toe: bool,
    assembled: bool,
) -> Result<(Vec<IndexedMesh>, [BevelFrame; 2])> {
    let pair = BevelPair::new(s, teeth, shaft_angle, face, backlash_at_toe)?;
    // In each gear's own frame the apex is at the origin and the axis is +z.
    let mut frames = [0, 1].map(|i| BevelFrame { apex: [0.0; 3], axis: [0.0, 0.0, 1.0], pitch_cone: pair.gears[i].delta });
    let mut parts = Vec::new();
    for (which, &bore) in bores.iter().enumerate() {
        parts.push(bevel_gear_mesh(&pair, which, bore, hand)?);
    }
    if assembled {
        let rot = pair.mesh_rotation(0.0);
        parts[1].vertices = parts[1].vertices.iter().map(|&v| pair.to_world(1, rot, v)).collect();
        frames[1].axis = pair.to_world(1, rot, [0.0, 0.0, 1.0]);
        // Put the center of gear 1's flat back face (its lowest point) at the origin.
        let z0 = parts[0].vertices.iter().map(|v| v[2]).fold(f64::INFINITY, f64::min);
        for part in parts.iter_mut() {
            for v in part.vertices.iter_mut() {
                v[2] -= z0;
            }
        }
        for f in frames.iter_mut() {
            f.apex = [0.0, 0.0, -z0];
        }
    } else {
        let mut x = 0.0;
        for (which, part) in parts.iter_mut().enumerate() {
            let z0 = part.vertices.iter().map(|v| v[2]).fold(f64::INFINITY, f64::min);
            let reach = part.vertices.iter().map(|v| v[0].hypot(v[1])).fold(0.0, f64::max);
            if which == 1 {
                x += reach;
            }
            for v in part.vertices.iter_mut() {
                *v = [v[0] + x, v[1], v[2] - z0];
            }
            frames[which].apex = [x, 0.0, -z0];
            x += reach + 5.0;
        }
    }
    Ok((parts, frames))
}
