//! Gear geometry: 2D outlines of spur and ring gears.
//!
//! All lengths are in mm and all angles in radians.
//!
//! # Usage
//!
//! ```ignore
//! let system = GearSystem::new(&Params::new(pitch, pressure_angle, height, backlash,
//!                                           max_deviation, ds))?;
//! let sun = system.spur_gear(24)?;                   // Vec<(x, y)>
//! let ring = system.ring_gear(72)?;                  // the toothed edge
//! let outlines = system.planetary_gear(24, 24, 4, 0.0)?; // [sun, planets..., ring]
//! ```
//!
//! GearSystem checks everything it can when it is created, so a mistake shows
//! up right away with a readable message (GearError). After that, the only
//! thing a gear can be rejected for is too few teeth.
//!
//! # The guarantee
//!
//! Any two spur gears with at least `min_spur_teeth` teeth mesh. A spur gear
//! also meshes inside any ring gear that has at least 2 * min_spur_teeth more
//! teeth than it (true in every planetary set). "Mesh" means: the involutes
//! are in contact at all times, exactly backlash * cos(alpha) / 2 apart on
//! each side when the mesh is centered (see spur_mesh_rotation), and nothing
//! else comes closer.
//!
//! # Outline construction, for one tooth period
//!
//! Each outline is a chain of pieces whose end points are computed exactly and
//! shared, so no points are ever duplicated or nearly coincident:
//!
//! ```text
//! spur:  fillet -> flank -> tip land -> flank -> fillet -> root land
//! ring:  flank -> fillet -> root land -> fillet -> flank -> tip land
//! ```
//!
//! - Flank: the involute, continued below the base circle by a straight
//!   radial line. The two meet smoothly at the base circle, so the flank is a
//!   single curve, sampled evenly by arc length.
//! - Fillet: a circle of radius rho tangent to both the flank and the root
//!   circle (see `solve_fillet`).
//! - Tip and root lands: arcs about the gear center.
//!
//! Each piece is sampled so that the straight segments between its points
//! stray from the true curve by at most max_deviation, with no two points
//! closer than ds. Each piece leaves out its first point, which is the last
//! point of the piece before it.
//!
//! The outline never extends past the true shape: where the material is on
//! the inner side of a curve (spur flanks, tip and root lands), the points are
//! on the curve and the chords cut into the material; where it is on the
//! outer side (spur fillets; a ring's flanks, fillets and lands; a bore), the
//! outline is a tangent polygon whose segments touch the curve from the
//! material side. (Fillets use their own, looser tolerance, fillet_deviation.)
//! So the outline only ever loses material, by at most max_deviation: gaps
//! between meshing gears can only grow, by at most 2 * max_deviation.

use std::f64::consts::PI;
use std::fmt;

/// A planetary set's layout: ring teeth, [(planet center, planet
/// rotation)], ring rotation (see GearSystem::planetary_layout).
pub type PlanetaryLayout = (usize, Vec<(P, f64)>, f64);

pub type P = (f64, f64);

/// Shortest tip or root land kept, in mm (or 2 * ds, if that is longer: see
/// GearSystem::min_land). A root land that would be shorter becomes a fully
/// rounded root; a tooth count that would give a shorter tip land (or root
/// gap) is below the system's minimum. It only exists to keep the geometry
/// from degenerating (near-coincident points, zero-width features); it is far
/// below anything a printer can make.
pub const MIN_LAND: f64 = 0.05;

/// Largest tooth count searched for the minimums.
const MAX_TEETH: usize = 10000;

/// A parameter combination that can't make a valid gear. The message says why.
#[derive(Debug, Clone, PartialEq)]
pub struct GearError(pub String);

impl fmt::Display for GearError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for GearError {}

pub type Result<T> = std::result::Result<T, GearError>;

fn err<T>(message: impl Into<String>) -> Result<T> {
    Err(GearError(message.into()))
}

/// inv(a) = tan(a) - a: how far the involute has turned at pressure angle a.
pub fn involute(a: f64) -> f64 {
    a.tan() - a
}

/// Number of equal steps for an arc of this radius and sweep angle.
///
/// A chord spanning the angle t on a circle of radius R strays from the arc
/// by its sagitta, R (1 - cos(t / 2)). That is at most max_deviation for
/// t <= 2 acos(1 - max_deviation / R), which sets the fewest steps. The chord
/// is 2 R sin(t / 2) long, which must stay at least ds; that caps the number
/// of steps. (The small allowances keep exact multiples, up to rounding
/// error, from getting an extra step or losing one.)
pub fn arc_steps(radius: f64, sweep: f64, max_deviation: f64, ds: f64) -> usize {
    let sweep = sweep.abs();
    if radius <= 0.0 || sweep == 0.0 {
        return 1;
    }
    let t_dev = 2.0 * (1.0 - max_deviation / radius).max(-1.0).acos();
    let k = (sweep / t_dev - 1e-9).ceil();
    let t_min = 2.0 * (ds / (2.0 * radius)).min(1.0).asin();
    let k_max = (sweep / t_min + 1e-9).floor();
    (k.min(k_max) as i64).max(1) as usize
}

/// Number of steps for an arc drawn as a tangent polygon (see arc_points).
///
/// The vertices sit at radius R / cos(t / 2) for a step angle t, which is at
/// most max_deviation beyond the arc for t <= 2 acos(R / (R + max_deviation)).
/// The shortest segments, at the ends, are R tan(t / 2) long and must stay at
/// least ds. At least one step, so that the polygon never cuts inside.
pub fn tangent_arc_steps(radius: f64, sweep: f64, max_deviation: f64, ds: f64) -> usize {
    let sweep = sweep.abs();
    if radius <= 0.0 || sweep == 0.0 {
        return 1;
    }
    let k = (sweep / (2.0 * (radius / (radius + max_deviation)).acos()) - 1e-9).ceil();
    let k_max = (sweep / (2.0 * (ds / radius).atan()) + 1e-9).floor();
    (k.min(k_max) as i64).max(1) as usize
}

pub fn polar(r: f64, theta: f64) -> P {
    (r * theta.cos(), r * theta.sin())
}

pub fn rotate(p: P, angle: f64) -> P {
    let (c, s) = (angle.cos(), angle.sin());
    (p.0 * c - p.1 * s, p.0 * s + p.1 * c)
}

/// Reflect across the x-axis (the center line of the tooth or gap).
fn mirror(p: P) -> P {
    (p.0, -p.1)
}

pub fn place(points: &[P], rotation: f64, center: P) -> Vec<P> {
    points
        .iter()
        .map(|&p| {
            let q = rotate(p, rotation);
            (q.0 + center.0, q.1 + center.1)
        })
        .collect()
}

fn hypot(x: f64, y: f64) -> f64 {
    x.hypot(y)
}

/// Points along the circular arc about `center` from `start` to `end`, going
/// the short way, staying within max_deviation of the arc, at least ds apart.
/// Leaves out `start`; the last point is exactly `end`.
///
/// inside (outside = false): points on the arc, joined by chords, which lie
/// inside the circle (see arc_steps). Use it where the material is inside the
/// circle (convex), so the chords cut into the material, never past it.
///
/// outside: a tangent polygon, for where the material is outside the circle
/// (concave): every segment lies on a tangent of the arc, so the polygon stays
/// in the material. With k steps of angle t, the vertices are at angles t/2,
/// 3t/2, ... from start, at radius R / cos(t / 2): the segment from start to
/// the first vertex lies on the tangent at start, each middle segment on the
/// tangent at a step's midpoint, the last on the tangent at end (see
/// tangent_arc_steps).
///
/// u = start - center and v = end - center. The signed angle from u to v is
/// atan2(u x v, u . v), which is in (-pi, pi], so the short way.
pub fn arc_points(center: P, start: P, end: P, max_deviation: f64, ds: f64, outside: bool) -> Vec<P> {
    let (ux, uy) = (start.0 - center.0, start.1 - center.1);
    let (vx, vy) = (end.0 - center.0, end.1 - center.1);
    let sweep = (ux * vy - uy * vx).atan2(ux * vx + uy * vy);
    let mut points = Vec::new();
    if outside {
        let k = tangent_arc_steps(hypot(ux, uy), sweep, max_deviation, ds);
        let step = sweep / k as f64;
        let stretch = 1.0 / (step / 2.0).cos();
        for i in 0..k {
            let (x, y) = rotate((ux, uy), step / 2.0 + step * i as f64);
            points.push((center.0 + x * stretch, center.1 + y * stretch));
        }
    } else {
        let k = arc_steps(hypot(ux, uy), sweep, max_deviation, ds);
        for i in 1..k {
            let (x, y) = rotate((ux, uy), sweep * i as f64 / k as f64);
            points.push((center.0 + x, center.1 + y));
        }
    }
    points.push(end);
    points
}

/// The upper flank (theta > 0) of a shape that is symmetric about angle 0 and
/// whose thickness at the pitch circle is `thickness` (an arc length). For a
/// spur gear the shape is a tooth; for a ring gear it is a gap.
///
/// Half-angle of the shape at radius R (for R at or above the base circle):
///
/// ```text
/// half_angle(R) = thickness / (2 r_pitch) + inv(alpha) - inv(a_R),
/// a_R = acos(r_base / R)
/// ```
///
/// It shrinks as R grows, so the shape tapers outward. Below the base circle
/// there is no involute; the flank is the radial line at half_angle(r_base).
///
/// The flank is parametrized by arc length s from the base circle: s >= 0 on
/// the involute (s = r_base tan^2(a) / 2), s < 0 on the radial line
/// (s = R - r_base).
#[derive(Debug, Clone, Copy)]
pub struct Flank {
    pub r_base: f64,
    pub constant: f64,
}

impl Flank {
    pub fn new(r_base: f64, r_pitch: f64, alpha: f64, thickness: f64) -> Self {
        Flank { r_base, constant: thickness / (2.0 * r_pitch) + involute(alpha) }
    }

    pub fn pressure_angle_at(&self, r: f64) -> f64 {
        if r > self.r_base {
            (self.r_base / r).acos()
        } else {
            0.0
        }
    }

    pub fn half_angle(&self, r: f64) -> f64 {
        self.constant - involute(self.pressure_angle_at(r))
    }

    pub fn s_at(&self, r: f64) -> f64 {
        if r <= self.r_base {
            return r - self.r_base;
        }
        let t = (self.r_base / r).acos().tan();
        self.r_base * t * t / 2.0
    }

    /// Point at arc length s, the pressure angle a there (0 on the radial
    /// line), and its polar angle theta.
    pub fn at(&self, s: f64) -> (P, f64, f64) {
        let (r, a) = if s <= 0.0 {
            (self.r_base + s, 0.0)
        } else {
            let a = (2.0 * s / self.r_base).sqrt().atan();
            (self.r_base / a.cos(), a)
        };
        let theta = self.constant - involute(a);
        (polar(r, theta), a, theta)
    }

    /// Unit tangent at arc length s, pointing outward (growing R). The normal
    /// is sin(a) e_r + cos(a) e_theta (see solve_fillet), so the tangent is
    /// cos(a) e_r - sin(a) e_theta.
    pub fn tangent(&self, s: f64) -> P {
        let (_, a, theta) = self.at(s);
        (
            a.cos() * theta.cos() + a.sin() * theta.sin(),
            a.cos() * theta.sin() - a.sin() * theta.cos(),
        )
    }

    fn polyline(&self, values: &[f64], outside: bool) -> Vec<P> {
        let pts: Vec<P> = values.iter().map(|&s| self.at(s).0).collect();
        if !outside {
            return pts;
        }
        let mut out = vec![pts[0]];
        for i in 0..values.len() - 1 {
            let (d, e) = (self.tangent(values[i]), self.tangent(values[i + 1]));
            let (p, q) = (pts[i], pts[i + 1]);
            let cross = d.0 * e.1 - d.1 * e.0;
            if cross.abs() < 1e-12 {
                out.push(((p.0 + q.0) / 2.0, (p.1 + q.1) / 2.0));
            } else {
                let lam = ((q.0 - p.0) * e.1 - (q.1 - p.1) * e.0) / cross;
                out.push((p.0 + lam * d.0, p.1 + lam * d.1));
            }
        }
        out.push(*pts.last().unwrap());
        out
    }

    /// Points from s_start to s_end that stay within max_deviation of the
    /// flank, at least ds apart. Leaves out the point at s_start.
    ///
    /// inside (outside = false): points on the flank joined by chords, which
    /// lie on the inner side of the curve: inside a spur tooth (the material).
    /// outside: a tangent polygon, for a ring's flank, whose material is on the
    /// outer side of the curve: the vertices are where the tangents at
    /// neighbouring sample points meet, so every segment lies on a tangent and
    /// the polygon stays in the material. The ends stay on the flank.
    ///
    /// The radial line below the base circle is straight: no points inside it.
    /// On the involute, the curvature radius at arc length s is
    /// sqrt(R^2 - r_base^2) = r_base tan(a) = sqrt(2 r_base s) (the length of
    /// unwound string). A chord of length L on a curve of curvature radius c
    /// strays by about L^2 / (8 c), so the local step can be
    /// L(s) = sqrt(8 max_deviation c(s)). Spacing the points evenly in
    /// u(s) = integral of ds / L(s) = (4/3) s^(3/4) / (sqrt(8 max_deviation) (2 r_base)^(1/4))
    /// gives every chord that step; the number of steps is the length of the
    /// range in u, rounded up.
    pub fn points(&self, s_start: f64, s_end: f64, max_deviation: f64, ds: f64, outside: bool) -> Vec<P> {
        let (lo, hi) = (s_start.min(s_end), s_start.max(s_end));
        assert!(!(outside && lo < 0.0), "tangent polygons are only for the involute part");
        let mut values = vec![lo];
        if lo < 0.0 && 0.0 < hi && -lo >= ds {
            values.push(0.0); // the base circle, where the involute begins
        }
        if hi > 0.0 {
            let a = lo.max(0.0);
            let scale = (4.0 / 3.0) / ((8.0 * max_deviation).sqrt() * (2.0 * self.r_base).powf(0.25));
            let (u0, u1) = (scale * a.powf(0.75), scale * hi.powf(0.75));
            let mut n = ((u1 - u0 - 1e-9).ceil() as i64).max(1) as usize;
            loop {
                let mut candidate = values.clone();
                for i in 1..=n {
                    let u = (u0 + (u1 - u0) * i as f64 / n as f64) / scale;
                    candidate.push(u.powf(4.0 / 3.0));
                }
                *candidate.last_mut().unwrap() = hi;
                let pts = self.polyline(&candidate, outside);
                let shortest = pts
                    .windows(2)
                    .map(|w| hypot(w[1].0 - w[0].0, w[1].1 - w[0].1))
                    .fold(f64::INFINITY, f64::min);
                if shortest >= ds || n == 1 {
                    values = candidate;
                    break;
                }
                n -= 1;
            }
        } else if *values.last().unwrap() != hi {
            values.push(hi);
        }
        let mut pts = self.polyline(&values, outside);
        if s_start > s_end {
            pts.reverse();
        }
        pts.remove(0);
        pts
    }
}

/// Fillet circle of radius rho tangent to the flank and the root circle.
///
/// `side` is +1 when air is on the side of larger theta (spur gear: air is
/// outside the tooth) and -1 when it is on the side of smaller theta (ring
/// gear: air is inside the gap).
///
/// The normal of an involute at a point is tangent to the base circle. In
/// polar terms, with pressure angle a at the point, the unit normal is
/// sin(a) e_r + cos(a) e_theta (just e_theta on the radial line). So for a
/// point P on the flank, the center of the circle tangent there is
///
/// ```text
/// C = P + rho * side * (sin(a) e_r + cos(a) e_theta),
/// ```
///
/// and the circle also touches the root circle when |C| = r_root + side * rho
/// (from outside the root circle for a spur gear, from inside for a ring).
/// |C| changes monotonically along the flank, so bisection finds P. Q, where
/// the fillet meets the root circle, is C scaled to radius r_root.
///
/// Returns (s_P, P, C, Q), or None if no point on the flank works (rho is too
/// big to fit before the far end of the flank).
fn solve_fillet(flank: &Flank, side: f64, r_root: f64, rho: f64, s_corner: f64, s_far: f64) -> Option<(f64, P, P, P)> {
    let target = r_root + side * rho;
    let center = |s: f64| -> (P, P) {
        let ((x, y), a, theta) = flank.at(s);
        let (nr, nt) = (a.sin(), a.cos());
        let nx = side * (nr * theta.cos() - nt * theta.sin());
        let ny = side * (nr * theta.sin() + nt * theta.cos());
        ((x, y), (x + rho * nx, y + rho * ny))
    };
    let g = |s: f64| {
        let c = center(s).1;
        hypot(c.0, c.1) - target
    };
    let g_corner = g(s_corner);
    if g_corner * g(s_far) > 0.0 {
        return None;
    }
    let (mut lo, mut hi) = (s_corner, s_far);
    for _ in 0..80 {
        let mid = (lo + hi) / 2.0;
        if g(mid) * g_corner > 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let s = (lo + hi) / 2.0;
    let (p, c) = center(s);
    let d = hypot(c.0, c.1);
    Some((s, p, c, (c.0 * r_root / d, c.1 * r_root / d)))
}

/// Computed geometry of one tooth period.
#[derive(Debug, Clone)]
pub struct Profile {
    pub teeth: usize,
    pub r_pitch: f64,
    pub r_base: f64,
    pub r_tip: f64,
    pub r_root: f64,
    pub flank: Flank,
    pub s_tip: f64,
    pub s_p: f64,
    pub p: P,
    /// The fillet's center (None for a sharp corner).
    pub c: Option<P>,
    pub q: P,
    pub rho: f64,
    pub full_round: bool,
    pub r_contact: f64,
}

/// What a GearSystem is made from (see GearSystem for the meaning of each).
#[derive(Debug, Clone, Copy)]
pub struct Params {
    pub pitch: f64,
    pub pressure_angle: f64,
    pub height: f64,
    pub backlash: f64,
    pub max_deviation: f64,
    pub ds: f64,
    pub helix_angle: f64,
    pub clearance: Option<f64>,
    pub fillet_radius: Option<f64>,
    pub fillet_deviation: Option<f64>,
}

impl Params {
    /// The required values; helix angle 0 and every option at its default.
    pub fn new(pitch: f64, pressure_angle: f64, height: f64, backlash: f64, max_deviation: f64, ds: f64) -> Self {
        Params {
            pitch,
            pressure_angle,
            height,
            backlash,
            max_deviation,
            ds,
            helix_angle: 0.0,
            clearance: None,
            fillet_radius: None,
            fillet_deviation: None,
        }
    }
}

/// Which partner a gear's fillet is checked against (see corner_paths).
#[derive(Debug, Clone, Copy)]
enum Partner {
    Rack,
    Ring(usize),
    Spur(usize),
}

/// Everything that is set once. Gears made from one system mesh with each
/// other (see the module documentation for the exact guarantee).
///
/// - pitch: arc length from one tooth to the next along the pitch circle. A
///   gear with z teeth has pitch radius z * pitch / (2 pi). (module = pitch / pi)
/// - pressure_angle: angle between the line of action and the pitch circle's
///   tangent, in the plane across the gear.
/// - height: working tooth height. Teeth reach height/2 beyond the pitch
///   circle; gaps go height/2 + clearance below it.
/// - backlash: free play between two meshing gears, as an arc length on the
///   pitch circle. Each gear takes half: spur teeth are backlash/2 thinner,
///   ring gaps backlash/2 wider.
/// - max_deviation: largest distance the straight segments between outline
///   points may stray from the true curves.
/// - ds: smallest distance between two neighbouring points.
/// - helix_angle: tooth slant at the pitch circle (used by the 3D code).
/// - clearance: gap between a tip and the mating root. Default: the standard
///   0.25 * module.
/// - fillet_radius: radius of the root fillets. Default:
///   clearance / (1 - sin(alpha)), the largest rounding that fits the
///   clearance of a standard rack tooth. Each gear uses at most this, and less
///   where its gap is too narrow (down to a fully rounded root) or where the
///   fillet would reach a mating tooth's contact zone. 0 means sharp root
///   corners.
/// - fillet_deviation: tolerance for drawing the fillets, which can be much
///   looser than max_deviation: a fillet's exact shape only matters for
///   strength, not for meshing. Fillets are drawn as tangent polygons, so a
///   coarser one only takes material out of the corner (toward a sharp
///   corner); it never gets closer to a mating tooth. Default 0.05 mm, and
///   never tighter than max_deviation.
///
/// Computed when the system is created:
/// - min_spur_teeth: fewest teeth a spur gear can have.
/// - min_ring_teeth: fewest teeth a ring gear can have (3 * min_spur_teeth or
///   more, the smallest planetary set).
#[derive(Debug, Clone)]
pub struct GearSystem {
    pub pitch: f64,
    pub pressure_angle: f64,
    pub height: f64,
    pub backlash: f64,
    pub max_deviation: f64,
    pub ds: f64,
    /// Lands shorter than this count as none (see MIN_LAND). At least 2 * ds,
    /// so a land drawn as a tangent polygon keeps its points ds apart.
    pub min_land: f64,
    pub helix_angle: f64,
    pub clearance: f64,
    pub fillet_deviation: f64,
    pub fillet_radius: f64,
    pub min_spur_teeth: usize,
    pub min_ring_teeth: usize,
    /// Largest tooth count searched for the minimums (MAX_TEETH, or less for
    /// new_quick).
    teeth_limit: usize,
}

impl GearSystem {
    pub fn new(params: &Params) -> Result<Self> {
        Self::create(params, MAX_TEETH, true)
    }

    /// new(), but searching the minimum tooth counts only up to teeth_limit
    /// and without working out which value to blame when none is found: fast
    /// even for hopeless parameters (for searching parameters).
    pub fn new_quick(params: &Params, teeth_limit: usize) -> Result<Self> {
        Self::create(params, teeth_limit, false)
    }

    /// Whether new_quick(params, teeth_limit) makes a gear set in which these
    /// tooth counts work: a spur gear with `spur` teeth and, if given, a ring
    /// with `ring` teeth. Much quicker than finding the minimums: a tooth
    /// count works exactly when it's at least the minimum (see the guarantee
    /// above), so each count is checked on its own.
    pub fn teeth_fit(params: &Params, spur: usize, ring: Option<usize>, teeth_limit: usize) -> bool {
        let Ok(s) = Self::checked(params, teeth_limit) else { return false };
        let spur_ok = |z: usize| (3..=teeth_limit).contains(&z) && s.spur_problem(z).is_none();
        // The gear set needs some ring to work (within the limit), and the
        // ring minimum is also at least 3 * the spur minimum.
        let ring_ok = |r: usize| r <= teeth_limit && spur_ok(r / 3) && s.ring_problem(r).is_none();
        spur_ok(spur) && ring_ok(ring.unwrap_or(teeth_limit))
    }

    fn create(params: &Params, teeth_limit: usize, blame: bool) -> Result<Self> {
        let mut system = Self::checked(params, teeth_limit)?;
        match system.minimums() {
            Ok((spur, ring)) => {
                system.min_spur_teeth = spur;
                system.min_ring_teeth = ring;
                Ok(system)
            }
            Err(e) => Err(if blame { system.blame_spacing().unwrap_or(e) } else { e }),
        }
    }

    /// The gear set with every check but the tooth minimums (left 0).
    fn checked(params: &Params, teeth_limit: usize) -> Result<Self> {
        let Params { pitch, pressure_angle, height, backlash, max_deviation, ds, helix_angle, .. } = *params;
        if !(pitch > 0.0) {
            return err("The module must be positive.");
        }
        if !(0.0 < pressure_angle && pressure_angle <= PI / 4.0) {
            return err("The pressure angle must be between 0 and 45 degrees.");
        }
        if !(height > 0.0) {
            return err("The tooth height must be positive.");
        }
        if !(0.0 <= backlash && backlash < pitch / 2.0) {
            return err(format!(
                "The backlash must be at least 0 and less than half the tooth spacing \
                 (π × module / 2 = {:.3} mm).",
                pitch / 2.0
            ));
        }
        if !(max_deviation > 0.0) {
            return err("The surface accuracy must be positive.");
        }
        if !(ds > 0.0) {
            return err("The smallest point spacing must be positive.");
        }
        if !(0.0..PI / 2.0).contains(&helix_angle) {
            return err("The helix angle must be between 0 and 90 degrees (to slant the other way, change the twist direction).");
        }
        let module = pitch / PI;
        let clearance = params.clearance.unwrap_or(0.25 * module);
        if clearance < 0.0 {
            return err("The root gap can't be negative.");
        }
        let fillet_deviation = params.fillet_deviation.unwrap_or(0.05f64.max(max_deviation));
        if !(fillet_deviation > 0.0) {
            return err("The root rounding accuracy must be positive.");
        }
        let fillet_radius = params.fillet_radius.unwrap_or(clearance / (1.0 - pressure_angle.sin()));
        if fillet_radius < 0.0 {
            return err("The root rounding radius can't be negative.");
        }
        if 0.0 < fillet_radius && fillet_radius < MIN_LAND {
            return err(format!(
                "A root rounding radius of {:.3} mm is too small to model (minimum {} mm). Use 0 for \
                 sharp corners at the bottom of the gaps.",
                fillet_radius, MIN_LAND
            ));
        }
        // Teeth stay in contact (contact ratio >= 1) only if they are long
        // enough along the line of action. Even for two racks, the most
        // contact there can be is height / sin(alpha), against a spacing of
        // pitch * cos(alpha) between teeth.
        if height / pressure_angle.sin() < pitch * pressure_angle.cos() {
            return err(
                "The teeth are too short for this pressure angle: meshing gears would lose contact \
                 between one tooth and the next. Use a taller tooth height or a larger pressure angle.",
            );
        }
        Ok(GearSystem {
            pitch,
            pressure_angle,
            height,
            backlash,
            max_deviation,
            ds,
            min_land: MIN_LAND.max(2.0 * ds),
            helix_angle,
            clearance,
            fillet_deviation,
            fillet_radius,
            min_spur_teeth: 0,
            min_ring_teeth: 0,
            teeth_limit,
        })
    }

    /// (min_spur_teeth, min_ring_teeth), or why no tooth count works.
    fn minimums(&self) -> Result<(usize, usize)> {
        let spur = self.find_minimum("spur", |s, z| s.spur_problem(z))?;
        let ring = self.find_minimum("ring", |s, z| s.ring_problem(z))?;
        Ok((spur, (3 * spur).max(ring)))
    }

    /// When no tooth count works only because the point spacing is large
    /// (every tooth tip and gap bottom needs room for two points, ds apart:
    /// min_land = 2 ds), a message saying so, with the largest spacing that
    /// would work (found by bisection).
    fn blame_spacing(&self) -> Option<GearError> {
        if self.min_land <= MIN_LAND {
            return None;
        }
        let with_spacing = |ds: f64| {
            let mut s = self.clone();
            s.ds = ds;
            s.min_land = MIN_LAND.max(2.0 * ds);
            s.minimums().is_ok()
        };
        if !with_spacing(MIN_LAND / 2.0) {
            return None; // something else is wrong too
        }
        let (mut lo, mut hi) = (MIN_LAND / 2.0, self.ds);
        for _ in 0..40 {
            let mid = (lo + hi) / 2.0;
            if with_spacing(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        Some(GearError(format!(
            "The smallest point spacing ({} mm) is too large for these teeth: every tooth tip and \
             every gap bottom needs room for two points that far apart. Use at most {:.3} mm.",
            self.ds,
            (lo * 1000.0).floor() / 1000.0
        )))
    }

    pub fn module(&self) -> f64 {
        self.pitch / PI
    }

    pub fn pitch_radius(&self, teeth: usize) -> f64 {
        teeth as f64 * self.pitch / (2.0 * PI)
    }

    pub fn base_radius(&self, teeth: usize) -> f64 {
        self.pitch_radius(teeth) * self.pressure_angle.cos()
    }

    /// A full circle about the origin as a counterclockwise list of (x, y),
    /// within max_deviation of the circle, sides at least ds long.
    ///
    /// For a ring gear's rim (material inside), the points are on the circle
    /// and the sides cut slightly inside it. For a hole such as a bore
    /// (material outside), hole = true gives a circumscribed polygon: its sides
    /// touch the circle and its corners are outside it, at radius
    /// R / cos(pi / k), so the hole is never smaller than asked for.
    pub fn circle(&self, radius: f64, hole: bool) -> Result<Vec<P>> {
        if !(radius > 0.0) {
            return err("A circle's radius must be positive.");
        }
        let (dev, ds) = (self.max_deviation, self.ds);
        Ok(if hole {
            let k = tangent_arc_steps(radius, 2.0 * PI, dev, ds / 2.0).max(3);
            let r = radius / (PI / k as f64).cos();
            (0..k).map(|i| polar(r, 2.0 * PI * (i as f64 + 0.5) / k as f64)).collect()
        } else {
            let k = arc_steps(radius, 2.0 * PI, dev, ds).max(3);
            (0..k).map(|i| polar(radius, 2.0 * PI * i as f64 / k as f64)).collect()
        })
    }

    // ----------------------------------------------------- minimum tooth counts

    /// Fewest teeth for which a rack's tooth tips stay on the involute:
    /// height / 2 <= r_pitch * sin^2(alpha).
    pub fn z_rack(&self) -> f64 {
        PI * self.height / (self.pitch * self.pressure_angle.sin().powi(2))
    }

    /// Average number of tooth pairs in contact for two spur gears: the length
    /// of the line of action inside both tip circles, divided by the spacing
    /// of teeth along it (pitch * cos(alpha)).
    fn external_contact_ratio(&self, za: usize, zb: usize) -> f64 {
        let (alpha, h) = (self.pressure_angle, self.height);
        let mut length = -(self.pitch_radius(za) + self.pitch_radius(zb)) * alpha.sin();
        for z in [za, zb] {
            let r = self.pitch_radius(z);
            length += ((r + h / 2.0).powi(2) - (r * alpha.cos()).powi(2)).sqrt();
        }
        length / (self.pitch * alpha.cos())
    }

    fn internal_contact_ratio(&self, z_planet: usize, z_ring: usize) -> f64 {
        let (alpha, h) = (self.pressure_angle, self.height);
        let (r_p, r_r) = (self.pitch_radius(z_planet), self.pitch_radius(z_ring));
        let length = ((r_p + h / 2.0).powi(2) - (r_p * alpha.cos()).powi(2)).sqrt()
            - ((r_r - h / 2.0).powi(2) - (r_r * alpha.cos()).powi(2)).sqrt()
            + (r_r - r_p) * alpha.sin();
        length / (self.pitch * alpha.cos())
    }

    /// Why `teeth` can't be the system's spur minimum, or None. Every
    /// condition gets easier with more teeth, so the first count that passes
    /// works for all counts above it too.
    fn spur_problem(&self, teeth: usize) -> Option<&'static str> {
        let r = self.pitch_radius(teeth);
        let r_tip = r + self.height / 2.0;
        let r_root = r - self.height / 2.0 - self.clearance;
        if r_root <= 0.0 {
            return Some("the tooth gaps would reach past the center");
        }
        let flank = Flank::new(r * self.pressure_angle.cos(), r, self.pressure_angle, (self.pitch - self.backlash) / 2.0);
        if 2.0 * flank.half_angle(r_tip) * r_tip < self.min_land {
            return Some("the teeth would come to a point before reaching their full height");
        }
        if (2.0 * PI / teeth as f64 - 2.0 * flank.half_angle(r_root)) * r_root < self.min_land {
            return Some("neighbouring teeth would meet before the gaps between them reach their full depth");
        }
        // Mating tips must stay on the involute part of this gear's flank. For
        // an external mate the worst case is a rack: teeth >= z_rack. A ring is
        // stricter by about 2 z_ring / (2 z_ring - teeth); the smallest ring
        // around a planet with this many teeth has 3 * teeth (sun = planet),
        // which gives 6/5. (In the rotating tests this limit is accurate to
        // within a tooth, on the safe side.)
        if (teeth as f64) < 1.2 * self.z_rack() {
            return Some(
                "the tooth tips of a mating gear (a ring gear, in the worst case) would reach too deep \
                 into the gaps, below where the tooth sides can be shaped to mesh",
            );
        }
        if self.external_contact_ratio(teeth, teeth) < 1.0 {
            return Some("two such gears would lose contact between one tooth and the next");
        }
        if self.internal_contact_ratio(teeth, 3 * teeth) < 1.0 {
            return Some("such a gear would lose contact with its ring between one tooth and the next");
        }
        None
    }

    /// Why `teeth` can't be the system's ring minimum, or None.
    fn ring_problem(&self, teeth: usize) -> Option<&'static str> {
        let r = self.pitch_radius(teeth);
        let rb = r * self.pressure_angle.cos();
        let r_tip = r - self.height / 2.0;
        let r_root = r + self.height / 2.0 + self.clearance;
        if r_tip < rb {
            return Some("the tooth tips would reach too far toward the center, where the tooth sides can't be shaped to mesh");
        }
        let flank = Flank::new(rb, r, self.pressure_angle, (self.pitch + self.backlash) / 2.0);
        if (2.0 * PI / teeth as f64 - 2.0 * flank.half_angle(r_tip)) * r_tip < self.min_land {
            return Some("the teeth would come to a point before reaching their full height");
        }
        if 2.0 * flank.half_angle(r_root) * r_root < self.min_land {
            return Some("the gaps between the teeth would close before reaching their full depth");
        }
        None
    }

    /// The fewest teeth that work, by halving: a tooth count works exactly
    /// when it's at least the minimum (see the guarantee above).
    fn find_minimum(&self, kind: &str, problem: impl Fn(&Self, usize) -> Option<&'static str>) -> Result<usize> {
        if self.teeth_limit >= 3 && problem(self, self.teeth_limit).is_none() {
            let (mut bad, mut good) = (2, self.teeth_limit); // bad fails (or is below 3), good works
            while good - bad > 1 {
                let mid = (bad + good) / 2;
                if problem(self, mid).is_none() {
                    good = mid;
                } else {
                    bad = mid;
                }
            }
            return Ok(good);
        }
        err(format!(
            "This gear set can't make {} gears with any number of teeth: {}. Use a smaller tooth height \
             or root gap, less backlash, or a larger module.",
            kind,
            problem(self, self.teeth_limit).unwrap_or("")
        ))
    }

    fn check_teeth(&self, kind: &str, teeth: usize) -> Result<()> {
        let minimum = if kind == "spur" { self.min_spur_teeth } else { self.min_ring_teeth };
        if teeth < minimum {
            let mut name = kind.to_string();
            name[..1].make_ascii_uppercase();
            return err(format!(
                "{} gears in this gear set need at least {} teeth (asked for {}).",
                name, minimum, teeth
            ));
        }
        Ok(())
    }

    // ----------------------------------------------------------- fillet choice

    /// Points swept by the tooth-tip corners of mating gears, in this gear's
    /// own frame (unrotated outline), while the mesh turns through about three
    /// teeth around tooth / gap 0.
    ///
    /// The partner starts from the centered mesh (spur_mesh_rotation /
    /// ring_mesh_rotation) and is moved by `shift`, an arc length on its pitch
    /// circle: positive moves it away from this gear's upper flank (the flank
    /// at theta > 0 of tooth / gap 0), so shift = +backlash/2 puts that flank
    /// on the coast side with all the play, and -backlash/2 puts it on the
    /// drive side, touching.
    fn corner_paths(&self, own_teeth: usize, partners: &[Partner], shift: f64) -> Vec<P> {
        let (alpha, h, p, j) = (self.pressure_angle, self.height, self.pitch, self.backlash);
        let r = self.pitch_radius(own_teeth);
        let n = 400;
        let span = 1.5 * 2.0 * PI / own_teeth as f64;
        let angles: Vec<f64> = (0..=n).map(|i| span * (2.0 * i as f64 / n as f64 - 1.0)).collect();
        let mut points = Vec::new();

        let corners = |radius: f64, half: f64, teeth: usize| -> Vec<f64> {
            let _ = radius;
            let mut out = Vec::new();
            for k in -3i32..4 {
                for sgn in [1.0, -1.0] {
                    out.push(sgn * half + 2.0 * PI * k as f64 / teeth as f64);
                }
            }
            out
        };
        let spur_corners = |teeth: usize| -> (f64, Vec<f64>) {
            let rp = self.pitch_radius(teeth);
            let f = Flank::new(rp * alpha.cos(), rp, alpha, (p - j) / 2.0);
            let half = f.half_angle(rp + h / 2.0);
            (rp + h / 2.0, corners(rp + h / 2.0, half, teeth))
        };
        let ring_corners = |teeth: usize| -> (f64, Vec<f64>) {
            let rr = self.pitch_radius(teeth);
            let f = Flank::new(rr * alpha.cos(), rr, alpha, (p + j) / 2.0);
            let half = f.half_angle(rr - h / 2.0); // gap half-angle at the ring's tip
            (rr - h / 2.0, corners(rr - h / 2.0, half, teeth))
        };

        for &partner in partners {
            match partner {
                Partner::Rack => {
                    // Own spur gear at the origin, pitch point (r, 0). The rack's
                    // pitch line is x = r and its teeth point toward -x; a rack
                    // gap is centered on the pitch point when own tooth 0 is.
                    // Turning the gear by phi moves the rack by r * phi along +y.
                    let half_tip = (p - j) / 4.0 - h / 2.0 * alpha.tan();
                    let x = r - h / 2.0;
                    for k in -3i32..4 {
                        for sgn in [1.0, -1.0] {
                            // The rack tooth at +y faces the upper flank.
                            let y0 = p / 2.0 + k as f64 * p + sgn * half_tip + shift;
                            for &phi in &angles {
                                points.push(rotate((x, y0 + r * phi), -phi));
                            }
                        }
                    }
                }
                Partner::Ring(teeth) => {
                    // Own spur gear is the planet. In the ring's frame the planet
                    // is at (a, 0); the ring turns by phi * z_planet / z_ring.
                    let rr = self.pitch_radius(teeth);
                    let a = rr - r;
                    let (radius, corner_angles) = ring_corners(teeth);
                    for &phi in &angles {
                        // Turning the ring counterclockwise moves its teeth at the
                        // pitch point toward +y, away from the upper flank.
                        let phi_r = phi * own_teeth as f64 / teeth as f64 + shift / rr;
                        for &t in &corner_angles {
                            let (cx, cy) = polar(radius, t + phi_r);
                            points.push(rotate((cx - a, cy), -phi));
                        }
                    }
                }
                Partner::Spur(teeth) => {
                    // Own gear is the ring; the partner is a planet at (a, 0)
                    // turning by phi * z_ring / z_planet.
                    let rp = self.pitch_radius(teeth);
                    let a = r - rp;
                    let (radius, corner_angles) = spur_corners(teeth);
                    for &phi in &angles {
                        // Turning the planet counterclockwise moves its teeth at
                        // the pitch point toward +y, into the ring gap's upper flank.
                        let phi_p = phi * own_teeth as f64 / teeth as f64 - shift / rp;
                        for &t in &corner_angles {
                            let (cx, cy) = polar(radius, t + phi_p);
                            points.push(rotate((cx + a, cy), -phi));
                        }
                    }
                }
            }
        }
        points
    }

    /// Corner paths with the upper flank on the coast side (needs the whole
    /// backlash along the line of action) and on the drive side (needs no
    /// cutting in). The outline is symmetric, so the lower flank's fillet sees
    /// the mirror image.
    fn driven_obstacles(&self, teeth: usize, partners: &[Partner]) -> Vec<(Vec<P>, f64)> {
        let j = self.backlash;
        vec![
            (self.corner_paths(teeth, partners, j / 2.0), j * self.pressure_angle.cos()),
            (self.corner_paths(teeth, partners, -j / 2.0), 0.0),
        ]
    }

    /// Pick the fillet for one gear. Returns (rho, s_P, P, C, Q, full_round).
    ///
    /// land_center is the polar angle of the middle of the root land next to
    /// the upper flank. `remaining` is the angle left between Q and that
    /// middle; the root land is 2 * remaining * r_root long. (The tooth-count
    /// minimum guarantees a root land of at least min_land without a fillet.)
    ///
    /// A fillet whose arc would be shorter than 2 * ds is left out (a sharp
    /// corner): drawn as a tangent polygon, it would need points closer than
    /// ds. Leaving it out only removes material from the corner.
    ///
    /// The fillet must also stay clear of mating teeth:
    /// - its top, P, stays on the root side of r_contact, the nearest radius to
    ///   the root where a mating tooth touches this flank, so the fillet never
    ///   replaces flank that is used;
    /// - it stays clear of `obstacles`, a list of (points, required gap): the
    ///   paths swept by mating tooth-tip corners when one gear drives the
    ///   other. Just past the end of contact a corner keeps moving and dips
    ///   slightly below r_contact. When this flank is on the coast side the
    ///   required gap is the whole backlash along the line of action,
    ///   backlash * cos(alpha); on the drive side, 0 (no cutting in). Only
    ///   points in the gap the fillet faces count (polar angle within
    ///   gap_angles): seen from C, the fillet's sector also reaches through the
    ///   tooth into the next gap.
    #[allow(clippy::too_many_arguments)]
    fn fillet(
        &self,
        flank: &Flank,
        side: f64,
        r_root: f64,
        s_corner: f64,
        s_far: f64,
        land_center: f64,
        r_contact: f64,
        obstacles: &[(Vec<P>, f64)],
        gap_angles: (f64, f64),
    ) -> (f64, f64, P, Option<P>, P, bool) {
        let corner = flank.at(s_corner).0;
        let reach = 3.0 * self.fillet_radius + 1.0;
        let (lo_angle, hi_angle) = gap_angles;
        let nearby: Vec<(Vec<P>, f64)> = obstacles
            .iter()
            .map(|(points, gap)| {
                let kept = points
                    .iter()
                    .copied()
                    .filter(|o| {
                        let a = o.1.atan2(o.0);
                        hypot(o.0 - corner.0, o.1 - corner.1) < reach && lo_angle < a && a < hi_angle
                    })
                    .collect();
                (kept, *gap)
            })
            .collect();

        // Signed clearance from a point to the fillet arc from P to Q about C
        // (the short way). C is in the air, so within the arc's sector a point
        // inside the circle is in the air, rho - |w| from the arc, and a point
        // outside it is in the material (negative clearance).
        let clear_of_obstacles = |p: P, c: P, q: P, rho: f64| -> bool {
            let (ux, uy) = (p.0 - c.0, p.1 - c.1);
            let (vx, vy) = (q.0 - c.0, q.1 - c.1);
            let sweep = (ux * vy - uy * vx).atan2(ux * vx + uy * vy);
            for (points, min_gap) in &nearby {
                for &(ox, oy) in points {
                    let (wx, wy) = (ox - c.0, oy - c.1);
                    let t = (ux * wy - uy * wx).atan2(ux * wx + uy * wy);
                    let d = if 0.0 <= t * sweep && t.abs() <= sweep.abs() {
                        rho - hypot(wx, wy)
                    } else {
                        hypot(ox - p.0, oy - p.1).min(hypot(ox - q.0, oy - q.1))
                    };
                    if d < *min_gap {
                        return false;
                    }
                }
            }
            true
        };

        let solve = |rho: f64| -> Option<(f64, (f64, P, P, P))> {
            let sol = solve_fillet(flank, side, r_root, rho, s_corner, s_far)?;
            let remaining = side * (land_center - sol.3 .1.atan2(sol.3 .0));
            if side * (hypot(sol.1 .0, sol.1 .1) - r_contact) > 0.0 {
                return None; // P reaches the contact zone
            }
            if !clear_of_obstacles(sol.1, sol.2, sol.3, rho) {
                return None;
            }
            Some((remaining, sol))
        };

        let sharp = (0.0, s_corner, corner, None, corner, false);
        let keep_if_long_enough = |choice: (f64, f64, P, Option<P>, P, bool)| {
            let (rho, _, p, c, q, _) = choice;
            let c = c.unwrap();
            let (ux, uy, vx, vy) = (p.0 - c.0, p.1 - c.1, q.0 - c.0, q.1 - c.1);
            let sweep = (ux * vy - uy * vx).atan2(ux * vx + uy * vy).abs();
            if rho * sweep >= 2.0 * self.ds {
                choice
            } else {
                sharp
            }
        };

        let rho = self.fillet_radius;
        if rho == 0.0 {
            return sharp;
        }
        if let Some((rem, sol)) = solve(rho) {
            if 2.0 * rem * r_root >= self.min_land {
                return keep_if_long_enough((rho, sol.0, sol.1, Some(sol.2), sol.3, false));
            }
        }

        // The requested fillet doesn't fit: use the largest one that does.
        let (mut lo, mut hi) = (0.0, rho);
        let mut best: Option<(f64, (f64, P, P, P))> = None;
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
        let Some((best_rem, best)) = best else { return sharp };
        if 2.0 * best_rem * r_root >= self.min_land {
            // limited by the contact zone
            return keep_if_long_enough((lo, best.0, best.1, Some(best.2), best.3, false));
        }
        // Limited by the gap: the two fillets meet in a fully rounded root.
        keep_if_long_enough((lo, best.0, best.1, Some(best.2), polar(r_root, land_center), true))
    }

    // --------------------------------------------------------------- spur gear

    pub fn spur_profile(&self, teeth: usize) -> Result<Profile> {
        self.check_teeth("spur", teeth)?;
        let (alpha, h) = (self.pressure_angle, self.height);
        let r = self.pitch_radius(teeth);
        let rb = r * alpha.cos();
        let r_tip = r + h / 2.0;
        let r_root = r - h / 2.0 - self.clearance;

        // Lowest point a mating tooth touches on this flank. Along the line of
        // action, measured from where it touches this gear's base circle:
        // - a rack's tip line meets it at r sin(alpha) - (h/2) / sin(alpha);
        // - the tip of the smallest ring it can sit in (teeth + 2 * min_spur
        //   teeth) meets it at sqrt(r_tip_ring^2 - rb_ring^2) - a sin(alpha),
        //   with a = r_ring - r the center distance.
        // Every other external mate reaches less far than the rack.
        let r_ring = self.pitch_radius(teeth + 2 * self.min_spur_teeth);
        let reach = (r * alpha.sin() - h / 2.0 / alpha.sin()).min(
            ((r_ring - h / 2.0).powi(2) - (r_ring * alpha.cos()).powi(2)).sqrt() - (r_ring - r) * alpha.sin(),
        );
        let r_contact = (rb * rb + reach * reach).sqrt();

        let flank = Flank::new(rb, r, alpha, (self.pitch - self.backlash) / 2.0);
        let (s_root, s_tip) = (flank.s_at(r_root), flank.s_at(r_tip));
        let partners = [Partner::Rack, Partner::Ring(teeth + 2 * self.min_spur_teeth)];
        let obstacles = self.driven_obstacles(teeth, &partners);
        let (rho, s_p, p, c, q, full_round) = self.fillet(
            &flank,
            1.0,
            r_root,
            s_root,
            s_tip,
            PI / teeth as f64,
            r_contact,
            &obstacles,
            (0.0, 2.0 * PI / teeth as f64), // the gap between tooth 0 and tooth 1
        );
        Ok(Profile { teeth, r_pitch: r, r_base: rb, r_tip, r_root, flank, s_tip, s_p, p, c, q, rho, full_round, r_contact })
    }

    /// Closed 2D outline of a spur gear centered at the origin,
    /// counterclockwise, as a list of (x, y) with no repeated end point.
    /// Tooth 0 is centered on the +x axis.
    pub fn spur_gear(&self, teeth: usize) -> Result<Vec<P>> {
        let g = self.spur_profile(teeth)?;
        let (dev, ds, step) = (self.max_deviation, self.ds, 2.0 * PI / teeth as f64);
        let fdev = self.fillet_deviation;
        let tip = g.flank.at(g.s_tip).0;
        let (p_lo, q_lo) = (mirror(g.p), mirror(g.q));

        let mut period = Vec::new();
        if g.rho > 0.0 {
            period.extend(arc_points(mirror(g.c.unwrap()), q_lo, p_lo, fdev, ds, true)); // lower fillet
        }
        period.extend(g.flank.points(g.s_p, g.s_tip, dev, ds, false).into_iter().map(mirror)); // lower flank, outward
        period.extend(arc_points((0.0, 0.0), mirror(tip), tip, dev, ds, false)); // tip land
        period.extend(g.flank.points(g.s_tip, g.s_p, dev, ds, false)); // upper flank, inward
        if g.rho > 0.0 {
            period.extend(arc_points(g.c.unwrap(), g.p, g.q, fdev, ds, true)); // upper fillet
        }
        if !g.full_round {
            period.extend(arc_points((0.0, 0.0), g.q, rotate(q_lo, step), dev, ds, false)); // root land
        }
        Ok((0..teeth).flat_map(|i| period.iter().map(move |&p| rotate(p, i as f64 * step))).collect())
    }

    // --------------------------------------------------------------- ring gear

    pub fn ring_profile(&self, teeth: usize) -> Result<Profile> {
        self.check_teeth("ring", teeth)?;
        let (alpha, h) = (self.pressure_angle, self.height);
        let r = self.pitch_radius(teeth);
        let rb = r * alpha.cos();
        let r_tip = r - h / 2.0; // ring teeth point inward
        let r_root = r + h / 2.0 + self.clearance;

        // Highest point a planet's tip touches on this flank. The largest
        // planet the ring can hold has teeth - 2 * min_spur_teeth teeth; its
        // tip meets the line of action at a sin(alpha) + sqrt(r_tip_p^2 - rb_p^2)
        // from where the line touches the ring's base circle.
        let r_p = self.pitch_radius(teeth - 2 * self.min_spur_teeth);
        let reach = (r - r_p) * alpha.sin() + ((r_p + h / 2.0).powi(2) - (r_p * alpha.cos()).powi(2)).sqrt();
        let r_contact = (rb * rb + reach * reach).sqrt();

        // The ring's gap has the shape of a spur tooth, backlash/2 wider.
        let flank = Flank::new(rb, r, alpha, (self.pitch + self.backlash) / 2.0);
        let (s_root, s_tip) = (flank.s_at(r_root), flank.s_at(r_tip));
        // Check against the smallest and the largest planet the ring can hold.
        let partners = [Partner::Spur(self.min_spur_teeth), Partner::Spur(teeth - 2 * self.min_spur_teeth)];
        let obstacles = self.driven_obstacles(teeth, &partners);
        let (rho, s_p, p, c, q, full_round) = self.fillet(
            &flank,
            -1.0,
            r_root,
            s_root,
            s_tip,
            0.0,
            r_contact,
            &obstacles,
            (-PI / teeth as f64, PI / teeth as f64), // gap 0, between the neighbouring tooth centers
        );
        Ok(Profile { teeth, r_pitch: r, r_base: rb, r_tip, r_root, flank, s_tip, s_p, p, c, q, rho, full_round, r_contact })
    }

    /// Closed 2D inner outline (the toothed edge) of a ring gear centered at
    /// the origin, counterclockwise, as a list of (x, y) with no repeated end
    /// point. Gap 0 is centered on the +x axis.
    pub fn ring_gear(&self, teeth: usize) -> Result<Vec<P>> {
        let g = self.ring_profile(teeth)?;
        let (dev, ds, step) = (self.max_deviation, self.ds, 2.0 * PI / teeth as f64);
        let fdev = self.fillet_deviation;
        let tip = g.flank.at(g.s_tip).0; // inner end of the upper flank
        let (p_lo, q_lo) = (mirror(g.p), mirror(g.q));

        let mut period: Vec<P> = g.flank.points(g.s_tip, g.s_p, dev, ds, true).into_iter().map(mirror).collect(); // lower flank, outward
        if g.rho > 0.0 {
            period.extend(arc_points(mirror(g.c.unwrap()), p_lo, q_lo, fdev, ds, true)); // lower fillet
        }
        if !g.full_round {
            period.extend(arc_points((0.0, 0.0), q_lo, g.q, dev, ds, true)); // root land
        }
        if g.rho > 0.0 {
            period.extend(arc_points(g.c.unwrap(), g.q, g.p, fdev, ds, true)); // upper fillet
        }
        period.extend(g.flank.points(g.s_p, g.s_tip, dev, ds, true)); // upper flank, inward
        period.extend(arc_points((0.0, 0.0), tip, rotate(mirror(tip), step), dev, ds, true)); // tip land
        Ok((0..teeth).flat_map(|i| period.iter().map(move |&p| rotate(p, i as f64 * step))).collect())
    }

    // ----------------------------------------------------------------- meshing

    /// Rotation of spur gear B so that it meshes with spur gear A with the
    /// backlash split equally between the two sides of every tooth.
    ///
    /// Gear A is centered at the origin and rotated by rotation_a (tooth 0 of
    /// an unrotated outline points along +x). Gear B is centered at distance
    /// r_a + r_b in the direction `direction`, so the pitch point is on A's
    /// pitch circle at angle `direction`.
    ///
    /// Centered mesh: whenever a tooth center of A is at the pitch point, a gap
    /// center of B is there too. Both outlines are symmetric about their tooth
    /// and gap center lines, so the play is then equal on both sides. The pitch
    /// circles roll without slipping, so an angle d of A past the pitch point
    /// matches an angle d * z_a / z_b of B past it, on the other side
    /// (external gears turn in opposite directions). B's gap 0 is at its
    /// rotation + pi / z_b, and the pitch point seen from B is at
    /// direction + pi:
    ///
    /// ```text
    /// rotation_b + pi / z_b - (direction + pi) = -(rotation_a - direction) * z_a / z_b
    /// ```
    ///
    /// Turning A by any angle and B by this formula keeps the mesh centered.
    pub fn spur_mesh_rotation(teeth_a: usize, rotation_a: f64, teeth_b: usize, direction: f64) -> f64 {
        direction + PI - PI / teeth_b as f64 - (rotation_a - direction) * teeth_a as f64 / teeth_b as f64
    }

    /// Rotation of a ring gear (centered at the origin) so that it meshes with
    /// a spur gear whose center is at distance r_ring - r_planet in the
    /// direction `direction`, backlash split equally.
    ///
    /// The pitch point is in the same direction seen from both centers, and
    /// the two gears turn the same way. Ring gap 0 points along its rotation,
    /// so a planet tooth center at the pitch point matches a ring gap center
    /// there when
    ///
    /// ```text
    /// rotation_ring - direction = (rotation_planet - direction) * z_planet / z_ring
    /// ```
    pub fn ring_mesh_rotation(teeth_planet: usize, rotation_planet: f64, teeth_ring: usize, direction: f64) -> f64 {
        direction + (rotation_planet - direction) * teeth_planet as f64 / teeth_ring as f64
    }

    // --------------------------------------------------------------- planetary

    /// Checks a planetary set and returns (ring teeth, [(planet center,
    /// planet rotation)], ring rotation), every mesh centered. The sun and
    /// ring are centered at the origin; planets are spaced evenly, planet 1 on
    /// the +x axis. The ring has teeth_sun + 2 * teeth_planet teeth, so the
    /// center distances match.
    pub fn planetary_layout(
        &self,
        teeth_sun: usize,
        teeth_planet: usize,
        planets: usize,
        sun_rotation: f64,
    ) -> Result<PlanetaryLayout> {
        self.check_teeth("spur", teeth_sun)?;
        self.check_teeth("spur", teeth_planet)?;
        if planets < 1 {
            return err("The number of planets must be a whole number, at least 1.");
        }
        let teeth_ring = teeth_sun + 2 * teeth_planet;
        if teeth_ring < self.min_ring_teeth {
            return err(format!(
                "The ring would have {} teeth (sun + 2 × planet), but ring gears in this gear set need \
                 at least {}. Use more sun or planet teeth.",
                teeth_ring, self.min_ring_teeth
            ));
        }
        // Planet k sits at angle 2 pi k / planets. Seen from the sun, that is
        // (teeth_sun / planets) * k teeth around; seen from the ring, the
        // planet's teeth line up with the ring's only if the total turns in
        // whole teeth, which works out to this divisibility condition.
        if !(teeth_sun + teeth_ring).is_multiple_of(planets) {
            return err(format!(
                "{} planets can't be spaced evenly: sun teeth + ring teeth ({} + {} = {}; the ring has \
                 sun + 2 × planet teeth) must be divisible by the number of planets.",
                planets,
                teeth_sun,
                teeth_ring,
                teeth_sun + teeth_ring
            ));
        }
        let a = self.pitch_radius(teeth_sun) + self.pitch_radius(teeth_planet);
        if planets > 1 {
            let gap = 2.0 * a * (PI / planets as f64).sin() - 2.0 * (self.pitch_radius(teeth_planet) + self.height / 2.0);
            if gap < self.clearance {
                return err(format!(
                    "{} planets don't fit around the sun: neighbouring planets' tips would be {:.3} mm \
                     apart. Use fewer or smaller planets.",
                    planets, gap
                ));
            }
        }
        let mut layout = Vec::new();
        for k in 0..planets {
            let direction = 2.0 * PI * k as f64 / planets as f64;
            let rotation = Self::spur_mesh_rotation(teeth_sun, sun_rotation, teeth_planet, direction);
            layout.push((polar(a, direction), rotation));
        }
        let ring_rotation = Self::ring_mesh_rotation(teeth_planet, layout[0].1, teeth_ring, 0.0);
        Ok((teeth_ring, layout, ring_rotation))
    }

    /// Outlines of a planetary set, placed and rotated so every mesh is
    /// centered: [sun, planet_1, ..., planet_n, ring].
    pub fn planetary_gear(&self, teeth_sun: usize, teeth_planet: usize, planets: usize, sun_rotation: f64) -> Result<Vec<Vec<P>>> {
        let (teeth_ring, layout, ring_rotation) = self.planetary_layout(teeth_sun, teeth_planet, planets, sun_rotation)?;
        let planet = self.spur_gear(teeth_planet)?;
        let mut outlines = vec![place(&self.spur_gear(teeth_sun)?, sun_rotation, (0.0, 0.0))];
        for (center, rotation) in layout {
            outlines.push(place(&planet, rotation, center));
        }
        outlines.push(place(&self.ring_gear(teeth_ring)?, ring_rotation, (0.0, 0.0)));
        Ok(outlines)
    }
}
