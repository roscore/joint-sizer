//! Spatial (3D) serial-chain joint sizing: gravity torque, inertia about each joint axis, dynamic
//! torque and actuator (motor + gearbox) fit checks, with a worst-case pose search inside joint
//! limits. Pure Rust, no dependencies.
//!
//! Model
//! - World frame: Z up, gravity along -Z.
//! - Joint `i` rotates about one axis of its local frame (X, Y or Z). Frames compose:
//!   R_i = R_{i-1} · Rot(axis_i, q_i), with R_{-1} = identity.
//! - Link `i` starts at joint `i` and points along a chosen local direction (±X, ±Y, ±Z) of
//!   frame `i`; joint `i + 1` sits at its end. The last link ends at the tip.
//! - Each link is a thin uniform rod. The actuator of joint `i` is a point mass at joint `i`
//!   (for i > 0 it rides on link `i - 1`). The payload is a point mass at the tip, optionally
//!   offset along the local +Z of the last frame.

/// Standard gravity, m/s².
pub const G: f64 = 9.80665;
/// Maximum number of joints supported by the UI.
pub const MAX_JOINTS: usize = 7;
/// Default gearbox efficiency.
pub const DEFAULT_EFFICIENCY: f64 = 0.85;

pub type V3 = [f64; 3];
type M3 = [[f64; 3]; 3];

fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn scale(a: V3, s: f64) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn mm(a: &M3, b: &M3) -> M3 {
    let mut r = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    r
}
fn mv(a: &M3, v: V3) -> V3 {
    [dot(a[0], v), dot(a[1], v), dot(a[2], v)]
}
const I3: M3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Joint rotation axis in the joint's local frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    pub fn unit(self) -> V3 {
        match self {
            Axis::X => [1.0, 0.0, 0.0],
            Axis::Y => [0.0, 1.0, 0.0],
            Axis::Z => [0.0, 0.0, 1.0],
        }
    }
    fn rot(self, q: f64) -> M3 {
        let (s, c) = q.sin_cos();
        match self {
            Axis::X => [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
            Axis::Y => [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]],
            Axis::Z => [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
        }
    }
    pub fn code(self) -> &'static str {
        match self {
            Axis::X => "x",
            Axis::Y => "y",
            Axis::Z => "z",
        }
    }
    pub fn from_code(s: &str) -> Axis {
        match s {
            "x" => Axis::X,
            "z" => Axis::Z,
            _ => Axis::Y,
        }
    }
}

/// Direction of a link in its joint's local frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dir {
    Px,
    Nx,
    Py,
    Ny,
    Pz,
    Nz,
}

impl Dir {
    pub const ALL: [Dir; 6] = [Dir::Px, Dir::Nx, Dir::Py, Dir::Ny, Dir::Pz, Dir::Nz];
    pub fn unit(self) -> V3 {
        match self {
            Dir::Px => [1.0, 0.0, 0.0],
            Dir::Nx => [-1.0, 0.0, 0.0],
            Dir::Py => [0.0, 1.0, 0.0],
            Dir::Ny => [0.0, -1.0, 0.0],
            Dir::Pz => [0.0, 0.0, 1.0],
            Dir::Nz => [0.0, 0.0, -1.0],
        }
    }
    pub fn code(self) -> &'static str {
        match self {
            Dir::Px => "+x",
            Dir::Nx => "-x",
            Dir::Py => "+y",
            Dir::Ny => "-y",
            Dir::Pz => "+z",
            Dir::Nz => "-z",
        }
    }
    pub fn from_code(s: &str) -> Dir {
        Dir::ALL.iter().copied().find(|d| d.code() == s).unwrap_or(Dir::Px)
    }
}

/// One joint, the link it drives and its actuator mass.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Joint {
    pub axis: Axis,
    pub dir: Dir,
    /// Link length, m.
    pub length: f64,
    /// Link mass, kg (thin uniform rod).
    pub mass: f64,
    /// Actuator mass at this joint, kg.
    pub actuator_mass: f64,
    /// Joint limits, degrees.
    pub min_deg: f64,
    pub max_deg: f64,
}

impl Joint {
    pub fn new(axis: Axis, dir: Dir, length: f64, mass: f64, actuator_mass: f64) -> Joint {
        Joint { axis, dir, length, mass, actuator_mass, min_deg: -180.0, max_deg: 180.0 }
    }
    pub fn limits(mut self, min_deg: f64, max_deg: f64) -> Joint {
        self.min_deg = min_deg;
        self.max_deg = max_deg;
        self
    }
    fn lo(&self) -> f64 {
        self.min_deg.min(self.max_deg)
    }
    fn hi(&self) -> f64 {
        self.min_deg.max(self.max_deg)
    }
}

/// A serial chain with a tip payload.
#[derive(Clone, Debug, PartialEq)]
pub struct Chain {
    pub joints: Vec<Joint>,
    /// Payload mass at the tip, kg.
    pub payload: f64,
    /// Payload centre-of-mass offset from the tip along the last frame's local +Z, m.
    pub payload_offset: f64,
}

/// Motor + gearbox entered by the user for one joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Actuator {
    /// Motor continuous (rated) torque, N·m.
    pub cont_torque: f64,
    /// Motor peak torque, N·m.
    pub peak_torque: f64,
    /// Motor speed at the operating point, rpm.
    pub rpm: f64,
    /// Gear ratio N (output turns slower by N).
    pub ratio: f64,
    /// Gearbox efficiency 0..1.
    pub efficiency: f64,
}

/// Motion targets shared by all joints.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Motion {
    /// Peak angular acceleration, rad/s².
    pub alpha: f64,
    /// Maximum angular speed, rad/s.
    pub omega: f64,
    /// Safety factor applied to torque requirements.
    pub safety: f64,
}

struct Mass {
    c: V3,
    m: f64,
    /// Thin-rod direction and length (None = point mass).
    rod: Option<(V3, f64)>,
    /// The mass loads joints 0..=distal_of.
    distal_of: usize,
}

/// The chain evaluated at one pose.
pub struct Pose {
    /// Joint positions plus the tip (n + 1 points), world frame.
    pub points: Vec<V3>,
    /// World-frame unit axis of each joint.
    pub axes: Vec<V3>,
    /// Payload centre of mass.
    pub payload_at: V3,
    masses: Vec<Mass>,
}

/// Forward kinematics at joint angles in degrees (missing entries = 0).
pub fn pose(chain: &Chain, q_deg: &[f64]) -> Pose {
    let n = chain.joints.len();
    let mut r = I3;
    let mut p = [0.0; 3];
    let mut points = vec![p];
    let mut axes = Vec::with_capacity(n);
    let mut masses = Vec::new();
    for (i, j) in chain.joints.iter().enumerate() {
        r = mm(&r, &j.axis.rot(q_deg.get(i).copied().unwrap_or(0.0).to_radians()));
        axes.push(mv(&r, j.axis.unit()));
        let d = mv(&r, j.dir.unit());
        let next = add(p, scale(d, j.length));
        masses.push(Mass { c: scale(add(p, next), 0.5), m: j.mass, rod: Some((d, j.length)), distal_of: i });
        if i > 0 {
            masses.push(Mass { c: p, m: j.actuator_mass, rod: None, distal_of: i - 1 });
        }
        p = next;
        points.push(p);
    }
    let payload_at = add(p, mv(&r, [0.0, 0.0, chain.payload_offset]));
    if n > 0 {
        masses.push(Mass { c: payload_at, m: chain.payload, rod: None, distal_of: n - 1 });
    }
    Pose { points, axes, payload_at, masses }
}

impl Pose {
    /// (|gravity torque|, inertia) about joint `j`'s axis.
    pub fn load(&self, j: usize) -> (f64, f64) {
        let a = self.axes[j];
        let o = self.points[j];
        let mut tau = 0.0;
        let mut inertia = 0.0;
        for m in self.masses.iter().filter(|m| m.distal_of >= j) {
            let r = sub(m.c, o);
            tau += dot(a, cross(r, [0.0, 0.0, -m.m * G]));
            let ra = dot(r, a);
            inertia += m.m * (dot(r, r) - ra * ra).max(0.0);
            if let Some((u, l)) = m.rod {
                let ua = dot(u, a);
                inertia += m.m * l * l / 12.0 * (1.0 - ua * ua).max(0.0);
            }
        }
        (tau.abs(), inertia)
    }
}

/// Deterministic pseudo-random numbers in [0, 1).
struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Maximise `|τ_g| + weight · I` about joint `j` over all poses inside the joint limits.
/// Coordinate search on a 25-point grid per joint from several seeds, then step-halving refine.
/// Returns (best pose in degrees, |τ_g| there, I there).
pub fn worst_pose(chain: &Chain, j: usize, weight: f64) -> (Vec<f64>, f64, f64) {
    let n = chain.joints.len();
    let eval = |q: &[f64]| {
        let (t, i) = pose(chain, q).load(j);
        (t + weight * i, t, i)
    };
    let clamp = |k: usize, v: f64| v.clamp(chain.joints[k].lo(), chain.joints[k].hi());
    let mut seeds: Vec<Vec<f64>> = vec![(0..n).map(|k| clamp(k, 0.0)).collect()];
    let mut rng = Lcg(0x5eed + j as u64);
    for _ in 0..12 {
        seeds.push((0..n).map(|k| {
            let (lo, hi) = (chain.joints[k].lo(), chain.joints[k].hi());
            lo + (hi - lo) * rng.next()
        }).collect());
    }
    let mut best = (seeds[0].clone(), f64::NEG_INFINITY, 0.0, 0.0);
    for mut q in seeds {
        let mut cur = eval(&q).0;
        for _ in 0..3 {
            for k in 0..n {
                let (lo, hi) = (chain.joints[k].lo(), chain.joints[k].hi());
                for s in 0..=24 {
                    let old = q[k];
                    q[k] = lo + (hi - lo) * s as f64 / 24.0;
                    let v = eval(&q).0;
                    if v > cur + 1e-12 {
                        cur = v;
                    } else {
                        q[k] = old;
                    }
                }
            }
        }
        let mut step = 7.5;
        while step > 0.02 {
            let mut improved = false;
            for k in 0..n {
                for sgn in [-1.0, 1.0] {
                    let old = q[k];
                    q[k] = clamp(k, old + sgn * step);
                    let v = eval(&q).0;
                    if v > cur + 1e-12 {
                        cur = v;
                        improved = true;
                    } else {
                        q[k] = old;
                    }
                }
            }
            if !improved {
                step /= 2.0;
            }
        }
        if cur > best.1 {
            let (v, t, i) = eval(&q);
            best = (q, v, t, i);
        }
    }
    (best.0, best.2, best.3)
}

/// Requirement for one joint (safety factor already applied to torques).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Requirement {
    /// Worst-case static gravity torque, N·m (no safety factor).
    pub gravity: f64,
    /// Inertia about the joint axis at the peak-torque pose, kg·m².
    pub inertia: f64,
    /// Continuous torque needed = worst gravity × SF.
    pub cont: f64,
    /// Peak torque needed = max over poses of (gravity + I·α) × SF.
    pub peak: f64,
    /// Output speed needed, rpm.
    pub rpm: f64,
}

/// Requirement plus the poses (degrees) where the worst cases occur.
#[derive(Clone, Debug, PartialEq)]
pub struct JointLoad {
    pub req: Requirement,
    pub gravity_pose: Vec<f64>,
    pub peak_pose: Vec<f64>,
}

pub fn requirements(chain: &Chain, motion: Motion) -> Vec<JointLoad> {
    let rpm = rad_s_to_rpm(motion.omega);
    (0..chain.joints.len())
        .map(|j| {
            let (gq, g, _) = worst_pose(chain, j, 0.0);
            let (pq, pg, pi) = if motion.alpha > 0.0 {
                worst_pose(chain, j, motion.alpha)
            } else {
                let (_, i) = pose(chain, &gq).load(j);
                (gq.clone(), g, i)
            };
            let peak = (pg + pi * motion.alpha).max(g);
            JointLoad {
                req: Requirement { gravity: g, inertia: pi, cont: g * motion.safety, peak: peak * motion.safety, rpm },
                gravity_pose: gq,
                peak_pose: pq,
            }
        })
        .collect()
}

/// Output side of a motor + gearbox.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Output {
    pub cont: f64,
    pub peak: f64,
    pub rpm: f64,
}

pub fn output(a: &Actuator) -> Output {
    Output {
        cont: a.cont_torque * a.ratio * a.efficiency,
        peak: a.peak_torque * a.ratio * a.efficiency,
        rpm: if a.ratio > 0.0 { a.rpm / a.ratio } else { 0.0 },
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fit {
    Ok,
    Tight,
    Fail,
}

/// Smallest of available/needed over continuous torque, peak torque and speed.
pub fn min_margin(req: &Requirement, out: &Output) -> f64 {
    let r = |have: f64, need: f64| if need <= 0.0 { f64::INFINITY } else { have / need };
    r(out.cont, req.cont).min(r(out.peak, req.peak)).min(r(out.rpm, req.rpm))
}

/// Ok with ≥ 10 % headroom on every axis, Tight when it just passes, Fail otherwise.
pub fn fit(req: &Requirement, out: &Output) -> Fit {
    let m = min_margin(req, out);
    if m < 1.0 - 1e-9 {
        Fit::Fail
    } else if m < 1.1 {
        Fit::Tight
    } else {
        Fit::Ok
    }
}

/// Range of gear ratios [lo, hi] that meet torque (continuous and peak) and speed for this motor.
/// `hi` is infinite when no speed is required. None when no ratio works.
pub fn ratio_range(req: &Requirement, a: &Actuator) -> Option<(f64, f64)> {
    let eta = a.efficiency;
    if eta <= 0.0 || a.cont_torque <= 0.0 || a.peak_torque <= 0.0 {
        return None;
    }
    let lo = (req.cont / (a.cont_torque * eta)).max(req.peak / (a.peak_torque * eta));
    let hi = if req.rpm > 0.0 { a.rpm / req.rpm } else { f64::INFINITY };
    if lo <= hi { Some((lo, hi)) } else { None }
}

pub fn rad_s_to_rpm(w: f64) -> f64 {
    w * 60.0 / (2.0 * std::f64::consts::PI)
}

/// N·m → kgf·cm (1 N·m ≈ 10.197 kgf·cm).
pub fn nm_to_kgfcm(t: f64) -> f64 {
    t * 100.0 / G
}

/// Example chains with matching actuators.
pub fn preset(name: &str) -> Option<(Chain, Vec<Actuator>)> {
    use Axis::*;
    use Dir::*;
    let act = |c, p, rpm, n, e| Actuator { cont_torque: c, peak_torque: p, rpm, ratio: n, efficiency: e };
    let j = Joint::new;
    match name {
        // Base yaw + shoulder / elbow pitch.
        "desk_arm" => Some((
            Chain {
                joints: vec![j(Z, Pz, 0.08, 0.2, 0.4), j(Y, Px, 0.25, 0.3, 0.4).limits(-90.0, 90.0), j(Y, Px, 0.20, 0.2, 0.3).limits(-150.0, 150.0)],
                payload: 0.5,
                payload_offset: 0.0,
            },
            vec![act(0.3, 0.9, 3000.0, 50.0, 0.85), act(0.3, 0.9, 3000.0, 50.0, 0.85), act(0.15, 0.45, 3000.0, 50.0, 0.85)],
        )),
        // Hip roll + hip pitch + knee + ankle pitch, leg hanging down.
        "leg" => Some((
            Chain {
                joints: vec![
                    j(X, Nz, 0.05, 0.5, 1.0).limits(-30.0, 30.0),
                    j(Y, Nz, 0.40, 3.0, 1.0).limits(-120.0, 30.0),
                    j(Y, Nz, 0.40, 2.0, 0.8).limits(0.0, 150.0),
                    j(Y, Px, 0.12, 0.5, 0.5).limits(-45.0, 45.0),
                ],
                payload: 0.0,
                payload_offset: 0.0,
            },
            vec![act(6.0, 18.0, 300.0, 9.0, 0.9), act(6.0, 18.0, 300.0, 9.0, 0.9), act(6.0, 18.0, 300.0, 9.0, 0.9), act(1.5, 4.5, 300.0, 9.0, 0.9)],
        )),
        // UR-style 6-DoF arm: base yaw, shoulder / elbow / wrist pitch, wrist yaw, tool roll.
        "arm6" => Some((
            Chain {
                joints: vec![
                    j(Z, Pz, 0.15, 1.0, 1.2),
                    j(Y, Px, 0.30, 1.2, 0.9),
                    j(Y, Px, 0.25, 0.8, 0.7),
                    j(Y, Px, 0.08, 0.3, 0.4),
                    j(Z, Px, 0.08, 0.3, 0.4),
                    j(X, Px, 0.05, 0.1, 0.3),
                ],
                payload: 1.0,
                payload_offset: 0.05,
            },
            vec![
                act(1.0, 3.0, 3000.0, 100.0, 0.8),
                act(1.0, 3.0, 3000.0, 100.0, 0.8),
                act(0.6, 1.8, 3000.0, 100.0, 0.8),
                act(0.2, 0.6, 3000.0, 100.0, 0.8),
                act(0.2, 0.6, 3000.0, 100.0, 0.8),
                act(0.2, 0.6, 3000.0, 100.0, 0.8),
            ],
        )),
        // Franka-style 7-DoF arm (yaw/pitch alternating).
        "arm7" => Some((
            Chain {
                joints: vec![
                    j(Z, Pz, 0.33, 2.0, 1.5),
                    j(Y, Pz, 0.0, 0.0, 1.2).limits(-105.0, 105.0),
                    j(Z, Pz, 0.32, 1.5, 1.0),
                    j(Y, Pz, 0.0, 0.0, 0.9).limits(-175.0, 0.0),
                    j(Z, Pz, 0.38, 1.2, 0.7),
                    j(Y, Px, 0.09, 0.3, 0.5).limits(0.0, 215.0),
                    j(Z, Nz, 0.10, 0.2, 0.4),
                ],
                payload: 1.0,
                payload_offset: 0.05,
            },
            vec![
                act(1.5, 4.5, 3000.0, 100.0, 0.8),
                act(1.5, 4.5, 3000.0, 100.0, 0.8),
                act(1.0, 3.0, 3000.0, 100.0, 0.8),
                act(1.0, 3.0, 3000.0, 100.0, 0.8),
                act(0.4, 1.2, 3000.0, 100.0, 0.8),
                act(0.4, 1.2, 3000.0, 100.0, 0.8),
                act(0.4, 1.2, 3000.0, 100.0, 0.8),
            ],
        )),
        _ => None,
    }
}
