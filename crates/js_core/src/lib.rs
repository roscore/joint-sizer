//! Planar serial-chain joint sizing: gravity torque, distal inertia, dynamic torque and
//! actuator (motor + gearbox) fit checks. Pure Rust, no dependencies.
//!
//! Model: revolute joints in a vertical plane, gravity along -Y. Link `i` runs from joint `i`
//! to joint `i + 1` (the last link ends at the tip). Each link is a uniform rod; the actuator of
//! joint `i` is a point mass sitting at joint `i`; the payload is a point mass at the tip.
//! Angles are relative (joint `i` rotates link `i` relative to link `i - 1`), 0 = horizontal.

/// Standard gravity, m/s².
pub const G: f64 = 9.80665;

/// One link of the chain plus the actuator that drives it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Link {
    /// Link length in metres.
    pub length: f64,
    /// Link mass in kg (uniform rod).
    pub mass: f64,
    /// Mass of the actuator at the proximal joint of this link, kg.
    pub actuator_mass: f64,
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

/// Orientation of a joint axis when the chain is fully extended along +X.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    /// Horizontal axis, perpendicular to the arm (shoulder, elbow, knee). Gravity-loaded.
    Pitch,
    /// Vertical axis (base turntable, wrist yaw). No gravity torque, full swing inertia.
    Yaw,
    /// Axis along the arm (forearm / wrist roll). Loaded only through the roll offset.
    Roll,
}

impl Axis {
    pub fn code(self) -> &'static str {
        match self {
            Axis::Pitch => "pitch",
            Axis::Yaw => "yaw",
            Axis::Roll => "roll",
        }
    }
    pub fn from_code(s: &str) -> Axis {
        match s {
            "yaw" => Axis::Yaw,
            "roll" => Axis::Roll,
            _ => Axis::Pitch,
        }
    }
}

/// Default distance of the distal centre of mass from a roll axis, m (gripper / tool offset).
pub const DEFAULT_ROLL_OFFSET: f64 = 0.05;

/// Default gearbox efficiency.
pub const DEFAULT_EFFICIENCY: f64 = 0.85;

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

/// Point masses of the chain at a pose: (x, y, mass, own inertia about its COM, first joint index
/// whose torque it loads). A mass "belongs" to joint j's load when its index >= j.
struct Mass {
    x: f64,
    y: f64,
    m: f64,
    i_com: f64,
    /// The mass loads joint j for all j <= `distal_of`.
    distal_of: usize,
}

/// Joint positions (n + 1 points including the tip) for relative angles in degrees.
pub fn joint_positions(links: &[Link], angles_deg: &[f64]) -> Vec<(f64, f64)> {
    let mut pts = Vec::with_capacity(links.len() + 1);
    let (mut x, mut y, mut th) = (0.0, 0.0, 0.0);
    pts.push((x, y));
    for (i, l) in links.iter().enumerate() {
        th += angles_deg.get(i).copied().unwrap_or(0.0).to_radians();
        x += l.length * th.cos();
        y += l.length * th.sin();
        pts.push((x, y));
    }
    pts
}

fn masses(links: &[Link], angles_deg: &[f64], payload: f64) -> (Vec<(f64, f64)>, Vec<Mass>) {
    let pts = joint_positions(links, angles_deg);
    let mut ms = Vec::new();
    for (i, l) in links.iter().enumerate() {
        let (a, b) = (pts[i], pts[i + 1]);
        ms.push(Mass {
            x: (a.0 + b.0) / 2.0,
            y: (a.1 + b.1) / 2.0,
            m: l.mass,
            i_com: l.mass * l.length * l.length / 12.0,
            distal_of: i,
        });
        // The actuator at joint i rides on link i-1: it loads joints 0..i-1, not joint i itself
        // (it sits on that joint's axis). Joint 0's actuator sits on the base and loads nothing.
        if i > 0 {
            ms.push(Mass { x: a.0, y: a.1, m: l.actuator_mass, i_com: 0.0, distal_of: i - 1 });
        }
    }
    if let Some(&tip) = pts.last() {
        if !links.is_empty() {
            ms.push(Mass { x: tip.0, y: tip.1, m: payload, i_com: 0.0, distal_of: links.len() - 1 });
        }
    }
    (pts, ms)
}

/// Static gravity torque at each joint (N·m, magnitude) for the given pose.
pub fn gravity_torques(links: &[Link], angles_deg: &[f64], payload: f64) -> Vec<f64> {
    let (pts, ms) = masses(links, angles_deg, payload);
    (0..links.len())
        .map(|j| {
            let jx = pts[j].0;
            ms.iter().filter(|m| m.distal_of >= j).map(|m| m.m * G * (m.x - jx)).sum::<f64>().abs()
        })
        .collect()
}

/// Inertia of everything distal to each joint about that joint's axis (kg·m², parallel axis).
pub fn distal_inertias(links: &[Link], angles_deg: &[f64], payload: f64) -> Vec<f64> {
    let (pts, ms) = masses(links, angles_deg, payload);
    (0..links.len())
        .map(|j| {
            let (jx, jy) = pts[j];
            ms.iter()
                .filter(|m| m.distal_of >= j)
                .map(|m| m.i_com + m.m * ((m.x - jx).powi(2) + (m.y - jy).powi(2)))
                .sum()
        })
        .collect()
}

/// Requirement for one joint (safety factor already applied to torques).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Requirement {
    /// Worst-case static gravity torque, N·m (no safety factor).
    pub gravity: f64,
    /// Distal inertia about the joint, fully extended, kg·m².
    pub inertia: f64,
    /// Continuous torque needed = gravity × SF.
    pub cont: f64,
    /// Peak torque needed = (gravity + I·α) × SF.
    pub peak: f64,
    /// Output speed needed, rpm.
    pub rpm: f64,
}

/// Worst case for an all-pitch chain = fully extended horizontal (all angles 0), which maximises
/// both the gravity moment arm and the distal inertia for every joint.
pub fn requirements(links: &[Link], payload: f64, motion: Motion) -> Vec<Requirement> {
    requirements_axes(links, &[], payload, motion, DEFAULT_ROLL_OFFSET)
}

/// Requirements with a per-joint axis type (missing entries = Pitch), fully extended chain.
/// - Pitch: gravity torque and inertia about a horizontal axis.
/// - Yaw: no gravity torque; same swing inertia (distances lie in the horizontal plane).
/// - Roll: everything distal is lumped at `roll_offset` from the axis (worst case horizontal):
///   torque = m·g·r, inertia = m·r². An approximation for wrists and tools.
pub fn requirements_axes(links: &[Link], axes: &[Axis], payload: f64, motion: Motion, roll_offset: f64) -> Vec<Requirement> {
    let zeros = vec![0.0; links.len()];
    let tg = gravity_torques(links, &zeros, payload);
    let inertia = distal_inertias(links, &zeros, payload);
    let (_, ms) = masses(links, &zeros, payload);
    let rpm = rad_s_to_rpm(motion.omega);
    let r = roll_offset.max(0.0);
    (0..links.len())
        .map(|j| {
            let (g, i) = match axes.get(j).copied().unwrap_or(Axis::Pitch) {
                Axis::Pitch => (tg[j], inertia[j]),
                Axis::Yaw => (0.0, inertia[j]),
                Axis::Roll => {
                    let m: f64 = ms.iter().filter(|m| m.distal_of >= j).map(|m| m.m).sum();
                    (m * G * r, m * r * r)
                }
            };
            Requirement {
                gravity: g,
                inertia: i,
                cont: g * motion.safety,
                peak: (g + i * motion.alpha) * motion.safety,
                rpm,
            }
        })
        .collect()
}

/// Maximum number of joints supported by the UI.
pub const MAX_JOINTS: usize = 7;

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

/// A ready-made chain: links, axis per joint, payload, actuator per joint.
pub type Preset = (Vec<Link>, Vec<Axis>, f64, Vec<Actuator>);

/// Example chains.
pub fn preset(name: &str) -> Option<Preset> {
    use Axis::*;
    let act = |c, p, rpm, n, e| Actuator { cont_torque: c, peak_torque: p, rpm, ratio: n, efficiency: e };
    let l = |length, mass, actuator_mass| Link { length, mass, actuator_mass };
    match name {
        "desk_arm" => Some((
            vec![l(0.25, 0.3, 0.4), l(0.20, 0.2, 0.3)],
            vec![Pitch, Pitch],
            0.5,
            vec![act(0.3, 0.9, 3000.0, 50.0, 0.85), act(0.15, 0.45, 3000.0, 50.0, 0.85)],
        )),
        "leg" => Some((
            vec![l(0.40, 3.0, 1.0), l(0.40, 2.0, 0.8), l(0.10, 0.5, 0.5)],
            vec![Pitch, Pitch, Pitch],
            0.0,
            vec![act(6.0, 18.0, 300.0, 9.0, 0.9), act(6.0, 18.0, 300.0, 9.0, 0.9), act(1.5, 4.5, 300.0, 9.0, 0.9)],
        )),
        // UR-style 6-DoF arm: base yaw, shoulder / elbow / wrist pitch, wrist yaw, tool roll.
        "arm6" => Some((
            vec![
                l(0.05, 0.5, 1.2),
                l(0.30, 1.2, 0.9),
                l(0.25, 0.8, 0.7),
                l(0.08, 0.3, 0.4),
                l(0.08, 0.3, 0.4),
                l(0.05, 0.1, 0.3),
            ],
            vec![Yaw, Pitch, Pitch, Pitch, Yaw, Roll],
            1.0,
            vec![
                act(1.0, 3.0, 3000.0, 100.0, 0.8),
                act(1.0, 3.0, 3000.0, 100.0, 0.8),
                act(0.6, 1.8, 3000.0, 100.0, 0.8),
                act(0.2, 0.6, 3000.0, 100.0, 0.8),
                act(0.2, 0.6, 3000.0, 100.0, 0.8),
                act(0.2, 0.6, 3000.0, 100.0, 0.8),
            ],
        )),
        _ => None,
    }
}
