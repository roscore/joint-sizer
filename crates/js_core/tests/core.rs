use js_core::*;
use Axis::*;
use Dir::*;

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs().max(1e-9)
}

fn chain(joints: Vec<Joint>, payload: f64) -> Chain {
    Chain { joints, payload, payload_offset: 0.0 }
}

const STILL: Motion = Motion { alpha: 0.0, omega: 0.0, safety: 1.0 };

#[test]
fn planar_pitch_chain_matches_hand_calculation() {
    // 0.5 m + 0.5 m, 1 kg rods, 0.2 kg actuator at the elbow, 1 kg payload; worst = horizontal.
    let c = chain(vec![Joint::new(Y, Px, 0.5, 1.0, 0.0), Joint::new(Y, Px, 0.5, 1.0, 0.2)], 1.0);
    let r = requirements(&c, STILL);
    let base = G * (1.0 * 0.25 + 0.2 * 0.5 + 1.0 * 0.75 + 1.0 * 1.0);
    let elbow = G * (1.0 * 0.25 + 1.0 * 0.5);
    assert!(close(r[0].req.gravity, base, 1e-9), "{} vs {}", r[0].req.gravity, base);
    assert!(close(r[1].req.gravity, elbow, 1e-9), "{} vs {}", r[1].req.gravity, elbow);
}

#[test]
fn vertical_yaw_axis_never_sees_gravity() {
    let c = chain(vec![Joint::new(Z, Pz, 0.1, 1.0, 0.0), Joint::new(Y, Px, 0.4, 1.0, 0.5), Joint::new(Y, Px, 0.3, 0.5, 0.3)], 1.0);
    let r = requirements(&c, STILL);
    assert!(r[0].req.gravity < 1e-9, "{}", r[0].req.gravity);
    assert!(r[1].req.gravity > 1.0);
}

#[test]
fn rod_inertia_about_each_axis() {
    // Rod along X spun about Z (end-on): m L² / 3.
    let p = pose(&chain(vec![Joint::new(Z, Px, 1.2, 3.0, 0.0)], 0.0), &[0.0]);
    assert!(close(p.load(0).1, 3.0 * 1.44 / 3.0, 1e-12));
    // Same rod spun about its own axis: thin rod → 0, and no gravity torque.
    let p = pose(&chain(vec![Joint::new(X, Px, 1.2, 3.0, 0.0)], 0.0), &[37.0]);
    assert!(p.load(0).0 < 1e-12 && p.load(0).1 < 1e-12);
    // Rod sticking out sideways from a roll axis behaves like a pitch joint.
    let c = chain(vec![Joint::new(X, Py, 1.0, 2.0, 0.0)], 0.0);
    let r = requirements(&c, Motion { alpha: 3.0, omega: 0.0, safety: 1.0 });
    assert!(close(r[0].req.gravity, 2.0 * G * 0.5, 1e-9));
    assert!(close(r[0].req.peak, 2.0 * G * 0.5 + 2.0 / 3.0 * 3.0, 1e-9));
}

#[test]
fn joint_limits_bound_the_search() {
    // Horizontal (0°) is outside [-90°, -80°]; the worst case is the -80° limit.
    let c = chain(vec![Joint::new(Y, Px, 1.0, 2.0, 0.0).limits(-90.0, -80.0)], 0.0);
    let r = &requirements(&c, STILL)[0];
    assert!(close(r.req.gravity, 2.0 * G * 0.5 * 80f64.to_radians().cos(), 1e-9), "{}", r.req.gravity);
    assert!(close(r.gravity_pose[0], -80.0, 1e-9));
}

#[test]
fn payload_offset_loads_a_roll_joint() {
    let c = Chain { joints: vec![Joint::new(X, Px, 0.1, 0.0, 0.0)], payload: 2.0, payload_offset: 0.05 };
    let r = &requirements(&c, STILL)[0];
    assert!(close(r.req.gravity, 2.0 * G * 0.05, 1e-9), "{}", r.req.gravity);
}

#[test]
fn search_bounds_random_poses_on_six_dof_arm() {
    let (c, _) = preset("arm6").unwrap();
    let alpha = 4.0;
    let r = requirements(&c, Motion { alpha, omega: 1.0, safety: 1.0 });
    let mut s = 12345u64;
    let mut rnd = || {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((s >> 11) as f64 / (1u64 << 53) as f64) * 360.0 - 180.0
    };
    for _ in 0..400 {
        let q: Vec<f64> = (0..6).map(|_| rnd()).collect();
        let p = pose(&c, &q);
        for j in 0..6 {
            let (t, i) = p.load(j);
            assert!(t <= r[j].req.gravity * (1.0 + 1e-6) + 1e-9, "joint {j} gravity {t} > {}", r[j].req.gravity);
            assert!(t + i * alpha <= r[j].req.peak * (1.0 + 1e-6) + 1e-9, "joint {j} peak");
        }
    }
}

#[test]
fn gearbox_scales_torque_and_speed() {
    let a = Actuator { cont_torque: 0.5, peak_torque: 1.5, rpm: 3000.0, ratio: 20.0, efficiency: 0.8 };
    let o = output(&a);
    assert!(close(o.cont, 8.0, 1e-12) && close(o.peak, 24.0, 1e-12) && close(o.rpm, 150.0, 1e-12));
}

#[test]
fn ratio_range_matches_fit_check() {
    let req = Requirement { gravity: 10.0, inertia: 0.1, cont: 15.0, peak: 30.0, rpm: 30.0 };
    let base = Actuator { cont_torque: 0.5, peak_torque: 1.5, rpm: 3000.0, ratio: 1.0, efficiency: 0.85 };
    let (lo, hi) = ratio_range(&req, &base).expect("range");
    let at = |n: f64| fit(&req, &output(&Actuator { ratio: n, ..base }));
    assert_ne!(at(lo * 1.0001), Fit::Fail);
    assert_ne!(at(hi * 0.9999), Fit::Fail);
    assert_eq!(at(lo * 0.98), Fit::Fail);
    assert_eq!(at(hi * 1.02), Fit::Fail);
    assert!(ratio_range(&req, &Actuator { rpm: 300.0, ..base }).is_none());
}

#[test]
fn presets_are_consistent_and_fit() {
    assert!(close(nm_to_kgfcm(1.0), 10.197, 0.0001));
    let m = Motion { alpha: 5.0, omega: 90f64.to_radians(), safety: 1.5 };
    for name in ["desk_arm", "leg", "arm6", "arm7"] {
        let (c, acts) = preset(name).unwrap();
        assert_eq!(c.joints.len(), acts.len());
        assert!(c.joints.len() <= MAX_JOINTS);
        for (j, (l, a)) in requirements(&c, m).iter().zip(&acts).enumerate() {
            assert_ne!(fit(&l.req, &output(a)), Fit::Fail, "{name} J{} {:?}", j + 1, l.req);
        }
    }
    assert_eq!(preset("arm7").unwrap().0.joints.len(), 7);
    assert!(preset("nope").is_none());
}
