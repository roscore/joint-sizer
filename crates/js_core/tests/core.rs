use js_core::*;

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs().max(1e-12)
}

fn rod(length: f64, mass: f64) -> Link {
    Link { length, mass, actuator_mass: 0.0 }
}

#[test]
fn single_rod_horizontal_and_vertical() {
    let links = [rod(1.0, 2.0)];
    let t = gravity_torques(&links, &[0.0], 0.0);
    assert!(close(t[0], 9.80665, 0.001), "{t:?}");
    let v = gravity_torques(&links, &[90.0], 0.0);
    assert!(v[0].abs() < 1e-9, "{v:?}");
}

#[test]
fn two_links_with_payload_sum_m_g_x() {
    // 0.5 m + 0.5 m, 1 kg rods, 0.2 kg actuator at the elbow, 1 kg payload at the tip.
    let links = [
        Link { length: 0.5, mass: 1.0, actuator_mass: 0.0 },
        Link { length: 0.5, mass: 1.0, actuator_mass: 0.2 },
    ];
    let t = gravity_torques(&links, &[0.0, 0.0], 1.0);
    let base = G * (1.0 * 0.25 + 0.2 * 0.5 + 1.0 * 0.75 + 1.0 * 1.0);
    let elbow = G * (1.0 * 0.25 + 1.0 * 0.5); // elbow actuator sits on its own axis: excluded
    assert!(close(t[0], base, 1e-9), "{} vs {}", t[0], base);
    assert!(close(t[1], elbow, 1e-9), "{} vs {}", t[1], elbow);
}

#[test]
fn inertia_rod_about_end_and_point_mass() {
    let i = distal_inertias(&[rod(1.2, 3.0)], &[0.0], 0.0);
    assert!(close(i[0], 3.0 * 1.2 * 1.2 / 3.0, 1e-9));
    let p = distal_inertias(&[rod(0.8, 0.0)], &[37.0], 2.0);
    assert!(close(p[0], 2.0 * 0.8 * 0.8, 1e-9)); // pose does not change distance to the axis
}

#[test]
fn worst_case_bounds_other_poses() {
    let links = [rod(0.4, 1.0), rod(0.3, 0.6), rod(0.2, 0.3)];
    let worst = gravity_torques(&links, &[0.0; 3], 0.5);
    for a in (-90..=90).step_by(15) {
        for b in (-90..=90).step_by(30) {
            let t = gravity_torques(&links, &[a as f64, b as f64, -(b as f64)], 0.5);
            for j in 0..3 {
                assert!(t[j] <= worst[j] + 1e-9, "joint {j} pose {a},{b}");
            }
        }
    }
}

#[test]
fn requirements_apply_safety_and_dynamics() {
    let links = [rod(1.0, 2.0)];
    let r = requirements(&links, 0.0, Motion { alpha: 3.0, omega: 2.0 * std::f64::consts::PI, safety: 1.5 })[0];
    assert!(close(r.cont, 9.80665 * 1.5, 1e-9));
    assert!(close(r.peak, (9.80665 + 2.0 / 3.0 * 3.0) * 1.5, 1e-9));
    assert!(close(r.rpm, 60.0, 1e-9));
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
    let slow = Actuator { rpm: 300.0, ..base };
    assert!(ratio_range(&req, &slow).is_none()); // needs N ≥ 35 for torque but N ≤ 10 for speed
}

#[test]
fn units_and_presets() {
    assert!(close(nm_to_kgfcm(1.0), 10.197, 0.0001));
    for p in ["desk_arm", "leg", "arm6"] {
        let (links, axes, _, acts) = preset(p).unwrap();
        assert_eq!(links.len(), acts.len());
        assert_eq!(links.len(), axes.len());
        assert!(links.len() <= MAX_JOINTS);
    }
    assert!(preset("nope").is_none());
}

#[test]
fn yaw_has_no_gravity_but_same_inertia() {
    let links = [rod(0.5, 1.0), rod(0.5, 1.0)];
    let m = Motion { alpha: 2.0, omega: 1.0, safety: 1.0 };
    let pitch = requirements_axes(&links, &[Axis::Pitch, Axis::Pitch], 0.5, m, 0.05);
    let yaw = requirements_axes(&links, &[Axis::Yaw, Axis::Pitch], 0.5, m, 0.05);
    assert_eq!(yaw[0].gravity, 0.0);
    assert!(close(yaw[0].inertia, pitch[0].inertia, 1e-12));
    assert!(close(yaw[0].peak, pitch[0].inertia * 2.0, 1e-12));
    assert_eq!(yaw[1], pitch[1]); // later joints are unaffected
    assert_eq!(requirements(&links, 0.5, m), pitch); // default = all pitch
}

#[test]
fn roll_lumps_distal_mass_at_offset() {
    let links = [rod(0.3, 1.0), Link { length: 0.1, mass: 0.2, actuator_mass: 0.3 }];
    let m = Motion { alpha: 10.0, omega: 1.0, safety: 1.0 };
    let r = requirements_axes(&links, &[Axis::Pitch, Axis::Roll], 2.0, m, 0.1)[1];
    let distal = 0.2 + 2.0; // link 2 + payload (its own actuator sits on the axis)
    assert!(close(r.gravity, distal * G * 0.1, 1e-12));
    assert!(close(r.inertia, distal * 0.01, 1e-12));
}

#[test]
fn six_dof_arm_preset_fits_with_its_motors() {
    let (links, axes, payload, acts) = preset("arm6").unwrap();
    let reqs = requirements_axes(&links, &axes, payload, Motion { alpha: 5.0, omega: 90f64.to_radians(), safety: 1.5 }, DEFAULT_ROLL_OFFSET);
    assert_eq!(reqs.len(), 6);
    assert_eq!(reqs[0].gravity, 0.0); // base yaw
    assert!(reqs[1].gravity > reqs[2].gravity && reqs[2].gravity > reqs[3].gravity); // shoulder > elbow > wrist
    for (r, a) in reqs.iter().zip(&acts) {
        assert_ne!(fit(r, &output(a)), Fit::Fail, "{r:?}");
    }
}
