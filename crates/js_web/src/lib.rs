//! joint-sizer browser UI. Plain web-sys: one delegated input listener recomputes everything.
use js_core::*;
use std::cell::{Cell, RefCell};
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::{CanvasRenderingContext2d, Document, HtmlCanvasElement, HtmlElement, HtmlInputElement, HtmlSelectElement};

thread_local! {
    static KO: Cell<bool> = const { Cell::new(false) };
    static LAST: RefCell<Option<(Chain, Vec<JointLoad>)>> = const { RefCell::new(None) };
}

fn ko() -> bool {
    KO.with(|k| k.get())
}

/// (key, English, Korean)
const TEXT: &[(&str, &str, &str)] = &[
    ("title", "Robot joint torque & gear ratio calculator", "로봇 관절 토크·감속비 계산기"),
    ("subtitle", "3D serial chain up to 7 joints. For every joint it searches all poses inside the joint limits for the worst gravity and peak torque, then checks your motor and gearbox.",
     "최대 7관절 3차원 로봇팔·다리. 관절마다 가동 범위 안의 모든 자세를 찾아 중력 토크와 최대 토크가 가장 큰 자세를 구하고, 모터와 감속기가 충분한지 판정해요."),
    ("sec_chain", "Chain", "링크 구성"),
    ("preset", "Preset", "예시"),
    ("p_custom", "Custom", "직접 입력"),
    ("p_desk_arm", "3-DoF desk arm (yaw + 2 pitch)", "3자유도 책상 로봇팔 (요 + 피치 2)"),
    ("p_leg", "4-DoF leg (hip roll / hip / knee / ankle)", "4자유도 다리 (고관절 롤·고관절·무릎·발목)"),
    ("p_arm6", "6-DoF arm (UR-style)", "6자유도 로봇팔 (UR 방식)"),
    ("p_arm7", "7-DoF arm (Franka-style)", "7자유도 로봇팔 (Franka 방식)"),
    ("joints", "Joints", "관절 수"),
    ("payload", "Payload (kg)", "끝단 하중 (kg)"),
    ("payload_off", "Payload offset along tool Z (m)", "하중 편심 (공구 Z방향, m)"),
    ("sec_motion", "Motion targets", "동작 목표"),
    ("alpha", "Peak angular acceleration (rad/s²)", "최대 각가속도 (rad/s²)"),
    ("omega", "Max joint speed (deg/s)", "최대 관절 속도 (deg/s)"),
    ("safety", "Safety factor", "안전율"),
    ("unit", "Torque unit", "토크 단위"),
    ("sec_links", "Joints, links and actuators", "관절·링크·액추에이터"),
    ("help_frames", "World Z is up. Each joint rotates about X, Y or Z of its own frame; the link then points along the chosen local direction to the next joint. At all-zero angles the frames line up with the world.",
     "월드 Z축이 위쪽이에요. 관절은 자기 좌표계의 X·Y·Z 중 한 축으로 돌고, 링크는 고른 방향(로컬 좌표)으로 다음 관절까지 뻗어요. 모든 각도가 0이면 좌표계가 월드와 같아요."),
    ("h_joint", "Joint", "관절"),
    ("h_axis", "Axis", "회전축"),
    ("h_dir", "Link dir", "링크 방향"),
    ("h_len", "Length (m)", "길이 (m)"),
    ("h_mass", "Link kg", "링크 kg"),
    ("h_act", "Actuator kg", "액추에이터 kg"),
    ("h_min", "Min °", "최소 °"),
    ("h_max", "Max °", "최대 °"),
    ("h_cont", "Motor cont. N·m", "모터 연속 N·m"),
    ("h_peak", "Motor peak N·m", "모터 최대 N·m"),
    ("h_rpm", "Motor rpm", "모터 rpm"),
    ("h_ratio", "Gear ratio", "감속비"),
    ("h_eff", "Efficiency", "효율"),
    ("sec_result", "Result per joint", "관절별 결과"),
    ("r_grav", "Worst gravity", "최대 중력 토크"),
    ("r_inertia", "Inertia (kg·m²)", "관성 (kg·m²)"),
    ("r_need", "Needed cont. / peak", "필요 연속 / 최대"),
    ("r_have", "Output cont. / peak", "출력 연속 / 최대"),
    ("r_speed", "Speed need / have (rpm)", "속도 필요 / 출력 (rpm)"),
    ("r_fit", "Fit", "판정"),
    ("r_range", "Working gear ratios", "가능한 감속비"),
    ("r_pose", "Worst pose", "최악 자세"),
    ("show", "show", "보기"),
    ("fit_ok", "✅ OK", "✅ 충분"),
    ("fit_tight", "⚠️ Tight", "⚠️ 빠듯"),
    ("fit_fail", "❌ Too weak", "❌ 부족"),
    ("no_ratio", "none — pick a stronger or faster motor", "없음 — 더 센 모터나 더 빠른 모터가 필요해요"),
    ("sec_view", "3D view", "3D 보기"),
    ("view_pose", "Pose", "자세"),
    ("v_home", "All joints at 0°", "모든 관절 0°"),
    ("v_worst", "worst peak-torque pose", "최대 토크 최악 자세"),
    ("yaw_view", "Turn view (°)", "시점 회전 (°)"),
    ("note", "Estimates only: rigid thin-rod links, point-mass actuators and payload, no friction, backlash, heating, rotor inertia or velocity terms; the worst-pose search is numerical. Verify on the real robot. Runs entirely in your browser — nothing is sent anywhere.",
     "추정값이에요. 링크는 가는 막대, 액추에이터와 하중은 점질량으로 보고 마찰·백래시·발열·회전자 관성·속도 항은 넣지 않았어요. 최악 자세는 수치 탐색으로 찾아요. 실제 로봇에서 꼭 확인하세요. 모든 계산은 브라우저 안에서만 하고, 아무 데도 보내지 않아요."),
    ("lang", "한국어", "English"),
];

fn t(key: &str) -> &'static str {
    TEXT.iter().find(|(k, _, _)| *k == key).map(|(_, en, kr)| if ko() { *kr } else { *en }).unwrap_or("")
}

fn doc() -> Document {
    web_sys::window().unwrap().document().unwrap()
}

fn input(id: &str) -> Option<HtmlInputElement> {
    doc().get_element_by_id(id)?.dyn_into().ok()
}

fn select(id: &str) -> Option<HtmlSelectElement> {
    doc().get_element_by_id(id)?.dyn_into().ok()
}

fn num(id: &str, default: f64) -> f64 {
    input(id).and_then(|i| i.value().trim().parse::<f64>().ok()).filter(|v| v.is_finite()).unwrap_or(default)
}

fn set_html(id: &str, html: &str) {
    if let Some(e) = doc().get_element_by_id(id) {
        e.set_inner_html(html);
    }
}

fn apply_text() {
    let list = doc().query_selector_all("[data-i18n]").unwrap();
    for i in 0..list.length() {
        if let Some(e) = list.item(i).and_then(|n| n.dyn_into::<HtmlElement>().ok()) {
            if let Some(k) = e.get_attribute("data-i18n") {
                e.set_text_content(Some(t(&k)));
            }
        }
    }
    doc().document_element().unwrap().set_attribute("lang", if ko() { "ko" } else { "en" }).ok();
}

/// Numeric per-joint columns: (id prefix, header key).
const COLS: [(&str, &str); 10] = [
    ("len", "h_len"), ("mass", "h_mass"), ("act", "h_act"), ("min", "h_min"), ("max", "h_max"),
    ("cont", "h_cont"), ("peak", "h_peak"), ("rpm", "h_rpm"), ("ratio", "h_ratio"), ("eff", "h_eff"),
];

fn build_rows(c: &Chain, acts: &[Actuator]) {
    let mut h = String::from("<thead><tr><th data-i18n=\"h_joint\"></th><th data-i18n=\"h_axis\"></th><th data-i18n=\"h_dir\"></th>");
    for (_, key) in COLS.iter().filter(|(c, _)| !c.is_empty()) {
        h.push_str(&format!("<th data-i18n=\"{key}\"></th>"));
    }
    h.push_str("</tr></thead><tbody>");
    for (i, (j, a)) in c.joints.iter().zip(acts).enumerate() {
        h.push_str(&format!("<tr><td>J{}</td><td><select id=\"ax{i}\">", i + 1));
        for ax in [Axis::X, Axis::Y, Axis::Z] {
            let sel = if ax == j.axis { " selected" } else { "" };
            h.push_str(&format!("<option value=\"{0}\"{sel}>{1}</option>", ax.code(), ax.code().to_uppercase()));
        }
        h.push_str(&format!("</select></td><td><select id=\"dir{i}\">"));
        for d in Dir::ALL {
            let sel = if d == j.dir { " selected" } else { "" };
            h.push_str(&format!("<option value=\"{0}\"{sel}>{1}</option>", d.code(), d.code().to_uppercase()));
        }
        h.push_str("</select></td>");
        let vals = [j.length, j.mass, j.actuator_mass, j.min_deg, j.max_deg, a.cont_torque, a.peak_torque, a.rpm, a.ratio, a.efficiency];
        for ((col, _), v) in COLS.iter().zip(vals) {
            let min = if *col == "min" || *col == "max" { "" } else { " min=\"0\"" };
            h.push_str(&format!("<td><input id=\"{col}{i}\" type=\"number\" inputmode=\"decimal\" step=\"any\"{min} value=\"{v}\"></td>"));
        }
        h.push_str("</tr>");
    }
    h.push_str("</tbody>");
    set_html("rows", &h);
    let mut v = String::from("<option value=\"home\" data-i18n=\"v_home\"></option>");
    for i in 0..c.joints.len() {
        v.push_str(&format!("<option value=\"{i}\">J{} — {}</option>", i + 1, t("v_worst")));
    }
    set_html("view_pose", &v);
    apply_text();
}

fn read_rows(n: usize) -> (Chain, Vec<Actuator>) {
    let mut joints = Vec::new();
    let mut acts = Vec::new();
    for i in 0..n {
        let g = |c: &str, d: f64| num(&format!("{c}{i}"), d).max(0.0);
        let axis = select(&format!("ax{i}")).map(|s| Axis::from_code(&s.value())).unwrap_or(Axis::Y);
        let dir = select(&format!("dir{i}")).map(|s| Dir::from_code(&s.value())).unwrap_or(Dir::Px);
        let lo = num(&format!("min{i}"), -180.0).clamp(-360.0, 360.0);
        let hi = num(&format!("max{i}"), 180.0).clamp(-360.0, 360.0);
        joints.push(Joint::new(axis, dir, g("len", 0.3), g("mass", 0.5), g("act", 0.3)).limits(lo, hi));
        acts.push(Actuator {
            cont_torque: g("cont", 1.0),
            peak_torque: g("peak", 3.0),
            rpm: g("rpm", 3000.0),
            ratio: g("ratio", 50.0),
            efficiency: g("eff", DEFAULT_EFFICIENCY).min(1.0),
        });
    }
    let chain = Chain { joints, payload: num("payload", 0.0).max(0.0), payload_offset: num("payload_off", 0.0) };
    (chain, acts)
}

fn joint_count() -> usize {
    select("joints").and_then(|s| s.value().parse().ok()).unwrap_or(3).clamp(1, MAX_JOINTS)
}

fn fmt_t(v: f64, kgf: bool) -> String {
    if kgf { format!("{:.1}", nm_to_kgfcm(v)) } else { format!("{:.2}", v) }
}

fn recompute() {
    let n = joint_count();
    let (chain, acts) = read_rows(n);
    let motion = Motion {
        alpha: num("alpha", 5.0).max(0.0),
        omega: num("omega", 90.0).max(0.0).to_radians(),
        safety: num("safety", 1.5).max(1.0),
    };
    let kgf = select("unit").map(|s| s.value() == "kgfcm").unwrap_or(false);
    let u = if kgf { "kgf·cm" } else { "N·m" };
    let loads = requirements(&chain, motion);

    let mut h = format!(
        "<thead><tr><th data-i18n=\"h_joint\"></th><th><span data-i18n=\"r_grav\"></span> ({u})</th><th data-i18n=\"r_inertia\"></th>\
         <th><span data-i18n=\"r_need\"></span> ({u})</th><th><span data-i18n=\"r_have\"></span> ({u})</th>\
         <th data-i18n=\"r_speed\"></th><th data-i18n=\"r_fit\"></th><th data-i18n=\"r_range\"></th><th data-i18n=\"r_pose\"></th></tr></thead><tbody>"
    );
    for (i, (l, a)) in loads.iter().zip(&acts).enumerate() {
        let r = &l.req;
        let o = output(a);
        let (cls, key) = match fit(r, &o) {
            Fit::Ok => ("ok", "fit_ok"),
            Fit::Tight => ("tight", "fit_tight"),
            Fit::Fail => ("no", "fit_fail"),
        };
        let m = min_margin(r, &o);
        let head = if m.is_finite() { format!(" {:+.0}%", (m - 1.0) * 100.0) } else { String::new() };
        let range = match ratio_range(r, a) {
            Some((lo, hi)) if hi.is_finite() => format!("{:.1} – {:.1}", lo, hi),
            Some((lo, _)) => format!("≥ {:.1}", lo),
            None => t("no_ratio").to_string(),
        };
        let pose_txt = l.peak_pose.iter().map(|d| format!("{:.0}", d)).collect::<Vec<_>>().join(", ");
        h.push_str(&format!(
            "<tr><td>J{}</td><td>{}</td><td>{:.4}</td><td>{} / {}</td><td>{} / {}</td><td>{:.0} / {:.0}</td>\
             <td><span class=\"badge {cls}\" data-i18n=\"{key}\"></span>{head}</td><td>{range}</td>\
             <td><button type=\"button\" class=\"ghost show\" data-j=\"{i}\" title=\"{pose_txt}\" data-i18n=\"show\"></button> <small>[{pose_txt}]°</small></td></tr>",
            i + 1, fmt_t(r.gravity, kgf), r.inertia, fmt_t(r.cont, kgf), fmt_t(r.peak, kgf),
            fmt_t(o.cont, kgf), fmt_t(o.peak, kgf), r.rpm, o.rpm
        ));
    }
    h.push_str("</tbody>");
    set_html("result", &h);
    apply_text();
    LAST.with(|s| *s.borrow_mut() = Some((chain, loads)));
    draw();
}

/// Isometric-ish projection: rotate about world Z by `yaw`, tilt 30°, Z up on screen.
fn project(p: V3, yaw: f64) -> (f64, f64) {
    let (s, c) = yaw.sin_cos();
    let x = c * p[0] - s * p[1];
    let y = s * p[0] + c * p[1];
    let tilt = 30f64.to_radians();
    (x, -(p[2] * tilt.cos() + y * tilt.sin()))
}

fn draw() {
    let Some(c) = doc().get_element_by_id("view").and_then(|e| e.dyn_into::<HtmlCanvasElement>().ok()) else { return };
    let Ok(Some(ctx)) = c.get_context("2d") else { return };
    let ctx: CanvasRenderingContext2d = ctx.dyn_into().unwrap();
    let (w, hh) = (c.width() as f64, c.height() as f64);
    ctx.clear_rect(0.0, 0.0, w, hh);
    LAST.with(|s| {
        let s = s.borrow();
        let Some((chain, loads)) = s.as_ref() else { return };
        let sel = select("view_pose").map(|s| s.value()).unwrap_or_default();
        let (q, hl): (Vec<f64>, Option<usize>) = match sel.parse::<usize>() {
            Ok(j) if j < loads.len() => (loads[j].peak_pose.clone(), Some(j)),
            _ => (vec![0.0; chain.joints.len()], None),
        };
        let yaw = num("yaw_view", 35.0).to_radians();
        let p = pose(chain, &q);
        let mut pts: Vec<(f64, f64)> = p.points.iter().map(|&x| project(x, yaw)).collect();
        let pay = project(p.payload_at, yaw);
        pts.push(pay);
        let reach: f64 = chain.joints.iter().map(|j| j.length).sum::<f64>() + chain.payload_offset.abs();
        let sc = (hh * 0.42) / reach.max(1e-6);
        let (cx, cy) = (w / 2.0, hh / 2.0);
        let to = |(x, y): (f64, f64)| (cx + x * sc, cy + y * sc);
        let dark = web_sys::window().and_then(|w| w.match_media("(prefers-color-scheme: dark)").ok().flatten()).map(|m| m.matches()).unwrap_or(false);
        let (fg, accent, hot, grid) = if dark { ("#e6e9ee", "#60a5fa", "#f87171", "#3a4250") } else { ("#1b1f24", "#2563eb", "#dc2626", "#d6dbe3") };
        // floor grid at z = 0
        ctx.set_line_width(1.0);
        ctx.set_stroke_style_str(grid);
        let g = reach.max(0.1);
        for k in -4..=4 {
            let f = g * k as f64 / 4.0;
            for (a, b) in [([f, -g, 0.0], [f, g, 0.0]), ([-g, f, 0.0], [g, f, 0.0])] {
                let (x1, y1) = to(project(a, yaw));
                let (x2, y2) = to(project(b, yaw));
                ctx.begin_path();
                ctx.move_to(x1, y1);
                ctx.line_to(x2, y2);
                ctx.stroke();
            }
        }
        // world axes
        ctx.set_font("12px system-ui, sans-serif");
        for (v, name, col) in [([g * 0.3, 0.0, 0.0], "X", "#ef4444"), ([0.0, g * 0.3, 0.0], "Y", "#22c55e"), ([0.0, 0.0, g * 0.3], "Z", "#3b82f6")] {
            let (x0, y0) = to(project([0.0; 3], yaw));
            let (x1, y1) = to(project(v, yaw));
            ctx.set_stroke_style_str(col);
            ctx.set_line_width(2.0);
            ctx.begin_path();
            ctx.move_to(x0, y0);
            ctx.line_to(x1, y1);
            ctx.stroke();
            ctx.set_fill_style_str(col);
            ctx.fill_text(name, x1 + 3.0, y1 - 3.0).ok();
        }
        // links
        ctx.set_line_width(6.0);
        ctx.set_line_cap("round");
        for k in 0..chain.joints.len() {
            let (x1, y1) = to(pts[k]);
            let (x2, y2) = to(pts[k + 1]);
            ctx.set_stroke_style_str(if hl.map(|h| k >= h).unwrap_or(false) { hot } else { accent });
            ctx.begin_path();
            ctx.move_to(x1, y1);
            ctx.line_to(x2, y2);
            ctx.stroke();
        }
        // joint axes + labels
        for k in 0..chain.joints.len() {
            let (x, y) = to(pts[k]);
            let a = p.axes[k];
            let e = project([p.points[k][0] + a[0] * g * 0.12, p.points[k][1] + a[1] * g * 0.12, p.points[k][2] + a[2] * g * 0.12], yaw);
            let (ex, ey) = to(e);
            ctx.set_line_width(2.0);
            ctx.set_stroke_style_str(fg);
            ctx.begin_path();
            ctx.move_to(2.0 * x - ex, 2.0 * y - ey);
            ctx.line_to(ex, ey);
            ctx.stroke();
            ctx.set_fill_style_str(if Some(k) == hl { hot } else { fg });
            ctx.begin_path();
            ctx.arc(x, y, 6.0, 0.0, std::f64::consts::TAU).ok();
            ctx.fill();
            ctx.fill_text(&format!("J{}", k + 1), x + 8.0, y - 8.0).ok();
        }
        if chain.payload > 0.0 {
            let (x, y) = to(pay);
            ctx.set_fill_style_str(fg);
            ctx.fill_rect(x - 6.0, y - 6.0, 12.0, 12.0);
            ctx.fill_text(&format!("{} kg", chain.payload), x + 9.0, y + 14.0).ok();
        }
        ctx.set_fill_style_str(fg);
        ctx.fill_text("g ↓", w - 34.0, 18.0).ok();
    });
}

fn load_preset(name: &str) {
    if let Some((c, acts)) = preset(name) {
        if let Some(s) = select("joints") {
            s.set_value(&c.joints.len().to_string());
        }
        if let Some(p) = input("payload") {
            p.set_value(&c.payload.to_string());
        }
        if let Some(p) = input("payload_off") {
            p.set_value(&c.payload_offset.to_string());
        }
        build_rows(&c, &acts);
    }
}

/// Keep existing row values when the joint count changes; new joints copy the last one.
fn resize_rows() {
    let n = joint_count();
    let have = (0..MAX_JOINTS).take_while(|i| input(&format!("len{i}")).is_some()).count();
    let (mut c, mut acts) = read_rows(have.max(1));
    while c.joints.len() < n {
        c.joints.push(*c.joints.last().unwrap());
        acts.push(*acts.last().unwrap());
    }
    c.joints.truncate(n);
    acts.truncate(n);
    build_rows(&c, &acts);
}

fn on(id: &str, ev: &str, f: impl FnMut(web_sys::Event) + 'static) {
    if let Some(e) = doc().get_element_by_id(id) {
        let cb = Closure::<dyn FnMut(web_sys::Event)>::new(f);
        e.add_event_listener_with_callback(ev, cb.as_ref().unchecked_ref()).ok();
        cb.forget();
    }
}

fn storage() -> Option<web_sys::Storage> {
    web_sys::window()?.local_storage().ok().flatten()
}

fn set_custom() {
    if let Some(s) = select("preset") {
        s.set_value("custom");
    }
}

#[wasm_bindgen(start)]
pub fn start() {
    let saved = storage().and_then(|s| s.get_item("joint-sizer-lang").ok().flatten());
    let browser_ko = web_sys::window().and_then(|w| w.navigator().language()).map(|l| l.to_lowercase().starts_with("ko")).unwrap_or(false);
    KO.with(|k| k.set(saved.map(|s| s == "ko").unwrap_or(browser_ko)));

    load_preset("arm6");
    if let Some(s) = select("preset") {
        s.set_value("arm6");
    }
    apply_text();
    recompute();

    on("preset", "change", |_| {
        if let Some(s) = select("preset") {
            load_preset(&s.value());
        }
        recompute();
    });
    on("joints", "change", |_| {
        set_custom();
        resize_rows();
        recompute();
    });
    on("lang", "click", |_| {
        let now = !ko();
        KO.with(|k| k.set(now));
        if let Some(s) = storage() {
            s.set_item("joint-sizer-lang", if now { "ko" } else { "en" }).ok();
        }
        let v = select("view_pose").map(|s| s.value());
        resize_rows();
        if let (Some(s), Some(v)) = (select("view_pose"), v) {
            s.set_value(&v);
        }
        recompute();
    });
    on("view_pose", "change", |_| draw());
    on("yaw_view", "input", |_| draw());
    on("result", "click", |e| {
        let j = e.target().and_then(|t| t.dyn_into::<web_sys::Element>().ok()).and_then(|el| el.get_attribute("data-j"));
        if let (Some(j), Some(s)) = (j, select("view_pose")) {
            s.set_value(&j);
            draw();
            if let Some(v) = doc().get_element_by_id("view") {
                v.scroll_into_view();
            }
        }
    });
    on("app", "input", |e| {
        let id = e.target().and_then(|t| t.dyn_into::<web_sys::Element>().ok()).map(|el| el.id()).unwrap_or_default();
        if ["preset", "joints", "view_pose", "yaw_view"].contains(&id.as_str()) {
            return;
        }
        let per_joint = COLS.iter().any(|(c, _)| !c.is_empty() && id.starts_with(c)) || id.starts_with("ax") || id.starts_with("dir");
        if per_joint || id.starts_with("payload") {
            set_custom();
        }
        recompute();
    });
}
