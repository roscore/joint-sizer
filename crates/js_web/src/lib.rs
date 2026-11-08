//! joint-sizer browser UI. Plain web-sys: one delegated input listener recomputes everything.
use js_core::*;
use std::cell::Cell;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::{CanvasRenderingContext2d, Document, HtmlCanvasElement, HtmlElement, HtmlInputElement, HtmlSelectElement};

thread_local! {
    static KO: Cell<bool> = const { Cell::new(false) };
}

fn ko() -> bool {
    KO.with(|k| k.get())
}

/// (key, English, Korean)
const TEXT: &[(&str, &str, &str)] = &[
    ("title", "Robot joint torque & gear ratio calculator", "로봇 관절 토크·감속비 계산기"),
    ("subtitle", "Which motor and gearbox does each joint need? Planar chain, worst case = fully extended and horizontal.",
     "관절마다 어떤 모터와 감속기가 필요할까요? 평면 링크 기준, 최악 조건은 팔을 수평으로 쭉 뻗은 자세예요."),
    ("sec_chain", "Chain", "링크 구성"),
    ("preset", "Preset", "예시"),
    ("p_custom", "Custom", "직접 입력"),
    ("p_desk_arm", "2-link desk arm", "2관절 책상 로봇팔"),
    ("p_leg", "3-DoF leg (hip / knee / ankle)", "3자유도 다리 (고관절·무릎·발목)"),
    ("joints", "Joints", "관절 수"),
    ("payload", "Payload at tip (kg)", "끝단 하중 (kg)"),
    ("sec_motion", "Motion targets", "동작 목표"),
    ("alpha", "Peak angular acceleration (rad/s²)", "최대 각가속도 (rad/s²)"),
    ("omega", "Max joint speed (deg/s)", "최대 관절 속도 (deg/s)"),
    ("safety", "Safety factor", "안전율"),
    ("unit", "Torque unit", "토크 단위"),
    ("sec_links", "Links and actuators", "링크와 액추에이터"),
    ("h_joint", "Joint", "관절"),
    ("h_len", "Link length (m)", "링크 길이 (m)"),
    ("h_mass", "Link mass (kg)", "링크 질량 (kg)"),
    ("h_act", "Actuator mass (kg)", "액추에이터 질량 (kg)"),
    ("h_cont", "Motor cont. torque (N·m)", "모터 연속 토크 (N·m)"),
    ("h_peak", "Motor peak torque (N·m)", "모터 최대 토크 (N·m)"),
    ("h_rpm", "Motor speed (rpm)", "모터 속도 (rpm)"),
    ("h_ratio", "Gear ratio", "감속비"),
    ("h_eff", "Gear efficiency", "감속기 효율"),
    ("sec_result", "Result per joint", "관절별 결과"),
    ("r_grav", "Gravity torque", "중력 토크"),
    ("r_inertia", "Inertia (kg·m²)", "관성 (kg·m²)"),
    ("r_need", "Needed cont. / peak", "필요 연속 / 최대"),
    ("r_have", "Output cont. / peak", "출력 연속 / 최대"),
    ("r_speed", "Speed need / have (rpm)", "속도 필요 / 출력 (rpm)"),
    ("r_fit", "Fit", "판정"),
    ("r_range", "Working gear ratios", "가능한 감속비"),
    ("fit_ok", "✅ OK", "✅ 충분"),
    ("fit_tight", "⚠️ Tight", "⚠️ 빠듯"),
    ("fit_fail", "❌ Too weak", "❌ 부족"),
    ("no_ratio", "none — pick a stronger or faster motor", "없음 — 더 센 모터나 더 빠른 모터가 필요해요"),
    ("sec_view", "Chain (fully extended)", "링크 모양 (쭉 뻗은 자세)"),
    ("note", "Estimates only: rigid links, no friction, backlash or heating. Verify on the real robot. Runs entirely in your browser — nothing is sent anywhere.",
     "추정값이에요. 강체 링크로 보고 마찰·백래시·발열은 넣지 않았어요. 실제 로봇에서 꼭 확인하세요. 모든 계산은 브라우저 안에서만 하고, 아무 데도 보내지 않아요."),
    ("lang", "한국어", "English"),
];

fn t(key: &str) -> &'static str {
    TEXT.iter()
        .find(|(k, _, _)| *k == key)
        .map(|(_, en, kr)| if ko() { *kr } else { *en })
        .unwrap_or("")
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

const COLS: [(&str, &str); 8] = [
    ("len", "h_len"), ("mass", "h_mass"), ("act", "h_act"), ("cont", "h_cont"),
    ("peak", "h_peak"), ("rpm", "h_rpm"), ("ratio", "h_ratio"), ("eff", "h_eff"),
];

/// Rebuild the per-joint input table with the given values.
fn build_rows(links: &[Link], acts: &[Actuator]) {
    let mut h = String::from("<thead><tr><th data-i18n=\"h_joint\"></th>");
    for (_, key) in COLS {
        h.push_str(&format!("<th data-i18n=\"{key}\"></th>"));
    }
    h.push_str("</tr></thead><tbody>");
    for (i, (l, a)) in links.iter().zip(acts).enumerate() {
        let vals = [l.length, l.mass, l.actuator_mass, a.cont_torque, a.peak_torque, a.rpm, a.ratio, a.efficiency];
        h.push_str(&format!("<tr><td>J{}</td>", i + 1));
        for ((col, _), v) in COLS.iter().zip(vals) {
            h.push_str(&format!(
                "<td><input id=\"{col}{i}\" type=\"number\" inputmode=\"decimal\" step=\"any\" min=\"0\" value=\"{v}\"></td>"
            ));
        }
        h.push_str("</tr>");
    }
    h.push_str("</tbody>");
    set_html("rows", &h);
    apply_text();
}

fn read_rows(n: usize) -> (Vec<Link>, Vec<Actuator>) {
    let mut links = Vec::new();
    let mut acts = Vec::new();
    for i in 0..n {
        let g = |c: &str, d: f64| num(&format!("{c}{i}"), d).max(0.0);
        links.push(Link { length: g("len", 0.3), mass: g("mass", 0.5), actuator_mass: g("act", 0.3) });
        acts.push(Actuator {
            cont_torque: g("cont", 1.0),
            peak_torque: g("peak", 3.0),
            rpm: g("rpm", 3000.0),
            ratio: g("ratio", 50.0),
            efficiency: g("eff", DEFAULT_EFFICIENCY).min(1.0),
        });
    }
    (links, acts)
}

fn joint_count() -> usize {
    select("joints").and_then(|s| s.value().parse().ok()).unwrap_or(2).clamp(2, 4)
}

fn fmt_t(v: f64, kgf: bool) -> String {
    if kgf { format!("{:.1}", nm_to_kgfcm(v)) } else { format!("{:.2}", v) }
}

fn recompute() {
    let n = joint_count();
    let (links, acts) = read_rows(n);
    let payload = num("payload", 0.0).max(0.0);
    let motion = Motion {
        alpha: num("alpha", 5.0).max(0.0),
        omega: num("omega", 90.0).max(0.0).to_radians(),
        safety: num("safety", 1.5).max(1.0),
    };
    let kgf = select("unit").map(|s| s.value() == "kgfcm").unwrap_or(false);
    let u = if kgf { "kgf·cm" } else { "N·m" };
    let reqs = requirements(&links, payload, motion);

    let mut h = format!(
        "<thead><tr><th data-i18n=\"h_joint\"></th><th><span data-i18n=\"r_grav\"></span> ({u})</th><th data-i18n=\"r_inertia\"></th>\
         <th><span data-i18n=\"r_need\"></span> ({u})</th><th><span data-i18n=\"r_have\"></span> ({u})</th>\
         <th data-i18n=\"r_speed\"></th><th data-i18n=\"r_fit\"></th><th data-i18n=\"r_range\"></th></tr></thead><tbody>"
    );
    for (i, (r, a)) in reqs.iter().zip(&acts).enumerate() {
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
        h.push_str(&format!(
            "<tr><td>J{}</td><td>{}</td><td>{:.4}</td><td>{} / {}</td><td>{} / {}</td><td>{:.0} / {:.0}</td>\
             <td><span class=\"badge {cls}\" data-i18n=\"{key}\"></span>{head}</td><td>{range}</td></tr>",
            i + 1, fmt_t(r.gravity, kgf), r.inertia, fmt_t(r.cont, kgf), fmt_t(r.peak, kgf),
            fmt_t(o.cont, kgf), fmt_t(o.peak, kgf), r.rpm, o.rpm
        ));
    }
    h.push_str("</tbody>");
    set_html("result", &h);
    apply_text();
    draw(&links, payload);
}

fn draw(links: &[Link], payload: f64) {
    let Some(c) = doc().get_element_by_id("view").and_then(|e| e.dyn_into::<HtmlCanvasElement>().ok()) else { return };
    let Ok(Some(ctx)) = c.get_context("2d") else { return };
    let ctx: CanvasRenderingContext2d = ctx.dyn_into().unwrap();
    let (w, hh) = (c.width() as f64, c.height() as f64);
    ctx.clear_rect(0.0, 0.0, w, hh);
    let pts = joint_positions(links, &vec![0.0; links.len()]);
    let total: f64 = links.iter().map(|l| l.length).sum::<f64>().max(1e-6);
    let s = (w - 80.0) / total;
    let (ox, oy) = (40.0, hh / 2.0);
    let dark = web_sys::window().and_then(|w| w.match_media("(prefers-color-scheme: dark)").ok().flatten()).map(|m| m.matches()).unwrap_or(false);
    let (fg, accent) = if dark { ("#e6e9ee", "#60a5fa") } else { ("#1b1f24", "#2563eb") };
    // base
    ctx.set_fill_style_str(fg);
    ctx.fill_rect(ox - 14.0, oy - 18.0, 8.0, 36.0);
    ctx.set_line_width(6.0);
    ctx.set_stroke_style_str(accent);
    for k in 0..links.len() {
        ctx.begin_path();
        ctx.move_to(ox + pts[k].0 * s, oy);
        ctx.line_to(ox + pts[k + 1].0 * s, oy);
        ctx.stroke();
    }
    ctx.set_fill_style_str(fg);
    ctx.set_font("12px system-ui, sans-serif");
    for k in 0..links.len() {
        let x = ox + pts[k].0 * s;
        ctx.begin_path();
        ctx.arc(x, oy, 8.0, 0.0, std::f64::consts::TAU).ok();
        ctx.fill();
        ctx.fill_text(&format!("J{}", k + 1), x - 7.0, oy - 14.0).ok();
    }
    let tip = ox + pts[links.len()].0 * s;
    if payload > 0.0 {
        let side = 10.0 + payload.sqrt() * 6.0;
        ctx.fill_rect(tip - side / 2.0, oy, side.min(40.0), side.min(40.0));
        ctx.fill_text(&format!("{payload} kg"), tip - 14.0, oy + side.min(40.0) + 14.0).ok();
    }
    // gravity arrow
    ctx.fill_text("g ↓", w - 34.0, 18.0).ok();
}

fn load_preset(name: &str) {
    if let Some((links, payload, acts)) = preset(name) {
        if let Some(s) = select("joints") {
            s.set_value(&links.len().to_string());
        }
        if let Some(p) = input("payload") {
            p.set_value(&payload.to_string());
        }
        build_rows(&links, &acts);
    }
}

/// Keep existing row values when the joint count changes; new joints copy the last one.
fn resize_rows() {
    let n = joint_count();
    let have = (0..4).take_while(|i| input(&format!("len{i}")).is_some()).count();
    let (mut links, mut acts) = read_rows(have.max(1));
    while links.len() < n {
        links.push(*links.last().unwrap());
        acts.push(*acts.last().unwrap());
    }
    links.truncate(n);
    acts.truncate(n);
    build_rows(&links, &acts);
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

#[wasm_bindgen(start)]
pub fn start() {
    let saved = storage().and_then(|s| s.get_item("joint-sizer-lang").ok().flatten());
    let browser_ko = web_sys::window()
        .and_then(|w| w.navigator().language())
        .map(|l| l.to_lowercase().starts_with("ko"))
        .unwrap_or(false);
    KO.with(|k| k.set(saved.map(|s| s == "ko").unwrap_or(browser_ko)));

    load_preset("desk_arm");
    if let Some(s) = select("preset") {
        s.set_value("desk_arm");
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
        if let Some(s) = select("preset") {
            s.set_value("custom");
        }
        resize_rows();
        recompute();
    });
    on("lang", "click", |_| {
        let now = !ko();
        KO.with(|k| k.set(now));
        if let Some(s) = storage() {
            s.set_item("joint-sizer-lang", if now { "ko" } else { "en" }).ok();
        }
        apply_text();
        recompute();
    });
    on("app", "input", |e| {
        let id = e.target().and_then(|t| t.dyn_into::<web_sys::Element>().ok()).map(|el| el.id()).unwrap_or_default();
        if id != "preset" && id != "joints" {
            if COLS.iter().any(|(c, _)| id.starts_with(c)) || id == "payload" {
                if let Some(s) = select("preset") {
                    s.set_value("custom");
                }
            }
            recompute();
        }
    });
}
