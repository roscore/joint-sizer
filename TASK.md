# Task: Robot joint actuator sizing calculator (Rust → WebAssembly, static web app)

Original open-source tool. Write all code yourself; do not copy anyone's code, tables or assets.
Use textbook rigid-body statics/dynamics only. No vendor motor databases — users type their own specs.

## Product
A single-page web app (GitHub Pages, sub-path `/joint-sizer/`) for hobby/research roboticists who ask
"which motor + gearbox does this arm / leg joint need?".

1. **Planar serial chain model** (2–6 revolute joints, vertical plane, gravity -Y):
   per link: length (m), link mass (kg, uniform rod by default or custom COM offset), joint/actuator mass
   placed at the joint (kg). Payload mass at the end effector (kg). Joint angles (deg) per joint with
   sliders, plus a "worst case" button (all links horizontal = max gravity moment).
   Presets: "2-link desk arm", "3-DoF humanoid leg (hip/knee/ankle)", "6-joint arm (planar projection)".
   Draw the chain to scale on a canvas (links, joints, COM dots, payload), draggable angle handles optional.
2. **Per-joint load**: static gravity torque (N·m) at the current pose and the worst case; inertia about the
   joint of everything distal (kg·m², parallel axis, pose-dependent, report worst = fully extended);
   dynamic torque = τ_gravity + I·α for a user-given peak angular acceleration (rad/s² or "reach speed ω in
   t seconds" helper); required output speed (rpm) from a given max ω. Safety factor input (default 1.5).
3. **Actuator check** per joint: user enters motor rated (continuous) torque, peak torque, no-load or rated
   speed (rpm), gear ratio, gearbox efficiency (default 0.85), optional motor rotor inertia (reflected
   inertia = J_rotor·N²). Output: available output continuous/peak torque, output speed, fits? (✅/⚠️/❌),
   headroom %, torque-margin bar chart, mechanical power at output (W). "Suggest ratio" finder: range of gear
   ratios that satisfy both torque (peak with SF) and speed requirements; show "no ratio works" if empty.
   Optional duty cycle (fraction of time at peak) → RMS torque vs continuous rating.
4. Korean + English UI toggle (default follows browser language). Mobile friendly. Dark/light follows
   system. Units: SI, with optional kgf·cm display for torque (common in hobby servo specs).
   State shareable via URL fragment (#s=...), never sent to a server. Copy-result button (plain text
   summary). JSON export/import of the whole model (download/upload file locally).

## Core crate (pure Rust, no web deps, unit tested) — `crates/js_core`
- Forward kinematics, COM positions, gravity torque per joint, distal inertia (parallel axis),
  dynamic torque, reflected inertia, actuator fit check, ratio range solver, RMS duty torque, unit conversion.
- Tests (all hand-computable): single 1 m link, 2 kg uniform rod, horizontal → 9.80665 N·m (±0.1%);
  same link vertical → ~0; 2-link horizontal with payload 1 kg at 0.5+0.5 m: base torque = sum of m·g·x;
  elbow torque only counts distal masses; inertia of uniform rod about end = m·L²/3; point mass m·r²;
  worst case ≥ any sampled pose (property-style test over a grid); gear ratio scales torque by N·η and speed
  by 1/N; reflected inertia = J·N²; ratio solver returns a range consistent with the fit check
  (endpoints pass, outside fails) and empty when impossible; 1 N·m = 10.197 kgf·cm.
- `crates/js_web`: UI. Prefer plain web-sys — keep the bundle small.

## Repo hygiene
- README (Korean first, then English): what it is, the math and assumptions (planar, rigid, quasi-static
  + simple dynamic term, no friction/backlash/thermal model beyond RMS), how to use, local build
  (`cargo test -p js_core`, `trunk build --release`), MIT license, "estimates only — verify with real tests".
- LICENSE: MIT, copyright "joint-sizer contributors". `.gitignore` for target/ dist/.
- `index.html` with a CSP meta that blocks external requests:
  `default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; object-src 'none'; base-uri 'self'`.
  In `Trunk.toml` set `filehash = false` and
  `pattern_script = "<script type=\"module\" src=\"{base}boot.js\"></script>"` with an external
  `web/boot.js` (copied via `<link data-trunk rel="copy-file">`) that imports `./js_web.js` and calls
  init, so no inline script is needed.
- `Trunk.toml`: `public_url = "/joint-sizer/"`.
- `.github/workflows/pages.yml`: on push to main + workflow_dispatch; checkout, rust stable + wasm32
  target, install trunk 0.22.1 binary release, `sh tools/privacy_check.sh`,
  `cargo test -p js_core --release`, `trunk build --release`, configure-pages@v5,
  upload-pages-artifact@v3 (dist), deploy job with actions/deploy-pages@v4
  (permissions pages: write, id-token: write).
- `tools/privacy_check.sh` already exists; keep it and keep it clean.

## Done when
- `cargo test -p js_core` green; `trunk build --release` succeeds; `sh tools/privacy_check.sh` clean.
- Commit everything with clear commits. Do NOT create new GitHub repos or change repo settings.
