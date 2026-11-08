# Task: Robot joint actuator sizing calculator (Rust → WebAssembly, static web app)

Original open-source tool. Write all code yourself; do not copy anyone's code, tables or assets.
Use textbook rigid-body statics/dynamics only. No vendor motor databases — users type their own specs.

## Product (SMALL scope — keep the whole tool ~400-800 lines of Rust)
A single-page web app (GitHub Pages, sub-path `/joint-sizer/`): "which motor + gearbox does this joint need?"

1. **Planar chain, 2–4 revolute joints** (vertical plane, gravity -Y): per link length (m), link mass (kg,
   uniform rod), actuator mass at the joint (kg); payload at the tip (kg). Worst case = all links horizontal
   (no pose sliders). Two presets: "2-link desk arm", "3-DoF leg". Simple canvas drawing of the extended chain.
2. **Per-joint requirement**: worst-case gravity torque, distal inertia (parallel axis), dynamic torque
   τ = τ_g + I·α for a user peak α (rad/s²), required output rpm from max ω, safety factor (default 1.5).
3. **Actuator check**: user enters motor continuous + peak torque, speed (rpm), gear ratio, efficiency
   (default 0.85). Output per joint: ✅/⚠️/❌ with headroom %, plus a "suggest ratio" range (torque & speed).
4. Korean + English toggle (browser default). Mobile friendly. Optional kgf·cm display.
Out of scope (do NOT build): pose sliders, duty-cycle RMS, reflected rotor inertia, URL share, JSON
import/export, copy button, dark-mode logic beyond CSS prefers-color-scheme.

## Core crate (pure Rust, no web deps, unit tested) — `crates/js_core`
- Forward kinematics, COM positions, gravity torque per joint, distal inertia (parallel axis),
  dynamic torque, actuator fit check, ratio range solver, unit conversion.
- Tests (all hand-computable): single 1 m link, 2 kg uniform rod, horizontal → 9.80665 N·m (±0.1%);
  same link vertical → ~0; 2-link horizontal with payload 1 kg at 0.5+0.5 m: base torque = sum of m·g·x;
  elbow torque only counts distal masses; inertia of uniform rod about end = m·L²/3; point mass m·r²;
  gear ratio scales torque by N·η and speed
  by 1/N; ratio solver returns a range consistent with the fit check
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
