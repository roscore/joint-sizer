# joint-sizer

**로봇 관절 토크·감속비 계산기** — 로봇팔이나 다리의 관절마다 어떤 모터와 감속기가 필요한지 브라우저에서 바로 계산해요.

웹에서 바로 쓰기: https://roscore.github.io/joint-sizer/

## 무엇을 하나요

1. **링크 입력**: 관절 2~7개. 관절마다 회전축(피치·요·롤), 링크 길이·질량, 관절에 달린 액추에이터 질량, 끝단 하중을 넣어요. 예시로 "2관절 책상 로봇팔", "3자유도 다리", "6자유도 로봇팔(UR 방식)"이 들어 있어요.
2. **관절별 요구값**: 팔을 수평으로 쭉 뻗은 최악 자세에서
   - 중력 토크 (N·m 또는 kgf·cm)
   - 관절 뒤쪽 전체의 관성 (평행축 정리)
   - 필요한 연속 토크 = 중력 토크 × 안전율
   - 필요한 최대 토크 = (중력 토크 + 관성 × 최대 각가속도) × 안전율
   - 필요한 출력 속도 (rpm)
3. **모터 적합 판정**: 모터의 연속·최대 토크, 속도, 감속비, 감속기 효율을 넣으면 출력 토크와 속도를 계산해서 ✅ 충분 / ⚠️ 빠듯(여유 10% 미만) / ❌ 부족으로 보여 줘요.
4. **감속비 추천**: 그 모터로 토크와 속도를 모두 만족하는 감속비 범위를 알려 줘요. 범위가 없으면 더 센 모터나 더 빠른 모터가 필요하다는 뜻이에요.

한국어/영어 전환, 휴대폰 화면, 다크 모드를 지원해요.

## 계산 가정 (한계)

- 팔을 수평으로 쭉 뻗은 자세에서 계산해요. 링크는 균일한 막대, 액추에이터와 하중은 점질량으로 봐요.
- 피치 관절(어깨·팔꿈치·손목 굽힘)은 중력 토크와 관성을 모두 받아요. 요 관절(베이스 회전)은 중력 토크 없이 관성만 받아요. 롤 관절(손목 회전)은 그 뒤 질량 전체가 "롤 관절 하중 편심"(기본 5 cm)만큼 떨어져 있다고 보고 근사해요.
- 피치 관절만 있는 팔에서는 이 자세가 모든 관절의 중력 모멘트 팔과 관성이 가장 큰 최악 자세예요. 요·롤이 섞이면 자세에 따라 달라질 수 있어 근사값이에요.
- 마찰, 백래시, 발열, 모터 회전자 관성, 듀티 사이클은 넣지 않았어요.
- **추정값이에요. 실제 로봇에서 꼭 확인하세요.**

## 개인정보

- 모든 계산은 브라우저 안에서만 해요. 서버, 분석 도구, 외부 요청이 없어요 (CSP로 막아 둠).
- 언어 설정만 브라우저의 localStorage에 저장해요.

## 직접 빌드

```sh
rustup target add wasm32-unknown-unknown
cargo test -p js_core
trunk build --release   # dist/ 생성
```

---

# joint-sizer (English)

**Robot joint torque & gear ratio calculator** — find out which motor and gearbox each joint of an arm or leg needs, right in your browser.

Use it: https://roscore.github.io/joint-sizer/

- 2–7 revolute joints, each Pitch (gravity-loaded), Yaw (vertical axis) or Roll (along the arm): link length/mass, actuator mass at each joint, payload at the tip. Presets: 2-link desk arm, 3-DoF leg, 6-DoF UR-style arm.
- Per joint, at the worst case (fully extended, horizontal): gravity torque, distal inertia (parallel axis), continuous torque = gravity × safety factor, peak torque = (gravity + I·α) × safety factor, and required output rpm.
- Motor + gearbox check: output torque = motor torque × ratio × efficiency, output speed = motor rpm / ratio, shown as ✅ OK / ⚠️ Tight (< 10 % headroom) / ❌ Too weak.
- Gear ratio finder: the range of ratios that satisfies both torque and speed for the given motor.
- Korean/English, mobile friendly, dark mode follows the system.

**Assumptions:** chain evaluated fully extended and horizontal; yaw joints get inertia only; roll joints lump the distal mass at a user offset (default 5 cm); rigid uniform links, point-mass actuators and payload, no friction, backlash, heating, rotor inertia or duty cycle. Estimates only — verify on real hardware.

**Privacy:** everything runs client-side. No server, no analytics, no external requests (blocked by CSP). Only the language choice is kept in localStorage.

**Build:** `cargo test -p js_core`, then `trunk build --release`.

## License

MIT — see [LICENSE](LICENSE).
