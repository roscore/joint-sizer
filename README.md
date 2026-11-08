# joint-sizer

**로봇 관절 토크·감속비 계산기** — 로봇팔이나 다리의 관절마다 어떤 모터와 감속기가 필요한지 브라우저에서 바로 계산해요.

웹에서 바로 쓰기: https://roscore.github.io/joint-sizer/

## 무엇을 하나요

1. **3차원 로봇 입력**: 관절 1~7개. 관절마다 회전축(자기 좌표계의 X·Y·Z), 링크가 뻗는 방향(±X·±Y·±Z), 링크 길이·질량, 액추에이터 질량, 가동 범위(최소·최대 각도)를 넣어요. 끝단 하중과 공구 방향 편심도 넣을 수 있어요. 월드 Z축이 위쪽이에요.
2. **예시**: 3자유도 책상 로봇팔, 4자유도 다리, 6자유도 로봇팔(UR 방식), 7자유도 로봇팔(Franka 방식).
3. **최악 자세 탐색**: 관절마다 가동 범위 안의 자세를 수치 탐색해서
   - 중력 토크가 가장 큰 자세 → 필요한 연속 토크 = 최대 중력 토크 × 안전율
   - (중력 토크 + 관성 × 최대 각가속도)가 가장 큰 자세 → 필요한 최대 토크
   - 관성은 관절 축에 대한 값이에요 (가는 막대 + 점질량, 평행축 정리).
4. **모터 적합 판정**: 모터의 연속·최대 토크, 속도, 감속비, 감속기 효율을 넣으면 ✅ 충분 / ⚠️ 빠듯(여유 10% 미만) / ❌ 부족으로 보여 줘요.
5. **감속비 추천**: 그 모터로 토크와 속도를 모두 만족하는 감속비 범위를 알려 줘요.
6. **3D 보기**: 기본 자세나 관절별 최악 자세를 3D로 그려요. 시점을 돌려 볼 수 있어요.

한국어/영어 전환, 휴대폰 화면, 다크 모드를 지원해요.

## 계산 가정 (한계)

- 링크는 가는 균일 막대, 액추에이터와 하중은 점질량으로 봐요.
- 정역학 중력 토크 + 관절 축 관성 × 각가속도만 계산해요. 여러 관절이 함께 움직일 때 생기는 원심력·코리올리 힘, 마찰, 백래시, 발열, 모터 회전자 관성은 넣지 않았어요.
- 최악 자세는 수치 탐색(격자 + 여러 시작점 + 국소 개선)으로 찾아요. 아주 특이한 구조에서는 실제 최악값보다 조금 작게 나올 수 있어요.
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

- 3D serial chain, 1–7 revolute joints. Each joint rotates about X, Y or Z of its own frame; its link points along ±X/±Y/±Z of that frame. Link length/mass, actuator mass, joint limits, tip payload with a tool-Z offset. World Z is up.
- Presets: 3-DoF desk arm, 4-DoF leg, 6-DoF UR-style arm, 7-DoF Franka-style arm.
- Worst-pose search per joint inside the joint limits: max gravity torque (→ continuous requirement) and max gravity + I·α (→ peak requirement), inertia taken about the joint axis.
- Motor + gearbox check (✅ / ⚠️ < 10 % headroom / ❌) and the gear-ratio range that satisfies torque and speed.
- 3D view of the home pose or each joint's worst pose, rotatable.
- Korean/English, mobile friendly, dark mode follows the system.

**Assumptions:** thin uniform rods, point-mass actuators and payload; static gravity + axis inertia × α only (no Coriolis/centrifugal coupling, friction, backlash, heating, rotor inertia); the worst pose is found numerically. Estimates only — verify on real hardware.

**Privacy:** everything runs client-side. No server, no analytics, no external requests (blocked by CSP). Only the language choice is kept in localStorage.

**Build:** `cargo test -p js_core`, then `trunk build --release`.

## License

MIT — see [LICENSE](LICENSE).
