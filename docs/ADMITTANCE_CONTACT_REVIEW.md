# 스케치 & 어드미턴스 접촉 제어 시스템 평가

작성일: 2026-08-13 (2026-08-14 실기 커미셔닝으로 0장·0.5장 추가)
대상: `admittance_controller` + `rbpodo_painting_control` + `moveit_executor` 접촉 경로
근거: 코드/설정 정적 분석 + **2026-08-14 실기 접촉 시험 (0장, 0.5장)**

> **현재 상태 (2026-08-14 05:30):** 목표력 3 N에서 **20/20 스텝 전체 완주 달성**
> (3개 행 도장, 에러 0건, 실측 힘 2.83~3.13 N).  전체 경위와 최종 설정은
> **0.5장**에 있다.  0장은 그 시작점이 된 F/T 프레임 반전 문제다.

---

## 0. 실기 시험으로 확정된 진짜 근본 원인 (2026-08-14)

**F/T 센서 프레임이 URDF와 물리 센서 간에 180° 뒤집혀 있었다.**
아래 1~7장의 정적 분석(강성/가드/오버슈트)은 여전히 유효한 개선이지만,
"접촉하면 문제 생긴다"의 직접 원인은 그보다 앞단인 **측정 부호**였다.

### 증상 재현 (t는 모니터 기준)

```
t=690.3  CONTACT_SEARCH 진입 (힘 OFF, 위치 탐색)
t=695.5  롤러-벽 접촉 시작
t=696.0  filtered fy_tcp = -3.19 N → CONTACT_DIRECTION_MISMATCH latch → 전체 abort
```

기존 임계(`contact_opposite_force_n: 1.5`)에서는 접촉 순간 -1.5 N에서 즉시 latch.
**"접촉하고 아래로 안 내려가"의 정체가 이것이었다** (RAMP_UP 후퇴 가설은 기각).

### 증거 체인

| # | 관측 | 결론 |
|---|---|---|
| 1 | TCP+Y ↔ 벽 법선 각도 1.9° (TF 실측) | 공구 자세는 정확 → 자세 문제 아님 |
| 2 | 벽에 눌린 정상 상태에서 fy_tcp = **-5 N 지속** (물리적으로 +5 N여야 함) | 측정 체인 부호 반전 |
| 3 | 접촉 중 τx_ft = -0.11, d·fy_meas = -0.29 (d=0.1153 m, **부호 일치**) | 전축(작용/반작용) 반전이면 부호가 반대여야 함 → **프레임 회전 오류** (in-plane 축 하나 + Z만 반전) |
| 4 | 방향 라벨된 횡방향 손 푸시("우")가 Y-정상 해석과만 부합 | X축 아닌 **Y축 기준 180°** |

### 수정 (`rbpodo_description/robots/rb_6dof.xacro`)

```
ft_sensor_joint: rpy (0, -π/2, π/2) → (0, +π/2, π/2)      # ft 프레임에 Ry(π)
tcp_joint:       xyz (z,-x,-y) → (-z,-x,+y), rpy (-π/2,0,-π/2) → (+π/2,0,-π/2)
```

**link6→tcp 누적 변환은 수치 검증으로 비트 동일** (회전=항등, 병진 동일).
→ TCP 기구학, MoveIt, D405 핸드아이 캘리브레이션, 메시 배치 전부 무영향.
바뀌는 것은 ft_link 프레임의 방향 해석뿐이며, 모든 소비자(어드미턴스 컨트롤러,
force monitor, wrench reference)가 URDF/TF를 통해 자동으로 올바른 부호를 받는다.

이 수정이 없으면 모니터 임계만 뒤집어도 **어드미턴스가 양성 피드백**이 되어
컴플라이언스 켜는 순간 벽으로 파고드는 방향으로 폭주한다. 반드시 소스(URDF) 수정이어야 함.

### 부수 확인 (같은 세션)

- `damping_absolute` 파라미터: 실기 컨트롤러에서 정상 로드·활성화 ✅
- 모니터 bias 인터록: 벽 접촉 -5 N을 8분간 bias로 흡수하지 않고 유지 ✅
- 신규 진단 토픽(`normal_trim_m` 등) 정상 발행 ✅
- guard/tare/pre-contact 시퀀스: 설계 의도대로 동작 ✅

### 검증 결과 (2026-08-14 심야 런)

**프레임 수정 검증 완료 — 최초로 페인트 스트로크 완주.**

```
t=443.7  CONTACT_SEARCH 시작 (bias 게이트 2.5 N 수정 후 tare/bias 정상 통과)
t=449.5  접촉 검출: fy 양수 ✅ → RAMP_UP 자동 진행 (direction mismatch 없음)
t=451.7  PAINT 진입 — 진입 후 ~2.5 s 채터 (fy 0↔5.5 N, 접촉모드 ζ≈0.44 언더댐핑)
t=455.0  정착: fy = 1.5~1.8 N (목표 1.6 N, ±0.15 N) — 힘 제어 검증 ✅
t=457.5  RAMP_DOWN (지령 추종 클린) → RETRACT → TRAVEL (7번 스텝, 1행 완주)
t=458.8~ RETRACT/TRAVEL 중 fy ≈ 0.8~1.4 N 지속 — 접촉 아님 (운영자 육안 확인 +
         10 mm 후퇴 중에도 힘이 안 빠짐). 자세 변화에 따른 eft 잔차(유령 힘):
         관절토크 추정 렌치의 tare는 tare 자세 근처에서만 유효
t=462.8  TRAVEL 가감속 동역학 과도 → travel 모드 raw_debounce 0.0 단일샘플
         RAW_IMPACT → abort (벽 긁힘 아님)
```

trim 사용 최대 2 mm / 예산 20 mm. `rate_saturated` 채터 중 일시 True (진단 정상 동작).

**후속 조치 (적용됨):** 나머지 6개 모드(travel/retract/idle/approach_precontact/
final_retract/abort)의 `raw_debounce_s: 0.0 → 0.02`.

**후속 튜닝 2차 (2026-08-14 적용됨):**

eft 유령 힘이 목표력과 같은 자릿수라는 실측(자세 변화만으로 ~1 N, 유휴 40분
드리프트 >3 N)에 따라 목표력을 상향. 진입 바운스(ζ0.44에서 피크 3.4×)가 3 N
목표에서는 8 N 봉투를 치므로 감쇠 보강이 전제 조건으로 함께 들어감.

| 항목 | 변경 | 근거 |
|---|---|---|
| `default_paint_force_n` / `desired_contact_force_n` | 1.6 → **3.0** | 신호가 유령/드리프트를 압도 |
| `contact_detect_threshold_n` / release | 1.0/0.6 → **1.5/0.8** | 검출을 유령 바닥(~1 N) 위로 |
| `damping_absolute[1]` (+ratio 백업 동기) | 120 → **160** | 접촉 ζ 0.44→0.59, 진입 바운스 억제 |

D=160 검증: ζ(Ke=1600)=0.79, ζ(3000)=0.59; 자유공간 3 N 오차에서 19 mm/s;
안정성 한계 Ke < 8,373 N/m (기존 5,480에서 개선). 램프 1.5 N/s ≤ 슬루 5 N/s.

**운영 수칙:** T3 단독 재시작 금지 — tare된 FT + 누적 드리프트를 bias 0으로
읽어 IDLE 봉투(3 N)에 걸림 (실측). T3는 반드시 T1과 함께 재기동.
`default_paint_force_n`은 sketch_to_waypoints 기동 시 캐시 → 변경 시 T2 재기동 필요.

---

## 0.5 커미셔닝 완료 — 20/20 완주까지 (2026-08-14 심야)

0장의 프레임 수정 이후 밤새 이어진 실기 세션 기록.  **목표력 3 N에서 전체 20
스텝 완주**로 끝났다.  이 절만 읽으면 현재 설정과 그 근거를 알 수 있다.

### 최종 검증 설정

| 항목 | 값 | 비고 |
|---|---|---|
| 목표 접촉력 | **3.0 N** | 5 N·6 N은 재현성 있게 실패 (아래) |
| 법선 M / K / D | 6.0 / 100 / **300** | ζ=1.81 과감쇠 |
| `min_normal_drive_force_n` | **4.0** | 단방향 접촉 하한 클램프 |
| `max_normal_trim_m` | 0.020 | |
| `filter_coefficient` | 0.20 | τ=0.04 s |
| PAINT 봉투 fy / norm / raw | 30 / 35 / 45 | 목표의 10배 (커미셔닝 여유) |
| 속도 (paint/approach/travel/retract/search) | 0.020 / 0.005 / 0.030 / 0.010 / 0.002 | 전부 원래값 |
| `runtime_tare_quiet_s` | 2.0 | |
| `paint_cartesian_planning_timeout_s` | 15.0 | 넓은 작업영역 대응 |

실측 결과: 3개 행, 스트로크당 20초, 힘 평균 **2.89 / 3.08 / 2.94 N**, abort 0건.

### 순차적으로 걷어낸 6개 장애

| # | 증상 | 진짜 원인 | 수정 |
|---|---|---|---|
| 1 | 접촉 즉시 abort | `ft_link` 프레임 180° 반전 | URDF (0장) |
| 2 | PRECONTACT_TARE 데드락 | bias 게이트(1.0 N)가 eft 드리프트보다 낮음 | 게이트 2.5 N |
| 3 | 2행 tare 거부 | 이동 직후 서보 잔진동, 정착 0.75 s 부족 | `runtime_tare_quiet_s` 2.0 |
| 4 | 단일 노이즈로 전체 정지 | `raw_debounce_s: 0.0` (미분 트립에 디바운스 0) | 전 모드 0.02 |
| 5 | **FJT 타임아웃 hard stop 5회** | 접촉 확정이 스텝 자연완료 40 ms 전에 도착 → cancel이 SUCCESS와 경합 → 결과 유실 | **접촉탐색 중 cancel 제거** |
| 6 | **PAINT 중 힘 폭주 (-40~-60 N)** | ① 웨이포인트가 실제 벽보다 안쪽 ② 접촉 채터 | **깊이 보정 + 감쇠 300** |

#### 5번 — cancel 경합

탐색 스텝(0.5 mm)은 ~0.4 s에 자연 완료되는데 접촉 확정은 모니터 필터+홀드
때문에 ~0.36 s에 도착한다.  거기서 cancel을 쏘면 controller가 수락하자마자
goal이 SUCCESS로 끝나며 terminal result가 미아가 된다.  스텝이 이미 충분히
짧으므로 **취소하지 않고 자연 완료를 기다리는 것**이 구조적 해법
(`moveit_executor.py` 조기접촉 분기).  추가 침투는 최대 한 스텝.

#### 6-① 깊이 보정 — 벽 뒤를 목표로 밀고 있었다

PAINT 웨이포인트는 D405 평면에서 나오는데, CONTACT_SEARCH가 실측한 벽 위치는
버려지고 있었다.  실제 벽이 평면보다 앞에 있으면 경로가 **벽 안쪽**을 가리키고,
이송이 진행될수록 더 깊이 박혀 힘이 램프처럼 상승했다.

```
보정량 = precontact_clearance - 실제_탐색이동거리
```

실측 검산: 평면 X=0.5240, 탐색 7 mm 이동 → 보정 +3.0 mm → 웨이포인트 X=0.5210
= 실측 벽 위치와 정확히 일치.  행마다 보정량이 **+1 ~ +4 mm로 달랐다** — 평면
하나로 벽 전체를 맞출 수 없다는 직접 증거.

RETRACT/FINAL_RETRACT도 같은 보정을 받아야 한다.  PAINT만 옮기면 이탈 여유가
보정량만큼 줄어 `CONTACT_ESCAPE_NOT_BOUNDED_NORMAL_OUTWARD`로 걸린다 (실측:
+3 mm 보정 → 이탈 7 mm vs 하한 8 mm).

#### 6-② 접촉 채터 — 트램펄린

운영자가 "EOAT가 벽 앞뒤로 트램펄린처럼 흔들린다"고 관찰.  100 Hz 로그로 정량화:

```
주파수 2.20 Hz,  진폭 ±3.08 N,  평균 2.84 N
→ 접촉 강성 Ke ≈ 1046 N/m,  ζ = 0.96
→ 평형 침투 2.9 mm 인데 진동이 ±2.9 mm  ⇒ 매 주기 침투 0 도달 = 접촉 이탈
```

선형 안정성은 멀쩡했다(ζ 0.96).  원인은 **접촉의 단방향성** — 벽은 밀 뿐 당기지
못하므로 이탈 구간이 생기고 재충돌한다.  처방은 과감쇠:

| | D=160 | D=300 |
|---|---|---|
| 진폭 | ±3.08 N | ±0.95~1.60 N |
| 최저 힘 | 0.00 N | 1.10~2.16 N |
| **접촉 이탈 비율** | **7.3%** | **0%** |

이탈이 0이 되자 fz 단조증가 → 힘 붕괴 현상도 함께 사라졌다.  초당 2.2회 ×
20초 = 44회 재충돌이 접선 방향으로 누적되던 것으로 보인다 (인과 확정은 아님).

#### 하한 클램프 (`min_normal_drive_force_n`)

폭주 증폭 경로를 끊는다.  측정 법선력이 음수가 되면
`F_total = 측정 + 지령 = (-46) + (-3) = -49 N`이 되어 어드미턴스가 속도 캡으로
벽에 파고든다.  벽은 당길 수 없으므로 지령보다 더 음수인 구동력은 물리적으로
불가능 → 하한을 걸면 나쁜 값에도 평소 속도로만 누른다.

**가드가 아니라 그 반대다** — 멈추게 하는 게 아니라 멈추지 않게 한다.  실측:
같은 행이 6초 만에 죽던 것이 19초(스트로크의 95%)를 버텼다.
모니터는 클램프 전 원본 값을 계속 보므로 안전 판정에는 영향이 없다.
**값은 항상 지령 접촉력보다 커야 한다** (안 그러면 정상 접근을 잘라낸다).

### 목표력 상향 시도 — 3 N이 현재 한계

| 목표 | 결과 | 붕괴 직전 실측 fy 최대 |
|---|---|---|
| **3.0 N** | **20/20 완주** | 4.23 N |
| 5.0 N | RAMP_UP 중 붕괴 | 2.95 N |
| 6.0 N | RAMP_UP 중 붕괴 | 3.15 N |

3 N에서는 4.2 N까지 나오므로 센서/계측의 절대 한계는 아니다.  **3 N 부근을
지속 유지하려 할 때** 무너진다.  5 N 런에서는 램프 중간에 접촉을 한 번 놓쳤다가
(0.00 → -0.75 N) 회복한 뒤 최종 붕괴했다 — 목표가 높을수록 접촉이 불안정해진다.

기구적 이상은 운영자가 확인 결과 없음(벽·마운트·브래킷 정상, 롤러 자유 회전).
**원인 미상.**

### 미해결 / 다음 세션

1. **3 N 천장** — 위 표. 가장 큰 미해결 항목
2. **fz 단조증가 → 힘 붕괴** — D=300으로 발생 빈도는 크게 줄었으나 기전 미규명.
   fz가 fy보다 **0.6초 먼저** 신호를 준다 (정상 최대 -2.5 N → 붕괴 시 -8 N).
   조기 감지가 필요해지면 fz 축 한도를 12 → 5 N (정상 최대의 2배)로 조이면 된다
3. **K=0 (순수 힘 제어)** — 1회 시도했으나 계측 실패로 결과 미확보.
   장점: 힘 오차 0.3 N 제거, 웨이포인트 위치와 힘의 결합 해소, 깊이 보정 불필요.
   **주의: K=0이면 `damping_ratio` 백업 경로가 0이 되어 `damping_absolute[1]`이
   유일한 감쇠원이 된다.** 비우면 무감쇠 자유질량
4. **rx/rz 회전 컴플라이언스** — 5.5장 참조. 컨트롤러에 회전 trim/rate 경계
   추가가 선행되어야 함 (현재 모든 클램프가 병진 블록에만 적용)
5. **eft 유령 힘** — 자세 변화만으로 ~1 N, 유휴 40분 드리프트 >3 N

### 커미셔닝 중 조정한 타임아웃 (실측 근거)

| 항목 | 값 | 근거 |
|---|---|---|
| `contact_search_timeout_s` | 30.0 | 스텝 간 0.15 s dwell 추가로 최악 24 s |
| `fjt_cancel_timeout_s` | 5.0 | 수락된 cancel의 결과 지연 |
| `paint_cartesian_planning_timeout_s` | 15.0 | 0.85×0.79 m 작업영역에서 3 s 초과 |
| `ACM_TRANSACTION_TIMEOUT_S` (코드 상수) | 8.0 | 2 s 초과 시 ACM unknown → 전체 재기동 강제 |

### 운영 수칙

- **T3 단독 재기동 금지** — 반드시 T1과 함께
- **T2 재기동 필요** — 목표력·속도 변경 시 (경로 생성 시점에 값이 박힘)
- **T1 재기동 필요** — `admittance_controller` 리빌드 후 (구 `.so`가 신규
  파라미터를 undeclared로 거부)
- `human_collab:=true` 금지 — 하드웨어 1.2 N 데드밴드가 저역 힘 제어를 파괴

---

## 0. 결론 요약

**질문("guard가 너무 타이트한가?")에 대한 답: 부분적으로 맞지만, 그것이 근본 원인은 아닙니다.**

Guard 중 실제로 과도하게 타이트한 항목은 2개뿐입니다 (`raw_force_derivative_nps` + `raw_debounce_s=0`,
`contact_off_axis_force_n=3.0`). 나머지 임계값은 오히려 합리적이거나 느슨한 편입니다.

진짜 문제는 **어드미턴스 파라미터가 "힘 제어"가 아니라 "위치 제어"로 설정되어 있다**는 것입니다.
법선축에 `stiffness = 1000 N/m` 스프링이 걸려 있어서, 목표 힘 1.6 N으로 만들 수 있는 최대 변위가
**1.6 mm**입니다. 그런데 시스템이 감내해야 하는 기하 오차(D405 평면 피팅 잔차, 롤러 각도 오차)는
**5~35 mm** 수준입니다. 즉 컴플라이언스 권한이 필요량보다 한 자릿수 부족합니다.

그 결과 접촉 시 다음 중 하나가 반드시 발생합니다:

- 벽이 계획면보다 **멀면**: 롤러가 닿지 않고 힘 = 0 → 접촉 상실
- 벽이 계획면보다 **가까우면**: 스프링이 밀어붙여 힘이 `1.0 N/mm`씩 증가 → 과힘 latch
- 롤러가 **한쪽 끝만** 닿으면 (각도 컴플라이언스 없음): 접선력/토크 급증 → `OFF_AXIS_CONTACT` /
  `TORQUE_LIMIT` latch

**Guard 임계값을 올리면 abort는 멈추지만 힘 제어는 여전히 동작하지 않습니다.** 임피던스 파라미터를
먼저 고쳐야 합니다.

우선순위:

| 순위 | 항목 | 영향 |
|---|---|---|
| **P0-0** | **CONTACT_SEARCH 구간에서 컴플라이언스가 완전히 OFF** | 첫 접촉을 강체로 함. 과침투가 그대로 힘 스파이크 |
| **P0-1** | 법선축 `stiffness: 1000 N/m` | 힘 제어 불가. 표면오차 1 mm = 힘오차 1 N |
| **P0-2** | 회전 컴플라이언스 없음 (`selected_axes` Y만) + 175 mm 롤러 | 한쪽 끝 접촉 → 접선력/토크 latch |
| **P0-3** | `max_normal_trim_m: 0.005` < 기하 불확실성 | 정상 동작 중 trim 소진 → `ADMITTANCE_TRIM_LIMIT` |
| **P1-1** | `raw_force_derivative_nps: 900` + `raw_debounce_s: 0.0` | 단일 샘플 노이즈로 즉시 abort |
| **P1-2** | CONTACT_SEARCH 검출 지연 ≈ 0.22 s → 오버슈트 | 정지 시점에 이미 `NORMAL_OVERFORCE` 근접 |
| **P1-3** | `filter_coefficient: 0.02` 오버라이드 (τ ≈ 0.49 s) | 힘 피드백 지연, 기존 튜닝값을 이유 없이 되돌림 |
| **P2** | `_pending_reason` 공유 슬롯 버그 | filtered/raw 디바운스가 서로 리셋 |
| **P2** | `normal_limit_reached` 3중 의미 혼재 | 정상 포화를 고장으로 오판 |
| **P2** | `human_collab` 1.2 N 데드밴드 지뢰 | 켜는 순간 1.6 N 힘 제어 완전 파괴 |

---

## 1. 현재 접촉 경로 (파악한 구조)

```
D405 평면 (권위) ──> sketch_to_waypoints ──> moveit_executor
                                                  │
                                    mode / force_n / enable
                                                  ▼
                                    painting_wrench_reference_node
                                       (램프 + 슬루 + TCP→ft 변환)
                                                  │  requested_wrench
                                                  ▼
                                    painting_wrench_guard_node
                                       (12종 blocker, 실패 시 정확히 0)
                                                  │  /admittance_controller/wrench_reference
                                                  ▼
   ft_sensor (tare 후) ─────────> admittance_controller (100 Hz)
                                    offsetted = 측정 + 지령      (admittance_controller.cpp:594)
                                    M·ẍ + D·ẋ + K·x = offsetted
                                                  │  normal_limit_reached
                                                  ▼
   ft_sensor / _raw ────────> painting_force_monitor_node (50 Hz)
                                    6축 envelope + latch → abort
```

동작 모드 시퀀스:
`IDLE → APPROACH_PRECONTACT(10 mm 이격) → 런타임 tare → CONTACT_SEARCH(힘 OFF, 위치 탐색)
→ RAMP_UP(2 s) → PAINT → RAMP_DOWN(1 s) → RETRACT`

구조 자체는 잘 설계되어 있습니다. 게이트/리스/ACK/fail-closed 패턴은 산업 수준이고,
`wrench_guard.py`와 `force_safety.py`를 ROS에서 분리해 단위 테스트한 것도 좋습니다.
문제는 **제어 물리 파라미터**에 있습니다.

---

## 2. P0 근본 원인

### 2.0 접촉 순간에 실제로 일어나는 일 (타임라인)

#### P0-0. CONTACT_SEARCH 전 구간이 강체 위치 제어입니다

이것이 "동작하다가 접촉에서 문제"의 직접적인 메커니즘입니다. 근거 체인:

| 단계 | 위치 | 내용 |
|---|---|---|
| 1 | `moveit_executor.py:7030` | `_publish_painting_command("CONTACT_SEARCH", 0.0, enable=False)` |
| 2 | `wrench_guard.py:15` | `NONZERO_MODES = {"RAMP_UP", "PAINT", "RAMP_DOWN"}` — CONTACT_SEARCH **미포함** |
| 3 | `wrench_guard.py:124-125` | → blocker `MODE_NOT_FORCE_CAPABLE` |
| 4 | `wrench_guard.py:172-178` | blocker 존재 → 출력 `ZERO_WRENCH`, **`compliance_enabled = False`** |
| 5 | `admittance_controller.cpp:585-592` | `!compliance_active_` → 측정 렌치·지령 렌치 **둘 다 0으로 치환** |

결과: **어드미턴스 오프셋이 0으로 수렴하여, 첫 접촉을 만드는 바로 그 구간에서 로봇은
6축 전부 위치 제어인 무한 강성 소스가 됩니다.** 컴플라이언스는 접촉이 이미 성립하고
힘이 이미 튄 *뒤에* RAMP_UP에서야 켜집니다.

#### 탐색 속도 — 평균은 강제되지만 세그먼트 내 피크는 더 높습니다

`_plan_contact_search_step`이 넘기는 `velocity_scaling_factor`는 조인트 리미트 대비
스케일일 뿐이지만, 계획 결과는 `moveit_executor.py:8991`
`_limit_trajectory_to_cartesian_speed`가 `duration = distance / speed_mps`로 재스케일합니다.
따라서 **세그먼트 평균 2 mm/s는 실제로 강제됩니다.**

다만 재스케일은 타이밍을 균일 배율로 늘리는 것이라 프로파일 형상은 유지되고,
0.5 mm 삼각 프로파일의 **피크 속도는 평균의 1.5~2배(3~4 mm/s)** 입니다.
접촉은 세그먼트 어디에서든 발생할 수 있으므로 오버슈트 계산에는 피크를 써야 합니다.

#### 검출 → 정지 지연 합산

| 구간 | 지연 | 근거 |
|---|---|---|
| 모니터 IIR 필터 | 100 ms | `force_filter_tau_s: 0.10` |
| 접촉 확정 유지 | 100 ms | `contact_confirm_duration_s: 0.10` |
| 모니터 발행 주기 | 20 ms | `publish_rate_hz: 50.0` |
| executor 감시 타이머 | 20 ms | `moveit_executor.py:10624` `create_timer(0.02, check_guard)` |
| FJT cancel 왕복 + 감속 | 30~80 ms | — |
| **합계** | **270~320 ms** | |

#### 단계별 힘 추이 (Kₑ = 롤러+벽 접촉 강성)

**① CONTACT_SEARCH 정지 시점** — 침투 초과 = 속도 × 0.27 s (검출 1.5 N 기준)

| Kₑ | 평균 2 mm/s (0.54 mm) | 피크 3 mm/s (0.81 mm) |
|---|---|---|
| 3,000 N/m (폼 롤러 면접촉) | 3.1 N | 3.9 N |
| 8,000 N/m (롤러 끝단/경사 접촉) | 5.8 N | **7.9 N** ← 기존 `force_axis_n[1]: 8.0` 턱밑 |

컴플라이언스가 꺼져 있으므로 이 힘을 흡수할 수단이 전혀 없습니다.
경사 접촉(P0-2)이면 Kₑ가 커지고 접선력·토크까지 동반되어 여기서 바로 latch됩니다.

**② RAMP_UP 진입 (컴플라이언스 ON 되는 순간)**

지령은 0에서 시작하는데 측정은 이미 3.9 N입니다. 이 불평형력이 로봇을 벽에서 밀어냅니다.

- 가속도 `3.9/8 = 0.49 m/s²` → 클램프 `0.050` 대비 **10배 포화** → `normal_limit_reached` TRUE
- 후퇴 속도 `3.9/715 = 5.5 mm/s`
- 힘이 3.9 N → **1.0 N으로 급락** (`contact_release_threshold_n: 0.8` 바로 위 — 접촉 상실 직전)

**③ RAMP_UP 완료 후 정착값** — 스프링이 탐색 정지 위치를 기억합니다

```
F_최종 = [K·F_탐색정지 + Kₑ·F_목표] / (K + Kₑ)
       = [1000×3.9 + 3000×1.6] / 4000 = 2.18 N     (목표 1.6 N 대비 +36%)
```

**즉 CONTACT_SEARCH의 과침투 오차가 `K/(K+Kₑ) = 25%` 비율로 최종 접촉력 오차에 그대로
전이됩니다.** 탐색이 어디서 멈추든 그 오차가 힘 오차로 남습니다.
K를 100 N/m로 낮추면 같은 식이 `[100×3.9 + 3000×1.6]/3100 = 1.61 N`이 되어
전이 비율이 **25% → 3%** 로 떨어집니다.
이것이 P0-1이 왜 근본 원인인지 보여주는 가장 명확한 지점입니다.

**④ PAINT 이송 중**

| 항목 | 값 |
|---|---|
| 이송 속도 | 20 mm/s (`paint_speed_mps: 0.020`) |
| 힘 필터 지연 | τ = 0.49 s → **10 mm 이동한 뒤에야 힘 변화 인지** |
| 필요 법선 속도 (파장 100 mm, 진폭 4 mm 요철) | 5 mm/s |
| 가능 법선 속도 (1.6 N 오차 기준) | **2.24 mm/s** |

**필요량의 45%밖에 못 냅니다.** 요철을 만나면 힘이 튀거나 접촉이 끊어지고,
누적 trim이 5 mm에 닿으면 `ADMITTANCE_TRIM_LIMIT`로 abort합니다.

#### 이 타임라인이 시사하는 것

접촉에서 문제가 나는 이유는 "guard가 예민해서"가 아니라
**접촉을 만드는 구간에 컴플라이언스가 없고, 켜진 뒤에도 스프링 때문에 힘 제어가 안 되기 때문**입니다.
Guard는 그 결과로 발생한 실제 과힘/과토크를 정직하게 잡아내고 있는 것에 가깝습니다.

**추가 수정 (4.1~4.5절과 병행):**

- `wrench_guard.py:15`의 `NONZERO_MODES`에 `CONTACT_SEARCH`를 추가하고, 탐색 중에는
  **목표 힘의 30~50% 수준(0.5~0.8 N)을 지령**하여 컴플라이언트하게 접근하도록 변경.
  그러면 접촉 자체가 힘 제어로 성립하고 CONTACT_SEARCH → RAMP_UP 전환 시 불연속이 사라집니다.
- 탐색 속도를 실제로 강제하려면 `velocity_scaling` 대신 계획된 궤적의
  타임스탬프를 직접 재파라미터화해야 합니다 (또는 스텝을 0.2 mm로 줄여 프로파일 상한을 낮춤).

### P0-1. 법선축 스프링이 힘 제어를 무력화

`config/painting_system_real.yaml:33-38`

```yaml
admittance:
  selected_axes:  [false, true, false, false, false, false]
  mass:           [10.0, 8.0, 10.0, 1.0, 1.0, 1.0]
  damping_ratio:  [2.828427, 4.0, 2.828427, ...]
  stiffness:      [1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0]
```

`admittance_rule_impl.hpp:146-147`에서 감쇠는 다음과 같이 유도됩니다:

```
D = damping_ratio · 2 · √(M · K) = 4.0 · 2 · √(8.0 · 1000) = 715.5 N·s/m
```

법선축 실효 파라미터: **M = 8 kg, D = 715.5 N·s/m, K = 1000 N/m**

여기서 나오는 숫자들:

| 항목 | 계산 | 값 |
|---|---|---|
| 1.6 N으로 만들 수 있는 최대 변위 | `F/K = 1.6/1000` | **1.6 mm** |
| 1.6 N에서의 정상상태 법선 속도 | `F/D = 1.6/715.5` | **2.24 mm/s** |
| 표면 오차 → 힘 오차 민감도 | `K·Kₑ/(K+Kₑ)`, Kₑ≫K | **≈ 1.0 N/mm** |

세 번째 줄이 핵심입니다. **계획면이 실제 벽과 1 mm 어긋나면 접촉력이 1.6 N에서 0.6 N 또는
2.6 N으로 바뀝니다.** 2 mm 어긋나면 접촉이 끊어지거나 3.6 N이 됩니다.

그런데 `d405_surface_refiner` 허용 오차는 (`painting_system_real.yaml:183-186`):

```yaml
max_rms_residual_m: 0.004    # 4 mm
max_residual_m:     0.012    # 12 mm
max_normal_delta_deg: 12.0   # 12도
```

즉 **평면 모델이 4 mm RMS까지 합격 처리되는데, 컴플라이언스 권한은 1.6 mm**입니다.
힘 제어 루프가 흡수해야 할 오차가 흡수 능력의 2.5배입니다.

> 힘 제어 축의 stiffness는 0이어야 합니다. `K·x` 항이 지령 힘과 직접 경쟁하기 때문에,
> K가 유한한 한 정상상태 힘 오차는 항상 `K · (표면오차)`만큼 남습니다.

**단, `stiffness: 0.0`을 그냥 넣으면 안 됩니다.** `D = ratio · 2 · √(M·K)`이므로
K = 0 → **D = 0** → 순수 이중적분기(자유질량) → 발산합니다.
`controllers_admittance.yaml:69-76` 주석의 "K=0에서 조인트 드리프트"는 정확히 이 현상이며,
당시 필터로 덮은 것이 원인 미해결의 흔적입니다. 해결책은 4.1절 참조.

### P0-2. 175 mm 롤러에 회전 컴플라이언스가 없음

`selected_axes: [false, true, false, false, false, false]` — **병진 Y 1축만 컴플라이언트**입니다.
rx/rz(롤러 기울기)는 완전한 위치 제어입니다.

롤러 제원 (`rr_00a_b_eoat_no_camera.urdf.xacro:7-8`): 길이 **175 mm**, 반경 26 mm.
선접촉이므로 벽면 법선과 롤러 축의 각도 오차 θ가 그대로 양끝 간극 차이가 됩니다:

| 법선 각도 오차 | 175 mm 양끝 간극차 | trim 예산(5 mm) 대비 |
|---|---|---|
| 0.5° | 1.5 mm | 30% |
| **1.6°** | **4.9 mm** | **100% (소진)** |
| 3° | 9.2 mm | 184% |
| 12° (refiner 허용 상한) | 37 mm | 740% |

**즉 법선 추정이 1.6°만 틀려도 trim 예산 전체가 각도 오차 한 항목으로 소진됩니다.**
D405 hand-eye 캘리브레이션 + RANSAC 평면 피팅의 현실적 법선 정확도는 잘해야 ±1~2°입니다.

각도가 틀어진 상태로 누르면 롤러는 한쪽 끝만 닿고, 그 지점에서:

- 마찰에 의한 접선력 발생 → `contact_off_axis_force_n: 3.0` (CONTACT_SEARCH) 초과 → `OFF_AXIS_CONTACT`
- 중심에서 87 mm 떨어진 편심 하중 → 10 N 접촉 시 **0.87 N·m** →
  CONTACT_SEARCH `torque_axis_nm: [1.0, 1.0, 1.0]` 바로 아래 → `TORQUE_LIMIT` 임박

이것이 "접촉 시 문제"의 가장 흔한 물리적 시나리오입니다.

### P0-3. trim 예산이 기하 불확실성보다 작음

`painting_system_real.yaml:19-21`

```yaml
max_normal_trim_m: 0.005          # 5 mm
max_normal_velocity_mps: 0.010
max_normal_acceleration_mps2: 0.050
```

`admittance_rule_impl.hpp:288`에서 `|trim| ≥ 5 mm`이면 `normal_limit_reached_ = true`,
이것이 `painting_force_monitor_node.py:806-817`에서 RAMP_UP/PAINT/RAMP_DOWN 중 0.25 s 지속 시
`ADMITTANCE_TRIM_LIMIT` latch → 모션 abort입니다.

필요한 trim 예산을 합산하면:

| 오차원 | 크기 |
|---|---|
| D405 평면 RMS 잔차 (허용치) | 4 mm |
| 법선 각도 오차 1° × 롤러 반길이 87.5 mm | 1.5 mm |
| CONTACT_SEARCH 정지 오버슈트 (2.4절) | 0.5~1 mm |
| 벽면 자체 요철 | 2~5 mm |
| **합계** | **8~11 mm** |

**5 mm 예산으로는 정상 운전 중에도 trim이 소진됩니다.** 그러면 이건 "고장"이 아니라
"정상적인 권한 부족"인데, 시스템은 이를 abort로 처리합니다.

### P0-4. 가속도 클램프가 정상 과도응답에서 포화

`max_normal_acceleration_mps2: 0.050`, M = 8 kg → **불평형력 0.40 N만 넘으면 클램프 포화**.
목표 힘이 1.6 N인데 0.4 N에서 포화한다는 것은, 모든 접촉 과도 구간에서
`normal_limit_reached`가 참이 된다는 뜻입니다.

계산해보면 RAMP_UP 진입 시(측정 1.5 N, 지령 0 N) 가속도는 `1.5/8 = 0.1875 m/s²`로 즉시 포화하고,
`M/D = 11 ms` 시상수로 감쇠하여 **약 15 ms 후 해제**됩니다.
15 ms < 250 ms 이므로 이것만으로 abort가 나지는 않지만, **정상 동작이 고장 플래그를 올린다**는
설계 결함이며, D를 정상화(4.1절)하면 이 15 ms가 수백 ms로 늘어나 실제 abort가 됩니다.

---

## 3. Guard 평가 (타이트/느슨 판정)

### 3.1 실제로 타이트한 것

#### `raw_force_derivative_nps` + `raw_debounce_s: 0.0` ← **가장 위험**

`force_safety.py:348-358`

```python
@staticmethod
def _derivative(current, previous, now, previous_at):
    if previous_at is None or now <= previous_at:
        return (0.0,) * 6
    dt = now - previous_at            # ← dt 하한 없음
    return tuple((value - old) / dt for value, old in zip(current, previous))
```

문제 3가지가 겹칩니다:

1. **`dt`가 헤더 스탬프가 아니라 콜백 수신 시각(`time.monotonic()`) 기준**입니다.
   ROS 2 / DDS 전달 지터로 두 메시지가 몰려 도착하면 `dt`가 10 ms가 아니라 1 ms가 됩니다.
2. **`dt` 하한이 없습니다.** 미분값이 그대로 10배 부풀려집니다.
3. **`raw_debounce_s: 0.0`** → 단 한 샘플로 즉시 `RAW_IMPACT` latch.

정상 `dt = 10 ms`(100 Hz)에서 900 N/s를 트립하려면 샘플 간 9 N 변화가 필요하지만,
`dt = 1 ms`로 몰려 도착하면 **0.9 N 변화만으로 트립**합니다. ±200 N 급 AFT200 센서에서
0.9 N은 노이즈 수준입니다.

여기에 접촉 순간 EOAT의 구조 공진(롤러 175 mm 캔틸레버, 수십~수백 Hz)이 겹치면
링잉 진폭 1.5 N × 100 Hz → `2π·100·1.5 ≈ 942 N/s`로 900 N/s를 정면으로 넘깁니다.

> **접촉 순간 즉시 abort가 나고 있다면 1순위 용의자입니다.**

#### `contact_off_axis_force_n: 3.0` (CONTACT_SEARCH)

목표 법선력이 1.6 N인데 접선력 상한이 3.0 N입니다. P0-2의 편심 접촉이 발생하면
마찰계수 0.5 기준 법선 6 N에서 이미 3 N에 도달합니다. 롤러 각도가 완벽하다는 전제에서만
성립하는 값입니다.

#### `contact_opposite_force_n: 1.5` (CONTACT_SEARCH)

tare 잔차나 부호 반전 하나로 트립됩니다. 목표 힘과 같은 크기(1.5 vs 1.6 N)라는 게 문제입니다.

### 3.2 오히려 느슨하거나 무의미한 것

| 파라미터 | 값 | 판정 |
|---|---|---|
| `force_axis_n[1]` (PAINT) | 15.0 N | 목표 1.6 N의 9배 — 과도하게 느슨. 실제 이상 감지 불가 |
| `force_norm_n` (PAINT) | 18.0 N | 동일 |
| `max_normal_velocity_mps` | 0.010 | D=715이라 실제 속도는 2.24 mm/s. **사실상 비활성** |
| `sensor_force_saturation_n` | 190 N | 정상 |
| `force_derivative_nps` (filtered) | 200 N/s | τ=0.1 s 필터 뒤라 20 N 계단이 필요. 느슨 |

**진단이 어려운 이유가 여기 있습니다.** 정상 동작 범위(1.6 N)와 abort 임계(15 N) 사이가
비어 있어서, 뭔가 잘못되면 조용히 9배까지 벗어난 뒤에야 latch가 걸립니다.
목표 힘 대비 2~3배 지점에 **경고(warn) 레벨**이 필요합니다.

### 3.3 Guard 로직 자체의 버그

#### `_pending_reason` 슬롯이 filtered/raw 채널 간 공유됨

`force_safety.py:270-271, 298-317`

```python
self._pending_reason = NONE      # 단일 슬롯
self._pending_since  = None
```

`process_filtered()`와 `process_raw()`가 **같은 슬롯을 공유**합니다. 결과:

- `process_filtered()`의 `if candidate is None: self._clear_pending()` (489-490행)이
  **raw 채널의 대기 중인 RAW_IMPACT 디바운스를 지웁니다.**
- 반대로 raw impact가 대기 중이면 filtered 후보의 디바운스 타이머가 리셋됩니다.

두 채널이 100 Hz로 교차 실행되므로, **filtered 디바운스(20 ms = 2샘플)는 raw가 개입하면
사실상 영원히 완성되지 않을 수 있습니다.** 이것은 검출 누락(안전 저하) 방향의 버그입니다.
채널별로 pending 슬롯을 분리해야 합니다.

#### `normal_limit_reached`가 3가지 다른 의미를 하나의 Bool로 방출

`admittance_rule_impl.hpp:288, 315-316, 350`에서 이 플래그가 참이 되는 조건:

1. `|trim| ≥ max_normal_trim_m` — **진짜 "권한 소진"** (abort 정당)
2. 가속도 클램프 발동 — **정상 포화** (abort 부당)
3. 속도 클램프 발동 — **정상 포화** (abort 부당)

2·3번은 리미터가 제 일을 하고 있다는 뜻이지 고장이 아닙니다.
그런데 소비자(`painting_force_monitor_node.py:806-817`)는 셋을 구분하지 못하고
0.25 s 지속 시 무조건 `ADMITTANCE_TRIM_LIMIT`로 abort합니다.

`admittance_rule.hpp:160`에 `normal_trim_m()` 접근자가 **이미 존재하는데 publish되지 않습니다.**
분리 발행이 쉬운 수정입니다 (4.3절).

---

## 4. 수정안

### 4.1 [P0] 법선축을 실제 힘 제어로 전환

먼저 컨트롤러에 **절대 감쇠 파라미터**를 추가해야 합니다. 현재는 `D = ratio·2·√(M·K)`
경로밖에 없어서 K를 낮추면 D도 같이 무너집니다.

`admittance_controller_parameters.yaml`에 추가:

```yaml
damping_absolute: {
  type: double_array,
  default_value: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  description: "축별 절대 감쇠 [N·s/m]. 0보다 크면 damping_ratio 대신 이 값을 사용.
                힘 제어 축(stiffness≈0)에서 damping_ratio가 무의미해지는 문제를 해결.",
  validation: { fixed_size<>: 6 }
}
```

`admittance_rule_impl.hpp:146-147`:

```cpp
admittance_state_.damping[idx] =
  (parameters_.admittance.damping_absolute[i] > 0.0)
    ? parameters_.admittance.damping_absolute[i]
    : parameters_.admittance.damping_ratio[i] * 2 *
        std::sqrt(admittance_state_.mass[idx] * admittance_state_.stiffness[idx]);
```

그 다음 프로파일 (`painting_system_real.yaml`):

```yaml
admittance_controller:
  ros__parameters:
    max_normal_trim_m: 0.020              # 5 mm → 20 mm (3.1절 예산 8~11 mm + 여유)
    max_normal_velocity_mps: 0.015        # 표면 추종에 필요한 5 mm/s 확보
    max_normal_acceleration_mps2: 0.50    # 0.05 → 0.50 (정상 과도에서 포화 금지)

    ft_sensor:
      filter_coefficient: 0.20            # 0.02 → 0.20 (τ 0.49 s → 0.04 s)

    admittance:
      selected_axes: [false, true, false, true, false, true]   # rx, rz 회전 컴플라이언스 추가
      mass:          [10.0, 6.0, 10.0, 0.05, 1.0, 0.05]
      stiffness:     [1000.0, 100.0, 1000.0, 0.5, 100.0, 0.5]  # 법선 1000 → 100
      damping_ratio: [2.828427, 2.45, 2.828427, 2.0, 2.828427, 2.0]
      damping_absolute: [0.0, 120.0, 0.0, 0.6, 0.0, 0.6]       # 법선/기울기축 절대값
      joint_damping: 12.0
```

근거:

| 값 | 근거 |
|---|---|
| `stiffness[1] = 100` | 표면오차 민감도 1.0 → **0.1 N/mm**. 1.6 N으로 **16 mm** 변위 권한 확보 |
| `damping_absolute[1] = 120` | 1.6 N 오차에서 법선속도 `1.6/120 = 13 mm/s` — 20 mm/s 도장 이송 중 표면 추종 가능 (기존 2.24 mm/s는 부족) |
| `mass[1] = 6` | 필터 지연과의 안정성 마진 확보 (아래 부등식) |
| `filter_coefficient 0.20` | τ = `dt(1−α)/α` = 0.04 s. 0.02는 τ=0.49 s로 **base yaml이 "너무 느리다"고 명시한 방향**으로 되돌린 값 |
| `selected_axes` rx/rz | 175 mm 선접촉이 스스로 벽면에 정렬되도록 함 (P0-2 해소) |

**안정성 확인 부등식** (필터 τ + 어드미턴스 + 환경강성 Kₑ의 3차 특성방정식 Routh 조건):

```
(M + D·τ)(D + K·τ)  >  M·τ·(K + Kₑ)
```

제안값 대입: `(6 + 120·0.04)(120 + 100·0.04) = 10.8 × 124 = 1339`
→ `Kₑ < 1339/(6·0.04) − 100 =` **약 5,480 N/m**

폼 롤러의 실측 Kₑ는 보통 1,600~3,200 N/m이므로 마진이 있습니다.
**단, Kₑ는 반드시 실측해야 합니다** (6.2절). 5,480을 넘으면 M을 줄이거나 τ를 더 줄여야 합니다.

### 4.2 [P1] raw 미분 가드 강건화

`force_safety.py`:

```python
MIN_DERIVATIVE_DT_S = 0.005     # 100 Hz 스트림 기준 하한

@staticmethod
def _derivative(current, previous, now, previous_at):
    if previous_at is None or now <= previous_at:
        return (0.0,) * 6
    dt = max(now - previous_at, MIN_DERIVATIVE_DT_S)   # 지터로 인한 증폭 차단
    return tuple((value - old) / dt for value, old in zip(current, previous))
```

추가로 `dt`를 **수신 시각이 아니라 헤더 스탬프 차이**로 계산하는 것이 근본 해법입니다
(`message_age_s`가 이미 전달되고 있으므로 스탬프 접근 가능).

프로파일:

```yaml
contact_search:
  raw_force_derivative_nps: 1500.0     # 900 → 1500 (EOAT 공진 링잉 여유)
  raw_debounce_s: 0.02                 # 0.0 → 0.02 (단일 샘플 latch 금지, 2샘플 지속 요구)
  contact_off_axis_force_n: 6.0        # 3.0 → 6.0 (rx/rz 컴플라이언스 도입 후 재하향 검토)
  contact_opposite_force_n: 3.0        # 1.5 → 3.0 (목표력 1.6 N의 2배 지점)
```

> `raw_debounce_s: 0.0`은 **어떤 모드에서도** 쓰면 안 됩니다. 미분 기반 트립에 디바운스 0은
> 노이즈 한 샘플 = 즉시 정지와 동의어입니다. 진짜 충돌은 20 ms 이상 지속되므로 검출력 손실은 없습니다.

### 4.3 [P1] `normal_limit_reached` 의미 분리

`admittance_controller.cpp` (400행 부근 publisher, 617행 부근 발행):

```cpp
// 기존 ~/normal_limit_reached 는 "trim 경계 도달"만 의미하도록 축소
msg.data = std::abs(admittance_->normal_trim_m()) >= params.max_normal_trim_m;
limit_reached_rt_publisher_->try_publish(msg);

// 신규: 진단용 실측 trim (abort 판단에는 쓰지 않음)
std_msgs::msg::Float64 trim_msg;
trim_msg.data = admittance_->normal_trim_m();
normal_trim_rt_publisher_->try_publish(trim_msg);
```

`admittance_rule_impl.hpp`에서는 가속도/속도 클램프가 `normal_limit_reached_`를 올리지 않도록
분리 (별도 `rate_saturated_` 플래그로 진단만).

이렇게 하면 `ADMITTANCE_TRIM_LIMIT`가 **"컴플라이언스 권한을 실제로 다 썼다"**는
정확한 의미만 갖게 됩니다.

### 4.4 [P1] CONTACT_SEARCH 오버슈트 축소

현재 검출 지연 합산:

| 구간 | 지연 |
|---|---|
| 모니터 IIR 필터 `force_filter_tau_s: 0.10` | 100 ms |
| `contact_confirm_duration_s: 0.10` | 100 ms |
| 모니터 발행 주기 (`publish_rate_hz: 50`) | 20 ms |
| executor 반응 + FJT cancel | ~50 ms |
| **합계** | **≈ 270 ms** |

탐색 속도 2 mm/s → **0.54 mm 침투 초과**, 여기에 `contact_search_step_m: 0.0005`
스텝 완주분이 더해져 실질 **0.5~1.0 mm**.

Kₑ = 5,000 N/m이면 정지 시점 힘은 `1.5 + 5000×0.0007 ≈ 5 N`,
Kₑ = 8,000 N/m이면 **7.1 N** — CONTACT_SEARCH `force_axis_n[1] = 8.0` 바로 아래입니다.
**마진이 1 N도 안 됩니다.**

```yaml
painting_force_monitor:
  ros__parameters:
    force_filter_tau_s: 0.04            # 0.10 → 0.04
    contact_confirm_duration_s: 0.04    # 0.10 → 0.04
    contact_detect_threshold_n: 1.0     # 1.5 → 1.0 (목표 1.6 N보다 확실히 아래)
    publish_rate_hz: 100.0              # 50 → 100

moveit_executor:
  ros__parameters:
    contact_search_speed_mps: 0.001     # 0.002 → 0.001
```

이러면 지연 ≈ 100 ms, 침투 초과 ≈ 0.1 mm로 줄어듭니다.

> `contact_detect_threshold_n: 1.5`가 목표 힘 `1.6 N`과 거의 같다는 점도 문제입니다.
> "접촉 감지"와 "목표 도달"이 구분되지 않아 CONTACT_SEARCH가 이미 목표 힘까지 눌러버립니다.
> 감지는 목표의 50~60% 지점(1.0 N 이하)이어야 합니다.

### 4.5 [P2] 경고 레벨 도입

현재 1.6 N(정상) ↔ 15 N(abort) 사이가 비어 있습니다. 그 사이에 비-latch 경고를 넣으면
"조용히 벗어나는" 구간이 사라집니다.

```yaml
limits:
  paint:
    force_axis_n: [6.0, 8.0, 6.0]     # 15 → 8 (목표 1.6 N의 5배)
    force_norm_n: 10.0                # 18 → 10
    warn_force_axis_n: [3.0, 4.0, 3.0]   # 신규: latch 없이 진단 토픽만
```

### 4.6 [P2] `human_collab` 지뢰 차단

`rbpodo_hardware_interface.hpp:304`

```cpp
static constexpr double kFtCollabDeadband{1.2};   // [N]
```

`human_collab:=true`로 띄우면 **어드미턴스가 소비하는 `ft_sensor` 렌치에 축당 1.2 N
데드밴드가 걸립니다.** 목표 접촉력이 1.6 N이므로 그 중 75%가 데드밴드 안에 들어가
힘 제어가 bang-bang이 되고 접촉 채터링이 확실히 발생합니다.

현재 `rb10_painting_system.launch.py`는 이 인자를 넘기지 않아 기본값 `false`이므로
**지금은 안전하지만**, 아무 경고 없이 켤 수 있는 상태입니다. 도장 프로파일에서는
명시적으로 거부하도록 launch에 assert를 넣으십시오:

```python
# rb10_painting_system.launch.py
# 1.2 N 데드밴드가 1.6 N 접촉력 제어를 파괴하므로 도장 경로에서는 금지
"human_collab": "false",
```

---

## 5. 요약 패치 표

| 파일 | 항목 | 현재 | 제안 |
|---|---|---|---|
| `painting_system_real.yaml:37` | `stiffness[1]` | 1000.0 | **100.0** |
| `painting_system_real.yaml:35` | `mass[1]` | 8.0 | 6.0 |
| `painting_system_real.yaml:34` | `selected_axes` | Y만 | Y만 유지 — 5.5절 참조, 컨트롤러에 회전 경계 추가가 선행되어야 함 |
| `painting_system_real.yaml:36` | 감쇠 | ratio 4.0 (=715 N·s/m) | `damping_absolute[1] = 120` |
| `painting_system_real.yaml:19` | `max_normal_trim_m` | 0.005 | **0.020** |
| `painting_system_real.yaml:20` | `max_normal_velocity_mps` | 0.010 | 0.015 |
| `painting_system_real.yaml:21` | `max_normal_acceleration_mps2` | 0.050 | **0.50** |
| `painting_system_real.yaml:24` | `filter_coefficient` | 0.02 | **0.20** |
| `painting_system_real.yaml:369` | `raw_force_derivative_nps` (search) | 900.0 | 1500.0 |
| `painting_system_real.yaml:379` | `raw_debounce_s` (search) | **0.0** | **0.02** |
| `painting_system_real.yaml:376` | `contact_off_axis_force_n` | 3.0 | 6.0 |
| `painting_system_real.yaml:375` | `contact_opposite_force_n` | 1.5 | 3.0 |
| `painting_system_real.yaml:275` | `force_filter_tau_s` | 0.10 | 0.04 |
| `painting_system_real.yaml:290` | `contact_confirm_duration_s` | 0.10 | 0.04 |
| `painting_system_real.yaml:288` | `contact_detect_threshold_n` | 1.5 | 1.0 |
| `painting_system_real.yaml:101` | `contact_search_speed_mps` | 0.002 | 0.001 |
| `force_safety.py:357` | `_derivative` dt 하한 | 없음 | `max(dt, 0.005)` |
| `force_safety.py:270` | `_pending_reason` | 단일 슬롯 | filtered/raw 분리 |
| `admittance_rule_impl.hpp:146` | 감쇠 유도 | ratio만 | `damping_absolute` 우선 |
| `admittance_rule_impl.hpp:315,350` | rate clamp → `normal_limit_reached_` | 병합 | 분리 |
| `admittance_controller.cpp:617` | `normal_trim_m` 발행 | 없음 | 신규 토픽 |

---

## 5.5 실제 적용된 변경 (2026-08-13)

빌드/테스트 상태: `colcon build admittance_controller` 통과,
C++ 44 tests / Python 81 tests 전부 통과. **실기 검증은 아직 수행하지 않았습니다.**

### 컨트롤러 (C++)

| 파일 | 변경 |
|---|---|
| `admittance_controller_parameters.yaml` | `admittance.damping_absolute` 6-array 신규 (기본 전부 0.0) |
| `admittance_rule_impl.hpp:146-157` | `damping_absolute[i] > 0`이면 해당 축에서 `damping_ratio` 대신 사용 |
| `admittance_rule_impl.hpp:288-294` | `normal_limit_reached_`를 **trim 소진 전용**으로 축소 |
| `admittance_rule_impl.hpp:322,357` | 가속도/속도 클램프는 `normal_rate_saturated_`로 분리 |
| `admittance_rule.hpp:157-165` | `normal_rate_saturated()` 접근자 추가 |
| `admittance_controller.cpp:400-415, 625-640` | `~/normal_rate_saturated`(Bool), `~/normal_trim_m`(Float64) 신규 발행 |

`~/normal_limit_reached`의 **의미가 바뀌었습니다.** 이제 "컴플라이언스 변위 예산 소진"만
의미하며, 정상 과도의 rate 포화는 더 이상 여기로 나오지 않습니다.
`painting_force_monitor_node`의 `ADMITTANCE_TRIM_LIMIT` 판정은 코드 수정 없이
그대로 올바른 의미를 갖게 됩니다.

### 안전 로직 (Python)

| 파일 | 변경 |
|---|---|
| `force_safety.py:41-51` | `MIN_DERIVATIVE_DT_S = 0.005`, 채널 상수 도입 |
| `force_safety.py:387` | `dt = max(now - previous_at, MIN_DERIVATIVE_DT_S)` — 지터 증폭 차단 |
| `force_safety.py:265-276, 330-355` | `_pending_reason`/`_pending_since`를 filtered/raw **채널별로 분리** |
| `force_safety.py:562-571` | raw 채널 pending은 impact가 해제되면 항상 리셋 |

### 프로파일 (`painting_system_real.yaml`)

5장 표대로 적용. 검증 결과:

T1(`rb10_moveit_full.launch.py`) 병합 순서 그대로 재현한 최종 유효값:

```
축   sel     M       K        D         (ratio 경로)
x    False   10.0    1000.0   565.69    565.69
y    True     6.0     100.0   120.00    120.02   <- 법선 (힘 제어)
z    False   10.0    1000.0   565.69    565.69
rx   False    1.0     100.0    56.57     56.57
ry   False    1.0     100.0    56.57     56.57
rz   False    1.0     100.0    56.57     56.57

filter alpha 0.20 -> tau 0.040 s @100 Hz
max_normal_trim_m 0.020 / vel 0.015 / accel 0.50
안정성          Ke < 5,480 N/m
표면오차 민감도  0.10 N/mm   (기존 1.00)
1.6 N 변위 권한  16.0 mm     (기존 1.6)
1.6 N 법선 속도  13.3 mm/s   (기존 2.24)
```

`contact_search_timeout_s: 10.0 → 20.0`도 함께 올렸습니다. 15 mm 전 구간을 소진하는
탐색은 30스텝 × (0.25 s 이송 + MoveIt Cartesian 왕복) ≈ 10.5 s가 필요해서,
거리 한계(`contact_search_max_distance_m`)가 아니라 타임아웃으로 먼저 FAULT가 났습니다.
실질 안전 경계인 거리 한계는 그대로입니다.
같은 이유로 `contact_search_speed_mps`는 **0.002를 유지**했습니다 —
0.001로 낮추면 18 s가 되어 타임아웃이 확정됩니다. 오버슈트는 지연 단축만으로
0.8 mm → 0.2~0.3 mm로 이미 줄었습니다.

> 안전 여유 하나: `damping_ratio[1] = 2.45`를 `2·2.45·√(6×100) = 120.02`가 되도록
> 맞춰 두었습니다. 만에 하나 `damping_absolute`가 적용되지 않아도 감쇠는 동일한
> 120 N·s/m로 유도되므로, 무감쇠 자유질량 상태로는 빠지지 않습니다.

### 추가된 회귀 테스트

| 테스트 | 검증 대상 |
|---|---|
| `test_derivative_dt_floor_rejects_jitter_amplified_rate` | 1 ms 간격 1 N 노이즈가 RAW_IMPACT를 만들지 않을 것 |
| `test_filtered_and_raw_debounce_windows_are_independent` | raw 샘플이 filtered 디바운스를 리셋하지 않을 것 |
| `test_raw_debounce_survives_interleaved_quiet_filtered_samples` | 그 역방향 |
| `test_raw_debounce_window_resets_when_the_impact_clears` | 해제 시 정상 리셋 |
| `damping_absolute_overrides_ratio_per_axis` (C++) | 축별 override가 실제로 적용될 것 |
| `damping_defaults_to_the_ratio_derivation` (C++) | 기본값에서 기존 동작 불변 |

네 Python 테스트와 `damping_absolute_overrides_ratio_per_axis`는 수정 전 코드에 대해
**실제로 실패하는 것을 확인**했습니다.

### 적용하지 않은 것

#### rx/rz 롤러 기울기 컴플라이언스 — 컨트롤러에 회전 경계가 없어 보류

**P0-2의 처방인 `selected_axes` rx/rz 활성화는 되돌렸습니다.**
한 번 넣었다가 실기 투입 직전에 검토하면서 발견한 사항입니다.

`admittance_rule_impl.hpp`의 모든 클램프는 `X_ddot.block<3,1>(0,0)`,
즉 **병진 블록에만** 적용됩니다. `max_normal_trim_m`, `max_normal_velocity_mps`,
`max_normal_acceleration_mps2` 어느 것도 회전 블록(3..5)을 건드리지 않습니다.

따라서 rx/rz를 컴플라이언트로 만들면 **경계가 전혀 없는 회전 자유도**가 생깁니다.
`stiffness = 0.5 N·m/rad` 기준으로 0.1 N·m가 지속되면 `0.1/0.5 = 0.2 rad = 11°`까지
공구가 돌아가는데, 이 0.1 N·m는 모니터의 `torque_axis_nm: 1.0` abort 한계보다
한참 아래라 아무것도 멈추지 않습니다. 벽면 근처에서 EOAT가 11° 회전하는 것은
허용할 수 없습니다.

**선행 조건:** 법선축과 같은 방식의 회전 trim/rate 경계를 컨트롤러에 추가
(`max_rotation_trim_rad`, `max_rotation_velocity_radps`,
`max_rotation_acceleration_radps2` + 회전 블록 클램프). 그 다음에야 rx/rz를 켤 수 있습니다.

그때까지 P0-2는 **미해결**입니다. 즉 법선 각도 오차가 1.6°를 넘으면 롤러는 여전히
한쪽 끝만 닿습니다. 6.1절 로그에서 `OFF_AXIS_CONTACT` / `TORQUE_LIMIT`가 주된 latch
사유로 나온다면, 이 회전 경계 작업이 다음 순위입니다.

#### 컴플라이언트 CONTACT_SEARCH

**P0-0의 컴플라이언트 CONTACT_SEARCH도 구현하지 않았습니다.** 이유:

- `wrench_guard.NONZERO_MODES`, `painting_wrench_reference_node.ZERO_FORCE_MODES`,
  executor의 `_contact_search_mode_ack_blockers`(`force_enabled is not False`를 명시적으로 요구)
  가 모두 "탐색 중 힘은 반드시 0"이라는 불변식으로 엮여 있어, 셋을 동시에 바꾸는
  안전 semantics 변경입니다. 별도 설계 패스가 필요합니다.
- 한편 **P0-1 수정이 P0-0의 하류 영향 대부분을 이미 제거합니다.** 위 ③에서 보듯
  탐색 정지 오차의 힘 전이가 25% → 3%로 떨어지고, 4.4절 지연 단축이 오버슈트 자체를
  0.8 mm → 0.2~0.3 mm로 줄입니다.
- 남는 것은 강체 탐색 구간의 순간 피크 힘뿐이며, 이는 `contact_detect_threshold_n`
  1.5 → 1.0 하향과 검출 지연 단축으로 완화됩니다.

`d405_surface_refiner.max_normal_delta_deg: 12.0` → 3.0 하향도 적용하지 않았습니다.
이 값은 평면 재추정 **수락 기준**이라, 조이면 접촉 문제와 무관하게 인식 단계가
거부될 수 있습니다. 7장 참조 — 인식 정확도를 먼저 실측한 뒤 결정하십시오.

---

## 6. 검증 절차

### 6.1 먼저 로그로 확정할 것 (분석은 정적, 실기 확인 필요)

접촉 실패 시 다음을 캡처하십시오. 어느 가설이 맞는지 여기서 갈립니다.

```bash
ros2 topic echo /painting_admittance/safety_status --once   # 실패 직후
```

- `reason` 필드:
  - `RAW_IMPACT` → **P1-1 (raw 미분 + debounce 0)** 확정. 4.2절부터 적용
  - `ADMITTANCE_TRIM_LIMIT` → **P0-3 (trim 예산)** 확정. 4.1 + 4.3절
  - `OFF_AXIS_CONTACT` / `TORQUE_LIMIT` → **P0-2 (각도 컴플라이언스)** 확정. 4.1절 `selected_axes`
  - `NORMAL_OVERFORCE` → **P1-2 (탐색 오버슈트)** 확정. 4.4절
- `normal_limit_active_duration_s`, `normal_limit_reached` — trim 소진 여부
- `raw_force_derivative_nps` — 900 대비 실측 피크
- `contact_tangential_force_n` — 3.0 대비 실측

동시 기록:

```bash
ros2 bag record \
  /force_torque_sensor_broadcaster/wrench \
  /force_torque_sensor_broadcaster_raw/wrench \
  /painting_admittance/force_tcp_filtered \
  /painting_admittance/command_force_tcp_y_n \
  /painting_admittance/safety_status \
  /admittance_controller/normal_limit_reached \
  /admittance_controller/compliance_active \
  /painting_admittance/current_mode
```

### 6.2 환경 강성 Kₑ 실측 (4.1절 안정성 판정에 필수)

힘 제어 OFF, 위치 제어로 롤러를 벽면에 0.5 mm씩 눌러가며 정상상태 `Fy` 기록:

```
Kₑ = ΔF / Δx   [N/m]
```

0.5, 1.0, 1.5, 2.0 mm 4점이면 충분합니다. 이 값이 **5,480 N/m을 넘으면**
4.1절 제안값은 불안정하므로 `mass[1]`을 4.0으로 낮추고 `filter_coefficient`를 0.3으로 올린 뒤
부등식을 다시 확인하십시오.

### 6.3 단계적 투입 순서

5.5절 변경은 **한 번에 다 들어가 있습니다.** 실기에서는 되돌리면서 단계적으로 올리십시오.
각 단계에서 `dry_run:=true`로 토픽만 먼저 확인한 뒤 실접촉으로 넘어갑니다.

먼저 `painting_system_real.yaml`에서 다음만 임시로 되돌려 1단계를 만듭니다:

```yaml
admittance:
  stiffness:        [1000.0, 1000.0, 1000.0, 100.0, 100.0, 100.0]
  selected_axes:    [false, true, false, false, false, false]
  damping_absolute: [0.0, 715.5, 0.0, 0.0, 0.0, 0.0]
```

1. **가드 완화 + 필터/지연 개선만** (위 임시 되돌림 상태) — 접촉이 abort 없이 성립하는지 확인.
   힘 정확도는 아직 나쁨. 여기서 6.2절 Kₑ를 실측하십시오
2. **Kₑ 확인** — 5,480 N/m 미만인지. 넘으면 `mass[1]`을 4.0으로 낮추고 부등식 재계산
3. **stiffness 단계 하향** — `stiffness[1]`을 1000 → 500 → 200 → 100으로,
   각 단계에서 `damping_absolute[1]`을 120으로 두고 힘 오버슈트/채터 관찰
4. **envelope 하향분 검증** (4.5절) — 6~8 N 한도에서 오탐이 없는지

(rx/rz 컴플라이언스는 컨트롤러 회전 경계 작업 이후의 별도 단계입니다. 5.5절 참조.)

> 3단계에서 `damping_absolute` 없이 stiffness만 낮추면 **감쇠가 0이 되어 자유질량 발산**합니다.
> `damping_absolute[1]`이 0이 아닌지 반드시 확인하고 넘어가십시오.
> `ros2 topic echo /admittance_controller/status`의 `damping` 배열로 실측 확인할 수 있습니다.

### 6.4 신규 진단 토픽

```bash
ros2 topic echo /admittance_controller/normal_trim_m         # 실제 소비 중인 변위 (m)
ros2 topic echo /admittance_controller/normal_limit_reached   # trim 예산 소진 (abort 근거)
ros2 topic echo /admittance_controller/normal_rate_saturated  # rate 포화 (정상, 진단용)
```

`normal_trim_m`의 피크가 `max_normal_trim_m: 0.020`에 얼마나 근접하는지가
예산이 충분한지에 대한 직접 증거입니다. 5 mm를 넘는다면 기존 설정에서는
반드시 abort했을 상황입니다.

---

## 7. 남은 위험 / 별도 검토 필요

- **목표 힘 1.6 N은 공정값이 아닙니다.** 175 mm 롤러에 1.6 N이면 선하중 0.009 N/mm로,
  실제 도장에 필요한 압력의 1/10 이하입니다. 공정 힘(10~30 N)으로 올리면 4.1절 파라미터를
  재검토해야 합니다 (권한 요구는 줄고, Kₑ 비선형성은 커짐).
- **AFT200(±200 N)으로 1.6 N을 제어하는 것은 풀스케일의 0.8%**입니다. 센서 분해능/드리프트가
  목표값과 같은 자릿수입니다. 공정 힘을 올리지 않을 거라면 저용량 센서 검토가 필요합니다.
- **중력 보상이 비어 있습니다** (`painting_system_real.yaml:29-31`, `CoG.force: 0.0`).
  현재는 pre-contact 자세에서의 런타임 tare가 대신하고 있어 평면 벽(자세 일정)에서는 동작하지만,
  행 간 자세가 바뀌거나 곡면으로 확장하면 즉시 깨집니다. EOAT 무게/CoG 실측 필요.
- **`d405_surface_refiner`의 `max_normal_delta_deg: 12.0`은 롤러 기하와 모순**입니다.
  175 mm 롤러에서 12°는 양끝 37 mm 차이입니다. rx/rz 컴플라이언스를 넣어도 흡수 불가.
  **3° 이하로 조여야 합니다.**
- 본 문서는 정적 분석입니다. 6.1절 로그로 실제 latch 사유를 확정한 뒤 우선순위를 조정하십시오.
