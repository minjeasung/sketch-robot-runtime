# F/T 토크 기반 롤러 평행 제어 (v5 + 안정화 수정)

## 목적과 기본 상태

175 mm 롤러의 좌우 하중이 한쪽으로 몰릴 때 F/T 센서의 접촉 중심 토크를 이용해 롤러를 벽과 평행하게 유지한다. 기존 TCP `+Y` 방향의 20–30 N 힘 밴드는 그대로 유지하고, 롤러 장축 `+X`의 하중 편차만 `Rz`로 천천히 보정한다.

v5의 실기 기본값은 다음과 같다.

- ros2_control 계산 주기: 100 Hz
- 토크 추가 저역통과 필터: 0.10 s
- 계측: 활성화
- 회전 제어: 비활성화 (`roller_balance.control_enabled: false`)

회전 제어가 기본 비활성인 이유는 실제 도구의 F/T 부호, 무게중심(CoG), 잔류 토크, 접촉 중심 위치를 확인하지 않은 상태에서 자동 회전을 허용하지 않기 위해서다. 토픽은 발행되지만, 기존 compliance가 비활성일 때는 컨트롤러 입력 wrench가 0으로 대체된다. 따라서 이 상태의 CoP/토크 진단을 실제 센서 계측으로 해석하면 안 된다. 제어와 독립적인 센서 확인에는 기존 raw/filtered F/T 토픽을 사용한다.

## 제어량

센서 원점 토크를 그대로 쓰지 않는다. 설정된 롤러/벽 접촉 중심으로 wrench를 이동한 뒤 다음 값을 계산한다.

```text
Fn    = abs(Fy)
Tz,c  = Tz(sensor) - (r(sensor→contact) × F)z - torque_bias
x_cop = torque_to_cop_sign * Tz,c / Fn
```

`x_cop`은 롤러 중앙에서 실제 하중 중심이 좌우로 얼마나 벗어났는지 나타낸다. `Fn < 8 N`에서는 나눗셈과 무접촉 잡음 증폭을 피하기 위해 유효하지 않은 값으로 취급하고 자세 제어를 하지 않는다.

하중 중심이 중앙이라는 사실은 전체 접촉면의 압력이 균일함을 보장하지 않는다. 이 제어의 직접적인 목표는 좌우 하중 쏠림 완화이며, 전체 접촉 상태와 도장 품질은 별도 실험으로 확인한다.

토크 목표는 한 점이 아니라 힘에 따라 변하는 범위다.

| 정상력 | 보정 정지: `Fn × 5 mm` | 보정 시작: `Fn × 8 mm` |
|---:|---:|---:|
| 20 N | 0.10 Nm | 0.16 Nm |
| 30 N | 0.15 Nm | 0.24 Nm |

바깥 8 mm에서 보정을 시작하고 안쪽 5 mm에 들어오면 보정을 멈춘 채 현재 회전 trim을 유지한다. 따라서 센서 잡음 때문에 토크 0을 계속 왕복해서 추종하지 않는다. 밴드 밖에서도 5 mm 경계 밖의 토크만 제어 입력으로 사용한다.

## 부드러운 보정과 안전 정지의 구분

- 기존 접촉 도장 경로 기준속도 **20 mm/s**, 정상력 밴드 **20–30 N**은 변경하지 않는다. 보정이 포화되어도 nominal 경로의 시간/속도나 실행 enable을 변경하지 않는다.
- 보정 명령의 소프트 한계: **±0.45°** (`soft_limit_ratio: 0.9` × 하드 한계 0.5°). 한계에서 속도를 갑자기 0으로 만들지 않고, 남은 거리와 정지거리를 계산하여 미리 감속한다. 도달 후에는 자세 보정량만 유지한다. 반대 방향 보정은 다시 허용된다.
- 하드 회전 경계: **±0.5°**, 기존 값 유지. 실제 자세뿐 아니라 `reference + 누적 관절 보정`을 FK로 계산한 명령 자세도 검사한다. 로봇이 못 따라와도 명령 자체가 무제한 누적되지 않는다.
- 최종 보정 각속도 **±0.2°/s**, 가속·감속도 **±1.0°/s²**. 관절 감쇠와 속도 제한 이후에 적용하고, IK 결과를 다시 순방향 변환하여 검사한다. 실제 다음 명령 자세의 유한 회전량도 확인한다. 미분 IK와 유한 FK 사이 오차 여유로 내부 계획은 최대값의 98%를 사용한다.
- Rz 절대 감쇠 **25 N·m·s/rad** 유지. 접촉 해제 시에도 명령 보정량을 기준으로 원래 자세로 부드럽게 복귀한다.
- `soft_limit_active`와 `rate_saturated`는 정상적인 제한기 동작이며 **이 신호만으로 도장 전체를 정지시키지 않는다**. 반면 실제/명령 자세의 하드 경계 도달이나 유효한 제한 명령을 계산할 수 없는 기구학 오류에서는 새 명령을 쓰지 않고 컨트롤러에 ERROR를 반환한다. 컨트롤러 매니저가 해당 체인을 비활성화한다. 기존 모니터의 0.25 s `ROLLER_BALANCE_LIMIT` 감시는 추가 방어로 유지한다.
- 기존 raw/filtered 힘·토크, 센서 포화, 센서 유효성, 로봇 인터록의 안전 한계는 완화하지 않는다. 계산 실패 시 보정량을 갑자기 제거한 nominal 자세를 쓰던 동작도 제거했다.

회전 제한은 nominal 경로에 더하는 **보정 명령**에 대한 제한이다. 전체 로봇 궤적, 실제 서보 응답, 접촉점의 합성 속도나 진동 없는 접촉을 보장하는 값이 아니다. 현재 Jacobian의 기준점은 F/T 원점이므로 실제 롤러 접촉점의 횡이동/마찰 영향도 실기에서 확인해야 한다. 100 Hz는 계산 주기이며 실제 접촉 안정성은 도구 강성, 지연, 마찰, 센서 보정에 따라 달라진다.

## 진단 토픽

```text
/admittance_controller/roller_balance/contact_valid
/admittance_controller/roller_balance/contact_torque_nm
/admittance_controller/roller_balance/cop_offset_m
/admittance_controller/roller_balance/correction_active
/admittance_controller/roller_balance/rotation_trim_rad
/admittance_controller/roller_balance/commanded_trim_rad
/admittance_controller/roller_balance/soft_limit_active
/admittance_controller/roller_balance/rate_saturated
/admittance_controller/roller_balance/limit_reached
```

## 실기 활성화 절차

1. 로봇을 자유 공간의 실제 도장 자세에 정지시키고 기존 runtime tare가 성공하는지 확인한다.
2. 자동 회전을 꺼둔 상태에서 승인된 시험 지그/절차로 롤러 양 끝 하중에 대한 F/T 부호를 확인한다. compliance까지 비활성인 경우 위의 CoP 진단 대신 raw/filtered F/T를 기록하여 접촉 중심으로 변환한다. 동력이 켜진 로봇과 벽 사이에 손을 넣어 시험하지 않는다.
3. 실제 접촉 중심까지의 TCP 오프셋 `contact_center_offset_m: [0.0, -0.27020, 0.0]`과 잔류 `torque_bias_nm`을 로그로 확인한다.
4. 한쪽 끝 하중에서 계산된 `abs(cop_offset_m)`가 대략 롤러 반길이 이내이고, 중앙 하중에서는 0 부근인지 확인한다.
5. 필요한 경우 `torque_to_cop_sign`을 바꿔 CoP 좌우 표기를 맞춘다.
6. 방호/비상정지 및 센서 유효성을 확인한 승인된 저속 시험에서만 `painting_system_real.yaml`의 `roller_balance.control_enabled`를 `true`로 변경한 뒤 컨트롤러를 재시작한다. 관련 파라미터는 실행 중 오활성화를 막기 위해 read-only다.
7. 유효 접촉 조건(측정/명령 정상력 모두 8 N 이상)을 만족하는 정지 접촉에서 `rotation_feedback_sign`을 검증한다. 보정이 하중 중심을 더 바깥으로 보내면 즉시 중지하고 원인을 확인한다. 안전 한계를 높여 계속 운전하지 않는다.
8. 먼저 한 점 정지 접촉, 다음 짧은 단일 스트로크, 마지막으로 다중 스트로크 순서로 확장한다. 각 단계에서 CoP, trim, rate saturation, 원시 안전 토크를 함께 기록한다.

활성화 전에는 로봇 E-stop에 손이 닿는 상태에서 저속·저하중으로 부호를 검증해야 한다. 토크 부호는 절댓값으로 바꾸면 안 된다. 절댓값은 정상력 `Fn`에만 사용한다.

## 오프라인 회귀 검증

`test_roller_balance_dynamics`는 실제 제어 함수를 사용하여 정지/추종 지연, 반복 방향 반전, 밴드 진입 감속, 접촉 해제 복귀, 가변 주기, 결합/비선형 기구학과 감쇠 IK, 하드 경계/기구학 실패, 20 mm/s nominal 경로 유지 등을 검사한다. 이는 실제 벽/롤러 접촉 실험을 대체하지 않는다. `commanded_trim_rad`와 `rotation_trim_rad`를 함께 기록해야 명령과 실제 추종을 구분할 수 있다.
