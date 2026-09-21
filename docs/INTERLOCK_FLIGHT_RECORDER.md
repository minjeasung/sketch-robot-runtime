# 로봇 내부 인터록 비행 기록기

## 목적

이 기록기는 로봇이 움직이는 동안 최근 데이터를 메모리에만 보관하다가 다음 사건 중 하나가 발생하면 기록을 파일로 남긴다.

- Rainbow 로봇 상태에서 collision, self-collision, SOS, soft-estop, EMS 또는 안전보드 정지 비트가 새로 검출됨
- 정상 운전 중 robot-ready 상태가 사라짐
- 하드웨어 motion inhibit가 활성화됨
- `/motion_abort=true`가 발행됨

정상 운전 중에는 디스크에 고속 데이터를 쓰지 않는다. 사건 발생 시 직전 12초와 이후 3초를 한 타임라인으로 저장한다.

## 실행

`rb10_painting_system.launch.py`를 평소와 같이 실행하면 기록기가 기본으로 함께 실행된다. 별도 터미널은 필요하지 않다.

하드웨어 플러그인과 `SystemState` 메시지가 변경되었으므로 처음 사용할 때는 T1을 포함한 전체 로봇 스택을 새로 실행해야 한다.

시작 로그에서 다음 문구를 확인한다.

```text
Interlock flight recorder ready: pre=12.0s post=3.0s ... enabled=True
```

## 정지 후 확인

가능하면 정지 후 3초 동안 T1을 끄지 않는다. 그러면 정지 이후 상태까지 완전히 기록된다. 바로 종료하더라도 종료 시점까지 수집된 내용은 저장된다.

가장 최근 기록 위치:

```bash
cat ~/.ros/painting_interlock_records/LATEST.txt
```

최근 기록의 요약:

```bash
record_dir=$(cat ~/.ros/painting_interlock_records/LATEST.txt)
jq . "$record_dir/summary.json"
```

하드웨어가 인터록을 처음 본 정확한 샘플:

```bash
rg 'INTERLOCK_SNAPSHOT' "$record_dir/timeline.jsonl"
```

전체 시간축에서 트리거와 로봇 상태만 확인:

```bash
jq -c 'select(.source == "trigger" or .source == "system_state")' \
  "$record_dir/timeline.jsonl"
```

분석을 요청할 때는 `LATEST.txt`가 가리키는 디렉터리 경로만 알려주면 된다. 주요 파일은 다음 두 개다.

- `summary.json`: 최초 및 후속 트리거, 기록 범위, 데이터 종류별 개수
- `timeline.jsonl`: 모든 샘플의 사건 기준 상대 시간(`relative_to_trigger_s`)과 원본 값

## 인터록 원인 비트

하드웨어 스냅샷의 `reason_mask`와 기록기의 `decoded_reasons`는 같은 의미를 사용한다.

| 비트 | 이름 | 의미 |
|---:|---|---|
| 0 | `collision` | 외부 충돌 감지 |
| 1 | `self_collision` | 자체 충돌 감지 |
| 2 | `sos` | 로봇 암 장치 오류/SOS |
| 3 | `soft_estop` | 소프트 정지 |
| 4 | `ems` | 소프트웨어/기구학 emergency stop |
| 5 | `safety_board_sos` | 안전보드 SOS |
| 6 | `safety_ems2` | 안전보드 EMS2 |
| 7 | `safety_prs` | 안전보드 PRS |
| 8 | `safety_hss` | 안전보드 HSS |
| 9 | `safety_sss` | 안전보드 SSS |

여러 원인이 같은 제어 주기에 켜지면 모두 한 스냅샷에 남는다. 기존 boolean 필드와 달리 collision/self-collision의 SDK packed 원본 값도 보존한다.

## 기록 데이터

- Rainbow `SystemState`: 관절 reference/angle/error, 전류, 온도, joint info, TCP reference/actual, 원본 EFT, robot/task state
- 내부 안전 상태: collision/self-collision 원본 word, SOS, soft-estop, EMS, 초기화 상태, freedrive, information chunk 1–4, safety-board word
- ros2_control 하드웨어 최초 인터록 스냅샷: 위 상태와 ROS 관절 명령, trajectory/force/compliance 활성 상태, read-cycle 시간
- 힘: `ft_link` 및 TCP의 raw/filtered wrench
- 페인팅 상태: mode, execution/readiness/safety/guard status, 목표 힘, contact, force/compliance/trajectory 활성 상태
- WARN 이상의 `/rosout` 메시지와 `/motion_abort`

## 판독 원칙

1. `relative_to_trigger_s`가 가장 이른 이상을 최초 원인으로 본다.
2. `[INTERLOCK_SNAPSHOT]`의 `reason_mask`가 0이 아니면 Rainbow 상태가 직접 정지를 요구한 것이다.
3. `motion_abort`가 먼저이고 내부 비트가 뒤따르면 소프트웨어/힘 안전정지가 하드웨어 inhibit를 유발한 후속 연쇄일 가능성이 크다.
4. collision과 함께 관절 오차·전류·TCP 힘/토크가 증가했는지 비교해 물리 충돌, 추종 불량, 제어주기 지연을 구분한다.
5. `read_period_ms`가 비정상적으로 크면 servo-stream 지연과 내부 정지의 시간적 상관관계를 확인한다.

이 계측은 Rainbow SDK가 공개하는 내부 상태까지는 정확히 구분한다. 컨트롤러 펌웨어 내부의 비공개 충돌 판정 계산식이나 세부 임계값 자체는 노출되지 않으므로, 그런 경우에는 기록된 비트·관절전류·추종오차·EFT를 함께 사용해 원인을 좁힌다.

## 설정

기본값은 `painting_system_real.yaml`의 `interlock_flight_recorder` 항목에 있다.

- `enabled: true`
- `pretrigger_seconds: 12.0`
- `posttrigger_seconds: 3.0`
- `output_directory: ""` — 비어 있으면 `~/.ros/painting_interlock_records`

기록기만 끄려면 launch argument `enable_interlock_flight_recorder:=false`를 사용한다. 내부 하드웨어 스냅샷 로그는 기록기와 무관하게 계속 출력된다.
