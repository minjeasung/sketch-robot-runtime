# RB10 스케치 기반 어드미턴스 도장 시스템 — 갱신 구현 사양

문서 기준일: 2026-08-10  
기준 워크스페이스: `/home/Minjea/sketch_robot_ws`의 현재 작업 트리  
원문 요구사항: `docs/CODEX_RB10_PAINTING_SYSTEM_REVISION_PROMPT.md`

> **현재 판정: 실제 로봇 도장 No-Go**
>
> 이 문서는 현재 소스에 존재하는 구현과 2026-08-10에 확정된 자동 검증 결과를 설명한다. 전체 workspace Release build 11 packages와 `colcon test-result` 179 tests/0 errors/0 failures/9 skipped가 완료되었고, 정적 검사와 제한된 top fake launch smoke도 통과했다. 다만 실제 RB10/D405/AFT200, browser/rosbridge E2E와 원문 10개 fake-hardware 시나리오 전체는 미검증이다. 이 상태에서 `real_painting_enabled:=true`, `dry_run:=false`, `painting_force_enabled:=true` 조합으로 실제 도장을 승인해서는 안 된다.

기존 `FINAL_PAINTING_ROBOT_SYSTEM.md`에는 version 2, 0.220 m roller, 0.031 m 혼합 offset 등 현재 계약과 맞지 않는 설명이 남아 있다. 실제 수정본의 기준은 이 문서와 아래 source-of-truth 파일이다.

## 1. 상태 표기와 적용 범위

이 문서에서 사용하는 표현은 다음과 같다.

- **구현됨**: 현재 작업 트리의 소스와 설정에서 해당 로직을 확인했다는 뜻이다.
- **자동 검증 완료**: 아래 13절에 기록한 build, test, 정적 검사 또는 제한된 fake smoke 범위에서 결과가 확정되었다는 뜻이다.
- **통합 미검증**: 자동 test가 통과했더라도 전체 ROS graph의 해당 E2E 시나리오를 아직 재현하지 않았다는 뜻이다.
- **실기 미검증**: RB10, 실제 controller chain, D405, AFT200 및 실제 벽을 사용한 검증이 필요하다는 뜻이다.
- **Go**는 코드 존재나 unit test 파일 존재만으로 선언하지 않는다. 실기 commissioning 체크리스트까지 통과해야 한다.

지원 대상으로 삼는 환경은 평평하고 고정된 벽, 한 번의 D405 근접 평면 측정, TCP Y축 한 축만의 normal admittance, 선택된 사각 작업영역 안의 roller 도장이다. 곡면 추종, PAINT 중 카메라 기반 자세 갱신, 이동 중 평면 fusion은 지원 계약이 아니다.

## 2. 구현 source of truth

| 영역 | 기준 파일 |
|---|---|
| 최상위 운용 launch | `src/sketch_control/launch/rb10_painting_system.launch.py` |
| 공통 도장 설정 | `src/sketch_control/config/painting_system_real.yaml` |
| MoveIt/controller wrapper | `src/sketch_control/launch/rb10_moveit_full.launch.py` |
| 실제 perception wrapper | `src/sketch_control/launch/rb10_real_perception_sketch.launch.py` |
| perception 구성 | `src/sketch_control/launch/rb10_perception_sketch.launch.py` |
| D405 단일 capture 및 lifecycle | `src/sketch_control/sketch_control/d405_surface_refiner_node.py`, `plane_lifecycle.py` |
| 작업영역 ID와 Wall Front | `src/sketch_control/sketch_control/wall_projector_node.py` |
| Fill/containment/v3 생성/marker | `src/sketch_control/sketch_control/work_area_geometry.py`, `sketch_to_waypoints_node.py` |
| v3 schema/hash/자세 | `src/rbpodo_painting_control/rbpodo_painting_control/segment_path.py` |
| 실행 상태 머신/readiness/ACM | `src/sketch_control/sketch_control/moveit_executor.py`, `painting_execution.py` |
| requested wrench | `src/rbpodo_painting_control/rbpodo_painting_control/painting_wrench_reference_node.py` |
| guarded final wrench | `src/rbpodo_painting_control/rbpodo_painting_control/painting_wrench_guard_node.py`, `wrench_guard.py` |
| 6축 F/T monitor | `src/rbpodo_painting_control/rbpodo_painting_control/painting_force_monitor_node.py`, `force_safety.py` |
| controller-side safety overlay | `src/admittance_controller/` |
| TCP Y admittance profile | top launch 기준 `src/sketch_control/config/painting_system_real.yaml`; `painting_normal_y.yaml`은 호환 launch용 |
| roller 형상/선택 collision link | `src/eoat_description/urdf/rr_00a_b_eoat_no_camera.urdf.xacro`, `src/sketch_control/urdf/rbpodo_with_eoat.urdf.xacro` |
| 웹 운용 gate | `web/index.html`, `web/js/app.js`, `web/style.css` |

## 3. 최종 시스템 구조

```text
operator / web UI
  -> target 및 work-area 선택
  -> wall_projector: 매 선택마다 새 work_area_id
  -> D405 capture trigger
  -> d405_surface_refiner: 한 cloud의 ROI/RANSAC 결과
  -> accepted status(work_area_id, plane_generation_id)
  -> accepted generation의 다음 refined PoseStamped 1개
  -> sketch_to_waypoints: 선택영역 containment + canonical v3 segment
  -> plan_hash + RViz marker + 호환 PoseArray
  -> moveit_executor: ID/hash/schema/readiness 재검증
  -> SAFETY_APPROACH / CONTACT_SEARCH / force-state process
  -> FollowJointTrajectory / MoveIt Cartesian planning
  -> joint_trajectory_controller -> admittance_controller overlay
  -> rbpodo_hardware -> RB10
```

힘 명령 경로는 motion 경로와 별도의 fail-closed chain을 사용한다.

```text
moveit_executor mode / desired force / enable / heartbeat
  -> painting_wrench_reference
  -> /painting_admittance/requested_wrench_reference
  -> painting_wrench_guard            # final topic의 단일 publisher
  -> /admittance_controller/wrench_reference

AFT200 filtered/raw wrench + TF + controller status
  -> painting_force_monitor
  -> safety status / contact / latched abort
  -> guard와 executor interlock
```

## 4. 좌표 규약과 측정 roller geometry

### 4.1 좌표와 힘 부호

```text
surface normal n = 벽에서 자유 공간으로 향하는 outward normal
TCP +Y           = n
TCP -Y           = 벽을 누르는 방향
TCP +X           = roller 긴 축
segment row xyz  = 물리 표면점(surface_point)
target wrench    = [0, -force_n, 0] in TCP
```

### 4.2 형상 기준값

| 항목 | 현재 기준값 | 근거와 의미 |
|---|---:|---|
| roller usable length | **0.175 m** | roller52 STL에서 분리한 cylinder component의 X 길이; Fill margin 계산에 사용 |
| roller diameter | **0.052 m** | 변경 roller component의 Y/Z 직경 |
| roller radius / contact geometry | **0.026 m** | 표면점에서 roller 회전축 중심까지의 물리 반경 |
| roller component local center | `(0, -0.018, 0.07675) m` | EOAT CAD frame의 측정 component 중심 |
| TCP→roller axis normal projection | **0.24420 m along TCP -Y** | URDF macro mount와 roller component center로부터 얻은 normal 방향 거리 |
| TCP→outer contact distance | **0.27020 m** | `0.24420 + 0.02600`; contact 상태에서 표면부터 TCP까지의 normal 거리 |

`0.220 m`를 roller usable length로 사용하거나 `0.031 m`를 하나의 contact offset으로 사용하는 것은 현재 계약이 아니다. `0.031 m`은 과거에 roller radius와 5 mm planning clearance를 혼합한 값이므로 폐기한다.

segment row의 표면점이 `S`, outward normal이 `n`, row clearance가 `d`라면 구현의 위치 관계는 다음과 같다.

```text
roller_axis = S + n * (0.02600 + d)
TCP         = roller_axis + n * 0.24420
            = S + n * (0.27020 + d)
```

따라서 현재 기본값에서:

| 상태 | row `offset_m` | 표면→roller axis | 표면→TCP normal 거리 |
|---|---:|---:|---:|
| PAINT/contact | 0.000 m | 0.026 m | 0.27020 m |
| APPROACH_PRECONTACT / CONTACT_SEARCH 시작 | 0.005 m | 0.031 m | 0.27520 m |
| stroke 간 RETRACT/TRAVEL | 0.010 m | 0.036 m | 0.28020 m |
| SAFETY_APPROACH / FINAL_RETRACT | 0.080 m | 0.106 m | 0.35020 m |

이 계산은 소스/mesh/URDF 일치 기준이다. 조립 공차와 실제 roller 외경은 실물 계측으로 다시 확인해야 하며, 그 전에는 실기 검증 완료로 간주하지 않는다.

## 5. Version 3 segment 실행 계약

### 5.1 root와 row 의미

실도장 경로는 `version: 3`, 최종 frame `link0`, `point_semantics: surface_point`를 사용한다. 필수 identity와 geometry root는 다음과 같다.

```json
{
  "version": 3,
  "frame_id": "link0",
  "path_id": "...",
  "plan_hash": "...",
  "work_area_id": "...",
  "plane_generation_id": "...",
  "point_semantics": "surface_point",
  "contact_geometry_offset_m": 0.026,
  "precontact_clearance_m": 0.005,
  "travel_clearance_m": 0.010,
  "safety_approach_offset_m": 0.080,
  "final_retreat_offset_m": 0.080,
  "tcp_normal_axis": "+y",
  "preserve_orientation_continuity": true,
  "rows": []
}
```

v3 path는 최종 frame에서 hash한 뒤에는 transform하거나 재투영할 수 없다. version 1/2 parser 호환은 명시적 legacy/dry-run용이며, strict real validator는 거부한다.

### 5.2 canonical SHA-256

`plan_hash`는 root의 `plan_hash` 필드 자체를 제외한 최종 JSON에 대해 계산한다.

```text
UTF-8 JSON
+ object key lexical sort
+ compact separators
+ finite float fixed 9 decimal places
+ negative zero normalization
-> SHA-256 lowercase 64-hex
```

parser는 payload hash를 다시 계산한다. strict validator는 parse 후 객체가 hashed raw payload와 완전히 같은지도 재검증한다. UI, plan status, RViz namespace, executor가 같은 hash를 사용한다.

### 5.3 real 실행 fail-closed 조건

`real_painting_enabled=true`이거나 non-dry 실행이면 다음 중 하나만 어긋나도 PoseArray fallback 없이 실행을 거부한다.

- matching v3 segment 존재
- `path_id`와 waypoint identity 일치
- `plan_hash`와 backend가 수락한 plan status 일치
- `work_area_id`가 현재 선택과 일치
- `plane_generation_id`가 현재 accepted D405 generation과 일치
- root geometry/축/연속성 필드 유효
- normal/tangent가 finite, non-zero, 서로 평행하지 않음
- 첫 mode가 `APPROACH_PRECONTACT`, 마지막 mode가 `FINAL_RETRACT`
- `CONTACT_SEARCH`, `RAMP_UP`, `PAINT`, `RAMP_DOWN` 순서와 force interlock 유효
- PAINT row offset은 0, non-contact row force는 0
- final retreat row 자체가 0.080 m를 포함

호환 `PoseArray`는 최종 segment row의 동일 geometry로 만들어 시각화/legacy 인터페이스에만 남긴다. v3 process가 끝나면 executor가 별도의 synthetic Stage 4 후퇴를 중복 실행하지 않는다.

## 6. 작업영역, Fill, 자유 스케치와 marker

### 6.1 작업영역 identity

`wall_projector_node`는 새 작업영역 선택마다 UUID 기반 `work_area_id`를 `/painting_system/work_area_state` JSON으로 발행한다. 같은 사각형을 다시 선택해도 새 operator event이므로 새 ID가 생긴다. 새 선택은 이전 refined plane, plan, free-space 확인을 무효화한다.

### 6.2 Fill

Fill은 전체 Wall Front가 아니라 `/work_area_pixels`에서 얻은 선택 사각형만 사용한다. pure geometry helper가 다음을 수행한다.

- roller 긴 축이 수평이라는 기준으로 좌우에 `roller_length / 2` margin 적용
- 기본 usable length 0.175 m와 overlap 0.30 사용
- 생성한 vertical serpentine stroke를 사각형 내부에 유지
- 실제 작업영역 폭이 roller 길이보다 작거나 유효 stroke 높이가 없으면 명시적으로 거부
- `/painting_system/fill_preview_pixels`에 backend가 실제 생성한 pixel stroke를 발행

Wall Front overlay는 이 backend preview를 그리므로 UI가 별도 Fill 알고리즘을 복제하지 않는다.

### 6.3 자유 스케치 containment

자유 스케치는 clip하지 않는다.

1. wall-front pixel 사각형에서 2D containment 검사
2. surface로 bilinear projection
3. 선택된 TL/TR/BR/BL 3D quadrilateral에서 plane/edge tolerance를 포함한 containment 재검사
4. 벗어난 점이 있으면 segment를 만들지 않고 JSON diagnostic에 reason, count, 일부 index 발행
5. 이전 marker는 `DELETEALL`로 제거

### 6.4 RViz와 compatibility output

marker는 최종 v3 row의 `segment_waypoint_position()`에서 직접 만든다. mode별 색상, CONTACT_SEARCH 최대 거리 arrow, SAFETY_APPROACH line, 실제 row offset을 사용한다. namespace에는 전체 `plan_hash`와 `plane_generation_id`를 포함하고 새 plan 전에 `DELETEALL`을 발행한다.

`/sketch_waypoints` PoseArray도 같은 최종 row geometry와 orientation continuity 계산을 사용하지만, real 실행 권한의 source는 v3 segment다.

## 7. D405 single-capture plane lifecycle

### 7.1 capture 정책

```text
새 target/work area 또는 명시적 invalidate
  -> 새 work_area_id / plane_generation_id
  -> capture arm(timeout 포함)
  -> 다음 유효 PointCloud2 한 개를 원자적으로 consume
  -> ROI -> voxel -> RANSAC -> 품질 검증
  -> ACCEPTED 또는 REJECTED
  -> 첫 ACCEPTED generation lock
```

real profile은 `stable_samples=1`, `spatial_samples=1`, `max_spatial_samples=1`, `lock_after_refinement=true`다. 여러 cloud나 여러 위치의 결과를 평균/SVD fusion하지 않는다. 후보 pose는 첫 capture 또는 이동 실패 뒤의 재시도용일 뿐, 성공 뒤 추가 측정용이 아니다.

품질 status에는 ROI/voxel point 수, inlier 수/비율, RMS/max residual, ZED 대비 normal/plane 차이, capture stamp, source frame, TF age, ID와 rejection reason이 포함된다. threshold는 공통 설정 파일에 있다. 현재 수치는 초기값이며 실제 D405 dataset으로 승인된 최종 threshold가 아니다.

### 7.2 status-before-Pose barrier

공용 status topic은 `/perception/d405_surface_refinement_status`이고 target/work-area 상태를 함께 운반한다. `sketch_to_waypoints`, executor와 UI는 명시적인 `mode == "work_area"`만 수락한다. mode가 없거나 target인 payload는 work-area acceptance를 열지 않는다.

accept 시 refiner는 다음 순서를 사용한다.

1. `accepted=true`, `work_area_id`, `plane_generation_id`를 포함한 status 발행
2. consumer는 이전 refined Pose cache를 비우고 해당 generation의 “다음 Pose 1개”를 arm
3. matching refined `PoseStamped` 발행
4. consumer가 정확히 한 번 cache하고 arm consume
5. arm되지 않은 stale/extra Pose는 무시

이 순서는 transient-local 이전 Pose가 새 status 직후 잠깐 사용되는 race를 차단한다. `accepted`는 JSON boolean `true`여야 하며 문자열 truthy 값은 거부한다.

### 7.3 invalidation과 PAINT lock

- 새 target/work-area, explicit re-refine, calibration hash 변경은 새 generation을 만든다.
- 단순히 30초가 지났다는 이유로 accepted plane을 버리거나 ZED로 real fallback하지 않는다.
- real path 생성에는 현재 `work_area_id`와 일치하는 accepted D405 status, generation ID 및 그 다음 refined Pose가 필요하다.
- PAINT 중 invalidation은 pending으로 보관하고 현재 plane snapshot을 바꾸지 않는다.
- PAINT 종료 뒤 pending invalidation을 적용한다.
- generation이 바뀌면 plan/hash/marker를 다시 만들고 operator가 재확인해야 한다.

## 8. 실행 상태 머신과 collision contract

v3 공정의 논리 순서는 다음과 같다.

```text
IDLE
  -> SAFETY_APPROACH                  # root 0.080 m
  -> APPROACH_PRECONTACT             # row 0.005 m
  -> CONTACT_SEARCH                   # zero wrench, compliance off
  -> RAMP_UP
  -> PAINT
  -> RAMP_DOWN
  -> RETRACT / TRAVEL                # stroke 사이 0.010 m
  -> 다음 APPROACH_PRECONTACT ...
  -> FINAL_RETRACT                   # authoritative row 0.080 m
  -> COMPLETE / IDLE
```

CONTACT_SEARCH는 기본 0.0005 m step, 0.002 m/s, 최대 0.010 m, timeout 8.0 s의 bounded Cartesian 반복이다. 각 step 전에 F/T freshness, TF, abort, controller fault와 contact를 확인한다. contact가 없고 거리/시간 한계에 도달하면 `CONTACT_NOT_FOUND` fault다. APPROACH 중 조기 contact가 확인되면 추가 접근을 중단하고 과힘이 아닌 경우 search 완료로 넘길 수 있다.

RAMP_DOWN은 두 단계 handshake다. 먼저 force enable을 유지한 채 reference ramp가 정확히 0까지 내려간다. 그 feedback을 받은 executor가 `enable=false`를 발행하고, disable 요청 이후에 새로 수신된 guard status가 `forwarding=false`, `compliance_enabled=false`임을 확인한 뒤에만 RETRACT로 진행한다. 이전/stale guard status로는 완료하지 않는다.

의도된 roller-wall 접촉은 `paint_eoat_no_camera_roller_contact_link`와 현재 active surface object 사이의 ACM pair만 CONTACT_SEARCH/PAINT 동안 허용한다. 기존 ACM을 확보하지 못하면 partial matrix를 새로 만들어 허용하지 않고 실패한다. PAINT/SEARCH 종료, abort, reset에서 이전 pair 값을 복원한다. bracket, D405, AFT200, 다른 robot link와 wall collision은 허용 대상이 아니다.

Cartesian fraction, empty trajectory, action result, cancel/timeout과 중복 execute는 fail-closed로 처리한다. 센서 stale, TF 오류, controller 오류나 미확인 충돌에서는 blind automatic retreat를 하지 않는다. wrench/compliance를 끄고 trajectory를 취소하며 fault를 latch한다.

## 9. 힘 안전 계층

### 9.1 requested wrench

`painting_wrench_reference`는 executor의 mode/force/enable을 받아 TCP `force_y` 음의 방향 명령을 rate-limited ramp로 만든다. 이 노드는 최종 controller topic에 직접 publish하지 않으며 `/painting_admittance/requested_wrench_reference`만 사용한다. non-contact, dry-run, abort, disable에서는 zero다.

### 9.2 guarded final output

`painting_wrench_guard`만 `/admittance_controller/wrench_reference`에 publish한다. 다음이 모두 참일 때만 requested wrench와 compliance enable을 전달한다.

- real painting enable
- requested wrench/frame finite, fresh, cap 이내
- fresh mode가 force-capable (`RAMP_UP`, `PAINT`, zero로 내려가는 `RAMP_DOWN`)
- fresh force enable true
- fresh executor heartbeat true
- F/T 및 ft→tcp TF valid/fresh
- safety status valid/fresh, abort not latched
- controller status fresh, controller fault 없음
- compliance-active acknowledgement가 제한 시간 안에 확인됨

하나라도 실패하면 주기적으로 exact zero와 `compliance_enable=false`를 발행한다. shutdown 때도 제한된 횟수의 zero를 보내지만, 이를 단독 안전장치로 간주하지 않는다.

### 9.3 controller-side overlay

설치된 upstream Jazzy controller가 마지막 reference를 유지하는 문제를 보완하기 위해 workspace에 `admittance_controller` overlay source가 있다. 현재 소스에는 다음 확장이 구현되어 있다.

- steady-clock `wrench_reference_timeout_s`
- heartbeat-timed `compliance_enable_timeout_s`, default disabled
- requested/measured wrench finite check
- invalid hardware NaN을 valid zero로 바꾸지 않고 compliance 차단
- compliance off일 때 zero excitation과 spring/damping return
- TCP Y normal trim/velocity/acceleration clamp
- `compliance_active`, `normal_limit_reached` status

top launch가 사용하는 `painting_system_real.yaml`의 현재 초기값은 timeout 0.10 s, trim 0.005 m, velocity 0.010 m/s, acceleration 0.050 m/s²이며 Y translation만 selected다. 자동 검증에서는 overlay가 workspace `install` prefix에서 resolve되고 top fake launch에서 controller chain이 활성화되는 것을 확인했다. 실제 controller manager/RB10에서의 RT timeout, disable, drift 및 limit 동작은 여전히 실기 미검증이다. `painting_normal_y.yaml`은 최상위 launch를 사용하지 않는 호환 경로용 profile이다.

### 9.4 6축 safety monitor

monitor는 filtered path와 raw fast path를 분리해 다음을 mode별로 감시한다.

- `|Fx|, |Fy|, |Fz|`, force norm
- `|Tx|, |Ty|, |Tz|`, torque norm
- force/torque derivative
- raw impact, saturation, non-finite
- filtered/raw message age, TF validity/age
- non-contact unexpected contact
- controller fault와 지속 normal-limit

abort reason은 latch되며 explicit reset 전 자동 해제되지 않는다. reset과 adaptive bias에는 IDLE, force disabled, trajectory inactive, stationary, explicit free-space confirmation, fresh/low wrench 등의 interlock가 적용된다. 설정 threshold는 commissioning 전 초기 안전값이지 실기 인증값이 아니다.

## 10. Readiness와 웹 UI gate

executor는 `/painting_system/readiness` JSON에 개별 check, blocker, current IDs/hash/state를 발행한다. real mode의 `ready=true`에는 최소 다음이 모두 필요하다.

- fresh joint state와 required controller active
- FollowJointTrajectory action available
- ZED surface 및 accepted D405 work-area plane
- TCP/F/T TF와 F/T freshness
- wrench guard freshness, controller fault clear
- force 사용 시 bias ready와 free-space confirmed
- abort not latched
- matching current v3 plan validated

웹 UI는 D405 status, `/painting_system/plan_status`, readiness, execution status를 함께 사용한다. 다음 조건을 만족하지 않으면 Path/Fill/Run을 비활성화한다.

- ROS 연결, target/work-area 선택
- `mode=work_area`의 accepted D405와 완전한 IDs
- plan의 path/hash/work-area/generation identity 일치
- backend plan validation과 readiness true
- 유효한 target force, 실행 중 아님, abort 없음

work-area/generation/abort/run transition에서 free-space 확인을 지운다. Run 확인창에는 plan hash 앞 8자리, target force, work-area ID, plane generation ID, path ID가 표시된다. UI는 안전의 유일한 gate가 아니며 backend가 같은 조건을 다시 검증한다.

현재 소스와 JavaScript 문법 검사는 확인했지만 실제 browser/rosbridge와 ROS graph를 연결한 E2E 동작은 미실행이다.

## 11. 공통 YAML과 최상위 launch

`painting_system_real.yaml`은 다음 노드와 controller에 같은 파일로 전달되는 도장 시스템 설정 진입점이다.

- `moveit_executor`
- `sketch_to_waypoints`
- `wall_projector`
- `d405_surface_refiner`
- `environment_scanner`
- `painting_wrench_reference`
- `painting_wrench_guard`
- `painting_force_monitor`
- `admittance_controller` through `ros2_control_node`

기본 interlock은 다음과 같다.

```yaml
real_painting_enabled: false
dry_run: true
painting_force_enabled / enable_force: false
```

최상위 `rb10_painting_system.launch.py`는 MoveIt/controller, 실제 perception wrapper, 세 force node, executor를 묶고 같은 config path와 enable 인자를 전달한다. D405 real profile은 single-capture와 lock을 강제한다. MoveIt child에는 빈 `admittance_profile`과 공통 config path를 넘기며, `rb10_moveit_full.launch.py`가 이 파일을 `ros2_control_node` parameter 목록의 마지막에 append한다. 따라서 top launch에서는 공통 YAML의 `admittance_controller` section이 authoritative final override다. 기존 `painting_normal_y.yaml`은 top launch를 사용하지 않는 이전 wrapper용 호환 profile이다.

## 12. 현재 실행·검증 절차

### 12.1 launch argument 확인

아래 argument/import 확인은 source된 workspace overlay 환경에서 성공했다.

```bash
cd ~/sketch_robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sketch_control rb10_painting_system.launch.py --show-args
```

### 12.2 fake/dry-run 기동 예시

실제 장치 연결을 피하는 검증 환경에서만 다음과 같이 사용한다. perception topic을 별도 fixture/simulator가 공급해야 경로 readiness를 검증할 수 있다.

```bash
ros2 launch sketch_control rb10_painting_system.launch.py \
  use_fake_hardware:=true \
  launch_zed_driver:=false \
  launch_d405_driver:=false \
  launch_rbpodo_eft_bridge:=false \
  launch_aft_ethernet_driver:=false \
  real_painting_enabled:=false \
  dry_run:=true \
  painting_force_enabled:=false
```

기본 top launch의 `use_fake_hardware`는 false이므로, 단순히 인자를 생략한 launch를 “하드웨어와 무관한 안전 smoke test”로 오해하면 안 된다. force interlock가 off여도 실제 driver/controller 연결 범위는 별도로 확인한다.

확정된 제한적 smoke에서는 `use_fake_hardware:=true`가 `fake_sensor_commands:=true`를 자동 적용했고 실제 robot 연결을 시도하지 않았다. core node가 기동되고 controller 5개가 active가 된 뒤, fake F/T가 주입되지 않은 조건에서 시스템은 `FT_STALE`/`ABORT`와 guarded wrench exact zero로 fail-closed 상태를 유지했다. 이는 F/T stale 음성 경로의 증빙이며, contact/paint 정상 경로나 아래 10개 fake 시나리오 전체 통과를 뜻하지 않는다.

### 12.3 실제 도장 인자

실제 도장에 필요한 조합은 구조상 다음 세 gate를 모두 명시해야 한다.

```text
real_painting_enabled=true
dry_run=false
painting_force_enabled=true
```

그러나 현재 판정은 No-Go이므로 실제 장치에서 이 조합을 실행하는 명령은 승인된 운용 절차가 아니다. 아래 14절을 모두 완료한 뒤 별도 작업허가와 현장 안전 절차로 전환한다.

## 13. 자동 검증 명령과 현재 결과

```bash
cd ~/sketch_robot_ws
source /opt/ros/jazzy/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
  source ~/ros2_ws/install/setup.bash
fi

python3 -m compileall src/sketch_control src/rbpodo_painting_control

PYTHONPATH=src/sketch_control:src/rbpodo_painting_control \
  python3 -m pytest -q \
  src/sketch_control/test \
  src/rbpodo_painting_control/test

colcon build --symlink-install \
  --allow-overriding admittance_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose

ros2 launch sketch_control rb10_painting_system.launch.py --show-args
```

| 검증 항목 | 현재 문서 상태 |
|---|---|
| Python `compileall` | 통과 |
| JavaScript `node --check` | 통과 |
| 변경 내용 `diff --check` | 통과 |
| xacro 생성 및 `check_urdf` | 통과 |
| 관련 unit/gtest/package tests | 전체 colcon test 집계에 포함, error/failure 0 |
| 전체 workspace Release build | **11 packages 성공** |
| 전체 `colcon test-result --verbose` | **179 tests, 0 errors, 0 failures, 9 skipped**; skip은 `cppcheck 2.13`의 알려진 self-skip |
| workspace overlay prefix | `admittance_controller`가 workspace `install` prefix에서 resolve됨 |
| top launch import/`--show-args` | 통과 |
| top fake launch smoke | core node 기동, controller 5개 active, robot 미연결, F/T 미주입 시 `FT_STALE`/`ABORT` 및 exact-zero 확인 |
| fake-hardware 10개 launch 시나리오 | 제한적 stale 음성 경로만 확인; 전체 자동 E2E 미완료 |
| browser/rosbridge UI E2E | 미실행 |
| 실제 RB10/D405/AFT200 | 미실행 |

## 14. 실로봇 Go 전 필수 commissioning

자동화 단계에서 다음 두 항목은 완료했다.

1. 전체 workspace Release build/test 결과 보존: 11 packages, 179 tests/0 errors/0 failures/9 skipped
2. workspace `install` prefix에서 `admittance_controller` overlay resolution과 top fake controller 활성화 확인

실제 도장 Go 전에는 다음 잔여 항목을 순서대로 수행한다.

1. fake hardware에서 matching segment 부재, ID/hash mismatch, plane invalidation 시 goal 0건 확인
2. fake hardware에서 CONTACT_SEARCH contact/no-contact, heartbeat/request/F/T stale, plane change 시 abort/zero 전체 시나리오 확인
3. UI/rosbridge에서 Path/Run gate와 confirmation identity E2E 확인
4. 실제 장치 전원을 넣기 전 TF tree, joint limits, controller chain, emergency stop 확인
5. D405 한 capture의 pointcloud 품질과 calibration/TF age를 실제 평판에서 검증
6. 조립된 roller usable length, radius, TCP→axis 및 TCP→contact를 실물 계측
7. 벽에서 떨어진 free-space에서 force sign, tare/bias, stale/enable/heartbeat 각각 차단 시 guarded zero 확인
8. controller-side compliance off에서 drift가 누적되지 않는지 확인
9. 낮은 속도·짧은 거리의 기준판 CONTACT_SEARCH, 조기 contact/no-contact/over-force/torque 시험
10. selective roller-surface ACM 허용/복원과 다른 EOAT link collision 유지 확인
11. 작은 작업영역의 connector 없는 1-stroke, PAINT plane lock, RAMP_DOWN 후 FINAL_RETRACT 확인
12. threshold와 admittance parameter를 측정 결과로 승인하고 변경 이력을 기록

## 15. 남은 blocker와 최종 판정

### P0 — 실제 도장 승인 전 해소 필수

- 실제 AFT200 부호, tare/bias, 6축 threshold, raw impact와 abort/cancel 동작이 검증되지 않았다.
- D405 calibration/TF/single-capture 품질 threshold와 first-accepted lock을 실제 camera/평판에서 검증하지 않았다.
- RB10에서 TCP→roller geometry, contact search 거리, selective ACM 및 실제 controller/overlay RT 동작을 검증하지 않았다.
- top fake launch 기동과 F/T stale fail-closed 경로는 확인했지만, 원문 10개 fake-hardware launch E2E 전체와 실제 browser/rosbridge gate는 미검증이다.

### P1 — 통합 품질 및 운용 유지보수

- top launch 밖의 호환 profile/legacy wrapper가 공통 YAML과 달라지지 않도록 회귀 검사를 유지해야 한다.
- D405/RANSAC, force envelope, admittance mass/damping/stiffness는 실제 dataset과 commissioning 결과로 재조정해야 한다.
- logging topic을 rosbag으로 수집하고 plan/hash/state/contact/guarded wrench/action result를 한 run 단위로 재현하는 절차를 확정해야 한다.

**최종 판정: No-Go.** 자동 build/test와 제한된 fake stale 경로는 통과했지만, 이는 실제 로봇 안전 검증을 대신하지 않는다. 남은 P0 항목과 hardware commissioning이 완료될 때까지 실제 force 도장은 승인하지 않는다.
