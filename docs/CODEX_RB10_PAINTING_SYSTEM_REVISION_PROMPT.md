# Codex 작업 지시서: RB10 스케치 기반 어드미턴스 도장 로봇 수정

## 0. 작업 목적

`~/sketch_robot_ws`의 현재 구현을 직접 점검하고 수정하여, RB10 롤러 도장 시스템을 다음 원칙에 맞는 **단일하고 일관된 실로봇 운용 경로**로 정리한다.

- ZED는 대상 선택과 전역 초기 평면을 제공한다.
- D405는 작업영역을 가까이에서 **한 번 측정**하여 최종 작업 평면을 보정한다.
- D405 평면은 도장 전에 확정하고, PAINT 중에는 업데이트하지 않는다.
- 접촉 의도는 F/T 임계값이 아니라 segment mode가 결정한다.
- MoveIt/JTC는 명목 경로를, admittance controller는 TCP Y 법선 방향 순응을 담당한다.
- 선택 작업영역, UI 미리보기, RViz marker, 최종 segment, 실제 실행 경로가 완전히 일치해야 한다.
- 경로, 평면, F/T, TF, controller 상태가 불완전하거나 오래되었으면 실제 로봇은 움직이지 않아야 한다.

이 작업은 단순 분석이나 개선안 제안으로 끝내지 말고, **실제 코드 수정, 설정 통합, 빌드, 테스트, 문서 갱신까지 수행**한다.

---

## 1. 작업 대상과 기준 파일

작업 workspace:

```text
~/sketch_robot_ws
```

우선 다음 파일과 실제 구현을 읽고 현재 동작을 확인한다.

```text
src/sketch_control/launch/rb10_moveit_full.launch.py
src/sketch_control/launch/rb10_real_perception_sketch.launch.py
src/sketch_control/launch/rb10_perception_sketch.launch.py
src/sketch_control/sketch_control/target_selector_node.py
src/sketch_control/sketch_control/environment_scanner_node.py
src/sketch_control/sketch_control/d405_surface_refiner_node.py
src/sketch_control/sketch_control/wall_projector_node.py
src/sketch_control/sketch_control/sketch_to_waypoints_node.py
src/sketch_control/sketch_control/moveit_executor.py
src/rbpodo_painting_control/rbpodo_painting_control/segment_path.py
src/rbpodo_painting_control/rbpodo_painting_control/painting_wrench_reference_node.py
src/rbpodo_painting_control/rbpodo_painting_control/painting_force_monitor_node.py
src/rbpodo_painting_control/launch/painting_admittance_control.launch.py
src/rbpodo_ros2/rbpodo_bringup/config/controllers_admittance.yaml
src/rbpodo_ros2/rbpodo_bringup/config/admittance_profiles/painting_normal_y.yaml
src/rbpodo_ros2/rbpodo_hardware/src/rbpodo_hardware_interface.cpp
web/index.html
web/js/app.js
web/style.css
zed_d405_apriltag_calibration.json
d405_eyeinhand_charuco_calibration.json
aft200_force_threshold.json
```

현재 시스템 설명서가 workspace에 있다면 함께 읽는다.

```text
FINAL_PAINTING_ROBOT_SYSTEM.md
```

파일이 없더라도 이 작업 지시서의 결정사항을 우선 적용한다.

### 작업 전 필수 확인

1. `git status --short`로 기존 사용자 변경사항을 확인한다.
2. 사용자 변경사항을 삭제하거나 되돌리지 않는다.
3. `build/`, `install/`, `log/` 아래 생성 파일은 직접 수정하지 않는다.
4. 아래 키워드의 실제 정의 위치를 모두 찾는다.

```bash
rg -n "contact_offset|travel_clearance|lock_after_refinement|D405_PREFLIGHT|path_id|version.?2|fill_work_area|legacy_noncontact_lift|wrench_reference|contact_detect|over_force|stale_timeout|plane.*fresh|30.0" \
  ~/sketch_robot_ws/src ~/sketch_robot_ws/web
```

5. 문서와 코드가 다르면 코드의 현재 동작을 먼저 기록한 뒤, 이 지시서에 맞게 수정한다.

---

## 2. 확정된 설계 결정: 임의로 변경하지 말 것

### 2.1 D405 평면 측정 정책

도장 대상은 충분히 평평하고 고정된 벽이라고 가정한다. 따라서 D405는 다음과 같이 사용한다.

```text
작업영역 선택
  -> D405 촬영 pose로 이동
  -> 로봇 정지 및 settle
  -> 한 번의 capture trigger
  -> 한 개 PointCloud2에서 ROI/RANSAC 평면 추정
  -> 유효하면 즉시 최종 평면으로 채택
  -> PAINT 종료 또는 새 작업영역 선택 전까지 lock
```

반드시 지킬 조건:

- 여러 위치의 D405 결과를 평균하지 않는다.
- 여러 평면을 SVD로 융합하지 않는다.
- 연속 이동 중 법선을 실시간 갱신하지 않는다.
- PAINT 중 카메라 결과로 TCP orientation이나 경로를 수정하지 않는다.
- 첫 후보 pose의 capture가 실패한 경우에만 다음 후보 pose를 시도할 수 있다.
- 여러 후보 pose는 **다중 측정용이 아니라 실패 시 재시도용**이다.
- 첫 유효 capture가 최종 결과다.
- 실도장 모드에서는 현재 작업영역에 연결된 유효한 D405 refined plane이 없으면 실행하지 않는다.
- ZED-only fallback은 preview/dry-run에서만 허용한다.

### 2.2 좌표 및 부호 규약

기존 규약을 유지한다.

```text
surface normal n = 표면에서 자유 공간으로 향하는 outward normal
TCP +Y           = outward normal
TCP -Y           = 표면을 누르는 방향
TCP +X           = 롤러 긴 축
segment point     = 실제 표면점 surface_point
```

목표 접촉력은 TCP 기준으로 다음과 같다.

```text
target_wrench_tcp.force = [0, -force_n, 0]
```

### 2.3 제어 구조

다음 구조를 유지한다.

```text
FollowJointTrajectory
  -> joint_trajectory_controller
  -> admittance_controller
  -> rbpodo_hardware
  -> RB10
```

- 접선 경로와 자세: MoveIt/JTC
- 법선 순응: TCP Y축 admittance
- 접촉/비접촉 의도: segment mode
- F/T: 접촉 확인과 안전 판단

### 2.4 운용 상태

최종 상태 흐름은 다음을 사용한다.

```text
IDLE
  -> SAFETY_APPROACH
  -> APPROACH_PRECONTACT
  -> CONTACT_SEARCH
  -> RAMP_UP
  -> PAINT
  -> RAMP_DOWN
  -> RETRACT
  -> TRAVEL
  -> ...
  -> FINAL_RETRACT
  -> IDLE
```

알 수 없는 충돌, 센서 stale, TF 오류, controller 오류에서는 자동 후퇴하지 않는다. 해당 경우는 목표 wrench를 0으로 만들고 trajectory를 취소한 뒤 fault를 latch한다.

---

## 3. 구현 우선순위

다음 순서대로 구현한다.

1. 경로 계약과 작업영역 일치
2. D405 single-capture plane lifecycle 통일
3. contact geometry 분리와 CONTACT_SEARCH 추가
4. force command fail-closed 및 controller timeout 확인
5. F/T 안전 envelope 확장
6. 단일 설정 파일과 상위 launch 구성
7. 자동 테스트 및 문서 갱신

한 단계의 핵심 테스트가 실패한 상태에서 다음 단계가 정상이라고 가정하지 않는다.

---

# 4. 필수 수정 사항

## A. Segment 경로를 fail-closed로 변경

### A-1. 실도장에서는 matching segment 없이는 절대 실행하지 않기

현재 `moveit_executor`에 일반 `PoseArray` fallback이 남아 있다면 다음처럼 바꾼다.

```text
real_painting_enabled=true
AND use_eoat_segments=true
```

인 경우:

- matching segment가 없으면 실행 거부
- segment schema version이 맞지 않으면 실행 거부
- `path_id`가 waypoint와 다르면 실행 거부
- plane generation이 다르면 실행 거부
- work area ID가 다르면 실행 거부
- plan hash가 다르면 실행 거부
- 빈 경로, partial path, invalid normal/tangent이면 실행 거부
- 어떠한 경우에도 PoseArray-only motion으로 내려가지 않음

PoseArray fallback은 visualization 또는 명시적인 dry-run mode에서만 허용한다.

### A-2. Segment schema 갱신

현재 version 2의 `contact_offset_m`은 물리 형상과 planning clearance를 혼합하므로 새 schema로 명확히 분리한다. 실도장용 schema를 version 3으로 올리는 것을 권장한다.

권장 root 필드:

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
  "tcp_normal_axis": "+y",
  "preserve_orientation_continuity": true,
  "rows": []
}
```

주의:

- `contact_geometry_offset_m=0.026`은 TCP 또는 roller center가 실제로 롤러 중심에 정의되어 있을 때만 맞다.
- URDF/CAD/TF를 확인하여 실제 TCP에서 롤러 외주 접촉점까지의 거리를 계산하고, 확인 결과를 설정 파일과 구현 보고서에 기록한다.
- 5 mm MoveIt 여유를 물리 contact offset에 더하지 않는다.
- 구 version 2는 dry-run/legacy parser에서 읽을 수 있어도 실도장에서는 거부한다.

### A-3. Plan hash

최종 실행에 쓰는 canonical segment JSON을 정렬된 key와 고정된 수치 정밀도로 직렬화한 뒤 SHA-256 hash를 만든다.

```text
plan_hash = SHA256(canonical_segment_json)
```

- marker를 만들 때 사용한 plan hash
- executor가 검증한 plan hash
- UI가 표시한 plan hash

가 모두 같아야 한다.

---

## B. 선택 작업영역, Fill, 자유 스케치, marker를 일치시키기

### B-1. 자동 Fill 범위 수정

`sketch_to_waypoints_node._on_fill_work_area()`가 전체 wall front를 사용하지 않도록 수정한다.

자동 Fill은 반드시 사용자가 선택한 작업영역 사각형 안에서만 생성한다.

```text
/work_area_pixels
/perception/work_area_corners
현재 wall_front_extent
```

를 같은 generation으로 연결하여 사용한다.

요구사항:

- Fill stroke의 모든 픽셀은 선택 사각형 내부
- 롤러 길이와 overlap을 고려한 margin 적용
- 작업영역 크기가 롤러보다 작거나 유효 stroke를 만들 수 없으면 명확한 오류로 거부
- UI preview와 backend가 동일한 수식과 파라미터를 사용

가능하면 Fill geometry 계산을 하나의 공통 Python 모듈로 만들고, UI는 backend가 발행한 preview를 표시하도록 한다. UI와 backend에 서로 다른 알고리즘을 중복 구현하지 않는다.

### B-2. 자유 스케치 영역 검증

자유 스케치 점이 작업영역을 벗어나면 자동 clip하지 말고 실행 경로 생성을 거부한다.

- 2D pixel point-in-rectangle 검사
- 3D surface point가 work-area quadrilateral 내부인지 재검사
- 작은 수치 오차만 허용하는 tolerance 파라미터 제공
- 오류 사유와 벗어난 점 개수를 UI/diagnostic topic에 발행

### B-3. RViz marker를 최종 segment에서 직접 생성

기존 `/sketch_waypoints` 기반 legacy connector marker와 실제 segment clearance가 다르면 제거하거나 legacy로 명확히 구분한다.

새 marker는 version 3 segment row에서 직접 생성한다.

권장 표현:

```text
PAINT          : 실제 surface/contact geometry 위치
APPROACH       : precontact 위치
CONTACT_SEARCH : 탐색 방향과 최대 거리
RETRACT/TRAVEL : 실제 offset_m 반영
```

- mode별 색상 구분
- marker namespace에 `plan_hash`와 `plane_generation_id` 포함
- RViz에서 보이는 높이와 실제 실행 높이가 동일
- 새 plan이 생성되면 이전 marker 삭제

---

## C. D405 single-capture 평면 정책을 코드 전체에서 통일

### C-1. 다중 fusion 비활성화

`d405_surface_refiner_node.py`에 기존 spatial sample, SVD fusion, multi-sample lock 코드가 남아 있더라도 최종 real profile에서는 사용하지 않는다.

코드를 다음 두 층으로 명확히 분리한다.

```text
capture_once_and_fit_plane()
validate_single_plane_result()
```

최종 동작:

1. capture trigger 수신
2. 다음 유효한 D405 PointCloud2 한 개 사용
3. ROI 추출
4. voxel downsample
5. RANSAC
6. inlier 및 품질 검증
7. 즉시 ACCEPTED 또는 REJECTED

### C-2. single capture 품질 지표 추가

한 번만 측정하는 대신 결과 품질을 명확히 기록한다.

최소 출력 항목:

```text
roi_point_count
inlier_count
inlier_ratio
rms_residual_m
max_residual_m
normal_delta_deg_from_zed
plane_shift_m_from_zed
capture_stamp
source_frame
transform_age_s
work_area_id
plane_generation_id
accepted / rejected
rejection_reason
```

현재 RANSAC 설정값은 설정 파일에서 관리한다. 임계값을 코드에 중복 하드코딩하지 않는다.

초기 real profile은 기존 값보다 오검출을 쉽게 통과시키지 않도록 별도 항목을 둔다. 단, 실제 센서 데이터 없이 임의로 최종 임계값을 확정하지 않는다. 다음 값을 모두 파라미터화한다.

```text
min_roi_points
min_inliers
min_inlier_ratio
ransac_distance_threshold_m
max_rms_residual_m
max_normal_delta_deg
max_plane_shift_m
max_tf_age_s
capture_timeout_s
```

### C-3. 후보 pose의 의미 정리

executor의 최대 5개 prescan pose는 다음 목적으로만 사용한다.

```text
첫 pose 이동 실패 또는 capture reject
  -> 다음 후보 pose 시도
```

- 성공한 뒤 다른 pose를 추가 측정하지 않음
- 결과를 평균하지 않음
- 첫 accepted capture를 lock
- 후보 pose별 실패 사유 기록

### C-4. Plane generation ID

다음 이벤트에서 새 `work_area_id` 또는 `plane_generation_id`를 만든다.

```text
새 target 선택
새 work area 선택
명시적 re-refine 요청
캘리브레이션 파일 변경을 감지한 경우
```

D405 refined plane은 다음 조건을 만족해야 현재 작업영역에서 유효하다.

```text
refined.work_area_id == active_work_area_id
refined.accepted == true
TF valid
capture timestamp valid
```

기존 30초 freshness timeout처럼 노드마다 다른 시간 기준으로 ZED로 되돌아가지 않는다.

유효성은 시간만으로 자동 만료시키지 말고, 위의 명시적 invalidation event로 관리한다. 필요하면 별도의 `max_plane_age_s`를 real profile에서 매우 길게 두되 모든 노드가 동일한 값을 사용한다.

### C-5. 실행 전 정책

Path 입력 단계는 current D405 refined plane이 ACCEPTED 된 뒤에만 활성화한다.

`Run Robot` 시:

- segment의 `plane_generation_id`가 현재 plane과 같아야 함
- 다르면 실행하지 않고 `PLAN_REGEN_REQUIRED` 상태 발행
- 실행 직전에 몰래 재투영하고 바로 움직이지 않음
- plane이 바뀌었다면 path와 marker를 다시 생성하고 작업자가 다시 확인해야 함

### C-6. PAINT 중 plane lock

PAINT가 시작되면:

- ZED/D405 plane update를 현재 plan에 반영하지 않음
- 카메라 topic은 diagnostic 용도로만 사용할 수 있음
- 실시간 normal update, orientation update, path projection 금지
- 새 work area가 선택되기 전까지 current plane을 유지

---

## D. 물리 contact geometry와 planning clearance 분리

### D-1. 파라미터 분리

기존 `contact_offset_m=0.031`을 다음 항목으로 분리한다.

```text
contact_geometry_offset_m
precontact_clearance_m
travel_clearance_m
safety_approach_offset_m
final_retreat_offset_m
```

초기값 예시:

```text
contact_geometry_offset_m = 실제 URDF/CAD에서 검증한 값
precontact_clearance_m     = 0.005
travel_clearance_m         = 0.010
safety_approach_offset_m   = 0.080
final_retreat_offset_m     = 0.080
```

pose 계산:

```text
contact_center = surface_point + normal * contact_geometry_offset_m
precontact     = contact_center + normal * precontact_clearance_m
travel_pose    = contact_center + normal * travel_clearance_m
```

### D-2. CONTACT_SEARCH 상태 추가

현재 `APPROACH -> RAMP_UP`을 다음처럼 변경한다.

```text
APPROACH_PRECONTACT
  -> CONTACT_SEARCH
  -> RAMP_UP
  -> PAINT
```

CONTACT_SEARCH 요구사항:

- 목표 wrench reference는 0
- force command enable은 false
- 표면 방향 `-normal`로만 이동
- 저속
- 최대 추가 이동거리 제한
- 최대 시간 제한
- contact confirmed 시 즉시 다음 상태로 전환
- over-force, F/T stale, TF invalid, controller error 시 fault
- contact 없이 최대 거리/시간 도달 시 `CONTACT_NOT_FOUND` fault

권장 구현은 큰 하나의 trajectory를 보내고 늦게 cancel하는 방식보다 작은 Cartesian step을 반복하는 방식이다.

초기 파라미터 예시:

```text
contact_search_step_m          = 0.0005 또는 0.001
contact_search_speed_mps       = 0.002
contact_search_max_distance_m  = 0.010
contact_search_timeout_s       = 8.0
```

실제 값은 설정 파일로 관리한다.

각 step마다:

1. 현재 fault 확인
2. contact 상태 확인
3. 다음 작은 Cartesian 목표 생성
4. collision/IK/FJT 성공 확인
5. 누적 이동거리 기록

### D-3. APPROACH 조기 접촉

APPROACH_PRECONTACT 중 예상보다 먼저 contact가 검출되면:

- 현재 trajectory cancel
- 추가 접근 금지
- 과힘이 아니고 센서가 정상이라면 CONTACT_SEARCH 완료로 처리하고 RAMP_UP으로 전환 가능
- 과힘 또는 알 수 없는 충돌이면 fault latch

### D-4. 의도된 roller-wall contact의 collision 처리

MoveIt planning scene에서 작업면이 collision object라면 의도된 접촉을 위해 다음을 구현한다.

- active work surface object ID를 명시적으로 관리
- roller의 실제 접촉 collision link를 명시
- CONTACT_SEARCH와 PAINT에서만 해당 roller link와 active surface 사이 collision을 허용
- 다른 robot link, D405, AFT200, bracket과 wall collision은 계속 금지
- RETRACT/TRAVEL/IDLE에서 allowed collision을 원복
- 상태 전환 실패 시 원복 보장

전체 wall collision을 끄거나 robot 전체와 wall을 allowed collision로 만들지 않는다.

---

## E. 힘 명령을 fail-closed로 만들기

### E-1. 단일 guarded output

`/admittance_controller/wrench_reference`에는 하나의 노드만 publish하도록 정리한다.

권장 구조:

```text
moveit_executor / state machine
  -> desired force, mode, enable, heartbeat
  -> painting_wrench_reference_node 또는 command node
  -> /painting_admittance/requested_wrench_reference
  -> painting_wrench_guard_node
  -> /admittance_controller/wrench_reference
```

`painting_wrench_guard_node`가 최종 controller topic의 유일한 publisher다.

### E-2. Guard 허용 조건

다음 조건이 모두 참일 때만 비영 wrench를 전달한다.

```text
real_painting_enabled
requested_wrench_fresh
mode_fresh
mode in {RAMP_UP, PAINT}
force_enable_fresh_and_true
executor_heartbeat_fresh
F/T valid and fresh
TF ft_link -> tcp valid and fresh
no abort latch
no controller fault
requested force within cap
```

하나라도 거짓이면 다음을 고정 주기로 publish한다.

```text
zero WrenchStamped
```

필수 파라미터:

```text
publish_rate_hz
requested_wrench_timeout_s
mode_timeout_s
enable_timeout_s
executor_heartbeat_timeout_s
ft_timeout_s
max_command_force_n
```

종료 signal을 받을 때도 zero를 여러 번 publish하되, 이것만을 안전 보장으로 간주하지 않는다.

### E-3. Controller 자체 reference timeout 확인

현재 설치된 Jazzy `admittance_controller`가 reference timeout 또는 stale reference zeroing 기능을 지원하는지 실제 source와 parameter를 확인한다.

1. 지원하면 real profile에서 반드시 활성화한다.
2. 지원하지 않으면 `/opt/ros`를 직접 수정하지 않는다.
3. 같은 버전의 `admittance_controller`를 workspace overlay로 가져와 최소 patch를 적용하는 방안을 검토한다.
4. 최소 patch에는 다음이 포함되어야 한다.

```text
wrench_reference_timeout_s
stale reference -> zero reference
invalid/non-finite F/T -> compliance motion 차단
optional compliance_enable interface/topic
normal admittance displacement clamp
normal admittance velocity clamp
```

5. overlay controller를 빌드·테스트하지 못하면 실 force를 기본 비활성 상태로 유지하고 구현 보고서에 P0 blocker로 기록한다.
6. 지원 여부를 추측하지 말고 실제 설치 버전에서 확인한다.

### E-4. 비접촉 mode에서 compliance motion 차단

TRAVEL, RETRACT, APPROACH_PRECONTACT, IDLE에서는 목표 wrench만 0으로 만드는 것으로 끝내지 않는다.

가능한 controller-side 구조를 사용하여:

- measured wrench가 admittance motion을 만들지 않도록 gate
- F/T는 collision monitoring에는 계속 사용
- 기존 admittance offset은 급격한 jump 없이 0으로 복귀
- 비접촉 상태에서 normal drift가 누적되지 않음

이를 controller plugin에서 구현하기 어렵다면 real force enable을 차단하고 unresolved blocker로 명시한다.

### E-5. 명시적인 admittance 한계

파라미터화하여 적용한다.

```text
max_normal_trim_m
max_normal_velocity_mps
max_normal_acceleration_mps2
max_force_slew_rate_nps
```

초기 안전 profile 예시는 보수적으로 두되 최종 수치는 실험으로 조정한다.

```text
max_normal_trim_m       = 0.005
max_normal_velocity_mps = 0.010
```

한계 도달 시 silent saturation만 하지 말고 diagnostic 상태를 발행하고, 지속되면 abort한다.

---

## F. F/T 안전 monitor 확장

### F-1. normal force 외 전체 wrench 감시

현재 TCP Y normal force 중심 안전 로직에 다음을 추가한다.

```text
|Fx|, |Fy|, |Fz|
force norm
|Tx|, |Ty|, |Tz|
torque norm
force derivative
torque derivative
sensor saturation
non-finite value
message age
TF age
```

모드별 threshold를 설정 파일에 둔다.

```text
PAINT
CONTACT_SEARCH
APPROACH_PRECONTACT
RETRACT
TRAVEL
IDLE
```

contact 확인은 normal force로 수행하지만, collision/abort는 6축 wrench로 판단한다.

### F-2. filtered path와 fast path 분리

- 접촉 확인, 정상 과힘 지속 판단: 기존 filtered signal 사용
- 순간 충격, 큰 torque, saturation: raw 또는 최소 필터 signal 사용

raw fast path는 짧은 debounce와 높은 threshold를 사용하고, 모든 threshold는 파라미터화한다.

### F-3. Abort reason과 latch

Boolean 하나만 보내지 말고 명확한 reason을 발행한다.

예:

```text
NONE
CONTACT_NOT_FOUND
NORMAL_OVERFORCE
TANGENTIAL_FORCE_LIMIT
TORQUE_LIMIT
RAW_IMPACT
FT_STALE
FT_NONFINITE
TF_INVALID
CONTROLLER_FAULT
PLAN_INVALID
PLANE_INVALID
UNEXPECTED_CONTACT
ADMITTANCE_TRIM_LIMIT
```

- abort는 latch
- 명시적 reset service 없이는 자동 해제 금지
- reset은 IDLE, force disabled, trajectory inactive, F/T valid, low wrench 조건에서만 허용

### F-4. Tare와 bias interlock

Tare는 다음 조건에서만 허용한다.

```text
mode == IDLE
force_enable == false
trajectory inactive
robot stationary
free-space condition
```

adaptive bias update도 단순히 TRAVEL mode라는 이유만으로 하지 않는다.

```text
low wrench
low TCP/joint velocity
no recent contact
no saturation
stationary window
```

조건을 만족할 때만 update한다.

---

## G. 상태 머신과 실행 검증

### G-1. 상태 전환 조건

각 상태는 성공/실패 조건과 timeout을 명시한다.

```text
SAFETY_APPROACH
APPROACH_PRECONTACT
CONTACT_SEARCH
RAMP_UP
PAINT
RAMP_DOWN
RETRACT
TRAVEL
FINAL_RETRACT
ABORT
```

상태가 바뀔 때 다음 정보를 diagnostic으로 발행한다.

```text
previous_state
next_state
reason
timestamp
plan_hash
plane_generation_id
current force
current contact state
```

### G-2. RAMP_UP

- CONTACT_SEARCH가 성공한 뒤에만 진입
- force reference를 0에서 target까지 rate-limited ramp
- contact confirmed와 ramp complete를 모두 확인
- target은 현재 기본 1.6 N을 유지하되 설정 파일에서 관리
- contact threshold와 target force의 margin은 실측 전 자동 변경하지 않음
- timeout 시 PAINT로 넘어가지 않고 fault

### G-3. RAMP_DOWN

- PAINT 종료 후 force reference가 0이 될 때까지 기다림
- guard가 zero를 실제 publish했는지 feedback/diagnostic 확인
- 이후에만 RETRACT

### G-4. Motion action 처리

- `compute_cartesian_path` fraction이 기준 미달이면 실행하지 않음
- collision check 활성화 확인
- empty trajectory 거부
- action result SUCCESS 후에만 다음 상태
- cancel 결과와 timeout 처리
- 중복 `/sketch_execute` 요청 방지
- 실행 중 새 path 수신 시 현재 plan에 반영하지 않고 pending 또는 reject

---

## H. 단일 설정 파일과 상위 launch

### H-1. 설정의 single source of truth

새 설정 파일을 만든다.

```text
src/sketch_control/config/painting_system_real.yaml
```

또는 패키지 구조상 더 적합한 공용 config 위치를 사용한다.

최소 포함 항목:

```yaml
system:
  real_painting_enabled: false
  dry_run: true

geometry:
  roller_radius_m: 0.026
  contact_geometry_offset_m: <verified value>
  precontact_clearance_m: 0.005
  travel_clearance_m: 0.010
  safety_approach_offset_m: 0.080
  final_retreat_offset_m: 0.080

perception:
  d405_single_capture: true
  d405_multi_sample_fusion: false
  allow_zed_fallback_in_real_mode: false
  settle_time_s: 1.0
  capture_timeout_s: 1.2
  max_prescan_poses: 5
  # ROI/RANSAC/quality parameters

motion:
  paint_speed_mps: 0.020
  approach_speed_mps: 0.005
  contact_search_speed_mps: 0.002
  retract_speed_mps: 0.010
  travel_speed_mps: 0.030
  contact_search_step_m: 0.0005
  contact_search_max_distance_m: 0.010
  contact_search_timeout_s: 8.0

force:
  target_force_n: 1.6
  contact_detect_n: 1.5
  contact_release_n: 0.8
  travel_collision_n: 3.0
  warning_force_n: 10.0
  abort_force_n: 15.0
  command_cap_n: 15.0
  ramp_up_s: 2.0
  ramp_down_s: 1.0
  # all-axis force/torque limits and derivative limits

watchdog:
  requested_wrench_timeout_s: ...
  mode_timeout_s: ...
  enable_timeout_s: ...
  executor_heartbeat_timeout_s: ...
  ft_timeout_s: 0.20

admittance_limits:
  max_normal_trim_m: 0.005
  max_normal_velocity_mps: 0.010
  max_normal_acceleration_mps2: ...
  max_force_slew_rate_nps: ...
```

기존 launch와 node의 중복 default를 제거하고 이 파일에서 값을 전달한다.

### H-2. 상위 launch

최종 운용용 launch를 만든다.

```text
src/sketch_control/launch/rb10_painting_system.launch.py
```

역할:

- 기존 robot/MoveIt/controller launch 포함
- ZED/D405/perception launch 포함
- force monitor, wrench guard, executor 포함
- 필요 시 rosbridge option 제공
- 동일 YAML을 모든 node에 전달
- controller와 필수 topic/TF가 준비되기 전 execution interlock 닫힘

기본값:

```text
real_painting_enabled=false
dry_run=true
```

사용자가 명시적으로 둘 다 바꾸지 않으면 실제 비영 force가 controller로 들어가지 않아야 한다.

### H-3. Startup readiness

실행 가능 상태는 다음을 모두 확인한 뒤에만 true다.

```text
hardware connected
required controllers active
FollowJointTrajectory action available
ZED topics valid
D405 point cloud valid
required TF valid
F/T valid and fresh
wrench guard active
abort not latched
current work area refined by D405
current plan validated
```

`/painting_system/readiness` 또는 diagnostic topic으로 각 조건을 보여준다.

---

## I. UI 수정

웹 UI에서 다음 상태를 명확히 표시한다.

```text
Target selected
Work area selected
D405 refining
D405 plane accepted/rejected
Plane generation ID
Path generated
Plan hash
Plan validated/rejected
Ready to run
Running
Abort reason
```

요구사항:

- D405 refined plane이 없으면 Path/Run 버튼 비활성
- plan이 invalidated되면 Run 버튼 즉시 비활성
- Fill preview는 backend 최종 path와 동일
- 작업영역 밖 자유 스케치는 오류 표시
- 사용되지 않는 `/measure_d405_plane` 버튼/토픽은 실제 backend에 연결하거나 제거
- Run 확인창에 최소한 plan hash 앞 8자리, target force, work-area ID, plane generation ID 표시

---

## J. Logging과 진단

다음 정보를 rosbag 또는 로그에서 재현할 수 있게 한다.

```text
active target/work area ID
D405 capture metrics
accepted plane point/normal
plane generation ID
segment plan hash
state transitions
contact search cumulative distance
requested wrench
actual guarded wrench
filtered/raw F/T
contact state
admittance trim/limit state
abort reason
trajectory action result
```

가능하면 `diagnostic_msgs/DiagnosticArray`를 사용하고, 기존 토픽 호환성이 필요하면 기존 Bool/String도 함께 유지한다.

---

# 5. 테스트 요구사항

## 5.1 Unit test

최소 다음 테스트를 추가한다.

### Segment/schema

- version 3 parse 성공
- version 2 real execution 거부
- missing path ID 거부
- mismatched plan hash 거부
- mismatched plane generation 거부
- invalid normal/tangent 거부
- non-contact force가 0으로 강제되거나 경로 거부

### Work area

- Fill의 모든 점이 선택 사각형 내부
- 작업영역 밖 자유 스케치 거부
- 2D와 3D containment 결과 일치
- 작은 작업영역에서 유효 stroke 없음 오류

### Plane lifecycle

- 한 capture 성공 시 즉시 accepted
- 첫 capture reject 후 두 번째 pose 성공
- accepted 뒤 추가 capture를 수행하지 않음
- 새 work area 시 이전 plane invalid
- 30초 경과만으로 ZED fallback하지 않음
- PAINT 중 새 D405 message가 plan을 변경하지 않음

### State machine

- APPROACH_PRECONTACT -> CONTACT_SEARCH
- contact 발견 -> RAMP_UP
- contact 없음 -> CONTACT_NOT_FOUND
- RAMP_UP timeout -> PAINT 금지
- RAMP_DOWN 완료 전 RETRACT 금지
- 중복 execute 거부

### Wrench guard

- stale requested wrench -> zero
- stale mode -> zero
- stale enable -> zero
- stale heartbeat -> zero
- F/T invalid -> zero
- abort latch -> zero
- TRAVEL/RETRACT -> zero
- force cap 초과 -> reject 또는 clamp + fault

### Safety monitor

- tangential force limit
- torque limit
- raw impact
- F/T stale
- TF invalid
- reset interlock

## 5.2 Integration/launch test

fake hardware에서 다음을 자동 검증한다.

1. matching segment 없음 -> FJT goal이 전송되지 않음
2. D405 refined plane 없음 -> real Run 거부
3. work area보다 큰 Fill 요청 -> 범위 밖 segment가 생성되지 않음
4. marker 위치와 segment offset이 일치
5. CONTACT_SEARCH에서 simulated contact -> 정상 전환
6. contact 없음 -> 최대 거리에서 abort
7. executor heartbeat 중단 -> guarded wrench 0
8. requested force publisher 중단 -> guarded wrench 0
9. F/T stale -> trajectory cancel 요청과 abort latch
10. 실행 중 plane generation 변경 -> 현재 plan 불변, 다음 실행 invalid

## 5.3 Build/test 명령

workspace underlay를 정확히 source한다.

```bash
cd ~/sketch_robot_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

변경한 Python 파일에는 최소한 compile check를 수행한다.

```bash
python3 -m compileall src/sketch_control src/rbpodo_painting_control
```

가능하면 변경 관련 패키지만 먼저 선택 빌드한 뒤 전체 빌드를 수행한다.

---

# 6. 실로봇 적용 전 수동 검증 시나리오

자동 테스트 이후에도 다음 순서로 검증할 수 있도록 문서를 작성한다.

## 6.1 Dry-run

- `real_painting_enabled=false`
- `dry_run=true`
- D405 single capture 상태와 plane ID 확인
- 작업영역 Fill/스케치 경로 확인
- RViz marker와 segment JSON 비교
- 상태 머신이 wrench 0으로 진행하는지 확인

## 6.2 Free-space force pipeline

- 로봇을 벽과 충분히 떨어뜨림
- wrench guard 출력 확인
- mode/enable/heartbeat를 각각 끊어 zero 전환 확인
- F/T stale/TF invalid injection
- admittance trim이 비접촉 mode에서 누적되지 않는지 확인

## 6.3 기준판 contact search

- 낮은 속도
- 작은 최대 탐색거리
- 페인트 없이 평평한 기준판
- 실제 접촉 검출 거리 기록
- 조기 접촉 처리 확인
- contact 없음 timeout 확인
- 과힘/torque abort 확인

## 6.4 짧은 1-stroke 도장

- 작은 작업영역
- stroke 하나
- connector 없음
- D405 한 번 측정 후 plane lock
- PAINT 중 plane이 변하지 않는지 확인
- RAMP_DOWN 후 RETRACT 순서 확인

---

# 7. 문서 갱신

코드 수정 후 다음 문서를 갱신하거나 새로 만든다.

```text
docs/FINAL_PAINTING_ROBOT_SYSTEM_UPDATED.md
docs/PAINTING_SYSTEM_IMPLEMENTATION_REPORT.md
```

`FINAL_PAINTING_ROBOT_SYSTEM_UPDATED.md`에는 실제 수정된 구현만 기록한다.

반드시 반영할 내용:

- D405는 단일 capture, 첫 유효 결과 채택
- 다중 측정/융합 없음
- PAINT 중 법선 업데이트 없음
- plane generation lifecycle
- version 3 segment contract
- contact geometry와 clearance 분리
- CONTACT_SEARCH 상태
- guarded wrench와 controller timeout 여부
- work area/Fill/marker 일치
- 단일 YAML과 최종 launch
- 남아 있는 실로봇 blocker

`PAINTING_SYSTEM_IMPLEMENTATION_REPORT.md`에는 다음을 기록한다.

```text
변경 파일 목록
각 파일의 변경 목적
빌드 결과
테스트 결과
실행하지 못한 테스트와 이유
controller native timeout 지원 여부
overlay controller patch 여부
검증된 TCP-to-roller contact offset
남아 있는 P0/P1 이슈
실로봇 Go/No-Go 판단
```

확인하지 않은 사항을 완료했다고 쓰지 않는다.

---

# 8. 금지 사항

다음은 수행하지 않는다.

- D405 다중 위치 평면 평균
- D405 다중 프레임 SVD fusion
- PAINT 중 실시간 normal update
- 카메라 노이즈를 직접 TCP orientation command로 연결
- 전체 robot-wall collision 비활성
- matching segment 없이 PoseArray fallback 실행
- D405 실패 후 실도장에서 조용히 ZED fallback
- plan 재투영 후 operator 확인 없이 즉시 실행
- 여러 node가 `/admittance_controller/wrench_reference`에 동시에 publish
- force/threshold 값을 여러 launch에 중복 하드코딩
- `/opt/ros` 파일 직접 수정
- 사용자 기존 변경사항 강제 reset
- 테스트 실패를 숨기거나 성공으로 보고

---

# 9. 완료 조건

다음 조건을 모두 만족해야 작업 완료로 간주한다.

```text
[ ] D405는 한 capture만 사용하고 첫 valid 결과를 lock한다.
[ ] PAINT 중 plane/normal은 업데이트되지 않는다.
[ ] real mode에서 D405 refined plane 없이는 실행되지 않는다.
[ ] Fill과 자유 스케치가 선택 작업영역을 벗어나지 않는다.
[ ] RViz marker가 최종 segment와 동일하다.
[ ] real mode에서 matching version 3 segment가 없으면 무동작이다.
[ ] contact geometry와 clearance가 분리되었다.
[ ] CONTACT_SEARCH가 거리/시간/힘 제한을 가진다.
[ ] non-contact mode의 controller target wrench는 항상 0이다.
[ ] stale command/mode/heartbeat/F/T에서 guarded output이 0이다.
[ ] controller native reference timeout 또는 동등한 controller-side 보호가 확인되었다.
[ ] all-axis force/torque 안전 envelope가 있다.
[ ] 설정값은 하나의 YAML에서 공급된다.
[ ] 상위 launch 기본값은 real force disabled이다.
[ ] unit/integration test가 추가되고 결과가 기록되었다.
[ ] 최종 시스템 문서와 구현 보고서가 갱신되었다.
```

controller-side stale reference 보호나 compliance gating을 완료하지 못한 경우에는 실제 force 도장을 Go로 표시하지 않는다.

---

# 10. Codex 최종 응답 형식

작업 후 다음 순서로 보고한다.

1. **구현 요약**
2. **변경 파일 목록과 핵심 변경점**
3. **빌드 결과**
4. **테스트 결과**
5. **실로봇에서 아직 확인해야 할 항목**
6. **남은 blocker**
7. **최종 실행 명령**
8. **Go/No-Go 판정**

코드 일부만 제안하고 끝내지 말고, 가능한 범위에서 실제 파일을 수정하고 테스트한다. 실제 source와 설치된 controller 기능을 확인할 수 없는 사항은 추측하지 말고 blocker로 명시한다.
