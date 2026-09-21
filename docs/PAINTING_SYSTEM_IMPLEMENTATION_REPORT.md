# RB10 도장 시스템 구현 보고서

보고서 기준일: 2026-08-10  
대상: `/home/Minjea/sketch_robot_ws` 현재 작업 트리  
요구사항: `docs/CODEX_RB10_PAINTING_SYSTEM_REVISION_PROMPT.md`

## 1. 결론

현재 작업 트리에는 version 3 segment/identity/hash, 선택 작업영역 geometry, D405 single-capture lifecycle, CONTACT_SEARCH 상태 머신, guarded wrench, 6축 force safety monitor, controller-side safety overlay, readiness/UI gate, 공통 설정과 최상위 launch가 소스 수준으로 통합되어 있다.

전체 workspace Release build는 11 packages 성공했고, 전체 `colcon test-result`는 179 tests/0 errors/0 failures/9 skipped로 종료했다. Python/JavaScript/diff/xacro/URDF 정적 검사와 top launch `--show-args`도 통과했다. 제한된 top fake launch에서는 core node 기동, controller 5개 active, workspace overlay prefix 및 F/T stale fail-closed를 확인했다. 실제 RB10/D405/AFT200와 browser/rosbridge E2E, 원문 10개 fake 시나리오 전체는 미실행 또는 미완료이므로 실로봇 판정은 여전히 **No-Go**다.

## 2. 감사 기준과 상태 정의

이 보고서는 현재 filesystem을 직접 읽어 작성했다. 작업 트리에는 다수의 기존 미커밋/미추적 변경이 있으며 단일 기준 commit이 제공되지 않았다. 따라서 “변경 파일”은 현재 구현을 구성하는 관련 파일 목록이지, 모두 이 한 작업에서 새로 만든 것으로 귀속한다는 의미가 아니다. 커밋은 만들지 않았다.

| 상태 | 의미 |
|---|---|
| 소스 구현 확인 | 요구 동작을 수행하는 현재 소스/설정이 존재함 |
| 자동 검증 완료 | 6절에 기록한 build/test/static check 범위에서 결과가 확정됨 |
| 제한적 smoke 확인 | 특정 top fake startup 또는 음성 경로를 확인했으나 전체 launch E2E를 뜻하지 않음 |
| 실기 미검증 | 실제 센서/controller/robot에서 동작과 수치가 승인되지 않음 |
| 미완/예외 | 원문 계약과 완전히 같지 않거나 추가 증빙/수정이 필요함 |

## 3. 요구사항별 구현 상태

| 요구 영역 | 현재 소스 구현 | 최종 검증 상태 | 남은 핵심 항목 |
|---|---|---|---|
| A. Segment fail-closed | v3 root, strict real validator, v2 legacy-only, canonical SHA-256, path/hash/work-area/plane ID matching, real PoseArray fallback 차단 | 관련 자동 test 포함, 전체 집계 통과 | 전체 executor/ROS graph에서 goal 0건 E2E 증빙 |
| B. Work area/Fill/sketch/marker | 선택 pixel rect Fill, 0.175 m roller half-margin/overlap, 작은 영역 거부, 2D+3D containment, backend preview, final row marker/DELETEALL | 관련 자동 test 포함, 전체 집계 통과 | 실제 Wall Front calibration과 RViz/UI 육안·수치 대조 |
| C. D405 lifecycle | 한 trigger당 한 cloud, first accepted lock, quality metrics, explicit invalidation, status-before-Pose arm/consume, PAINT deferred invalidation | 관련 자동 test 포함, 전체 집계 통과 | 실제 D405 calibration/pointcloud/threshold, 후보 pose retry E2E |
| D. Contact geometry/search/collision | 0.026 geometry와 clearance 분리, bounded CONTACT_SEARCH, 조기 contact, selective roller-surface ACM, final 0.080 row | 관련 자동 test 포함, 전체 집계 통과 | 실물 치수, RB10 search 거리/속도, ACM 복원 실기 확인 |
| E. Guarded wrench/controller timeout | requested/final topic 분리, single guard output, watchdog/cap/zero, workspace controller overlay timeout/compliance gate/limits | build/test 및 fake prefix 확인 | 실제 controller의 RT stale/disable/drift/limit 확인 |
| F. 6축 F/T safety | filtered/raw 6축 envelope, derivative/impact/saturation/stale/TF, reason latch, reset/bias interlock | 관련 자동 test와 fake F/T stale 음성 경로 통과 | 실제 AFT200 부호/noise/impact와 threshold 승인 |
| G. 상태 머신 | SAFETY_APPROACH→PRECONTACT→SEARCH→RAMP_UP→PAINT→RAMP_DOWN→RETRACT/TRAVEL→FINAL, action/fraction/timeout checks와 RAMP_DOWN disable/guard-zero handshake | 관련 자동 test 포함, 전체 집계 통과 | full fake-hardware launch 시나리오와 실기 cancel/abort |
| H. 설정/launch/readiness | controller를 포함한 공통 YAML, top launch, force 기본 off, backend readiness 세부 checks | `--show-args`와 제한적 fake startup 통과 | 실제 hardware launch parameter/runtime 확인 |
| I. UI | authoritative status panel, ID/hash/force/blockers, Path/Run disable, local invalidation, Run 확인창 | JavaScript 문법 검사 통과 | 실제 browser/rosbridge E2E |
| J. logging/diagnostic | plan/D405/readiness/execution/guard/safety JSON과 contact search distance, reason topic 존재 | 전체 자동 test 집계 통과 | rosbag 수집/재생 절차와 한 run correlation 검증 |

## 4. 핵심 구현 사실

### 4.1 측정 geometry

현재 roller52 mesh와 URDF가 사용하는 기준은 다음과 같다.

| 항목 | 값 |
|---|---:|
| roller usable length | **0.175 m** |
| roller diameter | **0.052 m** |
| roller radius / `contact_geometry_offset_m` | **0.026 m** |
| roller component local center | `(0, -0.018, 0.07675) m` |
| TCP→roller axis normal distance | **0.24420 m along TCP -Y** |
| TCP→outer contact normal distance | **0.27020 m** |

실행 geometry는 다음처럼 두 단계로 적용된다.

```text
surface point
  + normal * (contact_geometry_offset_m + row.offset_m)
  = roller axis target

roller axis target
  + normal * 0.24420
  = TCP target
```

즉 contact TCP standoff는 `0.026 + 0.24420 = 0.27020 m`다. 5 mm precontact, 10 mm travel, 80 mm safety/final retreat은 별도의 row/root clearance이며 물리 반경 0.026 m에 합쳐 하나의 “contact offset”으로 저장하지 않는다.

형상 근거 파일:

- `src/eoat_description/meshes/rr_00a_b_eoat_no_camera_roller52_mesh_metadata.txt`
- `src/eoat_description/urdf/rr_00a_b_eoat_no_camera.urdf.xacro`
- `src/sketch_control/urdf/rbpodo_with_eoat.urdf.xacro`
- `src/sketch_control/sketch_control/moveit_executor.py`

이 값은 mesh/URDF source 기준으로 일치하지만 실제 조립품 계측 결과는 아직 실기 미검증이다.

### 4.2 v3/hash/identity

`segment_path.py`의 현재 계약은 다음을 구현한다.

- `SEGMENT_SCHEMA_VERSION = 3`
- v3 필수 `path_id`, `plan_hash`, `work_area_id`, `plane_generation_id`
- `surface_point`, `tcp_normal_axis=+y`, orientation continuity 강제
- contact geometry, precontact, travel, safety approach, final retreat 분리
- key sort와 float 9자리의 canonical JSON
- root `plan_hash`를 제외한 SHA-256
- parse한 객체와 hashed raw payload 일치 재검증
- v2는 parser 호환만 유지하고 real validator에서 거부
- path 시작/종료 및 CONTACT_SEARCH/RAMP/PAINT force 순서 검증

`sketch_to_waypoints_node.py`는 accepted D405 IDs와 최종 `link0` surface rows로 payload를 만든 뒤 hash하고, 같은 parsed object에서 marker와 호환 PoseArray를 만든다. `moveit_executor.py`는 UI/plan-status acceptance를 신뢰만 하지 않고 같은 IDs/hash/schema를 다시 검사한다.

### 4.3 D405 lifecycle

`plane_lifecycle.py`와 `d405_surface_refiner_node.py`의 pure/ROS 계층은 분리되어 있다.

- capture trigger가 arm한 다음 PointCloud2 한 개만 consume
- ROI/voxel/RANSAC 한 번의 결과를 검증
- accept 뒤 추가 cloud 무시, 새 invalidation/trigger 전까지 lock
- 단순 elapsed time으로 accepted plane을 만료하지 않음
- PAINT active 중 invalidation을 지연하여 current snapshot 유지
- status에 quality metrics, `work_area_id`, `plane_generation_id`, rejection reason 포함
- accepted status를 refined Pose보다 먼저 발행

`sketch_to_waypoints`, executor와 UI는 공용 status topic에서 명시적인 `mode=work_area`만 처리한다. mode 누락이나 target status는 work-area acceptance를 열지 않는다. accepted status를 받으면 이전 Pose cache를 clear하고 다음 Pose 하나만 arm/consume하므로 target refinement status나 이전 transient Pose가 real work-area generation을 열지 않는다.

### 4.4 force safety와 controller 지원 여부

#### Upstream/native 상태

workspace의 `src/admittance_controller/LOCAL_SAFETY_PATCH.md`가 기록한 Jazzy 4.40.1 baseline은 원문이 요구한 stale wrench timeout과 explicit compliance heartbeat gate를 native 안전 계약으로 제공하지 않는 것으로 source audit되어 있다. `/opt/ros` 파일은 직접 수정하지 않았다.

#### Workspace overlay

`src/admittance_controller` overlay에는 다음 source patch가 존재한다.

- steady-clock wrench timeout
- compliance-enable heartbeat timeout과 default off
- measured/requested wrench finite gate
- non-finite hardware state 보존 후 compliance off
- off 상태 zero excitation과 smooth spring/damping return
- TCP Y normal trim, velocity, acceleration clamp
- compliance/limit status publisher

top launch의 관련 controller profile은 공통 `painting_system_real.yaml` 안에 있고 Y translation만 선택한다. 현재 설정은 trim 0.005 m, velocity 0.010 m/s, acceleration 0.050 m/s²다. `painting_normal_y.yaml`은 이전 호환 launch용 profile로 유지된다.

자동 검증에서 overlay package/plugin은 workspace `install` prefix에서 resolve되었고 top fake launch의 controller 5개가 active가 되었다. 따라서 build/plugin provenance는 더 이상 미확정 P0가 아니다. 다만 실제 RB10 controller manager에서의 RT stale/disable, drift와 normal limit 동작은 hardware commissioning 전까지 미검증이다.

#### Guard와 monitor

`painting_wrench_guard`는 `/admittance_controller/wrench_reference`의 의도된 유일 publisher다. requested wrench, mode, force enable, executor heartbeat, F/T, TF, safety, abort, controller 상태와 cap 중 하나라도 실패하면 exact zero와 compliance off를 발행한다.

RAMP_DOWN은 다음 두 단계를 명시적으로 구현한다.

```text
RAMP_DOWN reference reaches zero
  -> executor publishes force_enable=false
  -> disable edge 뒤 새 guard status가 zero output + compliance disabled 보고
  -> only then RETRACT
```

pure helper와 unit test는 disable 전 상태나 stale/pre-disable guard status를 완료로 인정하지 않는 계약을 담는다. 실제 ROS timing과 controller acknowledgement는 최종 통합 검증 대상이다.

`painting_force_monitor`는 filtered/raw 6축 force/torque, norm, derivative, saturation, message/TF age와 unexpected contact를 mode별 envelope로 평가한다. abort는 reason과 함께 latch되며, reset/bias는 free-space와 stationary를 포함한 interlock를 요구한다.

### 4.5 UI gate

`web/js/app.js`는 다음 authoritative backend topic을 구독한다.

```text
/perception/d405_surface_refinement_status
/painting_system/plan_status
/painting_system/readiness
/painting_system/execution_status
```

UI는 D405 acceptance와 IDs, plan path/hash/work-area/generation, backend validation/readiness, target force, running/abort 상태를 결합한다. plane/plan local invalidation이 생기면 Run을 즉시 막고, backend `ready=true`가 아니면 UI 상태만으로 Run을 열지 않는다. confirmation에는 hash 앞 8자리, target force, work-area ID, plane generation ID, path ID가 들어간다.

브라우저와 실제 rosbridge를 연결한 실행 증빙은 아직 없다.

### 4.6 공통 설정과 top launch

`src/sketch_control/config/painting_system_real.yaml`은 application node와 `admittance_controller` section으로 geometry, D405, motion/force, watchdog, controller limit, 6축 mode limit을 공급한다. 기본값은 real false, dry-run true, force false다.

`src/sketch_control/launch/rb10_painting_system.launch.py`는 다음을 하나의 entrypoint로 묶는다.

- `rb10_moveit_full.launch.py`
- `rb10_real_perception_sketch.launch.py`
- `painting_admittance_control.launch.py`
- `moveit_executor`

perception wrapper는 같은 config file과 real/dry 인자를 전달하고 D405를 `stable/spatial/max=1`, lock true로 정리한다.

공통 YAML에는 `admittance_controller` section도 들어 있다. top launch는 빈 별도 admittance profile과 `painting_config_file`을 MoveIt child에 전달하고, `rb10_moveit_full.launch.py`는 그 파일을 `ros2_control_node` parameter 목록 마지막에 append한다. 따라서 최상위 운용 경로에서는 공통 YAML이 controller의 authoritative final override다. `painting_normal_y.yaml`은 최상위 launch를 사용하지 않는 이전 호환 경로용이다.

## 5. 관련 변경 파일과 목적

### 5.1 Segment, geometry, path generation

| 파일 | 목적 |
|---|---|
| `src/rbpodo_painting_control/rbpodo_painting_control/segment_path.py` | v3 schema, canonical hash, strict real validation, mode/force/geometry 계약 |
| `src/rbpodo_painting_control/test/test_segment_path.py` | v3/hash/legacy/mismatch/vector/order/offset 회귀 검사 |
| `src/sketch_control/sketch_control/work_area_geometry.py` | ROS 비의존 Fill 및 2D/3D containment |
| `src/sketch_control/sketch_control/sketch_to_waypoints_node.py` | D405 gate, selected Fill/sketch, v3 생성, marker/PoseArray |
| `src/sketch_control/test/test_work_area_geometry.py` | roller margin, 작은 영역, 2D/3D containment 검사 |
| `src/sketch_control/test/test_eoat_segment_generation.py` | final segment/marker/IDs/D405 status-before-Pose adapter 검사 |

### 5.2 Plane lifecycle와 perception

| 파일 | 목적 |
|---|---|
| `src/sketch_control/sketch_control/plane_lifecycle.py` | single-cloud fit/quality와 explicit invalidation/paint lock state machine |
| `src/sketch_control/sketch_control/d405_surface_refiner_node.py` | one-shot capture ROS adapter, metrics/status-before-Pose |
| `src/sketch_control/sketch_control/wall_projector_node.py` | work-area UUID/state와 backend Fill preview 표시 |
| `src/sketch_control/test/test_plane_lifecycle.py` | one-capture, retry, ID/invalidation, no time fallback, PAINT lock 검사 |

### 5.3 Executor와 process safety

| 파일 | 목적 |
|---|---|
| `src/sketch_control/sketch_control/moveit_executor.py` | real plan gate, readiness, bounded search, ramp feedback, selective ACM, explicit final row |
| `src/sketch_control/sketch_control/painting_execution.py` | ROS 비의존 contact-search 및 identity gate decision logic |
| `src/sketch_control/test/test_painting_execution.py` | search bounds/fault/contact와 identity mismatch 검사 |
| `src/sketch_control/test/test_executor_plan_identity.py` | plan/waypoint callback identity race 검사 |
| `src/sketch_control/test/test_moveit_executor_fail_closed.py` | matching v3 부재 시 plan/FJT goal 미생성 adapter 검사 |

### 5.4 Force path와 controller

| 파일/영역 | 목적 |
|---|---|
| `src/rbpodo_painting_control/rbpodo_painting_control/painting_wrench_reference_node.py` | mode 기반 requested wrench, ramp/slew/cap/zero |
| `src/rbpodo_painting_control/rbpodo_painting_control/wrench_guard.py` | pure fail-closed truth table |
| `src/rbpodo_painting_control/rbpodo_painting_control/painting_wrench_guard_node.py` | final controller wrench/compliance output과 watchdog diagnostics |
| `src/rbpodo_painting_control/rbpodo_painting_control/force_safety.py` | 6축/raw/filtered latch와 bias/reset pure logic |
| `src/rbpodo_painting_control/rbpodo_painting_control/painting_force_monitor_node.py` | ROS F/T transform/monitor adapter |
| `src/rbpodo_painting_control/launch/painting_admittance_control.launch.py` | reference/guard/monitor 묶음과 공통 config 전달 |
| `src/rbpodo_painting_control/test/test_wrench_guard.py` | stale/mode/abort/cap zero truth table |
| `src/rbpodo_painting_control/test/test_force_safety.py` | 6축/raw/reset/bias/contact 검사 |
| `src/admittance_controller/` | Jazzy controller workspace safety overlay와 gtest |
| `src/sketch_control/config/painting_system_real.yaml`의 `admittance_controller` | top launch의 TCP Y-only admittance와 overlay limits |
| `src/rbpodo_ros2/rbpodo_bringup/config/admittance_profiles/painting_normal_y.yaml` | top launch 밖 이전 wrapper의 호환 profile |
| `src/rbpodo_ros2/rbpodo_hardware/test/test_hardware_safety.cpp` | hardware fail-closed/recovery 회귀 검사 |

### 5.5 EOAT, config, launch, UI와 packaging

| 파일/영역 | 목적 |
|---|---|
| `src/eoat_description/meshes/*roller52*`, `*support_only*` | 0.175 × 0.052 m roller와 선택 collision 분리 |
| `src/eoat_description/urdf/rr_00a_b_eoat_no_camera.urdf.xacro` | 실제 roller cylinder collision link |
| `src/sketch_control/urdf/rbpodo_with_eoat.urdf.xacro` | TCP→EOAT mount geometry |
| `src/sketch_control/config/painting_system_real.yaml` | application/controller 공통 real profile과 안전 기본값 |
| `src/sketch_control/launch/rb10_painting_system.launch.py` | 최상위 운용 entrypoint |
| `src/sketch_control/launch/rb10_real_perception_sketch.launch.py` | 실제 camera/bridge wrapper와 공통 config 전달 |
| `src/sketch_control/launch/rb10_perception_sketch.launch.py` | perception nodes와 single-capture override |
| `src/sketch_control/setup.py`, `package.xml` | config/launch 설치와 runtime dependency |
| `web/index.html`, `web/js/app.js`, `web/style.css` | authoritative status/readiness/Run UI gate |
| `docs/FINAL_PAINTING_ROBOT_SYSTEM_UPDATED.md` | 갱신된 시스템/운용 사양 |
| `docs/PAINTING_SYSTEM_IMPLEMENTATION_REPORT.md` | 본 구현·검증·blocker 보고서 |

## 6. Build와 test 결과

### 6.1 현재 확정 상태

| 항목 | 결과 |
|---|---|
| Python `compileall` | 통과 |
| JavaScript `node --check` | 통과 |
| 작업 트리 `diff --check` | 통과 |
| xacro 생성 및 `check_urdf` | 통과 |
| segment/work-area/plane/process/guard/safety unit test | 전체 colcon test 집계에 포함, error/failure 0 |
| `admittance_controller` overlay build/gtest | 전체 Release build/test 집계에 포함, error/failure 0 |
| `rbpodo_hardware` safety test | 전체 colcon test 집계에 포함, error/failure 0 |
| `sketch_control`/`rbpodo_painting_control` package test | 전체 colcon test 집계에 포함, error/failure 0 |
| 전체 workspace Release build | **11 packages 성공** |
| 전체 `colcon test-result --verbose` | **179 tests, 0 errors, 0 failures, 9 skipped**; skip은 `cppcheck 2.13`의 알려진 self-skip |
| workspace overlay prefix | `admittance_controller`가 workspace `install` prefix에서 resolve됨 |
| top launch `--show-args` | 통과 |
| top fake launch smoke | core node 기동, controller 5개 active, robot 미연결, F/T 미주입 시 `FT_STALE`/`ABORT` 및 guarded exact-zero 확인 |
| 원문 10개 fake-hardware launch E2E | 제한적 stale 음성 경로 외 미완료 |
| browser/rosbridge E2E | 미실행 |
| 실제 RB10/D405/AFT200 | 미실행 |

179건은 전체 test-result 집계이며 개별 package별 건수로 재배분하지 않는다. 9건 skip은 `cppcheck 2.13` 환경에서 알려진 self-skip로 기록되었고 error/failure는 아니다.

### 6.2 관련 검사 재현 명령

```bash
cd ~/sketch_robot_ws
source /opt/ros/jazzy/setup.bash

python3 -m compileall \
  src/sketch_control \
  src/rbpodo_painting_control

PYTHONPATH=src/sketch_control:src/rbpodo_painting_control \
  python3 -m pytest -q \
  src/rbpodo_painting_control/test/test_segment_path.py \
  src/rbpodo_painting_control/test/test_wrench_guard.py \
  src/rbpodo_painting_control/test/test_force_safety.py \
  src/sketch_control/test/test_work_area_geometry.py \
  src/sketch_control/test/test_eoat_segment_generation.py \
  src/sketch_control/test/test_plane_lifecycle.py \
  src/sketch_control/test/test_painting_execution.py \
  src/sketch_control/test/test_executor_plan_identity.py \
  src/sketch_control/test/test_moveit_executor_fail_closed.py
```

### 6.3 controller와 관련 package build/test

```bash
colcon build \
  --packages-select \
    admittance_controller \
    rbpodo_hardware \
    rbpodo_painting_control \
    sketch_control \
  --symlink-install \
  --allow-overriding admittance_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
colcon test \
  --packages-select \
    admittance_controller \
    rbpodo_hardware \
    rbpodo_painting_control \
    sketch_control \
  --event-handlers console_direct+
colcon test-result --verbose
```

### 6.4 전체 workspace 최종 검증

```bash
cd ~/sketch_robot_ws
source /opt/ros/jazzy/setup.bash
if [ -f ~/ros2_ws/install/setup.bash ]; then
  source ~/ros2_ws/install/setup.bash
fi

colcon build --symlink-install \
  --allow-overriding admittance_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
ros2 launch sketch_control rb10_painting_system.launch.py --show-args
```

위 전체 Release build와 test-result, compile/node/diff/xacro/URDF check 및 launch argument 확인은 성공으로 확정되었다. 실제 timestamp, commit/worktree fingerprint와 원본 console log는 별도 검증 산출물로 보존해야 한다.

## 7. 아직 실행되지 않았거나 증빙이 부족한 시험

제한된 top fake smoke에서는 `use_fake_hardware:=true`가 `fake_sensor_commands:=true`를 자동 적용하고 실제 robot 연결을 시도하지 않는 것을 확인했다. core node가 기동되고 controller 5개가 active가 되었으며, fake F/T를 주입하지 않았을 때 `FT_STALE`/`ABORT`와 guarded wrench exact zero가 유지되었다. 이는 fail-closed 음성 경로 한 가지의 증빙이지 정상 contact/paint E2E 성공이 아니다.

원문 5.2의 다음 10개 fake-hardware 시나리오는 pure/unit adapter 일부로 다뤄지지만 전체 ROS graph 결과는 아직 완결되지 않았다.

1. matching segment 없음 → MoveIt/FJT goal 0건
2. accepted D405 plane 없음 → real Run 거부
3. 큰 Fill 요청 → 범위 밖 segment 0건
4. marker 좌표와 canonical segment 좌표 일치
5. CONTACT_SEARCH simulated contact → 정상 RAMP_UP 전환
6. no-contact → 최대 거리에서 latched abort
7. executor heartbeat 중단 → guarded wrench exact zero/compliance off
8. requested wrench 중단 → guarded wrench exact zero/compliance off
9. F/T stale → `FT_STALE`/`ABORT`와 exact zero는 확인; trajectory cancel/action 결과까지는 추가 증빙 필요
10. 실행 중 plane generation 변경 → current plan 불변, 다음 실행 invalid

추가로 다음 실기 시험이 필요하다.

- D405 capture reject 뒤 다음 후보 pose retry와 first success lock
- 실제 RealSense frame prefix/TF age와 calibration hash invalidation
- AFT200 TCP Y 부호, raw/filtered timestamp와 saturation/NaN 경로
- controller timeout/enable heartbeat 중단 시 실제 joint command drift 없음
- selective ACM이 roller-active-surface pair만 변경하고 반드시 복원됨
- APPROACH 조기 접촉에서 goal cancel 완료 뒤 RAMP_UP 진입
- RAMP_DOWN의 guard zero/compliance-off acknowledgement 뒤에만 RETRACT
- FINAL_RETRACT row 0.080 m가 한 번만 실행됨
- 비상정지, action reject/abort/cancel timeout 후 복구 절차
- UI reconnect/transient status에서도 stale generation Run 불가

## 8. 남은 blocker

### P0 — hardware Go 전 필수

1. **full fake-hardware launch E2E 미완료**  
   원문 10개 시나리오의 topic/action 결과를 자동화하거나 재현 가능한 log로 남겨야 한다.

2. **D405 실측 검증 부재**  
   calibration, TF age, ROI/RANSAC threshold, single capture 품질, 실패 retry와 plane lock을 실제 camera로 확인해야 한다.

3. **AFT200/force 실측 검증 부재**  
   부호, tare/bias, noise, raw impact, 6축 limit, command cap과 abort/cancel을 기준 장비에서 검증해야 한다.

4. **RB10 motion/contact/ACM 실기 검증 부재**  
   TCP geometry, search speed/distance, early contact, selective collision allowance, controller/overlay RT 동작, ramp와 final retreat을 검증해야 한다.

5. **UI E2E 미실행**  
   browser/rosbridge reconnect와 ID/hash/generation invalidation을 포함한 Run gate를 실제 graph에서 확인해야 한다.

### P1 — 유지보수와 품질

1. top launch 밖의 호환 profile/legacy wrapper가 공통 YAML과 달라지지 않도록 회귀 검사를 유지해야 한다.
2. controller gravity compensation CoG/weight는 placeholder이므로 실제 EOAT 질량 특성 반영이 필요하다.
3. 현재 force/D405/admittance threshold는 초기값이며 실제 dataset 기반 승인값이 아니다.
4. full-run rosbag topic set, run ID와 plan hash 기반 correlation, 보관/분석 절차가 필요하다.
5. 기존 `FINAL_PAINTING_ROBOT_SYSTEM.md`가 stale하므로 운영자가 갱신 문서를 우선하도록 문서 인덱스/배포 절차를 정리해야 한다.

## 9. 최종 운용 entrypoint

argument 확인:

```bash
cd ~/sketch_robot_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sketch_control rb10_painting_system.launch.py --show-args
```

fake/dry-run skeleton:

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

실제 force 도장은 구조상 `real_painting_enabled=true`, `dry_run=false`, `painting_force_enabled=true`를 모두 요구한다. 현재는 No-Go이므로 이 조합을 실제 장치에서 실행하는 것을 승인하지 않는다.

## 10. Go/No-Go 판정

| 범위 | 판정 | 근거 |
|---|---|---|
| 소스 설계/구현 검토 | 조건부 완료 | 요구된 fail-closed 구조가 현재 source에 존재하며 hardware/E2E 증빙은 별도 필요 |
| 자동 build/test/static 검증 | **통과** | Release build 11 packages, 179 tests/0 errors/0 failures/9 skipped, static/URDF/argument checks 성공 |
| 제한적 top fake smoke | 부분 통과 | core node와 controller 5개 기동, robot 미연결, F/T stale fail-closed 확인 |
| fake-hardware 운용 승인 | No-Go | 원문 10개 full graph 시나리오 증빙 미완료 |
| 실제 RB10 free-space force 시험 | No-Go | 실제 controller/F/T/RT 동작 미검증 |
| 실제 기준판 contact 시험 | No-Go | D405/force/search/ACM 실기 미검증 |
| 실제 도장 | **No-Go** | P0 blocker 및 hardware commissioning 미완료 |

최종 Go 전환은 “코드가 있다”가 아니라 다음 증빙 묶음을 요구한다.

```text
기록된 Release build + full test result                 # 완료
+ fake-hardware launch E2E                              # 미완료
+ D405 single-capture hardware metrics
+ AFT200/guard/controller stale-and-abort tests
+ RB10 bounded contact/ACM/final-retract tests
+ UI identity gate E2E
+ 승인된 parameter record
```

남은 검증 완료 뒤 이 보고서의 실패/waiver, parameter revision, 담당자와 timestamp를 갱신한다. 현재 자동 검증 통과는 실제 force 도장 승인을 의미하지 않는다.
