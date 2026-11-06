# Platform EOAT and Execution Mode UI Implementation Plan

**2026-10-16 최신 재검증:** 왼팔·RB10 정상 도장/접촉 중 중단 통과는 유지한다.
오른팔은 기존 작업 종료 후 GPU 단독 관측 구간에서 재시험했지만, 첫 획 힘 유지
통과 뒤 두 번째 PAINT 진입의 `PAINT_NOMINAL_MEASURED_STATE_STALE`로 거부됐다.
GPU 공유만으로 설명할 수 없으며 명목 시작점 재검사의 source/clock 시각 경계가
추가 진단 대상이다. 별도 중단 시험은 이후 다른 계정 작업이 시작되어 접촉 전에
정리했다. 오른팔 정상·loaded cancel 및 실제 카메라 mixed 9조합은 미완료다.
이번에는 제어 코드를 변경하지 않았다. [실행 문서](../../PLATFORM_RUNTIME.md)의
오른팔 단독 GPU 재검증 절에 원자료와 판정 범위를 기록했다.

**최신 확정 기준 (2026-10-15):** 상용로봇 플랫폼은 **왼팔 내화뿜칠 spray v2 / 오른팔 Hongik V4 용접 EOAT + 레이저 센서**로 구성한다. 용접·뿜칠 대상체는 H_STEEL_USD다. 이 구성을 기본 장착 사양으로 사용한다. 레이저 센서 형상·CAD 프레임은 포함하며 실제 데이터 수신은 별도 연결 대상이다. 공정 선택에 따른 공구 교체 기능은 아래 계획대로 유지한다.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 실행 모드·플랫폼·팔·공정을 선택하면 해당 EOAT와 제어 구성을 적용하고, 상용로봇 플랫폼 양팔 및 RB10의 세 공정을 시뮬레이션/일부 시뮬레이션에서 실행한다.

**Architecture:** 실행 환경, 플랫폼, 활성 팔, 공정, 팔별 장착 상태를 별도 계약으로 정의한다. 한 구성에서 URDF/SRDF/USD/TCP/제어기 설정을 생성하고, 실제 카메라와 가상 카메라는 동일한 작업정보 인터페이스로 연결한다. 기존 실행기·Outpost·롤러 힘 제어를 재사용하며 설정 변경은 종료→생성→검증→재시작 트랜잭션으로 적용한다.

**Tech Stack:** Python, ROS 2 Jazzy, MoveIt, ros2_control, Isaac Sim 6.1, 기존 C++ IsaacPaintingSystem/admittance_controller, FastAPI, 기존 JavaScript UI, pytest, node:test.

**Spec:** `docs/superpowers/specs/2026-10-15-platform-eoat-ui-design.md` — 사용자 추가 요구를 포함한 현재 파일 전체를 함께 읽는다.

## 구현 체크포인트 — 2026-10-15

**코드 구현 완료, 라이브 검증 일부 미완료 (live verification incomplete).** Tasks 1–10의 코드
경로와 Task 11 보고서/probe는 구현되어 있다. 체크된 항목은 아래 기록의 코드·
단위/회귀 증거에 한하며, 18개 실행 조합이나 실제 카메라/롤러 힘 유지의 성공을
뜻하지 않는다. 구현과 라이브 검증이 한 항목에 섞인 경우 전체 항목은 미체크로 둔다.
Git 작성자 정보가 없어 임의 신원을 만들지 않았으며 모든 커밋 항목은 미완료다.
기존 운영 `isaac_assets`는 그대로 두고 새 자산은 `.runtime/platform/assets/`에 둔다.

| 범위 | 기록된 검증 증거 | 남은 실행 증거 |
| --- | --- | --- |
| 선택·장착·적용·API·관리 UI | 선택/API/RB20 130개, 생성기/장착/런치 89개, API/apply 198개, JS 140개; 실제 8081 기본/고급/모바일·스케치 화면 및 ZED 표시 | 실제 stack의 각 구성 적용/중단과 18행 실행 결과 |
| 롤러 센서·제어·회귀 | C++ 9개 통과; 최근 canonical Python 회귀 1852개 통과/18 skip | 양팔·RB10 각각 두 PAINT 획의 실측 힘 유지 및 실제 중단·후퇴 |
| 실제 카메라·측정 장면·RGB-D seam | mixed 수정 시점 넓은 회귀 1579개 통과/5 skip, 이후 D405·장면·런치 75개 통과; 고정 원 프레임과 refined revision dual ACK 검증 | 보정된 실제 ZED/D405 연결과 실제 선택 입력 → 가상 실행 |
| CAD 용접 fixture | 실제 CAD bead 면/법선·가림·모호성·provenance 및 native weld 22개 통과; 상용 양팔 0°/15°, RB10 60°/0° 실측 실행·위빙 통과 | 다른 작업선·각도 및 공정 중 별도 취소 |

행별 수치는 서로 겹치는 suite이며 합산하지 않는다. 과거 camera suite의 DDS
domain 232 충돌은 테스트 프로세스에서만 FASTDDS/FASTRTPS profile 변수를 해제하고
LOCALHOST discovery를 사용하여 후속 회귀에서 피했다. 최근 canonical 전체 회귀는 1852 passed / 18 skipped이며 이후 집중 회귀는
별도 기록한다. 위 수치를 모든 현재 변경의 최종 전체 완료로 확대하지 않는다.

시뮬레이션 9개 구성은 각각 관절 이동·복귀·취소 후 실측 정지와 비활성 팔
유지를 통과했다. 상용 왼팔·오른팔 용접은 가까운 `IfcFastener_WELD_004`를
work/travel 0°/15°로 실행해 위빙·추종 판정을 통과했다. RB10 용접도 동일 CAD
작업선을 60°/0°로 실행해 최대 위치 오차 0.443 mm로 통과했다. 상용 양팔과 RB10
뿜칠은 실제 렌더 RGB-D 선택부터 실행·실측 추종까지 통과했다. 롤러의
두 PAINT 획 힘 유지·이탈은 상용 왼팔에서 통과했다. 오른팔·RB10의 전체
롤러 실행과 세 구성의 접촉 중 중단은 아직 통과하지 못했다.
mixed 9개와 물리 로봇은 `not_run`이며 전체 18개 조합 완료로 집계하지 않는다.
주 작업 폴더의 전체 Python 회귀는 1,852 passed / 18 skipped, JS는 140 passed다.
후속 변경의 집중 검증과 최신 실행 증거·한계는 [실행 문서](../../PLATFORM_RUNTIME.md)를 따른다.


추가 구현: 명목 형상이 없는 동적 target registry, 선택한 고정 RGB-D로만 평면
추출, 카메라 쪽 표면 법선을 사용하는 weld standoff, D405 측정 mesh 갱신과
정확한 두 엔진 revision 수락 장벽, 취소 완료 전 설정 변경 차단을 반영했다.
고정 D405의 정지 캡처는 장면 갱신을 허용하되 실제 trajectory 소유 중 갱신은 거부한다.

최신 상세 로그: [공통 기록](../../../.superpowers/sdd/2026-10-15-platform-eoat-ui/progress.md),
[카메라·장면·런치](../../../.superpowers/sdd/2026-10-15-platform-eoat-ui/camera-progress.md),
[롤러](../../../.superpowers/sdd/2026-10-15-platform-eoat-ui/roller-progress.md),
[UI·API](../../../.superpowers/sdd/2026-10-15-platform-eoat-ui/ui-progress.md).
실행 명령과 현재 제한은 [PLATFORM_RUNTIME](../../PLATFORM_RUNTIME.md)을 따른다.

현재 적용 정책: 기본 revisioned apply는 simulation/mixed만 시작한다. real RB10도
기본 apply에서는 차단하며, 기존 물리 `work`/`spray_motion_test`는 종료 후 고급
설정으로 사용한다. 카메라 registration은 측정된 `fixed_measured`만 허용한다.
움직이는 실제 카메라의 촬영 시점 pose adapter는 구현되지 않았다. 새 CAD fixture는
노출된 CAD face의 실제 법선/표면 중선을 사용하며 자동 실물 인식을 주장하지 않는다.

## Global Constraints

- 화면의 Construct 명칭은 **상용로봇 플랫폼**, 기본 플랫폼 선택지는 **상용로봇 플랫폼 / RB10 베이스**다. 기존 RB20은 고급 설정에서 정확한 모델명으로 보존한다.
- 실행 모드는 **시뮬레이션 / 일부 시뮬레이션 / 실환경**, 공정은 **내화뿜칠 / 용접 / 롤러도장**이다. 작업 팔은 상용로봇 플랫폼에서만 표시한다.
- 일부 시뮬레이션은 **실제 카메라 + Isaac 로봇**이며 세 공정 모두 필수다. `fake`, `dry_run`, 로봇 없는 `zed_preview`로 대체하지 않는다.
- 상용로봇 플랫폼 양팔 모두 롤러의 접근·접촉·목표 접촉력 유지·경로 추종·중단·이탈을 지원한다. 형상 표시만으로 완료 처리하지 않는다.
- 양팔에 같은 공구를 장착할 수 있지만 작업 실행은 한 번에 한 팔이다. 비활성 팔의 공구는 유지한다.
- 상용로봇 플랫폼과 용접 공구는 Hongik v4 ZIP을 사용한다. 컨택트 팁 TCP를 유지하고 이전 150 mm 시험 토치는 추가하지 않는다. 실측 교정된 공구로 표시하지 않는다.
- 용접·뿜칠 시뮬레이션의 기본 대상은 H_STEEL_USD다. 기존 testbed/H-beam 충돌을 남기지 않으며 새 대상과 맞지 않는 예전 용접 fixture를 재사용하지 않는다.
- 실제 손목 카메라 자세를 가상 손목 자세로 대체하지 않는다. 가상 접촉력을 실물 힘 측정으로 보고하지 않는다.
- 신규 실로봇·분사·용접 출력 드라이버는 범위 밖이다. 기존 실환경 기능과 실행 인터록을 유지한다.
- 현재 작업 트리의 기존 수정 및 미추적 파일을 보존한다. 구현 시작 시 using-git-worktrees 절차로 작업 공간을 준비하고 이 세션의 SRDF 수정과 사용자 변경을 빠뜨리지 않는다. 무단 reset/clean/stash 금지.
- 구현 전 2026-10-15 확인 시 Isaac Sim·관련 ROS 프로세스와 관리 서버(8081)는 종료 상태였다. 이후 사용자 승인 범위에서 새 자산·설정의 일관성을 검증하고 simulator 라이브 통합을 진행한다. 실제 로봇 이동은 요청하지 않는다.

## Review Focus

1. 두 브라우저의 설정 경합, 적용 도중 실패/재접속: 최신 의도와 실제 적용 구성이 혼동되지 않아야 한다. Task 3/10에서 검증한다.
2. 동일 EOAT 양팔 장착, 오른팔 공구의 왼팔 이동: 링크 충돌·거울반전·토치 이중 오프셋이 없어야 한다. Task 2/4에서 검증한다.
3. 카메라 재연결·중복 프레임·시뮬레이터 pause/reset: 오래된 영상이 새 시각으로 살아나면 안 된다. Task 6에서 검증한다.
4. 실제 D405는 정지했는데 가상 팔만 이동: 실제 영상 TF가 가상 손목을 따라가면 안 된다. Task 6/7/9에서 검증한다.
5. 공구/작업면 변경 직전 경로와 명령 재전송: 이전 경로·힘 명령이 새 구성에서 실행되면 안 된다. Task 3/5/7/11에서 검증한다.

## 구성과 파일 경계

기존 대형 실행기를 새로 작성하지 않는다. 다음 작은 모듈이 설정·기하·입력 계약을 맡고 기존 실행기에는 선택된 계약을 주입한다. 아래 ‘신규’ 경로는 구현 시 생성할 파일이다.

| 책임 | 주요 경로 |
| --- | --- |
| 실행 선택과 지원 조합 | 신규 `src/sketch_control/sketch_control/runtime_selection.py` |
| 공구 정의와 팔별 모델 생성 | 신규 `src/sketch_control/sketch_control/eoat_catalog.py`, 신규 `scripts/prepare_platform_assets.py` |
| 적용 트랜잭션·상태 | 신규 `src/sketch_control/sketch_control/runtime_apply.py`, 기존 `process_supervisor.py`, `system_api.py` |
| 공통 시뮬레이션 진입점 | 신규 `scripts/platform_simulation_app.py`, 신규 `src/sketch_control/launch/platform_simulation.launch.py` |
| 측정 힘·제어기 | 기존 `src/isaac_painting_hardware/`, `scripts/rb20_painting_sensors.py`, `moveit_executor.py` |
| 실제 카메라 정합·시간 | 신규 `camera_registration.py`, `mixed_frame_sync.py` — 모두 `src/sketch_control/sketch_control/` 아래 |
| 공통 측정 작업면 | 신규 `task_scene.py`, 신규 `task_scene_node.py`, 신규 `scripts/platform_task_scene.py` |
| 실제 영상 용접선 | 신규 `rgbd_weld_provider.py`, 신규 `rgbd_weld_provider_node.py`, 기존 `weld_input_node.py` |
| 관리 UI | 기존 `web/system.html`, `web/js/system.js`, 신규 `web/js/runtime_settings.js` |
| 반복 실행 검증 | 신규 `scripts/check_platform_runtime.py`, 신규 `tests/platform_runtime/` |

## 공통 인터페이스

이 이름을 뒤의 작업에서 그대로 사용한다. 내부 공정 ID는 기존 `spray/weld/paint`를 유지한다.

```text
RuntimeSelection:
  schema_version=2
  execution_mode: simulation | mixed | real
  platform: commercial | rb10
  active_arm: left | right | single
  process: spray | weld | paint
  tools: {left: ToolId, right: ToolId} 또는 {single: ToolId}
ToolId: spray_v2 | hongik_v4 | roller
RuntimeRequest: {selection: RuntimeSelection, connections: object, expected_revision: int}
RuntimeState: {revision: int, desired: RuntimeSelection, applied: RuntimeSelection|null,
              phase: idle|stopping|building|starting|ready|failed,
              asset_revision: string|null, capabilities: list, error: object|null}
ArmContract: {model_id, joints[6], group, base_frame, flange_link, tcp_link,
              controller, wrench_frame, sensor_joint, contact_link, tool_revision}
AssetBundle: {revision, directory, selection, arms, urdf, physics_urdf, srdf,
              usd, controllers, tool_profiles, source_hashes}
```

`connections`는 기존 로봇 IP/Outpost 장치 선택/보정 파일 설정을 구조화한다. 허용 필드는 서버 스키마로 제한하며 임의 명령은 받지 않는다. real에서 `tools`는 장착 요구 사항이며 물리적 자동 교체 성공을 뜻하지 않는다.

신규 API: `GET/POST /runtime/configuration`, `POST /runtime/apply` (`expected_revision` 필수). 기존 `/configuration`, `/prepare-system`, `/shutdown-system`은 유지한다. 레거시 프로필로 정확히 표현할 수 없는 새 구성을 기존 `/configuration`로 덮어쓰는 요청은 409로 거부한다. 레거시 수정도 같은 revision을 증가시킨다. `/status`에 `runtime_configuration: RuntimeState`를 추가한다.

## 작업 순서와 검증 환경

**구현 전 시뮬레이션 종료 상태 확인(과거 기록, 2026-10-15):** 로컬 프로세스 목록에서 Isaac Sim,
시뮬레이션 실행기, MoveIt·ros2_control·rosbridge 등 관련 ROS 프로세스가 없음을 확인했다.
`http://127.0.0.1:8081/status`는 연결 거부로 관리 서버도 종료 상태다.
확인 시 이미 종료되어 있어 추가 종료 명령은 필요하지 않았다. 운영 자산 전환 완료를 뜻하지 않는다.

**적용 시 운영 전환 순서:** 종료 상태에서 최신 URDF/SRDF/USD·EOAT 프로필과
H_STEEL_USD 충돌 장면을 생성하고, Hongik TCP에 맞게 용접 프로필·용접선을 갱신한다.
좌우 장착 형상·TCP·충돌 모델의 일관성을 검증한 다음 Isaac/ROS/관리 서버를 재시작한다.
재시작 후 상태와 카메라 시야·도달성을 확인하며, 이전 대상·경로·실행 요청은 재사용하지 않는다.
레이저 센서 형상·CAD 프레임 포함과 실제 센서 데이터 수신 구현 상태는 별도로 표시한다.

**사전 자산 도입(완료):** 두 ZIP을 원본 그대로 보관하고 해시를 기록했다.
`prepare_construct_assets.prepare(..., source_model='hongik_v4')`와
`prepare_construct_runtime.prepare(..., urdf_path=...)`로 오른팔 Hongik/왼팔 spray
후보를 생성할 수 있다. `scripts/prepare_h_steel.py`는 원본 USD를 참조하는 정적
장면과 동일 좌표의 충돌 삼각형을 만든다. 관련 테스트 16개 통과. 결과는
`.runtime/imports/hongik_v4_hsteel/`이며 운영 `isaac_assets`를 교체하지 않았다.
위 결과는 구현 전 도입 단계의 기록이다. 이후 양팔 공구 교체·새 용접 프로필·공통
런처를 구현했으며 최신 검증 범위는 앞의 체크포인트를 따른다. 상세: `docs/HONGIK_H_STEEL_IMPORT.md`.

의존 순서: **1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11**.
묶음 A(1–4)는 공구 선택/재시작, B(5–9)는 실제 실행 및 혼합 입력, C(10–11)는 UI/종합 검증이다. 단계별로 검증하되 B의 세 공정을 누락한 채 최종 완료로 표시하지 않는다.

초기 Python 테스트는 `source scripts/construct_ros_env.sh` 후 `/usr/bin/python3 -m pytest`를 사용한다. RB 패키지가 필요한 경우 `.runtime/rb20_painting/install/{rbpodo_moveit_config,rbpodo_description,eoat_description}`를 `AMENT_PREFIX_PATH`에 추가한다. Task 4 이후에는 신규 `scripts/platform_ros_env.sh`의 통합 overlay를 사용한다. JavaScript는 `node --test`를 사용한다. 새 테스트를 먼저 실패시킨 뒤 구현한다. 각 작업의 마지막 커밋은 해당 작업 파일만 포함한다. Git 작성자 정보가 없으면 임의 신원을 만들지 말고 변경을 보존해 보고한다.

---

### Task 1: 실행 모드와 팔/공정 선택 계약

**Files:** 신규 `runtime_selection.py`, 신규 `src/sketch_control/test/test_runtime_selection.py`; 수정 `robot_models.py`, `process_supervisor.py`.
**Interfaces:** `normalize_selection(payload: dict, previous: RuntimeSelection | None) -> RuntimeSelection`; `capabilities(selection: RuntimeSelection) -> list[dict]`; `legacy_to_selection(options: dict) -> RuntimeSelection | None`. dataclass/enum으로 위 공통 계약을 정의한다.

- [x] **1. 실패 테스트 작성:** `test_process_selects_only_active_arm_tool`은 왼팔 paint 선택 결과가 `left=roller,right=hongik_v4`인지 확인한다. `test_modes_are_not_inferred_from_dry_run`은 fake/dry_run/zed_preview가 simulation/mixed로 오표기되지 않음을 확인한다. `test_rb20_keeps_legacy_identity`는 RB20을 RB10으로 변환하지 않는다.
- [x] **2. 실패 확인:** `python3 -m pytest -q src/sketch_control/test/test_runtime_selection.py` — 신규 함수 미구현으로 실패해야 한다.
- [x] **3. 구현:** commercial은 left/right, rb10은 single만 허용한다. 공정→공구 매핑을 서버에서 결정하고 불일치 요청은 거부한다. capabilities는 `supported`, `reason`, `verification`을 구분한다. 상용플랫폼 실환경·실환경 용접은 기존처럼 미지원이며 미검증을 성공으로 바꾸지 않는다.
- [x] **4. 통과 확인:** 위 테스트와 `test_robot_models.py`, `test_system_api.py`, `test_rb20_painting_supervisor.py`를 실행한다. 기존 프로필 실행 플래그가 그대로인지 확인한다.
- [ ] **5. 커밋:** 선택 계약과 그 테스트만 포함해 `feat: separate runtime environment platform arm and process`.

### Task 2: 세 EOAT를 양팔/RB10에 장착하는 모델 생성

**Files:** 신규 `eoat_catalog.py`, 신규 `src/sketch_control/config/eoat_catalog.json`, 신규 `scripts/prepare_platform_assets.py`, `scripts/import_platform_usd.py`, 신규 `tests/platform_runtime/test_tool_mounts.py`; 수정 `scripts/prepare_construct_assets.py`, `prepare_construct_runtime.py`, `prepare_rb20_painting_assets.py`.
**Interfaces:** `load_tool(tool_id: str, root: Path) -> ToolDefinition`; `mount_tool(robot: Element, arm: str, tool: ToolDefinition) -> ArmContract`; `generate_assets(root: Path, selection: RuntimeSelection, output: Path) -> AssetBundle`. ToolDefinition은 원본 메시/관성, flange 변환, TCP, 도구축, 센서/접촉 링크, 교정 여부를 가진다. USD importer CLI는 `--urdf PATH --output PATH`로 고정하고 기존 importer의 설치된 Isaac API를 재사용한다.

- [x] **1. 실패 테스트 작성:** left/right/single × 세 공구를 생성해 링크 이름 유일성, SRDF 참조 유효성, TCP 변환을 확인한다. 양팔 roller/roller도 검사한다. `test_weld_tip_applied_once`는 Hongik 컨택트 팁 TCP에 예전 0.150 m 오프셋이 붙지 않음을 확인한다. 별도 와이어 돌출 설정은 노즐-팁 차이 1.2 mm를 반영해 한 번만 적용한다. `test_replacement_removes_previous_tool_links`는 spray 전환 후 이전 공구와 카메라 링크가 제거됨을 확인한다.
- [x] **2. 실패 확인:** `python3 -m pytest -q tests/platform_runtime/test_tool_mounts.py`.
- [x] **3. 구현:** 롤러는 기존 EOAT/AFT200/D405 xacro 하위 트리를 재사용한다. 뿜칠은 `assets/construct/spray_v2/mount.json`, 플랫폼/용접은 `assets/construct/hongik_v4/source/`를 사용한다. Hongik 도구와 TCP가 link6의 서로 다른 고정 가지에 연결된 점을 보존한다. `weld_sim_profile.json`의 150 mm 시험 공구를 새 프로필로 대체하고 native URDF 공구 충돌을 검증하도록 실행기를 연결한다. 손목 접합 좌표를 모델별로 명시하고 강체 변환으로 이동한다. AFT 센서 조인트는 physics URDF에서 반력 측정이 가능하도록 보존한다. 원본 파일을 덮어쓰지 않는다. 전체 자산 생성은 importer가 성공하고 USD가 생성된 뒤에만 AssetBundle을 반환한다. CPU 기하 테스트는 별도 내부 URDF 생성 함수를 사용하며 USD 통합 검증은 Task 4에서 수행한다.
- [ ] **4. 통과 확인:** 신규 테스트 및 `tests/construct/test_hongik_import.py`, `test_h_steel_import.py`, `test_assets.py`, `test_spray_v2_mount.py`, `test_weld_tool.py`, `tests/rb20_painting/test_assets.py`. 새 기본은 left spray/right Hongik이며 오른팔 D435가 없다. 기반 메시 의존성과 기존 생성기 호환 테스트는 유지하되 이전 공구를 최신 기본 구성으로 노출하지 않는다.
- [ ] **5. 커밋:** `feat: generate arm independent EOAT assemblies`.

### Task 3: 자산 저장과 설정 적용 트랜잭션

**Files:** 신규 `runtime_apply.py`, 신규 `src/sketch_control/test/test_runtime_apply.py`; 수정 `process_supervisor.py`, `system_api.py`.
**Interfaces:** `RuntimeApplyManager.configure(request: RuntimeRequest) -> RuntimeState`; `async apply(expected_revision: int) -> RuntimeState`; 주입 가능한 `build(selection, directory)->AssetBundle`, `stop()`, `start(bundle)` 콜백을 사용한다.

- [x] **1. 실패 테스트 작성:** 두 클라이언트가 같은 revision으로 수정하면 두 번째 요청은 409. 실행/정지 중 교체는 거부. 생성 실패 시 이전 자산이 남고 `applied`가 새 값이 되지 않음. 프로세스 시작 실패 시 남은 자식 프로세스가 정리됨. 재접속 시 desired/applied가 유지됨을 확인한다. 실제 ROS 대신 기존 테스트의 disposable subprocess를 사용한다.
- [x] **2. 실패 확인:** `python3 -m pytest -q src/sketch_control/test/test_runtime_apply.py`.
- [x] **3. 구현:** `.runtime/platform/assets/<content-hash>/`에 불변 자산 묶음을 만들고 임시 디렉터리 검증 후 원자적으로 rename한다. 해시는 공구·모델·생성기 버전·장착 설정을 포함한다. 카메라 교정 변경은 runtime/scene revision도 변경한다. `.runtime/platform/configuration.json`에 버전과 desired/applied를 원자 저장한다. 관련 프로세스가 실제 동일 revision으로 준비된 후에만 ready로 전환한다.
- [x] **4. 통과 확인:** 신규 테스트 및 `test_system_api.py`, `test_isaac_supervisor.py`. 선택 팔·공구 변경 시 이전 작업영역/경로/힘 명령 식별자가 무효화됨을 검증한다. 준비 실패 후 실제 로봇 실행기로 fallback하지 않아야 한다.
- [ ] **5. 커밋:** `feat: apply runtime configurations transactionally`.

### Task 4: 공통 시뮬레이션 런처와 비접촉 공정 실행

코드 구현과 단위 회귀는 완료했다. 3/4번 항목에 포함된 라이브 시야·도달성·계획/실행/취소 판정은 진행 중이므로 해당 복합 항목은 미체크다.

**Files:** 신규 `scripts/platform_simulation_app.py`, `run_platform_simulation.sh`, `build_platform_sim.sh`, `platform_ros_env.sh`, 신규 `src/sketch_control/launch/platform_simulation.launch.py`; 수정 `scripts/import_construct_usd.py`, `isaac_sim_construct.py`, `construct_weld_scene.py`, `construct_moveit.launch.py`, `robot_models.py`, `weld_input_node.py`, `weld_contract.py`, `process_supervisor.py`; 신규 `tests/platform_runtime/test_launch_contract.py`.
**Interfaces:** 런처 인자 `assets:=<bundle.directory>`, `active_arm:=left|right|single`, `execution_mode:=simulation|mixed`, `process_mode:=spray|weld|paint`를 공통화한다. CLI `scripts/run_platform_simulation.sh --assets DIR --active-arm ARM --mode MODE --process PROCESS`는 고정 인자만 받아 실행한다.

- [x] **1. 실패 테스트 작성:** commercial 양팔과 rb10 single에서 선택한 six-joint controller/TCP/base_frame을 사용하는지 검사한다. weld 입력의 `World` 하드코딩을 제거해 RB10 `link0`도 지원한다. 이미 생성된 용접 공구에 `WeldTorch`가 중복 생성되지 않아야 한다. simulation/mixed 자산에 실장비 hardware plugin이 없음을 확인한다.
- [x] **2. 실패 확인:** `python3 -m pytest -q tests/platform_runtime/test_launch_contract.py`.
- [ ] **3. 구현:** 기존 Construct/USD importer와 RB20 센서·장면 구현에서 필요한 공통 함수를 추출해 재사용한다. 새 overlay는 Construct, RB10/RB20, 센서·admittance 패키지를 함께 빌드한다. 루트/조인트/공구/카메라 참조는 AssetBundle에서 읽는다. 용접·뿜칠 시뮬레이션은 `prepare_h_steel.py`의 USD와 충돌 메시를 동일 배치로 사용하고 기존 beam/support 박스와 testbed 충돌은 제거한다. H형강 위치와 카메라 시야·로봇 도달성을 확인해 배치를 명시하며, 예전 용접 fixture는 사용하지 않는다. 비활성 팔과 헤드는 기존 상태를 유지하고 명령을 전달하지 않는다. 기존 명령/상태 topics 및 실행기 경로를 재사용한다.
- [ ] **4. 통과 확인:** 신규 테스트와 `test_construct_executor_contract.py`, `test_weld_launch.py`, `test_weld_executor.py`. Isaac에서 세 팔 대상의 spray/weld 기본 경로 계획·실행·취소를 확인하고 arm/model/tool revision과 추종 기록을 저장한다. 이 단계의 시험용 용접선은 simulation 검증에만 사용한다.
- [ ] **5. 커밋:** `feat: run selected platform arm and tool in Isaac`.

### Task 5: 상용로봇 플랫폼 양팔 및 RB10 롤러 힘 제어

4번의 C++/Python 단위 회귀는 통과했으나, 두 획 실측 힘 유지와 UI 중단·후퇴는 현재 라이브 검증 진행 중이다.

**Files:** 수정 `src/isaac_painting_hardware/include/isaac_painting_hardware/state_contract.hpp`, `system.hpp`, `src/system.cpp`, `test/test_state_contract.cpp`; 수정 `scripts/rb20_painting_sensors.py`, `platform_simulation_app.py`, `platform_simulation.launch.py`, `moveit_executor.py`; 신규 `tests/platform_runtime/test_roller_contract.py`.
**Interfaces:** `StateContract(session, revision, expected_joints)`는 활성 팔의 6축을 받되 기존 RB 기본값을 유지한다. 각 팔의 state/command topic은 namespace로 구분하고 ArmContract의 sensor_joint/wrench_frame/contact_link를 사용한다. 기존 State/Command 메시지의 six-axis payload는 유지한다.

- [x] **1. 실패 테스트 작성:** 왼팔 상태를 오른팔 제어기가 거부하는지, 명령 순서/센서 좌표 변환이 맞는지, 이전 session/revision 명령·오래된 반력이 거부되는지 검사한다. `test_only_active_roller_target_pair_is_allowed`는 해당 롤러/작업면 쌍만 접촉 허용되고 비활성 팔은 허용되지 않음을 확인한다.
- [x] **2. 실패 확인:** 새 overlay에서 `colcon --log-base .runtime/platform/log test --build-base .runtime/platform/build --install-base .runtime/platform/install --packages-select isaac_painting_hardware` 및 `python3 -m pytest -q tests/platform_runtime/test_roller_contract.py`를 실행한다. C++ 결과는 `colcon test-result --test-result-base .runtime/platform/build/isaac_painting_hardware --verbose`로 확인한다.
- [x] **3. 구현:** 기존 RB20 반력 측정, 중력 보상, tare, admittance/JTC 전환을 선택한 팔에 연결한다. 기존 force monitor의 `ft_frame`/`tcp_frame` 파라미터를 ArmContract에서 공급한다. 실행기의 전역 롤러 링크 상수 사용은 ArmContract 조회로 바꾸고 레거시 기본값은 유지한다. 기존 비영 힘 허용 조건(`real_painting_enabled=true`, `dry_run=false`, `painting_force_enabled=true`)을 시뮬레이터 전용 hardware plugin 확인 후 적용한다. 이 레거시 플래그로 UI 실행 모드를 추론하지 않는다. 반력을 목표값으로 합성하거나 센서 유효성 검사를 해제하지 않는다. 중단·접촉 해제 시 원래 ACM을 복원한다.
- [ ] **4. 통과 확인:** C++ 패키지 테스트, `tests/rb20_painting/test_sim_contract.py`, `test_contact_collision_acm.py`, 신규 테스트. Isaac 양팔/단일팔에서 기본 20 N, hold band 20–30 N, 두 PAINT 구간, 접근→tare→탐색→가압→감압→후퇴를 확인한다. 기존 판정대로 각 획 첫 1초 제외 후 최소 5초의 95% 이상이 18–32 N이어야 한다. UI 중단도 검증한다.
- [ ] **5. 커밋:** `feat: support measured roller control on selected arms`.

### Task 6: 실제 카메라 정합과 시각 처리

5번의 단위/Outpost 계약 회귀는 통과했다. 실제 카메라 수신 검증과 커밋은 별도 미완료다.

**Files:** 신규 `camera_registration.py`, `mixed_frame_sync.py`, 신규 `src/sketch_control/test/test_mixed_camera.py`; 수정 `outpost_bridge_node.py`, `robot_models.py`, `src/sketch_control/launch/rb10_painting_system.launch.py`, `scripts/construct_perception.launch.py`, `platform_simulation.launch.py`.
**Interfaces:** `CameraRegistration(camera_id, serial, optical_frame, target_frame, translation_m, quaternion_xyzw, revision, pose_source)`; `FrameBinding(source_session, source_frame_id, capture_stamp, receipt_monotonic, sim_stamp, clock_epoch, registration_revision)`; `MixedFrameSync.bind(frame, ros_now_ns, steady_now, epoch) -> FrameBinding`.

- [x] **1. 실패 테스트 작성:** 실제 RGB/depth/CameraInfo가 동일 source frame에 묶이는지, 중복/역순/재연결 프레임과 pause/reset 이전 프레임이 거부되는지 검사한다. 실제 D405가 고정된 상황에서 가상 팔 이동으로 camera TF가 바뀌지 않아야 한다. 잘못된 serial/보정 revision/누락 pose는 명시 오류여야 한다.
- [x] **2. 실패 확인:** `python3 -m pytest -q src/sketch_control/test/test_mixed_camera.py`.
- [x] **3. 구현:** Outpost 프레임 계약을 유지하고 mixed에서 실제 카메라 장치만 선택한다. 원본 capture 시각/프레임 ID와 수신 steady 시각을 보존하며, 유일한 새 프레임만 현재 ROS clock에 바인딩한다. 시뮬레이션 clock reset은 epoch를 변경하고 캐시/계획을 폐기한다. pause 중 수신 프레임은 실행 입력으로 승격하지 않는다. 기존 freshness 한계를 완화하지 않는다.
- [x] **4. 정합 연결:** 고정 실카메라는 선택한 가상 로봇 기준으로 측정한 정적 보정을 사용한다. 실제 D405는 보정된 고정 거치 또는 촬영 시점의 실제 자세가 필요하다. 가상 손목 TF만으로 실제 D405를 해석하지 않는다. mixed에서는 실제 로봇 드라이버를 시작하지 않고 물리 카메라를 움직이는 명령도 보내지 않는다.
- [ ] **5. 통과/커밋:** 신규 테스트와 `test_outpost_integration.py` 통과 후 `feat: bind physical camera frames to simulation sessions`. 실제 카메라 수신 확인은 별도 장비 결과로 기록한다.

### Task 7: 실제 측정 작업면의 공통 장면 반영

4번의 단위 기하/ACK/ACM 회귀는 통과했다. 실제 카메라 입력별 양 엔진 적용 기록은 아직 통합 증거가 필요하다.

**Files:** 신규 `task_scene.py`, `task_scene_node.py`, 신규 `scripts/platform_task_scene.py`, 신규 `tests/platform_runtime/test_task_scene.py`; 수정 `src/sketch_control/setup.py`, `platform_simulation_app.py`, `platform_simulation.launch.py`, 작업면 전달부가 있는 `moveit_executor.py`.
**Interfaces:** `TaskSurface(surface_id, revision, frame_id, vertices_m, faces, normal, frame_binding)`; `make_surface(measurement, registration) -> TaskSurface`; `/task_scene/request`와 `/task_scene/status`의 JSON에 surface revision 및 Isaac/MoveIt 각각의 적용 ACK를 포함한다.

- [x] **1. 실패 테스트 작성:** 알려진 보정으로 측정면을 옮겼을 때 ROS/Isaac 꼭짓점·법선·단위가 일치하는지 확인한다. 한쪽 적용 실패/지연 또는 기존 revision ACK로 준비 상태가 열리지 않아야 한다. 표면 뒤집힘·빈 geometry·NaN은 거부한다.
- [x] **2. 실패 확인:** `python3 -m pytest -q tests/platform_runtime/test_task_scene.py`.
- [x] **3. 구현:** 원본 측정 영역을 유한한 면으로 생성한다. 접촉용 뒤쪽 두께를 추가할 경우 측정된 전면 위치를 유지하고 두 엔진에 동일하게 적용한다. 작업 중 장면 수정은 거부한다. 양쪽 ACK 후만 경로 검증을 허용하고 변경 시 이전 경로/접촉 허용을 무효화한다. 여기서 생성하는 것은 측정 작업면이며 구현되지 않은 장애물 인식을 주장하지 않는다.
- [ ] **4. 통과 확인:** 신규 테스트와 `test_zed_spray_projection.py`, `test_contact_collision_acm.py`. 실제/녹화 입력별 보정 변환·표면 revision·두 엔진 ACK를 기록한다.
- [ ] **5. 커밋:** `feat: synchronize measured task surfaces with Isaac and MoveIt`.

### Task 8: 실제 RGB-D에서 지정한 용접선 입력

4번 체크는 Python/JS 계약 회귀에 한한다. 실제 장비의 카메라 frame ID와 실행 보고서는 아직 수집되지 않았다.

**Files:** 신규 `rgbd_weld_provider.py`, `rgbd_weld_provider_node.py`, 신규 `src/sketch_control/test/test_rgbd_weld_provider.py`; 수정 `weld_input_node.py`, `weld_contract.py`, `src/sketch_control/setup.py`, `web/js/weld_workflow.js`, `web/test/weld_workflow.test.js`.
**Interfaces:** `project_seam(polyline_px, depth_m, intrinsics, frame_binding, registration) -> SeamCatalog`. `/welding/extraction_request`에 선택 프레임에 대한 사용자 polyline을 추가하고, 기존 `/welding/extraction_result`로 request/target/generation 식별자가 일치하는 결과를 반환한다. 기존 fixture 요청 스키마는 유지한다.

- [x] **1. 실패 테스트 작성:** 일정 깊이 평면의 두 픽셀을 역투영한 3D 위치/방향을 수치 검증한다. depth 누락, 불연속 구간, 선택 프레임 만료, target 변경 후 늦은 결과를 거부한다. mixed에서는 fixture 결과가 수락되지 않아야 한다. RB10의 기준 프레임도 확인한다.
- [x] **2. 실패 확인:** `python3 -m pytest -q src/sketch_control/test/test_rgbd_weld_provider.py`와 `node --test web/test/weld_workflow.test.js`.
- [x] **3. 구현:** 사용자가 실제 영상 위에 용접 진행선을 지정하고 같은 RGB-D 프레임으로 역투영한다. 선 방향과 선택된 인접 작업면 법선을 기존 Seam의 reference_up/자세 입력으로 전달한다. 접합면 정보가 없어 자세를 확정할 수 없으면 사용자가 공정 자세를 지정하게 하며 자동 인식 성공으로 꾸미지 않는다. 기존 순서·방향·위빙 UI를 유지한다.
- [x] **4. 통과 확인:** 신규 테스트, `test_weld_input.py`, `test_weld_contract.py`, `test_weld_session.py`, `web/test/weld_transport.test.js`. 실제 장비 확인 시 시험 fixture가 아닌 카메라 프레임 ID가 보고서에 남아야 한다.
- [ ] **5. 커밋:** `feat: drive welding from user selected physical RGBD seams`.

### Task 9: 세 공정의 일부 시뮬레이션 통합

**Files:** 수정 `platform_simulation.launch.py`, `platform_simulation_app.py`, `process_supervisor.py`, 기존 인식 런치 `src/sketch_control/launch/rb10_perception_sketch.launch.py`; 신규 `src/sketch_control/test/test_mixed_runtime.py`.
**Interfaces:** Task 1의 mixed 선택이 Task 6의 실제 카메라 + Task 7의 측정 장면 + Task 4/5의 시뮬레이터로 해석된다. 공정별 인식기는 기존 spray/paint 경로 또는 Task 8의 external weld provider다.

- [x] **1. 실패 테스트 작성:** 3개 활성 팔 대상(commercial left/right, rb10 single) × 3개 공정의 생성 프로세스 구성을 검사한다. 실제 robot hardware plugin, 가상 카메라 publisher, fixture weld provider가 없고 공정별 실제 카메라/시뮬레이션 제어기가 존재해야 한다. paint는 D405 정합과 force_pipeline이 필요하다.
- [x] **2. 실패 확인:** `python3 -m pytest -q src/sketch_control/test/test_mixed_runtime.py`.
- [x] **3. 구현:** 상용플랫폼 런처의 virtual ZED 고정과 일반 런치의 native backend 강제 결합을 명시적 mode 정책으로 분리한다. 카메라 종류만 바꾸고 기존 경로 생성기를 복제하지 않는다. paint의 실제 D405 촬영은 고정/실측 pose에서 수행하며 자동 ‘가상 팔 이동→실카메라 접근 완료’ 처리를 금지한다. 부족한 준비 항목은 이유와 함께 기다리게 한다.
- [x] **4. 통과 확인:** 신규 테스트, `test_construct_perception.py`, `test_painting_system_config.py`, `test_spray_motion_test.py`, `test_rb20_painting_launch.py`. 각 조합의 준비→계획→실행→중단 연결은 Task 11에서 측정한다.
- [ ] **5. 커밋:** `feat: support mixed camera simulation for all three processes`.

### Task 10: 8081 관리 UI와 기존 스케치 화면 연결

4번 브라우저 증거는 disposable API fixture 서버의 UI/상태 검증이다. 실제 Isaac/ROS 공정 실행을 검증한 스크린샷으로 해석하지 않는다.

**Files:** 수정 `web/system.html`, `web/js/system.js`, `web/index.html`, `web/js/app.js`, `web/js/weld_workflow.js`; 신규 `web/js/runtime_settings.js`, `web/test/runtime_settings.test.js`; 수정 `web/test/system_process_mode.test.js`.
**Interfaces:** `normalizeForm(state)`, `selectionFromForm(form)`, `renderRuntimeState(state)`는 Task 1/3 스키마를 그대로 사용한다. 서버 capabilities가 조합 허용 여부의 기준이다.

- [x] **1. 실패 테스트 작성:** 세 실행 모드/두 플랫폼/세 공정의 정확한 한국어 라벨과 상용플랫폼에서만 작업 팔 표시를 검사한다. 공정 변경 시 활성 팔만 자동 장착 변경, 미적용 설정 표시, 중복 적용 클릭 차단, 409 후 최신 상태 표시, 재접속 복원을 검사한다. RB20 상태를 RB10으로 표시하지 않는다.
- [x] **2. 실패 확인:** `node --test web/test/runtime_settings.test.js web/test/system_process_mode.test.js`.
- [x] **3. 구현:** 기본 화면은 실행 모드→로봇 플랫폼→작업 팔→공정→자동 EOAT 요약 순서다. 주소/토큰/프로필/개별 프로세스/로그는 고급 설정으로 이동한다. mixed/real에서는 실제 카메라 선택과 보정 상태를 접근 가능하게 둔다. 설정 적용/재시작은 Task 3 API를 사용하고 단순 선택만으로 서버를 재시작하지 않는다. 적용 완료 후 스케치 화면의 기존 작업 정보를 무효화한다.
- [x] **4. 통과 확인:** 신규 테스트와 `node --test web/test/*.test.js`. 브라우저에서 기본/고급 화면, 시작/종료/실패, 로딩 중 중복 클릭, 좌우 팔 변경 및 세 공정 흐름을 확인하고 스크린샷을 저장한다. 사용자 수정 중인 공통 3단계 workflow를 덮어쓰지 않는다.
- [ ] **5. 커밋:** `feat: simplify runtime UI around environment platform and process`.

### Task 11: 전체 조합 검증과 사용자 문서

문서와 evidence verifier/probe는 작성했다. 4번 전체 18조합은 현재 라이브 검증 진행 중이며, 5번 문서/커밋 복합 항목은 커밋 미수행으로 남긴다.

**Files:** 신규 `scripts/check_platform_runtime.py`, `tests/platform_runtime/test_report_contract.py`, `docs/PLATFORM_RUNTIME.md`; 수정 `docs/CONSTRUCT_ISAAC_SIM.md`, `SPRAY_MOTION_TEST.md`, `WELDING_ISAAC.md`, `RB20_PAINTING_ISAAC.md`, 설계 문서의 구현 상태.
**Interfaces:** CLI `--mode simulation|mixed --platform commercial|rb10 --arm left|right|single --process spray|weld|paint --output PATH`; 기본은 읽기/준비 검사만 수행하고 실제 시뮬레이션 작업 실행에는 `--execute`가 필요하다. real 실행은 이 검증기에서 지원하지 않는다.

- [x] **1. 실패 테스트 작성:** 보고서가 명령 성공과 관절 실측 추종을 구별하고 fixture/녹화/실카메라 출처를 명시하는지 검사한다. Task 5의 힘 판정 조건, 오래된 입력, 중단 미완료, 다른 revision을 실패로 판정한다. 카메라 미연결은 `not_run`으로 기록하고 passed로 집계하지 않는다.
- [x] **2. 실패 확인:** `python3 -m pytest -q tests/platform_runtime/test_report_contract.py`.
- [x] **3. 구현/회귀:** 보고서에 선택 구성, asset/tool/scene/calibration revision, 카메라 ID·프레임 출처, 제어기·관절 추종, 법선/이격, 롤러 반력, 용접 위빙, 취소 결과를 저장한다. Task 1–10 관련 Python/C++/JS 테스트를 실행하고 기존 RB20 및 물리 실행 계약 회귀를 확인한다.
- [ ] **4. 통합 검증:** simulation/mixed 각각 `(commercial:left, commercial:right, rb10:single) × (spray,weld,paint)`의 **총 18개 실행 조합**을 확인한다. 같은 공구 양팔 장착과 교체 후 기존 경로 거부도 검사한다. 실제 카메라가 없는 mixed 항목은 완료로 처리하지 않고 장비 필요 항목을 구체적으로 남긴다. 가상 작업면만 있는 시험을 mixed 실장비 검증으로 바꾸지 않는다.
- [ ] **5. 문서/커밋:** 실행 모드 설명, 카메라 보정/고정 거치, 공구 교체, 각 공정 실행, 지원하지 않는 실환경 조합, 실패 복구를 문서화하고 `docs: document verified platform runtime combinations`로 해당 검증/문서 변경을 기록한다.

## 완료 판정과 자체 검토

설정 UI나 모델 생성만 완료되어도 전체 완료라고 보고하지 않는다. **18개 조합의 코드 경로 구현**, 상용플랫폼 양팔 롤러 힘 제어, 실제 카메라에서 지정한 용접선, 공구 교체 후 경로 무효화가 구현 범위다. 장비 없는 경우 코드 검증과 실제 카메라 통합 검증을 분리해 미완료 항목을 남긴다.

| 설계 요구 | 구현 작업 |
| --- | --- |
| 새 UI 분류/용어·레거시 보존 | 1, 3, 10 |
| 양팔·RB10의 EOAT/TCP/충돌 일치 | 2, 4 |
| 적용/재시작·캐시·실패 복구 | 3 |
| 상용플랫폼 양팔 롤러 힘 제어 | 5, 7, 11 |
| 세 공정의 실제 카메라 + 가상 로봇 | 6–9, 11 |
| D405 실제 자세·시각 처리 | 6, 7, 9 |
| 실제 영상 용접선과 기존 위빙 | 8, 9 |
| 지원 상태와 실제 검증 구분 | 1, 10, 11 |

자체 검토: 공통 인터페이스 명칭과 Task 간 의존 순서를 대조했다. Review Focus의 다섯 조건을 각 작업의 테스트에 배정했다. 외부 카메라 장비와 보정은 통합 검증의 의존 사항이며 임의 값으로 통과시키지 않는다. 이 문서는 최초 계획과 이후 구현 체크포인트를 함께 보존한다. 미체크 라이브 기준은 단위 테스트로 대신하지 않는다.

## 실행 방식 선택

사용자 승인 후 같은 세션의 분리 worktree에서 영역별 구현을 진행했다. 공구·팔·센서·장면 revision은 공통 계약으로 연결하고, 기존 사용자 변경을 보존했다. 새 구현/테스트는 위 체크포인트에 기록하고 라이브 통합 결과는 실제 수집 뒤 갱신한다.
