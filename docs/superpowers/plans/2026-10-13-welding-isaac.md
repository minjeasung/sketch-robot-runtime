# Shared Settings and Isaac Welding Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 기존 ZED 스케치 UI와 공통 Settings 패널에서 선택한 3D 용접심을 Isaac EOAT가 자세·위빙 조건에 맞춰 실행하게 한다.

**Architecture:** 공식 Michelo/Outpost와 기존 카메라·MoveIt·컨트롤러 경로를 유지한다. 용접 데이터 계약, 기하 계산, 상태, ROS 연결을 작은 모듈로 분리하고 기존 executor에는 위임 지점만 추가한다. 시험 심 제공자와 시뮬레이션 공정 출력은 향후 실제 어댑터와 같은 계약을 사용한다.

**Tech Stack:** Python 3, NumPy/SciPy, ROS 2 Jazzy, MoveIt, FollowJointTrajectory, 기존 Isaac Sim 6.1 Construct 환경, 기존 HTML/CSS/JavaScript·roslib, pytest, node:test.

**Spec:** `docs/superpowers/specs/2026-10-13-welding-isaac-design.md` (사용자가 2026-10-13 구현 요청으로 승인함).

## 구현 검증 기록 (2026-10-13)

직접 순차 구현과 독립 리뷰를 진행했다. 실제 반영 방식과 예외는 `.superpowers/sdd/2026-10-13-welding-isaac/progress.md`에 기록했다. 기존 실행기의 도장 전용 force/ACM 초기화에 의존하지 않도록, 같은 모델·MoveIt·FJT 계약을 사용하는 용접 전용 비동기 어댑터를 조합했다. 작업별 커밋은 Git 작성자 정보가 없어 생성하지 않았으며 기존 staged 파일을 보존했다.

브라우저 제어 도구에는 사용 가능한 브라우저가 없어 실제 화면 배치 검사는 보류했다. 기존 UI의 자동 이벤트 검사와 공개 ROS 인터페이스를 통한 Isaac 실제 이동 검증으로 동작을 확인하며, 브라우저 육안 검사를 했다고 주장하지 않는다.

## Global Constraints

- 공식 Michelo·Outpost 및 minjeasung 스케치 UI를 그대로 활용한다.
- 작업대상 스케치는 ZED 영상 위에서 한다. 스케치를 용접 경로로 사용하지 않는다.
- 추출 알고리즘은 개발 중이며 용접심을 3D 좌표로 제공할 예정이다.
- 기존 스케치 UI에 Settings 패널을 추가한다. 용접뿐 아니라 도장·뿜칠에서도 사용할 공통 패널이다.
- ZED 영상 위에 용접심 선을 표시하지 않아도 이 흐름이 완결되어야 한다.
- 첫 범위에서 도장·뿜칠의 새로운 실행 중 설정 API를 만들지는 않는다.
- 1차 구현은 실제 용접기 드라이버나 물리적인 아크·열·용입 시뮬레이션을 포함하지 않는다.
- 위치 오차 5 mm, 자세 오차 5 deg 이내를 초기 시뮬레이션 추종의 통과 기준으로 사용하며 실용접 정확도 기준으로 주장하지 않는다.
- 속도·위빙 주파수는 최종 궤적에서 요청 대비 5% 이내여야 한다. 시뮬레이션 시간으로 측정하고 느린 렌더링의 벽시계 시간과 구분한다.
- 기존 dirty 작업 트리와 staged 설계서를 보존한다. 기본 브랜치만 새 checkout하여 진행 중인 Construct 변경을 잃지 않는다. 작업별 커밋은 이번 변경만 포함한다. Git 작성자 정보가 없으면 사용자 identity를 추측하거나 전역 설정을 바꾸지 말고 검증된 변경을 남긴다.

## Review Focus

- 브라우저 재접속과 서버 재시작: 과거 설정 ACK·실행 명령이 재사용되지 않아야 한다. Task 3과 7의 세션 변경 테스트로 고정한다.
- planning 중 대상/설정/reset 변경과 늦게 온 MoveIt 응답: 과거 성공 응답이 READY를 복구하지 않아야 한다. Task 3과 6의 지연 future 테스트로 고정한다.
- quaternion q/-q, 경로 반전, 횡축 퇴화: 토치가 갑자기 뒤집히거나 임의 각도로 실행되지 않아야 한다. Task 2의 자세 테스트로 고정한다.
- 3D 결과의 시계·좌표계·단위 혼동: 최신 TF나 mm를 묵시적으로 사용하지 않아야 한다. Task 1과 4의 변환·metadata 테스트로 고정한다.
- 심 사이 이동과 중단 ACK 지연: 가상 용접은 OFF여야 하며 정지 미확인 상태에서 다음 심이나 재실행을 시작하지 않아야 한다. Task 3과 6의 lifecycle 테스트로 고정한다.

## File Structure and Shared Interfaces

아래 새 모듈은 `src/sketch_control/sketch_control/`에 둔다. 테스트는 `src/sketch_control/test/`의 같은 책임 이름을 사용한다. 기존 `rotation_utils.py`를 재사용하고 별도 수학 라이브러리를 설치하지 않는다.

| 모듈 | 책임과 공개 인터페이스 |
|---|---|
| `weld_contract.py` | immutable dataclass: TargetRequest, SeamCatalog, Seam, WeldSettings, ToolProfile, PlanIdentity, WeldSamples, PlannedSegment, WeldPlan; validate_target(payload), validate_catalog(payload, target), validate_settings(payload, profile), load_tool_profile(path, model_id); 잘못된 입력은 ValueError |
| `weld_geometry.py` | transform_catalog(catalog, transform), build_weld_samples(seam, settings, profile, reverse=False) → WeldSamples; 위치·자세·시간·중심선 진행 거리·구간 label 생성 |
| `weld_session.py` | WeldSession: request_target, accept_catalog, select, apply_settings, begin_plan, accept_plan, begin_execution, complete_segment, request_stop, confirm_stop, invalidate; immutable revision으로 결과 소유권 검사 |
| `weld_output.py` | SimWeldOutput: configure(settings), start_segment(plan_id, seam_id), stop_segment(reason), status(); 세션/구간과 생존 상태를 포함한 공정 상태 |
| `weld_input_node.py` | 기존 ZED metadata와 대상 요청을 묶어 fixture/external 제공자 결과 전달; TF 좌표 변환은 이 경계에서 수행 |
| `process_settings.py` | 공정별 서버 설정 snapshot 작성, 용접 설정 검증·revision 적용, paint/spray read-only metadata |
| `weld_execution.py` | WeldExecutionMixin: _init_weld_execution, _weld_plan, _weld_execute, _weld_abort, _weld_invalidate; 기존 MoveItExecutor를 통해 계획·컨트롤러 사용 |
| `weld_trajectory.py` | sample_joint_trajectory, validate_weld_timing, tracking_metrics; 최종 궤적 시간·FK 기반 검증과 통합 시험의 공통 계산 |
| `web/js/process_settings.js` | Settings 패널, 공정별 draft·서버 ACK와 적용 상태 |
| `web/js/weld_workflow.js` | 기존 ZED 스케치 대상 확정, 심 목록·순서·방향, 계획/실행 상태 표시 |
| `scripts/construct_weld_scene.py` | Isaac 시험 토치·경로·자세 표시. 주 simulation thread에서 USD 변경 |

WeldSamples는 `time_s[N]`, `center_distance_m[N]`, `tip_xyz_m[N,3]`, `tcp_xyz_m[N,3]`, `tcp_quat_xyzw[N,4]`, `phase[N]`를 가진다. quaternion은 TCP→catalog.frame_id 회전이다. ToolProfile은 `T_tcp_tip`, tool_axis, 작업물 reference_up, 국소 각도 기준, 충돌 형상, 제한값·시험 기본값, profile revision을 가진다. `T_world_tcp = T_world_tip @ inverse(T_tcp_tip)`를 사용한다.

PlanIdentity는 server_session_id, target_revision, catalog generation, selection_revision, settings_revision, tool_revision, scene_revision, model_id를 포함한다. WeldPlan은 plan_id, identity, 시작 전체 관절 상태, 순서 있는 PlannedSegment를 가진다. 각 segment는 seam_id, phase, 시작/종료 상태, RobotTrajectory와 대응하는 WeldSamples를 가진다.

ROS 전송은 기존 관례의 `std_msgs/String` JSON을 사용한다. 신규 command topic은 volatile이며 제어 메시지는 재접속 때 재전송하지 않는다. 상태만 transient-local로 제공하고 UI는 세션 ID가 일치하는 ACK만 사용한다.

| 신규 topic | 방향과 내용 |
|---|---|
| `/welding/target_request` | UI→입력 노드: TargetRequest |
| `/welding/extraction_request`, `/welding/extraction_result` | 입력 노드↔fixture/external 제공자: 원본 입력 계약; 외부 제공자는 이번 범위에서 연결 규격만 제공 |
| `/welding/seams` | 입력 노드→executor/UI: 검증·좌표 변환된 SeamCatalog |
| `/welding/selection`, `/welding/plan_request`, `/welding/execute` | UI→executor: command_id, server_session_id, expected revision; execute는 plan_id 필수 |
| `/welding/status` | executor→UI: 상태, ACK, identity, readiness, 오류, 현재 심과 구간 |
| `/welding/preview` | executor→Isaac 표시: identity가 붙은 3D 중심/위빙/자세 자료, invalidated면 표시 제거 |
| `/welding/output_status` | executor→UI/Isaac: 가상 용접 상태, 전류 설정, 세션/구간 식별 |
| `/process_settings/request`, `/process_settings/status` | UI↔executor: 공정별 설정과 확정 snapshot |
| `/perception/zed_frame_identity` | 기존 브리지→입력 노드/UI: 원본 image stamp, clock domain, stream generation, optical frame, image size |

기존 `/motion_abort`와 `/painting_system/process_mode`를 재사용한다. weld에서는 legacy `/sketch_execute`와 도장·뿜칠 전용 콜백이 동작을 시작하지 못하게 한다. process mode를 바꾸는 existing topic은 지원 capability를 검사하며 weld로의 스택 변경이 필요하면 기존 시스템관리 재시작 흐름을 사용한다.

## Task 1: 3D 입력과 설정 계약

**Files:** Create `weld_contract.py`, `test/test_weld_contract.py`, `config/weld_sim_profile.json` under sketch_control; Modify `setup.py` to install config only if its existing glob does not include JSON.

**Interfaces:** 위 표의 dataclass와 validate/load 함수. catalog는 위치만 있는 seam도 허용하며, 실행 가능 자세의 검사는 Task 2에서 수행한다. frozen dataclass의 배열·dict는 복사하거나 immutable로 보관한다.

- [x] 실패 테스트 작성: `test_xyz_only_seam_is_selectable`, `test_catalog_rejects_wrong_request_unit_duplicates_and_nonfinite`, `test_settings_reject_invalid_dwell_and_preserve_previous`, `test_profile_rejects_wrong_model`.
  Assertions: 두 점만 있는 seam 수신 성공; m 아닌 단위와 잘못된 revision은 ValueError; `left_dwell_s + right_dwell_s >= 1/frequency_hz` 거부; OFF 위빙에는 frequency로 나누지 않음; null/boolean 수치 거부.
- [x] `PYTHONPATH=src/sketch_control /usr/bin/python3 -m pytest src/sketch_control/test/test_weld_contract.py -q` 실행. 새 모듈 부재/요구 동작 부재로 실패 확인.
- [x] validate 함수와 dataclass 구현. polygon 범위·폐곡선, 점 수·좌표 범위·최대 payload 수량, 단위, profile 모델, quaternion 정규성, 설정 범위를 명시적으로 검증한다. 프로파일의 제한값은 구현과 테스트가 공유한다.
- [x] 동일 pytest 실행, 전부 PASS 확인. default는 시뮬레이션 예시이며 사용자가 적용하기 전 READY가 되지 않음을 검사한다.
- [ ] 이번 파일만 검토 후 `feat: define welding seam and settings contracts` 단위 커밋. 작성자 정보가 없으면 변경을 보존하고 이후 작업을 계속한다.

## Task 2: 토치 자세와 위빙 계산

**Files:** Create `weld_geometry.py`, `weld_trajectory.py`, `test/test_weld_geometry.py`, `test/test_weld_timing.py`; reuse `rotation_utils.py`.

**Interfaces:** Task 1 계약 → WeldSamples. `validate_weld_timing(samples, actual_time_s, tolerance=0.05) -> dict`는 segment 내 대응 샘플의 최종 시간으로 속도·주파수 오차를 검사한다. `tracking_metrics(planned, measured) -> dict`는 동시각 보간 후 위치·quaternion 각거리 최댓값과 위빙 폭·주파수를 반환한다.

- [x] 실패 테스트 작성: `test_tip_to_tcp_rotation_translation`, `test_reverse_recomputes_travel_angle`, `test_q_sign_does_not_flip_torch`, `test_parallel_reference_and_missing_pose_are_rejected`, `test_full_weave_width_dwell_and_ramp`, `test_retiming_over_five_percent_is_rejected`.
  Assertions: identity가 아닌 공구 변환의 역변환을 수치 확인; q와 -q의 각거리 0; amplitude=width/2; 한 주기=1/f; 횡방향 dwell 동안 중심선 전진; 기준축 퇴화 거부; 6% 시간 왜곡 거부, 4% 허용. 실제 fixture와 독립적인 기하 입력으로 검증한다.
- [x] `PYTHONPATH=src/sketch_control /usr/bin/python3 -m pytest src/sketch_control/test/test_weld_geometry.py src/sketch_control/test/test_weld_timing.py -q` 실행, 실패 확인.
- [x] 호 길이로 중심선을 샘플링하고 tangent/reference_up으로 오른손 국소 축을 생성한다. quaternion 입력은 연속 부호로 정렬하여 보간한다. 작업각은 진행축, 진행각은 횡축에 적용하고 tool profile의 도구 축으로 변환한다. 급격한 모서리·orientation 변화는 profile 한계로 거부한다.
- [x] 부드러운 왕복과 dwell의 시간 함수를 구현한다. 활성 sweep은 cosine half-wave, 좌우 정지는 plateau, 시작·종료 envelope는 smoothstep을 사용한다. 시간 간격과 공간 간격을 모두 제한하여 위빙 샘플 누락을 막고, endpoint와 centerline 직선으로 진입·이탈을 연결한다.
- [x] 동일 pytest 실행, PASS 확인. `tracking_metrics`는 같은 위치에서 잘못된 자세인 측정도 실패로 보고하는 테스트를 포함한다.
- [ ] `feat: generate orientation-aware welding and weaving paths` 단위 커밋.

## Task 3: 세션 상태와 공통 설정·가상 출력

**Files:** Create `weld_session.py`, `weld_output.py`, `process_settings.py`, `test/test_weld_session.py`, `test/test_process_settings.py`.

**Interfaces:** `WeldSession.begin_plan() -> PlanIdentity`; `accept_plan(identity, plan) -> bool`; `begin_execution(plan_id, identity) -> WeldPlan`; `invalidate(reason) -> None`. configure/settings ACK는 같은 server_session_id와 command_id를 돌려준다. preview와 output 상태에는 동일 identity를 쓴다.

- [x] 실패 테스트 작성: `test_changed_settings_reject_late_plan`, `test_restart_rejects_old_ack_and_execute`, `test_duplicate_execute_starts_once`, `test_stop_waits_for_controller_confirmation`, `test_output_off_during_transfer_and_failure`, `test_paint_spray_settings_are_current_and_read_only`.
  Assertions: 늦은 accept_plan=False; 이전 세션의 execute 거부; 두 번 클릭해도 시작 횟수 1; cancel 요청 후 STOPPING, 확인 후 STOPPED; transfer/abort에서 on=False; 변경 요청의 실패가 기존 확정 설정을 덮지 않음.
- [x] `PYTHONPATH=src/sketch_control /usr/bin/python3 -m pytest src/sketch_control/test/test_weld_session.py src/sketch_control/test/test_process_settings.py -q` 실행, 실패 확인.
- [x] 정의된 state machine과 revision 검사 구현. RUNNING/STOPPING에서는 설정·선택 변경 거부. frontend 연결이 끊겨도 stale readiness를 재사용하지 않도록 세션 snapshot과 heartbeat 상태 제공. 가상 출력 timeout은 wall monotonic으로 처리하고 시뮬레이션 pause와 controller 상태도 표시한다.
- [x] 동일 pytest 실행, PASS 확인. Settings snapshot에는 값, 단위, editable, 적용/재시작 정책, server_session_id를 포함한다. paint/spray의 현재값은 executor의 실제 선언 파라미터에서 가져온다.
- [ ] `feat: add revision-bound welding sessions and process settings` 단위 커밋.

## Task 4: ZED 대상 요청과 시험 심 제공자

**Files:** Create `weld_input_node.py`, `config/construct_weld_fixture.json`, `test/test_weld_input.py`; Modify `outpost_bridge_node.py`, `web/js/image_topic.js`, `web/test/image_topic.test.js`, `setup.py`.

**Interfaces:** `load_fixture(path, target, scene_revision) -> SeamCatalog`; `transform_catalog(catalog, T_target_source) -> SeamCatalog`. `/perception/zed_frame_identity`는 기존 RGB header와 같은 stamp/generation/size를 사용한다. fixture는 World 좌표로 제공하되 request identity를 보존한다.

- [x] 실패 테스트 작성: `test_target_uses_original_zed_frame_metadata`, `test_old_catalog_and_generation_rejected`, `test_fixture_never_runs_for_external_or_real_mode`, `test_optical_frame_requires_transform_at_capture`, `test_empty_result_stays_empty`. `image_topic.test.js`에 compressed callback이 해당 msg header를 보존하는 테스트 추가.
- [x] `source scripts/construct_ros_env.sh` 후 해당 Python 테스트 실행; `/home/minjea/robotics/node-v24.21.0-linux-x64/bin/node --test web/test/image_topic.test.js` 실행, 새 요구 실패 확인.
- [x] 기존 카메라 브리지에 identity 상태 발행을 추가하고 픽셀 요청과 매칭한다. 지원되는 clock domain과 TF 시각만 변환한다. fixture는 고정 World 좌표라 정확한 카메라 시간 동기화를 주장하지 않는다. 외부 결과에서 시계 변환 정보가 없으면 명확한 오류로 거부한다.
- [x] 기존 H형강 형상의 직선 이음부 최소 3개를 fixture로 정의한다. 원본 영역 포함관계를 검출했다고 표시하지 않는다. 곡선 자세 검증은 별도 synthetic 태그의 기하 fixture로 둔다. 카메라 헤드 이동·generation 변경·scene reset은 invalidate 발행.
- [x] 동일 테스트 실행, PASS 확인. Outpost의 카메라 ownership·원본 RGB/depth 계약 테스트도 실행한다.
- [ ] `feat: bind ZED targets to replaceable 3D seam providers` 단위 커밋.

## Task 5: Construct 공정 선택과 시험 토치 환경

**Files:** Modify `robot_models.py`, `process_supervisor.py`, `scripts/construct_executor.launch.py`, `scripts/construct_perception.launch.py`, `scripts/run_construct_executor.sh`, `scripts/run_construct_perception.sh`, `scripts/run_construct_demo.py`, `scripts/isaac_sim_construct.py`, `scripts/prepare_construct_runtime.py`, `src/sketch_control/launch/rb10_perception_sketch.launch.py`, `web/js/system.js`, `web/system.html`. Create `scripts/construct_weld_scene.py`, `test/test_weld_launch.py`, `tests/construct/test_weld_tool.py`; extend existing `test_isaac_supervisor.py` and `web/test/system_process_mode.test.js`.

**Interfaces:** `process_mode=weld`는 isaac_construct에서만 실행 허용. `weld_tool_profile`, `weld_fixture_file`, `weld_input_provider=fixture|external`는 서버가 검증한 실행 설정. 기존 spray 기본값 유지. simulation process에도 선택 모델과 공정·tool revision 전달.

- [x] 실패 테스트 작성: `test_construct_weld_launches_zed_without_d405_or_spray_generator`, `test_real_weld_fails_with_unsupported_driver`, `test_paint_spray_configuration_unchanged`, `test_isaac_and_moveit_use_same_torch_transform`, `test_preview_reset_removes_old_geometry`.
- [x] ROS 환경에서 해당 pytest와 `system_process_mode.test.js` 실행, 새 요구 실패 확인.
- [x] process mode의 2분기 처리를 세 공정으로 명시한다. weld perception에서는 기존 ZED 브리지를 실행하되 평면 기반 도장/뿜칠 generator를 실행하지 않는다. runtime 모드 전환은 capability로 제한하고 필요한 경우 전체 종료 후 설정 변경을 요구한다.
- [x] 공통 profile에서 Isaac 토치 visual/collision과 MoveIt attached collision을 생성한다. 기존 로봇 장착물 collision은 유지하고 추가 시험 토치의 장착 변환을 명시한다. 좌우 팔 중 선택한 팔에만 적용한다. task 6이 발행할 preview를 받아 Isaac main thread에서 곡선·방향 표시를 갱신하며 같은 ID로 오래된 표시를 교체한다.
- [x] 동일 테스트와 기존 Construct asset/모델/실행 설정 테스트 실행, PASS 확인. 테스트는 실행 파일의 문자열 검색만으로 대체하지 않고 실제 설정 builder의 결과·형상 변환을 검사한다.
- [ ] `feat: add welding process and explicit simulated torch profile` 단위 커밋.

## Task 6: 기존 실행기에 용접 계획·실행 연결

**Files:** Create `weld_execution.py`, `test/test_weld_executor.py`; Modify `moveit_executor.py`, `multi_surface_execution.py`; extend `weld_trajectory.py`.

**Interfaces:** WeldExecutionMixin을 기존 MoveItExecutor에 조합한다. 기존 `execute_trajectory_direct`, `_execute_trajectory_follow_joint`, `_cancel_active_follow_joint_goal`, `_request_motion_abort`, 모델 execution contract를 재사용한다. Task 1~3의 결과를 사용하며 `/welding/status`와 `/process_settings/status`를 발행한다.

- [x] 실패 테스트 작성: `test_plan_only_cannot_move`, `test_all_seams_planned_before_ready`, `test_partial_cartesian_and_collision_are_rejected`, `test_old_plan_future_cannot_restore_ready`, `test_wrong_arm_start_state_and_scene_reject_execute`, `test_abort_cancels_active_action_and_blocks_next_seam`, `test_legacy_paint_callbacks_do_not_run_in_weld`.
  Assertions: preview 중 controller call 0; fraction<1 거부; 계획된 첫 시작과 실제 전체 관절 상태 불일치 거부; 지연 future 무시; cancel 미확인 시 다음 controller call 0; D405/F/T/tare/분사 output 미사용.
- [x] `source scripts/construct_ros_env.sh` 후 `python3 -m pytest src/sketch_control/test/test_weld_executor.py -q` 실행, 실패 확인.
- [x] 계획은 MoveGroup plan_only와 GetCartesianPath 요청으로 순차 구성한다. 각 구간의 종료 상태를 다음 구간 시작으로 사용하고 모든 심·transfer를 검증한 뒤 READY로 만든다. Task 2의 TCP 자세를 waypoint에 적용한다. 현재 도장용 plan 메서드가 즉시 실행을 포함하면 그대로 호출하지 않는다.
- [x] 최종 시간·FK를 검사하고 필요한 시각의 joint trajectory를 생성한다. controller의 interpolation을 고려해 중간점도 충돌·자세·위빙 조건으로 검증한다. 위빙 시간 조건과 관절 velocity/acceleration 제한을 동시에 만족할 수 없으면 거부한다. 캐시된 계획만 실행하고 실제 start state를 재검사한다.
- [x] 기존 중단 요청을 용접 세션에 전달한다. pending MoveIt/action 응답은 token으로 처리하고 정지 ACK 전 완료를 주장하지 않는다. WELD 구간의 실제 시작/종료 feedback에 맞춰 가상 출력을 갱신한다. 오류·shutdown·heartbeat 만료는 OFF로 수렴한다.
- [x] 동일 pytest와 기존 `test_construct_executor_contract.py`, `test_spray_eoat_executor.py`, `test_painting_execution.py` 실행. 새로운 실패와 기존 baseline 실패를 구분한다.
- [ ] `feat: execute verified welding segments through the existing robot controller` 단위 커밋.

## Task 7: 기존 스케치 UI의 Settings와 용접 흐름

**Files:** Create `web/js/process_settings.js`, `web/js/weld_workflow.js`, `web/test/process_settings.test.js`, `web/test/weld_workflow.test.js`; Modify `web/index.html`, `web/sketch.css`, `web/js/app.js`, `web/js/plane_workflow.js`, `web/test/ui_harness.js`.

**Interfaces:** `window.ProcessSettings`와 `window.WeldWorkflow` 모듈을 기존 script load 순서에 연결한다. ProcessSettings의 `setSnapshot`, `setMode`, `isApplied`, `setRunning`과 WeldWorkflow의 `setStatus`, `requestTarget`, `requestPlan`, `execute`, `invalidate`가 공유 경계다. ROS command는 Task 3의 세션·revision을 사용한다.

- [x] 실패 테스트 작성: `test_settings_drafts_are_per_process`, `test_stale_ack_cannot_enable_run`, `test_zed_target_select_two_seams_then_apply_plan_execute`, `test_order_reverse_and_no_manual_weld_path`, `test_disconnect_clears_readiness_without_replaying_commands`, `test_settings_readonly_values_are_server_values`.
  Assertions: S1/S3만 순서대로 요청; 원본 pixels+header 보존; settings ACK 이전 Run 비활성; 다른 심 선택으로 plan 무효화; weld에서 `/sketch_execute` 미발행; stop은 기존 `/motion_abort`; paint/spray의 기존 선택 흐름 유지.
- [x] `/home/minjea/robotics/node-v24.21.0-linux-x64/bin/node --test web/test/process_settings.test.js web/test/weld_workflow.test.js` 실행, 실패 확인.
- [x] 기존 오른쪽 패널에 접이식 Settings 추가. 공정별 필드·단위·현재값·적용 상태 표시. 용접 모드의 단계는 대상/용접심/경로·실행으로 안내하며 기존 캔버스와 시작·중단을 재사용한다. ID/길이/자세 준비 상태·순서/방향은 목록에 표시한다. DOM 텍스트는 textContent로 처리한다.
- [x] 기존 실행 readiness·render·event handler에 명시적인 weld 위임 추가. 공정 판별이 불명확하면 실행 차단. 새 폼이 기존 화면 폭·키보드 포커스·버튼을 깨지 않게 기존 CSS 규칙을 확장한다. ZED 영상에 심 overlay는 추가하지 않는다.
- [x] `/home/minjea/robotics/node-v24.21.0-linux-x64/bin/node --test web/test/*.test.js` 실행: 기존 reconnect·spray·system 포함 81개 통과.
- [ ] 실제 브라우저의 세 공정 Settings·스크롤·접이식 동작 확인: 사용 가능한 브라우저 도구가 없어 미수행.
- [ ] `feat: add shared process settings and seam selection to the original sketch UI` 단위 커밋.

## Task 8: Isaac 실제 이동 검증과 사용자 실행 안내

**Files:** Create `scripts/check_construct_weld.py`, `test/test_weld_report.py`, `docs/WELDING_ISAAC.md`; Modify `docs/CONSTRUCT_ISAAC_SIM.md`의 실행 안내만. 실행 산출물은 `logs/construct-weld-report.json`, `logs/construct-weld-trace.csv`.

**Interfaces:** checker는 Task 4~7의 공개 topics로 동일 작업을 요청하고 controller 결과·joint_states·TF를 기록한다. CLI는 `--case complete|stop|invalid-frame|unreachable|reset`, `--report`, `--trace`. 실제 장비에서는 실행을 거부한다.

- [x] 보고서 실패 테스트 작성: `test_feedback_required_for_success`, `test_tracking_thresholds_and_retained_selection`, `test_wall_time_not_used_for_weave_frequency`. 위치 0.005 m 초과, 각도 5 deg 초과, 주파수/속도 오차 0.05 초과, 제외한 심 실행은 PASS가 될 수 없음을 고정한다. 누락 feedback도 실패다.
- [x] 해당 pytest 실행, 실패 확인 후 checker/report 구현. 시뮬레이션 flag만 믿지 않고 선택된 모델·controller hardware·ROS domain과 실제 프로세스 구성을 확인한다. timeout과 종료 시 OFF/중단을 보장한다.
- [x] baseline과 비교한 단위·UI 테스트를 실행하고 `bash scripts/build_construct_sim.sh`로 기존 빌드 경로를 검증한다. 의존성 변경이 필요하지 않으면 추가 설치를 하지 않는다.
- [x] 기존 관리 API의 weld 설정으로 Isaac을 준비하고 공개 ROS 토픽으로 ZED 대상 → 시험 3심 중 S1/S3 선택 → Settings 적용 → preview → 실제 이동 완료. UI 입력 이벤트는 원본 캔버스·헤더를 사용하는 harness로 검증.
- [ ] 실제 브라우저에서 위 흐름 육안 확인: 브라우저 도구 미제공. ROS 실행 증거와 구분한다.
- [x] 각 checker case를 `source scripts/construct_ros_env.sh` 환경에서 `python3 scripts/check_construct_weld.py --case complete --report logs/construct-weld-report.json --trace logs/construct-weld-trace.csv`와 같은 방식으로 실행한다. stop·invalid-frame·unreachable·reset도 각각 재현해 expected rejection과 실제 상태를 기록한다. 검사 전후 owned process만 정리한다.
- [x] 측정된 위치·자세·폭·속도·주파수 결과와 화면 증거를 검토한다. 실시간 실행이 막히면 원인을 기록하고 이동 검증을 완료로 처리하지 않는다. 최종 문서에는 실행 방법, fixture/실제 입력 연결점, Settings 의미, 미구현 범위와 측정 결과를 넣는다.
- [ ] `test: verify Isaac seam tracking and document welding workflow` 단위 커밋. 완료 전에 `verification-before-completion`으로 실제 결과를 확인하고, 선택한 실행 방식의 코드 리뷰를 수행한다.

## Execution Preparation and Review

실행 전 `using-git-worktrees` 지침으로 현재 checkout과 dirty 상태를 처리한다. git index와 tracked/untracked 작업 파일의 기준 상태를 보존한 뒤 진행한다. 현재 checkout은 linked worktree가 아니며 Construct 작업 다수가 미커밋 상태이므로 기본 HEAD만 복제한 환경을 기존 시스템으로 착각하지 않는다. 사용자 파일을 임의로 stash/clean/revert하지 않는다.

실행 환경의 Python 검사는 `source scripts/construct_ros_env.sh`로 ROS Jazzy와 기존 overlay를 활성화한다. 순수 기하·상태 모듈은 ROS 없이도 테스트 가능하게 유지한다. node는 `/home/minjea/robotics/node-v24.21.0-linux-x64/bin/node`를 사용한다. 제품 동작을 변경하기 전 기존 관련 테스트 결과를 baseline으로 남긴다.

계획 자체 검토: 설계서의 UI 보존·공통 설정(Task 7), 3D 입력·시계(Task 1/4), 자세·위빙(Task 2), 시뮬레이터 장착물(Task 5), 상태·중단(Task 3/6), 실제 이동 증거(Task 8)를 각각 연결했다. Review Focus 다섯 항목을 해당 task의 테스트로 배정했다. 통신 topic·dataclass 이름과 후속 task의 소비 인터페이스를 일치시켰다.

추천 실행 방식은 Native: 한 에이전트가 이 세션에서 순서대로 구현하고 끝에 독립 리뷰를 받는다. 이유는 UI·Supervisor·executor가 기존 dirty Construct 코드와 같은 상태를 공유하고 각 단계의 계약 의존성이 크기 때문이다. 사용자가 계획을 검토하고 실행 방식을 선택한 뒤 구현을 시작한다.
