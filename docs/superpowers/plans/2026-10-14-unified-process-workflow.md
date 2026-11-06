# Unified Three-Stage Process Workflow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 도장·뿜칠·용접을 대상 인식 → 작업 지정 → 계획·실행의 공통 화면으로 제공하고 실제 검증 범위와 실행 준비를 구별한다.

**Architecture:** 기존 공정별 상태·ROS transport 위에 공통 단계 표시 모듈을 추가한다. 각 공정은 기존 식별자와 실행 게이트를 유지하면서 표시용 상태를 제공한다. 공통 화면은 로봇 제어 판단을 새로 만들지 않는다.

**Tech Stack:** 기존 HTML/CSS, 브라우저 JavaScript, ROSLIB, Node.js `node:test`와 `vm` UI harness. 신규 프레임워크·패키지 설치 없음.

**Spec:** [확정 설계](../specs/2026-10-14-unified-process-workflow-design.md), 사용자 승인 2026-10-14.

## Global Constraints

- 공통 단계: **대상 인식**, **작업 지정**, **계획·실행**.
- 기존 내부 단계 키 `target`, `work_area`, `path`와 ROS 토픽 계약은 유지한다.
- 공통 주 버튼은 `계획 생성·검증`이다.
- 중단 버튼은 단계와 관계없이 접근 가능하게 유지한다.
- 도장·뿜칠의 공정 설정은 서버 값 확인용으로 유지한다.
- 용접 설정은 기존 적용 ACK와 설정 revision 확인 후 계획을 요청한다.
- 대상 스케치를 토치 경로로 사용하지 않는다.
- 이번 변경에 심의 2D 영상 투영이나 새 3D 뷰어는 포함하지 않는다.
- 단계 탭을 보았다는 이유로 완료 처리하지 않으며, 이전 단계로 돌아가 내용을 확인하는 것만으로 계획을 폐기하지 않는다.
- 새 결과 수신 시 강제로 화면을 전환하지 않고 다음 단계 버튼을 활성화한다.
- 실제 입력 변경은 기존 계획을 차단하며 과거 ACK/READY가 차단을 해제할 수 없다.
- 전체 실행기 통합·새 ROS 프로토콜·전체 궤적 사전 계획 추가·하드웨어 출력 어댑터·RL은 범위 밖이다.
- `/home/minjea/robotics/robot-system`은 수정하지 않는다. 실행 전 worktree 지침을 적용하고 미커밋 승인 문서를 보존한다.
- Git 작성자 설정이 없으면 identity를 추측하거나 전역 설정을 변경하지 않는다. 작업별 커밋을 생략하고 변경 파일·검증 결과를 남긴다.

## Review Focus

1. 원격 연결 직후 공정 모드보다 늦게 도착하는 이전 공정 상태: 현재 공정의 단계/실행을 열지 않는다. Task 4.
2. 계획 요청 중 설정 ACK와 상태 revision의 도착 순서 역전: 적용·계획 대기를 별개로 처리한다. Task 3.
3. 목록은 있으나 대상 identity가 바뀐 용접 상태: 후보 존재만으로 2단계 완료나 실행을 허용하지 않는다. Task 3.
4. 탭 왕복 후 이전 wall-front 픽셀·미완성 획이 남는 경우: 조회는 계획을 유지하되 새 경로 요청에는 현재 표면/완성된 입력만 쓴다. Task 2.
5. 중단 ACK 지연 중 단계 이동·재접속: 중단 버튼은 유지하고 새 계획/실행은 차단한다. Task 4.

## File Structure and Shared Interfaces

| 파일 | 책임 |
|---|---|
| `web/js/workflow.js` 신규 | 표시용 상태 계산, 공통 단계·패널·검증 배지 표시. ROS 발행 없음 |
| `web/js/app.js` | 도장/뿜칠 어댑터, 사용자 단계 이동, 경로 방식 선택과 기존 요청 연결 |
| `web/js/weld_workflow.js` | 용접 상태 어댑터, 단계별 심 목록·설정·계획 연결, 기존 transport 보존 |
| `web/js/process_settings.js` | 용접 설정의 draft/pending/applied 읽기 상태와 기존 적용 처리 |
| `web/js/plane_workflow.js` | 실제 로딩되는 평면 선택 UI와 공통 편집 잠금 연결 |
| `web/index.html`, `web/sketch.css` | 공통 단계 레이아웃, 독립 심 목록, 단일 계획 버튼, 검증 범위 안내 |
| `web/test/workflow.test.js` 신규 | 공통 표시 상태의 순수 테스트 |
| `web/test/unified_workflow.test.js` 신규 | 실제 UI 이벤트와 ROS 메시지 경계를 통한 공정 간 통합 테스트 |
| `web/test/ui_harness.js` 및 관련 기존 테스트 | 새 DOM 요소 지원과 사용자 명시적 단계 이동 반영 |

`web/js/multi_surface.js`는 현재 `index.html`에서 로딩하지 않는다. 관련 기능은
실제 로딩되는 `plane_workflow.js`를 기준으로 연결하고 사용하지 않는 사본은 변경하지 않는다.

공통 입력 `WorkflowInput`:

```text
mode: 'paint' | 'spray' | 'weld'
stage: 'target' | 'work_area' | 'path'
connected, modePending, planningOnly, running, stopping, pending: boolean
targetReady, jobReady, settingsApplied, canPlan, canRun: boolean
generated, validated, dirty: boolean
pathMethod: 'auto' | 'manual'              # paint/spray에서 사용
error: string
blockers: string[]                        # 현재 공정 백엔드/로컬 차단 사유
```

`pending`은 해당 공정의 인식·측정·선택·설정·계획 요청 대기에서 계산한다.
`canPlan`, `canRun`, `generated`, `validated`, `targetReady`, `jobReady`는 어댑터가
기존 identity와 게이트를 확인한 결과다. 원시 READY를 그대로 대입하지 않는다.

공개 인터페이스:

- `Workflow.derive(input: WorkflowInput) -> WorkflowView`: 순수 계산.
- `Workflow.render(view: WorkflowView) -> void`: 공통 DOM 표시만 변경.
- `getPaintingWorkflowInput() -> WorkflowInput`: `app.js`의 도장/뿜칠 어댑터.
- `WeldWorkflow.workflowInput() -> WorkflowInput`: 용접 어댑터.
- `ProcessSettings.stateView() -> {applied: boolean, dirty: boolean, pending: boolean}`:
  현재 공정 설정의 읽기 전용 요약. 기존 `isApplied()`는 유지.
- `navigateWorkflow(stage: string) -> boolean`: 사용자 의도에 따른 단계 이동.
  불가능하면 false, 성공하면 `switchWorkflow()`를 호출하고 true. ROS 발행 없음.
- `setPathMethod(method: 'auto' | 'manual') -> void`: 방식 변경 때 로컬 계획 차단.
- `requestWorkflowPlan() -> boolean`: 현재 공정의 게이트를 재확인한 뒤 기존 요청을 발행.

`WorkflowView`는 `stage`, `steps`(각 key/label/complete/accessible/reason),
`show`(target/area/seams/path/settings/execution/drawing), `canEdit`, `canPlan`,
`canRun`, `canStop`, `checks`(generated/validated/ready 각각 state/label/detail)를 가진다.
check.state는 `waiting | pending | complete | blocked | unavailable` 중 하나다.
`derive`는 누락값을 false/대기로 처리하고 backend true를 만들어내지 않는다.

`workflow.js`는 `app.js`보다 먼저 로딩하되 로딩 시 DOM이나 다른 스크립트를 읽지 않는다.
도장/뿜칠 `refreshPaintingUI()`와 용접 `render()`는 공정 표시를 갱신한 뒤 마지막에
공통 render를 호출한다. 공통 render가 이 함수들을 다시 호출하지 않도록 한다.

## Task 1: 공통 단계와 검증 표시 상태

**Files:** Create `web/js/workflow.js`, `web/test/workflow.test.js`.

**Interfaces:** Produces `Workflow.derive`, `Workflow.render`와 위의 두 상태 계약.
브라우저 `window.Workflow`와 Node 테스트용 CommonJS export를 제공한다.

- [x] **1. 순수 상태 테스트 작성.** `stage_names_are_shared`, `stage_visit_does_not_complete_work`,
  `generated_is_not_validated`, `preview_never_enables_run`, `pending_and_stop_lock_mutations`,
  `missing_values_fail_closed`를 작성한다. 기본 입력에서 다음을 검증한다.

  ```javascript
  assert.deepEqual(view.steps.map(s => s.label), ['대상 인식', '작업 지정', '계획·실행']);
  assert.equal(view.canRun, false);
  assert.equal(view.steps[1].complete, false);
  ```

- [x] **2. 실패 확인.** `node --test web/test/workflow.test.js`; 신규 모듈 미구현으로 실패해야 한다.
- [x] **3. 모듈 구현.** 단계 접근은 1단계 항상 조회 가능, 2단계 targetReady,
  3단계 jobReady가 필요하다. 실행/중단/요청 대기 중에는 이동과 편집을 잠근다.
  조회 접근과 입력 가능 조건을 구분한다. 계획 검사 상세는 paint/spray에
  `이동 구간 검사: 실행 중 수행`, weld에 `접근·진입·용접·이탈 사전 계획`,
  planningOnly에 `로봇 검증 미수행`을 표시한다. 정상 아닌 모드는 모든 실행을 차단한다.
- [x] **4. 통과 확인.** 같은 명령이 exit 0이고 전 테스트가 통과해야 한다.
- [x] **5. 변경 검토·커밋.** 이 task 파일만 검토하고 identity가 있을 때
  `feat: add common process workflow presentation state`로 커밋한다.

## Task 2: 도장·뿜칠 화면과 단일 계획 버튼 연결

**Files:** Modify `web/index.html`, `web/sketch.css`, `web/js/app.js`,
`web/js/plane_workflow.js`, `web/test/ui_harness.js`, `web/test/spray_workflow.test.js`,
`web/test/preview_ui.test.js`, `web/test/plane_clear.test.js`.
Create `web/test/unified_workflow.test.js`.

**Interfaces:** Consumes Task 1. Produces `getPaintingWorkflowInput`, `navigateWorkflow`,
`setPathMethod`, `requestWorkflowPlan`. 용접 분기는 Task 3에서 연결한다.

- [x] **1. UI 테스트 작성.** 실제 이벤트로 `target_reply_waits_for_next_click`,
  `area_reply_waits_for_next_click`, `stage_round_trip_preserves_plan`,
  `plan_button_dispatches_selected_path_once`, `unfinished_stroke_blocks_manual_plan`을 검증한다.
  서버 결과만 수신했을 때 현재 단계가 유지되고, 다음 버튼을 누를 때만 이동하며,
  탭 왕복 시 `ui.published.length`가 늘지 않아야 한다.
  기존 spray 테스트의 결과 수신 즉시 자동 전환 가정은 명시적 다음 버튼 클릭으로 변경한다.
- [x] **2. 실패 확인.** `node --test web/test/unified_workflow.test.js web/test/spray_workflow.test.js`.
  새 단계 유지·단일 버튼 동작에 대해 실패해야 한다.
- [x] **3. 레이아웃·이벤트 연결.** `btn-workflow-next`, `path-method` select(auto/manual),
  `btn-plan-workflow`, `weld-seam-actions`, `weld-seam-list`, `execution-section`,
  `workflow-stage-reason`, `workflow-generation-state`, `workflow-validation-scope`,
  `workflow-ready-state` ID를 추가한다. 기존 상태 ID와 transport 함수는 유지한다.
  기존 생성 버튼은 공통 버튼이 연결된 공정부터 숨긴다. 완료되지 않은 용접 연결은
  기존 render가 동작할 수 있게 보존한다. CSS `[hidden]`이 레이아웃 규칙에 의해
  무시되지 않도록 확인한다. 설정·실행 패널은 3단계, 중단은 전역에 둔다.
- [x] **4. 어댑터·잠금 연결.** 기존 paint/spray `paintingDerivedState()`와 blocker 함수를
  사용하고 targetReady는 대상/측정 완료, jobReady는 현재 확정 영역으로 계산한다.
  `updateZedSurfaceUI` 등 성공 응답의 `switchToWorkAreaMode`/`switchToPathMode` 호출을
  다음 단계 활성화로 바꾼다. 공정 변경에 필요한 명시적 초기화는 유지한다.
  pointer/undo/clear/평면 선택 핸들러에도 공통 편집 잠금을 적용한다.
  기존 fill·manual 로직은 함수로 추출하여 공통 버튼에서 한 번만 호출한다.
  기존 버튼 핸들러도 같은 함수에 위임하고 중복 listener를 추가하지 않는다.
- [x] **5. 통과 확인.** `node --test web/test/workflow.test.js web/test/unified_workflow.test.js web/test/spray_workflow.test.js web/test/preview_ui.test.js web/test/plane_clear.test.js`.
  프레임·영역 식별자, 수동 픽셀과 자동 생성 토픽이 이전 계약과 같아야 한다.
- [x] **6. 변경 검토·커밋.** 해당 파일만 검토하고 가능하면
  `feat: unify paint and spray workflow navigation`로 커밋한다.

## Task 3: 용접을 세 단계로 분리

**Files:** Modify `web/js/weld_workflow.js`, `web/js/process_settings.js`, `web/js/app.js`,
`web/test/weld_workflow.test.js`, `web/test/weld_transport.test.js`,
`web/test/process_settings.test.js`, `web/test/unified_workflow.test.js`.

**Interfaces:** Consumes Task 1/2. Produces `WeldWorkflow.workflowInput()`와
`ProcessSettings.stateView()`. 기존 command/select/requestPlan/execute API는 유지한다.

- [x] **1. UI·transport 테스트 작성.** `weld_uses_three_distinct_panels`,
  `weld_stage_two_disables_canvas_mutations`, `selection_ack_gates_next_step`,
  `xyz_seam_can_enter_settings_but_cannot_plan_without_pose`,
  `settings_ack_and_revision_both_required`, `stale_catalog_cannot_unlock_stages`를 작성한다.
  심 선택 명령은 `[{seam_id:'S3',reverse:true},{seam_id:'S1',reverse:false}]` 순서를
  보존하고, 3단계 버튼은 `/welding/plan_request`만 발행해야 한다.
- [x] **2. 실패 확인.** `node --test web/test/weld_workflow.test.js web/test/process_settings.test.js web/test/unified_workflow.test.js`.
  현재 1단계 고정·컨테이너 재사용 때문에 신규 assertion이 실패해야 한다.
- [x] **3. 단계별 표시와 상태 연결.** 1단계는 대상/fixture·external 요청, 2단계는
  독립 `weld-seam-list`, 3단계는 설정·심 요약·계획/실행으로 나눈다.
  계획 결과가 중간 경로를 별도로 제공하지 않으면 READY 전까지 생성·검사 모두
  대기로 표시한다. 검증 배지에도 `invalid`, pending, session/identity를 적용한다.
  작업 중단 상태는 기존 `running`에 포함된 STOPPING 의미를 보존한다.
- [x] **4. 설정과 편집 잠금 연결.** `stateView()`는 현재 내부 상태의 복사본만 반환한다.
  설정 적용 ACK와 snapshot 적용 여부를 둘 다 확인한다. 심 선택·설정은 계획 요청
  대기 중 잠그고, 2/3단계 pointer/undo/clear는 핸들러에서도 거부한다.
  target request는 1단계에서만 가능하게 한다. 공통 요청 버튼의 용접 분기를 연결한다.
  시험 심을 실제 추출 결과로 표시하지 않으며 ZED 영상은 참조로 유지한다.
- [x] **5. 통과 확인.** `node --test web/test/weld_workflow.test.js web/test/weld_transport.test.js web/test/process_settings.test.js web/test/unified_workflow.test.js`.
  번들 ROSLIB의 제어 메시지 비재전송 테스트도 통과해야 한다.
- [x] **6. 변경 검토·커밋.** 가능하면 `feat: split welding into shared three-stage workflow`로 커밋한다.

## Task 4: 공정 전환·늦은 응답·실행 준비 통합 검증

**Files:** Modify `web/test/unified_workflow.test.js`, `web/test/control_reconnect.test.js`,
`web/test/target_refine_gate.test.js`, 필요 시 앞 task의 UI 모듈.
Update `docs/WELDING_ISAAC.md`, `docs/SPRAY_MOTION_TEST.md`, `docs/RB20_PAINTING_ISAAC.md`.

**Interfaces:** Task 1–3 전체. Python/ROS 계약은 변경하지 않는다.

- [x] **1. 사용자 흐름 회귀 테스트 작성.** `old_mode_status_cannot_restore_ready`,
  `edited_method_or_geometry_blocks_old_ready`, `stop_ack_delay_blocks_new_work`,
  `reconnect_never_replays_plan_or_execute`, `paint_tare_approval_is_not_completed_tare`,
  `preview_and_motion_test_keep_output_limits`, `mode_change_clears_old_panel_state`를
  실제 UI 이벤트·토픽 callback으로 검증한다. 새 계획·실행 발행 수는 중단 대기와
  재접속 동안 0이어야 한다. 대상 변경 후 이전 generation/hash/plan_id 결과는 거부한다.
- [x] **2. 실패 여부와 원인 확인.** `node --test web/test/unified_workflow.test.js web/test/control_reconnect.test.js web/test/target_refine_gate.test.js`.
  이미 충족된 요구는 통과해도 된다. 실패한 경우 공정 어댑터/게이트 경계를 추적한다.
- [x] **3. 발견된 통합 문제만 수정.** 테스트를 통과시키기 위해 identity나 readiness
  검사를 제거하지 않는다. 유효한 관측 정보를 UI로 연결하고 부족한 상태는 대기로 남긴다.
- [x] **4. 전체 웹 회귀 실행.** `node --test web/test/*.test.js`; exit 0, 실패 0이 기준이다.
  초기 91개보다 늘어난 개수와 결과를 기록한다. 단지 배치가 바뀐 테스트는 새 사용자
  흐름으로 바꾸되 기존 안전/transport assertion은 유지한다.
- [ ] **5. 브라우저 확인 (환경 미제공으로 미수행).** 로컬 UI에 mock ROS transport를 공급하는 비구동 검사 또는
  준비된 Isaac 화면으로 paint/spray/weld의 1→2→3, 탭 왕복, 수동/자동, 용접 순서·방향,
  검증 범위, 미적용 설정, 중단 대기, 키보드 조작을 확인한다. 1280×720과 1920×1080에서
  주요 버튼·심 목록·상태 문구가 겹치지 않아야 한다. 브라우저 접근이 없으면 자동 테스트와
  육안 확인 미수행을 구분해 보고한다. 실제 로봇에는 연결하지 않는다.
- [x] **6. 문서 갱신.** 세 실행 안내의 단계·버튼·검증 범위를 실제 구현과 맞춘다.
  이 계획에 테스트 명령/결과와 브라우저 확인 범위를 기록한다. UI만 수정했다면 Python
  전체 재실행을 요구하지 않는다. 백엔드 변경이 필요해지면 범위를 다시 검토한다.
- [x] **7. 최종 차이 검토·커밋.** `git diff --check`와 전체 변경 파일을 확인한다.
  가능하면 `test: cover unified workflow transitions and document usage`로 커밋한다.

## Execution Handoff

추천: 이 세션에서 **직접 순차 구현**. 네 작업이 동일한 UI 상태·DOM·transport 경계를
차례로 연결하므로 작업별 구현 에이전트 전환의 이득이 크지 않다. 완료 후 독립 리뷰로
identity·중단·재접속과 검증 범위 표시를 점검한다.

대안: 작업별 구현·리뷰 에이전트 방식. 각 작업마다 별도 구현과 리뷰를 진행하므로
중간 검토가 촘촘하지만 컨텍스트 전달과 검토 횟수가 늘어난다.

구현 계획 검토와 실행 방식 선택 전에는 제품 코드를 변경하지 않는다.

## 구현 검증 기록 (2026-10-14)

- 네 작업 구현 완료. 웹 회귀: `node --test web/test/*.test.js` — 118개 통과, 실패 0 (기준 91개).
- `git diff --check` 통과. Python/ROS 계약 변경 없음.
- 연결된 브라우저가 없어 1280×720·1920×1080 육안/키보드 검사는 미수행.
  실제 UI 이벤트와 ROS callback을 실행하는 harness로 단계·잠금·발행 계약을 검증했다.
- 실제 로봇 연결·구동 없음. 새 Isaac 실행 시험 없음.
- Git 작성자 설정이 없어 계획의 예외에 따라 커밋을 생략했다. 변경 파일과 검증 기록을 보존한다.
- 독립 코드 리뷰의 중요 지적 4건(안전 해제 후 UI 잠금, 용접 대상 사전 거부, 미완성 자유선, 공정 필드 없는 이전 실행 상태)을 재현 테스트 RED→GREEN으로 수정했다. 사소한 보류 항목 없음.
- 서버 재시작 후에는 화면 새로고침을 통한 복구가 필요하다. 새 세션 증거가 없는 재연결만으로 중단 잠금을 해제하지 않는다.
- 원래 IDE 작업 경로에 17개 변경 파일 반영 완료. 덮어쓰기 전 기존 코드가 기준 커밋과 동일하고 검토 중 추가 수정이 없음을 확인했다. 반영 후 웹 테스트 **118/118 통과**, `git diff --check` 통과.
