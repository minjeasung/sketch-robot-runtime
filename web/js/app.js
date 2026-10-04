// sketch_robot UI — rosbridge 연결 + sketch-guided target/work-area/path.
// 현재 구현은 Three.js 가 아니라 native canvas overlay 를 사용한다.

let processMode = "paint";
let sprayMotionTest = false;
let planningOnly = false;
let backendPreview = null;
let processModePending = true;
let multiPlaneBusy = false;
let stopRequested = false;

const WS_URL = `ws://${window.location.hostname || "localhost"}:9090`;
const WORK_AREA_CORNERS_TOPIC = "/perception/work_area_corners";
const ZED_LEFT_IMAGE_TOPIC = "/zed/zed_node/rgb/color/rect/image";
const D405_REFINEMENT_STATUS_TOPIC = "/perception/d405_surface_refinement_status";
const ZED_TARGET_LOCK_TOPIC = "/perception/zed_target_lock";
const ZED_SURFACE_STATUS_TOPIC = "/perception/zed_surface_status";
const zedSelection = new window.ZedSurfaceGate.ZedSurfaceGate();
const PLAN_STATUS_TOPIC = "/painting_system/plan_status";
const READINESS_TOPIC = "/painting_system/readiness";
const EXECUTION_STATUS_TOPIC = "/painting_system/execution_status";
const FREE_SPACE_CONFIRMED_TOPIC = "/painting_admittance/free_space_confirmed";
const ROLLER_LENGTH_M = 0.175;
const DEFAULT_WORK_AREA_W_M = 0.5;
const DEFAULT_WORK_AREA_H_M = 0.4;

const $ = (id) => document.getElementById(id);
$("ws-url").textContent = WS_URL;

function setStatus(state, text) {
  const node = $("status");
  node.classList.remove("connecting", "connected", "disconnected", "error");
  node.classList.add(state);
  $("status-text").textContent = ({connecting: "연결 중", connected: "연결됨", disconnected: "연결 끊김", error: "연결 오류"})[state] || text;
}

function logEvent(line) {
  const node = $("events");
  const ts = new Date().toISOString().slice(11, 23); // HH:MM:SS.sss
  node.textContent = (node.textContent + `[${ts}] ${line}\n`).split("\n").slice(-150).join("\n");
  node.scrollTop = node.scrollHeight;
}

// ---- ROS 연결 ----
const ros = new ROSLIB.Ros({ url: WS_URL });
// Large/slow video frames must not queue ahead of plan results or commands.
const imageRos = new ROSLIB.Ros({ url: WS_URL });
let rosConnected = false;
let zedImageReliable = false;

async function refreshImageTransport() {
  if (typeof fetch !== "function") return;
  try {
    const response = await fetch(new URL("/status", window.location.href));
    if (!response.ok) return;
    const {configuration} = await response.json();
    const reliable = configuration?.camera_backend === "outpost" && configuration?.profile !== "fake";
    if (zedImageReliable !== reliable) {
      zedImageReliable = reliable;
      if (currentView === "zed_raw") subscribeView(currentView);
    }
  } catch (_) { /* Standalone web hosting keeps native-driver defaults. */ }
}

ros.on("connection", () => {
  rosConnected = true;
  requireFreshAuthoritativeState("ROS connected; awaiting fresh backend status");
  setStatus("connected", "connected");
  logEvent("connection opened");
  refreshImageTransport();
  // A browser reconnect must never retain a prior operator safety assertion.
  resetFreeSpaceConfirmation("ROS reconnected", true);
  refreshPaintingUI();
});

ros.on("error", (err) => {
  rosConnected = false;
  requireFreshAuthoritativeState("ROS error; backend status is stale");
  setStatus("error", "error");
  logEvent(`error: ${err && err.message ? err.message : err}`);
  resetFreeSpaceConfirmation("ROS error", false);
  refreshPaintingUI();
});

ros.on("close", () => {
  rosConnected = false;
  requireFreshAuthoritativeState("ROS disconnected; backend status is stale");
  setStatus("disconnected", "disconnected");
  logEvent("connection closed");
  resetFreeSpaceConfirmation("ROS disconnected", false);
  refreshPaintingUI();
});

setStatus("connecting", "connecting…");
logEvent(`connecting to ${WS_URL}`);

// ---- 작업영역 크기: 경로의 롤러 폭 미리보기에 사용 ----
const workAreaCorners = new ROSLIB.Topic({
  ros: ros,
  name: WORK_AREA_CORNERS_TOPIC,
  messageType: "geometry_msgs/PoseArray",
});

let latestWorkAreaSizeM = null;

function dist3(a, b) {
  const dx = a.x - b.x;
  const dy = a.y - b.y;
  const dz = a.z - b.z;
  return Math.hypot(dx, dy, dz);
}

function updateWorkAreaSizeFromCorners(msg) {
  if (processMode === "spray" || processModePending) return;
  if (!msg.poses || msg.poses.length < 4) return;
  const tl = msg.poses[0].position;
  const tr = msg.poses[1].position;
  const br = msg.poses[2].position;
  const bl = msg.poses[3].position;
  const w = 0.5 * (dist3(tr, tl) + dist3(br, bl));
  const h = 0.5 * (dist3(bl, tl) + dist3(br, tr));
  if (!Number.isFinite(w) || !Number.isFinite(h) || w <= 1e-4 || h <= 1e-4) return;
  latestWorkAreaSizeM = { w, h };
  updateSketchStats();
  redrawSketch();
}

workAreaCorners.subscribe(updateWorkAreaSizeFromCorners);
logEvent(`subscribed to ${WORK_AREA_CORNERS_TOPIC}`);

// ---- Authoritative painting-system status ---------------------------------
// The backend owns plane/plan validity. Local flags only fail closed during the
// interval between a UI edit and the corresponding newer backend status.
const paintingState = {
  d405: {},
  plan: {},
  readiness: {},
  execution: {},
  d405Seq: 0,
  zedSeq: 0,
  planSeq: 0,
  readinessSeq: 0,
  lastGeneratedPathId: "",
  targetSelectionState: "unknown",
  local: {
    planeInvalidated: false,
    planInvalidated: false,
    awaitingD405: false,
    selectionIdentityPending: false,
    awaitingPlan: false,
    d405BaselineSeq: 0,
    planBaselineSeq: 0,
    previousWorkAreaId: "",
    previousPlanPathId: "",
    unsentInputEdit: false,
    invalidationReason: "",
  },
};

function requireFreshAuthoritativeState(reason) {
  processModePending = true;
  zedSelection.reset();
  paintingState.zedSeq = 0;
  paintingState.targetSelectionState = "rejected";
  paintingState.d405Seq = 0;
  paintingState.planSeq = 0;
  paintingState.readinessSeq = 0;
  paintingState.local.planeInvalidated = true;
  paintingState.local.planInvalidated = true;
  paintingState.local.awaitingD405 = false;
  paintingState.local.awaitingPlan = false;
  paintingState.local.invalidationReason = reason;
}

const freeSpaceConfirmedPub = new ROSLIB.Topic({
  ros: ros,
  name: FREE_SPACE_CONFIRMED_TOPIC,
  messageType: "std_msgs/Bool",
});
let freeSpaceConfirmed = false;

function setFreeSpaceConfirmed(value, publish = true, reason = "") {
  const next = value === true;
  freeSpaceConfirmed = next;
  const checkbox = $("free-space-confirmed");
  if (checkbox) checkbox.checked = next;
  if (publish && rosConnected) {
    freeSpaceConfirmedPub.publish(new ROSLIB.Message({ data: next }));
    logEvent(`published ${FREE_SPACE_CONFIRMED_TOPIC}=${next}${reason ? ` (${reason})` : ""}`);
  }
  // The operator half of the Run interlock changes synchronously; do not leave
  // a previously enabled Run button open while waiting for the next status tick.
  refreshPaintingUI();
}

function resetFreeSpaceConfirmation(reason, publish = true) {
  const wasConfirmed = freeSpaceConfirmed;
  setFreeSpaceConfirmed(false, publish, reason);
  if (wasConfirmed && !publish && reason) {
    logEvent(`free-space confirmation cleared (${reason})`);
  }
}

$("free-space-confirmed").addEventListener("change", (ev) => {
  if (!ev.target.checked) {
    setFreeSpaceConfirmed(false, true, "operator cleared");
    return;
  }
  const derived = paintingDerivedState();
  if (derived.running || derived.abortReason) {
    setFreeSpaceConfirmed(false, true, "unsafe state rejected confirmation");
    return;
  }
  const ok = window.confirm(
    "계획상 작업면 10 mm 앞에서 정지하며, 실제 측정 간격이 최소 7 mm일 때만 자동 F/T 영점 조정을 승인합니다.\n\n" +
    "선택한 면과 접근 경로가 정확하며, 해당 위치에서 롤러가 벽, 바닥, 지그 및 사람과 완전히 비접촉임을 확인했습니까?",
  );
  if (!ok) {
    setFreeSpaceConfirmed(false, false);
    logEvent("free-space confirmation cancelled by operator");
    return;
  }
  setFreeSpaceConfirmed(true, true, "explicit operator confirmation");
});

function parseJsonStatus(topicName, msg) {
  try {
    const payload = JSON.parse(msg.data || "{}");
    if (!payload || typeof payload !== "object" || Array.isArray(payload)) {
      throw new Error("JSON payload is not an object");
    }
    return payload;
  } catch (err) {
    logEvent(`${topicName}: invalid JSON (${err.message || err})`);
    return null;
  }
}

function normalizedState(value) {
  return String(value || "").trim().toLowerCase();
}

function textId(value) {
  if (value === undefined || value === null) return "";
  return String(value).trim();
}

function firstPresent(...values) {
  for (const value of values) {
    if (value !== undefined && value !== null && value !== "") return value;
  }
  return undefined;
}

function booleanValue(value) {
  if (typeof value === "boolean") return value;
  if (typeof value === "number") return value !== 0;
  if (typeof value === "string") {
    const normalized = normalizedState(value);
    if (["true", "yes", "ok", "ready", "active", "passed", "valid", "selected"].includes(normalized)) return true;
    if (["false", "no", "not_ready", "inactive", "failed", "invalid", "missing"].includes(normalized)) return false;
  }
  if (value && typeof value === "object") {
    return firstBoolean(value.ready, value.ok, value.passed, value.valid, value.active, value.value);
  }
  return undefined;
}

function firstBoolean(...values) {
  for (const value of values) {
    const parsed = booleanValue(value);
    if (parsed !== undefined) return parsed;
  }
  return undefined;
}

function normalizedCheckName(value) {
  return normalizedState(value).replace(/[\s/-]+/g, "_");
}

function readinessCheck(...aliases) {
  const checks = paintingState.readiness.checks;
  const wanted = new Set(aliases.map(normalizedCheckName));
  if (Array.isArray(checks)) {
    for (const entry of checks) {
      if (!entry || typeof entry !== "object") continue;
      const name = normalizedCheckName(firstPresent(entry.name, entry.id, entry.check, entry.key));
      if (wanted.has(name)) return booleanValue(entry);
    }
    return undefined;
  }
  if (checks && typeof checks === "object") {
    for (const [name, value] of Object.entries(checks)) {
      if (wanted.has(normalizedCheckName(name))) return booleanValue(value);
    }
  }
  return undefined;
}

function currentWorkAreaId() {
  if (processMode === "spray") return textId(zedSelection.surface?.work_area_id);
  return textId(firstPresent(
    paintingState.d405.work_area_id,
    paintingState.readiness.work_area_id,
    paintingState.plan.work_area_id,
  ));
}

function invalidatePlanLocally(reason, awaitingPlan = false) {
  const local = paintingState.local;
  const alreadySame = local.planInvalidated && local.invalidationReason === reason;
  local.planInvalidated = true;
  local.awaitingPlan = awaitingPlan;
  local.planBaselineSeq = paintingState.planSeq;
  local.invalidationReason = reason;
  if (!alreadySame) logEvent(`plan invalidated locally: ${reason}`);
  refreshPaintingUI();
}

function beginD405Refresh(reason) {
  const local = paintingState.local;
  local.unsentInputEdit = false;
  local.previousWorkAreaId = currentWorkAreaId();
  local.planeInvalidated = true;
  local.awaitingD405 = true;
  local.selectionIdentityPending = true;
  local.d405BaselineSeq = paintingState.d405Seq;
  invalidatePlanLocally(reason, false);
  refreshPaintingUI();
}

function currentSurfaceStatus() {
  return processMode === "spray" ? (zedSelection.surface || {}) : paintingState.d405;
}

function currentSurfaceSequence() {
  return processMode === "spray" ? paintingState.zedSeq : paintingState.d405Seq;
}

function surfaceAcceptedPayload(payload) {
  return processMode === "spray"
    ? Boolean(zedSelection.surface && payload === zedSelection.surface)
    : d405AcceptedPayload(payload);
}

function invalidateSelection(reason, clearTarget = true) {
  if (clearTarget) zedSelection.reset();
  else zedSelection.clearWorkArea();
  paintingState.zedSeq = 0;
  paintingState.d405 = {};
  paintingState.d405Seq = 0;
  paintingState.plan = {};
  paintingState.planSeq = 0;
  paintingState.readiness = {};
  paintingState.readinessSeq = 0;
  paintingState.lastGeneratedPathId = "";
  paintingState.local.planeInvalidated = true;
  paintingState.local.selectionIdentityPending = true;
  paintingState.local.awaitingD405 = false;
  paintingState.local.unsentInputEdit = true;
  if (clearTarget) paintingState.targetSelectionState = "rejected";
  clearTargetRefineWait();
  waitingWorkAreaRefine = false;
  latestWorkAreaSizeM = null;
  strokesMap.work_area = [];
  strokesMap.path = [];
  resetFreeSpaceConfirmation(reason, true);
  invalidatePlanLocally(reason);
}

function syncZedSelection(action) {
  if (action === "ignore") return;
  const local = paintingState.local;
  paintingState.targetSelectionState = zedSelection.target ? "selected" : "pending";
  if (action === "invalidated") {
    local.planeInvalidated = true;
    local.selectionIdentityPending = true;
    invalidatePlanLocally("ZED selection invalidated");
  } else if (zedSelection.surface) {
    paintingState.zedSeq += 1;
    local.planeInvalidated = false;
    local.selectionIdentityPending = false;
    local.unsentInputEdit = false;
    if (workflowMode === "work_area") switchToPathMode();
  } else if (zedSelection.target && workflowMode === "target") {
    switchToWorkAreaMode();
  }
  reconcileValidatedPlan();
  refreshPaintingUI();
}

new ROSLIB.Topic({ ros, name: ZED_TARGET_LOCK_TOPIC, messageType: "std_msgs/String" }).subscribe(msg => {
  if (processMode !== "spray" || processModePending) return;
  const payload = parseJsonStatus(ZED_TARGET_LOCK_TOPIC, msg);
  if (!payload) { invalidateSelection("invalid ZED target lock JSON"); return; }
  const previous = zedSelection.lock?.plane_generation_id;
  const action = zedSelection.receiveTarget(payload);
  if (action === "locked" && previous !== payload.plane_generation_id) {
    paintingState.local.planeInvalidated = true;
    paintingState.local.selectionIdentityPending = true;
    strokesMap.work_area = [];
    strokesMap.path = [];
    invalidatePlanLocally("ZED plane locked; select a work area");
    if (currentView === "wall_front") subscribeView("wall_front");
  }
  syncZedSelection(action);
});

new ROSLIB.Topic({ ros, name: ZED_SURFACE_STATUS_TOPIC, messageType: "std_msgs/String" }).subscribe(msg => {
  if (processMode !== "spray" || processModePending) return;
  const payload = parseJsonStatus(ZED_SURFACE_STATUS_TOPIC, msg);
  if (!payload) { invalidateSelection("invalid ZED surface status JSON"); return; }
  syncZedSelection(zedSelection.receiveSurface(payload));
});

function beginPlanRequest(reason) {
  paintingState.local.previousPlanPathId = textId(firstPresent(
    paintingState.readiness.path_id,
    paintingState.plan.path_id,
    paintingState.lastGeneratedPathId,
  ));
  paintingState.local.unsentInputEdit = false;
  invalidatePlanLocally(reason, true);
}

function d405AcceptedPayload(payload) {
  return payload && payload.accepted === true && normalizedState(payload.mode) === "work_area";
}

function statusReason(payload) {
  return textId(firstPresent(
    payload.reason,
    payload.rejection_reason,
    payload.message,
    payload.detail,
    payload.abort_reason,
  ));
}

function normalizedAbortReason(value) {
  const reason = textId(value);
  if (["", "none", "ok", "clear", "no_abort", "not_latched"].includes(normalizedState(reason))) {
    return "";
  }
  return reason;
}

function reconcileValidatedPlan() {
  const local = paintingState.local;
  const surface = currentSurfaceStatus();
  const plan = paintingState.plan;
  const readiness = paintingState.readiness;
  if (local.unsentInputEdit || local.awaitingPlan || local.selectionIdentityPending ||
      currentSurfaceSequence() === 0 || paintingState.readinessSeq === 0 || processModePending) {
    return;
  }
  if (!["generated", "validated"].includes(normalizedState(plan.state)) ||
      !surfaceAcceptedPayload(surface)) {
    return;
  }
  const identitiesMatch = window.ZedSurfaceGate.matchesPlan(surface, plan, readiness, processMode);
  const backendValidated = firstBoolean(
    readiness.plan_validated,
    readinessCheck("current_plan_validated", "plan_validated", "valid_plan"),
  ) === true;
  if (identitiesMatch && backendValidated) {
    local.planInvalidated = false;
    local.invalidationReason = "";
  }
}

const d405StatusSub = new ROSLIB.Topic({
  ros: ros,
  name: D405_REFINEMENT_STATUS_TOPIC,
  messageType: "std_msgs/String",
});
const planStatusSub = new ROSLIB.Topic({
  ros: ros,
  name: PLAN_STATUS_TOPIC,
  messageType: "std_msgs/String",
});
const readinessSub = new ROSLIB.Topic({
  ros: ros,
  name: READINESS_TOPIC,
  messageType: "std_msgs/String",
});
const executionStatusSub = new ROSLIB.Topic({
  ros: ros,
  name: EXECUTION_STATUS_TOPIC,
  messageType: "std_msgs/String",
});

d405StatusSub.subscribe((msg) => {
  if (processMode === "spray" || processModePending) return;
  const payload = parseJsonStatus(D405_REFINEMENT_STATUS_TOPIC, msg);
  if (!payload) {
    paintingState.d405 = { state: "rejected", accepted: false, mode: "work_area" };
    paintingState.d405Seq += 1;
    paintingState.local.planeInvalidated = true;
    resetFreeSpaceConfirmation("invalid D405 status JSON", true);
    invalidatePlanLocally("invalid D405 status JSON", false);
    return;
  }
  const previous = paintingState.d405;
  const previousWorkArea = textId(previous.work_area_id);
  const previousGeneration = textId(previous.plane_generation_id);
  const previousAccepted = d405AcceptedPayload(previous);
  paintingState.d405 = payload;
  paintingState.d405Seq += 1;

  const local = paintingState.local;
  const accepted = d405AcceptedPayload(payload);
  const state = normalizedState(payload.state);
  const newWorkArea = textId(payload.work_area_id);
  const newGeneration = textId(payload.plane_generation_id);

  if (accepted) {
    if (local.selectionIdentityPending) {
      const newer = paintingState.d405Seq > local.d405BaselineSeq;
      const changedArea = !local.previousWorkAreaId ||
        (newWorkArea && newWorkArea !== local.previousWorkAreaId);
      if (newer && changedArea) {
        local.planeInvalidated = false;
        local.awaitingD405 = false;
        local.selectionIdentityPending = false;
      }
    } else {
      // Covers initial page load and authoritative changes made by another client.
      local.planeInvalidated = false;
    }
  } else {
    local.planeInvalidated = true;
    if (["rejected", "failed", "timeout", "invalidated"].includes(state)) {
      local.awaitingD405 = false;
    }
    invalidatePlanLocally(`D405 ${state || "not accepted"}`, false);
  }

  const areaChanged = previousWorkArea !== newWorkArea && Boolean(previousWorkArea || newWorkArea);
  const generationChanged = previousGeneration !== newGeneration &&
    Boolean(previousGeneration || newGeneration);
  const acceptanceChanged = previousAccepted !== accepted;
  if (areaChanged || generationChanged || acceptanceChanged) {
    resetFreeSpaceConfirmation("D405 work area / generation / acceptance changed", true);
    if (textId(paintingState.plan.plane_generation_id) !== newGeneration) {
      invalidatePlanLocally("D405 plane generation changed", false);
    }
  }

  logEvent(`D405 work area: ${state || (accepted ? "accepted" : "unknown")}${statusReason(payload) ? ` (${statusReason(payload)})` : ""}`);
  reconcileValidatedPlan();
  refreshPaintingUI();
});

planStatusSub.subscribe((msg) => {
  const payload = parseJsonStatus(PLAN_STATUS_TOPIC, msg);
  if (!payload) {
    paintingState.plan = { state: "rejected", reason: "INVALID_PLAN_STATUS_JSON" };
    paintingState.planSeq += 1;
    invalidatePlanLocally("invalid plan status JSON", false);
    return;
  }
  paintingState.plan = payload;
  paintingState.planSeq += 1;
  const state = normalizedState(payload.state);
  const local = paintingState.local;

  if (["generated", "validated"].includes(state)) {
    const candidatePathId = textId(payload.path_id);
    const surface = currentSurfaceStatus();
    const candidateMatchesSurface = surfaceAcceptedPayload(surface) &&
      textId(payload.work_area_id) === textId(surface.work_area_id) &&
      textId(payload.plane_generation_id) === textId(surface.plane_generation_id) &&
      (processMode !== "spray" || payload.process_mode === "spray");
    const candidateIsNew = Boolean(candidatePathId) &&
      (!local.previousPlanPathId || candidatePathId !== local.previousPlanPathId);
    if (candidatePathId) paintingState.lastGeneratedPathId = candidatePathId;
    const requestedCandidateArrived = local.awaitingPlan &&
      paintingState.planSeq > local.planBaselineSeq && candidateIsNew;
    const authoritativeCandidateAllowed = !local.unsentInputEdit &&
      !processModePending && candidateMatchesSurface && (!local.awaitingPlan || requestedCandidateArrived);
    if (!local.planInvalidated || authoritativeCandidateAllowed) {
      local.planInvalidated = false;
      local.awaitingPlan = false;
      local.invalidationReason = "";
    }
  } else if (["rejected", "invalidated", "failed"].includes(state)) {
    local.planInvalidated = true;
    local.awaitingPlan = false;
    local.invalidationReason = statusReason(payload) || `backend plan ${state}`;
  } else if (["plane_accepted", "plane_rejected"].includes(state)) {
    local.planInvalidated = true;
    local.awaitingPlan = false;
    local.invalidationReason = "plane changed; regenerate path";
  }

  logEvent(`plan status: ${state || "unknown"}${statusReason(payload) ? ` (${statusReason(payload)})` : ""}`);
  reconcileValidatedPlan();
  refreshPaintingUI();
  redrawSketch();
});

let previousAbortReason = "";
let previousExecutionState = "";
let freeSpaceResetForActiveRun = false;

function handleRunAndAbortTransition() {
  const derived = paintingDerivedState();
  const executionState = normalizedState(paintingState.execution.state);
  // The executor latches the confirmed Run prerequisite before publishing
  // SAFETY_APPROACH. Clearing the live operator permission on that authoritative
  // edge cannot race the /sketch_execute gate.
  const acceptedRunState = [
    "safety_approach", "approach", "approach_precontact",
    "contact_search", "contact_search_complete", "contact", "ramp_up",
    "paint", "ramp_down", "travel", "retract", "final_retract",
    "return_home", "complete",
  ].includes(executionState);
  if (derived.running && acceptedRunState && !freeSpaceResetForActiveRun) {
    resetFreeSpaceConfirmation("Run accepted by executor", true);
    freeSpaceResetForActiveRun = true;
  }
  if (executionState === "complete" && previousExecutionState !== "complete") {
    resetFreeSpaceConfirmation("execution complete", true);
  }
  if (!derived.running) {
    freeSpaceResetForActiveRun = false;
  }
  if (derived.abortReason && derived.abortReason !== previousAbortReason) {
    resetFreeSpaceConfirmation("abort latched", true);
  }
  previousAbortReason = derived.abortReason;
  previousExecutionState = executionState;
}

readinessSub.subscribe((msg) => {
  const payload = parseJsonStatus(READINESS_TOPIC, msg);
  if (!payload) {
    paintingState.readiness = {
      ready: false,
      running: false,
      state: "STATUS_INVALID",
      plan_blockers: ["INVALID_READINESS_JSON"],
    };
    paintingState.readinessSeq = 0;
    paintingState.local.planInvalidated = true;
    paintingState.local.invalidationReason = "invalid readiness JSON";
    refreshPaintingUI();
    return;
  }
  paintingState.readiness = payload;
  paintingState.readinessSeq += 1;
  reconcileValidatedPlan();
  handleRunAndAbortTransition();
  refreshPaintingUI();
  redrawSketch();
});

executionStatusSub.subscribe((msg) => {
  const payload = parseJsonStatus(EXECUTION_STATUS_TOPIC, msg);
  if (!payload) {
    paintingState.execution = {
      state: "ERROR",
      reason: "INVALID_EXECUTION_STATUS_JSON",
    };
    handleRunAndAbortTransition();
    refreshPaintingUI();
    return;
  }
  paintingState.execution = payload;
  handleRunAndAbortTransition();
  refreshPaintingUI();
});

for (const topicName of [D405_REFINEMENT_STATUS_TOPIC, PLAN_STATUS_TOPIC, READINESS_TOPIC, EXECUTION_STATUS_TOPIC]) {
  logEvent(`subscribed to ${topicName}`);
}

function paintingDerivedState() {
  const surface = currentSurfaceStatus();
  const plan = paintingState.plan;
  const readiness = paintingState.readiness;
  const execution = paintingState.execution;
  const local = paintingState.local;

  const workAreaId = processMode === "spray" || currentSurfaceSequence() > 0
    ? textId(surface.work_area_id)
    : textId(readiness.work_area_id);
  const planeGenerationId = processMode === "spray" || currentSurfaceSequence() > 0
    ? textId(surface.plane_generation_id)
    : textId(readiness.plane_generation_id);
  const pathId = textId(firstPresent(plan.path_id, readiness.path_id));
  const planHash = textId(firstPresent(plan.plan_hash, readiness.plan_hash));

  const targetSelected = processMode === "spray" ? Boolean(zedSelection.target && !processModePending) : firstBoolean(
    readiness.target_selected,
    readinessCheck("target_selected", "active_target_selected"),
    paintingState.targetSelectionState === "selected"
      ? true
      : paintingState.targetSelectionState === "rejected" ? false : undefined,
    textId(readiness.active_target_id) ? true : undefined,
    workAreaId ? true : undefined,
  );
  const workAreaSelected = processMode === "spray" ? Boolean(zedSelection.surface && !local.selectionIdentityPending) : firstBoolean(
    readiness.work_area_selected,
    readinessCheck("work_area_selected", "current_work_area_selected"),
    workAreaId ? true : undefined,
  );

  const surfaceState = normalizedState(surface.state) || (surface.accepted === true ? "accepted" : "waiting");
  const surfaceIdentityComplete = Boolean(workAreaId && planeGenerationId);
  const surfaceMatchesReadiness = workAreaId === textId(readiness.work_area_id) &&
    planeGenerationId === textId(readiness.plane_generation_id);
  const surfaceAccepted = currentSurfaceSequence() > 0 && !local.selectionIdentityPending &&
    surfaceAcceptedPayload(surface) && surfaceIdentityComplete &&
    surfaceMatchesReadiness && !local.planeInvalidated &&
    (processMode !== "spray" || readiness.process_mode === "spray");

  const planState = normalizedState(plan.state) || "none";
  const generated = ["generated", "validated"].includes(planState);
  const planIdentitiesMatch = window.ZedSurfaceGate.matchesPlan(surface, plan, readiness, processMode);
  const validationCheck = processMode === "spray" ? firstBoolean(
    readiness.plan_validated,
    readinessCheck("current_plan_validated", "plan_validated", "valid_plan"),
  ) : firstBoolean(
    plan.validated,
    readiness.plan_validated,
    readinessCheck("current_plan_validated", "plan_validated", "valid_plan"),
    readiness.ready === true ? true : undefined,
  );
  const planValidated = paintingState.readinessSeq > 0 && generated &&
    planIdentitiesMatch && validationCheck === true &&
    !local.planInvalidated;

  const executionState = normalizedState(execution.state);
  const runningStates = new Set([
    "running", "executing", "safety_approach", "pre_sketch_ready_pose",
    "approach", "approach_precontact", "precontact_tare",
    "precontact_tare_complete", "contact_search",
    "contact_search_complete", "contact", "ramp_up", "paint",
    "ramp_down", "travel", "retract", "final_retract", "return_home",
  ]);
  const running = multiPlaneBusy || readiness.running === true || execution.running === true || runningStates.has(executionState);
  const executionFailed = ["abort", "aborted", "fault", "error", "emergency_stop"].includes(executionState);
  const abortReason = textId(firstPresent(
    normalizedAbortReason(readiness.abort_reason),
    normalizedAbortReason(execution.abort_reason),
    executionFailed
      ? normalizedAbortReason(firstPresent(execution.reason, execution.message))
      : undefined,
  ));
  // Prefer the generator's confirmed goal; readiness mirrors the validated
  // canonical segment force and provides the authoritative fallback.
  const targetForce = firstPresent(plan.target_force_n, readiness.target_force_n, execution.target_force_n);
  const targetForceNumber = Number(targetForce);
  const targetForceValid = processMode === "spray" || (Number.isFinite(targetForceNumber) && targetForceNumber > 0.0);
  // Require both halves of the explicit safety handshake. The local half closes
  // the short window before a just-published false reaches backend readiness;
  // the backend half proves that the executor's interlocked gate consumed true.
  const backendFreeSpaceConfirmed = readinessCheck("free_space_confirmed") === true;
  const planGenerated = !local.planInvalidated && window.ZedSurfaceGate.previewPlanGenerated(surface, plan, readiness);
  const ready = !planningOnly && window.ZedSurfaceGate.executionAllowed(readiness.ready, readiness) && surfaceAccepted && planValidated &&
    targetForceValid && !processModePending && (processMode === "spray" || (freeSpaceConfirmed && backendFreeSpaceConfirmed)) &&
    !running && !abortReason;

  return {
    targetSelected,
    workAreaSelected,
    surfaceState,
    surfaceAccepted,
    workAreaId,
    planeGenerationId,
    pathId,
    planHash,
    planState,
    planValidated,
    planGenerated,
    ready,
    running,
    abortReason,
    targetForce,
    backendFreeSpaceConfirmed,
  };
}

function setPill(id, text, stateClass) {
  const node = $(id);
  if (!node) return;
  node.textContent = text;
  node.className = `state-pill ${stateClass}`;
  node.title = text;
}

function falseReadinessChecks() {
  const checks = paintingState.readiness.checks;
  if (!checks || typeof checks !== "object") return [];
  if (Array.isArray(checks)) {
    return checks
      .filter((entry) => entry && typeof entry === "object" && booleanValue(entry) === false)
      .map((entry) => textId(firstPresent(entry.name, entry.id, entry.check, entry.key)))
      .filter(Boolean);
  }
  return Object.entries(checks)
    .filter(([, value]) => booleanValue(value) === false)
    .map(([name]) => name);
}

function backendBlockers() {
  const value = firstPresent(
    paintingState.readiness.plan_blockers,
    paintingState.readiness.blockers,
  );
  let blockers = [];
  if (Array.isArray(value)) {
    blockers = value.map((entry) => {
      if (entry && typeof entry === "object") {
        return textId(firstPresent(entry.reason, entry.name, entry.id, entry.message));
      }
      return textId(entry);
    }).filter(Boolean);
  } else if (value && typeof value === "object") {
    blockers = Object.entries(value)
      .filter(([, active]) => booleanValue(active) !== false)
      .map(([name]) => name);
  } else if (value) {
    blockers = [textId(value)];
  }
  if (processMode !== "spray" && !d405AcceptedPayload(paintingState.d405) && statusReason(paintingState.d405)) {
    blockers.push(`D405: ${statusReason(paintingState.d405)}`);
  }
  if (["rejected", "invalidated", "failed"].includes(normalizedState(paintingState.plan.state)) &&
      statusReason(paintingState.plan)) {
    blockers.push(`plan: ${statusReason(paintingState.plan)}`);
  }
  blockers.push(...falseReadinessChecks());
  return [...new Set(blockers)];
}

function buttonBlockReason(derived, kind) {
  const reasons = [];
  if (!rosConnected) reasons.push("ROS disconnected");
  if (processModePending) reasons.push("process mode acknowledgement pending");
  if (derived.running) reasons.push("execution already running");
  if (kind === "path" || kind === "run") {
    if (paintingState.readinessSeq === 0) reasons.push("backend readiness status unavailable");
    if (derived.targetSelected !== true) reasons.push("target not selected");
    if (derived.workAreaSelected !== true) reasons.push("work area not selected");
    if (!derived.surfaceAccepted) reasons.push(`current ${processMode === "spray" ? "ZED" : "D405"} work area not accepted`);
  }
  if (kind === "run") {
    if (planningOnly) reasons.push("ZED preview only; robot execution unavailable");
    if (stopRequested) reasons.push("operator stop requested");
    if (!derived.planValidated) reasons.push("current plan not validated");
    if (processMode !== "spray" && (!Number.isFinite(Number(derived.targetForce)) || Number(derived.targetForce) <= 0.0)) {
      reasons.push("target force missing/invalid");
    }
    if (processMode !== "spray" && !freeSpaceConfirmed) reasons.push("free space not confirmed by operator");
    if (processMode !== "spray" && !derived.backendFreeSpaceConfirmed) reasons.push("backend free-space confirmation pending");
    if (paintingState.readiness.ready !== true) reasons.push("backend readiness=false");
    if (derived.abortReason) reasons.push(`abort: ${derived.abortReason}`);
  }
  return reasons;
}

function refreshPaintingUI() {
  const derived = paintingDerivedState();
  const local = paintingState.local;
  const spray = processMode === "spray";
  const measuring = local.awaitingD405 || multiPlaneBusy;
  const surfaceFailed = ["failed", "rejected"].includes(derived.surfaceState);
  $("painting-surface-label").textContent = spray ? "ZED 작업영역" : "D405 평면 측정";
  setPill("painting-d405-state", derived.surfaceAccepted ? "완료" : measuring ? (spray ? "확정 중" : "측정 중") : surfaceFailed ? "확인 필요" : "대기",
    derived.surfaceAccepted ? "good" : measuring ? "pending" : surfaceFailed ? "bad" : "unknown");
  const planFailed = ["rejected", "failed"].includes(derived.planState);
  $("painting-plan-label").textContent = planningOnly ? "경로 생성" : "경로 검증";
  const planComplete = planningOnly ? derived.planGenerated : derived.planValidated;
  setPill("painting-plan-validation", planComplete ? "완료" : local.awaitingPlan ? (planningOnly ? "생성 중" : "검증 중") : planFailed ? "확인 필요" : "대기",
    planComplete ? "good" : local.awaitingPlan ? "pending" : planFailed ? "bad" : "unknown");
  const backendConfirmed = readinessCheck("free_space_confirmed") === true;
  setPill("free-space-state", freeSpaceConfirmed ? (backendConfirmed ? "승인됨" : "확인 중") : "미승인",
    freeSpaceConfirmed ? (backendConfirmed ? "good" : "pending") : "bad");

  const blockers = backendBlockers();
  if (local.planeInvalidated) blockers.unshift(spray ? "ZED work area acceptance pending" : local.awaitingD405 ? "D405 refinement pending" : "D405 plane invalidated");
  if (local.planInvalidated) blockers.unshift(local.invalidationReason || "plan invalidated by local edit");
  $("painting-blockers").textContent = [...new Set(blockers)].join("\n") || "진단 항목 없음";
  $("painting-abort-reason").textContent = derived.abortReason ? `작업 중단: ${derived.abortReason}` : "";
  $("painting-abort-reason").hidden = !derived.abortReason;

  const cs = typeof currentStrokes === "function" ? currentStrokes() : [];
  const pathContext = workflowMode === "path" && currentView === "wall_front";
  const pathGate = rosConnected && !processModePending && pathContext && !derived.running &&
    paintingState.readinessSeq > 0 &&
    derived.targetSelected === true && derived.workAreaSelected === true && derived.surfaceAccepted;
  $("btn-set-target").disabled = !rosConnected || zedFrameCount === 0 || processModePending || derived.running ||
    workflowMode !== "target" || cs.length === 0 || currentView !== "zed_raw";
  $("btn-set-work-area").disabled = !rosConnected || zedFrameCount === 0 || processModePending || derived.running ||
    workflowMode !== "work_area" || currentView !== "wall_front" || derived.targetSelected !== true ||
    (spray && (zedFrameCount === 0 || !sprayRectangleReady(cs)));
  $("btn-execute").disabled = spray || !pathGate || cs.length === 0;
  $("btn-execute").hidden = spray;
  $("btn-fill-work-area").textContent = spray ? "자동 도포 경로 생성" : "영역 자동 채우기";
  $("btn-fill-work-area").disabled = !pathGate;
  $("btn-run-robot").disabled = planningOnly || !rosConnected || !derived.ready || stopRequested;
  $("btn-clear").disabled = derived.running || cs.length === 0;
  $("btn-undo").disabled = derived.running || cs.length === 0;
  $("free-space-confirmed").disabled = !rosConnected || derived.running || Boolean(derived.abortReason);
  $("btn-stop-robot").disabled = planningOnly || !rosConnected;
  $("btn-stop-robot").hidden = planningOnly;
  document.querySelectorAll('input[name="workflow-mode"], input[name="sketch-mode"]').forEach(input => {
    input.disabled = derived.running || processModePending ||
      (spray && input.name === "sketch-mode" && (workflowMode === "path" || (workflowMode === "work_area" && input.value !== "rect")));
  });

  $("target-actions").hidden = workflowMode !== "target";
  $("work-area-actions").hidden = workflowMode !== "work_area";
  $("path-actions").hidden = workflowMode !== "path";
  $("tare-confirmation").hidden = processMode === "spray" || workflowMode !== "path";
  const hints = {
    target: "작업할 대상을 둘러 그린 뒤 평면을 추출하세요.",
    work_area: spray ? "ZED Wall Front · 사각형 작업영역" : "측정한 평면 위에 칠할 영역을 그리세요.",
    path: planningOnly ? "자동 도포 경로 미리보기 · 로봇 도달성·충돌 검증 전" : spray ? "자동 도포 경로 · 충돌 및 실행 검증" : "영역을 자동으로 채우거나, 원하는 경로를 직접 그리세요.",
  };
  $("work-area-description").textContent = spray ? "ZED 평면 · 사각형 영역" : "칠할 영역을 둘러 그리세요. 그리지 않으면 정면 영상 전체를 사용합니다.";
  $("path-actions-title").textContent = spray ? "자동 도포 경로" : "작업 경로";
  $("view-card-title").textContent = currentView === "wall_front" && spray ? "ZED Wall Front" : VIEW_TITLES[currentView];
  $("workflow-hint").textContent = derived.running ? "로봇 작업 중에는 스케치를 수정할 수 없습니다." : hints[workflowMode];
  $("sketch-canvas").setAttribute("aria-label", hints[workflowMode]);

  let summary = "실행 조건을 확인 중입니다. 연결·진단에서 상세 내용을 확인하세요.";
  if (!rosConnected) summary = "영상·작업 서버 연결을 기다리고 있습니다.";
  else if (derived.abortReason) summary = "작업이 중단되었습니다. 원인을 확인하세요.";
  else if (stopRequested) summary = "중단 요청을 보냈습니다. 로봇의 응답을 기다립니다.";
  else if (multiPlaneBusy) summary = spray ? "ZED 평면 선택을 확인 중입니다." : "선택한 평면에 접근하여 측정 중입니다.";
  else if (derived.running) summary = "로봇이 작업 중입니다.";
  else if (processModePending) summary = "작업 방식 변경을 확인 중입니다.";
  else if (derived.ready) summary = "준비 완료. 작업을 시작할 수 있습니다.";
  else if (paintingState.readinessSeq === 0) summary = planningOnly ? "ZED 작업 서버 상태를 확인하고 있습니다." : "로봇 상태를 확인하고 있습니다.";
  else if (measuring) summary = "평면 측정 결과를 기다리고 있습니다.";
  else if (derived.targetSelected !== true) summary = spray ? "ZED 대상 평면을 선택·확정하세요." : "대상을 선택하고 평면을 측정하세요.";
  else if (derived.workAreaSelected !== true) summary = "칠할 작업영역을 확정하세요.";
  else if (!derived.surfaceAccepted) summary = spray ? "ZED 작업영역 확인을 기다립니다." : "평면 측정 상태를 확인하세요.";
  else if (local.awaitingPlan) summary = planningOnly ? "작업 경로를 생성하고 있습니다." : "작업 경로를 생성·검증하고 있습니다.";
  else if (planningOnly && derived.planGenerated) summary = "경로 생성 완료. 로봇 도달성·충돌 검증은 연결 후 진행하세요.";
  else if (!derived.planValidated) summary = planFailed ? "경로 검증에 실패했습니다. 영역이나 경로를 확인하세요." : "작업 경로를 생성하세요.";
  else if (processMode !== "spray" && !freeSpaceConfirmed) summary = "접촉 전 F/T 영점 조정을 승인하세요.";
  else if (processMode !== "spray" && !backendConfirmed) summary = "영점 조정 승인을 확인 중입니다.";
  $("execution-summary").textContent = summary;
  $("execution-summary").classList.toggle("is-running", derived.running);
  $("btn-run-robot").textContent = planningOnly ? "ZED 미리보기 · 로봇 실행 안 함" : derived.running ? "작업 진행 중" : "작업 시작";

  const pathReasons = buttonBlockReason(derived, "path");
  $("btn-execute").title = pathReasons.length ? pathReasons.join("; ") : "그린 경로를 생성하고 검증합니다";
  $("btn-fill-work-area").title = pathReasons.length ? pathReasons.join("; ") : "선택한 작업영역을 채우는 경로를 생성합니다";
  const runReasons = buttonBlockReason(derived, "run");
  $("btn-run-robot").title = runReasons.length ? runReasons.join("; ") : "확인 후 로봇이 움직입니다";
  return derived;
}

// ---- View mode (ZED Raw / Wall Front) + image subscriber ----
// 두 view 의 sketch strokes 는 의미가 다름 (원본 카메라 픽셀 vs 벽 평면 픽셀) → 분리 보관.
const VIEW_TOPICS = {
  zed_raw:    "/zed/zed_node/rgb/color/rect/image",
  wall_front: "/perception/wall_front_view",
};
const VIEW_TITLES = {
  zed_raw:    "ZED 카메라",
  wall_front: "작업면 정면",
};

let currentView = "zed_raw";
let currentImageSub = null;

const zedCanvas = $("zed-canvas");
const zedCtx = zedCanvas.getContext("2d");
let zedFrameCount = 0;

function decodeImageData(msg) {
  // sensor_msgs/Image, encoding=rgb8 → roslibjs 가 base64 string 으로 data 전달.
  const w = msg.width, h = msg.height;
  const bin = atob(msg.data);
  // step (한 row 의 바이트 수) — rgb8 면 w*3. 다른 encoding 대비 일반화.
  const channels = (msg.encoding === "rgba8" || msg.encoding === "bgra8") ? 4 : 3;
  const buf = new Uint8ClampedArray(w * h * 4);
  let j = 0;
  if (msg.encoding === "rgb8") {
    for (let i = 0; i < bin.length; i += 3) {
      buf[j++] = bin.charCodeAt(i);
      buf[j++] = bin.charCodeAt(i + 1);
      buf[j++] = bin.charCodeAt(i + 2);
      buf[j++] = 255;
    }
  } else if (msg.encoding === "bgr8") {
    for (let i = 0; i < bin.length; i += 3) {
      buf[j++] = bin.charCodeAt(i + 2);
      buf[j++] = bin.charCodeAt(i + 1);
      buf[j++] = bin.charCodeAt(i);
      buf[j++] = 255;
    }
  } else if (msg.encoding === "bgra8") {
    for (let i = 0; i < bin.length; i += 4) {
      buf[j++] = bin.charCodeAt(i + 2);
      buf[j++] = bin.charCodeAt(i + 1);
      buf[j++] = bin.charCodeAt(i);
      buf[j++] = 255;
    }
  } else if (msg.encoding === "rgba8") {
    for (let i = 0; i < bin.length; i += 4) {
      buf[j++] = bin.charCodeAt(i);
      buf[j++] = bin.charCodeAt(i + 1);
      buf[j++] = bin.charCodeAt(i + 2);
      buf[j++] = bin.charCodeAt(i + 3);
    }
  } else {
    // unknown encoding — gray fallback
    for (let i = 0; i < bin.length; i += channels) {
      const v = bin.charCodeAt(i);
      buf[j++] = v; buf[j++] = v; buf[j++] = v; buf[j++] = 255;
    }
  }
  return new ImageData(buf, w, h);
}

function resizeImageCanvas(width, height) {
  if (width !== zedCanvas.width || height !== zedCanvas.height) {
    zedCanvas.width = width;
    zedCanvas.height = height;
    if (sketchCanvas.width !== width || sketchCanvas.height !== height) {
      sketchCanvas.width = width;
      sketchCanvas.height = height;
      redrawSketch();
    }
  }
}

function handleImageMsg(msg) {
  try {
    const imgData = decodeImageData(msg);
    resizeImageCanvas(msg.width, msg.height);
    zedCtx.putImageData(imgData, 0, 0);
  } catch (e) {
    logEvent(`image decode 실패: ${e.message || e}`);
    return;
  }

  imageDisplayed(msg.width, msg.height, msg.encoding);
}

function handlePreviewImage(image) {
  resizeImageCanvas(image.width, image.height);
  zedCtx.drawImage(image, 0, 0);
  imageDisplayed(image.width, image.height, "JPEG preview");
}

function imageDisplayed(width, height, encoding) {
  zedFrameCount += 1;
  $("camera-empty").hidden = true;

  if (zedFrameCount === 1) {
    logEvent(`first image on ${VIEW_TOPICS[currentView]} (${width}×${height}, ${encoding})`);
  }
  refreshPaintingUI();
}

function subscribeView(viewName) {
  if (currentImageSub) {
    try { currentImageSub.unsubscribe(); } catch (_) {}
    currentImageSub = null;
  }
  const topic = VIEW_TOPICS[viewName];
  const sub = createPreviewStream({
    ros: imageRos,
    name: topic,
    reliable: viewName === "zed_raw" && zedImageReliable,
    onRaw: msg => {
      if (currentImageSub === sub && currentView === viewName) handleImageMsg(msg);
    },
    onPreview: image => {
      if (currentImageSub === sub && currentView === viewName) handlePreviewImage(image);
    },
    onError: error => logEvent(`image decode 실패: ${error.message || error}`),
  });
  currentImageSub = sub;
  // 새 view 의 첫 frame 도착 전 — stats reset 으로 fps 계산 정확하게.
  zedFrameCount = 0;
  zedCtx.clearRect(0, 0, zedCanvas.width, zedCanvas.height);
  $("camera-empty").hidden = false;
  logEvent(`subscribed to ${topic}`);
}

function switchView(viewName) {
  if (viewName === currentView || !VIEW_TOPICS[viewName]) return;
  // 진행 중 stroke 정리 (모드 무관, 다른 view 로 가면 의미 없음)
  pendingRect = null;
  currentStroke = null;
  pendingLine = null;
  currentMouse = null;
  currentView = viewName;
  $("view-card-title").textContent = VIEW_TITLES[viewName];
  // sketch 즉시 redraw (새 view 의 strokes 로)
  redrawSketch();
  // 새 topic subscribe
  subscribeView(viewName);
}

// 초기 구독
subscribeView(currentView);
let imageReconnectTimer = null;
imageRos.on("close", () => {
  if (currentImageSub) currentImageSub.unsubscribe();
  currentImageSub = null;
  zedFrameCount = 0;
  zedCtx.clearRect(0, 0, zedCanvas.width, zedCanvas.height);
  $("camera-empty").hidden = false;
  currentStroke = null; pendingRect = null; pendingLine = null; currentMouse = null;
  refreshPaintingUI();
  if (imageReconnectTimer === null) imageReconnectTimer = setTimeout(() => {
    imageReconnectTimer = null;
    imageRos.connect(WS_URL);
  }, 1000);
});
imageRos.on("connection", () => {
  clearTimeout(imageReconnectTimer); imageReconnectTimer = null;
  if (!currentImageSub) subscribeView(currentView);
});
imageRos.on("error", () => logEvent("영상 연결 오류 — 재연결 대기"));


// ============================================================================
// Sketch overlay (Freehand + Line 모드) — canvas native 좌표 (px) 기준 보관.
// 3D 변환은 B3.3 에서. 여기는 시각 + 데이터 저장만.
// ============================================================================
const sketchCanvas = $("sketch-canvas");
const sketchCtx = sketchCanvas.getContext("2d");

// workflow 별 stroke 분리. Target/Work Area/Path 의 의미가 다르므로 합치지 않는다.
const strokesMap = { target: [], work_area: [], path: [] };
let workflowMode = "target";
function currentStrokes() { return strokesMap[workflowMode]; }
function sprayRectangleReady(strokes) {
  return window.ZedSurfaceGate.rectangleReady(strokes, sketchCanvas.width, sketchCanvas.height);
}

let currentStroke = null;     // 진행 중 freehand stroke (mousedown ~ mouseup)
let pendingLine = null;       // Line 모드: 첫 점 찍힌 후 두 번째 클릭 대기 중인 stroke
let pendingRect = null;       // Rect 모드: drag 중인 직사각형 {start, end}
let currentMouse = null;      // 가장 최근 pointer 위치 (Line preview 용)
let sketchMode = "freehand";
const STROKE_COLOR = "#22d3ee";  // cyan
const STROKE_WIDTH = 3;

function getNativeCoords(ev) {
  // CSS 로 줄어든 canvas 의 client 좌표 → native (canvas.width/height) 좌표
  const rect = sketchCanvas.getBoundingClientRect();
  const scaleX = sketchCanvas.width / rect.width;
  const scaleY = sketchCanvas.height / rect.height;
  return {
    u: (ev.clientX - rect.left) * scaleX,
    v: (ev.clientY - rect.top) * scaleY,
  };
}

function updateSketchStats() {
  $("sketch-strokes-count").textContent = `${currentStrokes().length}개 스케치`;
  refreshPaintingUI();
}

function rollerFootprintScale() {
  if (processMode === "spray") return null;
  if (currentView !== "wall_front" || workflowMode !== "path") return null;
  if (sketchCanvas.width <= 0 || sketchCanvas.height <= 0) return null;
  const physicalW = latestWorkAreaSizeM ? latestWorkAreaSizeM.w : DEFAULT_WORK_AREA_W_M;
  const physicalH = latestWorkAreaSizeM ? latestWorkAreaSizeM.h : DEFAULT_WORK_AREA_H_M;
  if (physicalW <= 1e-4 || physicalH <= 1e-4) return null;
  return {
    widthPx: ROLLER_LENGTH_M * sketchCanvas.width / physicalW,
    estimated: !latestWorkAreaSizeM,
  };
}

function sampledStrokePoints(points, spacingPx = 14) {
  if (!points || points.length === 0) return [];
  const out = [points[0]];
  let last = points[0];
  for (let i = 1; i < points.length; i++) {
    const p = points[i];
    if (Math.hypot(p.u - last.u, p.v - last.v) >= spacingPx) {
      out.push(p);
      last = p;
    }
  }
  const tail = points[points.length - 1];
  if (out[out.length - 1] !== tail) out.push(tail);
  return out;
}

function screenTangentAt(points, idx, fallback = { u: 1.0, v: 0.0 }) {
  if (!points || points.length < 2) return fallback;
  let a;
  let b;
  if (idx <= 0) {
    a = points[0];
    b = points[1];
  } else if (idx >= points.length - 1) {
    a = points[points.length - 2];
    b = points[points.length - 1];
  } else {
    a = points[idx - 1];
    b = points[idx + 1];
  }
  const du = b.u - a.u;
  const dv = b.v - a.v;
  const norm = Math.hypot(du, dv);
  if (norm < 1e-6) return fallback;
  return { u: du / norm, v: dv / norm };
}

function rollerAxisFromScreenTangent(tangent) {
  const axis = { u: -tangent.v, v: tangent.u };
  const norm = Math.hypot(axis.u, axis.v);
  if (norm < 1e-6) return { u: 1.0, v: 0.0 };
  return { u: axis.u / norm, v: axis.v / norm };
}

function drawRollerBar(pt, widthPx, alpha, withTicks = false, axis = null) {
  const half = widthPx * 0.5;
  const dir = axis || { u: 1.0, v: 0.0 };
  const ax = dir.u;
  const ay = dir.v;
  const tx = -ay;
  const ty = ax;
  sketchCtx.save();
  sketchCtx.lineCap = "round";
  sketchCtx.lineWidth = 5;
  sketchCtx.strokeStyle = `rgba(107, 214, 163, ${alpha})`;
  sketchCtx.beginPath();
  sketchCtx.moveTo(pt.u - ax * half, pt.v - ay * half);
  sketchCtx.lineTo(pt.u + ax * half, pt.v + ay * half);
  sketchCtx.stroke();
  if (withTicks) {
    sketchCtx.lineWidth = 2;
    sketchCtx.strokeStyle = "rgba(255, 255, 255, 0.72)";
    const tick = 9;
    sketchCtx.beginPath();
    sketchCtx.moveTo(pt.u - ax * half - tx * tick, pt.v - ay * half - ty * tick);
    sketchCtx.lineTo(pt.u - ax * half + tx * tick, pt.v - ay * half + ty * tick);
    sketchCtx.moveTo(pt.u + ax * half - tx * tick, pt.v + ay * half - ty * tick);
    sketchCtx.lineTo(pt.u + ax * half + tx * tick, pt.v + ay * half + ty * tick);
    sketchCtx.stroke();
  }
  sketchCtx.restore();
}

function drawRollerFootprints() {
  const scale = rollerFootprintScale();
  if (!scale) return;
  for (const s of currentStrokes()) {
    const sampled = sampledStrokePoints(s.points);
    for (let i = 0; i < sampled.length; i++) {
      const tangent = screenTangentAt(sampled, i);
      const axis = rollerAxisFromScreenTangent(tangent);
      drawRollerBar(sampled[i], scale.widthPx, 0.24, false, axis);
    }
  }
  if (pendingLine && currentMouse) {
    const a = pendingLine.points[0];
    const b = currentMouse;
    const steps = Math.max(2, Math.ceil(Math.hypot(b.u - a.u, b.v - a.v) / 28));
    const tangent = screenTangentAt([a, b], 0);
    const axis = rollerAxisFromScreenTangent(tangent);
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      drawRollerBar({
        u: a.u + (b.u - a.u) * t,
        v: a.v + (b.v - a.v) * t,
      }, scale.widthPx, 0.16, false, axis);
    }
  }
  if (currentMouse) {
    let tangent = { u: 1.0, v: 0.0 };
    if (currentStroke && currentStroke.points.length >= 2) {
      const pts = currentStroke.points;
      tangent = screenTangentAt([pts[pts.length - 2], currentMouse], 0);
    } else if (pendingLine) {
      tangent = screenTangentAt([pendingLine.points[0], currentMouse], 0);
    }
    drawRollerBar(
      currentMouse,
      scale.widthPx,
      0.72,
      true,
      rollerAxisFromScreenTangent(tangent),
    );
  }
}

function redrawSketch() {
  sketchCtx.clearRect(0, 0, sketchCanvas.width, sketchCanvas.height);
  if (typeof drawPlaneCandidates === "function") drawPlaneCandidates();
  drawRollerFootprints();
  const derived = paintingDerivedState();
  if (currentView === "wall_front" && workflowMode === "path" &&
      (derived.planGenerated || derived.planValidated)) {
    const lines = window.ZedSurfaceGate.previewStrokes(backendPreview, derived.pathId);
    sketchCtx.save();
    sketchCtx.strokeStyle = "#22d3ee";
    sketchCtx.lineWidth = 3;
    for (const points of lines) {
      sketchCtx.beginPath();
      sketchCtx.moveTo(points[0].u, points[0].v);
      for (const point of points.slice(1)) sketchCtx.lineTo(point.u, point.v);
      sketchCtx.stroke();
    }
    sketchCtx.restore();
  }
  sketchCtx.lineCap = "round";
  sketchCtx.lineJoin = "round";
  sketchCtx.lineWidth = STROKE_WIDTH;
  sketchCtx.strokeStyle = STROKE_COLOR;

  for (const s of currentStrokes()) {
    if (s.points.length === 0) continue;
    sketchCtx.beginPath();
    sketchCtx.moveTo(s.points[0].u, s.points[0].v);
    if (s.type === "freehand") {
      for (let i = 1; i < s.points.length; i++) {
        sketchCtx.lineTo(s.points[i].u, s.points[i].v);
      }
    } else if (s.type === "line" && s.points.length >= 2) {
      sketchCtx.lineTo(s.points[1].u, s.points[1].v);
    } else if (s.type === "rect" && s.points.length >= 4) {
      for (let i = 1; i < 4; i++) {
        sketchCtx.lineTo(s.points[i].u, s.points[i].v);
      }
      sketchCtx.closePath();
    }
    sketchCtx.stroke();
  }

  // Rect 모드 preview (drag 중 점선 직사각형)
  if (pendingRect && pendingRect.end) {
    const a = pendingRect.start, b = pendingRect.end;
    sketchCtx.save();
    sketchCtx.setLineDash([8, 6]);
    sketchCtx.strokeStyle = STROKE_COLOR;
    sketchCtx.lineWidth = STROKE_WIDTH;
    sketchCtx.strokeRect(a.u, a.v, b.u - a.u, b.v - a.v);
    sketchCtx.restore();
  }

  // Line 모드 preview (첫 점 찍힌 후, 두 번째 클릭 전)
  if (pendingLine && currentMouse) {
    sketchCtx.save();
    sketchCtx.setLineDash([8, 6]);
    sketchCtx.strokeStyle = "rgba(34, 211, 238, 0.65)";
    sketchCtx.beginPath();
    sketchCtx.moveTo(pendingLine.points[0].u, pendingLine.points[0].v);
    sketchCtx.lineTo(currentMouse.u, currentMouse.v);
    sketchCtx.stroke();
    sketchCtx.restore();
    // 첫 점 marker (원)
    sketchCtx.fillStyle = STROKE_COLOR;
    sketchCtx.beginPath();
    sketchCtx.arc(pendingLine.points[0].u, pendingLine.points[0].v,
                  5, 0, Math.PI * 2);
    sketchCtx.fill();
  }

  updateSketchStats();
}

// ---- Pointer event handlers (mouse + touch 통합) ----
sketchCanvas.addEventListener("pointerdown", (ev) => {
  ev.preventDefault();
  if (zedFrameCount === 0) {
    logEvent("영상 수신 후 영역을 선택하세요");
    return;
  }
  if (paintingDerivedState().running || processModePending ||
      (processMode === "spray" && (workflowMode === "path" ||
        (workflowMode === "work_area" && (!zedSelection.target || currentView !== "wall_front" || zedFrameCount === 0))))) {
    logEvent("stroke ignored: execution is running");
    return;
  }
  sketchCanvas.setPointerCapture(ev.pointerId);
  const c = getNativeCoords(ev);
  currentMouse = c;
  paintingState.local.unsentInputEdit = true;
  if (processMode === "spray") {
    if (workflowMode === "target") invalidateSelection("ZED target edited");
    else {
      zedSelection.clearWorkArea();
      paintingState.local.planeInvalidated = true;
      paintingState.local.selectionIdentityPending = true;
      strokesMap.work_area = [];
      sketchMode = "rect";
    }
  }
  invalidatePlanLocally(`${workflowMode} stroke edited`, false);
  if (sketchMode === "freehand") {
    currentStroke = { type: "freehand", points: [c] };
    currentStrokes().push(currentStroke);
  } else if (sketchMode === "rect") {
    // 직사각형: drag 시작점. pointerup 에서 4 corner stroke 로 확정.
    pendingRect = { start: c, end: c };
  } else {
    // line
    if (!pendingLine) {
      pendingLine = { type: "line", points: [c] };
    } else {
      pendingLine.points.push(c);
      currentStrokes().push(pendingLine);
      pendingLine = null;
    }
  }
  redrawSketch();
});

sketchCanvas.addEventListener("pointermove", (ev) => {
  currentMouse = getNativeCoords(ev);
  if (sketchMode === "freehand" && currentStroke) {
    currentStroke.points.push(currentMouse);
    redrawSketch();
  } else if (sketchMode === "rect" && pendingRect) {
    pendingRect.end = currentMouse;
    redrawSketch();
  } else if (sketchMode === "line" && pendingLine) {
    redrawSketch();
  }
});

function finishFreehand(ev) {
  if (pendingRect) {
    const a = pendingRect.start, b = pendingRect.end || pendingRect.start;
    const u0 = Math.min(a.u, b.u), u1 = Math.max(a.u, b.u);
    const v0 = Math.min(a.v, b.v), v1 = Math.max(a.v, b.v);
    if (u1 - u0 > 3 && v1 - v0 > 3) {
      currentStrokes().push({
        type: "rect",
        points: [{ u: u0, v: v0 }, { u: u1, v: v0 },
                 { u: u1, v: v1 }, { u: u0, v: v1 }],
      });
    }
    pendingRect = null;
    redrawSketch();
    try { sketchCanvas.releasePointerCapture(ev.pointerId); } catch (_) {}
    return;
  }
  if (currentStroke) {
    currentStroke = null;
    redrawSketch();
  }
  try { sketchCanvas.releasePointerCapture(ev.pointerId); } catch (_) {}
}

sketchCanvas.addEventListener("pointerup", finishFreehand);
sketchCanvas.addEventListener("pointercancel", finishFreehand);

sketchCanvas.addEventListener("pointerleave", () => {
  currentMouse = null;
  if (pendingLine) redrawSketch();   // preview 사라짐
});

// ESC: 진행 중 line 취소
document.addEventListener("keydown", (ev) => {
  if (ev.key === "Escape" && pendingLine) {
    pendingLine = null;
    redrawSketch();
  }
});

// ---- 모드 라디오 ----
document.querySelectorAll('input[name="sketch-mode"]').forEach((r) => {
  r.addEventListener("change", () => {
    sketchMode = document.querySelector('input[name="sketch-mode"]:checked').value;
    // 모드 전환 시 진행 중 stroke 정리
    if (sketchMode !== "line" && pendingLine) {
      pendingLine = null;
    }
    if (sketchMode !== "freehand" && currentStroke) {
      currentStroke = null;
    }
    if (sketchMode !== "rect" && pendingRect) {
      pendingRect = null;
    }
    redrawSketch();
  });
});

function switchWorkflow(mode) {
  if (!strokesMap[mode]) return;
  currentStroke = null;
  pendingLine = null;
  pendingRect = null;
  currentMouse = null;
  workflowMode = mode;
  if (processMode === "spray" && mode === "work_area") {
    sketchMode = "rect";
    document.querySelector('input[name="sketch-mode"][value="rect"]').checked = true;
  }
  // Work area pixels always refer to the projector's Wall Front view.
  const targetView = mode === "target" ? "zed_raw" : "wall_front";
  switchView(targetView);
  redrawSketch();
}

document.querySelectorAll('input[name="workflow-mode"]').forEach((r) => {
  r.addEventListener("change", () => {
    const mode = document.querySelector('input[name="workflow-mode"]:checked').value;
    switchWorkflow(mode);
  });
});

// ---- Clear / Undo (현재 view 의 strokes 만) ----
$("btn-clear").addEventListener("click", () => {
  if (paintingDerivedState().running) return;
  paintingState.local.unsentInputEdit = true;
  currentStrokes().length = 0;
  pendingLine = null;
  pendingRect = null;
  currentStroke = null;
  if (processMode === "spray") invalidateSelection("ZED selection cleared", workflowMode === "target");
  invalidatePlanLocally(`${workflowMode} cleared`, false);
  redrawSketch();
});

$("btn-undo").addEventListener("click", () => {
  if (paintingDerivedState().running) return;
  const cs = currentStrokes();
  if (cs.length > 0) {
    cs.pop();
    paintingState.local.unsentInputEdit = true;
    if (processMode === "spray") invalidateSelection("ZED selection undone", workflowMode === "target");
    invalidatePlanLocally(`${workflowMode} undo`, false);
    redrawSketch();
  }
});

// ---- Publish workflow strokes as PoseArray ---------------------------------
const TARGET_SELECTION_TOPIC = "/target_selection_pixels";
const TARGET_REFINE_STATUS_TOPIC = "/target_refine_status";
const WORK_AREA_PIXELS_TOPIC = "/work_area_pixels";
const ZED_WORK_AREA_REQUEST_TOPIC = "/painting_system/zed_work_area_request";
const REFINE_WORK_AREA_TOPIC = "/refine_work_area";
const WORK_AREA_REFINE_STATUS_TOPIC = "/work_area_refine_status";
const SKETCH_PIXELS_TOPIC = "/sketch_pixels";
const SKETCH_EXECUTE_TOPIC = "/sketch_execute";
const FILL_WORK_AREA_TOPIC = "/fill_work_area";
const targetSelectionPub = new ROSLIB.Topic({
  ros: ros,
  name: TARGET_SELECTION_TOPIC,
  messageType: "geometry_msgs/PoseArray",
});
const targetRefineStatusSub = new ROSLIB.Topic({
  ros: ros,
  name: TARGET_REFINE_STATUS_TOPIC,
  messageType: "std_msgs/String",
});
const workAreaPub = new ROSLIB.Topic({
  ros: ros,
  name: WORK_AREA_PIXELS_TOPIC,
  messageType: "geometry_msgs/PoseArray",
});
const zedWorkAreaRequestPub = new ROSLIB.Topic({
  ros: ros,
  name: ZED_WORK_AREA_REQUEST_TOPIC,
  messageType: "std_msgs/String",
});
const refineWorkAreaPub = new ROSLIB.Topic({
  ros: ros,
  name: REFINE_WORK_AREA_TOPIC,
  messageType: "std_msgs/Bool",
});
const workAreaRefineStatusSub = new ROSLIB.Topic({
  ros: ros,
  name: WORK_AREA_REFINE_STATUS_TOPIC,
  messageType: "std_msgs/String",
});
const sketchPub = new ROSLIB.Topic({
  ros: ros,
  name: SKETCH_PIXELS_TOPIC,
  messageType: "geometry_msgs/PoseArray",
});
const sketchExecutePub = new ROSLIB.Topic({
  ros: ros,
  name: SKETCH_EXECUTE_TOPIC,
  messageType: "std_msgs/Bool",
});
const fillWorkAreaPub = new ROSLIB.Topic({
  ros: ros,
  name: FILL_WORK_AREA_TOPIC,
  messageType: "std_msgs/Empty",
});
const TARGET_REFINE_UI_TIMEOUT_MS = 8000;
let waitingTargetRefine = null;
let targetRefineTimeoutHandle = null;
let waitingWorkAreaRefine = false;

function clearTargetRefineWait() {
  if (targetRefineTimeoutHandle !== null) {
    window.clearTimeout(targetRefineTimeoutHandle);
    targetRefineTimeoutHandle = null;
  }
  waitingTargetRefine = null;
}

function beginTargetRefineWait(selectionStamp) {
  clearTargetRefineWait();
  waitingTargetRefine = window.TargetRefineGate.begin(
    selectionStamp, Date.now(), TARGET_REFINE_UI_TIMEOUT_MS,
  );
  const expectedStampKey = waitingTargetRefine.stampKey;
  targetRefineTimeoutHandle = window.setTimeout(() => {
    if (
      !waitingTargetRefine ||
      waitingTargetRefine.stampKey !== expectedStampKey
    ) return;
    clearTargetRefineWait();
    paintingState.targetSelectionState = "rejected";
    logEvent("target refinement timed out; real path remains disabled");
    refreshPaintingUI();
  }, TARGET_REFINE_UI_TIMEOUT_MS + 50);
}

function switchToWorkAreaMode() {
  const radio = document.querySelector('input[name="workflow-mode"][value="work_area"]');
  if (radio) radio.checked = true;
  switchWorkflow("work_area");
}

function switchToPathMode() {
  const radio = document.querySelector('input[name="workflow-mode"][value="path"]');
  if (radio) radio.checked = true;
  switchWorkflow("path");
}

targetRefineStatusSub.subscribe((msg) => {
  if (processMode === "spray" || processModePending) return;
  let payload = {};
  try {
    payload = JSON.parse(msg.data || "{}");
  } catch (_) {
    payload = { state: msg.data || "unknown" };
  }
  const state = normalizedState(payload.state) || "unknown";
  const reason = String(payload.rejection_reason || "").trim();
  const ratio = Number(payload.inlier_ratio);
  const broadRatio = Number(payload.broad_inlier_ratio);
  const fitPointCount = Number(payload.voxel_point_count || payload.roi_point_count || 0);
  const ratioText = Number.isFinite(ratio) && fitPointCount > 0
    ? `, D405 local=${(100.0 * ratio).toFixed(1)}%`
    : "";
  const broadRatioText = Number.isFinite(broadRatio) && fitPointCount > 0
    ? `, broad=${(100.0 * broadRatio).toFixed(1)}%`
    : "";
  const reasonText = reason ? `, reason=${reason}` : "";
  logEvent(
    `target refine: ${state}${reasonText}${ratioText}${broadRatioText}`,
  );
  if (!waitingTargetRefine) return;

  const decision = window.TargetRefineGate.assess(
    waitingTargetRefine, payload, Date.now(),
  );
  if (decision.action === "ignore") {
    logEvent(`ignored stale target refine status (${decision.reason})`);
    return;
  }
  waitingTargetRefine = decision.pending;
  if (decision.action === "success") {
    clearTargetRefineWait();
    paintingState.targetSelectionState = "selected";
    switchToWorkAreaMode();
  } else if (decision.action === "failure") {
    clearTargetRefineWait();
    paintingState.targetSelectionState = "rejected";
    logEvent("target refinement failed; real path remains disabled (no ZED fallback)");
  }
  refreshPaintingUI();
});

workAreaRefineStatusSub.subscribe((msg) => {
  if (processMode === "spray" || processModePending) return;
  let payload = {};
  try {
    payload = JSON.parse(msg.data || "{}");
  } catch (_) {
    payload = { state: msg.data || "unknown" };
  }
  const state = normalizedState(payload.state) || "unknown";
  logEvent(`work area refine: ${state}`);
  if (!waitingWorkAreaRefine) return;

  if (state === "done" || state === "accepted") {
    waitingWorkAreaRefine = false;
    switchToPathMode();
    logEvent("work-area workflow completed; waiting for authoritative D405 acceptance");
  } else if (["failed", "rejected", "timeout", "busy", "aborted", "abort_latched"].includes(state)) {
    waitingWorkAreaRefine = false;
    logEvent("D405 refine failed; staying in Work Area view");
  }
  refreshPaintingUI();
});

let lastPixelStampMs = 0;
function nowRosTime() {
  const ms = Math.max(Date.now(), lastPixelStampMs + 1);
  lastPixelStampMs = ms;
  return {
    sec: Math.floor(ms / 1000),
    nanosec: (ms % 1000) * 1_000_000,
  };
}

function posesFromStrokes(strokes) {
  const poses = [];
  strokes.forEach((s, strokeIndex) => {
    for (const pt of s.points) {
      poses.push({
        position: { x: pt.u, y: pt.v, z: strokeIndex },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
      });
    }
  });
  return poses;
}

function publishPixels(pub, topicName, frameId, strokes, stamp = null) {
  const poses = posesFromStrokes(strokes);
  if (poses.length === 0) return false;
  const messageStamp = stamp || nowRosTime();
  const msg = new ROSLIB.Message({
    header: {
      stamp: messageStamp,
      frame_id: frameId,
    },
    poses: poses,
  });
  pub.publish(msg);
  logEvent(`published ${poses.length} points to ${topicName} (frame=${frameId})`);
  return messageStamp;
}

$("btn-set-target").addEventListener("click", () => {
  if (workflowMode !== "target" || currentView !== "zed_raw") return;
  if (zedFrameCount === 0) return;
  if (!rosConnected || processModePending || paintingDerivedState().running || !currentStrokes().length) return;
  if (typeof clearPlaneSelection === "function") clearPlaneSelection(true);
  if (processMode === "spray") invalidateSelection("new ZED target selected");
  paintingState.targetSelectionState = "pending";
  resetFreeSpaceConfirmation("new target selected", true);
  if (processMode !== "spray") beginD405Refresh("new target selected");
  // The D405 refiner arms exactly one capture only after it has received and
  // stored this new target surface.  Publishing a separate Bool here used to
  // race the target surface across two DDS topics and could refine the old
  // target instead.
  const selectionStamp = nowRosTime();
  clearTargetRefineWait();
  if (publishPixels(
    targetSelectionPub,
    TARGET_SELECTION_TOPIC,
    "zed_raw",
    currentStrokes(),
    selectionStamp,
  )) {
    logEvent("평면 후보 추출 중 — 측정할 면을 선택하세요");
  } else {
    clearTargetRefineWait();
    paintingState.targetSelectionState = "rejected";
    logEvent("target selection has no pixels; D405 refinement was not armed");
  }
  refreshPaintingUI();
});

$("btn-set-work-area").addEventListener("click", () => {
  if (workflowMode !== "work_area" || currentView !== "wall_front") return;
  if (zedFrameCount === 0) return;
  if (!rosConnected || processModePending || paintingDerivedState().running) return;
  let strokes = currentStrokes();
  if (processMode === "spray") {
    if (!zedSelection.target || zedFrameCount === 0 || !sprayRectangleReady(strokes)) return;
    const stamp = nowRosTime();
    if (!zedSelection.beginWorkArea(stamp)) return;
    paintingState.local.planeInvalidated = true;
    paintingState.local.selectionIdentityPending = true;
    paintingState.local.unsentInputEdit = false;
    invalidatePlanLocally("ZED work area requested");
    // The shared stamp is a correlation identity; the generation binds the
    // rectangle to its plane without comparing browser and robot clocks.
    zedWorkAreaRequestPub.publish(new ROSLIB.Message({ data: JSON.stringify({
      source: "zed",
      plane_generation_id: zedSelection.lock.plane_generation_id,
      header: { frame_id: "wall_front", stamp },
      pixels: strokes[0].points.map(({ u, v }) => [u, v]),
    }) }));
    publishPixels(workAreaPub, WORK_AREA_PIXELS_TOPIC, "wall_front", strokes, stamp);
    return;
  }
  const drawn = strokes.reduce((acc, s) => acc + s.points.length, 0) > 0;
  if (!drawn) {
    // 박스를 안 그렸으면 wall_front 전체(D405 정면 뷰)를 작업영역으로 사용.
    const w = sketchCanvas.width, h = sketchCanvas.height;
    if (w <= 1 || h <= 1) {
      logEvent("wall_front 이미지 미수신 — Set Work Area 보류");
      return;
    }
    strokes = [{ type: "box", points: [
      { u: 0, v: 0 }, { u: w - 1, v: 0 },
      { u: w - 1, v: h - 1 }, { u: 0, v: h - 1 },
    ] }];
    logEvent("작업영역 박스 미지정 → wall_front 전체를 작업영역으로 사용");
  }
  resetFreeSpaceConfirmation("new work area selected", true);
  beginD405Refresh("new work area selected");
  if (publishPixels(workAreaPub, WORK_AREA_PIXELS_TOPIC, "wall_front", strokes)) {
    waitingWorkAreaRefine = true;
    refineWorkAreaPub.publish(new ROSLIB.Message({ data: true }));
    logEvent(`published refine request to ${REFINE_WORK_AREA_TOPIC}`);
    logEvent("waiting for D405 work-area refinement before Path mode");
  }
});

$("btn-fill-work-area").addEventListener("click", () => {
  const derived = paintingDerivedState();
  const blockers = buttonBlockReason(derived, "path");
  if (workflowMode !== "path" || currentView !== "wall_front" || blockers.length) {
    logEvent(`Fill rejected by UI gate: ${blockers.join("; ") || "wrong workflow/view"}`);
    refreshPaintingUI();
    return;
  }
  beginPlanRequest("backend fill requested");
  fillWorkAreaPub.publish(new ROSLIB.Message({}));
  // The overlay uses only the backend-generated pixels for this path ID.
  logEvent("published /fill_work_area; waiting for backend plan_status before Run is enabled");
});

$("btn-execute").addEventListener("click", () => {
  if (processMode === "spray") return;
  const cs = currentStrokes();
  if (cs.length === 0) return;
  const derived = paintingDerivedState();
  const blockers = buttonBlockReason(derived, "path");
  if (workflowMode !== "path" || currentView !== "wall_front" || blockers.length) {
    logEvent(`Path rejected by UI gate: ${blockers.join("; ") || "wrong workflow/view"}`);
    refreshPaintingUI();
    return;
  }

  beginPlanRequest("free-sketch path requested");
  if (!publishPixels(sketchPub, SKETCH_PIXELS_TOPIC, "wall_front", cs)) return;
  logEvent("path pixels sent; waiting for backend plan validation before Run is enabled");
});

// ---- Run Robot: confirm 후 /sketch_execute Bool(true) publish ----
$("btn-run-robot").addEventListener("click", () => {
  const derived = refreshPaintingUI();
  const blockers = buttonBlockReason(derived, "run");
  if (!derived.ready || blockers.length) {
    logEvent(`Run rejected by authoritative gate: ${blockers.join("; ") || "readiness=false"}`);
    return;
  }
  const hash8 = derived.planHash.slice(0, 8);
  const forceNumber = Number(derived.targetForce);
  const forceText = Number.isFinite(forceNumber) ? `${forceNumber.toFixed(2)} N` : "missing";
  const ok = window.confirm(
    "로봇 실제 실행을 승인합니까?\n\n" +
    `작업 방식: ${processMode === "spray" ? "내화뿜칠" : "롤러 도장"}\n` +
    (processMode === "paint" ? `목표 접촉력: ${forceText}\n\n` : "\n") +
    (processMode === "spray"
      ? (sprayMotionTest
        ? "현재 EOAT로 작업면에서 50 cm 이격하여 실제 이동합니다. 뿜칠건 미장착 이동 검증이며 분사 출력은 항상 OFF입니다."
        : "로봇이 작업면에서 50 cm 이격하여 이동합니다. 도포 경로에서만 뿜칠건이 켜집니다.")
      : "로봇이 즉시 움직입니다. 10 mm pre-contact에서 정지한 후 간격을 확인하고 F/T tare를 수행합니다."),
  );
  if (!ok) {
    logEvent("Run Robot cancelled by user");
    return;
  }
  sketchExecutePub.publish(new ROSLIB.Message({ data: true }));
  logEvent(`published ${SKETCH_EXECUTE_TOPIC} for plan ${hash8}`);
});

// Uses the executor's existing abort interface; this is a stop request, not a
// claim that the robot has stopped. Backend abort status remains authoritative.
const motionAbortPub = new ROSLIB.Topic({ros, name: "/motion_abort", messageType: "std_msgs/Bool"});
$("btn-stop-robot").addEventListener("click", () => {
  if (!rosConnected) return;
  stopRequested = true;
  motionAbortPub.publish(new ROSLIB.Message({data: true}));
  resetFreeSpaceConfirmation("operator requested stop", true);
  logEvent("작업 중단 요청 전송 (/motion_abort)");
  refreshPaintingUI();
});

updateSketchStats();
logEvent("sketch overlay ready (mode=freehand)");

// Backend pixels share the generated path timestamp; stale frames never draw.
new ROSLIB.Topic({ros, name: "/painting_system/fill_preview_pixels",
  messageType: "geometry_msgs/PoseArray"}).subscribe(msg => {
  backendPreview = msg;
  redrawSketch();
});
