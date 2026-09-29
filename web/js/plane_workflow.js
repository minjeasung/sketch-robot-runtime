// Catalog selection is shared; only paint selection requests D405 motion.
let planeCatalog = { generation: "", planes: [] };
let planeState = {};
const selectedPlaneIds = new Set();
const planeColors = ["#38bdf8", "#f59e0b", "#a78bfa", "#4ade80", "#fb7185", "#22d3ee", "#f472b6", "#a3e635"];
const selectPlanesPub = new ROSLIB.Topic({ ros, name: "/painting_system/select_planes", messageType: "std_msgs/String" });
const activatePlanePub = new ROSLIB.Topic({ ros, name: "/painting_system/activate_plane", messageType: "std_msgs/String" });
const processModePub = new ROSLIB.Topic({ ros, name: "/painting_system/set_process_mode", messageType: "std_msgs/String" });
let requestedProcessMode = "";
let lastPlaneRender = "";

function clearPlaneSelection(clearCatalog = false) {
  planeState = {};
  selectedPlaneIds.clear();
  if (clearCatalog) {
    planeCatalog = { generation: "", planes: [] };
    zedSelection.setCatalog("");
  }
  lastPlaneRender = "";
  renderPlaneList();
}

function renderPlaneList() {
  const spray = processMode === "spray";
  const running = paintingDerivedState().running;
  const blocked = !rosConnected || running || processModePending;
  const signature = JSON.stringify([planeCatalog, planeState, [...selectedPlaneIds], blocked, processMode, sprayMotionTest]);
  if (signature === lastPlaneRender) return;
  lastPlaneRender = signature;
  const list = $("plane-candidates");
  list.replaceChildren();
  list.setAttribute("aria-label", spray ? "확정할 ZED 평면 선택" : "측정할 평면 선택");
  planeCatalog.planes.forEach((plane, index) => {
    const label = document.createElement("label");
    const swatch = document.createElement("span");
    swatch.className = "plane-swatch";
    swatch.style.background = planeColors[index % planeColors.length];
    swatch.setAttribute("aria-hidden", "true");
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.checked = selectedPlaneIds.has(plane.id);
    checkbox.disabled = blocked;
    checkbox.addEventListener("change", () => {
      if (blocked) return;
      if (checkbox.checked) selectedPlaneIds.add(plane.id);
      else selectedPlaneIds.delete(plane.id);
      invalidateSelection("catalog plane selection edited");
      renderPlaneList();
      redrawSketch();
    });
    label.append(checkbox, swatch, document.createTextNode(`면 ${index + 1}`));
    list.append(label);
  });
  $("btn-refine-planes").textContent = spray ? "선택한 ZED 면 확정" : "선택한 면 D405 측정";
  $("btn-refine-planes").disabled = blocked || !selectedPlaneIds.size;
  const select = $("active-plane");
  select.replaceChildren();
  const empty = document.createElement("option");
  empty.value = "";
  empty.textContent = spray ? "작업할 ZED 평면 선택" : "작업할 측정 평면 선택";
  select.append(empty);
  const measured = Array.isArray(planeState.measured) ? planeState.measured : [];
  planeCatalog.planes.filter(plane => measured.includes(plane.id)).forEach(plane => {
    const option = document.createElement("option");
    option.value = plane.id;
    option.textContent = `면 ${planeCatalog.planes.indexOf(plane) + 1}`;
    select.append(option);
  });
  select.value = planeState.active_id || "";
  select.disabled = blocked || !measured.length;
  $("active-plane-field").hidden = !measured.length;
  $("process-mode").disabled = blocked || sprayMotionTest;
}

function drawPlaneCandidates() {
  if (currentView !== "zed_raw") return;
  const sx = sketchCanvas.width / (planeCatalog.image_width || sketchCanvas.width);
  const sy = sketchCanvas.height / (planeCatalog.image_height || sketchCanvas.height);
  planeCatalog.planes.forEach((plane, index) => {
    const points = plane.polygon_px || [];
    if (points.length < 3) return;
    sketchCtx.save();
    sketchCtx.strokeStyle = planeColors[index % planeColors.length];
    sketchCtx.lineWidth = selectedPlaneIds.has(plane.id) ? 5 : 2;
    sketchCtx.beginPath();
    points.forEach((point, i) => {
      if (i) sketchCtx.lineTo(point[0] * sx, point[1] * sy);
      else sketchCtx.moveTo(point[0] * sx, point[1] * sy);
    });
    sketchCtx.closePath();
    sketchCtx.stroke();
    sketchCtx.globalAlpha = 0.15;
    sketchCtx.fillStyle = planeColors[index % planeColors.length];
    sketchCtx.fill();
    sketchCtx.globalAlpha = 1;
    sketchCtx.font = "bold 22px sans-serif";
    sketchCtx.fillText(`면 ${index + 1}`, points[0][0] * sx, points[0][1] * sy);
    sketchCtx.restore();
  });
}

new ROSLIB.Topic({ ros, name: "/perception/target_planes", messageType: "std_msgs/String" }).subscribe(msg => {
  const payload = parseJsonStatus("/perception/target_planes", msg);
  if (processModePending || !payload || !Array.isArray(payload.planes) || !textId(payload.generation)) return;
  if (payload.generation === planeCatalog.generation) return;
  invalidateSelection("new plane catalog");
  planeCatalog = payload;
  planeState = {};
  selectedPlaneIds.clear();
  zedSelection.setCatalog(payload.generation);
  $("planes-status").textContent = payload.error || `${payload.planes.length}개 평면 · ${processMode === "spray" ? "확정할 ZED 면 선택" : "측정할 면 선택"}`;
  renderPlaneList();
  redrawSketch();
});

$("btn-refine-planes").addEventListener("click", () => {
  if (!rosConnected || processModePending || paintingDerivedState().running || !selectedPlaneIds.size) return;
  if (processMode === "paint" && !window.confirm("선택한 평면들을 D405로 측정하기 위해 로봇이 순서대로 접근합니다. 시작할까요?")) return;
  invalidateSelection("selected catalog planes requested");
  if (processMode === "spray" && !zedSelection.selectPlanes(planeCatalog.generation, [...selectedPlaneIds])) return;
  multiPlaneBusy = true;
  paintingState.targetSelectionState = "pending";
  selectPlanesPub.publish(new ROSLIB.Message({ data: JSON.stringify({ generation: planeCatalog.generation, ids: [...selectedPlaneIds] }) }));
  renderPlaneList();
  refreshPaintingUI();
});

new ROSLIB.Topic({ ros, name: "/painting_system/planes", messageType: "std_msgs/String" }).subscribe(msg => {
  const payload = parseJsonStatus("/painting_system/planes", msg);
  if (processModePending || !payload || payload.generation !== planeCatalog.generation) return;
  if (processMode === "spray" && payload.source && payload.source !== "zed") return;
  const changed = payload.active_id !== planeState.active_id || planeState.state !== "ready";
  planeState = payload;
  multiPlaneBusy = payload.running === true;
  const states = processMode === "spray"
    ? { ready: "ZED 평면 확정 · 작업영역 선택", failed: "ZED 평면 선택 실패", candidates: "확정할 ZED 면 선택" }
    : { ordering: "이동이 적은 측정 순서 계산 중", measuring: "D405 접근·측정 중", ready: "측정 완료 · 평면별 작업영역 선택", failed: "측정 실패", candidates: "측정할 면 선택" };
  $("planes-status").textContent = payload.error || states[payload.state] || payload.state;
  if (payload.state === "ready") {
    if (processMode === "spray") {
      if (zedSelection.target && zedSelection.lock?.plane_id === payload.active_id) syncZedSelection("target");
    } else {
      paintingState.targetSelectionState = "selected";
      if (changed) {
        strokesMap.work_area = [];
        strokesMap.path = [];
        beginD405Refresh("active plane changed");
        switchToWorkAreaMode();
      }
    }
  }
  renderPlaneList();
  refreshPaintingUI();
  redrawSketch();
});

$("active-plane").addEventListener("change", event => {
  const id = event.target.value;
  if (!id || !rosConnected || processModePending || paintingDerivedState().running) return;
  if (processMode === "paint" && !window.confirm("선택한 면의 작업영역을 그릴 수 있도록 D405가 다시 접근합니다. 시작할까요?")) {
    event.target.value = planeState.active_id || "";
    return;
  }
  invalidateSelection("active plane changed");
  if (processMode === "spray") zedSelection.selectPlanes(planeCatalog.generation, [id]);
  else beginD405Refresh("switch selected plane");
  multiPlaneBusy = true;
  activatePlanePub.publish(new ROSLIB.Message({ data: JSON.stringify({ generation: planeCatalog.generation, id }) }));
  refreshPaintingUI();
  renderPlaneList();
});

$("process-mode").addEventListener("change", event => {
  if (!rosConnected || processModePending || paintingDerivedState().running || sprayMotionTest) return;
  requestedProcessMode = event.target.value;
  if (!["paint", "spray"].includes(requestedProcessMode)) return;
  processModePending = true;
  invalidateSelection("process mode changed");
  strokesMap.target = [];
  clearPlaneSelection(true);
  processModePub.publish(new ROSLIB.Message({ data: requestedProcessMode }));
  refreshPaintingUI();
});

new ROSLIB.Topic({ ros, name: "/painting_system/process_mode", messageType: "std_msgs/String" }).subscribe(msg => {
  const payload = parseJsonStatus("/painting_system/process_mode", msg);
  if (!payload || !["paint", "spray"].includes(payload.mode)) return;
  if (requestedProcessMode && payload.mode !== requestedProcessMode && !payload.error) return;
  const changed = processMode !== payload.mode || sprayMotionTest !== (payload.spray_motion_test === true);
  const wasPending = processModePending;
  processMode = payload.mode;
  sprayMotionTest = payload.spray_motion_test === true;
  zedSelection.setMode(processMode);
  processModePending = false;
  requestedProcessMode = "";
  $("process-mode").value = processMode;
  $("process-mode-state").textContent = payload.error || (sprayMotionTest
    ? "EOAT 이동 검증 · 실제 로봇 이동 · 50 cm 이격 · 분사 항상 OFF"
    : processMode === "spray" ? "ZED 평면 · 50 cm 이격 · 도포 경로에서 분사" : "작업면에 접촉하여 도장합니다.");
  if (changed || wasPending) {
    invalidateSelection("process mode acknowledged; select a new target");
    strokesMap.target = [];
    clearPlaneSelection(true);
    document.querySelector('input[name="workflow-mode"][value="target"]').checked = true;
    switchWorkflow("target");
  }
  refreshPaintingUI();
  renderPlaneList();
});

ros.on("close", () => {
  multiPlaneBusy = false;
  requestedProcessMode = "";
  clearPlaneSelection(true);
});
renderPlaneList();
setInterval(renderPlaneList, 1000);
