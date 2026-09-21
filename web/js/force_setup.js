// sketch_robot Force Setup UI - camera-derived surface normal + F/T target setup.

const WS_URL = `ws://${window.location.hostname || "localhost"}:9090`;

const SCAN_TRIGGER_TOPIC = "/perception/scan_trigger";
const PLANES_TOPIC = "/perception/planes";
const PLANE_LABELS_TOPIC = "/perception/plane_labels";
const WORK_AREA_PLANE_TOPIC = "/perception/work_area_plane";
const REFINED_WORK_AREA_PLANE_TOPIC = "/perception/work_area_plane_refined";
const D405_REFINE_CAPTURE_TOPIC = "/d405/refine_capture";
const D405_REFINE_STATUS_TOPIC = "/perception/d405_surface_refinement_status";
const TARGET_SELECTION_TOPIC = "/target_selection_pixels";
const TARGET_SURFACE_TOPIC = "/perception/target_surface";
const WORK_AREA_PIXELS_TOPIC = "/work_area_pixels";

const FT_STATUS_TOPIC = "/ft/status";
const FT_ZERO_TOPIC = "/ft/zero";
const FT_TARGET_CONFIG_TOPIC = "/ft/target_config";

const VIEW_TOPICS = {
  zed_raw: "/zed/zed_node/rgb/color/rect/image",
  wall_front: "/perception/wall_front_view",
};
const VIEW_TITLES = {
  zed_raw: "ZED LEFT CAMERA",
  wall_front: "WALL FRONT VIEW",
};

const $ = (id) => document.getElementById(id);
$("ws-url").textContent = WS_URL;

function setStatus(state, text) {
  const node = $("status");
  node.classList.remove("connecting", "connected", "disconnected", "error");
  node.classList.add(state);
  $("status-text").textContent = text;
}

function logEvent(line) {
  const node = $("events");
  const ts = new Date().toISOString().slice(11, 23);
  node.textContent += `[${ts}] ${line}\n`;
  node.scrollTop = node.scrollHeight;
}

function setValue(id, text) {
  const node = $(id);
  if (node) node.textContent = text;
}

function setDisabled(id, disabled) {
  const node = $(id);
  if (node) node.disabled = disabled;
}

function bindClick(id, handler) {
  const node = $(id);
  if (node) node.addEventListener("click", handler);
}

function fmt(v, digits = 3) {
  const n = Number(v);
  return Number.isFinite(n) ? n.toFixed(digits) : "-";
}

function fmtN(v, digits = 2) {
  const n = Number(v);
  return Number.isFinite(n) ? n.toFixed(digits) : "-";
}

function nowRosTime() {
  const ms = Date.now();
  return {
    sec: Math.floor(ms / 1000),
    nanosec: (ms % 1000) * 1000000,
  };
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function norm(a) {
  return Math.hypot(a[0], a[1], a[2]);
}

function normalize(a) {
  const n = norm(a);
  if (n < 1e-12) return [0.0, 0.0, 1.0];
  return [a[0] / n, a[1] / n, a[2] / n];
}

function quatRotate(q, v) {
  const qv = [q[0], q[1], q[2]];
  const uv = cross(qv, v);
  const uuv = cross(qv, uv);
  return [
    v[0] + 2.0 * (q[3] * uv[0] + uuv[0]),
    v[1] + 2.0 * (q[3] * uv[1] + uuv[1]),
    v[2] + 2.0 * (q[3] * uv[2] + uuv[2]),
  ];
}

function normalToQuat(normal) {
  const n = normalize(normal);
  const z = [0.0, 0.0, 1.0];
  const d = Math.max(-1.0, Math.min(1.0, dot(z, n)));
  if (d > 0.9999) return [0.0, 0.0, 0.0, 1.0];
  if (d < -0.9999) return [1.0, 0.0, 0.0, 0.0];
  const axis = normalize(cross(z, n));
  const angle = Math.acos(d);
  const s = Math.sin(angle / 2.0);
  return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(angle / 2.0)];
}

function poseNormal(pose) {
  const q = pose.orientation || {};
  return normalize(quatRotate(
    [Number(q.x) || 0.0, Number(q.y) || 0.0, Number(q.z) || 0.0, Number(q.w) || 1.0],
    [0.0, 0.0, 1.0],
  ));
}

function normalText(n) {
  return `(${fmt(n[0])}, ${fmt(n[1])}, ${fmt(n[2])})`;
}

function poseText(pose) {
  if (!pose) return "-";
  const p = pose.position || {};
  return `p=(${fmt(p.x)}, ${fmt(p.y)}, ${fmt(p.z)}), n=${normalText(poseNormal(pose))}`;
}

function clonePoseWithNormal(pose, normal) {
  const p = pose.position || {};
  const q = normalToQuat(normal);
  return {
    position: {
      x: Number(p.x) || 0.0,
      y: Number(p.y) || 0.0,
      z: Number(p.z) || 0.0,
    },
    orientation: { x: q[0], y: q[1], z: q[2], w: q[3] },
  };
}

const ros = new ROSLIB.Ros({ url: WS_URL });

ros.on("connection", () => {
  setStatus("connected", "connected");
  logEvent("connection opened");
});

ros.on("error", (err) => {
  setStatus("error", "error");
  logEvent(`error: ${err && err.message ? err.message : err}`);
});

ros.on("close", () => {
  setStatus("disconnected", "disconnected");
  logEvent("connection closed");
});

setStatus("connecting", "connecting...");
logEvent(`connecting to ${WS_URL}`);

const scanTriggerPub = new ROSLIB.Topic({
  ros: ros,
  name: SCAN_TRIGGER_TOPIC,
  messageType: "std_msgs/Empty",
});
const planesSub = new ROSLIB.Topic({
  ros: ros,
  name: PLANES_TOPIC,
  messageType: "geometry_msgs/PoseArray",
});
const planeLabelsSub = new ROSLIB.Topic({
  ros: ros,
  name: PLANE_LABELS_TOPIC,
  messageType: "std_msgs/String",
});
const workAreaPlanePub = new ROSLIB.Topic({
  ros: ros,
  name: WORK_AREA_PLANE_TOPIC,
  messageType: "geometry_msgs/PoseStamped",
});
const workAreaPlaneSub = new ROSLIB.Topic({
  ros: ros,
  name: WORK_AREA_PLANE_TOPIC,
  messageType: "geometry_msgs/PoseStamped",
});
const refinedPlaneSub = new ROSLIB.Topic({
  ros: ros,
  name: REFINED_WORK_AREA_PLANE_TOPIC,
  messageType: "geometry_msgs/PoseStamped",
});
const d405CapturePub = new ROSLIB.Topic({
  ros: ros,
  name: D405_REFINE_CAPTURE_TOPIC,
  messageType: "std_msgs/Bool",
});
const d405StatusSub = new ROSLIB.Topic({
  ros: ros,
  name: D405_REFINE_STATUS_TOPIC,
  messageType: "std_msgs/String",
});
const targetSelectionPub = new ROSLIB.Topic({
  ros: ros,
  name: TARGET_SELECTION_TOPIC,
  messageType: "geometry_msgs/PoseArray",
});
const targetSurfaceSub = new ROSLIB.Topic({
  ros: ros,
  name: TARGET_SURFACE_TOPIC,
  messageType: "geometry_msgs/PoseStamped",
});
const workAreaPixelsPub = new ROSLIB.Topic({
  ros: ros,
  name: WORK_AREA_PIXELS_TOPIC,
  messageType: "geometry_msgs/PoseArray",
});
const ftStatusSub = new ROSLIB.Topic({
  ros: ros,
  name: FT_STATUS_TOPIC,
  messageType: "std_msgs/String",
});
const ftZeroPub = new ROSLIB.Topic({
  ros: ros,
  name: FT_ZERO_TOPIC,
  messageType: "std_msgs/Bool",
});
const ftTargetConfigPub = new ROSLIB.Topic({
  ros: ros,
  name: FT_TARGET_CONFIG_TOPIC,
  messageType: "std_msgs/String",
});

let latestPlanes = null;
let latestLabels = [];
let selectedPlaneIndex = -1;
let selectedPlaneAuto = true;
let selectedNormalSign = 1.0;
let latestTargetSurface = null;
let latestWorkAreaPlane = null;
let latestRefinedPlane = null;
let latestD405Status = {};
let latestFtStatus = {};
let latestNormalForceN = null;
let latestForceSign = 1.0;
let setupComplete = false;

// ---- Camera view + minimal setup sketch -----------------------------------
const imageCanvas = $("zed-canvas");
const imageCtx = imageCanvas ? imageCanvas.getContext("2d") : null;
const sketchCanvas = $("sketch-canvas");
const sketchCtx = sketchCanvas ? sketchCanvas.getContext("2d") : null;

let currentView = "zed_raw";
let currentImageSub = null;
let imageFrameCount = 0;
let workflowMode = "target";
let sketchMode = "freehand";
let currentStroke = null;
let pendingRect = null;
let currentMouse = null;
let waitingForTargetSurface = false;
const strokesMap = { target: [], work_area: [] };

function currentStrokes() {
  return strokesMap[workflowMode] || [];
}

function rosImageBytes(data) {
  if (typeof data === "string") {
    const bin = atob(data);
    const out = new Uint8Array(bin.length);
    for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
    return out;
  }
  if (data instanceof Uint8Array) return data;
  if (Array.isArray(data)) return Uint8Array.from(data);
  if (data && typeof data === "object") return Uint8Array.from(Object.values(data));
  throw new Error(`unsupported image data type: ${typeof data}`);
}

function decodeImageData(msg) {
  const w = Number(msg.width);
  const h = Number(msg.height);
  const bytes = rosImageBytes(msg.data);
  const encoding = String(msg.encoding || "").toLowerCase();
  const step = Number(msg.step) || 0;
  let channels = 3;
  if (encoding === "rgba8" || encoding === "bgra8" || encoding === "8uc4") {
    channels = 4;
  } else if (encoding === "mono8" || encoding === "8uc1") {
    channels = 1;
  } else if (step > 0 && w > 0) {
    channels = Math.max(1, Math.floor(step / w));
  }
  const rowStep = step > 0 ? step : w * channels;
  const buf = new Uint8ClampedArray(w * h * 4);
  for (let y = 0; y < h; y++) {
    const row = y * rowStep;
    for (let x = 0; x < w; x++) {
      const i = row + x * channels;
      const j = (y * w + x) * 4;
      if (encoding === "rgb8") {
        buf[j] = bytes[i];
        buf[j + 1] = bytes[i + 1];
        buf[j + 2] = bytes[i + 2];
        buf[j + 3] = 255;
      } else if (encoding === "bgr8") {
        buf[j] = bytes[i + 2];
        buf[j + 1] = bytes[i + 1];
        buf[j + 2] = bytes[i];
        buf[j + 3] = 255;
      } else if (encoding === "rgba8") {
        buf[j] = bytes[i];
        buf[j + 1] = bytes[i + 1];
        buf[j + 2] = bytes[i + 2];
        buf[j + 3] = bytes[i + 3];
      } else if (encoding === "bgra8" || encoding === "8uc4") {
        buf[j] = bytes[i + 2];
        buf[j + 1] = bytes[i + 1];
        buf[j + 2] = bytes[i];
        buf[j + 3] = bytes[i + 3] ?? 255;
      } else if (encoding === "mono8" || encoding === "8uc1" || channels === 1) {
        const v = bytes[i];
        buf[j] = v;
        buf[j + 1] = v;
        buf[j + 2] = v;
        buf[j + 3] = 255;
      } else if (channels >= 4) {
        buf[j] = bytes[i + 2];
        buf[j + 1] = bytes[i + 1];
        buf[j + 2] = bytes[i];
        buf[j + 3] = bytes[i + 3] ?? 255;
      } else {
        buf[j] = bytes[i];
        buf[j + 1] = bytes[i + 1] ?? bytes[i];
        buf[j + 2] = bytes[i + 2] ?? bytes[i];
        buf[j + 3] = 255;
      }
    }
  }
  return new ImageData(buf, w, h);
}

function handleImageMsg(msg) {
  if (!imageCanvas || !imageCtx || !sketchCanvas) return;
  if (msg.width !== imageCanvas.width || msg.height !== imageCanvas.height) {
    imageCanvas.width = msg.width;
    imageCanvas.height = msg.height;
    sketchCanvas.width = msg.width;
    sketchCanvas.height = msg.height;
    redrawSketch();
  }
  try {
    imageCtx.putImageData(decodeImageData(msg), 0, 0);
  } catch (e) {
    logEvent(`image decode failed: ${e.message || e}`);
    return;
  }
  imageFrameCount += 1;
  setValue(
    "image-status",
    `${msg.width} x ${msg.height}, ${msg.encoding}, frames=${imageFrameCount}`,
  );
}

function subscribeView(viewName) {
  if (!VIEW_TOPICS[viewName]) return;
  if (currentImageSub) {
    try { currentImageSub.unsubscribe(); } catch (_) {}
    currentImageSub = null;
  }
  const topic = VIEW_TOPICS[viewName];
  currentImageSub = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: "sensor_msgs/Image",
    throttle_rate: 0,
    queue_size: 1,
  });
  currentImageSub.subscribe(handleImageMsg);
  imageFrameCount = 0;
  setValue("image-status", `waiting ${topic}`);
  logEvent(`subscribed to ${topic}`);
}

function switchView(viewName) {
  if (!VIEW_TOPICS[viewName]) return;
  currentView = viewName;
  const titleEl = $("view-card-title");
  if (titleEl && titleEl.firstChild) {
    titleEl.firstChild.nodeValue = `${VIEW_TITLES[viewName]} `;
  }
  setValue("view-card-topic", `(${VIEW_TOPICS[viewName]}, sensor_msgs/Image)`);
  document.querySelectorAll('input[name="view-mode"]').forEach((r) => {
    r.checked = r.value === viewName;
  });
  currentStroke = null;
  pendingRect = null;
  currentMouse = null;
  subscribeView(viewName);
  redrawSketch();
  updateSketchPanel();
}

function switchWorkflow(mode) {
  if (!strokesMap[mode]) return;
  workflowMode = mode;
  document.querySelectorAll('input[name="workflow-mode"]').forEach((r) => {
    r.checked = r.value === mode;
  });
  const targetView = mode === "target" ? "zed_raw" : "wall_front";
  if (targetView !== currentView) {
    switchView(targetView);
  }
  currentStroke = null;
  pendingRect = null;
  currentMouse = null;
  redrawSketch();
  updateSketchPanel();
}

function getNativeCoords(ev) {
  const rect = sketchCanvas.getBoundingClientRect();
  const scaleX = sketchCanvas.width / rect.width;
  const scaleY = sketchCanvas.height / rect.height;
  return {
    u: (ev.clientX - rect.left) * scaleX,
    v: (ev.clientY - rect.top) * scaleY,
  };
}

function updateSketchPanel() {
  const strokes = currentStrokes();
  const points = strokes.reduce((acc, s) => acc + s.points.length, 0);
  setValue("view-workflow-text", `${currentView} / ${workflowMode}`);
  setValue("sketch-counts", `${strokes.length} / ${points}`);
  setDisabled(
    "btn-set-target",
    workflowMode !== "target" || currentView !== "zed_raw" || points === 0,
  );
  setDisabled(
    "btn-set-work-area",
    workflowMode !== "work_area" || currentView !== "wall_front",
  );
  setDisabled("btn-d405-refine-from-camera", !latestWorkAreaPlane);
}

function redrawSketch() {
  if (!sketchCtx || !sketchCanvas) return;
  sketchCtx.clearRect(0, 0, sketchCanvas.width, sketchCanvas.height);
  sketchCtx.lineCap = "round";
  sketchCtx.lineJoin = "round";
  sketchCtx.lineWidth = 3;
  sketchCtx.strokeStyle = "#22d3ee";

  for (const s of currentStrokes()) {
    if (!s.points || s.points.length === 0) continue;
    sketchCtx.beginPath();
    sketchCtx.moveTo(s.points[0].u, s.points[0].v);
    if (s.type === "rect" && s.points.length >= 4) {
      for (let i = 1; i < 4; i++) sketchCtx.lineTo(s.points[i].u, s.points[i].v);
      sketchCtx.closePath();
    } else {
      for (let i = 1; i < s.points.length; i++) sketchCtx.lineTo(s.points[i].u, s.points[i].v);
    }
    sketchCtx.stroke();
  }

  if (pendingRect && pendingRect.end) {
    const a = pendingRect.start;
    const b = pendingRect.end;
    sketchCtx.save();
    sketchCtx.setLineDash([8, 6]);
    sketchCtx.strokeStyle = "#22d3ee";
    sketchCtx.strokeRect(a.u, a.v, b.u - a.u, b.v - a.v);
    sketchCtx.restore();
  }
  updateSketchPanel();
}

function finishPointer(ev) {
  if (pendingRect) {
    const a = pendingRect.start;
    const b = pendingRect.end || pendingRect.start;
    const u0 = Math.min(a.u, b.u);
    const u1 = Math.max(a.u, b.u);
    const v0 = Math.min(a.v, b.v);
    const v1 = Math.max(a.v, b.v);
    if (u1 - u0 > 3 && v1 - v0 > 3) {
      currentStrokes().push({
        type: "rect",
        points: [
          { u: u0, v: v0 },
          { u: u1, v: v0 },
          { u: u1, v: v1 },
          { u: u0, v: v1 },
        ],
      });
    }
    pendingRect = null;
  }
  currentStroke = null;
  try { sketchCanvas.releasePointerCapture(ev.pointerId); } catch (_) {}
  redrawSketch();
}

function posesFromStrokes(strokes) {
  const poses = [];
  for (const s of strokes) {
    for (const pt of s.points) {
      poses.push({
        position: { x: pt.u, y: pt.v, z: 0.0 },
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
      });
    }
  }
  return poses;
}

function publishPixels(pub, topicName, frameId, strokes) {
  const poses = posesFromStrokes(strokes);
  if (poses.length === 0) return false;
  pub.publish(new ROSLIB.Message({
    header: { stamp: nowRosTime(), frame_id: frameId },
    poses,
  }));
  logEvent(`published ${poses.length} points to ${topicName} (${frameId})`);
  return true;
}

function publishTargetSelection() {
  if (workflowMode !== "target" || currentView !== "zed_raw") return;
  if (!publishPixels(
    targetSelectionPub,
    TARGET_SELECTION_TOPIC,
    "zed_raw",
    currentStrokes(),
  )) return;
  waitingForTargetSurface = true;
  logEvent("waiting for target surface, then switch to Work Area");
}

function publishWorkAreaSelection() {
  if (workflowMode !== "work_area" || currentView !== "wall_front") return;
  let strokes = currentStrokes();
  const drawn = strokes.reduce((acc, s) => acc + s.points.length, 0) > 0;
  if (!drawn) {
    const w = sketchCanvas.width;
    const h = sketchCanvas.height;
    if (w <= 1 || h <= 1) {
      logEvent("wall_front image not ready");
      return;
    }
    strokes = [{
      type: "rect",
      points: [
        { u: 0, v: 0 },
        { u: w - 1, v: 0 },
        { u: w - 1, v: h - 1 },
        { u: 0, v: h - 1 },
      ],
    }];
    logEvent("work area empty -> using full wall_front view");
  }
  publishPixels(workAreaPixelsPub, WORK_AREA_PIXELS_TOPIC, "wall_front", strokes);
}

if (sketchCanvas) {
  sketchCanvas.addEventListener("pointerdown", (ev) => {
    ev.preventDefault();
    sketchCanvas.setPointerCapture(ev.pointerId);
    const c = getNativeCoords(ev);
    currentMouse = c;
    if (sketchMode === "rect") {
      pendingRect = { start: c, end: c };
    } else {
      currentStroke = { type: "freehand", points: [c] };
      currentStrokes().push(currentStroke);
    }
    redrawSketch();
  });
  sketchCanvas.addEventListener("pointermove", (ev) => {
    currentMouse = getNativeCoords(ev);
    if (pendingRect) {
      pendingRect.end = currentMouse;
      redrawSketch();
    } else if (currentStroke) {
      currentStroke.points.push(currentMouse);
      redrawSketch();
    }
  });
  sketchCanvas.addEventListener("pointerup", finishPointer);
  sketchCanvas.addEventListener("pointercancel", finishPointer);
  sketchCanvas.addEventListener("pointerleave", () => {
    currentMouse = null;
  });
}

document.querySelectorAll('input[name="view-mode"]').forEach((r) => {
  r.addEventListener("change", () => switchView(r.value));
});

document.querySelectorAll('input[name="workflow-mode"]').forEach((r) => {
  r.addEventListener("change", () => switchWorkflow(r.value));
});

document.querySelectorAll('input[name="sketch-mode"]').forEach((r) => {
  r.addEventListener("change", () => {
    sketchMode = r.value;
    currentStroke = null;
    pendingRect = null;
    redrawSketch();
  });
});

bindClick("btn-undo", () => {
  const strokes = currentStrokes();
  if (strokes.length > 0) strokes.pop();
  redrawSketch();
});

bindClick("btn-clear", () => {
  currentStrokes().length = 0;
  currentStroke = null;
  pendingRect = null;
  redrawSketch();
});

bindClick("btn-set-target", publishTargetSelection);
bindClick("btn-set-work-area", publishWorkAreaSelection);
bindClick("btn-d405-refine-from-camera", requestD405Refine);

function labelForPlane(index) {
  return latestLabels.find((p) => Number(p.id) === index) || {};
}

function bestPlaneIndex() {
  if (!latestPlanes || !latestPlanes.poses || latestPlanes.poses.length === 0) {
    return -1;
  }
  const wall = latestLabels.find((p) => p.type === "wall");
  if (wall && Number.isInteger(Number(wall.id))) return Number(wall.id);
  if (latestLabels.length > 0) {
    let best = latestLabels[0];
    for (const label of latestLabels) {
      if (Number(label.n_inliers || 0) > Number(best.n_inliers || 0)) {
        best = label;
      }
    }
    return Number(best.id);
  }
  return 0;
}

function selectedPlaneNormal() {
  if (!latestPlanes || selectedPlaneIndex < 0) return null;
  const pose = latestPlanes.poses[selectedPlaneIndex];
  if (!pose) return null;
  const n = poseNormal(pose);
  return [
    n[0] * selectedNormalSign,
    n[1] * selectedNormalSign,
    n[2] * selectedNormalSign,
  ];
}

function renderPlaneList() {
  const list = $("plane-list");
  if (!list) return;
  list.textContent = "";
  if (!latestPlanes || !latestPlanes.poses || latestPlanes.poses.length === 0) {
    const empty = document.createElement("div");
    empty.className = "plane-empty";
    empty.textContent = "no planes";
    list.appendChild(empty);
    return;
  }
  latestPlanes.poses.forEach((pose, index) => {
    const label = labelForPlane(index);
    const normal = poseNormal(pose);
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "plane-option";
    if (index === selectedPlaneIndex) btn.classList.add("selected");
    const kind = label.type || "plane";
    const inliers = Number.isFinite(Number(label.n_inliers))
      ? `${Number(label.n_inliers)} pts`
      : "pts -";
    const size = Array.isArray(label.size)
      ? `${fmt(label.size[0], 2)} x ${fmt(label.size[1], 2)} m`
      : "-";
    btn.textContent = `#${index} ${kind} | ${inliers} | ${size} | n=${normalText(normal)}`;
    btn.addEventListener("click", () => {
      selectedPlaneIndex = index;
      selectedPlaneAuto = false;
      selectedNormalSign = 1.0;
      updateSurfacePanel();
      renderPlaneList();
    });
    list.appendChild(btn);
  });
}

function updateSurfacePanel() {
  const count = latestPlanes && latestPlanes.poses ? latestPlanes.poses.length : 0;
  const selected = selectedPlaneIndex >= 0 ? `#${selectedPlaneIndex}` : "-";
  setValue("zed-scan-state", `${count} plane(s), selected ${selected}`);
  setValue(
    "target-surface",
    latestTargetSurface
      ? `${latestTargetSurface.header.frame_id || "-"} ${poseText(latestTargetSurface.pose)}`
      : "-",
  );

  const selectedPose =
    latestPlanes && selectedPlaneIndex >= 0
      ? latestPlanes.poses[selectedPlaneIndex]
      : null;
  const label = selectedPlaneIndex >= 0 ? labelForPlane(selectedPlaneIndex) : {};
  setValue(
    "selected-plane",
    selectedPose
      ? `#${selectedPlaneIndex} ${label.type || "plane"} ${poseText(selectedPose)}`
      : "-",
  );
  const normal = selectedPlaneNormal();
  setValue(
    "selected-normal",
    normal ? `${normalText(normal)} sign=${selectedNormalSign > 0 ? "+1" : "-1"}` : "-",
  );
  setValue(
    "work-area-plane",
    latestWorkAreaPlane
      ? `${latestWorkAreaPlane.header.frame_id || "-"} ${poseText(latestWorkAreaPlane.pose)}`
      : "-",
  );
  const d405State = latestD405Status.state || "waiting";
  const d405Ok = latestD405Status.ok === true ? "ok" : "not ready";
  const d405Extra = Number.isFinite(Number(latestD405Status.shift_m))
    ? ` shift=${(Number(latestD405Status.shift_m) * 1000).toFixed(1)} mm`
    : "";
  setValue("d405-refine-state", `${d405State} (${d405Ok})${d405Extra}`);
  setValue(
    "refined-plane",
    latestRefinedPlane
      ? `${latestRefinedPlane.header.frame_id || "-"} ${poseText(latestRefinedPlane.pose)}`
      : "-",
  );
  setDisabled("btn-adopt-plane", !selectedPose);
  setDisabled("btn-flip-normal", !selectedPose);
  setDisabled("btn-d405-refine", !latestWorkAreaPlane && !selectedPose);
  updateSketchPanel();
}

function publishSelectedPlane(reason) {
  if (!latestPlanes || selectedPlaneIndex < 0) {
    logEvent("surface setup: no selected ZED plane");
    return false;
  }
  const sourcePose = latestPlanes.poses[selectedPlaneIndex];
  if (!sourcePose) {
    logEvent("surface setup: selected plane missing");
    return false;
  }
  const normal = selectedPlaneNormal();
  const msg = new ROSLIB.Message({
    header: {
      stamp: nowRosTime(),
      frame_id: latestPlanes.header.frame_id || "zed_left_camera_frame",
    },
    pose: clonePoseWithNormal(sourcePose, normal),
  });
  workAreaPlanePub.publish(msg);
  latestWorkAreaPlane = msg;
  logEvent(`${reason}: published ${WORK_AREA_PLANE_TOPIC} from ZED plane #${selectedPlaneIndex}`);
  updateSurfacePanel();
  return true;
}

function requestD405Refine() {
  if (!latestWorkAreaPlane && !publishSelectedPlane("surface setup")) {
    return;
  }
  latestD405Status = { ok: false, state: "capture_requested" };
  d405CapturePub.publish(new ROSLIB.Message({ data: true }));
  logEvent(`published ${D405_REFINE_CAPTURE_TOPIC}`);
  updateSurfacePanel();
}

planesSub.subscribe((msg) => {
  latestPlanes = msg;
  selectedPlaneIndex = bestPlaneIndex();
  selectedPlaneAuto = true;
  selectedNormalSign = 1.0;
  logEvent(`received ${PLANES_TOPIC}: ${msg.poses ? msg.poses.length : 0} plane(s)`);
  renderPlaneList();
  updateSurfacePanel();
});
logEvent(`subscribed to ${PLANES_TOPIC}`);

planeLabelsSub.subscribe((msg) => {
  try {
    const payload = JSON.parse(msg.data || "{}");
    latestLabels = Array.isArray(payload.planes) ? payload.planes : [];
  } catch (_) {
    latestLabels = [];
  }
  if (selectedPlaneAuto || selectedPlaneIndex < 0) {
    selectedPlaneIndex = bestPlaneIndex();
  }
  renderPlaneList();
  updateSurfacePanel();
});
logEvent(`subscribed to ${PLANE_LABELS_TOPIC}`);

workAreaPlaneSub.subscribe((msg) => {
  latestWorkAreaPlane = msg;
  updateSurfacePanel();
});
logEvent(`subscribed to ${WORK_AREA_PLANE_TOPIC}`);

refinedPlaneSub.subscribe((msg) => {
  latestRefinedPlane = msg;
  updateSurfacePanel();
});
logEvent(`subscribed to ${REFINED_WORK_AREA_PLANE_TOPIC}`);

d405StatusSub.subscribe((msg) => {
  try {
    latestD405Status = JSON.parse(msg.data || "{}");
  } catch (_) {
    latestD405Status = { ok: false, state: msg.data || "unknown" };
  }
  updateSurfacePanel();
});
logEvent(`subscribed to ${D405_REFINE_STATUS_TOPIC}`);

targetSurfaceSub.subscribe((msg) => {
  latestTargetSurface = msg;
  logEvent(`received ${TARGET_SURFACE_TOPIC}`);
  if (waitingForTargetSurface) {
    waitingForTargetSurface = false;
    switchWorkflow("work_area");
  }
  updateSurfacePanel();
});
logEvent(`subscribed to ${TARGET_SURFACE_TOPIC}`);

bindClick("btn-zed-scan", () => {
  scanTriggerPub.publish(new ROSLIB.Message({}));
  latestPlanes = null;
  latestLabels = [];
  selectedPlaneIndex = -1;
  selectedPlaneAuto = true;
  selectedNormalSign = 1.0;
  logEvent(`published ${SCAN_TRIGGER_TOPIC}`);
  renderPlaneList();
  updateSurfacePanel();
});

bindClick("btn-adopt-plane", () => publishSelectedPlane("surface setup"));

bindClick("btn-flip-normal", () => {
  selectedNormalSign *= -1.0;
  if (latestWorkAreaPlane) {
    publishSelectedPlane("surface normal flipped");
  } else {
    updateSurfacePanel();
  }
});

bindClick("btn-d405-refine", requestD405Refine);

function derivedAbortForce(target) {
  return Math.round(Math.max(5.0, target + 3.0, target * 2.0) * 10) / 10;
}

function derivedContactForce(target) {
  return Math.round(Math.max(0.4, target * 0.4) * 10) / 10;
}

function derivedWarnForce(target, abort) {
  const warn = Math.max(target + 0.8, target * 1.5);
  return Math.round(Math.min(warn, abort - 0.2) * 10) / 10;
}

function derivedTorqueWarn(torqueAbort) {
  return Math.round(Math.max(0.05, torqueAbort * 0.5) * 100) / 100;
}

function activeNormalText(payload) {
  if (Array.isArray(payload.surface_normal_base) && payload.surface_normal_base.length >= 3) {
    return normalText(payload.surface_normal_base);
  }
  if (latestRefinedPlane) return normalText(poseNormal(latestRefinedPlane.pose));
  if (latestWorkAreaPlane) return normalText(poseNormal(latestWorkAreaPlane.pose));
  return "-";
}

function updateForceSetupPanel() {
  const payload = latestFtStatus || {};
  const hasFtStatus = Object.keys(payload).length > 0;
  const state = payload.state || "waiting";
  const ok = payload.ok === true ? "ok" : "not ready";
  setValue("ft-state", hasFtStatus ? `${state} (${ok})` : "no /ft/status");
  setValue(
    "ft-surface",
    hasFtStatus
      ? `${payload.surface_source || "-"} ${payload.bias_ready ? "bias ready" : "bias not ready"}`
      : "start ft_normal_controller",
  );
  setValue("ft-active-normal", activeNormalText(payload));
  setValue(
    "ft-normal-force",
    Number.isFinite(latestNormalForceN) ? `${fmtN(latestNormalForceN)} N` : "-",
  );
  setValue(
    "ft-target-summary",
    `target=${fmtN(payload.target_force_n, 1)} N, ` +
      `contact=${fmtN(payload.contact_threshold_n, 1)} N, ` +
      `abort=${fmtN(payload.abort_force_n, 1)} N, ` +
      `sign=${fmtN(latestForceSign, 0)}`,
  );
  setValue(
    "ft-correction",
    Number.isFinite(Number(payload.correction_m))
      ? `${(Number(payload.correction_m) * 1000).toFixed(1)} mm`
      : "-",
  );
  const torqueSigned = Number(payload.torque_balance_nm);
  const torqueAbs = Number(payload.torque_balance_abs_nm);
  const torqueAbort = Number(payload.torque_abort_nm);
  setValue(
    "ft-balance-torque",
    Number.isFinite(torqueSigned)
      ? `signed=${torqueSigned.toFixed(3)} Nm, abs=${torqueAbs.toFixed(3)} / ${torqueAbort.toFixed(3)} Nm`
      : "-",
  );
  const rotRad = Number(payload.orientation_correction_rad);
  setValue(
    "ft-orientation-correction",
    Number.isFinite(rotRad)
      ? `${(rotRad * 180 / Math.PI).toFixed(2)} deg`
      : "-",
  );
  setValue(
    "ft-setup-state",
    setupComplete
      ? `saved target=${$("ft-target-input").value} N, abort=${$("ft-abort-input").value} N, torque=${$("ft-torque-abort-input").value} Nm`
      : "not captured",
  );
  setDisabled("btn-ft-zero", !hasFtStatus);
  setDisabled("btn-ft-flip-sign", !hasFtStatus);
  setDisabled("btn-ft-apply-target", !hasFtStatus);
  setDisabled("btn-ft-capture-target", !hasFtStatus || !Number.isFinite(latestNormalForceN));
}

ftStatusSub.subscribe((msg) => {
  let payload = {};
  try {
    payload = JSON.parse(msg.data || "{}");
  } catch (_) {
    payload = { state: msg.data || "unknown" };
  }
  latestFtStatus = payload;
  if (Number.isFinite(Number(payload.normal_force_n))) {
    latestNormalForceN = Number(payload.normal_force_n);
  }
  if (Number.isFinite(Number(payload.force_sign))) {
    latestForceSign = Number(payload.force_sign) >= 0 ? 1.0 : -1.0;
  }
  updateForceSetupPanel();
});
logEvent(`subscribed to ${FT_STATUS_TOPIC}`);

function publishFtConfig(config, reason) {
  ftTargetConfigPub.publish(new ROSLIB.Message({
    data: JSON.stringify(config),
  }));
  logEvent(
    `${reason}: target=${config.target_force_n.toFixed(1)}N ` +
    `abort=${config.abort_force_n.toFixed(1)}N ` +
    `torque_abort=${Number(config.torque_abort_nm).toFixed(2)}Nm ` +
    `sign=${config.force_sign.toFixed(0)}`,
  );
}

function readForceTargetInputs() {
  const target = Number($("ft-target-input").value);
  const abort = Number($("ft-abort-input").value);
  const torqueAbort = Number($("ft-torque-abort-input").value);
  if (!Number.isFinite(target) || target <= 0.0) {
    logEvent("force setup: invalid target N");
    return null;
  }
  if (!Number.isFinite(abort) || abort <= target) {
    logEvent("force setup: abort N must be greater than target N");
    return null;
  }
  if (!Number.isFinite(torqueAbort) || torqueAbort <= 0.0) {
    logEvent("force setup: invalid torque abort Nm");
    return null;
  }
  const contact = derivedContactForce(target);
  const warn = derivedWarnForce(target, abort);
  const torqueWarn = derivedTorqueWarn(torqueAbort);
  return {
    target_force_n: target,
    contact_threshold_n: Math.round(contact * 10) / 10,
    warn_force_n: Math.round(warn * 10) / 10,
    abort_force_n: Math.round(abort * 10) / 10,
    torque_warn_nm: torqueWarn,
    torque_abort_nm: Math.round(torqueAbort * 100) / 100,
  };
}

function applyForceTarget(reason, forceSignOverride = null) {
  const values = readForceTargetInputs();
  if (!values) return false;
  const forceSign =
    forceSignOverride === null
      ? latestForceSign
      : (forceSignOverride >= 0 ? 1.0 : -1.0);
  publishFtConfig({ ...values, force_sign: forceSign }, reason);
  latestForceSign = forceSign;
  setupComplete = true;
  updateForceSetupPanel();
  return true;
}

bindClick("btn-ft-zero", () => {
  ftZeroPub.publish(new ROSLIB.Message({ data: true }));
  setupComplete = false;
  logEvent(`published ${FT_ZERO_TOPIC}`);
  updateForceSetupPanel();
});

bindClick("btn-ft-flip-sign", () => {
  const nextSign = -latestForceSign;
  const values = readForceTargetInputs();
  if (!values) return;
  publishFtConfig({ ...values, force_sign: nextSign }, "force sign flipped");
  latestForceSign = nextSign;
  setupComplete = true;
  updateForceSetupPanel();
});

bindClick("btn-ft-capture-target", () => {
  if (!Number.isFinite(latestNormalForceN)) {
    logEvent("force setup: /ft/status.normal_force_n not received");
    return;
  }
  const target = Math.abs(latestNormalForceN);
  if (target < 0.2) {
    logEvent("force setup: normal force too small");
    return;
  }
  $("ft-target-input").value = target.toFixed(1);
  $("ft-abort-input").value = derivedAbortForce(target).toFixed(1);
  const forceSign = latestNormalForceN < 0.0 ? -latestForceSign : latestForceSign;
  applyForceTarget("force target captured", forceSign);
});

bindClick("btn-ft-apply-target", () => {
  applyForceTarget("force target applied");
});

renderPlaneList();
subscribeView(currentView);
updateSurfacePanel();
updateForceSetupPanel();
