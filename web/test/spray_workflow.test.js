"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const { loadUI } = require("./ui_harness.js");
const corners = [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]];
const lock = {
  source: "zed", state: "locked", accepted: true, catalog_generation: "catalog-1", plane_id: "plane-1",
  plane_generation_id: "zed:catalog-1:plane-1:1750000000123456789", frame_id: "map",
  center: [0.5, 0.5, 1], normal: [0, 0, 1], corners, stamp: { sec: 1750000000, nanosec: 123456789 },
};
const target = {
  source: "zed", state: "locked", accepted: true, mode: "target",
  plane_generation_id: lock.plane_generation_id, work_area_id: "", selection_id: "",
  frame_id: "map", position: [0.5, 0.5, 1], orientation: [0, 0, 0, 1], corners,
  front_extent: [], view_width: 0, view_height: 0, target_stamp: lock.stamp,
};
const readiness = {
  process_mode: "spray", ready: false, running: false, target_selected: true, work_area_selected: true,
  work_area_id: "wa-1", plane_generation_id: lock.plane_generation_id,
};

async function selectTarget(browserTimeMs) {
  const ui = loadUI();
  if (browserTimeMs !== undefined) ui.evaluate(`Date.now = () => ${browserTimeMs}`);
  ui.ros.emit("connection");
  ui.emit("/painting_system/process_mode", { mode: "spray" });
  ui.frame("/zed/zed_node/rgb/color/rect/image");
  await ui.rectangle();
  await ui.element("btn-set-target").fire("click");
  assert.equal(ui.published.at(-1).name, "/target_selection_pixels");
  ui.emit("/perception/target_planes", { generation: "catalog-1", planes: [{ id: "plane-1", corners }] });
  const checkbox = ui.element("plane-candidates").children[0].children[0];
  checkbox.checked = true;
  await checkbox.fire("change");
  await ui.element("btn-refine-planes").fire("click");
  assert.deepEqual(JSON.parse(ui.published.at(-1).message.data), { generation: "catalog-1", ids: ["plane-1"] });
  ui.emit("/perception/zed_target_lock", lock);
  ui.emit("/painting_system/planes", { source: "zed", generation: "catalog-1", selected: ["plane-1"], measured: ["plane-1"], state: "ready", active_id: "plane-1", running: false });
  ui.emit("/perception/zed_surface_status", { ...target, accepted: false, state: "invalidated", plane_generation_id: "" });
  ui.emit("/perception/zed_surface_status", target);
  assert.equal(ui.evaluate("workflowMode"), "work_area");
  assert.equal(ui.evaluate("currentView"), "wall_front");
  assert.equal(ui.element("btn-set-work-area").disabled, true);
  ui.frame("/perception/wall_front_view");
  return ui;
}

async function selectArea(ui) {
  await ui.rectangle();
  assert.equal(ui.element("btn-set-work-area").disabled, false);
  await ui.element("btn-set-work-area").fire("click");
  const pixels = ui.published.at(-1);
  assert.equal(pixels.name, "/work_area_pixels");
  assert.equal(pixels.message.header.frame_id, "wall_front");
  assert.equal(pixels.message.poses.length, 4);
  const envelope = ui.published.at(-2);
  assert.equal(envelope.name, "/painting_system/zed_work_area_request");
  const request = JSON.parse(envelope.message.data);
  assert.deepEqual(request, {
    source: "zed", plane_generation_id: lock.plane_generation_id,
    header: pixels.message.header,
    pixels: [[10, 10], [90, 10], [90, 70], [10, 70]],
  });
  assert.deepEqual(pixels.message.poses.map(pose => [pose.position.x, pose.position.y]), request.pixels);
  const stamp = pixels.message.header.stamp;
  const selectionId = String(BigInt(stamp.sec) * 1000000000n + BigInt(stamp.nanosec));
  assert.equal(ui.evaluate("zedSelection.selectionId"), selectionId);
  const area = { ...target, mode: "work_area", work_area_id: "wa-1", selection_id: selectionId,
    front_extent: corners, view_width: 100, view_height: 80 };
  ui.emit("/perception/zed_surface_status", { ...target, state: "invalidated", accepted: false });
  ui.emit("/perception/zed_surface_status", { ...area, selection_id: "old" });
  assert.equal(ui.evaluate("workflowMode"), "work_area");
  ui.emit("/perception/zed_surface_status", area);
  assert.equal(ui.evaluate("workflowMode"), "path");
  ui.emit("/painting_system/readiness", readiness);
  return area;
}

function validate(ui) {
  const plan = { ...readiness, state: "generated", path_id: "path-1", plan_hash: "hash-1", validated: true };
  ui.emit("/painting_system/plan_status", plan);
  ui.emit("/painting_system/readiness", { ...plan, ready: true, plan_validated: true });
  return plan;
}

test("spray envelope binds generation and pixel identity without browser/ROS clock alignment", async () => {
  const ui = await selectTarget(1000);
  await selectArea(ui);
  const pixels = ui.published.find(entry => entry.name === "/work_area_pixels");
  assert.equal(pixels.message.header.stamp.sec, 1);
  assert.equal(ui.evaluate("paintingDerivedState().surfaceAccepted"), true);
});

test("paint work-area selection publishes legacy pixels and D405 refinement only", async () => {
  const ui = loadUI();
  ui.ros.emit("connection");
  ui.emit("/painting_system/process_mode", { mode: "paint" });
  ui.emit("/painting_system/readiness", { process_mode: "paint", target_selected: true });
  ui.evaluate("switchToWorkAreaMode()");
  ui.frame("/perception/wall_front_view");
  await ui.rectangle();
  assert.equal(ui.element("btn-set-work-area").disabled, false);
  await ui.element("btn-set-work-area").fire("click");
  assert.deepEqual(ui.published.slice(-2).map(entry => entry.name), ["/work_area_pixels", "/refine_work_area"]);
  assert.equal(ui.published.some(entry => entry.name === "/painting_system/zed_work_area_request"), false);
});

test("a pending mode change prevents both spray work-area publications", async () => {
  const ui = await selectTarget();
  await ui.rectangle();
  ui.element("process-mode").value = "paint";
  await ui.element("process-mode").fire("change");
  const before = ui.published.length;
  await ui.element("btn-set-work-area").fire("click");
  assert.equal(ui.published.length, before);
});

test("raw target to ZED lock to frontal rectangle to generated and validated coverage", async () => {
  const ui = await selectTarget();
  const area = await selectArea(ui);
  assert.equal(ui.element("btn-fill-work-area").disabled, false);
  assert.equal(ui.element("btn-execute").hidden, true);
  assert.equal(ui.element("btn-execute").disabled, true);
  assert.equal(ui.element("btn-run-robot").disabled, true);
  const before = ui.published.length;
  await ui.element("btn-execute").fire("click");
  await ui.rectangle();
  assert.equal(ui.published.length, before);
  assert.equal(ui.evaluate("strokesMap.path.length"), 0);
  await ui.element("btn-fill-work-area").fire("click");
  assert.equal(ui.published.at(-1).name, "/fill_work_area");
  const plan = validate(ui);
  assert.equal(ui.element("btn-run-robot").disabled, false);
  for (const topic of ["/perception/d405_surface_refinement_status", "/target_refine_status", "/work_area_refine_status"]) {
    ui.emit(topic, { accepted: false, state: "invalidated", mode: "work_area" });
    ui.emit(topic, { data: "broken JSON" }, true);
  }
  assert.equal(ui.element("btn-run-robot").disabled, false);
  ui.emit("/painting_system/readiness", { ...plan, ready: true, plan_hash: "old-hash" });
  assert.equal(ui.element("btn-run-robot").disabled, true);
  ui.emit("/painting_system/readiness", { ...plan, ready: false, plan_validated: false,
    checks: { current_plan_validated: false }, plan_blockers: ["COLLISION_CHECK_FAILED"] });
  assert.equal(ui.element("btn-run-robot").disabled, true);
  assert.equal(ui.evaluate("paintingDerivedState().planValidated"), false);
  assert.match(ui.element("painting-blockers").textContent, /COLLISION_CHECK_FAILED/);
  ui.emit("/painting_system/readiness", { ...plan, ready: true, plan_validated: true });
  await ui.element("btn-run-robot").fire("click");
  assert.equal(ui.published.at(-1).name, "/sketch_execute");
  assert.equal(ui.published.at(-1).message.data, true);
  assert.equal(ui.published.some(entry => entry.name === "/refine_work_area" || entry.name === "/sketch_pixels"), false);
  ui.emit("/perception/zed_surface_status", { ...area, state: "invalidated", accepted: false });
  assert.equal(ui.element("btn-run-robot").disabled, true);
});

test("mode switches clear selections and require matching acknowledgement and new ZED identities", async () => {
  const ui = await selectTarget();
  const area = await selectArea(ui);
  await ui.element("btn-fill-work-area").fire("click");
  const plan = validate(ui);
  ui.element("process-mode").value = "paint";
  await ui.element("process-mode").fire("change");
  assert.equal(ui.element("btn-run-robot").disabled, true);
  ui.emit("/painting_system/process_mode", { mode: "spray" });
  assert.equal(ui.evaluate("processModePending"), true);
  ui.emit("/painting_system/process_mode", { mode: "paint" });
  assert.equal(ui.evaluate("processModePending"), false);
  ui.element("process-mode").value = "spray";
  await ui.element("process-mode").fire("change");
  ui.emit("/painting_system/process_mode", { mode: "spray" });
  ui.emit("/perception/zed_target_lock", lock);
  ui.emit("/perception/zed_surface_status", area);
  ui.emit("/painting_system/plan_status", plan);
  ui.emit("/painting_system/readiness", { ...plan, ready: true });
  assert.equal(ui.evaluate("workflowMode"), "target");
  assert.equal(ui.evaluate("paintingDerivedState().targetSelected"), false);
  assert.equal(ui.element("btn-run-robot").disabled, true);
});

test("paint still requires D405 acceptance and operator free-space confirmation", async () => {
  const ui = loadUI();
  ui.ros.emit("connection");
  ui.emit("/painting_system/process_mode", { mode: "paint" });
  const plan = { ...readiness, process_mode: "paint", plane_generation_id: "d405-1", state: "generated",
    path_id: "paint-path", plan_hash: "paint-hash", target_force_n: 5, validated: true };
  ui.emit("/painting_system/readiness", { ...plan, ready: true, plan_validated: true, checks: { free_space_confirmed: true } });
  ui.emit("/painting_system/plan_status", plan);
  assert.equal(ui.element("btn-run-robot").disabled, true);
  ui.emit("/perception/d405_surface_refinement_status", { ...plan, mode: "work_area", accepted: true });
  assert.equal(ui.evaluate("paintingDerivedState().surfaceAccepted"), true);
  assert.equal(ui.element("btn-run-robot").disabled, true);
  ui.evaluate("switchToPathMode()");
  await ui.element("btn-fill-work-area").fire("click");
  const generated = { ...plan, path_id: "paint-path-2", plan_hash: "paint-hash-2" };
  ui.emit("/painting_system/plan_status", generated);
  ui.emit("/painting_system/readiness", { ...generated, ready: true, plan_validated: true, checks: { free_space_confirmed: true } });
  ui.element("free-space-confirmed").checked = true;
  await ui.element("free-space-confirmed").fire("change");
  assert.equal(ui.element("btn-run-robot").disabled, false);
  ui.emit("/perception/d405_surface_refinement_status", { ...plan, mode: "work_area", accepted: false, state: "invalidated" });
  assert.equal(ui.element("btn-run-robot").disabled, true);
});

test("dry-run spray cannot use D405 acceptance or a backend plan without ZED selection", () => {
  const ui = loadUI();
  ui.ros.emit("connection");
  ui.emit("/painting_system/process_mode", { mode: "spray", dry_run: true });
  ui.emit("/perception/d405_surface_refinement_status", { ...readiness, mode: "work_area", accepted: true });
  validate(ui);
  assert.equal(ui.element("btn-run-robot").disabled, true);
  assert.equal(ui.evaluate("paintingDerivedState().surfaceAccepted"), false);
});
