"use strict";

const assert = require("node:assert/strict");
const { loadUI } = require("./ui_harness.js");
const corners = [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]];
const lock = {
  source: "zed", state: "locked", accepted: true, catalog_generation: "1000000000", plane_id: "plane-1",
  plane_generation_id: "zed:1000000000:plane-1:1750000000123456789", frame_id: "map",
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

async function selectTarget(browserTimeMs, planningOnly = false) {
  const ui = loadUI();
  ui.evaluate("Date.now = () => 1000");
  ui.ros.emit("connection");
  ui.emit("/painting_system/process_mode", { mode: "spray", planning_only: planningOnly });
  ui.frame("/zed/zed_node/rgb/color/rect/image");
  await ui.rectangle();
  await ui.element("btn-set-target").fire("click");
  assert.equal(ui.published.at(-1).name, "/target_selection_pixels");
  if (browserTimeMs !== undefined) ui.evaluate(`Date.now = () => ${browserTimeMs}`);
  ui.emit("/perception/target_planes", { generation: "1000000000", planes: [{ id: "plane-1", corners }] });
  const checkbox = ui.element("plane-candidates").children[0].children[0];
  checkbox.checked = true;
  await checkbox.fire("change");
  await ui.element("btn-refine-planes").fire("click");
  assert.deepEqual(JSON.parse(ui.published.at(-1).message.data), { generation: "1000000000", ids: ["plane-1"] });
  ui.emit("/perception/zed_target_lock", lock);
  ui.emit("/painting_system/planes", { source: "zed", generation: "1000000000", selected: ["plane-1"], measured: ["plane-1"], state: "ready", active_id: "plane-1", running: false });
  ui.emit("/perception/zed_surface_status", { ...target, accepted: false, state: "invalidated", plane_generation_id: "" });
  ui.emit("/perception/zed_surface_status", target);
  assert.equal(ui.evaluate("workflowMode"), "target");
  await ui.element("btn-workflow-next").fire("click");
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
  assert.equal(ui.evaluate("workflowMode"), "work_area");
  ui.emit("/painting_system/readiness", readiness);
  await ui.element("btn-workflow-next").fire("click");
  assert.equal(ui.evaluate("workflowMode"), "path");
  return area;
}

function validate(ui) {
  const plan = { ...readiness, state: "generated", path_id: "path-1", plan_hash: "hash-1", validated: true };
  ui.emit("/painting_system/plan_status", plan);
  ui.emit("/painting_system/readiness", { ...plan, ready: true, plan_validated: true });
  return plan;
}


module.exports={selectTarget,selectArea,validate,readiness,lock,target};
