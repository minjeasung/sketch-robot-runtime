"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const { ZedSurfaceGate, stampPathId, matchesPlan, rectangleReady } = require("../js/zed_surface_gate.js");

const stamp = { sec: 1750000000, nanosec: 123456789 };
const corners = [[0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]];
const lock = {
  source: "zed", accepted: true, state: "locked", catalog_generation: "catalog-1",
  plane_id: "plane-1", plane_generation_id: "zed:catalog-1:plane-1:1750000000123456789",
  frame_id: "map", center: [0.5, 0.5, 1], normal: [0, 0, 1], corners, stamp,
};
const target = {
  source: "zed", mode: "target", accepted: true, state: "locked",
  plane_generation_id: lock.plane_generation_id, work_area_id: "", selection_id: "",
  frame_id: "map", position: [0.5, 0.5, 1], orientation: [0, 0, 0, 1],
  corners, front_extent: corners, view_width: 100, view_height: 80, target_stamp: stamp,
};
const area = { ...target, mode: "work_area", work_area_id: "wa-1", selection_id: "1750000001123456789" };
function selected() {
  const gate = new ZedSurfaceGate();
  gate.setMode("spray");
  gate.setCatalog("catalog-1");
  gate.selectPlanes("catalog-1", ["plane-1"]);
  assert.equal(gate.receiveTarget(lock), "locked");
  assert.equal(gate.receiveSurface(target), "target");
  return gate;
}
function accepted() {
  const gate = selected();
  gate.beginWorkArea({ sec: 1750000001, nanosec: 123456789 });
  assert.equal(gate.receiveSurface(area), "work_area");
  return gate;
}

test("pixel selection identities preserve nanoseconds beyond Number precision", () => {
  assert.equal(stampPathId(stamp), "1750000000123456789");
  for (const bad of [null, {}, { sec: 0, nanosec: 0 }, { sec: 1, nanosec: 1e9 }, { sec: "1", nanosec: 0 }]) {
    assert.equal(stampPathId(bad), "");
  }
});

test("spray work areas require one nondegenerate rectangle inside Wall Front", () => {
  const rect = { type: "rect", points: [{ u: 10, v: 10 }, { u: 90, v: 10 }, { u: 90, v: 70 }, { u: 10, v: 70 }] };
  assert.equal(rectangleReady([rect], 100, 80), true);
  assert.equal(rectangleReady([], 100, 80), false);
  assert.equal(rectangleReady([rect, rect], 100, 80), false);
  assert.equal(rectangleReady([{ ...rect, type: "freehand" }], 100, 80), false);
  assert.equal(rectangleReady([rect], 80, 80), false);
  assert.equal(rectangleReady([{ type: "rect", points: Array(4).fill({ u: 10, v: 10 }) }], 100, 80), false);
});

test("surface status arriving before the target lock still completes the selection", () => {
  const gate = new ZedSurfaceGate();
  gate.setMode("spray");
  gate.setCatalog("catalog-1");
  gate.selectPlanes("catalog-1", ["plane-1"]);
  assert.equal(gate.receiveSurface(target), "ignore");
  assert.equal(gate.receiveTarget(lock), "locked");
  assert.equal(gate.target.plane_generation_id, lock.plane_generation_id);
});

test("a target status can precede the first frontal image and work-area invalidation", () => {
  const gate = selected();
  assert.equal(gate.receiveSurface({ ...target, front_extent: [], view_width: 0, view_height: 0 }), "target");
  gate.beginWorkArea({ sec: 1750000001, nanosec: 123456789 });
  assert.equal(gate.receiveSurface({ ...target, accepted: false, state: "invalidated" }), "invalidated");
  assert.equal(gate.surface, null);
  assert.equal(gate.receiveSurface(area), "work_area");
});

test("an explicit catalog selection and matching target lock are required", () => {
  const gate = new ZedSurfaceGate();
  gate.setMode("spray");
  gate.setCatalog("catalog-1");
  assert.equal(gate.receiveTarget(lock), "ignore");
  gate.selectPlanes("catalog-1", ["plane-2"]);
  assert.equal(gate.receiveTarget(lock), "ignore");
  assert.equal(gate.receiveSurface(target), "ignore");
  assert.equal(gate.surface, null);
});

test("only the requested Wall Front rectangle can become the accepted work area", () => {
  const gate = selected();
  assert.equal(gate.receiveSurface(area), "ignore");
  gate.beginWorkArea({ sec: 1750000001, nanosec: 123456789 });
  for (const change of [
    { selection_id: "old-selection" }, { plane_generation_id: "zed:old" },
    { target_stamp: { sec: 2, nanosec: 0 } }, { source: "d405" },
    { frame_id: "other-frame" },
  ]) assert.equal(gate.receiveSurface({ ...area, ...change }), "ignore");
  assert.equal(gate.receiveSurface(area), "work_area");
  assert.equal(gate.surface.work_area_id, "wa-1");
});

test("geometry and acceptance must arrive atomically", () => {
  for (const change of [
    { corners: [] }, { front_extent: [[0, 0, 0]] }, { position: [0, NaN, 1] },
    { orientation: [0, 0, 0, 0] }, { view_width: 0 }, { work_area_id: "" },
    { accepted: false }, { state: "invalidated" },
  ]) {
    const gate = selected();
    gate.beginWorkArea({ sec: 1750000001, nanosec: 123456789 });
    assert.equal(gate.receiveSurface({ ...area, ...change }), "invalidated");
    assert.equal(gate.surface, null);
    assert.equal(gate.receiveSurface(area), "ignore");
  }
});

test("D405 and superseded ZED invalidations cannot erase the current work area", () => {
  const gate = accepted();
  for (const change of [
    { source: "d405" }, { plane_generation_id: "zed:old" }, { selection_id: "old-selection" },
  ]) assert.equal(gate.receiveSurface({ ...area, ...change, accepted: false, state: "invalidated" }), "ignore");
  assert.equal(gate.surface.work_area_id, "wa-1");
  assert.equal(gate.receiveSurface({ ...area, accepted: false, state: "invalidated" }), "invalidated");
  assert.equal(gate.surface, null);
});

test("unidentified ZED invalidations fail closed and cannot replay the accepted area", () => {
  const gate = accepted();
  assert.equal(gate.receiveSurface({ source: "zed", mode: "work_area", state: "invalidated", accepted: false }), "invalidated");
  assert.equal(gate.surface, null);
  assert.equal(gate.receiveSurface(area), "ignore");
});

test("mode changes, catalog changes, and plane reselection retire previous locks", () => {
  for (const reset of [
    gate => { gate.setMode("paint"); gate.setMode("spray"); },
    gate => gate.setCatalog("catalog-2"),
    gate => gate.selectPlanes("catalog-1", ["plane-1"]),
    gate => gate.reset(),
  ]) {
    const gate = accepted();
    reset(gate);
    assert.equal(gate.surface, null);
    assert.equal(gate.receiveSurface(area), "ignore");
    gate.setCatalog("catalog-1");
    gate.selectPlanes("catalog-1", ["plane-1"]);
    assert.equal(gate.receiveTarget(lock), "ignore");
  }
});

test("plan matching requires all accepted surface identities, mode, path, and hash", () => {
  const plan = {
    process_mode: "spray", work_area_id: "wa-1", plane_generation_id: lock.plane_generation_id,
    path_id: "path-1", plan_hash: "hash-1",
  };
  assert.equal(matchesPlan(area, plan, { ...plan }, "spray"), true);
  for (const change of [
    { process_mode: "paint" }, { work_area_id: "wa-old" }, { plane_generation_id: "zed:old" },
    { path_id: "path-old" }, { plan_hash: "hash-old" },
  ]) assert.equal(matchesPlan(area, plan, { ...plan, ...change }, "spray"), false);
  assert.equal(matchesPlan(area, { ...plan, plan_hash: "" }, plan, "spray"), false);
  assert.equal(matchesPlan({ ...area, accepted: false }, plan, plan, "spray"), false);
  assert.equal(matchesPlan({ ...area, source: "d405" }, plan, plan, "spray"), false);
});
