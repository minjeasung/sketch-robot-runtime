"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const { loadUI } = require("./ui_harness.js");

async function requestTarget(mode = "spray") {
  const ui = loadUI();
  ui.ros.emit("connection");
  ui.emit("/painting_system/process_mode", { mode });
  ui.frame("/zed/zed_node/rgb/color/rect/image");
  await ui.rectangle();
  await ui.element("btn-set-target").fire("click");
  return ui;
}

function resultForLastRequest(ui) {
  const { stamp } = ui.published.filter(p => p.name === "/target_selection_pixels").at(-1).message.header;
  return {
    generation: String(BigInt(stamp.sec) * 1000000000n + BigInt(stamp.nanosec)),
    image_width: 100, image_height: 80,
    planes: [{ id: "plane-1", polygon_px: [[10, 10], [90, 10], [90, 70], [10, 70]] }],
  };
}

function assertCleared(ui) {
  assert.equal(ui.element("plane-candidates").children.length, 0);
  assert.equal(ui.evaluate("planeCatalog.planes.length"), 0);
  assert.equal(ui.evaluate("selectedPlaneIds.size"), 0);
  assert.equal(ui.element("btn-refine-planes").disabled, true);
  assert.equal(ui.element("active-plane-field").hidden, true);
  assert.equal(ui.evaluate("zedSelection.catalog"), "");
}

for (const mode of ["spray", "paint"]) {
  for (const button of ["btn-clear", "btn-undo"]) {
    test(`${mode} ${button} removes the target's plane results and rejects replay`, async () => {
      const ui = await requestTarget(mode);
      const catalog = resultForLastRequest(ui);
      ui.emit("/perception/target_planes", catalog);
      assert.equal(ui.element("plane-candidates").children.length, 1);
      const checkbox = ui.element("plane-candidates").children[0].children[0];
      checkbox.checked = true;
      await checkbox.fire("change");
      ui.emit("/painting_system/planes", { generation: catalog.generation, state: "ready", measured: ["plane-1"], active_id: "plane-1" });
      if (mode === "paint") ui.evaluate('switchWorkflow("target")');
      assert.equal(ui.element("active-plane-field").hidden, false);
      await ui.element(button).fire("click");
      assertCleared(ui);
      ui.emit("/perception/target_planes", catalog);
      ui.emit("/painting_system/planes", { generation: catalog.generation, state: "ready", measured: ["plane-1"], active_id: "plane-1" });
      assertCleared(ui);
      assert.equal(ui.element("btn-clear").disabled, true);
      assert.equal(ui.element("btn-undo").disabled, true);
    });
  }
}

for (const button of ["btn-clear", "btn-undo"]) {
  test(`${button} cancels in-flight extraction, including after another request starts`, async () => {
    const ui = await requestTarget();
    const old = resultForLastRequest(ui);
    await ui.element(button).fire("click");
    ui.emit("/perception/target_planes", old);
    assertCleared(ui);
    ui.evaluate("Date.now = () => 1800000000000");
    await ui.rectangle();
    await ui.element("btn-set-target").fire("click");
    ui.emit("/perception/target_planes", old);
    assertCleared(ui);
    const fresh = resultForLastRequest(ui);
    ui.emit("/perception/target_planes", fresh);
    assert.equal(ui.element("plane-candidates").children.length, 1);
    assert.equal(ui.evaluate("planeCatalog.generation"), fresh.generation);
  });
}

test("undo invalidates extraction even when an earlier target stroke remains", async () => {
  const ui = await requestTarget();
  await ui.rectangle();
  await ui.element("btn-set-target").fire("click");
  ui.emit("/perception/target_planes", resultForLastRequest(ui));
  await ui.element("btn-undo").fire("click");
  assert.equal(ui.evaluate("strokesMap.target.length"), 1);
  assertCleared(ui);
});

test("editing the target retires its previous extraction", async () => {
  const ui = await requestTarget();
  const old = resultForLastRequest(ui);
  ui.emit("/perception/target_planes", old);
  await ui.rectangle();
  assertCleared(ui);
  ui.emit("/perception/target_planes", old);
  assertCleared(ui);
});

test("a disconnected target does not redisplay a latched catalog on reconnect", async () => {
  const ui = await requestTarget();
  const old = resultForLastRequest(ui);
  ui.emit("/perception/target_planes", old);
  ui.ros.emit("close");
  ui.ros.emit("connection");
  ui.emit("/perception/target_planes", old);
  assertCleared(ui);
});

test("clear and undo cannot remove planes during robot execution", async () => {
  const ui = await requestTarget();
  ui.emit("/perception/target_planes", resultForLastRequest(ui));
  ui.emit("/painting_system/readiness", { process_mode: "spray", running: true });
  for (const button of ["btn-clear", "btn-undo"]) {
    assert.equal(ui.element(button).disabled, true);
    await ui.element(button).fire("click");
    assert.equal(ui.element("plane-candidates").children.length, 1);
  }
});

for (const button of ["btn-clear", "btn-undo"]) {
  test(`${button} remains usable when only an extraction result is present`, async () => {
    const ui = await requestTarget();
    ui.emit("/perception/target_planes", resultForLastRequest(ui));
    ui.evaluate("strokesMap.target = []; refreshPaintingUI()");
    assert.equal(ui.element(button).disabled, false);
    await ui.element(button).fire("click");
    assertCleared(ui);
  });
}

test("shared plane UI displays depth consistency diagnostics when a version supplies them", async () => {
  const ui = await requestTarget();
  const catalog = resultForLastRequest(ui);
  catalog.planes[0].depth_consistency_warning = true;
  ui.emit("/perception/target_planes", catalog);
  const label = ui.element("plane-candidates").children[0];
  assert.ok(label.title, "the flagged candidate must explain its depth warning");
});
