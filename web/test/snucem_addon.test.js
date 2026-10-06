"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const { loadUI } = require("./ui_harness.js");

test("add-on configuration connects and displays upstream left image before selection", async () => {
  const topic = "/zed/zed_node/left/color/rect/image";
  const ui = loadUI("index.html", undefined, { image_topic: topic, rosbridge_url: "ws://robot:9095", addon: true });
  ui.ros.emit("connection");
  ui.emit("/painting_system/process_mode", { mode: "spray" });
  ui.frame(topic);
  assert.equal(ui.element("camera-empty").hidden, true);
  assert.equal(ui.evaluate("VIEW_TOPICS.zed_raw"), topic);
  await ui.rectangle();
  await ui.element("btn-set-target").fire("click");
  assert.equal(ui.published.at(-1).name, "/target_selection_pixels");
  assert.equal(ui.published.at(-1).message.header.frame_id, "zed_raw");
  assert.equal(ui.ros.initialUrl, "ws://robot:9095");
});
