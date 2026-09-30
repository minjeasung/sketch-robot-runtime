"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");
const { EventEmitter } = require("node:events");

test("front image subscription carries reliable QoS through reconnect and unsubscribe", () => {
  const context = vm.createContext({window: {}, console, setTimeout, clearTimeout});
  const js = path.join(__dirname, "../js");
  vm.runInContext(fs.readFileSync(path.join(js, "roslib.min.js"), "utf8"), context);
  context.ROSLIB = context.window.ROSLIB;
  vm.runInContext(fs.readFileSync(path.join(js, "image_topic.js"), "utf8"), context);
  const ros = new EventEmitter();
  ros.idCounter = 0;
  const sent = [];
  ros.callOnConnection = packet => sent.push(JSON.parse(JSON.stringify(packet)));
  const topic = context.createImageTopic({ros, name: "/perception/wall_front_view"});
  topic.subscribe(() => {});
  assert.equal(sent[0].qos.reliability, "reliable");
  assert.equal(sent[0].qos.depth, 1);
  assert.equal(sent[0].queue_length, 1);
  ros.emit("close");
  assert.deepEqual(sent[1], sent[0]);
  ros.emit("connection");
  topic.unsubscribe();
  assert.equal(sent.at(-1).op, "unsubscribe");
  const count = sent.length;
  ros.emit("close");
  assert.equal(sent.length, count);
});

test("native camera topics keep their publisher-compatible default QoS", () => {
  const {createImageTopic} = require("../js/image_topic.js");
  let packet;
  const topic = createImageTopic({name: "/zed/raw"}, {Topic: class {
    callForSubscribeAndAdvertise(message) {packet = message;}
  }});
  topic.callForSubscribeAndAdvertise({op: "subscribe"});
  assert.equal(packet.qos, undefined);
});

test("Outpost raw images explicitly request reliable transport", () => {
  const {createImageTopic} = require("../js/image_topic.js");
  let packet;
  const topic = createImageTopic({name: "/zed/raw", reliable: true}, {Topic: class {
    callForSubscribeAndAdvertise(message) {packet = message;}
  }});
  topic.callForSubscribeAndAdvertise({op: "subscribe"});
  assert.equal(packet.qos?.reliability, "reliable");
});

for (const [backend, profile, expected] of [
  ["outpost", "zed_preview", true], ["outpost", "real", true],
  ["native", "real", false], ["outpost", "fake", false],
]) test(`image QoS follows supervisor ${backend}/${profile}`, async () => {
  const {loadUI} = require("./ui_harness.js");
  const ui = loadUI("index.html", async () => ({ok: true, json: async () => ({
    configuration: {camera_backend: backend, profile},
  })}));
  await ui.evaluate("refreshImageTransport()");
  assert.equal(ui.evaluate("zedImageReliable"), expected);
});
