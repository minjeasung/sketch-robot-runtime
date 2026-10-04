"use strict";
const test = require('node:test');
const assert = require('node:assert/strict');
const {loadUI} = require('./ui_harness.js');

test('image traffic uses a separate connection from selection commands and results', () => {
  const ui = loadUI();
  ui.frame('/zed/zed_node/rgb/color/rect/image');
  const image = ui.topics.find(t => t.name === '/zed/zed_node/rgb/color/rect/image');
  const command = ui.topics.find(t => t.name === '/target_selection_pixels');
  const result = ui.topics.find(t => t.name === '/painting_system/plan_status');
  assert.notEqual(image.ros, command.ros);
  assert.equal(result.ros, command.ros);
  assert.equal(ui.connections.length, 2);
});

test('JPEG display preserves the sketch pixel grid when switching views', async () => {
  const ui = loadUI();
  await ui.previewFrame('/zed/zed_node/rgb/color/rect/image', 1280, 720);
  assert.equal(ui.element('zed-canvas').width, 1280);
  assert.equal(ui.element('sketch-canvas').height, 720);
  assert.equal(ui.element('camera-empty').hidden, true);
  ui.evaluate('switchView("wall_front")');
  assert.equal(ui.element('camera-empty').hidden, false);
  await ui.previewFrame('/perception/wall_front_view', 640, 360);
  assert.equal(ui.element('zed-canvas').width, 640);
  assert.equal(ui.element('sketch-canvas').height, 360);
  assert.equal(ui.evaluate('zedFrameCount'), 1);
});

test('image reconnect clears the old view and resubscribes without changing control connection', async () => {
  const ui = loadUI();
  const imageRos = ui.connections[1];
  assert.ok(imageRos);
  await ui.previewFrame('/zed/zed_node/rgb/color/rect/image');
  imageRos.emit('close');
  assert.equal(ui.element('camera-empty').hidden, false);
  assert.equal(ui.evaluate('zedFrameCount'), 0);
  ui.runTimers(1500);
  assert.equal(imageRos.connectCalls?.length, 1);
  imageRos.emit('connection');
  await ui.previewFrame('/zed/zed_node/rgb/color/rect/image');
  assert.equal(ui.evaluate('zedFrameCount'), 1);
});

test('image loss blocks target submission until a fresh image arrives', async () => {
  const ui = loadUI();
  ui.ros.emit('connection');
  ui.emit('/painting_system/process_mode', {mode:'spray', planning_only:true});
  await ui.previewFrame('/zed/zed_node/rgb/color/rect/image');
  await ui.rectangle();
  assert.equal(ui.element('btn-set-target').disabled, false);
  ui.connections[1].emit('close');
  assert.equal(ui.element('btn-set-target').disabled, true);
  await ui.element('btn-set-target').fire('click');
  assert.equal(ui.published.filter(p => p.name === '/target_selection_pixels').length, 0);
});
