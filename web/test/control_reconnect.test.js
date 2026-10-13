"use strict";
const test = require('node:test');
const assert = require('node:assert/strict');
const {loadUI} = require('./ui_harness.js');

test('control socket retries startup failure and unlocks sketch only after mode acknowledgement', async () => {
  const ui = loadUI();
  ui.ros.emit('error');
  ui.ros.emit('close');
  ui.runTimers(1500);
  assert.equal(ui.ros.connectCalls?.length, 1);
  ui.ros.emit('connection');
  await ui.previewFrame('/zed/zed_node/rgb/color/rect/image');
  await ui.rectangle();
  assert.equal(ui.element('btn-set-target').disabled, true);
  ui.emit('/painting_system/process_mode', {mode:'spray', spray_motion_test:true});
  assert.equal(ui.element('process-mode').value, 'spray');
  await ui.rectangle();
  assert.equal(ui.element('btn-set-target').disabled, false);
  assert.equal(ui.published.filter(p => p.name === '/sketch_execute').length, 0);
});

test('duplicate control failures schedule one retry and successful connection cancels it', () => {
  const ui = loadUI();
  ui.ros.emit('error');
  ui.ros.emit('close');
  ui.ros.emit('close');
  ui.runTimers(1500);
  assert.equal(ui.ros.connectCalls?.length, 1);
  ui.ros.emit('close');
  ui.ros.emit('connection');
  ui.runTimers(1500);
  assert.equal(ui.ros.connectCalls?.length, 1);
});

test('control reconnect invalidates old mode and plan until new backend acknowledgement', () => {
  const ui = loadUI();
  ui.ros.emit('connection');
  ui.emit('/painting_system/process_mode', {mode:'paint'});
  ui.ros.emit('close');
  ui.runTimers(1500);
  assert.equal(ui.ros.connectCalls?.length, 1);
  ui.ros.emit('connection');
  assert.equal(ui.evaluate('processModePending'), true);
  assert.equal(ui.element('btn-run-robot').disabled, true);
  ui.emit('/painting_system/process_mode', {mode:'spray'});
  assert.equal(ui.element('process-mode').value, 'spray');
  assert.equal(ui.element('btn-run-robot').disabled, true);
});
