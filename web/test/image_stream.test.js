"use strict";
const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

function streamHarness(closeFirst = false) {
  const images = [], timers = new Map(), rendered = [], raw = [], errors = [];
  let timerId = 0;
  const context = vm.createContext({window: {}, console,
    setTimeout: fn => {timers.set(++timerId, fn); return timerId;},
    clearTimeout: id => timers.delete(id),
    Image: class {
      constructor() {images.push(this);}
      set src(value) {this.url = value;}
    },
  });
  const js = path.join(__dirname, '../js');
  vm.runInContext(fs.readFileSync(path.join(js, 'roslib.min.js'), 'utf8'), context);
  context.ROSLIB = context.window.ROSLIB;
  vm.runInContext(fs.readFileSync(path.join(js, 'image_topic.js'), 'utf8'), context);
  const ros = new context.ROSLIB.Ros(), sent = [];
  ros.isConnected = true;
  ros.socket = {send: packet => sent.push(JSON.parse(packet))};
  let stream;
  if (closeFirst) ros.on('close', () => stream.unsubscribe());
  stream = context.createPreviewStream({ros, name:'/camera/image', reliable:true,
    onRaw: msg => raw.push(msg), onPreview: img => rendered.push(img),
    onError: error => errors.push(error)});
  return {ros, sent, stream, images, rendered, raw, errors,
    fallback() {for (const [id, fn] of [...timers]) {timers.delete(id); fn();}},
    compressed(data) {ros.emit('/camera/image/compressed', {format:'rgb8; jpeg compressed bgr8', data});},
    async decoded(index) {images[index].width=320; images[index].height=240; images[index].onload(); await new Promise(setImmediate);},
  };
}

test('preview subscribes to JPEG first, avoiding multi-megabyte raw frames', async () => {
  const h = streamHarness();
  assert.deepEqual(h.sent.filter(p => p.op==='subscribe').map(p => p.topic), ['/camera/image/compressed']);
  h.compressed('frame');
  await h.decoded(0);
  h.fallback();
  assert.equal(h.rendered.length, 1);
  assert.equal(h.images[0].url, 'data:image/jpeg;base64,frame');
  assert.equal(h.sent.filter(p => p.op==='subscribe').length, 1);
});

test('old servers fall back to raw, then stop raw when compressed becomes usable', async () => {
  const h = streamHarness(); h.fallback();
  h.ros.emit('/camera/image', {width:32});
  assert.equal(h.raw.length, 1);
  h.compressed('jpeg'); await h.decoded(0);
  assert.ok(h.sent.some(p => p.op==='unsubscribe' && p.topic==='/camera/image'));
  h.ros.emit('/camera/image', {width:32});
  assert.equal(h.raw.length, 1);
});

test('decoder drops intermediate frames and cancels frames from an old view', async () => {
  const h = streamHarness();
  h.compressed('one'); h.compressed('two'); h.compressed('three');
  assert.equal(h.images.length, 1);
  await h.decoded(0);
  assert.equal(h.images.length, 2);
  assert.equal(h.images[1].url, 'data:image/jpeg;base64,three');
  h.stream.unsubscribe(); await h.decoded(1); h.fallback();
  assert.equal(h.rendered.length, 1);
  assert.equal(h.sent.filter(p => p.op==='subscribe').length, 1);
  const count = h.sent.length;
  h.ros.emit('close');
  assert.equal(h.sent.length, count);
});

test('a corrupt JPEG does not disable raw fallback', async () => {
  const h = streamHarness(); h.compressed('corrupt');
  h.images[0].onerror(); await new Promise(setImmediate);
  h.fallback(); h.ros.emit('/camera/image', {width:32});
  assert.equal(h.errors.length, 1);
  assert.equal(h.raw.length, 1);
});

test('closing an image connection cannot replay retired view subscriptions', () => {
  // App listener is older than subscriptions created after a view switch.
  const h = streamHarness(true);
  h.fallback();
  const previous = h.sent.filter(p => p.op === 'subscribe').length;
  h.ros.emit('close');
  assert.equal(h.sent.filter(p => p.op === 'subscribe').length, previous);
});
