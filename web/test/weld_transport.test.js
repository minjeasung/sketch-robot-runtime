const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

test('weld drawing invalidation publishes through bundled ROSLIB without replay', () => {
  const context = vm.createContext({window: {}, console, setTimeout, clearTimeout});
  const js = path.join(__dirname, '../js');
  vm.runInContext(fs.readFileSync(path.join(js, 'roslib.min.js'), 'utf8'), context);
  context.ROSLIB = context.window.ROSLIB;
  context.window = context;
  const ros = new context.ROSLIB.Ros();
  const packets = [];
  ros.isConnected = true;
  ros.socket = {send: text => packets.push(JSON.parse(text))};
  Object.assign(context, {
    ros, rosConnected: true, processMode: 'paint',
    ProcessSettings: {setRunning() {}, isApplied() {return false;}},
    logEvent(message) {throw new Error(message);},
  });
  vm.runInContext(fs.readFileSync(path.join(js, 'weld_workflow.js'), 'utf8'), context);
  ros.emit('/welding/status', {data: JSON.stringify({
    server_session_id: 'session', identity: {target_revision: 'target'},
    state: 'EMPTY', running: false,
  })});
  context.WeldWorkflow.invalidate();
  const commands = packets.filter(p => p.op === 'publish' && p.topic === '/welding/invalidate');
  assert.equal(commands.length, 1);
  assert.equal(JSON.parse(commands[0].msg.data).server_session_id, 'session');
  // ROSLIB must not automatically advertise or resend this command on close.
  context.rosConnected = false;
  ros.emit('close');
  ros.emit('connection');
  assert.equal(packets.filter(p => p.topic === '/welding/invalidate').length, 2);
});
