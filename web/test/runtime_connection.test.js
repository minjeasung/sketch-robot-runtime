'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const {loadUI} = require('./ui_harness.js');

for (const page of ['index.html', 'setup.html']) {
  test(`${page} resolves owned websocket port before any ROS connection`, async () => {
    let finish;
    const ui = loadUI(page, url => {
      if (String(url).endsWith('/runtime-connection')) return new Promise(resolve => {finish = resolve;});
      return Promise.resolve({ok: false, status: 404, json: async () => ({})});
    });
    assert.equal(ui.connections.every(ros => !ros.initialUrl && !ros.connectCalls), true);
    assert.equal(typeof finish, 'function');
    finish({ok: true, json: async () => ({rosbridge_port: 9091, ros_domain_id: '106', simulation: true})});
    await new Promise(setImmediate);
    for (const ros of ui.connections) assert.deepEqual(ros.connectCalls, ['ws://localhost:9091']);
    assert.equal(ui.element('ws-url').textContent, 'ws://localhost:9091');
  });

  test(`${page} standalone hosting retains websocket port9090`, async () => {
    const ui = loadUI(page, async () => ({ok: false, status: 404}));
    await new Promise(setImmediate);
    for (const ros of ui.connections) assert.deepEqual(ros.connectCalls, ['ws://localhost:9090']);
  });
}

for (const page of ['index.html', 'setup.html']) {
  test(`${page} never connects to unrelated port9090 if routing lookup fails`, async () => {
    const ui = loadUI(page, async () => {throw new Error('API unavailable');});
    await new Promise(setImmediate);
    assert.equal(ui.connections.every(ros => !ros.initialUrl && !ros.connectCalls), true);
  });
}
test('new applied runtime assets invalidate every sketch stage, while a pending desired edit does not',async()=>{
 let revision='a';let desired='spray';
 const ui=loadUI('index.html',async url=>({ok:true,json:async()=>String(url).endsWith('/runtime-connection')?{rosbridge_port:9090}:{configuration:{profile:'isaac_construct'},runtime_configuration:{phase:'ready',asset_revision:revision,desired:{process:desired},applied:{process:'spray'}}}}));
 ui.ros.emit('connection');await new Promise(setImmediate);
 ui.evaluate("strokesMap.target=[{type:'line',points:[{u:1,v:1},{u:2,v:2}]}];strokesMap.work_area=[{type:'rect',points:[]}];strokesMap.path=[{type:'line',points:[]}]");
 desired='weld';await ui.evaluate('refreshImageTransport()');assert.equal(ui.evaluate('strokesMap.target.length'),1);
 revision='b';await ui.evaluate('refreshImageTransport()');
 assert.equal(ui.evaluate('strokesMap.target.length+strokesMap.work_area.length+strokesMap.path.length'),0);
 assert.equal(ui.element('btn-run-robot').disabled,true);
});
for(const mode of ['spray','paint'])test(`mixed ${mode} pins the physical target image and mirrors its exact region to the shared scene`,async()=>{
 const ui=loadUI('index.html',async url=>({ok:true,json:async()=>String(url).endsWith('/runtime-connection')?{rosbridge_port:9090}:{configuration:{profile:'isaac_platform',camera_backend:'outpost'},runtime:{simulation:true},runtime_configuration:{phase:'ready',asset_revision:'a',applied:{execution_mode:'mixed',process:mode}}}}));
 ui.ros.emit('connection');await new Promise(setImmediate);ui.emit('/painting_system/process_mode',{mode});
 const meta={generation:'g',width:100,height:80,frame_id:'optical',clock_domain:'ros',stamp:{sec:5,nanosec:7},valid:true,source:'physical',frame_binding:{source_session:'g',source_frame_id:10,capture_stamp:10,sim_stamp:5000000007,receipt_monotonic:1,clock_epoch:1,registration_revision:'r'}};
 ui.emit('/perception/zed_frame_identity',meta);await ui.previewFrame('/zed/zed_node/rgb/color/rect/image',100,80,{stamp:meta.stamp,frame_id:meta.frame_id});
 const rect=ui.queryAll('input[name="sketch-mode"]').find(r=>r.value==='rect');rect.checked=true;await rect.fire('change');await ui.rectangle();
 await ui.element('btn-set-target').fire('click');
 const pin=ui.published.find(p=>p.name==='/perception/selection_pin');assert.ok(pin);assert.deepEqual(JSON.parse(pin.message.data),meta);
 const scene=ui.published.find(p=>p.name==='/task_scene/selection');assert.ok(scene);const body=JSON.parse(scene.message.data);
 assert.deepEqual(body.frame_binding,meta.frame_binding);assert.deepEqual(body.stamp,meta.stamp);assert.deepEqual(body.polygon,[[10,10],[90,10],[90,70],[10,70]]);
 assert.equal(ui.published.some(p=>p.name==='/target_selection_pixels'),true);
 assert.equal(ui.published.some(p=>p.name==='/sketch_execute'),false);
});

test('successful same-asset reapply invalidates sketches only when the committed bundle changes',async()=>{
 let phase='ready',directory='/runtime/assets/revision-instance1',desiredRevision=1;
 const ui=loadUI('index.html',async url=>({ok:true,json:async()=>String(url).endsWith('/runtime-connection')?{rosbridge_port:9090}:{configuration:{profile:'platform'},runtime_configuration:{phase,revision:desiredRevision,asset_revision:'same',bundle_directory:directory,applied:{execution_mode:'simulation',platform:'commercial',active_arm:'left',process:'spray'}}}}));
 ui.ros.emit('connection');await new Promise(setImmediate);
 ui.evaluate("strokesMap.target=[{type:'line',points:[{u:1,v:1},{u:2,v:2}]}];strokesMap.work_area=[{type:'rect',points:[]}];strokesMap.path=[{type:'line',points:[]}]");
 desiredRevision=2;phase='idle';await ui.evaluate('refreshImageTransport()');
 assert.equal(ui.evaluate('strokesMap.target.length'),1,'saved desired revision must not invalidate applied drawings');
 phase='starting';await ui.evaluate('refreshImageTransport()');
 assert.equal(ui.evaluate('strokesMap.path.length'),1,'in-progress apply must not claim a committed bundle');
 phase='ready';directory='/runtime/assets/revision-instance2';await ui.evaluate('refreshImageTransport()');
 assert.equal(ui.evaluate('strokesMap.target.length+strokesMap.work_area.length+strokesMap.path.length'),0);
 assert.equal(ui.element('btn-run-robot').disabled,true);
});

for (const arm of ['left', 'right', 'single']) test(`operator approval reaches applied ${arm} arm, not desired arm`, async () => {
  const ui = loadUI('index.html', async url => ({ok: true, json: async () =>
    String(url).endsWith('/runtime-connection') ? {rosbridge_port: 9090} : {
      configuration: {profile: 'platform'}, runtime_configuration: {
        phase: 'ready', asset_revision: 'a',
        applied: {execution_mode: 'simulation', active_arm: arm, process: 'paint'},
        desired: {active_arm: arm === 'left' ? 'right' : 'left'},
      },
    }}));
  ui.ros.emit('connection');
  await new Promise(setImmediate);
  ui.emit('/painting_system/process_mode', {mode: 'paint'});
  ui.element('free-space-confirmed').checked = true;
  await ui.element('free-space-confirmed').fire('change');
  assert.deepEqual(ui.published.filter(p => p.message.data === true), [{
    name: `/platform/${arm}/painting_admittance/free_space_confirmed`, message: {data: true},
  }]);
  ui.element('free-space-confirmed').checked = false;
  await ui.element('free-space-confirmed').fire('change');
  assert.deepEqual(ui.published.at(-1), {
    name: `/platform/${arm}/painting_admittance/free_space_confirmed`, message: {data: false},
  });
});

test('switching applied arms clears approval on both routes and requires a new operator action', async () => {
  let arm = 'left';
  const ui = loadUI('index.html', async url => ({ok: true, json: async () =>
    String(url).endsWith('/runtime-connection') ? {rosbridge_port: 9090} : {
      runtime_configuration: {applied: {execution_mode: 'mixed', active_arm: arm, process: 'paint'}},
    }}));
  ui.ros.emit('connection'); await new Promise(setImmediate);
  ui.emit('/painting_system/process_mode', {mode: 'paint'});
  ui.element('free-space-confirmed').checked = true;
  await ui.element('free-space-confirmed').fire('change');
  const before = ui.published.length;
  arm = 'right'; await ui.evaluate('refreshImageTransport()');
  assert.equal(ui.element('free-space-confirmed').checked, false);
  assert.deepEqual(ui.published.slice(before), ['left', 'right'].map(side => ({
    name: `/platform/${side}/painting_admittance/free_space_confirmed`, message: {data: false},
  })));
  assert.equal(ui.element('btn-run-robot').disabled, true);
});

test('legacy real controller keeps the global operator approval topic', async () => {
  const ui = loadUI('index.html', async url => ({ok: true, json: async () =>
    String(url).endsWith('/runtime-connection') ? {rosbridge_port: 9090} : {
      runtime_configuration: {applied: {execution_mode: 'real', active_arm: 'single', process: 'paint'}},
    }}));
  ui.ros.emit('connection'); await new Promise(setImmediate);
  ui.emit('/painting_system/process_mode', {mode: 'paint'});
  ui.element('free-space-confirmed').checked = true;
  await ui.element('free-space-confirmed').fire('change');
  assert.deepEqual(ui.published.at(-1), {name: '/painting_admittance/free_space_confirmed', message: {data: true}});
});
