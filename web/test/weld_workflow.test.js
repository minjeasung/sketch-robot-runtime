const test=require('node:test'),assert=require('node:assert/strict');
const {loadUI}=require('./ui_harness');
function status(overrides={}) {return {server_session_id:'s',state:'SEAMS_READY',ready:false,running:false,
 identity:{server_session_id:'s',target_revision:'t',generation:'g',selection_revision:0,settings_revision:0},
 seams:[{seam_id:'S1',length_m:.1,pose_ready:true},{seam_id:'S2',length_m:.1,pose_ready:true},{seam_id:'S3',length_m:.1,pose_ready:true}],selection:[],...overrides};}
test('weld unlocks drawing tools after connection and enables target submission after drawing',async()=>{
 const ui=loadUI();ui.ros.emit('connection');
 const rect=ui.queryAll('input[name="sketch-mode"]').find(r=>r.value==='rect');
 assert.equal(rect.disabled,true);
 ui.emit('/painting_system/process_mode',{mode:'weld'});
 ui.emit('/welding/status',status({state:'EMPTY',seams:[]}));
 assert.equal(rect.disabled,false);
 rect.checked=true;await rect.fire('change');
 const meta={generation:'1',width:100,height:80,frame_id:'optical',clock_domain:'ros',stamp:{sec:5,nanosec:7}};
 ui.emit('/perception/zed_frame_identity',meta);
 await ui.previewFrame('/zed/zed_node/rgb/color/rect/image',100,80,{stamp:meta.stamp,frame_id:meta.frame_id});
 await ui.rectangle();
 const invalidate=JSON.parse(ui.published.find(p=>p.name==='/welding/invalidate').message.data);
 ui.emit('/welding/status',status({state:'EMPTY',seams:[],command_ack:{command_id:invalidate.command_id,ok:true}}));
 assert.equal(ui.evaluate('currentStrokes().length'),1);
 assert.equal(ui.element('btn-undo').disabled,false);
 assert.equal(ui.element('btn-clear').disabled,false);
 assert.equal(ui.element('btn-set-target').disabled,false);
 await ui.element('btn-set-target').fire('click');
 assert.equal(JSON.parse(ui.published.at(-1).message.data).polygon.length,4);
 assert.equal(ui.published.at(-1).name,'/welding/target_request');
 // Target replies do not have a command ACK; they complete via state/identity.
 ui.emit('/welding/status',status());
 assert.equal(ui.element('plane-candidates').children.length,3);
 assert.doesNotMatch(ui.element('events').textContent,/용접 상태 오류/);
 ui.emit('/welding/status',status({running:true,state:'RUNNING'}));
 assert.equal(rect.disabled,true);assert.equal(ui.element('btn-clear').disabled,true);
 ui.emit('/welding/status',status({state:'STOPPED'}));
 assert.equal(rect.disabled,false);
 await ui.element('btn-clear').fire('click');
 assert.equal(ui.evaluate('currentStrokes().length'),0);
 assert.equal(ui.element('btn-clear').disabled,true);
 ui.ros.emit('close');assert.equal(rect.disabled,true);
});
test('weld preserves ZED view and only submits selected seams with order and reverse',()=>{
 const ui=loadUI();ui.ros.emit('connection');ui.emit('/painting_system/process_mode',{mode:'weld'});ui.emit('/welding/status',status());
 assert.equal(ui.evaluate('processMode'),'weld');assert.equal(ui.evaluate('currentView'),'zed_raw');
 ui.evaluate("WeldWorkflow.select([{seam_id:'S3',reverse:true},{seam_id:'S1',reverse:false}])");
 const p=ui.published.at(-1);assert.equal(p.name,'/welding/selection');
 assert.deepEqual(JSON.parse(p.message.data).selection,[{seam_id:'S3',reverse:true},{seam_id:'S1',reverse:false}]);
 assert.equal(ui.published.some(p=>p.name==='/sketch_execute'),false);
});
test('disconnect clears welding readiness without replaying commands',()=>{
 const ui=loadUI();ui.ros.emit('connection');ui.emit('/painting_system/process_mode',{mode:'weld'});ui.emit('/welding/status',status({ready:true,state:'READY',plan_id:'p'}));
 ui.ros.emit('close');assert.equal(ui.element('btn-run-robot').disabled,true);
 assert.equal(ui.published.some(p=>p.name==='/welding/execute'),false);
});
test('ZED target carries displayed compressed image stamp and blocks run until settings ack',async()=>{
 const ui=loadUI();ui.ros.emit('connection');ui.emit('/painting_system/process_mode',{mode:'weld'});ui.emit('/welding/status',status());
 const meta={generation:'g',width:100,height:80,frame_id:'optical',clock_domain:'ros',stamp:{sec:5,nanosec:7}};
 ui.emit('/perception/zed_frame_identity',meta);
 // Exercise the production preview callback rather than writing workflow state directly.
 await ui.previewFrame('/zed/zed_node/rgb/color/rect/image',100,80,{stamp:meta.stamp,frame_id:'optical'});
 ui.evaluate("sketchMode='rect'");await ui.rectangle();await ui.element('btn-set-target').fire('click');
 const p=ui.published.find(p=>p.name==='/welding/target_request');assert.ok(p);
 const data=JSON.parse(p.message.data);assert.deepEqual(data.stamp,meta.stamp);assert.equal(data.polygon.length,4);
 assert.equal(ui.element('btn-run-robot').disabled,true);
});
test('rejected settings can be corrected and applied again',()=>{
 const ui=loadUI();ui.ros.emit('connection');ui.emit('/painting_system/process_mode',{mode:'weld'});ui.emit('/welding/status',status({state:'SELECTED',selection:[{seam_id:'S1',reverse:false}]}));
 ui.emit('/process_settings/status',{mode:'weld',server_session_id:'s',revision:0,applied:false,fields:{speed_mps:{label:'속도',value:.01,editable:true}}});
 ui.evaluate("ProcessSettings.edit('speed_mps', 1); ProcessSettings.apply()");
 const command=JSON.parse(ui.published.at(-1).message.data);
 ui.emit('/welding/status',status({state:'SELECTED',selection:[{seam_id:'S1',reverse:false}],error:'out of range',command_ack:{command_id:command.command_id,ok:false,error:'out of range'}}));
 assert.equal(ui.element('settings-apply').disabled,false);
 ui.evaluate("ProcessSettings.edit('speed_mps', .02); ProcessSettings.apply()");
 assert.equal(JSON.parse(ui.published.at(-1).message.data).values.speed_mps,.02);
});
test('returning to spray restores the original plane controls',()=>{
 const ui=loadUI();ui.ros.emit('connection');ui.emit('/painting_system/process_mode',{mode:'weld'});ui.emit('/welding/status',status());
 assert.equal(ui.element('btn-refine-planes').hidden,true);
 ui.emit('/painting_system/process_mode',{mode:'spray'});
 assert.equal(ui.element('btn-refine-planes').hidden,false);
 assert.equal(ui.element('btn-set-target').textContent,'평면 추출');
});
test('interleaved scene status cannot lose a pending Settings rejection',()=>{
 const ui=loadUI();ui.ros.emit('connection');ui.emit('/painting_system/process_mode',{mode:'weld'});ui.emit('/welding/status',status());
 ui.emit('/process_settings/status',{mode:'weld',server_session_id:'s',revision:0,applied:false,fields:{speed_mps:{label:'속도',value:.01,editable:true}}});
 ui.evaluate("ProcessSettings.edit('speed_mps', .02); ProcessSettings.apply()");
 const command=JSON.parse(ui.published.at(-1).message.data);
 ui.emit('/welding/status',status({identity:{...status().identity,scene_revision:'changed'},error:'scene changed'}));
 ui.emit('/welding/status',status({command_ack:{command_id:command.command_id,ok:false,error:'stale revision'}}));
 assert.equal(ui.element('settings-apply').disabled,false);
 assert.match(ui.element('settings-state').textContent,/변경됨/);
 ui.evaluate('ProcessSettings.apply()');
 assert.notEqual(JSON.parse(ui.published.at(-1).message.data).command_id,command.command_id);
});
