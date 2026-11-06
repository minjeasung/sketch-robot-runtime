const test=require('node:test'),assert=require('node:assert/strict');
const {selectTarget,selectArea,validate}=require('./workflow_fixtures');
test('target_reply_waits_for_next_click and area_reply_waits_for_next_click',async()=>{
 const ui=await selectTarget();await selectArea(ui);
 assert.equal(ui.evaluate('workflowMode'),'path');
});
test('stage_round_trip_preserves_plan without publishing commands',async()=>{
 const ui=await selectTarget();await selectArea(ui);validate(ui);
 const before=ui.published.length;
 for(const stage of ['target','work_area','path']) {
  const radio=ui.queryAll('input[name="workflow-mode"]').find(r=>r.value===stage);
  radio.checked=true;await radio.fire('change');
 }
 assert.equal(ui.published.length,before);assert.equal(ui.element('btn-run-robot').disabled,false);
});
test('plan_button_dispatches_selected_path_once and pending request locks input',async()=>{
 const ui=await selectTarget();await selectArea(ui);ui.frame('/perception/wall_front_view');
 const before=ui.published.length;
 await ui.element('btn-plan-workflow').fire('click');
 await ui.element('btn-plan-workflow').fire('click');
 assert.deepEqual(ui.published.slice(before).map(p=>p.name),['/fill_work_area']);
 assert.equal(ui.element('path-method').disabled,true);
 const count=ui.evaluate('currentStrokes().length');await ui.rectangle();
 assert.equal(ui.evaluate('currentStrokes().length'),count);
});
test('unfinished_stroke_blocks_manual_plan',async()=>{
 const ui=await selectTarget();await selectArea(ui);ui.frame('/perception/wall_front_view');
 ui.element('path-method').value='manual';await ui.element('path-method').fire('change');
 const line=ui.queryAll('input[name="sketch-mode"]').find(r=>r.value==='line');line.checked=true;await line.fire('change');
 await ui.element('sketch-canvas').fire('pointerdown',{pointerId:1,clientX:20,clientY:20});
 const before=ui.published.length;await ui.element('btn-plan-workflow').fire('click');
 assert.equal(ui.published.length,before);assert.equal(ui.element('btn-plan-workflow').disabled,true);
});
const {loadUI}=require('./ui_harness');
const weldStatus=(extra={})=>({server_session_id:'s',state:'SEAMS_READY',ready:false,running:false,runtime_ready:true,
 identity:{server_session_id:'s',target_revision:'t',generation:'g',selection_revision:0,settings_revision:0},
 seams:[{seam_id:'S1',length_m:.1,pose_ready:true}],selection:[],...extra});
function weldUI(extra={}) {const ui=loadUI();ui.ros.emit('connection');ui.emit('/painting_system/process_mode',{mode:'weld'});ui.emit('/welding/status',weldStatus(extra));return ui;}
function weldSettings(ui,revision=1,extra={}) {ui.emit('/process_settings/status',{mode:'weld',server_session_id:'s',revision,applied:revision>0,fields:{speed_mps:{label:'속도',value:.01,editable:true}},...extra});}
test('weld_uses_three_distinct_panels and stage two disables canvas mutations',async()=>{
 const ui=weldUI();assert.equal(ui.element('target-actions').hidden,false);
 assert.equal(ui.element('path-actions').hidden,true);
 await ui.element('btn-workflow-next').fire('click');
 assert.equal(ui.evaluate('workflowMode'),'work_area');assert.equal(ui.element('weld-seam-actions').hidden,false);
 assert.equal(ui.element('target-actions').hidden,true);assert.equal(ui.element('drawing-toolbar').hidden,true);
 const before=ui.published.length;await ui.rectangle();await ui.element('btn-clear').fire('click');
 assert.equal(ui.published.length,before);assert.equal(ui.evaluate('currentStrokes().length'),0);
});
test('selection_ack_gates_next_step and xyz seam can enter settings but needs a pose to plan',async()=>{
 const ui=weldUI({seams:[{seam_id:'S1',length_m:.1,pose_ready:false}]});
 await ui.element('btn-workflow-next').fire('click');
 ui.evaluate("WeldWorkflow.select([{seam_id:'S1',reverse:false}])");
 const command=JSON.parse(ui.published.at(-1).message.data);
 const selected=weldStatus({state:'SELECTED',selection:command.selection,seams:[{seam_id:'S1',length_m:.1,pose_ready:false}],
 identity:{...weldStatus().identity,selection_revision:1}});
 ui.emit('/welding/status',selected);
 assert.equal(ui.element('btn-workflow-next').disabled,true);
 ui.emit('/welding/status',{...selected,command_ack:{command_id:command.command_id,ok:true}});
 await ui.element('btn-workflow-next').fire('click');
 assert.equal(ui.evaluate('workflowMode'),'path');weldSettings(ui);
 assert.equal(ui.element('btn-plan-workflow').disabled,true);
 assert.match(ui.element('workflow-stage-reason').textContent,/자세/);
});
test('settings_ack_and_revision_both_required before one plan command',async()=>{
 const ui=weldUI({state:'SELECTED',selection:[{seam_id:'S1',reverse:false}]});
 await ui.element('btn-workflow-next').fire('click');await ui.element('btn-workflow-next').fire('click');
 weldSettings(ui,0);ui.evaluate("ProcessSettings.edit('speed_mps',.02); ProcessSettings.apply()");
 const command=JSON.parse(ui.published.at(-1).message.data);
 weldSettings(ui,1,{fields:{speed_mps:{label:'속도',value:.02,editable:true}}});
 assert.equal(ui.element('btn-plan-workflow').disabled,true);
 ui.emit('/welding/status',weldStatus({state:'SELECTED',selection:[{seam_id:'S1',reverse:false}],
 identity:{...weldStatus().identity,settings_revision:1},command_ack:{command_id:command.command_id,ok:true}}));
 await ui.element('btn-plan-workflow').fire('click');await ui.element('btn-plan-workflow').fire('click');
 assert.equal(ui.published.filter(p=>p.name==='/welding/plan_request').length,1);
});
test('stale_catalog_cannot_unlock_stages after target editing',()=>{
 const ui=weldUI();ui.evaluate('WeldWorkflow.invalidate()');
 ui.emit('/welding/status',weldStatus({ready:true,state:'READY',plan_id:'old'}));
 assert.equal(ui.element('btn-workflow-next').disabled,true);assert.equal(ui.element('btn-run-robot').disabled,true);
});
test('edited_method_or_geometry_blocks_old_ready',async()=>{
 const ui=await selectTarget();await selectArea(ui);const plan=validate(ui);
 ui.element('path-method').value='manual';await ui.element('path-method').fire('change');
 ui.emit('/painting_system/plan_status',plan);ui.emit('/painting_system/readiness',{...plan,ready:true,plan_validated:true});
 assert.equal(ui.element('btn-run-robot').disabled,true);
 assert.notEqual(ui.element('painting-plan-validation').textContent,'완료');
});
test('old_mode_status_cannot_replace_current_authoritative_state',async()=>{
 const ui=await selectTarget();await selectArea(ui);const plan=validate(ui);
 ui.emit('/painting_system/readiness',{...plan,process_mode:'paint',ready:true,running:true});
 assert.equal(ui.evaluate('paintingState.readiness.process_mode'),'spray');
 assert.equal(ui.evaluate('paintingDerivedState().running'),false);
});
test('stop_ack_delay_blocks_new_work and reconnect never replays a request',async()=>{
 const ui=await selectTarget();await selectArea(ui);validate(ui);
 await ui.element('btn-stop-robot').fire('click');const before=ui.published.length;
 await ui.element('btn-plan-workflow').fire('click');await ui.element('btn-run-robot').fire('click');
 assert.equal(ui.published.length,before);assert.equal(ui.element('btn-stop-robot').disabled,false);
 ui.ros.emit('close');ui.ros.emit('connection');
 assert.equal(ui.published.slice(before).some(p=>['/fill_work_area','/sketch_execute'].includes(p.name)),false);
});
test('mode_change_clears_old_panel_state and late weld readiness is not reusable',async()=>{
 const ui=weldUI({state:'SELECTED',selection:[{seam_id:'S1',reverse:false}]});
 await ui.element('btn-workflow-next').fire('click');await ui.element('btn-workflow-next').fire('click');
 ui.emit('/painting_system/process_mode',{mode:'spray'});
 assert.equal(ui.element('weld-seam-actions').hidden,true);assert.equal(ui.element('weld-selection-summary').hidden,true);
 ui.emit('/welding/status',weldStatus({state:'READY',ready:true,plan_id:'old'}));
 assert.equal(ui.element('btn-run-robot').disabled,true);
 ui.emit('/painting_system/process_mode',{mode:'weld'});
 assert.equal(ui.element('btn-workflow-next').disabled,true);
});
test('paint tare approval is not displayed as measured tare completion',()=>{
 const ui=loadUI();ui.ros.emit('connection');ui.emit('/painting_system/process_mode',{mode:'paint'});
 assert.match(ui.element('workflow-ready-detail').textContent,/実行|실행 중 확인/);
 assert.doesNotMatch(ui.element('workflow-ready-detail').textContent,/영점.*완료/);
});
test('preview and motion-test keep execution and output limits',async()=>{
 const ui=await selectTarget(undefined,true);await selectArea(ui);validate(ui);
 assert.equal(ui.element('btn-run-robot').disabled,true);
 assert.match(ui.element('workflow-validation-scope').textContent,/미수행/);
 ui.emit('/painting_system/process_mode',{mode:'spray',spray_motion_test:true});
 assert.match(ui.element('process-mode-state').textContent,/분사 항상 OFF/);
});
test('weld STOPPED acknowledgement releases pending command without restoring old plan',async()=>{
 const ui=weldUI({state:'SELECTED',selection:[{seam_id:'S1',reverse:false}]});
 await ui.element('btn-workflow-next').fire('click');await ui.element('btn-workflow-next').fire('click');
 weldSettings(ui,0);ui.evaluate('ProcessSettings.apply()');
 await ui.element('btn-stop-robot').fire('click');
 ui.emit('/welding/status',weldStatus({state:'STOPPING',running:true}));
 assert.equal(ui.element('settings-apply').disabled,true);
 ui.emit('/welding/status',weldStatus({state:'STOPPED',error:'stopped'}));
 assert.equal(ui.element('settings-apply').disabled,false);
 assert.equal(ui.element('btn-run-robot').disabled,true);
});
test('weld plan failures allow a retry without settings panel remaining locked',async()=>{
 const ui=weldUI({state:'SELECTED',selection:[{seam_id:'S1',reverse:false}],identity:{...weldStatus().identity,settings_revision:1}});
 await ui.element('btn-workflow-next').fire('click');await ui.element('btn-workflow-next').fire('click');weldSettings(ui);
 await ui.element('btn-plan-workflow').fire('click');const cmd=JSON.parse(ui.published.at(-1).message.data);
 ui.emit('/welding/status',weldStatus({state:'SELECTED',error:'unreachable',selection:[{seam_id:'S1',reverse:false}],
 identity:{...weldStatus().identity,settings_revision:1},command_ack:{command_id:cmd.command_id,ok:false}}));
 assert.equal(ui.element('btn-plan-workflow').disabled,false);
 assert.match(ui.element('execution-summary').textContent,/unreachable/);
});
test('safe abort reset unlocks editing but never restores the stopped plan',async()=>{
 const ui=await selectTarget();await selectArea(ui);const plan=validate(ui);
 ui.emit('/painting_system/execution_status',{state:'IDLE',execution_sequence:10});
 await ui.element('btn-stop-robot').fire('click');
 ui.emit('/painting_system/execution_status',{state:'IDLE',reason:'manual execution abort reset',execution_sequence:9});
 assert.equal(ui.evaluate('workflowView().canEdit'),false);
 ui.emit('/painting_system/execution_status',{state:'ABORT',reason:'motion_abort',execution_sequence:11});
 ui.emit('/painting_system/readiness',{...plan,ready:false,running:false,abort_reason:'motion_abort'});
 assert.equal(ui.evaluate('workflowView().canEdit'),false);
 ui.emit('/painting_system/execution_status',{state:'IDLE',reason:'manual execution abort reset',execution_sequence:12});
 assert.equal(ui.evaluate('workflowView().canEdit'),false);
 ui.emit('/painting_system/readiness',{...plan,ready:true,running:false,abort_reason:'NONE'});
 assert.equal(ui.evaluate('workflowView().canEdit'),true);
 ui.emit('/painting_system/plan_status',plan);
 assert.equal(ui.element('btn-run-robot').disabled,true);
});
test('unfinished freehand is cancelled by tab or method changes',async()=>{
 for(const change of ['tab','method']) {
  const ui=await selectTarget();await selectArea(ui);ui.frame('/perception/wall_front_view');
  ui.evaluate("setPathMethod('manual'); sketchMode='freehand'");
  await ui.element('sketch-canvas').fire('pointerdown',{pointerId:1,clientX:20,clientY:20});
  await ui.element('sketch-canvas').fire('pointermove',{pointerId:1,clientX:70,clientY:40});
  assert.equal(ui.element('btn-plan-workflow').disabled,true);
  if(change==='tab')ui.evaluate("navigateWorkflow('target');navigateWorkflow('path')");
  else ui.evaluate("setPathMethod('auto');setPathMethod('manual')");
  ui.frame('/perception/wall_front_view');
  const before=ui.published.length;await ui.element('btn-plan-workflow').fire('click');
  assert.equal(ui.published.length,before);assert.equal(ui.evaluate('currentStrokes().length'),0);
 }
});
test('late production-shaped execution status cannot lock the current process',async()=>{
 const ui=await selectTarget();await selectArea(ui);const plan=validate(ui);
 ui.emit('/painting_system/execution_status',{state:'PAINT',work_area_id:'old-paint-area',plane_generation_id:'old-paint-generation',plan_hash:'old-plan',execution_sequence:40});
 assert.equal(ui.evaluate('paintingDerivedState().running'),false);
 assert.equal(ui.evaluate('workflowView().canEdit'),true);
 ui.emit('/painting_system/execution_status',{state:'PAINT',work_area_id:plan.work_area_id,plane_generation_id:plan.plane_generation_id,plan_hash:plan.plan_hash,execution_sequence:41});
 assert.equal(ui.evaluate('paintingDerivedState().running'),true);
 ui.emit('/painting_system/process_mode',{mode:'paint'});
 assert.equal(ui.evaluate('paintingDerivedState().running'),false);
});
