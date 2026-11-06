'use strict';
const test=require('node:test'),assert=require('node:assert/strict');
const {loadUI}=require('./ui_harness');
const selection={schema_version:2,execution_mode:'simulation',platform:'commercial',active_arm:'left',process:'spray',tools:{left:'spray_v2',right:'none'}};
async function setup(overrides={}) {
 const requests=[];
 const state={revision:3,desired:structuredClone(selection),applied:structuredClone(selection),connections:{},applied_connections:{},phase:'ready',asset_revision:'a',error:null,capabilities:[],...overrides};
 const legacy={configuration:{profile:'isaac_construct',model_id:'construct_left',process_mode:'spray',robot_ip:'10.0.2.7',spray_eoat_profile:''},processes:[{name:'simulation',pid:123,state:'RUNNING'}],ros:{},ros_domain_id:0};
 const updateCaps=()=>state.capabilities=[{...state.desired,supported:state.desired.execution_mode!=='real',reason:state.desired.execution_mode==='real'?'실환경 드라이버 미연결':'',verification:'not_run'}]; updateCaps();
 let fail=null,finish=null;
 const ui=loadUI('system.html',async(url,options={})=>{
  requests.push({url,...options});
  if(url==='/status')return {ok:true,json:async()=>legacy};
  if(url.includes('/logs?'))return {ok:true,json:async()=>({lines:[]})};
  if(url==='/runtime/configuration'&&options.method==='POST'){
   if(fail){const status=fail;fail=null;state.revision++;state.desired.process='weld';state.desired.tools.left='hongik_v4';updateCaps();return {ok:false,status,json:async()=>({detail:'Revision mismatch'})};}
   const body=JSON.parse(options.body);assert.equal(body.expected_revision,state.revision);
   const patch=body.selection;state.desired={...state.desired,...patch};
   state.desired.tools=state.desired.platform==='rb10'?{single:'none'}:{left:'none',right:'none'};
   state.desired.tools[state.desired.active_arm]={spray:'spray_v2',weld:'hongik_v4',paint:'roller'}[state.desired.process];
   state.connections=body.connections;state.revision++;updateCaps();
  }
  if(url==='/runtime/apply'){state.phase='building';await new Promise(resolve=>{finish=()=>{state.phase='ready';state.applied=structuredClone(state.desired);state.applied_connections={...state.connections};resolve();};});}
  return {ok:true,json:async()=>structuredClone(state)};
 });
 await new Promise(setImmediate);
 return {ui,state,legacy,requests,failNext:()=>{fail=409;},finish:()=>finish()};
}
test('runtime choice updates only active EOAT and saves without applying',async()=>{
 const {ui,requests}=await setup();assert.ok(ui.element('runtime-process'),'runtime settings are present');
 ui.element('runtime-process').value='paint';await ui.element('runtime-process').fire('change');
 assert.match(ui.element('runtime-eoat').textContent,/롤러/);assert.match(ui.element('runtime-pending').textContent,/변경/);
 await ui.element('runtime-save').fire('click');
 const body=JSON.parse(requests.find(r=>r.url==='/runtime/configuration'&&r.method==='POST').body);
 assert.equal(body.selection.process,'paint');assert.equal(body.expected_revision,3);
 assert.match(ui.element('runtime-desired').textContent,/롤러도장/);assert.match(ui.element('runtime-applied').textContent,/내화뿜칠/);
 assert.equal(ui.evaluate('RuntimeSettings.snapshot().desired.tools.right'),'none');
 assert.equal(requests.some(r=>r.url==='/runtime/apply'),false);
});
test('platform and environment controls reflect mode while server blocks unsupported apply',async()=>{
 const {ui}=await setup();assert.ok(ui.element('runtime-mode'),'runtime settings are present');
 ui.element('runtime-mode').value='mixed';await ui.element('runtime-mode').fire('change');
 assert.match(ui.element('runtime-mode-help').textContent,/Isaac Sim.*실제 카메라/);
 assert.equal(ui.element('runtime-camera').hidden,false);assert.equal(ui.element('runtime-robot').hidden,true);
 ui.element('runtime-platform').value='rb10';await ui.element('runtime-platform').fire('change');assert.equal(ui.element('runtime-arm-field').hidden,true);
 ui.element('runtime-mode').value='real';await ui.element('runtime-mode').fire('change');await ui.element('runtime-save').fire('click');
 assert.equal(ui.element('runtime-robot').hidden,false);assert.equal(ui.element('runtime-apply').disabled,true);
 assert.match(ui.element('runtime-capability').textContent,/실환경 드라이버 미연결/);
 await ui.element('runtime-apply').fire('click');
});
test('stale revision reloads authoritative desired and applied configuration',async()=>{
 const {ui,failNext}=await setup();assert.ok(ui.element('runtime-process'),'runtime settings are present');
 ui.element('runtime-process').value='paint';await ui.element('runtime-process').fire('change');failNext();
 await ui.element('runtime-save').fire('click');
 assert.equal(ui.element('runtime-process').value,'weld');assert.match(ui.element('runtime-error').textContent,/다른|최신/);
 assert.match(ui.element('runtime-applied').textContent,/내화뿜칠/);
});
test('applying survives polling without duplicate apply and reconnect restores failures',async()=>{
 const {ui,state,requests,finish}=await setup();assert.ok(ui.element('runtime-apply'),'runtime settings are present');
 const applying=ui.element('runtime-apply').fire('click');await new Promise(setImmediate);
 await ui.evaluate('RuntimeSettings.refresh()');assert.match(ui.element('runtime-phase').textContent,/생성/);
 assert.equal(ui.element('runtime-apply').disabled,true);await ui.element('runtime-apply').fire('click');
 assert.equal(requests.filter(r=>r.url==='/runtime/apply').length,1);finish();await applying;
 state.phase='failed';state.error='asset build failed';await ui.evaluate('RuntimeSettings.refresh()');
 assert.match(ui.element('runtime-error').textContent,/asset build failed/);assert.match(ui.element('runtime-applied').textContent,/내화뿜칠/);
});
test('polling preserves unsaved draft and detects intervening server revisions',async()=>{
 const {ui,state}=await setup();assert.ok(ui.element('runtime-process'),'runtime settings are present');
 ui.element('runtime-process').value='paint';await ui.element('runtime-process').fire('change');state.revision++;
 await ui.evaluate('RuntimeSettings.refresh()');assert.equal(ui.element('runtime-process').value,'paint');
 assert.match(ui.element('runtime-pending').textContent,/다른|최신/);
});
test('reconnection clears a transport error and restores ready controls',async()=>{
 const {ui}=await setup();
 ui.evaluate("const workingFetch=fetch;fetch=async(url,options)=>{if(url==='/runtime/configuration')throw new Error('offline');return workingFetch(url,options);}");
 await ui.evaluate('RuntimeSettings.refresh()');assert.equal(ui.element('runtime-apply').disabled,true);assert.match(ui.element('runtime-error').textContent,/offline/);
 ui.evaluate('fetch=workingFetch');await ui.evaluate('RuntimeSettings.refresh()');
 assert.equal(ui.element('runtime-apply').disabled,false);assert.doesNotMatch(ui.element('runtime-error').textContent,/offline/);
});

test('execution status locks configuration edits and direct apply calls',async()=>{
 const {ui,legacy,requests}=await setup();legacy.ros.execution={running:true};await ui.evaluate('refresh()');
 assert.equal(ui.element('runtime-process').disabled,true);assert.equal(ui.element('runtime-apply').disabled,true);
 await ui.element('runtime-apply').fire('click');assert.equal(requests.some(r=>r.url==='/runtime/apply'),false);
 assert.match(ui.element('runtime-phase').textContent,/작업 실행/);
});
test('advanced legacy override acknowledges the visible runtime revision',async()=>{
 const {ui,legacy,requests}=await setup();legacy.processes[0].pid=null;await ui.evaluate('refresh()');
 ui.element('profile').value='isaac_rb20_paint';await ui.element('profile').fire('change');await ui.element('start').fire('click');
 const body=JSON.parse(requests.find(r=>r.url==='/configuration').body);assert.equal(body.expected_revision,3);assert.equal(body.profile,'isaac_rb20_paint');
});

for(const [source,value] of [['readiness',{running:true}],['welding',{running:true,state:'STOPPING'}],['motion_busy',true]]) {
 test(`actual ${source} motion contract locks configuration`,async()=>{
  const {ui,legacy,requests}=await setup();legacy.ros[source]=value;await ui.evaluate('refresh()');
  assert.equal(ui.element('runtime-process').disabled,true);
  await ui.element('runtime-apply').fire('click');assert.equal(requests.some(r=>r.url==='/runtime/apply'),false);
 });
}

test('runtime asset building can be cancelled after previous processes have stopped',async()=>{
 const {ui,legacy,requests,state}=await setup({phase:'building'});
 legacy.processes[0].pid=null;legacy.processes[0].state='STOPPED';legacy.runtime_configuration=state;
 await ui.evaluate('refresh()');
 assert.equal(ui.element('runtime-stop').disabled,false);
 await ui.element('runtime-stop').fire('click');
 assert.equal(requests.filter(r=>r.url==='/shutdown-system').length,1);
});

test('actionable startup reason remains visible after reconnect',async()=>{
 const reason='Paint hardware motion inhibited; full stack restart required. Inspect ros.log.';
 const {ui}=await setup({phase:'failed',error:reason});
 await ui.evaluate('RuntimeSettings.refresh()');
 assert.equal(ui.element('runtime-error').textContent,reason);
 assert.match(ui.element('runtime-phase').textContent,/실패/);
});

test('switching arm labels the unselected wrist as bare',async()=>{
 const {ui}=await setup();
 ui.element('runtime-process').value='paint';
 ui.element('runtime-arm').value='right';await ui.element('runtime-arm').fire('change');
 assert.match(ui.element('runtime-eoat').textContent,/오른팔:.*롤러.*왼팔: EOAT 없음/);
 await ui.element('runtime-save').fire('click');
 assert.equal(ui.evaluate('RuntimeSettings.snapshot().desired.tools.left'),'none');
 ui.element('runtime-arm').value='left';ui.element('runtime-process').value='spray';
 await ui.element('runtime-process').fire('change');
 assert.match(ui.element('runtime-eoat').textContent,/왼팔:.*Spray.*오른팔: EOAT 없음/);
});
