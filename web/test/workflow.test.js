const test = require('node:test');
const assert = require('node:assert/strict');
const {derive} = require('../js/workflow');
const input = overrides => ({mode:'paint',stage:'target',connected:true,modePending:false,
 planningOnly:false,running:false,stopping:false,pending:false,targetReady:false,jobReady:false,
 settingsApplied:true,canPlan:false,canRun:false,generated:false,validated:false,dirty:false,
 pathMethod:'auto',error:'',blockers:[],...overrides});
test('stage_names_are_shared and visiting a stage cannot complete it',()=>{
 for(const mode of ['paint','spray','weld']) {
  const v=derive(input({mode,stage:'path'}));
  assert.deepEqual(v.steps.map(s=>s.label),['대상 인식','작업 지정','계획·실행']);
  assert.equal(v.steps[1].complete,false);assert.equal(v.canRun,false);
 }
});
test('generated_is_not_validated',()=>{
 const v=derive(input({stage:'path',targetReady:true,jobReady:true,generated:true}));
 assert.equal(v.checks.generated.state,'complete');assert.equal(v.checks.validated.state,'waiting');
 assert.equal(v.canRun,false);
});
test('preview_never_enables_run even if upstream claims ready',()=>{
 const v=derive(input({stage:'path',targetReady:true,jobReady:true,planningOnly:true,canRun:true,validated:true,generated:true}));
 assert.equal(v.canRun,false);assert.equal(v.checks.validated.state,'unavailable');
});
test('pending_and_stop_lock_mutations while stop remains available',()=>{
 for(const flags of [{pending:true},{stopping:true},{running:true}]) {
  const v=derive(input({stage:'path',targetReady:true,jobReady:true,canPlan:true,canRun:true,generated:true,validated:true,...flags}));
  assert.equal(v.canEdit,false);assert.equal(v.canPlan,false);assert.equal(v.canRun,false);
  assert.equal(v.canStop,true);assert.ok(v.steps.every(s=>!s.accessible));
 }
});
test('missing_values_fail_closed and invalid modes never allow execution',()=>{
 for(const i of [{},input({mode:'unknown',canRun:true,canPlan:true})]) {
  const v=derive(i);assert.equal(v.canRun,false);assert.equal(v.canPlan,false);assert.equal(v.canEdit,false);
 }
});
test('navigation readiness and edit dirtiness are distinct',()=>{
 const v=derive(input({stage:'path',targetReady:true,jobReady:true,dirty:true,generated:true,validated:true,canRun:true}));
 assert.equal(v.steps[0].accessible,true);assert.equal(v.steps[1].accessible,true);
 assert.equal(v.canRun,false);assert.notEqual(v.checks.validated.state,'complete');
});
