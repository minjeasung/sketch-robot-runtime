const test=require('node:test'),assert=require('node:assert/strict');
const {loadUI}=require('./ui_harness');
const snap=(mode,revision=0)=>({mode,server_session_id:'s',revision,applied:revision>0,fields:{speed_mps:{label:'속도',unit:'mm/s',scale:1000,value:.01,editable:mode==='weld'}}});
test('settings drafts are per process and readonly values are server values',()=>{
 const ui=loadUI(); ui.emit('/process_settings/status',snap('weld'));
 assert.equal(ui.evaluate('typeof ProcessSettings'),'object');
 ui.evaluate("ProcessSettings.setMode('weld'); ProcessSettings.edit('speed_mps', .02)");
 ui.emit('/process_settings/status',snap('spray')); ui.evaluate("ProcessSettings.setMode('spray')");
 assert.equal(ui.evaluate('ProcessSettings.values().speed_mps'),.01);
 ui.evaluate("ProcessSettings.setMode('weld')"); assert.equal(ui.evaluate('ProcessSettings.values().speed_mps'),.02);
});
test('stale ack cannot enable run after a local edit',()=>{
 const ui=loadUI(); ui.emit('/process_settings/status',snap('weld',1)); ui.evaluate("ProcessSettings.setMode('weld'); ProcessSettings.edit('speed_mps', .02)");
 ui.emit('/process_settings/status',snap('weld',1)); assert.equal(ui.evaluate('ProcessSettings.isApplied()'),false);
});
test('explicit pose selection is displayed and sent as a string setting',async()=>{
 const ui=loadUI();ui.ros.emit('connection');ui.emit('/painting_system/process_mode',{mode:'weld'});
 ui.emit('/welding/status',{server_session_id:'s',identity:{},state:'SELECTED',seams:[],selection:[]});
 ui.emit('/process_settings/status',{...snap('weld'),fields:{pose_profile_id:{label:'기준 자세',value:'',editable:true,options:[{value:'',label:'선택 안 함'},{value:'demo',label:'시험 자세'}]}}});
 const input=ui.element('settings-fields').children[0].children[1];
 assert.equal(input.tagName.toLowerCase(),'select');
 input.value='demo';await input.fire('input');ui.evaluate('ProcessSettings.apply()');
 assert.equal(JSON.parse(ui.published.at(-1).message.data).values.pose_profile_id,'demo');
});
