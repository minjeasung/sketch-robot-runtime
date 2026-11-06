/* Presentation only: process adapters retain identity checks and ROS ownership. */
(function(root) {
  'use strict';
  const keys=['target','work_area','path'];
  const labels=['대상 인식','작업 지정','계획·실행'];
  function derive(i={}) {
    const known=['paint','spray','weld'].includes(i.mode);
    const online=known&&i.connected===true&&i.modePending===false;
    const busy=Boolean(i.running||i.stopping||i.pending);
    const unlocked=online&&!busy;
    const stage=keys.includes(i.stage)?i.stage:'target';
    const target=i.targetReady===true,job=target&&i.jobReady===true;
    const generated=online&&!i.dirty&&i.generated===true;
    const validated=generated&&!i.planningOnly&&i.validated===true;
    const ready=unlocked&&job&&validated&&i.settingsApplied===true&&i.canRun===true;
    const reasons=['',target?'':'먼저 대상을 인식·확정하세요.',job?'':'먼저 작업할 영역 또는 심을 확정하세요.'];
    const steps=keys.map((key,n)=>({key,label:labels[n],complete:[target,job,ready][n],
      accessible:!busy&&(n===0||online&&[true,target,job][n]),reason:busy?'현재 요청 또는 중단 확인을 기다립니다.':reasons[n]}));
    const detail=i.planningOnly?'로봇 검증 미수행':i.mode==='weld'?
      '접근·진입·용접·이탈 사전 계획': '경로·작업영역·공정 조건 검사 · 이동 구간 검사: 실행 중 수행';
    const check=(complete,label,detail='',unavailable=false)=>({
      state:unavailable?'unavailable':!online?'waiting':i.dirty?'blocked':complete?'complete':i.pending?'pending':i.error?'blocked':'waiting',label,detail});
    return {stage,mode:i.mode,steps,show:{target:stage==='target',area:stage==='work_area'&&i.mode!=='weld',
      seams:stage==='work_area'&&i.mode==='weld',path:stage==='path',settings:stage==='path',execution:stage==='path',
      drawing:i.mode==='weld'?stage==='target':stage!=='path'||i.pathMethod==='manual'},
      canEdit:unlocked,canPlan:unlocked&&stage==='path'&&job&&i.settingsApplied===true&&i.canPlan===true,
      canRun:stage==='path'&&ready,canStop:i.connected===true&&!i.planningOnly,
      next:steps[keys.indexOf(stage)+1]||null,
      reason:!online?'공정과 서버 연결을 확인하세요.':busy?'현재 요청 또는 중단 확인을 기다립니다.':
        i.error||((i.blockers||[]).filter(Boolean).join(' · '))||reasons[keys.indexOf(stage)],
      checks:{generated:check(generated,'경로 생성'),validated:check(validated,'계획 검사',detail,!!i.planningOnly),
        ready:check(ready,'실행 준비',i.planningOnly?'미리보기 전용':i.mode==='paint'?'접촉 전 영점·접촉력 유지: 실행 중 확인':'',!!i.planningOnly)}};
  }
  function render(v) {
    const $=id=>root.document.getElementById(id);
    const hide=(id,hidden)=>{const e=$(id);if(e)e.hidden=hidden;};
    for(const [id,key] of [['target-actions','target'],['work-area-actions','area'],['weld-seam-actions','seams'],
      ['path-actions','path'],['process-settings','settings'],['execution-section','execution'],['drawing-toolbar','drawing']]) hide(id,!v.show[key]);
    for(const radio of root.document.querySelectorAll('input[name="workflow-mode"]')) {
      const step=v.steps.find(s=>s.key===radio.value);
      radio.disabled=!step.accessible;radio.checked=v.stage===step.key;radio.title=step.reason;
    }
    for(const radio of root.document.querySelectorAll('input[name="sketch-mode"]')) radio.disabled=!v.canEdit||!v.show.drawing;
    const next=$('btn-workflow-next');if(next){next.hidden=!v.next;next.disabled=!v.next?.accessible;next.textContent=v.next?'다음: '+v.next.label:'';}
    if($('workflow-stage-reason'))$('workflow-stage-reason').textContent=v.reason||v.next?.reason||'';
    if($('btn-plan-workflow'))$('btn-plan-workflow').disabled=!v.canPlan;
    if($('path-method'))$('path-method').disabled=!v.canEdit;
    hide('path-method-field',v.mode==='weld');
    hide('weld-selection-summary',v.mode!=='weld'||!v.show.path);
    if($('btn-run-robot'))$('btn-run-robot').disabled=!v.canRun;
    if($('btn-stop-robot')){$('btn-stop-robot').disabled=!v.canStop;$('btn-stop-robot').hidden=false;}
    for(const id of ['btn-undo','btn-clear']) if($(id))$(id).disabled=$(id).disabled||!v.canEdit||!v.show.drawing;
    const text={waiting:'대기',pending:'처리 중',complete:'완료',blocked:'확인 필요',unavailable:'미수행'};
    for(const [key,id] of [['generated','workflow-generation-state'],['validated','painting-plan-validation'],['ready','workflow-ready-state']]) {
      const e=$(id),c=v.checks[key];if(e){e.textContent=text[c.state];e.className='state-pill '+(c.state==='complete'?'good':c.state==='blocked'?'bad':'unknown');e.title=c.detail;}
    }
    if($('painting-plan-label'))$('painting-plan-label').textContent='계획 검사';
    if($('workflow-validation-scope'))$('workflow-validation-scope').textContent=v.checks.validated.detail;
    if($('workflow-ready-detail'))$('workflow-ready-detail').textContent=v.checks.ready.detail;
  }
  const api={derive,render};
  if(typeof module!=='undefined'&&module.exports)module.exports=api;
  else root.Workflow=api;
})(typeof window==='undefined'?globalThis:window);
