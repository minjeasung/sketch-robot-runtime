/* Versioned desired/applied configuration. Selection never starts a process. */
(function(root){
  const el=id=>document.getElementById(id);
  const names={simulation:'시뮬레이션',mixed:'일부 시뮬레이션',real:'실환경',commercial:'상용로봇 플랫폼',rb10:'RB10 베이스',left:'왼팔',right:'오른팔',single:'단일 팔',spray:'내화뿜칠',weld:'용접',paint:'롤러도장'};
  const tools={spray:'spray_v2',weld:'hongik_v4',paint:'roller'};
  const toolNames={spray_v2:'Spray v2 · 내화뿜칠',hongik_v4:'Hongik V4 용접 EOAT + 레이저 센서',roller:'롤러 · AFT200 · D405'};
  const fields=['outpost_http','outpost_zed_hw_id','outpost_zed_serial','outpost_d405_hw_id','outpost_d405_serial','zed_calibration_file','d405_calibration_file','robot_ip'];
  let state=null,draft=null,baseRevision=null,dirty=false,busy=false,connected=false,message='',connectionError='',legacyStatus=null;
  const copy=value=>JSON.parse(JSON.stringify(value));
  const canonical=value=>JSON.stringify(value,Object.keys(value||{}).sort());
  const summary=s=>s?[s.execution_mode,s.platform,s.active_arm,s.process].map(k=>names[k]||k).join(' · '):'아직 적용하지 않음';
  const executionBusy=()=>Boolean(legacyStatus?.ros?.motion_busy||['readiness','execution','welding'].some(key=>legacyStatus?.ros?.[key]?.running||legacyStatus?.ros?.[key]?.stopping));
  const inFlight=()=>['stopping','building','starting'].includes(state?.phase);
  function normalizeForm(next){
    draft=copy(next.desired);baseRevision=next.revision;dirty=false;
    for(const [field,id] of [['execution_mode','mode'],['platform','platform'],['active_arm','arm'],['process','process']])el('runtime-'+id).value=draft[field];
    for(const key of fields)el('runtime-'+key.replaceAll('_','-')).value=next.connections?.[key]||'';
  }
  function selectionFromForm(){
    const platform=el('runtime-platform').value;
    const active_arm=platform==='rb10'?'single':(['left','right'].includes(el('runtime-arm').value)?el('runtime-arm').value:'left');
    // The server rebuilds bare wrists and mounts only the selected arm's tool.
    return {schema_version:2,execution_mode:el('runtime-mode').value,platform,active_arm,process:el('runtime-process').value};
  }
  function connectionsFromForm(){
    const result={...(state?.connections||{})};
    for(const key of fields){const value=el('runtime-'+key.replaceAll('_','-')).value.trim();if(value||key in result)result[key]=value;}
    return result;
  }
  function selectedCapability(){
    const selection=selectionFromForm();return (state?.capabilities||[]).find(c=>['execution_mode','platform','active_arm','process'].every(k=>c[k]===selection[k]));
  }
  function appliedSummary(){
    const config=legacyStatus?.configuration;
    if(config&&legacyStatus.processes?.some(p=>p.pid!==null)){
      if(config.model_id==='rb20_1900es'||config.profile==='isaac_rb20_paint')return 'RB20-1900ES · 기존 롤러도장 · Isaac Sim';
      if(!state?.applied)return `기존 구성 · ${config.model_id} · ${config.profile}`;
    }
    return summary(state?.applied);
  }
  function renderRuntimeState(next){
    if(next?.desired&&Number.isInteger(next.revision)){
      if(state&&next.revision<state.revision)return;
      state=copy(next);connected=true;if(!dirty)normalizeForm(state);
    }
    if(!draft){el('runtime-applied').textContent=appliedSummary();el('runtime-error').textContent=connectionError||message||'실행 구성 API 연결을 기다립니다.';el('runtime-apply').disabled=true;return;}
    const selection=selectionFromForm(),locked=busy||inFlight()||executionBusy(),capability=selectedCapability();
    el('runtime-arm-field').hidden=selection.platform!=='commercial';
    el('runtime-mode-help').textContent={simulation:'로봇: Isaac Sim · 카메라: Isaac Sim 가상 카메라',mixed:'로봇: Isaac Sim · 카메라: 실제 카메라',real:'로봇: 실제 로봇 · 카메라: 실제 카메라'}[selection.execution_mode];
    el('runtime-eoat').textContent=selection.platform==='commercial'
      ? `${names[selection.active_arm]}: ${toolNames[tools[selection.process]]} · ${selection.active_arm==='left'?'오른팔':'왼팔'}: EOAT 없음`
      : toolNames[tools[selection.process]];
    el('runtime-camera').hidden=selection.execution_mode==='simulation';
    el('runtime-robot').hidden=selection.execution_mode!=='real';el('runtime-robot-connection').hidden=selection.execution_mode!=='real';
    el('runtime-camera-status').textContent=state.phase==='ready'&&!dirty&&canonical(state.connections)===canonical(state.applied_connections)?'적용 구성 준비됨 · 영상·보정 상세는 준비 상태 확인':'연결·보정 확인 필요';
    el('runtime-desired').textContent=summary(state.desired);
    el('runtime-applied').textContent=appliedSummary();
    const changed=JSON.stringify(state.desired)!==JSON.stringify(state.applied)||canonical(state.connections)!==canonical(state.applied_connections);
    el('runtime-pending').textContent=dirty?(baseRevision!==state.revision?'다른 창에서 설정이 변경됨 · 저장 시 최신 상태를 확인합니다.':'저장하지 않은 변경 사항'):changed?'변경 대기 · 설정을 적용하세요.':'저장 구성과 적용 구성이 일치합니다.';
    el('runtime-phase').textContent=connected?({idle:'적용 대기',stopping:'프로세스 종료 중',building:'자산 생성·검증 중',starting:'프로세스 시작·준비 확인 중',ready:'준비됨',failed:'적용 실패'}[state.phase]||state.phase):'연결 끊김 · 상태 확인 필요';
    if(connected&&executionBusy())el('runtime-phase').textContent='작업 실행·중단 중 · 구성 변경 잠김';
    el('runtime-capability').textContent=!capability?'설정을 저장하면 서버에서 지원 여부를 확인합니다.':!capability.supported?'미지원 · '+capability.reason:[capability.reason,selection.process!=='paint'?'이동 시험 · 공정 출력 없음':'',capability.verification==='not_run'?'장비 통합 검증 미완료':''].filter(Boolean).join(' · ');
    el('runtime-error').textContent=connectionError||message||state.error||'';
    el('runtime-save').disabled=!connected||locked||!dirty;
    el('runtime-apply').disabled=!connected||locked||dirty||!capability?.supported;
    for(const id of ['mode','platform','arm','process','camera-load','zed-select','d405-select',...fields.map(k=>k.replaceAll('_','-'))])el('runtime-'+id).disabled=!connected||locked;
  }
  async function refresh(force=false){
    try{const next=await api('/runtime/configuration');if(!next?.desired)throw new Error('실행 구성 API를 사용할 수 없습니다.');if(force)dirty=false;connectionError='';renderRuntimeState(next);}
    catch(error){connected=false;connectionError=error.message;renderRuntimeState();}
  }
  async function perform(kind){
    if(!state||!connected||busy||inFlight()||executionBusy()||(kind==='apply'&&(dirty||!selectedCapability()?.supported)))return;
    busy=true;message='';renderRuntimeState();
    try{
      const body=kind==='save'?{selection:selectionFromForm(),connections:connectionsFromForm(),expected_revision:baseRevision}:{expected_revision:state.revision};
      const next=await api(kind==='save'?'/runtime/configuration':'/runtime/apply','POST',body);
      dirty=false;renderRuntimeState(next);
    }catch(error){
      if(error.status===409){await refresh(true);message='다른 작업 또는 설정 변경으로 요청을 적용하지 못했습니다. 최신 상태를 확인하세요. '+error.message;}
      else message=error.message;
    }finally{busy=false;await refresh();renderRuntimeState();}
  }
  function edit(){
    if(!draft)return;
    if(el('runtime-platform').value==='commercial'&&!['left','right'].includes(el('runtime-arm').value))el('runtime-arm').value='left';
    dirty=true;message='';renderRuntimeState();
  }
  for(const id of ['mode','platform','arm','process',...fields.map(k=>k.replaceAll('_','-'))]){
    el('runtime-'+id).addEventListener('change',edit);
    if(fields.map(k=>k.replaceAll('_','-')).includes(id))el('runtime-'+id).addEventListener('input',edit);
  }
  el('runtime-save').addEventListener('click',()=>perform('save'));el('runtime-apply').addEventListener('click',()=>perform('apply'));
  el('runtime-camera-load').addEventListener('click',async()=>{
    if(busy||inFlight())return;
    message='';
    try{
      const result=await api('/outpost/cameras?outpost_http='+encodeURIComponent(el('runtime-outpost-http').value.trim()));
      for(const [name,kind] of [['zed','zed'],['d405','realsense']]){
        const select=el('runtime-'+name+'-select');select.replaceChildren(new Option('보정한 카메라를 선택하세요',''));
        for(const camera of result.cameras.filter(c=>c.camera_type===kind)){const item=new Option(`${camera.camera_id} · ${camera.state}`,camera.hw_id);item.dataset.serial=camera.camera_id;select.add(item);}
      }
    }catch(error){message=error.message;}renderRuntimeState();
  });
  for(const name of ['zed','d405'])el('runtime-'+name+'-select').addEventListener('change',event=>{
    const item=event.target.selectedOptions[0];if(!item?.value)return;
    el(`runtime-outpost-${name}-hw-id`).value=item.value;el(`runtime-outpost-${name}-serial`).value=item.dataset.serial;edit();
  });
  root.RuntimeSettings={setLegacyStatus:value=>{legacyStatus=value;},normalizeForm,selectionFromForm,renderRuntimeState,refresh,snapshot:()=>state?copy(state):null};
})(window);
