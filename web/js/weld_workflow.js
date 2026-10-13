/* Seam selection in the original sketch page. No drawn path becomes a weld. */
(function(root){
  let status=null,frame=null,header=null,invalid=false,pending=null,sequence=0,listSignature='';
  const pubs={};
  function id(){return 'ui-'+Date.now()+'-'+(++sequence);}
  function send(topic,payload){
    if(!rosConnected)return false;
    if(!pubs[topic]){
      pubs[topic]=new ROSLIB.Topic({ros,name:topic,messageType:'std_msgs/String',reconnect_on_close:false});
      // The bundled ROSLIB leaves this method unbound when replay is disabled.
      pubs[topic].callForSubscribeAndAdvertise=ros.callOnConnection.bind(ros);
    }
    pubs[topic].publish(new ROSLIB.Message({data:JSON.stringify(payload)}));return true;
  }
  function derived(){return {running:Boolean(status?.running),ready:rosConnected&&!invalid&&!pending&&status?.runtime_ready===true&&status?.ready===true&&ProcessSettings.isApplied(),surfaceAccepted:Boolean(status?.seams?.length),planValidated:status?.ready===true};}
  const api={
    command(topic,payload){
      if(!status||!rosConnected||status.running||pending)return false;
      const commandId=id();
      const result=send(topic,{...payload,command_id:commandId,server_session_id:status.server_session_id,expected:status.identity});
      if(result)pending={identity:JSON.stringify(status.identity),state:status.state,topic,commandId};
      return result;
    },
    setStatus(p){
      if(!p||!p.identity||!p.server_session_id)return;
      if(status&&status.server_session_id!==p.server_session_id){invalid=true;pending=null;ProcessSettings.disconnected();}
      if(pending?.commandId&&p.command_ack?.command_id===pending.commandId){
        if(!p.command_ack.ok&&pending.topic==='/process_settings/request')ProcessSettings.reject();
        pending=null;
      }
      // Settings owns a second pending state; keep its command until the ACK
      // arrives, even if a scene/head update changes the plan identity first.
      if(pending&&pending.topic!=='/process_settings/request'&&(JSON.stringify(p.identity)!==pending.identity||p.state!==pending.state||p.error))pending=null;
      status=p;ProcessSettings.setRunning(p.running);api.render();
    },
    setImageHeader(p){header=p;},
    requestTarget(){
      if(!frame||!header||!status||!rosConnected||status.running)return;
      const strokes=currentStrokes();if(strokes.length!==1){logEvent('하나의 닫힌 대상 영역을 지정하세요.');return;}
      const stroke=strokes[0],points=stroke.points.map(p=>[p.u,p.v]);
      if(points.length<3)return;
      if(stroke.type!=='rect'&&Math.hypot(points[0][0]-points.at(-1)[0],points[0][1]-points.at(-1)[1])>10){logEvent('대상을 닫힌 영역으로 둘러 표시하세요.');return;}
      const request=id();invalid=false;
      send('/welding/target_request',{...frame,stamp:header.stamp,request_id:request,target_revision:request,
        polygon:points,provider:status.provider||'fixture',server_session_id:status.server_session_id});
      pending={identity:JSON.stringify(status.identity),state:status.state,topic:'target'};api.render();
    },
    select(selection){if(api.command('/welding/selection',{selection}))api.render();},
    requestPlan(){if(!invalid&&ProcessSettings.isApplied()&&status?.selection?.length)api.command('/welding/plan_request',{});api.render();},
    execute(){if(derived().ready)api.command('/welding/execute',{plan_id:status.plan_id});api.render();},
    invalidate(){invalid=true;if(status&&!status.running)api.command('/welding/invalidate',{});api.render();},
    derived,
    render(){
      if(processMode!=='weld'){listSignature='';return derived();}
      ProcessSettings.setMode('weld');
      const d=derived(),blocked=!rosConnected||processModePending||d.running||Boolean(pending);
      $('target-actions').hidden=false;$('work-area-actions').hidden=true;$('path-actions').hidden=false;
      $('btn-set-target').textContent=status?.provider==='external'?'용접심 추출 요청':'시험 용접심 불러오기';$('btn-set-target').disabled=blocked||!currentStrokes().length||!frame||!header;
      $('btn-refine-planes').hidden=true;$('active-plane-field').hidden=true;$('tare-confirmation').hidden=true;
      $('btn-fill-work-area').textContent='용접 경로 검증';$('btn-fill-work-area').disabled=blocked||invalid||!status?.selection?.length||!ProcessSettings.isApplied();
      $('btn-execute').hidden=true;$('btn-run-robot').disabled=!d.ready;$('btn-stop-robot').disabled=!rosConnected;$('btn-stop-robot').hidden=false;
      $('painting-surface-label').textContent='3D 용접심';$('painting-plan-label').textContent='토치·위빙 경로';
      setPill('painting-d405-state',d.surfaceAccepted?'수신됨':'대기',d.surfaceAccepted?'good':'unknown');
      setPill('painting-plan-validation',status?.state==='PLANNING'?'검증 중':d.planValidated?'완료':'대기',d.planValidated?'good':'unknown');
      $('process-mode-state').textContent='Isaac 용접 이동 시험 · 용접심 중심 경로와 자세 유지';
      $('planes-status').textContent=(status?.provider==='external'?'추출된 용접심':'시험 작업물에 등록된 용접심')+' · 선택 순서로 실행';
      $('workflow-hint').textContent='ZED 영상에서 작업대상을 둘러 표시하고, 오른쪽 목록에서 용접심을 선택하세요.';
      $('execution-summary').textContent=status?.error||({EMPTY:'대상을 지정하세요.',TARGET_READY:'용접심을 기다립니다.',SEAMS_READY:'용접심을 선택하세요.',SELECTED:'Settings 적용 후 경로를 검증하세요.',PLANNING:'접근·위빙·이탈 경로 검증 중',READY:'검증 완료 · 작업 시작 가능',RUNNING:'용접 경로 이동 중',STOPPING:'중단 확인 중',STOPPED:'작업 중단됨',COMPLETED:'선택한 용접심 이동 완료'}[status?.state]||'용접 실행기 연결 대기');
      if(status?.state==='RUNNING')$('execution-summary').textContent=`${status.current_seam_id} · ${({APPROACH:'접근',ENTRY:'진입',WELD:'용접·위빙',RETRACT:'이탈',TRANSFER:'다음 심으로 이동'})[status.current_phase]||status.current_phase}`;
      $('painting-blockers').textContent=status?.error||'';
      const key=JSON.stringify([status?.seams,status?.selection,blocked]);
      if(key!==listSignature){listSignature=key;$('plane-candidates').replaceChildren();
        for(const seam of status?.seams||[]){
          const row=document.createElement('div');row.className='seam-row';
          const label=document.createElement('label'),check=document.createElement('input'),selection=status.selection||[];
          const index=selection.findIndex(s=>s.seam_id===seam.seam_id);check.type='checkbox';check.checked=index>=0;check.disabled=blocked;
          check.addEventListener('change',()=>api.select(check.checked?[...selection,{seam_id:seam.seam_id,reverse:false}]:selection.filter(s=>s.seam_id!==seam.seam_id)));
          label.append(check,document.createTextNode(`${seam.seam_id} · ${Math.round(seam.length_m*1000)} mm · ${seam.frame_id||''}${index>=0?' · '+(index+1)+'번째':''}${seam.pose_ready?'':' · 자세 필요'}`));row.append(label);
          if(index>=0){
            const reverse=document.createElement('button');reverse.textContent=selection[index].reverse?'← 역방향':'→ 정방향';reverse.disabled=blocked;
            reverse.addEventListener('click',()=>api.select(selection.map((s,i)=>i===index?{...s,reverse:!s.reverse}:s)));row.append(reverse);
            if(index>0){const up=document.createElement('button');up.textContent='↑';up.setAttribute('aria-label','실행 순서 앞으로');up.disabled=blocked;up.addEventListener('click',()=>{const next=[...selection];[next[index-1],next[index]]=[next[index],next[index-1]];api.select(next);});row.append(up);}
          }$('plane-candidates').append(row);
        }
      }
      const drawingBlocked=!rosConnected||processModePending||d.running;
      for(const radio of document.querySelectorAll('input[name="workflow-mode"]'))radio.disabled=radio.value!=='target'||drawingBlocked;
      for(const radio of document.querySelectorAll('input[name="sketch-mode"]'))radio.disabled=drawingBlocked;
      $('btn-clear').disabled=drawingBlocked||!currentStrokes().length;
      $('btn-undo').disabled=drawingBlocked||!currentStrokes().length;
      return d;
    }
  };
  root.WeldWorkflow=api;
  new ROSLIB.Topic({ros,name:'/welding/status',messageType:'std_msgs/String'}).subscribe(msg=>{try{api.setStatus(JSON.parse(msg.data));}catch(e){logEvent('용접 상태 오류: '+e.message);}});
  new ROSLIB.Topic({ros,name:'/perception/zed_frame_identity',messageType:'std_msgs/String'}).subscribe(msg=>{try{frame=JSON.parse(msg.data);api.render();}catch(_){frame=null;}});
  ros.on('close',()=>{status=null;header=null;frame=null;invalid=true;pending=null;api.render();});
})(window);
