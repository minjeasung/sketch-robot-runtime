/* Fixture catalogs and frame-bound, depth-projected seam input share the three stages. */
(function(root){
  let status=null,frame=null,header=null,invalid=false,pending=null,sequence=0,listSignature='';
  const pubs={};
  const frames=new Map(),overlays=new Map();let frozen=null;
  const needsPhysicalFrame=()=>processMode==='weld'?status?.provider==='external':typeof executionRuntime!=='undefined'&&executionRuntime.execution_mode==='mixed';
  const frameKey=p=>p?.stamp&&p?.frame_id?`${p.frame_id}:${p.stamp.sec}:${p.stamp.nanosec}`:'';
  const retiredSessions=new Set(),retiredPlans=new Set();
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
  function derived(){
    const valid=processMode==='weld'&&rosConnected&&!invalid&&!pending&&status?.ready===true&&
      Boolean(status.plan_id)&&!retiredPlans.has(status.plan_id)&&ProcessSettings.isApplied();
    return {running:Boolean(status?.running),ready:valid&&status?.runtime_ready===true,
      surfaceAccepted:!invalid&&Boolean(status?.seams?.length),planValidated:valid};
  }
  const api={
    command(topic,payload){
      if(!status||!rosConnected||status.running||status.state==='PLANNING'||pending)return false;
      const commandId=id();
      const result=send(topic,{...payload,command_id:commandId,server_session_id:status.server_session_id,expected:status.identity});
      if(result){
        if(topic!=='/welding/execute'&&status.plan_id)retiredPlans.add(status.plan_id);
        pending={identity:JSON.stringify(status.identity),state:status.state,topic,commandId,payload};
      }
      return result;
    },
    setStatus(p){
      if(!p||!p.identity||!p.server_session_id||retiredSessions.has(p.server_session_id))return;
      if(status&&status.server_session_id!==p.server_session_id){
        retiredSessions.add(status.server_session_id);invalid=true;pending=null;ProcessSettings.disconnected();
      }
      if(status?.server_session_id===p.server_session_id){
        for(const key of ['selection_revision','settings_revision'])
          if(Number(p.identity[key])<Number(status.identity[key]))return;
      }
      if(status?.plan_id&&p.ready!==true)retiredPlans.add(status.plan_id);
      if(pending?.commandId&&p.command_ack?.command_id===pending.commandId){
        const accepted=p.command_ack.ok===true;
        if(!accepted&&pending.topic==='/process_settings/request')ProcessSettings.reject();
        const selectionApplied=pending.topic!=='/welding/selection'||!accepted||
          JSON.stringify(p.selection)===JSON.stringify(pending.payload.selection);
        if(selectionApplied)pending=null;
      }
      if(pending?.topic==='target'){
        // Validation may reject before accepting the requested revision. Such an
        // error can release editing, but must never bless the previous catalog.
        if(p.identity.target_revision===pending.targetRevision){
          if(p.state==='SEAMS_READY'||p.seams?.length||p.error)pending=null;
        }else if(p.error){invalid=true;pending=null;}
      }
      if(typeof stopRequested!=='undefined'&&stopRequested&&p.state==='STOPPED'&&p.running===false){
        stopRequested=false;pending=null;ProcessSettings.reject();
      }
      status=p;api.render();if(typeof redrawSketch==='function')redrawSketch();
    },
    setImageHeader(p){header=p;if(typeof redrawSketch==='function')redrawSketch();},
    setOverlay(p){
      if(!p?.header||!Array.isArray(p.seams)||!Number.isFinite(p.width)||!Number.isFinite(p.height))return;
      overlays.set(frameKey(p.header),p);while(overlays.size>30)overlays.delete(overlays.keys().next().value);
      if(typeof redrawSketch==='function')redrawSketch();
    },
    drawOverlay(ctx,width,height){
      if(processMode!=='weld'||currentView!=='zed_raw'||!rosConnected||invalid)return;
      const p=overlays.get(frameKey(header));
      if(!p||p.generation!==status?.identity?.generation||p.target_revision!==status?.identity?.target_revision||
          p.width!==width||p.height!==height||!status?.seams?.length)return;
      const selection=status.selection||[];
      const pointOK=v=>Array.isArray(v)&&v.length===2&&v.every(Number.isFinite);
      ctx.save();ctx.lineCap='round';ctx.lineJoin='round';
      p.seams.forEach((seam,index)=>{
        if(!status.seams.some(s=>s.seam_id===seam.seam_id)||!Array.isArray(seam.points_px))return;
        const order=selection.findIndex(s=>s.seam_id===seam.seam_id),chosen=order>=0;
        const points=chosen&&selection[order].reverse?[...seam.points_px].reverse():seam.points_px;
        const color=chosen?'#0891b2':'#ea580c';
        const path=()=>{let started=false;ctx.beginPath();for(const point of points){
          if(!pointOK(point)){started=false;continue;}
          if(started)ctx.lineTo(...point);else ctx.moveTo(...point);started=true;
        }ctx.stroke();};
        ctx.strokeStyle='#ffffff';ctx.lineWidth=7;path();
        ctx.strokeStyle=color;ctx.lineWidth=3;path();
        const label=(text,point,fill)=>{
          if(!pointOK(point)||point[0]<0||point[0]>width||point[1]<0||point[1]>height)return;
          const [x,y]=point;ctx.beginPath();ctx.arc(x,y,5,0,2*Math.PI);ctx.fillStyle=fill;ctx.fill();
          ctx.strokeStyle='#fff';ctx.lineWidth=2;ctx.stroke();
          ctx.font='bold 13px sans-serif';ctx.lineWidth=3;ctx.strokeStyle='#fff';
          ctx.strokeText(text,x+8,y-9);ctx.fillStyle='#0f172a';ctx.fillText(text,x+8,y-9);
        };
        label(chosen?'시작':`후보 ${index+1}`,points[0],chosen?'#16a34a':color);
        if(chosen)label('끝',points.at(-1),'#e11d48');
        // A direction arrow on a real contiguous segment; never bridge points behind the camera.
        if(chosen){const mid=Math.floor((points.length-1)/2),a=points[mid],b=points[mid+1];
          if(pointOK(a)&&pointOK(b)&&Math.hypot(b[0]-a[0],b[1]-a[1])>.01){
            const angle=Math.atan2(b[1]-a[1],b[0]-a[0]),x=(a[0]+b[0])/2,y=(a[1]+b[1])/2;
            ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x-10*Math.cos(angle-.5),y-10*Math.sin(angle-.5));
            ctx.lineTo(x-10*Math.cos(angle+.5),y-10*Math.sin(angle+.5));ctx.closePath();ctx.fillStyle=color;ctx.fill();
          }
        }
      });ctx.restore();
    },
    isFrameFrozen(){return needsPhysicalFrame()&&Boolean(frozen);},
    beginDrawing(){
      if(!needsPhysicalFrame())return true;
      if(frozen)return true;
      const match=frames.get(frameKey(header));
      if(!match?.frame_binding||match.valid!==true||match.source!=='physical'){logEvent('표시 영상과 일치하는 RGB-D 프레임 정보를 기다리세요.');return false;}
      if(!send('/perception/selection_pin',match))return false;
      frozen=JSON.parse(JSON.stringify(match));return true;
    },
    releaseFrame(){frozen=null;},
    submitTaskScene(strokes,selectionStamp){
      if(!needsPhysicalFrame())return true;
      if(!frozen?.frame_binding){logEvent('일치하는 실제 RGB-D 영상에서 대상을 다시 지정하세요.');return false;}
      if(strokes.length!==1||strokes[0].points.length<3){logEvent('실제 작업면을 하나의 닫힌 영역으로 지정하세요.');return false;}
      const stroke=strokes[0],polygon=stroke.points.map(p=>[p.u,p.v]);
      if(stroke.type!=='rect'&&Math.hypot(polygon[0][0]-polygon.at(-1)[0],polygon[0][1]-polygon.at(-1)[1])>10){logEvent('실제 작업면을 닫힌 영역으로 지정하세요.');return false;}
      const revision=`${selectionStamp.sec}.${selectionStamp.nanosec}`;
      return send('/task_scene/selection',{...frozen,polygon,request_id:revision,target_revision:revision});
    },
    requestTarget(){
      if(!frame||!header||!status||!rosConnected||status.running||pending||workflowMode!=='target')return;
      const strokes=currentStrokes();
      const external=status.provider==='external';
      if(strokes.length!==1){logEvent(external?'하나의 용접선을 지정하세요.':'하나의 닫힌 대상 영역을 지정하세요.');return;}
      const stroke=strokes[0],points=stroke.points.map(p=>[p.u,p.v]);
      let polygon=points;
      if(external){
        if(!frozen?.frame_binding||stroke.type==='rect'||points.length<2){logEvent('실제 영상에서 직선 또는 자유선으로 용접선을 지정하세요.');return;}
        const xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);
        const x0=Math.max(0,Math.min(...xs)-2),x1=Math.min(frozen.width-1,Math.max(...xs)+2);
        const y0=Math.max(0,Math.min(...ys)-2),y1=Math.min(frozen.height-1,Math.max(...ys)+2);
        polygon=[[x0,y0],[x1,y0],[x1,y1],[x0,y1]];
      }else{
        if(points.length<3)return;
        if(stroke.type!=='rect'&&Math.hypot(points[0][0]-points.at(-1)[0],points[0][1]-points.at(-1)[1])>10){logEvent('대상을 닫힌 영역으로 둘러 표시하세요.');return;}
      }
      let reference;
      if(external&&$('weld-reference-override').checked){
        const values=['x','y','z'].map(axis=>$('weld-reference-'+axis).value.trim());
        reference=values.map(Number);
        if(values.some(v=>!v)||!reference.every(Number.isFinite)||Math.abs(Math.hypot(...reference)-1)>.001){logEvent('유한한 단위 법선 벡터 (길이 1)를 입력하세요.');return;}
      }
      const request=id();invalid=false;
      const selected=external?frozen:{...frame,stamp:header.stamp};
      send('/welding/target_request',{...selected,request_id:request,target_revision:request,
        polygon,...(external?{polyline_px:points,frame_binding:frozen.frame_binding,...(reference?{reference_up:reference}:{})}:{}),provider:status.provider||'fixture',server_session_id:status.server_session_id});
      pending={identity:JSON.stringify(status.identity),state:status.state,topic:'target',targetRevision:request};api.render();
    },
    select(selection){if(api.command('/welding/selection',{selection}))api.render();},
    requestPlan(){if(workflowView().canPlan)api.command('/welding/plan_request',{});api.render();},
    execute(){if(workflowView().canRun)api.command('/welding/execute',{plan_id:status.plan_id});api.render();},
    invalidate(){invalid=true;if(status&&!status.running)api.command('/welding/invalidate',{});api.render();},
    reset(){status=null;pending=null;invalid=false;listSignature='';header=null;frame=null;frozen=null;frames.clear();overlays.clear();$('weld-reference-settings').hidden=true;$('weld-reference-override').checked=false;for(const axis of ['x','y','z'])$('weld-reference-'+axis).value='';},
    workflowInput(){
      const d=derived(),settings=ProcessSettings.stateView();
      const targetReady=!invalid&&Boolean(status?.identity?.target_revision&&status?.identity?.generation&&status?.seams?.length);
      const selection=status?.selection||[];
      const jobReady=targetReady&&selection.length>0&&selection.every(item=>status.seams.some(s=>s.seam_id===item.seam_id));
      const poseReady=jobReady&&selection.every(item=>status.seams.find(s=>s.seam_id===item.seam_id)?.pose_ready===true);
      const applied=settings.applied&&settings.server_session_id===status?.server_session_id&&settings.revision===status?.identity?.settings_revision;
      const busy=Boolean(pending||settings.pending||status?.state==='PLANNING');
      return {mode:processMode,stage:workflowMode,connected:rosConnected,modePending:processModePending,
        planningOnly:false,running:d.running,stopping:stopRequested||status?.state==='STOPPING',pending:busy,
        targetReady,jobReady,settingsApplied:applied,canPlan:jobReady&&poseReady&&applied,
        canRun:d.ready,generated:d.planValidated,validated:d.planValidated,dirty:invalid||settings.dirty,
        error:jobReady&&!poseReady?'선택한 심의 기준 자세를 설정하고 적용하세요.':!targetReady?(status?.error||''):'',blockers:[]};
    },
    derived,
    render(){
      if(processMode!=='weld'){listSignature='';return derived();}
      $('weld-reference-settings').hidden=status?.provider!=='external';
      ProcessSettings.setMode('weld');
      const d=derived(),blocked=!rosConnected||processModePending||d.running||Boolean(pending)||status?.state==='PLANNING'||stopRequested;
      $('weld-reference-override').disabled=blocked;
      for(const axis of ['x','y','z'])$('weld-reference-'+axis).disabled=blocked||!$('weld-reference-override').checked;
      ProcessSettings.setRunning(d.running||status?.state==='PLANNING'||Boolean(pending&&pending.topic!=='/process_settings/request')||stopRequested);
      $('btn-fill-work-area').hidden=true;$('weld-selection-summary').hidden=workflowMode!=='path';
      $('weld-selection-summary').textContent=(status?.selection||[]).map((s,i)=>`${i+1}. ${s.seam_id} · ${s.reverse?'역방향':'정방향'}`).join(' → ');
      $('btn-set-target').textContent=status?.provider==='external'?'지정 용접선 3D 구성':'시험 용접심 불러오기';$('btn-set-target').disabled=blocked||!currentStrokes().length||!frame||!header;
      $('btn-refine-planes').hidden=true;$('active-plane-field').hidden=true;$('tare-confirmation').hidden=true;
      $('btn-fill-work-area').textContent='용접 경로 검증';$('btn-fill-work-area').disabled=blocked||invalid||!status?.selection?.length||!ProcessSettings.isApplied();
      $('btn-execute').hidden=true;$('btn-run-robot').disabled=!d.ready;$('btn-stop-robot').disabled=!rosConnected;$('btn-stop-robot').hidden=false;
      $('painting-surface-label').textContent='3D 용접심';$('painting-plan-label').textContent='토치·위빙 경로';
      setPill('painting-d405-state',d.surfaceAccepted?'수신됨':'대기',d.surfaceAccepted?'good':'unknown');
      setPill('painting-plan-validation',status?.state==='PLANNING'?'검증 중':d.planValidated?'완료':'대기',d.planValidated?'good':'unknown');
      $('process-mode-state').textContent=status?.provider==='external'?'실제 카메라 용접선 · Isaac 이동 시험 · 공정 출력 없음':'Isaac 용접 이동 시험 · 용접심 중심 경로와 자세 유지';
      $('planes-status').textContent=(status?.provider==='external'?'추출된 용접심':'시험 작업물에 등록된 용접심')+' · 선택 순서로 실행';
      $('workflow-hint').textContent=({target:status?.provider==='external'?'실제 ZED 영상에 직선·자유선으로 용접선을 그리세요. 그리기 시작 시 영상을 고정합니다. 30초 안에 요청하세요. 지우면 새 영상을 선택할 수 있습니다.':'ZED 영상에서 작업대상을 둘러 표시하고 용접심을 요청하세요.',work_area:'용접심을 선택하고 실행 순서와 진행 방향을 지정하세요.',path:'Settings를 적용한 뒤 계획 생성·검증을 요청하세요.'})[workflowMode];
      $('execution-summary').textContent=status?.error||({EMPTY:'대상을 지정하세요.',TARGET_READY:'용접심을 기다립니다.',SEAMS_READY:'용접심을 선택하세요.',SELECTED:'Settings 적용 후 경로를 검증하세요.',PLANNING:'접근·위빙·이탈 경로 검증 중',READY:'검증 완료 · 작업 시작 가능',RUNNING:'용접 경로 이동 중',STOPPING:'중단 확인 중',STOPPED:'작업 중단됨',COMPLETED:'선택한 용접심 이동 완료'}[status?.state]||'용접 실행기 연결 대기');
      if(status?.state==='RUNNING')$('execution-summary').textContent=`${status.current_seam_id} · ${({APPROACH:'접근',ENTRY:'진입',WELD:'용접·위빙',RETRACT:'이탈',TRANSFER:'다음 심으로 이동'})[status.current_phase]||status.current_phase}`;
      $('painting-blockers').textContent=status?.error||'';
      const key=JSON.stringify([status?.seams,status?.selection,blocked]);
      if(key!==listSignature){listSignature=key;$('weld-seam-list').replaceChildren();
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
          }$('weld-seam-list').append(row);
        }
      }
      const drawingBlocked=blocked||workflowMode!=='target';

      for(const radio of document.querySelectorAll('input[name="sketch-mode"]'))radio.disabled=drawingBlocked||(status?.provider==='external'&&radio.value==='rect');
      $('btn-clear').disabled=drawingBlocked||!currentStrokes().length;
      $('btn-undo').disabled=drawingBlocked||!currentStrokes().length;
      Workflow.render(Workflow.derive(api.workflowInput()));
      return d;
    }
  };
  root.WeldWorkflow=api;
  for(const name of ['override','x','y','z']){
    // Editing orientation retires local approval; only a new projected target can
    // establish a new reference plane. It never silently changes a ready plan.
    if(typeof $==='function'&&$('weld-reference-'+name))$('weld-reference-'+name).addEventListener('change',()=>{invalid=true;api.render();});
  }
  new ROSLIB.Topic({ros,name:'/welding/status',messageType:'std_msgs/String'}).subscribe(msg=>{try{api.setStatus(JSON.parse(msg.data));}catch(e){logEvent('용접 상태 오류: '+e.message);}});
  new ROSLIB.Topic({ros,name:'/welding/seam_overlay',messageType:'std_msgs/String'}).subscribe(msg=>{try{api.setOverlay(JSON.parse(msg.data));}catch(_){}});
  new ROSLIB.Topic({ros,name:'/perception/zed_frame_identity',messageType:'std_msgs/String'}).subscribe(msg=>{try{frame=JSON.parse(msg.data);frames.set(frameKey(frame),frame);while(frames.size>30)frames.delete(frames.keys().next().value);api.render();}catch(_){frame=null;}});
  ros.on('close',()=>{status=null;header=null;frame=null;frozen=null;frames.clear();overlays.clear();invalid=true;pending=null;api.render();if(typeof redrawSketch==='function')redrawSketch();});
})(window);
