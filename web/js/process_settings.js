/* Shared panel; server snapshots and unapplied per-process drafts stay separate. */
(function(root){
  const states={}; let mode='paint',running=false,signature='';
  function state(){return states[mode] ||= {snapshot:null,draft:{},dirty:false,pending:null};}
  const api={
    setMode(value){if(mode!==value){mode=value;signature='';}render();},
    setRunning(value){running=Boolean(value);render();},
    values(){return {...state().draft};},
    isApplied(){const s=state();return Boolean(s.snapshot?.applied&&!s.dirty&&!s.pending);},
    edit(key,value){const s=state();s.draft[key]=value;s.dirty=true;render();if(root.WeldWorkflow)root.WeldWorkflow.render();},
    setSnapshot(snapshot){
      if(!snapshot||!snapshot.fields||!snapshot.server_session_id)return;
      const s=states[snapshot.mode] ||= {snapshot:null,draft:{},dirty:false,pending:null};
      const changed=s.snapshot?.server_session_id!==snapshot.server_session_id;
      if(s.snapshot&&!changed&&snapshot.revision<s.snapshot.revision)return;
      if(changed){s.dirty=false;s.pending=null;s.draft={};signature='';}
      if(s.pending&&snapshot.revision>s.pending.revision&&Object.entries(s.pending.values).every(([k,v])=>snapshot.fields[k]?.value===v)){
        s.pending=null;s.dirty=false;
      }
      s.snapshot=snapshot;
      if(!s.dirty&&!s.pending)s.draft=Object.fromEntries(Object.entries(snapshot.fields).map(([k,v])=>[k,v.value]));
      render();
    },
    reject(){const s=state();s.pending=null;s.dirty=true;render();},
    disconnected(){for(const s of Object.values(states)){s.pending=null;s.dirty=true;}render();},
    apply(){
      const s=state();if(mode!=='weld'||!rosConnected||running||!s.snapshot||s.pending)return;
      if(Object.entries(s.draft).some(([k,v])=>s.snapshot.fields[k]?.options?
        !s.snapshot.fields[k].options.some(o=>o.value===v):typeof v!=='boolean'&&!Number.isFinite(v)))return;
      s.pending={revision:s.snapshot.revision,values:{...s.draft}};
      if(!root.WeldWorkflow.command('/process_settings/request',{values:s.draft}))s.pending=null;
      render();
    }
  };
  function render(){
    const s=state(),snapshot=s.snapshot,fields=snapshot?.fields||{};
    const key=mode+':'+Object.keys(fields).join(',')+':'+snapshot?.server_session_id;
    if(signature!==key){
      signature=key; $('settings-fields').replaceChildren();
      for(const [name,f] of Object.entries(fields)){
        const label=document.createElement('label');label.className='settings-field';
        const title=document.createElement('span');title.textContent=f.label+(f.unit?' ('+f.unit+')':'');
        const input=document.createElement(f.options?'select':'input');
        if(f.options){for(const o of f.options){const option=document.createElement('option');option.value=o.value;option.textContent=o.label;input.append(option);}}
        else{input.type=typeof f.value==='boolean'?'checkbox':'number';input.step='any';}
        input.dataset.setting=name;input.setAttribute('aria-label',title.textContent);
        input.addEventListener('input',()=>api.edit(name,f.options?input.value:input.type==='checkbox'?input.checked:Number(input.value)/(f.scale||1)));
        label.append(title,input);$('settings-fields').append(label);
      }
    }
    for(const row of $('settings-fields').children){
      const input=row.children[1],name=input.dataset.setting,f=fields[name];
      if(f.options)input.value=s.draft[name]||'';
      else if(input.type==='checkbox')input.checked=Boolean(s.draft[name]);
      else if(document.activeElement!==input)input.value=s.draft[name]==null?'':String(s.draft[name]*(f.scale||1));
      input.disabled=!f.editable||running||!rosConnected||Boolean(s.pending);
      if(!f.editable&&s.draft[name]==null)input.placeholder='서버 값 대기';
    }
    $('settings-apply').hidden=mode!=='weld';
    $('settings-apply').disabled=!rosConnected||running||!snapshot||Boolean(s.pending);
    $('settings-state').textContent=!snapshot?'서버 설정을 기다립니다.':s.pending?'설정 적용 확인 중':mode!=='weld'?'현재 설정 · 변경은 시스템 실행 설정에서 적용합니다.':s.dirty?'변경됨 · 적용 후 경로를 다시 검증하세요.':api.isApplied()?'설정 적용됨':'시험 설정 · 적용 버튼을 누르세요.';
  }
  root.ProcessSettings=api;
  $('settings-apply').addEventListener('click',()=>api.apply());
  new ROSLIB.Topic({ros,name:'/process_settings/status',messageType:'std_msgs/String'}).subscribe(msg=>{try{api.setSnapshot(JSON.parse(msg.data));}catch(e){logEvent('설정 수신 오류: '+e.message);}});
  ros.on('close',()=>api.disconnected());
})(window);
