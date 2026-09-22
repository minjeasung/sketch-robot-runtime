// Plane catalog and explicit process selection; geometry stays in the backend.
let planeCatalog = {generation: '', planes: []};
let planeState = {};
const selectedPlaneIds = new Set();
const planeColors = ['#38bdf8','#f59e0b','#a78bfa','#4ade80','#fb7185','#22d3ee','#f472b6','#a3e635'];
const selectPlanesPub = new ROSLIB.Topic({ros, name:'/painting_system/select_planes', messageType:'std_msgs/String'});
const activatePlanePub = new ROSLIB.Topic({ros, name:'/painting_system/activate_plane', messageType:'std_msgs/String'});
const processModePub = new ROSLIB.Topic({ros, name:'/painting_system/set_process_mode', messageType:'std_msgs/String'});
let lastPlaneRender = "";
function renderPlaneList() {
  // Keep select menus and checkbox focus intact between unchanged status ticks.
  const signature = JSON.stringify([planeCatalog, planeState, [...selectedPlaneIds], rosConnected, paintingDerivedState().running, processModePending, sprayMotionTest]);
  if (signature === lastPlaneRender) return;
  lastPlaneRender = signature;
  const list = $('plane-candidates'); list.replaceChildren();
  planeCatalog.planes.forEach((plane,index) => {
    const label = document.createElement('label');
    const swatch = document.createElement('span'); swatch.className='plane-swatch'; swatch.style.background=planeColors[index%planeColors.length]; swatch.setAttribute('aria-hidden','true');
    const checkbox=document.createElement('input'); checkbox.type='checkbox'; checkbox.checked=selectedPlaneIds.has(plane.id);
    checkbox.disabled=paintingDerivedState().running;
    checkbox.addEventListener('change',()=>{ if(checkbox.checked) selectedPlaneIds.add(plane.id); else selectedPlaneIds.delete(plane.id); renderPlaneList(); redrawSketch(); });
    label.append(checkbox,swatch,document.createTextNode(`면 ${index+1}`)); list.append(label);
  });
  $('btn-refine-planes').disabled=!rosConnected || paintingDerivedState().running || !selectedPlaneIds.size;
  const select=$('active-plane'); select.replaceChildren();
  const empty=document.createElement('option'); empty.value=''; empty.textContent='작업할 측정 평면 선택'; select.append(empty);
  planeCatalog.planes.filter(p=>(planeState.measured||[]).includes(p.id)).forEach(p=>{
    const option=document.createElement('option'); option.value=p.id;
    option.textContent=`면 ${planeCatalog.planes.indexOf(p)+1}`; select.append(option);
  });
  select.value=planeState.active_id||''; select.disabled=!rosConnected || paintingDerivedState().running || !(planeState.measured||[]).length;
  $('active-plane-field').hidden=!(planeState.measured||[]).length;
  $('process-mode').disabled=!rosConnected || paintingDerivedState().running || processModePending || sprayMotionTest;
}
function drawPlaneCandidates() {
  if(currentView!=='zed_raw') return;
  const sx=sketchCanvas.width/(planeCatalog.image_width||sketchCanvas.width);
  const sy=sketchCanvas.height/(planeCatalog.image_height||sketchCanvas.height);
  planeCatalog.planes.forEach((plane,index)=>{
    const pts=plane.polygon_px||[]; if(pts.length<3) return;
    sketchCtx.save(); sketchCtx.strokeStyle=planeColors[index%planeColors.length];
    sketchCtx.lineWidth=selectedPlaneIds.has(plane.id)?5:2;
    sketchCtx.beginPath(); pts.forEach((p,i)=>{if(i)sketchCtx.lineTo(p[0]*sx,p[1]*sy);else sketchCtx.moveTo(p[0]*sx,p[1]*sy);});
    sketchCtx.closePath(); sketchCtx.stroke(); sketchCtx.globalAlpha=.15;
    sketchCtx.fillStyle=planeColors[index%planeColors.length]; sketchCtx.fill(); sketchCtx.globalAlpha=1;
    sketchCtx.font='bold 22px sans-serif'; sketchCtx.fillText(`면 ${index+1}`,pts[0][0]*sx,pts[0][1]*sy); sketchCtx.restore();
  });
}
new ROSLIB.Topic({ros,name:'/perception/target_planes',messageType:'std_msgs/String'}).subscribe(msg=>{
  let payload; try{payload=JSON.parse(msg.data);}catch{return;}
  if(!Array.isArray(payload.planes)||paintingDerivedState().running)return;
  planeCatalog=payload; planeState={}; selectedPlaneIds.clear();
  paintingState.targetSelectionState='rejected';
  $('planes-status').textContent=payload.error||`${payload.planes.length}개 평면 — 측정할 면 선택`;
  renderPlaneList(); redrawSketch(); refreshPaintingUI();
});
$('btn-refine-planes').addEventListener('click',()=>{
  if(paintingDerivedState().running||!selectedPlaneIds.size)return;
  if(!window.confirm('선택한 평면들을 D405로 측정하기 위해 로봇이 순서대로 접근합니다. 시작할까요?'))return;
  multiPlaneBusy=true; paintingState.targetSelectionState='pending';
  selectPlanesPub.publish(new ROSLIB.Message({data:JSON.stringify({generation:planeCatalog.generation,ids:[...selectedPlaneIds]})}));
  renderPlaneList(); refreshPaintingUI();
});
new ROSLIB.Topic({ros,name:'/painting_system/planes',messageType:'std_msgs/String'}).subscribe(msg=>{
  let payload;try{payload=JSON.parse(msg.data);}catch{return;}
  if(payload.generation!==planeCatalog.generation)return;
  const changed=payload.active_id!==planeState.active_id || planeState.state!=="ready";
  planeState=payload; multiPlaneBusy=payload.running===true;
  $('planes-status').textContent=payload.error||({ordering:'이동이 적은 측정 순서 계산 중',measuring:'D405 접근·측정 중',ready:'측정 완료 — 평면별 작업영역을 그리세요',failed:'측정 실패',candidates:'측정할 면 선택'}[payload.state]||payload.state);
  if(payload.state==='ready'){
    paintingState.targetSelectionState='selected';
    if(changed){strokesMap.work_area=[];strokesMap.path=[];beginD405Refresh('active plane changed');switchToWorkAreaMode();}
  }
  renderPlaneList(); refreshPaintingUI(); redrawSketch();
});
$('active-plane').addEventListener('change',ev=>{
  if(!ev.target.value||paintingDerivedState().running)return;
  if(!window.confirm('선택한 면의 작업영역을 그릴 수 있도록 D405가 다시 접근합니다. 시작할까요?')) {ev.target.value=planeState.active_id||'';return;}
  multiPlaneBusy=true;
  beginD405Refresh('switch selected plane');
  activatePlanePub.publish(new ROSLIB.Message({data:JSON.stringify({generation:planeCatalog.generation,id:ev.target.value})}));
});
$('process-mode').addEventListener('change',ev=>{
  if(paintingDerivedState().running)return;
  processModePending=true; invalidatePlanLocally('process mode changed',false);
  processModePub.publish(new ROSLIB.Message({data:ev.target.value}));
  refreshPaintingUI(); renderPlaneList();
});
new ROSLIB.Topic({ros,name:'/painting_system/process_mode',messageType:'std_msgs/String'}).subscribe(msg=>{
  let payload;try{payload=JSON.parse(msg.data);}catch{return;}
  if(!['paint','spray'].includes(payload.mode))return;
  const changed=processMode!==payload.mode || sprayMotionTest!==(payload.spray_motion_test===true);
  processMode=payload.mode; sprayMotionTest=payload.spray_motion_test===true; processModePending=false;
  $('process-mode').value=processMode;
  $('process-mode-state').textContent=payload.error||(sprayMotionTest
    ? 'EOAT 이동 검증 · 실제 로봇 이동 · 50 cm 이격 · 분사 항상 OFF'
    : processMode==='spray'?'작업면에서 50 cm 이격 · 칠할 때만 분사':'작업면에 접촉하여 도장합니다.');
  if(changed)invalidatePlanLocally('process mode acknowledged',false);
  refreshPaintingUI(); renderPlaneList();
});
ros.on('close',()=>{multiPlaneBusy=false;processModePending=true;renderPlaneList();});
renderPlaneList();
setInterval(renderPlaneList,1000);
