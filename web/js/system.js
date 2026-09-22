"use strict";
const byId = id => document.getElementById(id);
let pending = false, latest = null, settingsLoaded = false, token = "";
const labels = {STOPPED: "종료", STARTING: "기동 중", RUNNING: "실행 중", STOPPING: "종료 중", FAILED: "오류", EXITED: "종료됨"};
const processLabels = {robot_control: "robot control"};
const processLabel = name => processLabels[name] || name;
function options() {
  return {profile: byId("profile").value, robot_ip: byId("robot-ip").value.trim(), model_id: byId("model-id").value,
    launch_zed_driver: byId("zed").checked, launch_d405_driver: byId("d405").checked,
    camera_backend: byId('camera-backend').value, outpost_http: byId('outpost-http').value.trim(),
    outpost_zed_hw_id: byId('outpost-zed-hw-id').value.trim(), outpost_zed_serial: byId('outpost-zed-serial').value.trim(),
    outpost_d405_hw_id: byId('outpost-d405-hw-id').value.trim(), outpost_d405_serial: byId('outpost-d405-serial').value.trim(),
    launch_rviz: byId("rviz").checked};
}
async function api(path, method = "GET", body) {
  const headers = token ? {Authorization: `Bearer ${token}`} : {};
  if (body !== undefined) headers["Content-Type"] = "application/json";
  const res = await fetch(path, {method, headers, ...(body === undefined ? {} : {body: JSON.stringify(body)})});
  const payload = await res.json();
  if (!res.ok) throw new Error(typeof payload.detail === "string" ? payload.detail : JSON.stringify(payload.detail));
  return payload;
}
function profileHelp() {
  byId("profile-help").textContent = {
    dry_run: "실제 로봇·카메라에 연결하지만 작업 경로와 힘 출력은 모의 실행합니다.",
    work: "실제 이동과 도장 힘 제어를 허용합니다. 작업 실행은 스케치 화면에서 진행합니다.",
    spray_motion_test: "현재 EOAT로 실제 이동합니다. 뿜칠건 미장착 전용이며 분사 출력은 항상 OFF, 힘 제어는 사용하지 않습니다.",
    fake: "가상 로봇 하드웨어를 사용합니다. 카메라는 선택한 경우에만 켭니다."
  }[byId("profile").value];
}
function controls() {
  const active = latest && latest.processes.some(p => p.pid !== null);
  byId("start").disabled = pending || !latest || latest.system_prepared;
  byId("stop").disabled = pending || !latest || (!active && !latest.degraded);
  for (const id of ["profile", "robot-ip", "model-id", "zed", "d405", "rviz", 'camera-backend',
    'outpost-http', 'outpost-zed-hw-id', 'outpost-zed-serial', 'outpost-d405-hw-id', 'outpost-d405-serial',
    'outpost-load', 'outpost-zed-select', 'outpost-d405-select']) byId(id).disabled = pending || active;
  if (byId('camera-backend').value === 'outpost') {
    for (const id of ['zed', 'd405']) { byId(id).checked = false; byId(id).disabled = true; }
  }
  byId('outpost-settings').hidden = byId('camera-backend').value !== 'outpost';
  for (const button of document.querySelectorAll(".process-row button")) button.disabled = pending || !latest;
}
function renderProcesses(state) {
  const box = byId("processes"); box.replaceChildren();
  for (const p of state.processes) {
    const row = document.createElement("div"); row.className = "process-row";
    const text = document.createElement("span");
    text.textContent = `${processLabel(p.name)} · ${labels[p.state] || p.state}${p.pid ? ` · PID ${p.pid}` : ""}${p.error ? ` · ${p.error}` : ""}`;
    row.append(text);
    for (const [action, title] of [["start", "시작"], ["stop", "종료"], ["restart", "재시작"]]) {
      const button = document.createElement("button"); button.textContent = title;
      button.addEventListener("click", () => perform(async () => {
        if (action === "start" && !latest.processes.some(item => item.pid !== null)) await api("/configuration", "POST", options());
        await api(`/processes/${encodeURIComponent(p.name)}/${action}${action === "start" ? "" : "?cascade=true"}`, "POST");
      }));
      row.append(button);
    }
    box.append(row);
  }
  const select = byId("log-process");
  if (!select.options.length) for (const p of state.processes) select.add(new Option(processLabel(p.name), p.name));
}
async function refresh() {
  try {
    const state = await api("/status"); latest = state;
    if (!settingsLoaded) {
      const c = state.configuration;
      byId("profile").value = c.profile; byId("robot-ip").value = c.robot_ip;
      byId("model-id").value = c.model_id || "rb10_1300e_u";
      byId('camera-backend').value = c.camera_backend || 'native';
      for (const key of ['outpost_http', 'outpost_zed_hw_id', 'outpost_zed_serial', 'outpost_d405_hw_id', 'outpost_d405_serial']) {
        byId(key.replaceAll('_', '-')).value = c[key] || (key === 'outpost_http' ? 'http://127.0.0.1:8100' : '');
      }
      byId("zed").checked = c.launch_zed_driver; byId("d405").checked = c.launch_d405_driver; byId("rviz").checked = c.launch_rviz;
      settingsLoaded = true; profileHelp();
    }
    byId("state").textContent = state.degraded ? "프로세스 오류" : state.system_prepared ? "전체 프로세스 실행 중" : "API 연결됨";
    byId("domain").textContent = `ROS domain ${state.ros_domain_id}`;
    byId("detail").textContent = `로봇: ${state.configuration.model_id || "rb10_1300e_u"} · 서버 실행 모드: ${state.configuration.profile} · ${state.processes.filter(p => p.state === "RUNNING").length}/${state.processes.length} 실행 중`;
    byId("readiness").textContent = state.ros.readiness ? JSON.stringify(state.ros.readiness) : "최신 ROS 작업 준비 상태를 기다리는 중입니다.";
    byId("status-json").textContent = JSON.stringify(state, null, 2);
    renderProcesses(state);
    const logs = await api(`/processes/${encodeURIComponent(byId("log-process").value)}/logs?lines=150`);
    const box = byId("logs"), follow = box.scrollTop + box.clientHeight >= box.scrollHeight - 40;
    box.textContent = logs.lines.join("\n") || "아직 실행 로그가 없습니다.";
    if (follow) box.scrollTop = box.scrollHeight;
    controls();
  } catch (error) {
    latest = null; byId("state").textContent = "API 연결 확인 필요";
    byId("error").textContent = error.message; controls();
  }
}
async function perform(operation) {
  if (pending) return;
  pending = true; controls(); byId("error").textContent = "";
  try { await operation(); } catch (error) { byId("error").textContent = error.message; }
  finally { pending = false; await refresh(); }
}
byId("connect").addEventListener("click", () => { token = byId("token").value.trim(); byId("error").textContent = ""; refresh(); });
byId("start").addEventListener("click", () => perform(async () => {
  if (!latest.processes.some(p => p.pid !== null)) await api("/configuration", "POST", options());
  await api("/prepare-system", "POST");
}));
byId("stop").addEventListener("click", () => perform(() => api("/shutdown-system", "POST")));
byId("profile").addEventListener("change", () => {
  for (const id of ["zed", "d405", "rviz"]) byId(id).checked = byId("profile").value !== "fake";
  profileHelp();
  controls();
});
byId('camera-backend').addEventListener('change', controls);
byId('outpost-load').addEventListener('click', () => perform(async () => {
  await api('/configuration', 'POST', options());
  const result = await api('/outpost/cameras');
  for (const [name, kind] of [['zed', 'zed'], ['d405', 'realsense']]) {
    const select = byId(`outpost-${name}-select`);
    select.replaceChildren(new Option('보정한 카메라를 선택하세요', ''));
    for (const camera of result.cameras.filter(c => c.camera_type === kind)) {
      const item = new Option(`${camera.camera_id} · ${camera.state}`, camera.hw_id);
      item.dataset.serial = camera.camera_id; select.add(item);
    }
    if (select.options.length === 1) select.options[0].text = '연결된 카메라 없음 · Michelo에서 연결하세요';
  }
}));
for (const name of ['zed', 'd405']) byId(`outpost-${name}-select`).addEventListener('change', event => {
  const item = event.target.selectedOptions[0];
  if (!item.value) return;
  byId(`outpost-${name}-hw-id`).value = item.value;
  byId(`outpost-${name}-serial`).value = item.dataset.serial;
});
const micheloUrl = new URL(location.href); micheloUrl.port = '8101'; micheloUrl.pathname = '/console/'; micheloUrl.search = ''; micheloUrl.hash = '';
byId('michelo-link').href = micheloUrl.href;
async function poll() { await refresh(); setTimeout(poll, 1500); }
poll();
