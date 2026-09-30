"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const { loadUI } = require("./ui_harness.js");

async function supervisor(configuration = {}) {
  const requests = [];
  const state = {
    configuration: { profile: "dry_run", process_mode: "paint", robot_ip: "10.0.2.7", model_id: "rb10_1300e_u",
      camera_backend: "native", launch_zed_driver: false, launch_d405_driver: false, launch_rviz: false,
      ...configuration },
    processes: [{ name: "perception", pid: null, state: "STOPPED" }], ros: {}, ros_domain_id: 0,
    degraded: false, system_prepared: false,
  };
  const ui = loadUI("system.html", async (url, options) => {
    requests.push({ url, ...options });
    const payload = url === "/status" ? state : url.includes("/logs?") ? { lines: [] } : {};
    return { ok: true, json: async () => payload };
  });
  await new Promise(setImmediate);
  return { ui, requests, state };
}

test("camera-less startup sends the selected spray mode to the supervisor", async () => {
  const { ui, requests } = await supervisor();
  ui.element("process-mode").value = "spray";
  await ui.element("process-mode").fire("change");
  await ui.element("start").fire("click");
  const config = JSON.parse(requests.find(request => request.url === "/configuration").body);
  assert.equal(config.process_mode, "spray");
  assert.equal(config.profile, "dry_run");
  assert.equal(config.launch_zed_driver, false);
  assert.equal(config.launch_d405_driver, false);
  assert.equal(requests.some(request => request.url === "/prepare-system"), true);
});

test("EOAT profile survives reload and unrelated save, and locks while active", async () => {
  const path = "/workspace/profiles/nozzle outlet.json";
  const { ui, requests, state } = await supervisor({ spray_eoat_profile: path });
  assert.equal(ui.element("spray-eoat-profile").value, path);
  await ui.evaluate("refresh()");
  await ui.element("start").fire("click");
  assert.equal(JSON.parse(requests.find(r => r.url === "/configuration").body).spray_eoat_profile, path);
  state.processes[0].pid = 123;
  await ui.evaluate("refresh()");
  assert.equal(ui.element("spray-eoat-profile").disabled, true);
});

test("EOAT edits survive polling and can be explicitly cleared", async () => {
  const { ui, requests } = await supervisor({ spray_eoat_profile: "/old.json" });
  ui.element("spray-eoat-profile").value = "profiles/new nozzle.json";
  await ui.evaluate("refresh()");
  assert.equal(ui.element("spray-eoat-profile").value, "profiles/new nozzle.json");
  await ui.element("start").fire("click");
  assert.equal(JSON.parse(requests.find(r => r.url === "/configuration").body).spray_eoat_profile,
    "profiles/new nozzle.json");
  ui.element("spray-eoat-profile").value = "";
  await ui.element("start").fire("click");
  assert.equal(JSON.parse(requests.filter(r => r.url === "/configuration").at(-1).body).spray_eoat_profile, "");
});

test("startup reflects saved mode and locks the spray motion test profile to spray", async () => {
  const { ui, requests } = await supervisor({ process_mode: "spray" });
  assert.equal(ui.element("process-mode").value, "spray");
  ui.element("process-mode").value = "paint";
  ui.element("profile").value = "spray_motion_test";
  await ui.element("profile").fire("change");
  assert.equal(ui.element("process-mode").value, "spray");
  assert.equal(ui.element("process-mode").disabled, true);
  assert.equal(ui.element("d405").checked, false);
  await ui.element("start").fire("click");
  assert.equal(JSON.parse(requests.find(request => request.url === "/configuration").body).process_mode, "spray");
});

test("paint remains the startup default and active processes lock the menu", async () => {
  const { ui, state } = await supervisor({ process_mode: undefined });
  assert.equal(ui.element("process-mode").value, "paint");
  state.processes[0].pid = 123;
  await ui.evaluate("refresh()");
  assert.equal(ui.element("process-mode").disabled, true);
});

test("ZED preview start config excludes native drivers and robot controls", async () => {
  const { ui, requests } = await supervisor();
  ui.element("profile").value = "zed_preview";
  await ui.element("profile").fire("change");
  assert.equal(ui.element("process-mode").value, "spray");
  assert.equal(ui.element("process-mode").disabled, true);
  assert.equal(ui.element("robot-ip").disabled, true);
  assert.equal(ui.element("camera-backend").value, "outpost");
  for (const id of ["zed", "d405", "rviz"]) {
    assert.equal(ui.element(id).disabled, true);
    assert.equal(ui.element(id).checked, false);
  }
  await ui.element("start").fire("click");
  const config = JSON.parse(requests.find(request => request.url === "/configuration").body);
  assert.equal(config.profile, "zed_preview");
  assert.equal(config.camera_backend, "outpost");
  assert.equal(config.launch_rviz, false);
});
