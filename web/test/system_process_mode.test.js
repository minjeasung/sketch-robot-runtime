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
