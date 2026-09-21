"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const gate = require("../js/target_refine_gate.js");

const REQUEST_STAMP = { sec: 123, nanosec: 456000000 };

test("only the exact target stamp can complete the active request", () => {
  let pending = gate.begin(REQUEST_STAMP, 1000, 8000);

  const stale = gate.assess(pending, {
    state: "done",
    target_id: "old-target",
    target_stamp: { sec: 122, nanosec: 999000000 },
  }, 1001);
  assert.equal(stale.action, "ignore");
  assert.equal(stale.reason, "target_stamp_mismatch");

  const armed = gate.assess(pending, {
    state: "capture_armed",
    target_id: "current-target",
    target_stamp: REQUEST_STAMP,
  }, 1002);
  assert.equal(armed.action, "pending");
  assert.equal(armed.pending.targetId, "current-target");
  pending = armed.pending;

  const wrongId = gate.assess(pending, {
    state: "done",
    target_id: "different-target",
    target_stamp: REQUEST_STAMP,
  }, 1003);
  assert.equal(wrongId.action, "ignore");
  assert.equal(wrongId.reason, "target_id_mismatch");

  const done = gate.assess(pending, {
    state: "done",
    target_id: "current-target",
    target_stamp: REQUEST_STAMP,
  }, 1004);
  assert.equal(done.action, "success");
});

test("matching stamp remains authoritative when legacy status omits target id", () => {
  const pending = gate.begin(REQUEST_STAMP, 2000, 8000);
  const failed = gate.assess(pending, {
    state: "failed",
    rejection_reason: "inlier_ratio_rejected",
    target_stamp: REQUEST_STAMP,
  }, 2001);
  assert.equal(failed.action, "failure");
  assert.equal(failed.reason, "inlier_ratio_rejected");
});

test("missing identity, duplicate terminal with no request, and timeout fail closed", () => {
  const pending = gate.begin(REQUEST_STAMP, 3000, 10);
  assert.equal(gate.assess(pending, { state: "done" }, 3001).action, "ignore");
  assert.equal(gate.assess(null, {
    state: "done", target_stamp: REQUEST_STAMP,
  }, 3002).action, "ignore");
  assert.equal(gate.assess(pending, {
    state: "capture_armed", target_stamp: REQUEST_STAMP,
  }, 3011).action, "failure");
  assert.equal(gate.expired(pending, 3011), true);
});

test("zero or malformed stamps cannot create a request identity", () => {
  assert.equal(gate.stampKey({ sec: 0, nanosec: 0 }), "");
  assert.equal(gate.stampKey({ sec: 1, nanosec: 1_000_000_000 }), "");
  assert.throws(() => gate.begin({ sec: 0, nanosec: 0 }, 0, 1));
});
