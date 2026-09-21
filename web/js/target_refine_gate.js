(function attachTargetRefineGate(root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.TargetRefineGate = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function makeGate() {
  "use strict";

  const SUCCESS_STATES = new Set(["done", "accepted"]);
  const FAILURE_STATES = new Set([
    "failed", "rejected", "timeout", "busy", "aborted", "abort_latched",
  ]);

  function normalizedState(value) {
    return String(value == null ? "" : value).trim().toLowerCase();
  }

  function stampKey(stamp) {
    if (!stamp || typeof stamp !== "object") return "";
    const sec = Number(stamp.sec);
    const nanosec = Number(stamp.nanosec);
    if (
      !Number.isInteger(sec) || !Number.isInteger(nanosec) ||
      sec < 0 || nanosec < 0 || nanosec >= 1_000_000_000 ||
      (sec === 0 && nanosec === 0)
    ) return "";
    return `${sec}:${nanosec}`;
  }

  function begin(stamp, nowMs, timeoutMs) {
    const key = stampKey(stamp);
    if (!key) throw new Error("target refine request stamp must be non-zero");
    const startedMs = Number(nowMs);
    const durationMs = Math.max(0, Number(timeoutMs));
    return {
      stampKey: key,
      targetId: "",
      startedMs,
      deadlineMs: startedMs + durationMs,
    };
  }

  function assess(pending, payload, nowMs) {
    if (!pending) return { action: "ignore", reason: "no_pending", pending: null };
    if (Number(nowMs) > Number(pending.deadlineMs)) {
      return { action: "failure", reason: "ui_timeout", pending };
    }
    const incomingStampKey = stampKey(payload && payload.target_stamp);
    if (!incomingStampKey || incomingStampKey !== pending.stampKey) {
      return { action: "ignore", reason: "target_stamp_mismatch", pending };
    }

    const incomingTargetId = String(
      payload && payload.target_id ? payload.target_id : "",
    ).trim();
    if (
      pending.targetId && incomingTargetId &&
      pending.targetId !== incomingTargetId
    ) {
      return { action: "ignore", reason: "target_id_mismatch", pending };
    }
    const nextPending = {
      ...pending,
      targetId: pending.targetId || incomingTargetId,
    };
    const state = normalizedState(payload && payload.state);
    if (SUCCESS_STATES.has(state)) {
      return { action: "success", reason: "", pending: nextPending };
    }
    if (FAILURE_STATES.has(state)) {
      return {
        action: "failure",
        reason: String(payload.rejection_reason || state),
        pending: nextPending,
      };
    }
    return { action: "pending", reason: "", pending: nextPending };
  }

  function expired(pending, nowMs) {
    return Boolean(pending) && Number(nowMs) > Number(pending.deadlineMs);
  }

  return { begin, assess, expired, stampKey };
});
