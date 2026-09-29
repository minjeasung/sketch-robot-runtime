(function attachZedSurfaceGate(root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.ZedSurfaceGate = api;
})(typeof globalThis !== "undefined" ? globalThis : this, function makeGate() {
  "use strict";

  const id = value => typeof value === "string" ? value.trim() : "";
  const vector = (value, length) => Array.isArray(value) && value.length === length && value.every(Number.isFinite);
  const quad = value => Array.isArray(value) && value.length === 4 && value.every(point => vector(point, 3));
  function stampPathId(stamp) {
    if (!stamp || !Number.isSafeInteger(stamp.sec) || !Number.isInteger(stamp.nanosec) ||
        stamp.sec < 0 || stamp.nanosec < 0 || stamp.nanosec >= 1e9) return "";
    const ns = BigInt(stamp.sec) * 1000000000n + BigInt(stamp.nanosec);
    return ns > 0n ? String(ns) : "";
  }
  function locked(payload) {
    return Boolean(payload && payload.source === "zed" && payload.accepted === true &&
      payload.state === "locked" && id(payload.plane_generation_id).startsWith("zed:"));
  }
  function validTargetSurface(payload) {
    return locked(payload) && id(payload.frame_id) && vector(payload.position, 3) &&
      vector(payload.orientation, 4) && Math.abs(Math.hypot(...payload.orientation) - 1) < 0.01 &&
      quad(payload.corners) && Boolean(stampPathId(payload.target_stamp));
  }
  function validSurface(payload) {
    return validTargetSurface(payload) && quad(payload.front_extent) &&
      Number.isInteger(payload.view_width) && payload.view_width > 1 &&
      Number.isInteger(payload.view_height) && payload.view_height > 1 &&
      Boolean(stampPathId(payload.target_stamp));
  }

  class ZedSurfaceGate {
    constructor() {
      this.mode = "paint";
      this.catalog = "";
      this.retired = new Set();
      this.reset();
    }
    reset() {
      if (this.lock) this.retired.add(this.lock.plane_generation_id);
      this.lock = null;
      this.target = null;
      this.pendingSurface = null;
      this.selected = [];
      this.clearWorkArea();
    }
    clearWorkArea() {
      this.selectionId = "";
      this.surface = null;
    }
    setMode(mode) {
      if (mode !== this.mode) this.reset();
      this.mode = mode;
    }
    setCatalog(generation) {
      if (generation !== this.catalog) this.reset();
      this.catalog = id(generation);
    }
    selectPlanes(generation, ids) {
      this.reset();
      if (this.mode !== "spray" || !this.catalog || generation !== this.catalog ||
          !Array.isArray(ids) || !ids.length || !ids.every(value => id(value))) return false;
      this.selected = [...ids];
      return true;
    }
    beginWorkArea(stamp) {
      this.clearWorkArea();
      if (this.mode !== "spray" || !this.target || !this.lock) return false;
      this.selectionId = stampPathId(stamp);
      return Boolean(this.selectionId);
    }
    receiveTarget(payload) {
      if (this.mode !== "spray" || !payload || payload.source !== "zed") return "ignore";
      const generation = id(payload.plane_generation_id);
      if (generation && this.retired.has(generation)) return "ignore";
      if (payload.accepted !== true || payload.state !== "locked") {
        if (generation && (!this.lock || generation !== this.lock.plane_generation_id)) return "ignore";
        if (this.lock) this.retired.add(this.lock.plane_generation_id);
        this.lock = null;
        this.target = null;
        this.pendingSurface = null;
        this.clearWorkArea();
        return "invalidated";
      }
      if (!locked(payload) || payload.catalog_generation !== this.catalog ||
          generation !== `zed:${this.catalog}:${payload.plane_id}:${stampPathId(payload.stamp)}` ||
          !this.selected.includes(payload.plane_id) || !id(payload.frame_id) ||
          !vector(payload.center, 3) || !vector(payload.normal, 3) ||
          Math.abs(Math.hypot(...payload.normal) - 1) > 0.01 || !quad(payload.corners) ||
          !stampPathId(payload.stamp)) return "ignore";
      if (this.lock && generation !== this.lock.plane_generation_id) {
        this.retired.add(this.lock.plane_generation_id);
        this.target = null;
        this.clearWorkArea();
      }
      this.lock = payload;
      const pending = this.pendingSurface;
      this.pendingSurface = null;
      if (pending) this.receiveSurface(pending);
      return "locked";
    }
    receiveSurface(payload) {
      if (this.mode !== "spray" || !payload || payload.source !== "zed") return "ignore";
      if (!this.lock) {
        if (this.selected.length && payload.mode === "target" && locked(payload) &&
            !this.retired.has(payload.plane_generation_id)) this.pendingSurface = payload;
        return "ignore";
      }
      const generation = id(payload.plane_generation_id);
      if (generation && generation !== this.lock.plane_generation_id) return "ignore";
      if (!generation && payload.accepted === true) return "ignore";
      if (!["target", "work_area"].includes(payload.mode)) return "ignore";
      if (payload.mode === "work_area" && payload.accepted === false && !id(payload.selection_id)) {
        this.clearWorkArea();
        return "invalidated";
      }
      if (payload.mode === "work_area" &&
          (!this.selectionId || payload.selection_id !== this.selectionId)) return "ignore";
      if (payload.accepted === true && (
        payload.frame_id !== this.lock.frame_id ||
        stampPathId(payload.target_stamp) !== stampPathId(this.lock.stamp)
      )) return "ignore";
      const geometryValid = payload.mode === "target" ? validTargetSurface(payload) : validSurface(payload);
      if (!geometryValid || (payload.mode === "work_area" && !id(payload.work_area_id))) {
        // The projector invalidates its previous target-only record before
        // accepting the first rectangle. Preserve that outstanding request.
        if (payload.mode === "target" && this.selectionId && !this.surface && payload.accepted === false) {
          return "invalidated";
        }
        this.clearWorkArea();
        if (payload.mode === "target") {
          this.target = null;
        }
        return "invalidated";
      }
      if (payload.mode === "target") this.target = payload;
      else this.surface = payload;
      return payload.mode;
    }
  }

  function matchesPlan(surface, plan, readiness, mode) {
    if (!surface || !plan || !readiness || !id(plan.path_id) || !id(plan.plan_hash) ||
        !id(surface.work_area_id) || !id(surface.plane_generation_id)) return false;
    if (mode === "spray" && (!validSurface(surface) || surface.mode !== "work_area" ||
        plan.process_mode !== "spray" || readiness.process_mode !== "spray")) return false;
    return plan.work_area_id === surface.work_area_id && readiness.work_area_id === surface.work_area_id &&
      plan.plane_generation_id === surface.plane_generation_id && readiness.plane_generation_id === surface.plane_generation_id &&
      plan.path_id === readiness.path_id && plan.plan_hash === readiness.plan_hash;
  }
  function rectangleReady(strokes, width, height) {
    if (!Array.isArray(strokes) || strokes.length !== 1 || strokes[0].type !== "rect") return false;
    const points = strokes[0].points;
    if (!Array.isArray(points) || points.length !== 4 || !points.every(point =>
      Number.isFinite(point.u) && Number.isFinite(point.v) &&
      point.u >= 0 && point.v >= 0 && point.u < width && point.v < height)) return false;
    const [a, b, c, d] = points;
    return b.u - a.u > 3 && d.v - a.v > 3 && a.v === b.v && b.u === c.u && c.v === d.v && d.u === a.u;
  }
  return { ZedSurfaceGate, stampPathId, matchesPlan, rectangleReady };
});
