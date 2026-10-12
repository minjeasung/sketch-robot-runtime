# Sketch Robot architecture (main / ROS 2 Jazzy)

A navigation map for coding agents, **not** a verified, exhaustive wiring diagram.
For operation-specific behavior, inspect source, launch configuration and tests.
See `README.md` and `RUN.md` first.

## Major paths

1. **Inputs:** Michelo Outpost supplies ZED 2i and, for Paint, D405 data.
   `src/sketch_control/sketch_control/outpost_bridge_node.py` and related camera
   modules adapt camera information into the runtime. Spray's ZED-only path
   intentionally does not require a D405 scan.
2. **Perception and selection:** `src/sketch_control/sketch_control/` contains
   target/plane, work-area and surface modules; `web/` contains the browser
   sketch workflow. ROS messages and Rosbridge tie those layers together.
3. **Planning:** `sketch_to_waypoints_node.py` and related geometry/process
   modules turn selected regions into paths and process segments.
4. **Execution:** `moveit_executor.py` and process-specific executors interface
   with MoveIt and robot control. `src/rbpodo_ros2/` contains robot descriptions,
   bringup, hardware integration and MoveIt configuration.
5. **Contact control:** `src/rbpodo_painting_control/` contains wrench reference,
   guard, force monitoring, safety and segment helpers.
   `src/admittance_controller/` contains the controller implementation.
6. **Operations:** `process_supervisor.py`, `system_api.py` and
   `michelo_gateway.py` support supervision and UI/API integration.

Conceptual flow:

```text
Michelo Outpost (ZED / D405) -> bridge / ROS -> plane & target selection
                                          <-> browser sketch UI
  -> work area & process path -> MoveIt / execution -> controller / hardware
                                      ^                    ^
                                      |            guarded F/T feedback
                               readiness / safety gates
```

This is a high-level flow, not a statement that every link is used in every mode.

## Top-level locations

| Directory | Ownership / typical change |
| --- | --- |
| `src/sketch_control/` | Perception, sketch-to-path, supervisor, execution integration |
| `src/rbpodo_painting_control/` | Force safety, guarded force reference, process segments |
| `src/admittance_controller/` | Controller behavior — safety-critical |
| `src/rbpodo_ros2/` | Robot hardware, model, MoveIt, bringup — safety-critical |
| `src/eoat_description/` | EOAT URDF/meshes and geometry |
| `web/` | Browser control and sketch workflows |
| `config/` | Defaults and configuration examples |
| `scripts/` | Local setup, checks, version tools |
| `docs/` | Specifications, operating notes and validation reports |

Some ROS packages are bundled third-party dependencies under `src/`. Do not
assume the whole directory is first-party or safe to edit.

## Interfaces and invariants to check

- Distinguish Paint (ZED + D405) from Spray (ZED-only) and planning-only
  `zed_preview`. Consult `docs/MULTI_PLANE_SPRAY.md`.
- Target plane identity, work-area identity and timestamps are used to reject
  stale work. Inspect the corresponding contracts before changing messages.
- Preserve frame and unit conventions. For roller painting, the normal contact
  axis is **TCP Y**, pressing toward **TCP -Y**; see
  `src/rbpodo_painting_control/README.md` and
  `src/rbpodo_ros2/rbpodo_bringup/config/admittance_profiles/painting_normal_y.yaml`.
- The guarded controller wrench pathway and default-disabled real painting
  are safety mechanisms, not tuning conveniences.
- The Michelo Outpost service and independent integration surfaces are
  described in `docs/MICHELO_INTEGRATION.md` and `docs/SYSTEM_API.md`.
  `RUN.md` describes ports 8100 (Outpost), 8101 (bridge) and 8081 (API).
- Plane extraction versions `base` / `v0` / `v1` / `v2` are **snapshots**, not
  automatically a chronological quality ranking. Follow
  `docs/PLANE_EXTRACTION_VERSIONS.md`; do not casually switch algorithms.

## Where to look for regression tests

- Python plane-version control: `scripts/tests/test_plane_versions.py`
- ROS/perception/execution integration: `src/sketch_control/test/`
- Force and segment safety: `src/rbpodo_painting_control/test/`
- Controller C++ tests: `src/admittance_controller/test/`
- Browser behavior: `web/test/*.test.js`

Only the explicitly selected offline-safe subset runs in
`scripts/ci/offline_tests.sh`. Passing it does not mean the system is ready
to operate on hardware.

