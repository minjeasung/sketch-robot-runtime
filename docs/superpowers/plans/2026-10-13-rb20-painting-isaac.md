# RB20 Painting Isaac Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans for integration and verification; use superpowers:dispatching-parallel-agents for the independent asset/simulator and management tasks. Use test-driven-development for behavioral changes. The user explicitly requested implementation immediately after this plan; no further plan approval is required.

**Goal:** Run the original RB20 roller painting workflow against Isaac physics, camera data and measured six-axis tool reaction in this folder.

**Architecture:** Keep original perception, executor, MoveIt, JTC, admittance and guarded force nodes. A generated RB20 description selects a simulation-only hardware plugin. A dedicated Isaac application publishes session-bound measured state and rendered cameras; the original supervisor manages the complete stack.

**Tech Stack:** ROS 2 Jazzy, Isaac Sim 6.1 / PhysX, Python 3.12, C++17 ros2_control, existing FastAPI/JavaScript UI.

**Spec:** `docs/superpowers/specs/2026-10-13-rb20-painting-isaac-design.md`

## Global Constraints

- Work in `/home/minjea/robotics/sketch-robot-runtime-main` as requested; retain the current feature branch. No push or merge.
- Profile `isaac_rb20_paint`, model `rb20_1900es`, process `paint`; separate ROS domain 106, localhost only.
- Preserve real and Construct profile defaults; load no physical hardware plugin or camera driver in this profile.
- Existing original UI, D405 lifecycle, v3 identity/hash checks, collision checks, force guard, runtime tare and cancellation remain active.
- Sensor readings come from physics/rendering; invalid or old data cannot become synthetic valid zeros.
- Measured joint states, F/T, cameras and `/clock` use simulation timestamps; input/command watchdogs use steady time.
- Start with 500 Hz physics, 100 Hz controller and 15 Hz rendering; record actual performance.
- Existing 20 N command and 20–30 N force-hold band are the final contact test configuration.
- No automatic motion at startup. Original explicit target/work-area measurement may run its D405 prescan; contact/force output requires the original Run workflow. Pause/reset invalidates prior execution.

## Review Focus

- A source reuses old data with a fresh outer timestamp: compare session, advancing step and original sensor validity.
- An interrupted start leaves a stale readiness file: require owned PID, unique session and fresh heartbeat.
- The F/T joint excludes the roller or has been merged: verify the entire downstream tool and named fixed joint in USD before readiness.
- ROS simulation time stops while force output remains requested: separate steady watchdog and simulator command lease.
- The generated model drifts from MoveIt geometry: both use one description and manifest; compare FK and camera transforms.

## Shared interfaces

`isaac_painting_msgs/msg/State`: `std_msgs/Header header`, `string session_id`, `string scene_revision`, `uint64 step_index`, `string[] joint_names`, `float64[] positions`, `float64[] velocities`, `float64[] efforts`, `geometry_msgs/Wrench wrench`, `geometry_msgs/Wrench gravity_wrench`, `bool joints_valid`, `bool wrench_valid`, `bool playing`.

`isaac_painting_msgs/msg/Command`: `std_msgs/Header header`, `string session_id`, `string scene_revision`, `uint64 sequence`, `float64[] positions`, `bool enabled`.

Topics: `/isaac/rb20_painting/state` and `/isaac/rb20_painting/command`. Wrench and gravity wrench are expressed at `ft_link`. Subtract gravity exactly once before tare/filtering. Readiness file `.runtime/rb20_painting/managed-ready.json`: `pid`, `session_id`, `scene_revision`, `model_id`, `step_index`, `updated_monotonic`, `joints_valid`, `wrench_valid`, `cameras_ready`. File is atomically refreshed; supervisor needs values from its own child and age ≤ 2 s. Session survives pause but reset creates a new session and invalidates work.

Generated assets under `.runtime/rb20_painting/assets/`: `robot.urdf`, `robot.srdf`, `robot.usd`, `manifest.json`, `painting.yaml`, `controllers.yaml`, `scene.json`, `calibration/`. Simulator and ROS launcher use these files. Build/install is `.runtime/rb20_painting/{build,install,log}`.

## Task 1: Generate the RB20 model and run the physics/sensor application

**Files:** create `scripts/prepare_rb20_painting_assets.py`, `scripts/import_rb20_painting_usd.py`, `scripts/rb20_painting_app.py`, `scripts/rb20_painting_scene.py`, `scripts/rb20_painting_sensors.py`, `scripts/run_rb20_painting_isaac.sh`, `tests/rb20_painting/test_assets.py`, `tests/rb20_painting/test_sim_contract.py`.

**Consumes:** bundled robot/EOAT xacro, original painting/controller YAML, shared message contract, installed Isaac.
**Produces:** `prepare(root: Path, output: Path) -> dict` manifest, generated assets, owned standalone simulator with shared state/command and readiness contract.

- [x] Write tests for actual generated model: six names, mechanical transform preservation, plugin replacement, no physical IP, roller contact link, downstream F/T boundary, manifest consistency. Test command lease/session rejection with pure helpers.
- [x] Run `python3 -m pytest tests/rb20_painting/test_assets.py tests/rb20_painting/test_sim_contract.py -q` and observe missing behavior.
- [x] Implement asset preparation and USD importer. Preserve fixed sensor joint and use original mesh/collision geometry. Keep generated simulation inertia corrections explicit.
- [x] Implement the standalone app, rendering cameras, true joint/wrench feedback, gravity wrench, monotonic command lease, atomic readiness, Stop/Play invalidation and cleanup. Use installed API docs/source, not guessed joint indices.
- [x] Re-run tests and import/launch Isaac headless. Check six joints, live wrench, sensor boundary, clock and both rendered camera streams; log evidence.

## Task 2: Session-aware ros2_control hardware and tare

**Files:** create `src/isaac_painting_msgs/{msg/State.msg,msg/Command.msg,CMakeLists.txt,package.xml}`, `src/isaac_painting_hardware/{CMakeLists.txt,package.xml,isaac_painting_hardware.xml,include/isaac_painting_hardware/state_contract.hpp,include/isaac_painting_hardware/system.hpp,src/system.cpp,test/test_state_contract.cpp}`, `scripts/build_rb20_painting_sim.sh`, `scripts/rb20_painting_ros_env.sh`.

**Consumes:** shared State/Command messages and readiness identity.
**Produces:** `isaac_painting_hardware/IsaacPaintingSystem`, six measured joint states and position commands, `ft_sensor` and `ft_sensor_raw`, `/rbpodo_ft_tare/runtime_free_space_tare` (Trigger), inhibited/fault reporting.

- [x] Write pure C++ tests for session mismatch, duplicate/rewound step, malformed joint array, NaN, 0.20 s stale data, pause, force compensation and tare eligibility; legitimate zero measured force remains valid.
- [x] Compile/run tests and observe missing behavior; implement contract and ROS plugin with independent nonblocking callbacks/steady watchdog. Tare uses real samples and existing stationary/force-off/compliance-off interlocks.
- [x] Build into the isolated overlay with the repository admittance_controller, RB20 descriptions, perception packages and ROS messages. Do not build or launch physical RBPodo hardware.
- [x] Run C++ tests, load plugin in real controller_manager against disposable state publisher, check measured state propagation and rejection/timeout behavior. Then connect to the actual simulator.

## Task 3: Original ROS launch, camera/force pipeline and lifecycle integration

**Files:** create `src/sketch_control/launch/rb20_painting_isaac.launch.py`, `src/sketch_control/sketch_control/isaac_painting_lifecycle.py`, `scripts/run_rb20_painting_ros.sh`, `src/sketch_control/test/test_rb20_painting_launch.py`; modify `src/sketch_control/setup.py` only to register the lifecycle node if needed.

**Consumes:** generated robot/config/calibration and plugin from Tasks 1–2.
**Produces:** `component:=robot_control|perception|force_pipeline|executor|all` launcher; original `/joint_trajectory_controller/follow_joint_trajectory`, perception topics, strict executor and guarded force chain.

- [x] Test expanded launch configuration: world/link0 identity, generated robot description, both F/T broadcasters, original controller chain, use_sim_time on all relevant nodes, no native hardware driver. Test lifecycle state changes cancel/invalidate rather than auto-resume.
- [x] Run tests RED; implement launch composition reusing original nodes/configuration and a focused lifecycle adapter. Fix explicit simulation-time propagation where needed without changing real defaults.
- [x] Run tests GREEN; use real ROS graph to verify controller active state, F/T validity, transforms and camera/D405 processing. Verify actual TCP FK agrees with Isaac.

## Task 4: Existing supervisor and web management profile

**Files:** modify `src/sketch_control/sketch_control/process_supervisor.py`, `scripts/run_system_api.sh`, `web/system.html`, `web/js/system.js` (resolve existing management JS path before editing); create `src/sketch_control/test/test_rb20_painting_supervisor.py` and extend existing UI tests.

**Consumes:** simulator command `bash scripts/run_rb20_painting_isaac.sh`, ROS command `bash scripts/run_rb20_painting_ros.sh COMPONENT`, readiness contract.
**Produces:** profile `isaac_rb20_paint`, compatible model/process validation, process dependency registry, management option and `--rb20-paint` API entry point.

- [x] Add failing tests: accepted profile fixed to RB20/paint/native input, disabled physical drivers, command/dependency registry, foreign/stale/invalid readiness rejected, startup cancel cleans owned process, simulation reported in status.
- [x] Run supervisor/UI tests RED. Implement isolated profile builder/validation and UI option; retain original sketch interface and existing profile behavior.
- [x] Run new and existing supervisor/API/UI tests GREEN. Ensure API startup shell sources the dedicated overlay/domain before starting ROS monitor.

## Task 5: Real Isaac integration, regression and delivery

**Files:** create `scripts/check_rb20_painting_isaac.py`, `docs/RB20_PAINTING_ISAAC.md`; update `docs/ROBOT_MODELS.md` with a link, plan/ledger status and findings.

**Consumes:** all preceding tasks.
**Produces:** reusable CLI checks, measured report under `logs/rb20-painting/`, reproducible documented launch.

- [x] Add report/acceptance tests rejecting incomplete phase histories, absent feedback, out-of-band force, stale measurements and unacknowledged cancel. Run RED and implement evidence collection.
- [x] Run actual simulator + ROS stack: free-space tare, noncontact trajectory, two separate strokes at the original 20 N request with full phase sequence, and measured final retract. Compare measured feedback to spec tolerances.
- [x] Exercise actual moving-FJT source loss, contact-entry UI Stop, overforce, Pause/Stop/reset and old-plan invalidation. Record detection, disable, cancel acknowledgement and same-goal terminal status separately. Verify foreign-session rejection in the real DDS hardware fixture.
- [x] Inspect original browser workflow against rendered cameras, manual and generated paths, start/stop. Run affected Python/C++/JS regressions and report pre-existing failures separately.
- [x] Perform a fresh independent whole-change review and focused reviews of subsequent fixes; address confirmed important defects with regression tests and document numerical results and physical-model limitations.
- [x] Stop all owned services after verification. Owned API, six managed children and observers exited; ports 8070/9091 closed, readiness marker removed, unrelated port 9090 preserved.
- [x] Commit only scoped files; no push or merge. Final answer links run guide and reports what was actually verified.

**Qualification limits:** The implemented profile and its core measured painting loop are tested here. Separate stationary 20 N loading, camera-only failure, an unreachable path in the live physical stack, UI Stop during an already moving PAINT trajectory, and exact mechanical deceleration time were not individually measured. The whole-process input-loss test freezes physics as well as sensors and must not be presented as a mechanical stop-time test. These remaining design validation cases are explicitly recorded in the run guide rather than counted as passed.

## Execution record

- 2026-10-13: User authorized writing the plan followed immediately by implementation. Plan self-review covers all spec sections; asset/simulator and supervisor edits can proceed independently against the interfaces above. Integration, hardware, builds and GPU execution remain coordinated by the primary agent.

- 2026-10-14: Actual full ROS/Isaac graph passed all nine health checks after a verified loopback-only Fast DDS profile. `LOCALHOST` discovery had overridden the UDP environment knob and reintroduced shared-memory transport; the XML whitelist plus `SYSTEM_DEFAULT` removes that override. The 0.20 s watchdog remains unchanged. See `logs/rb20-painting/transport-diagnosis.json`.
- 2026-10-14: Independent review fixes cover fresh-command receipt timing, source-mesh binding in scene revision, per-stroke ordered tare acceptance, and fresh session-bound status checks. FJT completion now follows simulation time while cancellation and sensor watchdogs retain steady time. The original D405 trajectory completed successfully with measured arrival. Its subsequent capture correctly rejected a view outside safe support: the reused ±0.02 rad planning goal tolerance displaced the requested TCP by 22 mm. Live controller feedback tracked its commanded endpoint within 0.00090 rad, isolating the error to planning precision. D405-only goal precision was corrected; measured TCP arrival error is now 0.314 mm / 0.0223°. Full painting acceptance remains open.
- 2026-10-14: Managed API startup launched all six original/simulation services, and `/shutdown-system` stopped all six with exit code 0. Existing unrelated port 9090 remained untouched. Latest focused regression run: 137 passed. Isolated headless Chrome verified both original pages, domain 106 / WebSocket 9091 connection and rendered camera display; actual browser painting interaction remains open.

- 2026-10-14: D405 work-area acceptance now carries the measured plane and its capture stamp in one atomic record, preventing cross-topic delivery order from dropping valid geometry. The original two-stroke path passed validation and reached PRECONTACT_TARE. The run stopped on residual verification before contact. A real stationary sample showed torque tails above 0.05 Nm despite stationary joints and a 9.8 mm physical roller/wall clearance; a physics-step sampling diagnosis is in progress. Tare thresholds remain unchanged.

- 2026-10-14: Five-step physical sampling passed unchanged tare criteria in 20/20 standalone trials (instantaneous decimation: 3/20). The next complete ROS run passed actual hardware tare, independent monitor bias and guard reset, then entered CONTACT_SEARCH. First rigid roller/wall contact produced a large real reaction and the original FT_SATURATION guard stopped execution. Measured TCP advanced smoothly; the contact model is under diagnosis. D405 plane error was 0.0049 mm / 0.00027°. Source-state-bound command expiry and ROS-time force ramp corrections passed independent review and targeted tests.

- 2026-10-14: Corrected SI angular servo gains to the USD degree convention and bound a compliant 5000 N/m, 50 N·s/m material to the exact roller collider only. Independent review and 32 asset/simulator tests passed. Free-space chosen-servo TCP error was at most 1.16 mm / 0.052°, with at least 9.51 mm precontact gap. The old-drive material A/B sweep is diagnostic evidence only; full painting on the new combined profile remains under test. A checker cold-import race was reproduced and corrected with bounded strict health collection; 51 checker/assets/contracts tests passed. Fresh final regressions: 91 focused Python, 91 web, actual DDS hardware fixture and both C++ test suites passed.

- 2026-10-14: New-profile manual run completed the first PAINT/RAMP_DOWN/RETRACT/TRAVEL: 775 fresh post-warmup samples over 10.55 s, mean 21.53 N, p95 24.27 N, max 25.99 N and 100% within 18–32 N. Second precontact tare correctly rejected measured TCP position error just over 3 mm; investigation identified the immediate zero-planned-distance transition after TRAVEL. Original browser E2E passed real pointer target/D405/work-area selection, manual two-stroke validation, replacement autofill validation, tare confirmation, Run, PAINT-entry Stop and backend acknowledgement. Observer saw command disable within 9 ms and no re-enable over 5 s; stop happened before PAINT trajectory dispatch, so active-motion cancellation/mechanical stop timing require separate evidence.

- 2026-10-14: Added a maximum 2 s steady measured-arrival wait before tare for position-only settling; all existing 3 mm position, 7 mm clearance and 3° pose gates remain, and guard provenance/faults, contact, TF and identity errors fail immediately. Independent 27 tests and 184 affected regressions passed (10 independently confirmed baseline mocks excluded). Actual standalone Pause/Play/Stop/Play verified invalid/NaN feedback, camera invalidation and new-session reset. A checker target-input race was reproduced: the executor invalidation subscriber could appear before target_selector. Required-consumer discovery now waits for the actual selector; 20 checker/report tests passed independently.

- 2026-10-14: The next actual run completed both strokes and FINAL_RETRACT/COMPLETE. Both force evaluations passed (means 21.34 N and 21.79 N); the report remained failed because its depth-one execution subscription dropped the second instantaneous APPROACH transition. The execution observer alone now keeps 100 events. A real ROS burst fixture reproduced the loss before the change and retained all six transitions afterward; 21 collector/report tests passed. Independent review found no important remaining issue in this change or the complete implementation. A fresh full run is used for final acceptance, without reconstructing the missing historical event.

- 2026-10-14: v5 again physically completed both strokes (means 21.33 N / 21.44 N, in-band fractions 100% / 97.35%), but reader depth alone did not fix the lost event. An actual ASYNC DDS writer experiment reproduced 19/30 deliveries with writer depth 1 and reader depth 100. The original latest-status publisher remains unchanged; a separate reliable volatile depth-100 event stream now publishes the same JSON and sequence. A native producer/Observer burst and late-join fixture confirms six of six new events, zero historical event replay, and one latest status. Combined relevant regressions: 211 passed, 10 known baseline cases excluded; independent focused review and 27 checks passed. Evidence: `logs/rb20-painting/execution-event-publication-validation.json`.

- 2026-10-14: Actual source-loss test passed all 13 checks during an executing, physically moving SAFETY_APPROACH FJT. The owned simulator alone was suspended for 1 s and then resumed. Detection was 198 ms after the last observed input; disabled command followed detection by 1.1 ms, same-UUID CANCELED by 9.1 ms and cancel service ACK by 19.1 ms. Hardware/lifecycle stayed inhibited, work-area/plan validity stayed false, and no command re-enabled during 5 s after resume. See `logs/rb20-painting/actual-stale-stop-v2.json`. The original checker workflow correctly reports abort for this intentionally faulted run.

- 2026-10-14: Final v6 actual run passed all nine health checks and strict painting acceptance, with 76 contiguous actual events from sequence 1 through COMPLETE at 76. Stroke 1: 10.22 s, mean 21.39 N, p95 24.13 N, max 25.07 N, 98.57% within 18–32 N. Stroke 2: 10.68 s, mean 21.63 N, p95 27.02 N, max 28.06 N, 97.55% within band. D405 plane error 0.0047 mm / 0.00029°. Measured physics rate 363 Hz, real-time factor 0.726. Reports: `logs/rb20-painting/compliant-manual-v6-report.{json,trace.json,samples.csv}`. A separate read-only observer captured untrimmed controller references, measured feedback and direct normal trim for postprocessing.

- 2026-10-14: Same-cycle JTC reference versus feedback analysis passed: PAINT tangential RMS 0.804 / 0.816 mm, maximum 1.356 / 1.201 mm. Direct observed controller normal trim stayed within its 20 mm bound (maximum absolute 1.413 mm); no normal-limit flag was observed. Measured final outward retract was 79.950 mm and final roller surface gap 80.228 mm. Source and synchronization limitations are explicit in `logs/rb20-painting/v6-controller-reference-analysis.json`; final geometry is in `compliant-manual-v6-report-geometry.json`. Owned simulator/ROS/API/observers exited and generated readiness was removed. Summary with source hashes: `logs/rb20-painting/final-acceptance-summary.json`.
