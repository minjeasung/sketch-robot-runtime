# Change-to-test matrix (main / Ubuntu 24.04 / ROS 2 Jazzy)

Use this matrix together with [WORKFLOW.md](WORKFLOW.md) and
[SAFETY_RULES.md](SAFETY_RULES.md). It maps **representative paths to existing
tests**, not a complete proof that every integration is covered.

## Always run the offline baseline (Tier 0)

```bash
bash scripts/ci/offline_tests.sh
```

GitHub Actions runs this on pull requests and `main` pushes. It checks:

| Group | Existing test selection | Purpose |
| --- | --- | --- |
| Plane-version management | `scripts/tests/test_plane_versions.py` | Snapshot switching and preservation of unrelated Git changes |
| F/T and guard logic | `src/rbpodo_painting_control/test/test_force_safety.py` + `test_wrench_guard.py` | Sensor invalidity, abort/latch decisions and fail-closed wrench guard |
| Browser workflow | `web/test/*.test.js` | Browser sketch state and UI behavior with mocked ROS transport |

The force tests import the ROS-independent Python modules directly using
`PYTHONPATH=src/rbpodo_painting_control`. This suite never starts ROS, Michelo,
MoveIt, hardware, camera streams or a robot. **Passing it does not verify
runtime ROS integration, robot reachability, calibration or physical safety.**

## Select extra checks for the actual change

**The whole Tier 0 suite runs for every PR.** For the following areas, inspect
the named tests and add/update regression tests as needed. An "additional"
test below is **not automatically executed by this initial CI**; verify its
dependencies and run it only in a confirmed non-actuating environment.

| Changed area / likely paths | Existing targeted tests / checks | Level and review |
| --- | --- | --- |
| Browser state, sketch, images, system controls: `web/js/`, `web/` | `web/test/*.test.js` | Tier 0, included in CI; add a UI regression test for changed behavior |
| Plane algorithm or selection: `multi_plane_geometry.py`, `organized_planes.py`, `plane_lifecycle.py` | `test_organized_plane_extraction.py`, `test_plane_lifecycle.py`, `test_multi_plane_perception_core.py`; compare against recorded RGB/depth for accuracy | Additional isolated Python/ROS-dependent validation (Tier 1); recorded scene comparison is separate evidence |
| Plane snapshot tooling: `scripts/plane_versions.py`, `config/plane_extraction_versions.json` | `scripts/tests/test_plane_versions.py`; inspect preserved Git state | Tier 0, included in CI; do not auto-switch algorithms |
| Work area, coverage and path: `work_area_geometry.py`, `sketch_to_waypoints_node.py`, `segment_path.py`, spray planning | `test_work_area_geometry.py`, `test_polygon_work_area.py`, `test_segment_path.py`, `test_zed_spray_generation.py` | Additional isolated checks (Tier 1); generated path is not collision-certified |
| Force safety or guarded reference: `force_safety.py`, `wrench_guard.py` | `test_force_safety.py`, `test_wrench_guard.py` (CI); `test_wrench_guard_node.py`, `test_wrench_reference_node.py` (separate) | Tier 0 plus Tier 1 for ROS nodes; independent safety review required |
| Controller, hardware, MoveIt, actuator settings: `src/admittance_controller/`, `src/rbpodo_ros2/`, execution modules, safety-critical YAML | Controller/hardware C++ tests and relevant `test_moveit_executor_fail_closed.py`, `test_executor_plan_identity.py`, `test_painting_execution.py`; inspect default-off/timeout/frame settings | Tier 1 in isolated ROS/mock/fake environment; independent safety review; Tier 3 human gate before physical use |
| Camera bridge, supervisor or ROS API: `outpost_bridge_node.py`, `system_api.py`, `process_supervisor.py` | `test_outpost_integration.py`, `test_system_api.py`, `test_zed_preview.py`; check topic/message contract and camera freshness | Additional isolated Tier 1 checks, never assume the live camera is mocked |
| EOAT/model URDF, TF and calibration: `src/eoat_description/`, robot descriptions, calibration modules | `test_robot_models.py`, `test_spray_eoat_config.py`, `test_zed2i_assets.py`; inspect frame and geometry assumptions | Tier 1 + Tier 2 if isolated simulation is available; Tier 3 human calibration/clearance checks |

Paths in the table are illustrative. Tests under `src/sketch_control/test/`
and other ROS packages may import ROS even when their assertions look
hardware-independent. Do not add them blindly to a hosted CI job.

## Tiers, evidence and non-negotiable gates

- **Tier 0 (hosted CI):** static inspection and the three offline groups above;
  report exact test command, result and run URL.
- **Tier 1 (isolated ROS/mock):** package build and targeted tests after verifying
  all ROS interfaces point to a non-actuating test environment. Never run launch
  or controller-start commands on an unknown workstation.
- **Tier 2 (simulation/integration):** only in a separately verified isolated
  simulator; record mode, controller configuration and coverage.
- **Tier 3 (real robot):** physical motion, contact force, spraying and
  calibration require explicit human-operated approval; no agent automation.

For force/contact changes, inspect the TCP Y (press TCP -Y) frame contract,
wrench guard, stale F/T/TF/heartbeat responses, controller limits and
`real_painting_enabled=false` / dry-run defaults. The initial Tier 0 suite
tests the **pure logic**, not all launch/configuration invariants. Do not claim
those settings were automatically verified.

When a bug is found, add the smallest test that reproduces it, then implement
the fix. Preserve the failing example as a regression test. Document exceptions
and unrun higher-tier checks in the PR instead of silently treating them as
passed. An earlier report (`docs/VALIDATION_20260922.md`) mentioned existing
failures; those are historical observations, not a waiver for current failures.
