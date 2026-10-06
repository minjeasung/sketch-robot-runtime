# SNUCEM Spray Add-on Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans for native execution, or superpowers:subagent-driven-development if the user selects delegation. Steps use checkbox syntax for tracking.

**Goal:** Install our complete Spray web/sketch/path/automatic-execution workflow as a small add-on to an existing SNUCEM_Robot installation, without modifying any JongHyun file.

**Architecture:** The add-on owns its files, processes and settings. A wrapper imports the installed SNUCEM perception class and exports immutable finite plane snapshots. Our adapter, path generation and executor use that data and the external stack's actual model, while retaining the existing HTTP-management/ROS-workflow separation.

**Tech Stack:** Ubuntu 24.04 x86_64, ROS 2 Jazzy, Python 3.12, FastAPI/Uvicorn, NumPy/SciPy/OpenCV, existing browser JavaScript and rosbridge.

**Spec:** ../specs/2026-10-06-snucem-spray-addon-design.md

## Global Constraints

- All tracked changes belong to minjeasung/sketch-robot-runtime, branch codex/snucem-spray-addon.
- JongHyunSeo11/SNUCEM_Robot and its installed files must not be modified.
- Supported upstream reference: fe7531f2a3738d1e690371fc5d56f5f0a12932fd; validate the imported interface/files before starting.
- No upstream source/model copies in the distribution. No git clone, upstream build, driver installation or camera SDK installation in the add-on installer.
- Disable Python bytecode writes in upstream imports; logs, caches, model snapshots, locks and configuration live under the add-on installation/state root.
- Spray only. D405, roller force control, Isaac Sim and our plane extraction variants are not launched.
- RB10/RB20 use the live stack's link0/tcp frames and +Z nozzle direction.
- Never manufacture ready, gun acknowledgement or compliance status to satisfy an old precondition.
- Preserve plane boundaries, identity and invalidation; preserve existing manual/automatic path validation.
- Package goals: compressed <= 5,000,000 bytes, unpacked owned files <= 10,000,000 bytes, dependencies reported separately.
- Windows tests do not prove Ubuntu ROS integration or hardware operation.
- Each test/code task uses red -> green -> focused regression verification before its commit.

## Review Focus

1. Installer path aliases/symlinks or Python caches must not write into the upstream tree (Tasks 1, 6).
2. Perception timestamps may change while a selected plane remains stable; actual removal/geometry/calibration changes must revoke the plan (Task 2).
3. A valid plane may have disjoint support; the adapter must not fill empty space between cells (Task 2).
4. The same robot model name can describe a different TCP/tool profile; geometry fingerprints must invalidate execution and avoid a second offset/collision tool (Task 3).
5. Another controller may start after the automatic executor is ready; check ownership continuously and before every dispatch, then cancel without automatic resume (Task 4).

## Baseline and evidence

- Parent implementation: ebc36dd676602ad913a187b32762b3aedd15b88f.
- Design commit: 4316aab56289955952e9383fa0a4d79928652475.
- Existing source checkout is sparse and isolated under this chat's work directory.
- No AGENTS.md found in the checked source or enumerated repository tree.
- Windows focused baseline: 85 Python tests passed; 49 fixture setup errors occur because the existing ROS stub lacks sensor_msgs.msg.CompressedImage.
- Browser baseline for spray_workflow.test.js and plane_clear.test.js: 25 passed.
- Record this baseline explicitly. Repair the missing test stub only in our repository when adding the relevant regression coverage; do not describe it as a new runtime defect.

## File structure and shared contracts

All new Python modules live under addons/snucem_spray/snucem_spray_addon/.
Use an explicit bundle allowlist to include selected existing source files and static assets.

- paths.py / config.py: read-only upstream versus owned state paths; validated startup configuration.
- compatibility.py: supported upstream files/interfaces and ROS preflight requirements.
- planes.py: pure finite plane export/projection/selection/snapshot logic.
- perception.py: installed RB20SprayNode wrapper and versioned catalog publisher.
- plane_bridge.py: ROS catalog/CameraInfo/TF/selection adapter for the existing Sketch contract.
- model.py / model_node.py: live robot model, SRDF, mesh references, joint limits and identity.
- execution.py / execution_guard.py: existing automatic executor integration and exclusive motion ownership.
- runtime.py / api.py / cli.py: child process allowlist, external state, management API and entry points.
- install.py / packaging.py: owned installation and deterministic distribution.
- tests/ plus web/test/: pure contracts, API/process behavior, browser flows and packaging.
- tools/ros_fake_smoke.py: explicit Ubuntu-only external-stack integration check.

### Task 1: Read-only upstream attachment and owned paths

**Files:** Create paths.py, config.py, compatibility.py and tests/test_install_contract.py under addons/snucem_spray/.

**Interfaces:**
- AddonConfig: upstream_root, install_root, state_root, ros_domain_id, rosbridge_url, profile, camera topics and supported model/tool options.
- validate_paths(upstream_root: Path, install_root: Path, state_root: Path) -> validated paths.
- check_upstream(root: Path) -> report with supported revision/file signatures and missing requirements.
- runtime_environment(config: AddonConfig) -> environment mapping; set PYTHONDONTWRITEBYTECODE=1, owned ROS_LOG_DIR and cache paths.

- [ ] Write tests rejecting equal/nested/aliased upstream write destinations, invalid profiles and unsupported upstream interfaces. Snapshot fixture tree content/mtime before and after all checks.
- [ ] Run python -m pytest -q addons/snucem_spray/tests/test_install_contract.py; observe missing implementation failures.
- [ ] Implement validation using resolved paths, bounded configuration, read-only signature checks and an explicit write-root policy. Do not invoke upstream install/build/model-cache helpers.
- [ ] Re-run tests; verify the upstream fixture remains byte-for-byte unchanged.
- [ ] Commit with an explicit Task 1 ledger entry.

### Task 2: Reuse SNUCEM planes in the existing web selection workflow

**Files:** Create planes.py, perception.py, plane_bridge.py, tests/test_planes.py and tests/test_plane_bridge.py; extend web/test/plane_clear.test.js only where the new source contract requires coverage.

**Interfaces:**
- snapshot_from_catalogue(entries, *, frame_id, source_revision, stamp_ns, calibration_id) -> schema_version=1 dictionary.
- project_catalogue(snapshot, camera_info, transform, selection) -> existing /perception/target_planes payload with generation, frame_id, planes, image dimensions.
- CatalogSelection.update(snapshot) -> changed selected IDs; timestamp-only updates do not count as geometry changes.
- Perception wrapper publishes /snucem_sketch/plane_catalog; bridge consumes /target_selection_pixels and publishes /perception/target_planes.

- [ ] Write pure tests for stable IDs, source reset, finite normalized normals, mm/m rejection, disjoint cells, zero-area support, behind-camera points and missing TF.
- [ ] Write callback tests for request stamp echo, clear/undo, replay rejection, selected-plane removal and preservation under timestamp-only updates.
- [ ] Run both new test files and observe red results.
- [ ] Implement the exporter by subclassing/importing the installed RB20SprayNode in our own package. Snapshot _patch_memories under _observation_lock; use the installed plane footprint helpers. Never copy/alter the RANSAC implementation.
- [ ] Run the wrapper as the explicit replacement for the upstream node command; detect duplicate perception startup. Keep upstream algorithm configuration read-only.
- [ ] Convert real finite support cells to selectable regions without convex-hull bridging. Attach source identity and revision; infer-only support is not silently promoted to a measured work region.
- [ ] Preserve camera exposure/frame correspondence and existing browser generation semantics. Invalid/revoked data publishes explicit invalidation, not a new valid empty identity.
- [ ] Re-run the new tests and browser plane_clear suite; commit.

### Task 3: Bind paths to the external live robot and tool model

**Files:** Create model.py, model_node.py and tests/test_model.py; modify our robot_models.py, spray_eoat.py and executor model hooks only through an explicit external-stack option.

**Interfaces:**
- parse_stack_model(urdf: str, srdf: str, joint_limits: dict, resource_roots: dict) -> StackModel.
- StackModel supplies model_id, +z axis, link0/tcp, joint limits, semantic collision pairs, mesh references and content fingerprint.
- build_tool_profile(model: StackModel, owned_state: Path) -> confirmed-model profile path under owned state; no upstream output.
- External-model option is opt-in; existing main/default behavior remains covered.

- [ ] Write tests using small synthetic RB10/RB20 URDF/SRDFs: +Z, nozzle endpoint [0,0,0], changed mounting/mesh/tool fingerprint, missing joints, incomplete limits, foreign collision exclusions and ft_preview distinction.
- [ ] Observe red results.
- [ ] Read live /robot_description and the external move_group semantic/limit parameters; cross-check robot/controller identity.
- [ ] Resolve meshes read-only from installed package roots. Avoid rb20_spray.stack_model.cache_description's upstream default output; write snapshots only to owned state.
- [ ] Add explicit hooks so our executor uses external limits/SRDF/TCP rather than our default files, ready pose and duplicated attached EOAT.
- [ ] Make generated paths carry the external model/tool fingerprint and reject stale identities at validation/dispatch.
- [ ] Re-run new tests and existing robot/spray geometry/profile tests; commit.

### Task 4: Automatic path execution on the external stack

**Files:** Create execution.py, execution_guard.py and tests/test_execution.py; modify moveit_executor.py and spray_execution.py only at opt-in external-stack hooks.

**Interfaces:**
- ExecutionGuard.update_controller_state(state), update_model(fingerprint), update_source(revision), blockers(now) -> tuple[str, ...].
- ExternalSprayExecutor reuses MoveItExecutor's path/plan/cancel behavior and checks ExecutionGuard before accepting or dispatching motion.
- Shared status report includes actual readiness, external ownership, source/model identity, action availability and abort reason.

- [ ] Write tests for FollowJointTrajectory endpoint binding, stale joint/model/plane state, competing executor activation after readiness, pending goal cancellation and no automatic resume.
- [ ] Test rejection of Paint, force-enabled configuration, unverified physical gun feedback and missing required controller state.
- [ ] Observe red results.
- [ ] Implement a controller-manager-backed readiness check for position trajectory control with no active compliance controller; do not publish synthetic compliance messages.
- [ ] Preserve joint/scene/path/abort checks; add explicit external ownership checks at every command boundary and the periodic guard.
- [ ] Reuse existing dry-run and spray-motion-test distinctions. Hardware ON remains dependent on real device acknowledgement; motion-test keeps the output OFF.
- [ ] Avoid writes to upstream MoveIt configuration, model files or collision exclusions. Live scene changes are owned task objects and cleaned up by their IDs.
- [ ] Re-run focused executor/spray contracts; commit. Record ROS-runtime verification as pending until Task 7 is actually run.

### Task 5: Spray-only web and add-on management API

**Files:** Create api.py, runtime.py, cli.py, tests/test_api.py and tests/test_runtime.py; modify web/system.html, web/js/system.js, web/index.html and web/js/app.js through an explicit addon profile; add web/test/snucem_addon.test.js.

**Interfaces:**
- create_app(config, runtime) -> FastAPI; existing health/status/configuration/process prepare/shutdown shape remains usable.
- Runtime owns only exporter, bridge, projector, path generator, automatic executor and an optional owned rosbridge.
- External stack and external rosbridge are status entries, never stop/kill targets.
- GET /addon/capabilities describes Spray-only mode, SNUCEM source and external model/control ownership.

- [ ] Write API/process tests showing prepare never executes a path; rollback/shutdown stops only newly owned children and never external processes.
- [ ] Write browser tests hiding Paint/D405/force controls for addon mode, preserving ordinary runtime mode, and showing source/model readiness without implementation internals.
- [ ] Observe red results.
- [ ] Implement the process allowlist and ownership-aware status. Preserve authentication, no arbitrary shell parameters, configurable ROS domain/rosbridge URL and normal port-conflict handling.
- [ ] Reuse wall_projector and sketch_to_waypoints with external image/CameraInfo topics and matching TF; do not start Outpost or another camera driver.
- [ ] Extend the existing non-ROS projector test fixture with CompressedImage so relevant baseline tests execute.
- [ ] Verify manual freehand/line/polygon workflows, auto-fill, validation and explicit execution acknowledgement.
- [ ] Run API/runtime tests, existing browser suite and repaired projector regression suite; commit.

### Task 6: Small reproducible installer and distribution

**Files:** Create install.py, packaging.py, addon package metadata, scripts/package_snucem_spray.py, tests/test_packaging.py and docs/SNUCEM_SPRAY_ADDON.md.

**Interfaces:**
- build_bundle(repo: Path, output: Path) -> manifest and archive paths.
- install_bundle(bundle: Path, config: AddonConfig, *, activate: bool=False) -> installation report.
- CLI commands: doctor, install, serve, perception, bridge, model and executor; process commands only after a valid local configuration.
- No global site-package mutation or upstream setup.py/editable install invocation.

- [ ] Write tests verifying deterministic file allowlist, SHA256 manifest, forbidden assets/.git/build exclusion, symlink/path traversal rejection and size ceilings.
- [ ] Write installation tests for existing config preservation, failure rollback, active-service upgrade refusal and version-specific owned roots.
- [ ] Observe red results.
- [ ] Package the reusable source modules/web/configuration and register only required ament resources without triggering our full installer.
- [ ] Source the existing external ROS underlay read-only; create the add-on virtual environment and any generated resources under its own root.
- [ ] Refuse install/state roots inside upstream, including symlink aliases. Disable bytecode output in imported upstream code. Verify upstream fixture hashes before/after install/start/stop/update tests.
- [ ] Build archive, inspect contents, verify each checksum, dry-install from the archive and report compressed/unpacked/additional dependency sizes.
- [ ] Commit docs including actual commands, upstream pin, settings/port requirements and limitations.

### Task 7: Integration evidence, review and delivery

**Files:** Create tools/ros_fake_smoke.py, test fixtures and a validation report; update implementation ledger and installation guide.

- [ ] Add a contract smoke test that drives exported planes -> request-bound selection -> work polygon -> manual/auto generated path -> model-bound validation -> fake action dispatch/cancel.
- [ ] Provide an Ubuntu ROS integration command using an already-running external fake stack; do not start or mutate a real robot or upstream checkout.
- [ ] Run all feasible Python/browser/package checks, inspect complete exit codes and distinguish environmental setup errors from behavioral failures.
- [ ] On an available Ubuntu Jazzy environment, run actual external fake-stack smoke and record the model, topics/actions and results. If unavailable, label it unverified; do not mark hardware readiness complete.
- [ ] Obtain one fresh whole-branch review after implementation, fix material findings with reproducing tests and rerun affected suites.
- [ ] Verify git diff contains no upstream copies or changes; verify the upstream repository is read-only throughout.
- [ ] Push commits only to codex/snucem-spray-addon, open a draft PR in minjeasung/sketch-robot-runtime and attach it to this chat. Do not merge main.
- [ ] Deliver the installation archive, checksum manifest, installation guide and validation report under outputs/, with the branch/PR links.

## Execution handoff

Recommended method: native execution in this chat, with one independent final review.
The plane, model, executor and packaging changes share interfaces; implementing them in one context avoids repeated context setup.
No per-task implementer agents are needed. If the user chooses delegation, partition only work with stable contracts and separate file ownership.
