# Implementation ledger — SNUCEM Spray Add-on

Plan: 2026-10-06-snucem-spray-addon.md; branch: codex/snucem-spray-addon.

## Retarget decision

The user explicitly requested finishing the existing branch against **JongHyunSeo11/SNUCEM_Robot_22.04**, Ubuntu 22.04, ROS 2 Humble, and explicitly prohibited changing the upstream repository. The existing scope and read-only integration architecture remain in force. No additional approval is required to carry out this correction.

Supported upstream: `7b0a2edcc3d1659bdc2dcb495d4dd63f75899f1c`.
Source was inspected through the authenticated GitHub connector. Reference files remain outside this repository and are never distributed. No upstream writes or pushes are permitted.

Ruling: use the existing isolated checkout of the requested feature branch; no second worktree is needed.
Ruling: implement inline, with a fresh final review. Do not create per-task implementer agents.
Ruling: ROS processes use the existing Humble underlay and Python 3.10, with NumPy <2. The add-on does not install/build/patch the upstream stack or camera SDK.
Ruling: external stack URDF/SRDF/limits and mesh hashes bind the generated tool profile and execution. No second collision tool is attached.
Ruling: finite measured support is exported independently from inferred support. Work polygons must lie within the measured support; source or calibration changes revoke execution. Observation-only timestamp changes do not.

## Preflight

- Tasks 1/6 share validated upstream/install/state roots and the write-root policy.
- Tasks 2/4 share a source session and geometry fingerprint, with separate freshness time.
- Tasks 3/4 share the live model descriptor and immutable model/tool fingerprint.
- Tasks 5/6 share a fixed child-process allowlist; external processes are never stop targets.
- Local environment: Windows, Python 3.12; no WSL distribution, Docker or ROS installation.
- Initial browser baseline: 65 passed.
- Initial bare Python collection: 36 dependency/import errors, 2 skipped. These are an environment baseline, not new behavioral failures. Full details retained in work/baseline-python.txt outside this repository.
- Task 1 RED: 4 intended missing-implementation failures, 1 Python 3.10 syntax check passed.

## Progress

- [x] Inspect Humble upstream and retarget design/plan.
- [x] Task 1: owned paths, configuration, compatibility (`config.py`, `compatibility.py`, `test_contracts.py`).
- [x] Task 2: finite plane export, request-bound selection and invalidation (`planes.py`, `perception.py`, `plane_bridge.py`, `test_planes.py`).
- [x] Task 3: live external robot/tool model (`model.py`, `model_node.py`, `test_model_guard.py`, `test_runtime_hooks.py`).
- [x] Task 4: guarded automatic execution (`execution.py`, `execution_guard.py`, opt-in existing executor hooks; real robot acceptance remains pending).
- [x] Task 5: Spray-only management and web integration (`runtime.py`, `api.py`, `cli.py`, `web/addon.html`, existing Sketch page injection).
- [x] Task 6: deterministic distribution and installer (`packaging.py`, `install.py`, launcher scripts, `test_packaging.py`).
- [ ] Task 7: regression, integration harness, review and delivery.

## Execution decisions and evidence

- Preserved the user's authorization to finish the existing branch; no repeated design approval requested after retargeting.
- Used one integrated feature commit rather than artificial per-task commits after interrupted work. Future correction/evidence commits remain on the same feature branch.
- `paths.py` is consolidated into `config.py`; source closure follows AST imports of explicitly allowed entry points. The bundle includes no upstream source/assets.
- The installed upstream does not expose one stable rectangular footprint API satisfying gap preservation. The adapter triangulates **measured** support with a bounded 5 cm edge, without copying any extraction algorithm. It retains an immutable conservative footprint while new samples still cover it.
- Existing projector ROS stub lacked CompressedImage on Windows; reproduced 49 setup errors, then repaired only the fixture. Runtime behavior unchanged by this test repair.
- Additional owned-child path escape test reproduced missing validation, then passed after destination validation was added to writes.
- Fresh read-only code reviewer identified cloud freshness, verified_hold remeshing and shutdown-context problems; all corrected and rechecked. No remaining critical/important finding in the reviewed snapshot.
- Concentrated Windows Python verification: 215 passed, 2 skipped (symlink privilege; rclpy absent). Browser: 65 passed. Full cross-platform suite limitations documented separately.
- Added `.github/workflows/snucem-humble.yml` using Ubuntu22.04 / real Humble messages and an isolated ROS domain. `test_ros_smoke.py` checks constructors and blocked dispatch; it does not substitute for MoveIt/FJT/hardware acceptance.
