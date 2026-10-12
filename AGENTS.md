# Agent instructions — sketch-robot-runtime

This file is the starting point for Codex or other coding assistants. Read relevant
source and tests before editing; documentation is orientation, not proof of behavior.

## Project and entry points
- `main` targets Ubuntu 24.04 / ROS 2 Jazzy. Do not infer compatibility with
  Ubuntu 22.04 / Humble or the `codex/snucem-spray-addon` branch.
- Start with `README.md`, `RUN.md`, `docs/ARCHITECTURE.md` and the relevant
  feature documentation under `docs/`.
- Follow `docs/agent/WORKFLOW.md` for change planning, validation and handoff.
- Follow `docs/agent/SAFETY_RULES.md` for every task, especially motion/force changes.

## Change workflow
1. Identify the affected ROS packages, web modules, configuration, and consumers.
2. Summarize the expected behavior and risk before a nontrivial change.
3. Make the smallest reviewable change; keep unrelated edits out.
4. Add or update regression tests for changed logic. Preserve ROS topic/service,
   frame, unit, calibration, mode and safety contracts unless explicitly requested.
5. Run `bash scripts/ci/offline_tests.sh` when dependencies are available, then
   report exactly which additional checks ran or were blocked.
6. Report files changed, behavior changed, safety implications, test evidence,
   and unresolved assumptions. Use a PR for review; do not merge autonomously.

## Hard boundaries
- Never start or command physical robot motion, spraying, force output, homing,
  controller activation, or camera calibration as an agent-initiated check.
- Do not run hardware/launch/install scripts as part of the offline test suite.
  Never bypass interlocks, guard checks, fail-closed behavior, or default-off
  real-force settings to make a test pass.
- Never modify on-machine calibration, credentials, host-specific environment
  files, external Michelo installation, or third-party/vendor code without a
  separately justified and reviewed scope.
- Do not treat path generation or `zed_preview` success as proof of collision
  safety, reachability, calibration accuracy, or hardware readiness.
- For force or motion changes, follow the elevated review in
  `docs/agent/SAFETY_RULES.md` and retain a human hardware approval gate.

## Role guides (not automatically spawned agents)
- Implementation: `docs/agent/roles/developer.md`
- Independent review: `docs/agent/roles/reviewer.md`
- Test/verification: `docs/agent/roles/tester.md`

The role files define responsibilities and handoff checklists. They do not, by
themselves, create or schedule independent agents.

