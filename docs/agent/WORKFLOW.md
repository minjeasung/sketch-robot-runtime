# Agent change workflow

## Scope and planning
1. Read `AGENTS.md` and `docs/agent/SAFETY_RULES.md`.
2. Identify the issue's target mode (Paint, Spray, or `zed_preview`),
   relevant source files, ROS/web contracts, and current tests.
3. State the expected change, untouched behavior, assumptions and safety tier.
4. Work on a separate branch. Keep unrelated file edits and auto-formatting out.

## Role handoffs
- **Developer:** implement minimal changes; identify call sites and tests.
- **Reviewer:** check contracts, correctness, fail-closed paths, and unintended
  edits independently of the implementer's explanation.
- **Tester:** run relevant checks, record reproducible failures and limitations.
- **Human maintainer:** approve PR merge; explicitly supervise any real-world
  hardware validation or force enablement.

The role descriptions in `docs/agent/roles/` are checklists and may be followed
by the same assistant in separate passes. They are **not** an agent runtime.

## Validation tiers

| Tier | Environment | Agent-initiated? | Evidence |
| --- | --- | --- | --- |
| 0 | Static review and offline tests (`scripts/ci/offline_tests.sh`) | Yes | Exact commands / pass-fail |
| 1 | ROS build, mock/fake hardware and ROS-only tests | Only in isolated, confirmed non-actuating environment | Exact environment and results |
| 2 | Simulation, calibration or system integration | Only after environment/safety boundaries are confirmed | Mode, configuration, results |
| 3 | Physical robot, active force, spray or motion | **No; human-operated approval only** | Human-run checklist and records |

Do not automatically promote results from one tier to another. In particular,
a passing planning-only path does not validate a robot-execution path.

## Offline command (non-actuating)

```bash
bash scripts/ci/offline_tests.sh
```

This runs a **limited** Node.js UI test group and the isolated Git-based
plane-version-management Python tests. Requires Node.js, Python 3, Git and
`pytest`. It does not source ROS or configuration secrets, install packages,
start services, connect to cameras or execute controller/robot commands.

The `scripts/check_runtime.sh` script performs installation checks but sources
local ROS/runtime environment; it is **not** a CI command. Do not use
`scripts/run_michelo_sketch.sh`, launch files, or install scripts as automatic
validation. More complete tests in `src/*/test/` need ROS dependencies and
appropriately isolated test environments.

## Pull request handoff
- What changed and **what did not**.
- Affected modes/modules/ROS topics/frames/configurations.
- Exact tests and outcomes; distinguish previously known failures from new ones.
- Evidence needed before ROS, simulator or hardware use.
- Explicit note: no real-world robot actuation was performed unless a human
  actually performed and documented it.

Existing historical reports (e.g. `docs/VALIDATION_20260922.md`) are not
a substitute for fresh results.

