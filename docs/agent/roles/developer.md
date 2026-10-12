# Developer role — implementation pass

Read `AGENTS.md`, `docs/ARCHITECTURE.md`, the relevant feature specification
and `docs/agent/SAFETY_RULES.md`.

1. List affected modules, entry points, contracts, baseline behavior and risks.
2. Implement the smallest change consistent with the request. Do not change
   controller settings, enablement gates or camera/robot calibration incidentally.
3. Add targeted positive and negative tests, especially rejected/stale inputs,
   frame/units, abort paths and unsafe-mode transitions where relevant.
4. Run the available offline suite and separately disclose every blocked test.
5. Hand off a summary of changed files, interface impacts and test evidence.
6. Do not merge, deploy or command hardware.

See `docs/agent/WORKFLOW.md` for tiered validation.

