# Tester role — verification pass

Read `docs/agent/WORKFLOW.md` and `docs/agent/SAFETY_RULES.md`.

1. Map changed code to its relevant automated tests.
2. Run `bash scripts/ci/offline_tests.sh` if its prerequisites exist.
3. Record **command**, environment, exit code and failing test names.
4. Where ROS/simulator dependencies are needed, report "not run" rather than
   running services or hardware control automatically.
5. Compare failures with a known baseline; never silently accept failures as
   pre-existing without reproducible evidence.
6. Document manual-only steps and what remains unverified.

Do not change production thresholds or skip failing assertions to make a test
pass; pass findings back to the developer/reviewer.

