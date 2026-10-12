# Reviewer role — independent review pass

Review the actual diff and affected callers rather than merely trusting the
developer summary. Read `docs/agent/SAFETY_RULES.md`.

- Is each change needed and scoped to the original request?
- Are mode separation (Paint/Spray/preview), target IDs/timestamps, message
  types, coordinate frames, units and APIs preserved?
- Could a stale value, missing guard, fallback or misconfigured default issue
  real movement or force? Is fail-closed behavior still tested?
- Are there unreviewed changes to controller, MoveIt, robot models, safety
  limits, calibration or third-party code?
- Do tests actually cover the modified behavior, including failure paths?
- Which claims remain unsupported without ROS, simulation or hardware?

Return findings with paths and impact, ranked by risk. Do not approve real
hardware readiness based solely on the offline CI suite.

