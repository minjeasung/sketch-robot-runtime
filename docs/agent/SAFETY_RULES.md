# Robot safety boundaries for coding agents

This repository can affect real RB10/RB20 motion and contact processes.
Repository tests and code generation do **not** authorize physical operation.

## Non-negotiable boundaries
- Never autonomously connect to, enable, jog, home, calibrate, or move a
  physical robot; never actuate painting force, spray output or controller
  switching as a diagnostic step.
- Do not run launch, installation, runtime startup, motion, or hardware test
  scripts automatically. A command that appears to be a "test" may still
  publish real ROS commands in an operator's environment.
- No bypass of E-stop, interlock, collision protection, controller readiness,
  freshness/heartbeat checks, wrench guards, or fail-closed default paths.
- Do not enable `real_painting_enabled`, `painting_force_enabled` or a
  non-dry-run force configuration by default. Never relax a safety threshold
  just to satisfy a test.
- Never commit operator credentials, API tokens, robot addresses, local
  calibration artifacts or private raw camera data. Avoid editing the
  external Michelo installation without explicit separately scoped approval.

## Safety-critical change checklist
A change to any of the following needs dedicated tests and independent review:
- `src/admittance_controller/` (force compliance, timeouts, trim limits)
- `src/rbpodo_painting_control/` (F/T interpretation and safety gating)
- `src/rbpodo_ros2/rbpodo_hardware/` and robot bringup/MoveIt configurations
- `src/sketch_control/` execution, segment transitions, collision rules,
  target identity and enablement logic
- `config/` defaults that affect hardware, contact, frames or process modes

For roller contact, the normal direction is TCP Y and pressing is TCP -Y.
Wrench references must preserve frame transformations and pass through the
guarded publisher; consult `src/rbpodo_painting_control/README.md`.
A stale F/T, TF, executor or guard status must not silently grant force output.

Before approving a risky PR, verify software invariants, explicit safety
limits, retained default-off behavior and regression coverage. Document any
unverified hardware-dependent assumptions.

## Human-only real-world validation

A qualified operator must separately confirm workspace clearance, calibration,
tool/EOAT geometry, correct robot identity, force-sensor bias and sign,
collision/reachability plan, controller/guard state, speed/force limits,
emergency-stop readiness and a safe abort/retract procedure **before** physical
testing. The operator, not the coding agent, decides whether and how to test.

A passing CI check, ROS mock, dry-run or generated path **never** certifies
physical safety or readiness.

