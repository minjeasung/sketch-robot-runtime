# Original Runtime on Construct Isaac Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans; independent hardware, camera and supervisor changes are delegated under dispatching-parallel-agents.

**Goal:** Use minjeasung's original sketch and management application for Construct in Isaac; preserve the physical runtime.
**Architecture:** Original system_api / Supervisor serves original web pages. Original Outpost bridge, target selector, projector, waypoint generator and MoveIt executor remain the processing chain. An isaac_construct profile supplies simulator-specific robot, camera and startup configuration.
**Tech Stack:** ROS 2 Jazzy, FastAPI, existing JavaScript canvas UI, official Outpost/Michelo, Isaac Sim 6.1.
**Spec:** User's approved direction in this conversation: original UI and workflow, existing official services, only necessary hardware adaptations. Replaces the prior separate Construct sketch design.

## Global Constraints
- Preserve original sketch index.html, styling, layout and controls. Only adapt existing runtime copy to distinguish simulation and configured standoff in app.js/plane_workflow.js.
- No custom replacement Outpost or replacement sketch executor.
- Do not modify JongHyunSeo11 main or push changes.
- Preserve existing physical RB10/RB20 profiles and all collision, freshness, cancellation gates.
- Initial Construct profile uses existing ZED spray motion testing with no gun output; no fictitious D405 or force readings.
- Use Construct's actual URDF, TCP, controller, limits and collision geometry.
- Keep services stopped after validation so user can launch in VNC.

## Review Focus
- Stale startup readiness must not attach an unrelated simulator.
- Native camera/physical robot driver must never launch in Isaac profile.
- Wall-clock SDK frames and simulation TF must use an explicit timestamp policy.
- Motion cancellation must operate through the original executor and actual controller action.
- Changing a target/area must invalidate old paths through existing lifecycle code.

### Task 1: Original supervisor / management configuration
- [x] Add failing tests for profile commands, real-profile regression and readiness identity/timeouts.
- [x] Add isaac_construct option and Construct arm selection to existing management controls only.
- [x] Start and await owned simulator/services; run original perception, executor and rosbridge.
- [x] Verify unit tests.

### Task 2: Robot contract in original executor
- [x] Add failing tests for Construct group/TCP/joints/limits/scene and unchanged RB defaults.
- [x] Resolve robot-specific assumptions from model configuration inside existing executor.
- [x] Keep original path/gating/execution/cancellation; use native Construct action.
- [x] Verify executor/model regression tests.

### Task 3: Official camera to original perception
- [x] Test explicit simulation stamp policy and original default behavior.
- [x] Reuse original Outpost bridge and perception launch with dynamic simulator TF.
- [x] Sustain official Michelo local delivery subscription for raw IPC stream.
- [x] Verify camera/perception tests.

### Task 4: Launch and equipment configuration
- [x] Build original packages into Construct overlay and verify imports.
- [x] Derive tool profile from actual Construct URDF and mesh, add original executor launch.
- [x] Change --construct entry point to original system API; retire separate sketch UI/backend routes.
- [x] Test managed demo readiness/cleanup and command configuration.

### Task 5: Integration validation
- [x] Verify byte-identical original sketch HTML/CSS and original management layout; review minimal runtime-copy JS changes.
- [x] Run original browser target -> plane -> work area -> path pipeline against Isaac images.
- [x] Validate reachable path execution and cancellation on simulated left-arm controller.
- [x] Run relevant regressions and independent review; fix verified findings.
- [x] Document exact launch command, evidence and remaining limitations. Shut down owned services and verify GPU compute processes cleared.
