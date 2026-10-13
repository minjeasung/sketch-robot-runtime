# Construct Isaac Sim Implementation Plan

> **For agentic workers:** Execute inline following executing-plans and verification-before-completion.

**Goal:** Run the downloaded Construct robot in Isaac Sim and execute MoveIt trajectories.
**Architecture:** URDF/mesh assets become PhysX USD. Native ROS bridge exchanges clock and joint states/commands with topic_based_ros2_control and three trajectory controllers. MoveIt uses the original semantics and planning limits.
**Tech Stack:** Isaac Sim 6.1, ROS 2 Jazzy, USD, Python, MoveIt 2.
**Spec:** ../specs/2026-10-12-construct-isaac.md

## Global Constraints
- Existing checkout and ZIPs remain untouched.
- Fourteen original joint names; exact initial positions; World root.
- Localhost ROS domain 105; no real hardware plugins in generated description.

## Review Focus
- Mesh references must all resolve.
- Initial joint order must match the imported articulation.
- Head and both arm controllers must exist and use simulation state.
- Timeline start/reset must retain a stable initial pose.
- Planning/trajectory results must reflect simulated measured motion.

## Task 1: Assets and robot configuration
- [x] Add and run failing contract tests for asset preparation.
- [x] Prepare URDFs, limits, controller configuration and initial pose manifest.
- [x] Build the two source packages and the topic hardware plugin.
- [x] Import USD and verify 14 revolute joints.

## Task 2: Simulation and MoveIt
- [x] Add Isaac ROS bridge, initial pose, saved scene, and launch scripts.
- [x] Add MoveIt/ros2_control simulation launch.
- [x] Verify clock, 14-joint state, stable pose and all three controller actions.
- [x] Verify MoveIt planning and execution against measured joint feedback.

## Task 3: Delivery
- [x] Review new integration; resolve material findings.
- [x] Save reproducible checks, evidence and Korean instructions.
- [x] Launch GUI scene and provide exact commands/paths.

## Completion evidence
- Asset contract tests: 4 passed.
- Build: Construct description, MoveIt config and topic hardware built successfully.
- Import: 14 revolute joints; original 56 package files byte-identical to Downloads ZIPs.
- GUI Stop/Play test: CONSTRUCT_RESET_OK, maximum initial-pose error 0.000338 rad.
- Live integration: three FollowJointTrajectory controllers each passed outward/return motion; collision validity true; MoveIt left/right/head planning and execution each passed outward/return motion; Ruckig confirmed via live ROS parameters.
- Review findings resolved: logs directory creation, PhysX velocity limits, Jazzy Ruckig adapter, Stop/Play reset latch.
- Implementation rulings: source CAD camera inertia corrected in generated assets only; missing head acceleration/jerk supplied for simulation only, documented in CONSTRUCT_ISAAC_SIM.md.
- Local setup delivered on codex/construct-isaac-sim; no remote push or merge requested.
