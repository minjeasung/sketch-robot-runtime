> Superseded 2026-10-13: the custom Outpost implementation was removed at the user's request. See docs/CONSTRUCT_ISAAC_SIM.md for the official-service deployment and its current input limitation.

# Construct ZED Outpost Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Working simulated ZED SDK camera through Outpost-compatible IPC/ROS plus H beam in Construct scene.

**Architecture:** Official ZED Isaac extension sends stereo to account-local SDK. Simulation Outpost owns SDK receiver; existing gateway and ROS bridge consume its public contract. Shared beam boxes describe both physics and planning geometry.

**Tech Stack:** Isaac Sim 6.1, ZED SDK 5.5, Python, FastAPI, ZMQ, ROS2 Jazzy, MoveIt.

**Spec:** docs/superpowers/specs/2026-10-12-construct-zed-outpost.md

## Global Constraints
- Preserve original Downloads files and existing 14-axis control.
- Local SDK; HTTP localhost8100/8101; stream30000; ROSdomain105.
- Stereo SDK depth only; explicit simulation Outpost identity.

## Review Focus
- Camera stops: mark stale/error, never continue publishing stale frames.
- Camera calibration and optical frame must agree with SDK rays.
- Stop/Play: recover stream or expose clear error.
- Same beam boxes in PhysX and MoveIt; initial state remains valid.
- SIGINT cleans up owned SDK camera and socket, no real device discovery.

### Task 1: Runtime dependencies
- [x] Download/extract SDK under user account; install Python API in isolated venv; verify version.
- [x] Build official ZED Isaac extension and verify load.

### Task 2: Camera + scene
Files: scripts/construct_scene_objects.py, scripts/isaac_sim_construct.py, scripts/publish_construct_environment.py.
- [x] Define shared H-beam and support boxes and camera mount transform.
- [x] Attach official ZED2i asset and stereo helper to original moving head; render beam.
- [x] Apply matching collision objects to MoveIt and publish optical TF.

### Task 3: Outpost service
Files: scripts/construct_outpost.py, scripts/run_construct_camera.sh, tests/construct/test_outpost_service.py.
- [x] Add failing wire compatibility/stale-state tests using existing decoder.
- [x] Implement SDK-only receiver, calibration, multipart publisher, HTTP console and WebSockets.
- [x] Run contract tests and live SDK/HTTP/IPC/ROS checks, including paused-source shutdown and fresh restart checks.
- [x] Start gateway and existing ROS bridge with simulation-time ROS stamps.

### Task 4: Deliver
- [x] Save viewport/RGB/depth preview and validation JSON.
- [x] Validate robot state, collision objects, camera streams and TF.
- [x] Review changes; document startup/shutdown, EOAT and realism limits.
