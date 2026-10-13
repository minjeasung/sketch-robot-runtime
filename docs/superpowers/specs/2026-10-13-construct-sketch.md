# Construct stream sketch control

Authorized outcome: management window starts/stops the Construct Isaac stack; official ZED RGB/depth appears in a separate sketch window; drawing produces a previewed, collision-checked trajectory and the selected simulated arm executes it.

Keep official Outpost/Michelo services and their network/raw frame contracts. No upstream modifications. Preserve legacy RB10/RB20 UI and executor. Add `run_system_api.sh --construct` mode, serving Construct management at `/` and sketch at `/sketch/`. This is the existing application's new robot mode, not a replacement camera service. Use the existing dedicated local Construct checkout.

Management owns only children it starts. Already-running services are visible but cannot be silently adopted/killed. An explicit attach mode supports inspection/development. Stop cancels motion first, then stops owned demo process. Demo owns Isaac, MoveIt, environment, official camera services.

Input: subscribe to official Michelo WebSocket to maintain publisher, read matching lossless raw RGB/depth via official Outpost ZMQ. Frozen snapshot retains image, depth, intrinsics, camera world transform, head pose and stream generation. Camera must be stationary; reject stale frames and camera movement. RGB is displayed with path overlay. Live view resumes explicitly. Do not pretend software frame timestamps are Isaac /clock capture timestamps.

Geometry: sample depth at drawn native-resolution pixels, reject missing/out-of-bounds/discontinuous depth, back-project with intrinsics and transform to World. Offset tool target toward camera by a user-visible 0.30 m default (0.03–0.30). Default left arm (verified beam region), selectable right arm. Fixed current TCP orientation initially, permit choosing a tested approach orientation if needed. Original `<side>_manipulator_ee_point` is TCP. One arm at a time.

Plan: collision-aware MoveIt joint approach to first pose, then Cartesian path with avoid_collisions=true, require full fraction, reject large joint jumps. Retiming conservative from URDF joint velocity limits with bounded acceleration. Cache a unique plan tied to snapshot and all robot joints. Execute only the cached preview, reject changed start state/generation/head/reset; cancel selected controller action via stop. Display actual TCP trace and action result. No contact/paint/weld actuation.

Validation: offline geometry and state guards, management lifecycle/API tests and browser JS checks, then official streamed snapshot -> pixels -> MoveIt trajectory -> measured Isaac joint/TCP motion with report. Test stop/cancel and invalid requests. Current task requires actual movement verification, not just UI/API success.
