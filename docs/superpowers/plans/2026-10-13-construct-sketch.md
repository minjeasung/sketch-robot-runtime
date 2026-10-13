# Construct Sketch Implementation Plan

Goal: deliver the three requested stages through Construct management and sketch windows.
Spec: ../specs/2026-10-13-construct-sketch.md
Execution: inline in existing codex/construct-isaac-sim checkout; no upstream changes or pushes.

- [x] Geometry: scripts/construct_sketch_geometry.py + tests/construct/test_sketch.py. Test backprojection, transforms, sampling, invalid depth/pixels, spacing and offsets before implementation.
- [x] ROS motion: scripts/construct_sketch_motion.py. ROS executor thread, TF/joint capture, MoveIt approach plus full Cartesian plan, conservative timing, preview state guards, direct FollowJointTrajectory/cancel using the MoveIt-planned trajectory and measured trace. Validate on current sim before UI completion.
- [x] Runtime: scripts/construct_sketch_server.py. Official stream subscriber/raw frame cache, frozen snapshots, bounded API requests, single operation lock, stack ownership and lifecycle. Tests cover rejected requests and snapshot/plan identity.
- [x] UI: web/construct_system.html, web/construct.html, associated JS; scripts/run_construct_system.sh and run_system_api.sh --construct entry. Existing RB UI preserved. Live/freeze/draw/plan/execute/stop + state/logs.
- [x] Integration: restart owned demo under management, stream and plan a visible beam stroke, execute and record actual TCP/joint evidence; review, document limitations and leave running.

Review focus: stale/reset stream; changed head or robot start; partial Cartesian plan; simultaneous requests/cancel; shutdown only owned children.
