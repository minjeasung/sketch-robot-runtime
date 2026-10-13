# Construct ZED / Outpost / H-beam

User requests EOAT identification, missing Michelo Outpost structure, a downloaded ZED SDK with a realistic Isaac camera, and an appropriately sized H beam in front of both arms.

Preserve original Downloads robot and 14-axis simulation. Add official Stereolabs ZED 2i stereo sensor at the original head mount; send left/right images to a separately installed ZED SDK 5.5. SDK computes depth (streamDepth=false). Do not label renderer ground truth as stereo depth. Keep SDK installation account-local because system SDK is inaccessible. Keep real hardware untouched.

Existing repo includes gateway and Outpost consumer, but upstream Airsave source/release is inaccessible. Implement explicitly named simulation Outpost compatible with existing camera status and raw IPC contracts, plus console, event/preview WebSockets. A single process owns pyzed.Camera, receives only localhost simulated camera port 30000 and emits RGB/depth/calibration/monotonic frame metadata. HTTP binds localhost 8100, Michelo gateway localhost 8101. ROS bridge uses existing consumer and Construct domain 105. Publish timestamps in simulation time for ROS, retain wall clock on Outpost wire as required by FrameGuard. Add correct optical-frame TF to the moving head.

H beam comprises two flanges and a web, length 1.5m, width/height .3m, flange .02m, web .01m; place on original worktable if geometry allows, visible from ZED and near arm workspace. Use same geometry in PhysX and MoveIt; no fabricated manufacturer identity for EOAT. Document exact bounds/location and verified versus unavailable capabilities.

Verify SDK imports/version, live stereo frames and nonempty metric SDK depth, Outpost wire decoding, ROS RGB/depth/CameraInfo/cloud/TF, H-beam collision and robot initial validity. Verify current trajectory controllers remain available. Save viewport and camera preview. Do not claim official proprietary Michelo daemon/Windows frontend was installed.
