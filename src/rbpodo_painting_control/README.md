# rbpodo_painting_control

Painting roller helpers for ROS 2 admittance/contact control.

The primary controlled-contact path now uses the ROS 2 Jazzy
`admittance_controller`:

```text
MoveIt / JointTrajectoryController
  -> admittance_controller
  -> rbpodo_hardware
```

The former FZI helpers are archived outside the active package under
`legacy_fzi_backup/`. Robot painting tests should use the admittance profile and
wrench-reference nodes below.

## Painting / Contact Admittance Control

Start the real RB10 MoveIt/controller stack with the painting/contact
admittance profile:

```bash
ros2 launch sketch_control rb10_moveit_no_rviz.launch.py \
  use_isaac_sim:=false \
  use_sim_time:=false \
  use_admittance:=true \
  admittance_use_case:=painting \
  use_fake_hardware:=false
```

`admittance_use_case:=painting` selects `painting_normal_y` automatically and,
unless explicitly overridden, does not start the free-push reset or variable
impedance helpers.

`painting_normal_y.yaml` keeps the existing MoveIt/JTC chain and enables only
the TCP Y admittance axis:

```text
selected_axes = [false, true, false, false, false, false]
```

Frame convention for the roller EOAT:

- Surface `+X` is the local tangent/reference direction supplied by the path.
- Surface `+Y` is the second tangent/lateral direction.
- Surface `+Z` is the outward surface normal.
- TCP `+X` is Surface `+X`.
- TCP `+Y` is Surface `+Z`, the normal/contact axis.
- TCP `+Z` is Surface `-Y`.
- Pressing into the surface is TCP `-Y`.
- The RBPodo/AFT upstream normal channel has reversed sign during otherwise
  continuous wall contact, so its signed TCP `Fy` is retained for diagnostics
  but is not trusted as the reaction direction.

So the normal force axis is **TCP force.y**, not TCP force.z. The admittance
controller's `wrench_reference` must be in the F/T sensor frame.
`painting_wrench_reference_node` builds the conceptual TCP wrench and transforms
it into `ft_link`, but publishes only an internal request. The guard is the sole
controller-topic publisher:

```text
conceptual target in tcp: [0, -desired_contact_force_n, 0]
request: /painting_admittance/requested_wrench_reference (ft_link)
guarded output: /admittance_controller/wrench_reference (ft_link)
```

The wrench command still uses `target_wrench_sign=-1.0` to press into the wall.
The real profile uses `absolute_normal_force=true`: contact detection, ramp
handover, and the admittance normal feedback all interpret the measured normal
reaction as `abs(TCP Fy)`. Thus measured `+3 N` and `-3 N` both balance the
same commanded `-3 N`; the command direction itself is never absolutized.
Signed TCP `Fy`, tangential forces, and torques remain available in status and
their six-axis over-force/impact limits remain bidirectional.

The guard forwards a nonzero request only in `RAMP_UP` or `PAINT`, and only
while mode, enable, executor heartbeat, filtered/raw F/T, TF, safety status,
and controller status are all fresh. Any failed condition publishes an exact
zero wrench and disables controller compliance at the configured fixed rate.
`real_painting_enabled` defaults to `false` independently of `dry_run`.

Run the admittance painting support nodes in dry-run mode:

```bash
ros2 launch rbpodo_painting_control painting_admittance_control.launch.py \
  config_file:=/absolute/path/to/painting_system_real.yaml \
  dry_run:=true \
  enable_force:=false
```

`config_file` is optional. When supplied, the same ROS parameter YAML is loaded
first by the request, safety-monitor and guard nodes; explicit launch arguments
remain later overrides.

Run the segment-aware sketch executor with real-robot trajectory execution:

```bash
ros2 run sketch_control moveit_executor --ros-args \
  -p execution_backend:=follow_joint_trajectory \
  -p use_eoat_segments:=true \
  -p painting_force_enabled:=false
```

With these defaults the complete segment process can be checked while the
commanded wrench remains zero. Real force output requires all of the following:

- `dry_run:=false` on `painting_admittance_control.launch.py`
- `real_painting_enabled:=true` on `painting_admittance_control.launch.py`
- `painting_force_enabled:=true` on `moveit_executor`
- a positive `default_paint_force_n` in the sketch/perception launch
- fresh executor heartbeat, safety status, F/T, TF, and controller status

Validate TCP Y force sign, F/T bias, controller state, and clearance on hardware
before enabling those three settings together.

## Segment-driven execution

`sketch_to_waypoints_node` publishes a versioned process path on
`/sketch_eoat_segments`. Each generated stroke is a `PAINT` segment. Separate
strokes are connected by explicit
`RAMP_DOWN -> RETRACT -> TRAVEL -> APPROACH_PRECONTACT -> CONTACT_SEARCH ->
RAMP_UP` steps, regardless of their geometric direction or shape. The last
stroke ends with `RAMP_DOWN -> FINAL_RETRACT`.

`moveit_executor` executes one mode at a time and advances only after the
current trajectory action or force ramp reports completion. It publishes:

- `/painting_admittance/mode`
- `/painting_admittance/desired_force_n`
- `/painting_admittance/enable_force`

The timed `painting_segment_mode_node` is retained only for isolated bench
tests (`start_segment_mode_node:=false` by default). Do not run it alongside the
segment-aware executor because both nodes publish the same mode/force commands.

Safety behavior:

- Non-contact modes always command zero wrench.
- A `RAMP_DOWN` must complete before retract or travel.
- A `RAMP_UP` and, when configured, contact confirmation must complete before
  painting.
- `/painting_admittance/abort` is forwarded to `/motion_abort`.
- `/motion_abort` actively cancels an accepted `FollowJointTrajectory` goal and
  prevents the next segment from starting.
- Segment force is clamped by `max_paint_force_n`; unsafe ordering or clearance
  rejects the path before robot motion.

After TF and force sign validation, enable a very small test force:

```bash
ros2 topic pub /painting_admittance/desired_force_n std_msgs/msg/Float64 "{data: 1.0}" --once
ros2 topic pub /painting_admittance/enable_force std_msgs/msg/Bool "{data: true}" --once
ros2 topic pub /painting_admittance/mode std_msgs/msg/String "{data: 'RAMP_UP'}" --once
```

Diagnostics:

- `/painting_admittance/target_wrench_tcp`
- `/painting_admittance/target_wrench_ft`
- `/painting_admittance/current_mode`
- `/painting_admittance/command_force_tcp_y_n`
- `/painting_admittance/ramp_complete`
- `/painting_admittance/force_tcp_filtered`
- `/painting_admittance/normal_force_tcp_y`
- `/painting_admittance/contact_state`
- `/painting_admittance/contact_confirmed`
- `/painting_admittance/overforce_state`
- `/painting_admittance/abort_reason`
- `/painting_admittance/safety_status` (JSON)
- `/painting_admittance/wrench_guard_status` (JSON)
- `/painting_admittance/abort`
- `/motion_abort`

Safety faults remain latched. Reset `/painting_admittance/reset_safety` only
after IDLE, force-disabled, trajectory-inactive, stationary, free-space and
low-wrench interlocks are all true. Controller/guard latches can then be reset
with `/painting_admittance/reset_wrench_guard` after the upstream safety latch
has cleared.

Safety defaults:

- `dry_run:=true`, so the request node publishes no nonzero request.
- `real_painting_enabled:=false`, so the guard always publishes zero to the
  controller even if another input is accidentally enabled.
- `enable_force:=false`, so target force remains zero.
- `desired_contact_force_n:=0.0`.
- `max_command_force_n:=15.0`.
- `APPROACH_PRECONTACT`, `CONTACT_SEARCH`, `RETRACT`, `TRAVEL`,
  `FINAL_RETRACT`, `IDLE`, and `ABORT` always publish zero wrench and disable
  compliance.
- Only `RAMP_UP` and `PAINT` may pass nonzero TCP `force.y`, and only when every
  guard condition is valid.
- `RAMP_UP` and `RAMP_DOWN` change force smoothly.
- F/T noise is never used as the primary contact/non-contact decision.

Contact/non-contact is explicit process metadata. Do not infer it from
horizontal vs vertical, raster vs non-raster, stroke vs connector, or exact
`F/T == 0`. Raster/lawnmower patterns are only examples: any straight, curved,
vertical, diagonal, or spiral segment can be contact or non-contact depending
on its segment mode.

For controlled-contact painting, keep `variable_impedance` disabled unless it
has been explicitly tested for the painting profile. `admittance_use_case:=painting`
does this by default.

## Legacy FZI Backup

FZI cartesian compliance sources, launch files, and example path CSVs are kept
only as an archive under `legacy_fzi_backup/fzi/`. That directory contains
`COLCON_IGNORE`, so those packages are not discovered or built by this
workspace.

Do not launch the archived FZI path for the active sketch painting workflow.
Use `painting_admittance_control.launch.py` and the ROS 2
`admittance_controller` chain instead.
