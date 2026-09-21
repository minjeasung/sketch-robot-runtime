# RB10 painting safety overlay provenance

## Baseline

This workspace package is an overlay of the ROS 2 Jazzy
`admittance_controller` 4.40.1 source package. The package version is
intentionally unchanged so the upstream API baseline remains visible.

- Debian source: `ros-jazzy-admittance-controller 4.40.1-1noble`
- Installed binary at verification time:
  `ros-jazzy-admittance-controller 4.40.1-1noble.20260615.170327`
- ROS source pool object:
  `pool/main/r/ros-jazzy-admittance-controller/ros-jazzy-admittance-controller_4.40.1.orig.tar.gz`
- Upstream tarball SHA-256:
  `80ad58815505e4b24009e2ef8e04dcc07293c8824b595bee13c620aee219e3e3`
- Upstream release date recorded in `CHANGELOG.rst`: 2026-05-12

The provenance values above were checked with `dpkg-query`, `apt-cache
showsrc`, `package.xml`, and `CHANGELOG.rst` on 2026-08-10.

## Local safety delta

The overlay retains the upstream plugin class and existing ROS interfaces, and
adds the following fail-closed behavior for the RB10 painting system:

- steady-clock timeout for `~/wrench_reference`;
- explicit, heartbeat-timed `~/compliance_enable` gate, disabled by default;
- `~/compliance_active` and `~/normal_limit_reached` status publishers;
- finite checks for the sensor wrench and requested wrench before compliance;
- preservation of hardware NaN through `read_state_from_hardware()` so stale,
  read-error, or tare-invalid F/T cannot be mistaken for a valid zero wrench;
- bounded normal-axis trim, Cartesian velocity, and acceleration with a
  diagnostic limit flag;
- zero excitation with spring/damping return whenever compliance is gated off;
- a Jazzy build fallback when the optional `ros2_control_cmake` helper is not
  installed.

The raw `normal_limit_reached` signal is deliberately diagnostic. The painting
guard/monitor outside this package qualify it continuously for 0.25 s in a
force-control mode before latching an abort; a one-cycle acceleration or
velocity clamp is not itself a fault.

## Compatibility and deployment

No existing topic name, message type, parameter name, or pluginlib class name
was removed. The added parameters and topics are additive ROS API. The C++
controller class has additional internal/protected state, so do not mix a
binary compiled against the upstream headers with this overlay library. Build
and source the workspace as one unit so both headers and library resolve from
the same prefix.

The intentional behavior change is that a non-finite F/T state disables
compliance instead of being converted to 0 N. Other consumers of the hardware
state must therefore treat NaN as invalid telemetry. This is the fail-closed
contract used by the force-torque broadcaster and painting safety monitor.

Because `/opt/ros/jazzy` contains the upstream package, `colcon` prints an
override warning. Use `--allow-overriding admittance_controller` in automated
builds that promote override warnings to errors.

## Verification evidence

Commands run from `/home/Minjea/sketch_robot_ws` on 2026-08-10:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select admittance_controller rbpodo_hardware rbpodo_painting_control \
  --symlink-install --allow-overriding admittance_controller
source install/setup.bash
colcon test --packages-select admittance_controller rbpodo_hardware rbpodo_painting_control
```

Results at the time this note was written:

- build: all three selected packages completed successfully;
- `admittance_controller`: 40 gtest cases passed (39 controller cases plus one
  plugin-load case), 0 failures;
- the regression
  `nonfinite_ft_state_is_preserved_and_disables_compliance` verifies that NaN
  reaches the controller gate, compliance becomes inactive, and commands stay
  finite;
- companion `rbpodo_hardware`: four fail-closed/recovery gtests passed;
- companion `rbpodo_painting_control`: 36 safety/guard Python tests passed.

Compiler output contained only the existing Jazzy deprecation notices and the
generated-message default-constructor warning; there were no build errors.
