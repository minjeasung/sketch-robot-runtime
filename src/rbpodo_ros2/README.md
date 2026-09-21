# rbpodo_ros2

> :warning: **IMPORTANT WARNING**: This software is under active development. DO NOT USE in production to avoid potential instability.


## Installation

### Prerequisites

- Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Install [rbpodo](https://github.com/RainbowRobotics/rbpodo)
  ```bash
  sudo apt install -y build-essential cmake git
  git clone https://github.com/RainbowRobotics/rbpodo.git
  mkdir -p rbpodo/build
  cd rbpodo/build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make
  sudo make install
  ```
- Install ROS 2 package dependencies
  ```bash
  sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-joint-state-publisher \
    ros-humble-moveit \
    ros-humble-pluginlib \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control \
    ros-humble-rviz2 \
    ros-humble-urdf-launch \
    ros-humble-xacro 
  ```
- Set up environment
  ```bash
  source /opt/ros/humble/setup.bash
  ```

### Build From Source

1. Create a ROS 2 workspace
   ```bash
   mkdir -p ~/rbpodo_ros2_ws/src
   ```
2. Clone repo and build ``rbpodo_ros2`` packages:
   ```bash
   cd ~/rbpodo_ros2_ws
   git clone https://github.com/RainbowRobotics/rbpodo_ros2.git src/rbpodo_ros2
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.sh
   ```

## How to Use

```bash
source ~/rbpodo_ros2_ws/install/setup.bash
ros2 launch rbpodo_bringup rbpodo.launch.py model_id:=rb3_730es_u use_rviz:=true
```

```bash
source ~/rbpodo_ros2_ws/install/setup.bash
ros2 launch rbpodo_moveit_config moveit.launch.py model_id:="rb5_850e" robot_ip:="10.0.2.7" use_fake_hardware:=false
```

## Painting roller compliance control

The existing `admittance_controller` mode is a zero-force / free-push workflow.
It is useful for hand compliance, but it is not by itself a constant roller
pressure controller.

For painting/contact tests, prefer the ROS 2 `admittance_controller` chain so
MoveIt/JTC remains the nominal trajectory source:

```bash
ros2 launch rbpodo_moveit_config moveit.launch.py \
  model_id:=rb10_1300e_u \
  robot_ip:=10.0.2.7 \
  use_fake_hardware:=false \
  fake_sensor_commands:=false \
  use_admittance:=true \
  admittance_use_case:=painting \
  admittance_profile:=painting_normal_y
```

This loads `controllers_admittance.yaml` plus
`admittance_profiles/painting_normal_y.yaml` and should activate:

- `joint_state_broadcaster`
- `force_torque_sensor_broadcaster`
- `admittance_controller`
- `joint_trajectory_controller`

The intended chain is:

```text
MoveIt / JointTrajectoryController -> admittance_controller -> rbpodo_hardware
```

Painting wrench-reference publication is split into a separate package:

```bash
ros2 launch rbpodo_painting_control painting_admittance_control.launch.py \
  dry_run:=true \
  enable_force:=false
```

Frame convention:

- `link0` is the target pose base frame.
- TCP `+Y` is aligned with the outward surface normal.
- TCP `-Y` is the pressing direction into the surface.
- `ft_link` is the measured F/T frame.

If measured roller contact reaction is positive along TCP `+Y`, request a
normal reaction `F_ref` by publishing a conceptual TCP force
`target.force.y = -F_ref`; the painting node transforms it to `ft_link` before
publishing `/admittance_controller/wrench_reference`. Verify this sign at
1-2 N before any real painting contact.

First-contact sequence:

1. Start with `dry_run:=true` and `enable_force:=false`.
2. Verify `/force_torque_sensor_broadcaster/wrench` frame and sign by gently
   pushing the TCP.
3. Verify the path orientation: TCP `+Y` must point along the outward surface
   normal.
4. Run MoveIt/JTC motion with `painting_normal_y` and wrench reference still
   zero.
5. Try `enable_force:=true desired_contact_force_n:=1.0`.
6. Increase force gradually while tuning `painting_normal_y` mass, damping,
   stiffness, and `ft_sensor.filter_coefficient`.

The former FZI cartesian compliance path has been moved out of the active
workspace into `legacy_fzi_backup/`. Keep painting/contact tests on the
`admittance_controller` chain for controller consistency.
