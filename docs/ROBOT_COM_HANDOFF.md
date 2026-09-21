# Robot Computer Handoff

이 문서는 시뮬레이션 PC에서 하던 대화를 로봇 PC에서 새 Codex 세션으로 이어가기 위한 인수인계 메모다.

## 현재 원격 상태

- Repository: `https://github.com/minjeasung/sketch_robot`
- Branch: `main`
- Latest pushed commit: `1629636 Add D405 EOAT refinement and real robot backend`
- `zed-isaac-sim` 내부 캐시/생성물 변경은 커밋하지 않았다.

## 현재 구현 요약

- RB10 MoveIt robot description에 EOAT를 고정 링크로 포함했다.
- TCP에는 `AFT200 -> RR-00A_B no-camera EOAT -> D405` 순서로 붙는다.
- AprilTag 기반 ZED-D405 extrinsic calibration 결과를
  `~/sketch_robot_ws/zed_d405_apriltag_calibration.json`에서 자동 로드한다.
  런타임 target/work-area 선택 자체는 AprilTag가 아니라 ZED/D405 perception을 쓴다.
- ZED는 전체 scene/target/work-area 후보 인식에 사용한다.
- 사용자가 웹 UI에서 `Set Target`, `Set Work Area`, `Send Path` 순서로 의도를 준다.
- `Set Target` 이후 D405가 작업대상 대표 지점들을 prescan/refinement 해서 target surface 전체 평면을 먼저 보정한다.
- `Set Work Area` 이후 D405가 작업영역 근처를 다시 prescan/refinement 해서 실제 경로 주변 평면/거리를 보정한다.
- MoveIt collision에는 robot/EOAT, table/camera mount/static objects, active work-area surface가 들어간다.
- ZED point cloud는 target/work-area plane 인식에 사용하고, 작업대상 외 잔여 cloud를 voxel obstacle로 자동 등록하는 기능은 기본 비활성이다.
- 시뮬레이션 제어 토픽은 `/isaac_joint_command`로 분리했다.
- 실제 로봇 제어는 `/joint_trajectory_controller/follow_joint_trajectory` action을 사용해야 한다.

## 로봇 PC에서 업데이트

```bash
cd ~/sketch_robot_ws
git pull origin main

source /opt/ros/jazzy/setup.bash
source ~/rb10_ws/install/setup.bash

colcon build --packages-select eoat_description sketch_control
source ~/sketch_robot_ws/install/setup.bash
```

어드미턴스 제어를 쓸 때는 `src/rbpodo_ros2`의 수정본도 같은 workspace overlay에
빌드되어 있어야 한다. 그렇지 않으면 `~/rb10_ws/install`의 예전 rbpodo 패키지를
잡아서 `controllers_admittance.yaml`, F/T sensor interface, helper node가 실행에
반영되지 않을 수 있다.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/sketch_robot_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-select \
  rbpodo_msgs rbpodo_description rbpodo_hardware rbpodo_bringup rbpodo_moveit_config \
  eoat_description sketch_control
source ~/sketch_robot_ws/install/setup.bash
```

## 실로봇 실행 시 필수 차이

시뮬레이션과 다르게 아래 값이 중요하다.

```bash
use_isaac_sim:=false
use_sim_time:=false
execution_backend:=follow_joint_trajectory
```

`moveit_executor` 로그 첫 부분에 반드시 아래처럼 떠야 한다.

```text
execution_backend=follow_joint_trajectory
```

그렇지 않으면 실로봇이 아니라 시뮬레이션용 joint command backend로 실행된 것이다.

## 터미널별 실행

### Terminal 1: MoveIt full + RViz + 실제 RB10

```bash
source /opt/ros/jazzy/setup.bash
source ~/rb10_ws/install/setup.bash
source ~/sketch_robot_ws/install/setup.bash

ros2 launch sketch_control rb10_moveit_full.launch.py \
  robot_ip:=10.0.2.7 \
  use_isaac_sim:=false \
  use_sim_time:=false
```

공식 `rbpodo_moveit_config moveit.launch.py`는 RB10 단독 MoveIt/admittance 확인용으로만 쓴다.
스케치 로봇 실행은 EOAT/D405 URDF, custom SRDF, 카메라 RViz 설정, world/link0 bridge가 들어간
`sketch_control rb10_moveit_full.launch.py`를 기준으로 한다.

스케치 로봇의 force setup/일반 경로 실행에서는 raw ros2_control admittance를 켜지 않는다.
`use_admittance:=true`는 `/force_torque_sensor_broadcaster/wrench`의 6축 raw F/T에
직접 반응하므로, 벽 접촉/손가이드 중 로봇이 의도치 않게 밀려날 수 있다.
normal-aware force target은 Terminal 2의 `ft_normal_controller`와 `moveit_executor`
force guard 쪽에서 다룬다.

```bash
ros2 launch sketch_control rb10_moveit_full.launch.py \
  robot_ip:=10.0.2.7 \
  use_isaac_sim:=false \
  use_sim_time:=false \
  use_admittance:=false \
  human_collab:=true
```

raw admittance controller 체인을 실험할 때만 같은 launch에 아래 옵션을 명시적으로 추가한다.
실제 작업면 접촉 스케치 실행에는 쓰지 않는다.

```bash
ros2 launch sketch_control rb10_moveit_full.launch.py \
  robot_ip:=10.0.2.7 \
  use_isaac_sim:=false \
  use_sim_time:=false \
  use_admittance:=true \
  admittance_profile:=default \
  human_collab:=true
```

실로봇 없이 controller 체인만 확인할 때는 fake hardware + fake F/T로 띄운다.

```bash
ros2 launch sketch_control rb10_moveit_no_rviz.launch.py \
  use_fake_hardware:=true \
  use_isaac_sim:=false \
  use_sim_time:=false \
  fake_sensor_commands:=true \
  use_admittance:=true \
  human_collab:=true
```

### Terminal 2: real cameras + perception/sketch

실제 로봇에서는 Isaac Sim 이 depth/pointcloud 를 대신 만들지 않는다.
`rb10_real_perception_sketch.launch.py`가 아래 실제 카메라 wrapper를 같이 띄운다.

- ZED: `zed_wrapper` / ZED SDK
- D405: `realsense2_camera` / librealsense2
- AFT200: RB controller `SystemState.eft` -> `/aft200/ft`

simulation depth 변환 노드는 자동으로 꺼진다.
ZED는 외부 고정 카메라로만 쓰므로 wrapper의 positional tracking은 끄고,
`depth.depth_stabilization:=0`으로 depth stabilization도 꺼서 TF 대기로 멈추지 않게 한다.
AFT200이 로봇에 연결된 구성에서는 RB controller 가 외부 F/T 값을
`/rbpodo_hardware/system_state`의 `eft` 필드로 제공하고, `rbpodo_eft_bridge`가
이를 `/aft200/ft`로 변환한다. Terminal 1을 재시작해야 patched rbpodo hardware의
`/rbpodo_hardware/system_state` publisher가 뜬다.
직접 Ethernet으로 AFT200을 PC에 연결한 경우에만 `launch_aft_ethernet_driver:=true`를 쓴다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/rb10_ws/install/setup.bash
source ~/sketch_robot_ws/install/setup.bash

ros2 launch sketch_control rb10_real_perception_sketch.launch.py \
  zed_camera_model:=zed2i \
  d405_cloud_topic:=/d405/d405/depth/color/points \
  use_ft_normal_controller:=true
```

`use_ft_normal_controller:=true`를 빼먹으면 FT 노드가 뜨지 않아
moveit_executor가 `[FT REQUIRED] /ft/status 미수신`으로 모든 실행을 중단한다.

기본 perception 흐름은 시작 시 ZED point cloud를 5초 누적해 plane 후보만 갱신한다.
작업대상 외 plane/residual point를 voxel obstacle로 자동 등록하지 않는다.
수동 재스캔은 plane 후보를 다시 만들 때만 아래처럼 한다.

```bash
ros2 topic pub --once /perception/scan_trigger std_msgs/msg/Empty {}
```

스캔 시간을 늘리고 싶으면 Terminal 2 launch에 아래 값을 추가한다.

```bash
scene_scan_accumulation_sec:=8.0
```

ZED 잔여 point obstacle은 보수적인 planning을 만들 수 있어 기본 제거했다.
정말 필요할 때만 아래처럼 명시적으로 켠다.

```bash
publish_scene_obstacles:=true
scene_obstacle_voxel_m:=0.07
```

웹 UI의 `Set Target`은 `/target_selection_pixels`를 보낸 뒤
`/refine_target_surface`를 요청한다. `moveit_executor`가 D405를 target 중심/상하좌우
대표 지점으로 이동해 `/d405/refine_target_capture`를 보내고,
`d405_surface_refiner`가 `/perception/target_surface_refined`를 발행하면
`wall_projector`는 이 보정 target 기준으로 work-area/front-view를 만든다.

`Set Work Area`는 기존처럼 `/refine_work_area`를 요청하고, 결과는
`/perception/work_area_plane_refined`로 들어와 실제 경로를 다시 보정한다.

카메라 serial 을 고정해야 하면 아래처럼 추가한다.

```bash
ros2 launch sketch_control rb10_real_perception_sketch.launch.py \
  zed_camera_model:=zed2i \
  zed_serial_number:=YOUR_ZED_SERIAL \
  d405_serial_no:=YOUR_D405_SERIAL
```

이미 별도 터미널에서 ZED/D405 wrapper 를 직접 띄운 경우에만 기존 perception launch 를 사용한다.

```bash
ros2 launch sketch_control rb10_perception_sketch.launch.py \
  use_sim_depth_pointcloud:=false \
  use_sim_d405_depth_pointcloud:=false \
  use_d405_optical_tf:=false \
  d405_cloud_topic:=/d405/d405/depth/color/points
```

### Terminal 3: moveit executor

```bash
source /opt/ros/jazzy/setup.bash
source ~/rb10_ws/install/setup.bash
source ~/sketch_robot_ws/install/setup.bash

ros2 run sketch_control moveit_executor --ros-args \
  -p use_sim_time:=false \
  -p execution_backend:=follow_joint_trajectory \
  -p enable_zed_scene_obstacles:=false \
  -p contact_control_mode:=servo
```

`contact_control_mode:=servo`가 없으면 sampled(청크 재계획) 모드로 뜬다.
접촉 방식은 `contact_servo_normal_mode`로 고른다 (servo 모드에서만 동작):

- `penetration`(기본): 힘 피드백 없이 표면 안쪽 고정 침투(기본 5mm,
  `contact_servo_penetration_m`). 도포량은 롤러 스펀지 압축량으로 제어하고
  F/T는 과압 가드 전용. 기동 로그에
  `contact_servo_normal_mode=penetration(pen=5.0mm)` 확인.
- `force`: 기존 힘 서보(target_force_n 추종)로 복귀.

### Terminal 4: rosbridge

```bash
source /opt/ros/jazzy/setup.bash
source ~/rb10_ws/install/setup.bash
source ~/sketch_robot_ws/install/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

### Terminal 5: web UI

```bash
cd ~/sketch_robot_ws/web
python3 -m http.server 8000
```

Open:

```text
http://localhost:8000
```

## 실행 전 확인

```bash
ros2 action list | grep follow_joint_trajectory
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 topic echo /zed/zed_node/rgb/color/rect/image --once
ros2 topic echo /zed/zed_node/depth/depth_registered --once
ros2 topic echo /rbpodo_hardware/system_state --once
ros2 topic echo /aft200/status --once
ros2 topic hz /aft200/ft
ros2 topic echo /ft/status --once
ros2 topic info /d405/d405/depth/color/points -v
ros2 topic info /zed/zed_node/point_cloud/cloud_registered -v
ros2 run tf2_ros tf2_echo World zed_left_camera_frame
ros2 run tf2_ros tf2_echo zed_left_camera_frame zed_left_camera_frame_optical
ros2 run tf2_ros tf2_echo tcp d405_link
```

D405 point cloud topic 이름이 다르면 Terminal 2의 `d405_cloud_topic:=...` 값을 실제 topic으로 바꾼다.
ZED 토픽 이름은 기존 perception/web UI가 `/zed/zed_node/...` prefix 를 기준으로 구독한다.

## 안전 체크

- 첫 실로봇 테스트는 경로를 짧게 그린다.
- E-stop과 teach pendant 정지 버튼을 손 닿는 곳에 둔다.
- `Set Work Area` 뒤 D405 prescan이 이상한 큰 관절 우회 동작을 만들면 즉시 중지한다.
- FT sensor 영점은 접촉 전에 잡혀야 한다. 현재 코드는 sketch 시작 전 자동 zero 흐름을 포함한다.
- 실로봇 `follow_joint_trajectory` backend 에서는 `/ft/status` 가 없거나 stale 이면 접촉 실행을 중단한다.
- 작업면 접촉은 Stage 2/3에서만 허용한다. Stage 1 접근과 Stage 4 이탈에서는 작업면과 안전거리를 유지해야 한다.

## 새 Codex 세션에 붙여넣을 요약 프롬프트

```text
나는 sketch_robot_ws를 실제 RB10 로봇컴에서 실행하려고 한다.
repo는 https://github.com/minjeasung/sketch_robot, main 최신 커밋은
1629636 Add D405 EOAT refinement and real robot backend 이다.

현재 시스템은 RB10 + AFT200 force sensor + RR-00A_B roller EOAT + D405 + ZED이다.
ZED로 scene/target/work-area를 보고, 사용자가 web UI에서 Set Target, Set Work Area,
Send Path를 누른다. Set Target 후 D405 prescan/refinement로 target surface 전체 평면을
먼저 보정하고, Set Work Area 후 D405 prescan/refinement로 실제 경로 주변 평면/거리를
다시 보정한다.
MoveIt은 robot/EOAT 및 장애물 collision을 관리한다.

시뮬레이션에서는 /isaac_joint_command를 쓰지만, 실제 로봇에서는 반드시
moveit_executor를 execution_backend:=follow_joint_trajectory 로 실행해야 한다.
MoveIt launch도 use_isaac_sim:=false, use_sim_time:=false 로 실행해야 한다.

docs/ROBOT_COM_HANDOFF.md를 읽고, 실제 로봇컴에서 안전하게 실행/디버깅을 이어가자.
```
