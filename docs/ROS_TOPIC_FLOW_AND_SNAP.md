# Sketch Robot ROS Topic Flow and Snap Logic

이 문서는 실제 RB10 스케치 로봇 실행 중 어떤 노드가 어떤 토픽을 발행/구독하는지,
그리고 웹 UI에서 하는 세 번의 스케치가 어떻게 3D 표면 위 점으로 변환되는지 정리한다.

## 1. 실행 단위

실제 로봇 실행은 보통 아래 터미널 묶음으로 나눈다.

```text
Terminal 1: RB10 + MoveIt
  ros2 launch sketch_control rb10_moveit_full.launch.py ...
  - robot_state_publisher
  - ros2_control / rbpodo hardware
  - joint_trajectory_controller
  - move_group
  - RViz

Terminal 2: real cameras + perception
  ros2 launch sketch_control rb10_real_perception_sketch.launch.py ...
  - zed_wrapper
  - realsense2_camera for D405
  - rbpodo_eft_bridge
  - environment_scanner
  - target_selector
  - wall_projector
  - sketch_to_waypoints
  - d405_surface_refiner
  - ft_normal_controller
  - 기본 F/T target은 ~/sketch_robot_ws/aft200_force_threshold.json 에서 읽는다.

Terminal 3: robot executor
  ros2 run sketch_control moveit_executor --ros-args \
    -p use_sim_time:=false \
    -p execution_backend:=follow_joint_trajectory \
    -p enable_zed_scene_obstacles:=false \
    -p enable_ft_admittance:=true

Terminal 4: rosbridge
  ros2 launch rosbridge_server rosbridge_websocket_launch.xml

Terminal 5: web UI server
  cd ~/sketch_robot_ws/web && python3 -m http.server 8000
  - Force Setup: http://localhost:8000/setup.html
  - Sketch UI  : http://localhost:8000/index.html
```

실제 로봇에서는 최종 로봇 명령이 `/isaac_joint_command`가 아니라
`/joint_trajectory_controller/follow_joint_trajectory` action으로 나가야 한다.

## 2. 큰 그림

```text
ZED camera
  ├─ /zed/zed_node/rgb/color/rect/image
  ├─ /zed/zed_node/rgb/color/rect/camera_info
  ├─ /zed/zed_node/depth/depth_registered
  ├─ /zed/zed_node/depth/camera_info
  └─ /zed/zed_node/point_cloud/cloud_registered
          │
          ├─ environment_scanner
          │    ├─ /perception/planes
          │    ├─ /perception/plane_labels
          │    └─ /perception/obstacles
          │         기본 비활성. scene voxel obstacle은 자동 등록하지 않는다.
          │
          ├─ target_selector
          │    └─ /perception/target_surface
          │
          └─ wall_projector
               ├─ /perception/wall_front_view
               ├─ /perception/work_area_plane
               └─ /perception/work_area_corners

D405 camera
  └─ /d405/d405/depth/color/points
          │
          └─ d405_surface_refiner
               ├─ /perception/target_surface_refined
               └─ /perception/work_area_plane_refined

Web UI
  ├─ setup.html
  │    ├─ /ft/zero
  │    ├─ /ft/target_config
  │    └─ /rbpodo_ft_tare/set_freedrive  (service call)
  └─ index.html
       ├─ /target_selection_pixels
       ├─ /refine_target_surface
       ├─ /work_area_pixels
       ├─ /refine_work_area
       ├─ /sketch_pixels
       └─ /sketch_execute

moveit_executor
  ├─ subscribes: target/work-area/path/refine/status topics
  ├─ subscribes: /ft/status, /ft/admittance_correction
  ├─ publishes : /planning_scene, /d405/refine_target_capture,
  │              /d405/refine_capture, /ft/zero
  └─ sends     : FollowJointTrajectory action
```

## 3. Scene Scan

`environment_scanner`는 ZED point cloud를 일정 시간 누적해서 큰 평면 후보를 만든다.

```text
/zed/zed_node/point_cloud/cloud_registered
        │
        v
environment_scanner
        ├─ /perception/planes
        ├─ /perception/plane_labels
        └─ /perception/obstacles
```

현재 기본 정책은 보수적이다.

- `publish_scene_obstacles:=false`가 기본값이다.
- ZED residual point cloud를 자동 voxel obstacle로 넣지 않는다.
- MoveIt collision은 robot/EOAT, table, camera, camera mount, active surface 중심으로 쓴다.
- 수동 재스캔은 아래 토픽으로 한다.

```bash
ros2 topic pub --once /perception/scan_trigger std_msgs/msg/Empty {}
```

## 4. Sketch 1: Target Selection

사용자는 ZED raw 화면에서 작업대상을 대략 그린 뒤 `Set Target`을 누른다.

```text
Web UI, ZED Raw, Target mode
  -> /target_selection_pixels     PoseArray, frame_id="zed_raw"
  -> /refine_target_surface       Bool(true)
```

`target_selector`는 ZED depth image와 intrinsics를 사용해서 선택 영역의 3D 점들을 만든다.

```text
/target_selection_pixels
/zed/zed_node/depth/depth_registered
/zed/zed_node/depth/camera_info
        │
        v
target_selector
        └─ /perception/target_surface
```

여기서 target sketch는 개별 stroke 점을 모두 벽에 붙이는 것이 아니라,
stroke를 감싸는 ROI를 잡고 그 영역의 depth point로 대표 평면을 추정한다.

### Target Snap 방식

1. 사용자가 그린 픽셀들의 bounding box를 잡는다.
2. 주변 padding을 더한다.
3. ROI 안의 depth pixel을 stride 간격으로 샘플링한다.
4. 각 pixel `(u, v)`와 depth `z`를 카메라 3D 점으로 바꾼다.

```text
X = (u - cx) * z / fx
Y = (v - cy) * z / fy
Z = z
```

5. 얻은 3D 점들에 RANSAC plane fitting을 한다.
6. inlier 평균점을 plane centroid로 쓰고, plane normal을 quaternion으로 저장한다.
7. 결과를 `/perception/target_surface`로 발행한다.

즉, 첫 번째 스케치의 목적은 "벽 위의 한 경로"가 아니라
"작업대상 표면을 고르는 것"이다.

## 5. D405 Target Refinement

`Set Target` 이후 `moveit_executor`는 D405로 target surface를 한 번 더 본다.

```text
/perception/target_surface
/refine_target_surface
/joint_states
        │
        v
moveit_executor
        ├─ D405가 target 중심/상/하/좌/우를 보도록 로봇 이동
        ├─ /d405/refine_target_capture
        └─ /target_refine_status
```

`d405_surface_refiner`는 D405 point cloud에서 target plane 주변 ROI를 잡아 다시 plane fitting한다.

```text
/d405/d405/depth/color/points
/perception/target_surface
/d405/refine_target_capture
        │
        v
d405_surface_refiner
        └─ /perception/target_surface_refined
```

이후 `wall_projector`와 `moveit_executor`는 가능하면
`/perception/target_surface_refined`를 target 기준 surface로 사용한다.

## 6. Sketch 2: Work Area Selection

사용자는 다시 ZED raw 화면에서 실제 그림을 그릴 작업영역을 그린다.

```text
Web UI, ZED Raw, Work Area mode
  -> /work_area_pixels       PoseArray, frame_id="zed_raw"
  -> /refine_work_area       Bool(true)
```

`wall_projector`는 현재 target surface, 가능하면 D405 보정 target surface를 기준으로
work area 네 모서리를 3D로 만든다.

```text
/work_area_pixels
/perception/target_surface_refined
/zed/zed_node/rgb/color/rect/image
/zed/zed_node/rgb/color/rect/camera_info
        │
        v
wall_projector
        ├─ /perception/wall_front_view
        ├─ /perception/work_area_plane
        └─ /perception/work_area_corners
```

### Work Area Snap 방식

work area는 "픽셀 ray와 target plane의 교차점"으로 붙는다.

1. work area sketch의 bounding box를 만든다.
2. 노란 테이프 사각형이 감지되고 sketch 위치와 맞으면, yellow quad를 우선 쓴다.
3. yellow quad가 없으면 sketch bounding box의 네 모서리를 쓴다.
4. 각 corner pixel `(u, v)`에서 카메라 ray를 만든다.

```text
ray = inverse(K) * [u, v, 1]
```

5. target plane의 한 점 `p0`, normal `n`에 대해 ray와 plane의 교차점을 구한다.

```text
t = dot(p0, n) / dot(ray, n)
p = ray * t
```

6. 네 교차점이 `/perception/work_area_corners`가 된다.
7. 네 모서리 평균이 `/perception/work_area_plane`의 position이 된다.
8. 이 영역을 perspective transform해서 `/perception/wall_front_view`를 만든다.

즉, 두 번째 스케치는 실제 3D 작업영역 네 모서리를 target surface 위에 snap한다.

## 7. D405 Work Area Refinement

`Set Work Area` 이후에는 D405가 실제 작업영역 주변을 다시 본다.

```text
/perception/work_area_plane
/perception/work_area_corners
/refine_work_area
/joint_states
        │
        v
moveit_executor
        ├─ D405가 work area 주변 sample pose로 이동
        ├─ /d405/refine_capture
        └─ /work_area_refine_status
```

```text
/d405/d405/depth/color/points
/perception/work_area_plane
/perception/work_area_corners
/d405/refine_capture
        │
        v
d405_surface_refiner
        └─ /perception/work_area_plane_refined
```

`wall_projector`는 locked work area corners를 D405 refined plane에 다시 투영한다.

```text
signed = dot(corner - refined_point, refined_normal)
corner_refined = corner - signed * refined_normal
```

그래서 work area의 이미지 좌표 의미는 유지하면서, 3D 위치와 normal만 D405 기준으로 보정된다.

## 8. Optional Force Setup

Force Setup은 스케치 UI와 분리된 `web/setup.html`에서 필요할 때만 수행한다.
롤러, 벽, 도포 조건, 목표 접촉력이 바뀐 경우에만 이 페이지에서 target/abort를 다시
저장하고, 일반 스케치 실행은 `web/index.html`에서 저장된 값을 그대로 사용한다. 이 단계의
목적은 이후 normal-aware F/T layer가 쓸 목표 접촉력을 실제 작업면 normal 기준으로 다시
잡는 것이다.
스케치 실행에서는 Terminal 1의 raw ros2_control admittance(`use_admittance:=true`)를
켜지 않는다. raw admittance는 6축 F/T 크기에 직접 반응하므로 작업면 접촉 중 의도치 않은
로봇 움직임을 만들 수 있다.

```text
Force Setup UI, setup.html
  ├─ /ft/zero                          Bool(true)
  ├─ /rbpodo_ft_tare/set_freedrive      SetBool(true/false)
  ├─ subscribes /ft/status
  └─ /ft/target_config                  String(JSON)
```

작업자는 다음 순서로 설정한다.

```text
Zero F/T
Free Drive ON
손으로 롤러를 작업면에 누름
Capture Target  → 현재 /ft/status.normal_force_n 을 target 으로 저장
Free Drive OFF
Done
```

`Capture Target`은 현재 normal force `Fn`의 절댓값을 target으로 쓰고,
음수로 눌리는 경우에는 `force_sign`을 자동 반전해서 `/ft/target_config`로 보낸다.
UI가 보내는 JSON은 아래 필드를 포함한다.

`Free Drive OFF` 후에는 웹 UI가 `/rbpodo_admittance_helper/reset_admittance`를
best-effort로 호출한다. raw admittance를 끈 일반 스케치 실행에서는 이 서비스가 없어도
정상이며, UI는 그대로 완료 처리한다.

```json
{
  "force_sign": 1.0,
  "target_force_n": 6.5,
  "contact_threshold_n": 2.0,
  "warn_force_n": 9.6,
  "abort_force_n": 12.0
}
```

일반 스케치 UI는 Force Setup 단계를 포함하지 않는다. `Send Path`, `Fill Work Area`,
`Run Robot`은 현재 `/ft/status`의 target/abort 설정을 그대로 사용한다.

웹 UI가 `/ft/target_config`를 보내면 `ft_normal_controller`는 같은 값을
`~/sketch_robot_ws/aft200_force_threshold.json`에 저장한다. 다음 Terminal 2 실행은 이
파일을 읽어 `ft_force_sign`, `ft_target_force_n`, `ft_contact_threshold_n`,
`ft_warn_force_n`, `ft_abort_force_n`의 launch 기본값으로 사용한다.

## 9. Sketch 3: Path Drawing

사용자는 `Wall Front` 화면에서 실제 그릴 경로를 그린 뒤 `Send Path` 또는 `Execute`를 누른다.

```text
Web UI, Wall Front, Path mode
  -> /sketch_pixels       PoseArray, frame_id="wall_front"
```

`sketch_to_waypoints`는 wall front 픽셀을 work area 3D 사각형 안의 점으로 변환한다.

```text
/sketch_pixels
/perception/wall_front_view
/perception/work_area_plane_refined
/perception/work_area_corners
        │
        v
sketch_to_waypoints
        ├─ /sketch_waypoints
        └─ /sketch_markers
```

웹 UI는 path mode에서 `/perception/work_area_corners`로 계산한 실제 작업영역 폭을 사용해
RR-00A_B roller 길이 220mm를 wall_front pixel 단위로 환산한다. 이 값은 경로 명령을
바꾸지 않고, 사용자가 centerline을 그릴 때 롤러가 차지하는 폭을 볼 수 있게 하는
시각 보조 overlay로만 쓰인다. overlay 막대는 path tangent에 수직으로 그려서
실제 롤러 가로축 방향을 보여준다.

### Path Snap 방식

path sketch는 이미 보정된 wall front image 위에서 그려진다.
따라서 각 path pixel은 work area 3D corner 네 점 사이의 bilinear interpolation으로 표면점이 된다.

```text
su = u / (view_width  - 1)
sv = v / (view_height - 1)

top    = TL + (TR - TL) * su
bottom = BL + (BR - BL) * su
p_surface = top + (bottom - top) * sv
```

이 `p_surface`가 진짜 작업면 위의 점이다.
하지만 로봇 waypoint는 roller 중심점이므로 표면에서 normal 방향으로 약간 띄운다.

```text
EOAT_SURFACE_OFFSET = ROLLER_RADIUS + CONTACT_CLEARANCE
                    = 0.026 + 0.005
                    = 0.031 m

p_waypoint = p_surface + normal * EOAT_SURFACE_OFFSET
```

즉, path는 벽 표면점에 snap된 뒤,
실제 MoveIt waypoint는 롤러 반지름만큼 표면 밖으로 offset된 위치가 된다.

orientation은 모든 waypoint에서 다음 원칙을 따른다.

```text
tcp local -Y axis = 작업면으로 접근하는 방향
tcp local +X axis = roller long axis

roller long axis ⟂ path tangent
roller long axis ⟂ surface normal
```

그래서 EOAT roller는 항상 작업면과 평행하게 닿고, 경로 진행 방향으로 구를 수 있도록
각 waypoint마다 roll orientation이 달라질 수 있다.

## 10. Run Robot

경로가 `/sketch_waypoints`로 만들어진 뒤 사용자가 `Run Robot`을 누르면:

```text
Web UI
  -> /sketch_execute Bool(true)
        │
        v
moveit_executor
        ├─ /planning_scene 갱신
        ├─ /ft/zero
        ├─ MoveIt planning
        ├─ Stage 3 chunk마다 /ft/admittance_correction 반영
        └─ /joint_trajectory_controller/follow_joint_trajectory
```

`moveit_executor`는 다음 입력들도 함께 본다.

```text
/joint_states
/perception/target_surface_refined
/perception/work_area_plane_refined
/perception/work_area_corners
/sketch_waypoints
/ft/status
/ft/admittance_correction
```

실로봇 backend에서 `/ft/status`가 없거나 stale이면 접촉 실행을 중단한다. Stage 3 접촉
경로는 긴 trajectory 하나로 보내지 않고 짧은 Cartesian chunk로 나누며, 각 chunk 시작 전에
`/ft/admittance_correction`의 base-frame 보정 벡터를 TCP waypoint에 더한다. 이 방식은
raw ros2_control admittance가 아니라 normal-aware sampled admittance이다.

## 11. AFT200 / Force Flow

RB controller가 주는 `SystemState.eft`를 표준 wrench topic으로 바꾼다.

```text
/rbpodo_hardware/system_state
        │
        v
rbpodo_eft_bridge
        ├─ /aft200/ft
        └─ /aft200/status
```

`ft_normal_controller`는 표면 normal 기준의 normal force를 계산한다.

```text
/aft200/ft
/perception/work_area_plane_refined
/ft/target_config
        │
        v
ft_normal_controller
        ├─ /ft/status
        ├─ /ft/contact
        ├─ /ft/normal_force
        └─ /ft/admittance_correction
```

`/ft/target_config`는 웹 UI Force Setup이 보낸 런타임 force 설정이다.
`ft_normal_controller`는 이를 받아 `target_force_n`, `contact_threshold_n`,
`warn_force_n`, `abort_force_n`, `force_sign`을 즉시 갱신한다.
`/ft/admittance_correction`은 `normal_force_n - target_force_n` 오차를 작업면 normal
방향 base-frame 보정 벡터로 변환한 값이며, `moveit_executor`가 Stage 3 chunk 실행 전에
최대 보정량 안에서 사용한다.

## 12. AprilTag ZED-D405 Calibration

캘리브레이션은 런타임 target/work-area 선택과 분리된 절차다.
목적은 `World/link0` 기준에서 고정 ZED의 위치와 방향을 구하는 것이다.

```text
ZED image/info
  ├─ /zed/zed_node/rgb/color/rect/image
  └─ /zed/zed_node/rgb/color/rect/camera_info

D405 image/info
  ├─ /d405/d405/color/image_raw
  └─ /d405/d405/color/camera_info

Robot TF
  ├─ World -> link0
  └─ link0 -> ... -> tcp -> d405_color_optical_frame

AprilTag calibrator
  └─ ~/sketch_robot_ws/zed_d405_apriltag_calibration.json
```

캘리브레이터는 같은 AprilTag를 ZED와 D405가 동시에 본다고 가정한다.

```text
T_D405_tag : D405 image에서 solvePnP로 구함
T_ZED_tag  : ZED image에서 solvePnP로 구함
T_base_D405: robot TF에서 구함

T_base_tag = T_base_D405 * T_D405_tag
T_base_ZED = T_base_tag * inverse(T_ZED_tag)
T_world_ZED = T_world_base * T_base_ZED
```

결과 JSON에는 보통 아래 값이 저장된다.

```text
T_world_zed_optical
T_base_zed_optical
sample spread
outlier rejection result
```

런타임 perception launch는 이 JSON을 읽어서 static TF를 발행한다.

```text
World -> zed_left_camera_frame
zed_left_camera_frame -> zed_left_camera_frame_optical
```

이 TF가 있어야 ZED가 본 target/work-area/path가 robot/MoveIt 좌표계로 해석된다.

## 13. 빠른 디버그 토픽

```bash
ros2 topic hz /zed/zed_node/rgb/color/rect/image
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
ros2 topic hz /d405/d405/depth/color/points

ros2 topic echo /perception/target_surface --once
ros2 topic echo /perception/target_surface_refined --once
ros2 topic echo /perception/work_area_plane --once
ros2 topic echo /perception/work_area_plane_refined --once
ros2 topic echo /perception/work_area_corners --once

ros2 topic echo /target_refine_status
ros2 topic echo /work_area_refine_status

ros2 topic echo /sketch_waypoints --once
ros2 topic hz /aft200/ft
ros2 topic echo /ft/status --once

ros2 run tf2_ros tf2_echo World zed_left_camera_frame
ros2 run tf2_ros tf2_echo tcp d405_d405_link
ros2 run tf2_ros tf2_echo tcp d405_color_optical_frame
```
