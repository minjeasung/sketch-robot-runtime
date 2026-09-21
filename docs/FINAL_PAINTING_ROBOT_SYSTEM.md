# RB10 스케치 기반 어드미턴스 도장 로봇 최종 시스템

## 1. 문서 목적

이 문서는 `~/sketch_robot_ws`의 현재 구현을 기준으로, 실제 도장 작업에 사용할 단일 운용 구조를 설명한다. 대상은 RB10 로봇, 롤러 EOAT, AFT200 F/T 센서, 고정형 ZED 카메라, 손목 장착 D405 카메라, 웹 스케치 인터페이스, MoveIt 2, `ros2_control` 어드미턴스 컨트롤러를 포함한 전체 시스템이다.

문서의 목적은 다음 두 가지다.

1. 작업자가 최종 시스템의 데이터 흐름과 실행 순서를 한 문서에서 이해할 수 있게 한다.
2. 이 문서를 외부 검토자에게 전달하여 좌표계, 법선 추정, 접촉 전환, 제어 안정성, 안전 정지에 빠진 문제가 없는지 재검토할 수 있게 한다.

기준일은 2026-08-10이며, 현재 워크트리의 구현을 기준으로 작성했다. 실험 과정에서 파생된 별도 제어 경로는 이 문서의 운용 대상이 아니다.

## 2. 최종 시스템 한눈에 보기

```text
사용자
  |
  |  ZED Raw에서 대상 선택
  |  D405 Wall Front에서 작업영역 및 도장 선 입력
  v
웹 스케치 UI + rosbridge
  |
  +-> /target_selection_pixels
  +-> /work_area_pixels
  +-> /sketch_pixels 또는 /fill_work_area
  +-> /sketch_execute
  |
  v
ZED 전역 인식
  |- 전역 장면 스캔과 평면 후보
  |- 대상 ROI의 초기 표면점 및 초기 법선
  v
D405 근접 보정
  |- 대상/작업영역 근접 PointCloud2 RANSAC
  |- 표면 위치와 법선 보정
  |- 보정 평면 잠금
  v
sketch_to_waypoints_node
  |- /sketch_waypoints
  |- /sketch_eoat_segments  (version 2 JSON)
  v
moveit_executor
  |- D405 사전 측정 및 실행 직전 경로 재투영
  |- 세그먼트 상태 머신
  |- MoveIt 충돌 검사, IK, Cartesian path
  v
/joint_trajectory_controller/follow_joint_trajectory
  v
joint_trajectory_controller
  |- 명목 관절 궤적
  v
admittance_controller
  |- TCP Y축만 힘 순응
  |- 측정 wrench + 목표 wrench 사용
  v
rbpodo_hardware -> 실제 RB10
```

최종 설계 원칙은 다음과 같다.

- 접촉 의도는 F/T 값이 아니라 `PAINT`, `TRAVEL` 같은 경로 모드가 결정한다.
- ZED는 넓은 시야의 전역 기준을 만들고, D405는 가까운 작업면의 위치와 법선을 보정한다.
- 도장 스트로크 사이 연결 이동은 명시적인 비접촉 세그먼트다.
- MoveIt/JTC가 접선 방향 경로와 자세를 담당하고, ROS 2 어드미턴스 컨트롤러는 TCP Y 법선축만 순응시킨다.
- 힘 명령과 실제 wrench 측정은 서로 다른 채널이며, 둘 다 어드미턴스 컨트롤러에 들어간다.
- 작업 중 임의의 센서 법선으로 경로를 계속 흔들지 않는다. 실행 직전 보정된 하나의 작업 평면을 고정하여 사용한다.

## 3. 하드웨어 구성

| 구성 | 현재 역할 |
|---|---|
| Rainbow Robotics RB10-1300e-u | 6축 실제 로봇, 기본 IP `10.0.2.7` |
| AFT200 | EOAT 접촉 wrench 측정 |
| 롤러 EOAT | 실제 도장 접촉 도구 |
| ZED 2i | 고정형 전역 카메라, 대상 선택과 장면 평면 추정 |
| Intel RealSense D405 | TCP/EOAT 장착 근접 카메라, 작업면 위치와 법선 보정 |
| 작업 PC | ROS 2 Jazzy, MoveIt 2, ros2_control, 웹 UI 실행 |

### 3.1 롤러와 EOAT 형상

| 항목 | 값 |
|---|---:|
| 롤러 지름 | `0.052 m` |
| 롤러 반지름 | `0.026 m` |
| 롤러 길이 | `0.220 m` |
| MoveIt 접촉 여유 | `0.005 m` |
| 표면점에서 롤러 중심까지 명목 거리 | `0.031 m` |
| 스트로크 사이 추가 travel clearance | `0.010 m` |
| 첫 접근 전 safety offset | `0.080 m` |
| 작업 종료 후 최종 retreat offset | `0.080 m` |

세그먼트의 `(x, y, z)`는 롤러 중심이나 TCP 위치가 아니라 실제 표면점이다.

```text
roller_center = surface_point
              + surface_normal * (contact_offset_m + offset_m)

contact_offset_m = roller_radius + MoveIt_clearance
                 = 0.026 + 0.005
                 = 0.031 m
```

`offset_m`은 세그먼트별 추가 이격 거리다. `PAINT`에서는 보통 `0`, `RETRACT`와 `TRAVEL`에서는 현재 `0.010 m`다.

## 4. 좌표계와 부호 규약

### 4.1 주요 프레임

| 프레임 | 의미 |
|---|---|
| `world` | MoveIt launch가 제공하는 전역 프레임 |
| `World` | 인식 및 스케치 코드가 사용하는 전역 프레임, `world`와 identity bridge로 연결 |
| `link0` | MoveIt과 어드미턴스의 고정 기준 프레임, 실로봇에서는 `world`와 같은 축 |
| `tcp` | 로봇 TCP 및 어드미턴스 제어 프레임 |
| `ft_link` | AFT200 wrench 측정 프레임 |
| `zed_left_camera_frame[_optical]` | ZED 전역 카메라 프레임 |
| `d405_color_optical_frame` | D405 색상/광학 프레임 |

실로봇 launch에서는 `world -> link0`가 identity다. ZED 결과와 D405 결과는 TF를 통해 `World` 또는 ZED 기준에서 생성된 뒤, 최종 실행 전에 `link0`으로 변환된다.

### 4.2 표면 법선과 TCP 축

법선 `n`은 표면에서 자유 공간 쪽으로 향하는 outward normal이다.

```text
TCP +Y = surface outward normal n
TCP -Y = 표면을 누르는 방향 -n
TCP +X = 롤러의 긴 축
```

따라서 도장 목표 힘은 TCP 좌표에서 다음과 같다.

```text
target_wrench_tcp.force = [0, -force_n, 0]
```

현재 기본값은 `target_wrench_sign=-1.0`, `target_wrench_axis=force_y`다. 목표 wrench 노드는 이 TCP 힘을 `ft_link`로 회전 변환한 후 `/admittance_controller/wrench_reference`에 발행한다.

측정 접촉 반력은 정상적인 부호 설정에서 TCP `+Y`로 나타나야 한다. 접촉 확인 로직은 양의 TCP Y 힘을 사용하고, 과힘 판단은 절댓값을 사용한다.

## 5. 두 카메라의 역할 분담

두 카메라의 출력을 단순 평균하지 않는다. ZED가 전역 기준 평면을 만들고, D405가 그 기준 주변에서 더 정밀한 국소 평면을 다시 적합하는 계층형 구조다.

| 단계 | ZED | D405 |
|---|---|---|
| 전체 셀 관찰 | 담당 | 비담당 |
| 대상 선택 | ZED Raw 영상과 등록 depth 사용 | 선택 직후 현재 시야에서 보정 시도 |
| 초기 표면점/법선 | 담당 | ZED 평면을 기준으로 재추정 |
| 작업영역 정면 영상 | 기준 평면 제공 | 실제 color 영상을 평면에 정사영 |
| 작업영역 근접 법선 | 초기값 제공 | 최종 보정값 제공 |
| 실행 직전 확인 | fallback 기준 | fresh 보정값이 없으면 prescan 시도 |
| PAINT 중 실시간 법선 갱신 | 하지 않음 | 하지 않음 |

## 6. 카메라 캘리브레이션과 TF

### 6.1 ZED eye-to-hand

기본 캘리브레이션 파일:

```text
~/sketch_robot_ws/zed_d405_apriltag_calibration.json
```

기본 pose key:

```text
T_world_zed_optical
```

launch는 이 optical pose를 ROS 카메라 축 규약에 맞게 변환하여 다음 TF를 구성한다.

```text
World -> zed_left_camera_frame -> zed_left_camera_frame_optical
```

ZED 드라이버 자체 pose tracking TF는 사용하지 않고, 보정 파일에서 얻은 정적 외부 캘리브레이션을 전역 기준으로 사용한다.

### 6.2 D405 eye-in-hand

기본 캘리브레이션 파일:

```text
~/sketch_robot_ws/d405_eyeinhand_charuco_calibration.json
```

기본 pose key:

```text
T_d405_optical_to_tcp
```

파일의 optical pose를 RealSense optical 고정 회전과 결합하여 `tcp -> d405_link` 계열 TF를 만든다. 로봇 관절 TF와 결합하면 D405 cloud를 ZED 또는 `link0` 프레임으로 변환할 수 있다.

```text
link0 -> ... -> tcp -> D405 mount -> D405 optical
```

D405 hand-eye 오차는 최종 표면점과 법선에 직접 들어간다. 따라서 단순히 영상이 보이는지만 확인해서는 안 되고, 평면 거리와 법선 각도의 정량 검증이 필요하다.

## 7. ZED의 전역 법선 추출

### 7.1 대상 선택 기반 평면

웹 UI에서 사용자가 `ZED Raw` 영상의 대상 표면을 둘러싸면 `/target_selection_pixels`가 발행된다. `target_selector_node`는 다음 순서로 `/perception/target_surface`를 만든다.

1. 선택 stroke의 픽셀 bounding box에 16 px padding을 더한다.
2. 등록 depth를 4 px stride로 샘플링한다.
3. 유효 깊이 `0.15 m < Z < 5.0 m`만 사용한다.
4. 카메라 내부 파라미터로 각 픽셀을 3D로 역투영한다.
5. RANSAC으로 평면 `aX + bY + cZ + d = 0`을 적합한다.
6. inlier centroid를 초기 표면점 `p_zed`로 사용한다.
7. 정규화된 `(a,b,c)`를 초기 법선 `n_zed`로 사용한다.

픽셀 역투영식은 다음과 같다.

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth(u, v)
```

현재 target RANSAC 기준은 다음과 같다.

| 항목 | 값 |
|---|---:|
| 최소 3D 점 및 최소 inlier | `80` |
| RANSAC 거리 임계값 | `0.015 m` |
| RANSAC 반복 | `2000` |

평면 법선에는 부호 모호성이 있다. 코드는 카메라 원점을 자유 공간으로 보고 다음 조건으로 법선을 뒤집는다.

```text
if dot(n_zed, centroid) > 0:
    n_zed = -n_zed
```

그 결과 법선은 대상 표면에서 ZED 카메라가 있는 자유 공간 쪽을 향한다. `PoseStamped.orientation`은 local `+Z`가 이 법선을 가리키도록 저장된다.

### 7.2 전역 장면 평면 스캔

`environment_scanner_node`는 시작 시 기본 5초 동안 ZED point cloud를 누적하고 2 cm voxel downsample 후 최대 5개 평면을 반복 RANSAC으로 찾는다.

| 항목 | 값 |
|---|---:|
| 장면 누적 시간 | `5.0 s` |
| voxel 크기 | `0.020 m` |
| 최대 거리 | `5.0 m` |
| RANSAC 거리 | `0.010 m` |
| 최대 평면 수 | `5` |
| 최소 평면 inlier | `1000` |

법선의 Z 성분으로 수직면, 수평면, 기타 평면을 분류한다. 이 결과는 장면 이해와 planning scene 구성에 쓰이며, 사용자가 선택한 target plane이 실제 도장면의 우선 기준이다.

현재 실로봇 기본 설정은 자동 초기 스캔 `true`, 자동 반복 재스캔 `false`, ZED 잔여 point obstacle 발행 `false`다.

## 8. D405의 표면점 및 법선 보정

### 8.1 입력과 출력

입력:

```text
/perception/target_surface
/perception/work_area_plane
/perception/work_area_corners
/d405/d405/depth/color/points
```

출력:

```text
/perception/target_surface_refined
/perception/work_area_plane_refined
/perception/d405_surface_refinement_status
/target_refine_status
```

### 8.2 D405 point cloud 처리

D405 refiner는 ZED가 제공한 기준 평면 `(p0, n0)`을 먼저 받는다. D405 cloud를 TF로 기준 평면과 같은 프레임으로 변환한 뒤 다음 처리를 수행한다.

1. 기준 평면에서 일정 거리 안의 점만 ROI로 남긴다.
2. target 보정이면 기준점 주변의 사각 영역을 사용한다.
3. work-area 보정이면 선택된 네 모서리와 8 cm margin을 사용한다.
4. 3 mm voxel downsample을 수행한다.
5. 4 mm RANSAC 거리로 D405 평면을 적합한다.
6. 새 법선의 부호를 ZED 기준 법선과 일치시킨다.
7. ZED 법선과의 각도 및 기준 평면과의 이동량을 검사한다.
8. 통과하면 기준점 `p0`를 새 평면에 정사영해 보정 중심점을 만든다.

법선 부호 연속성:

```text
if dot(n_d405, n_zed) < 0:
    n_d405 = -n_d405
```

기준점의 signed shift와 보정점:

```text
d = -dot(n_d405, d405_inlier_center)
signed_shift = dot(p0, n_d405) + d
p_refined = p0 - signed_shift * n_d405
```

최종 실로봇 launch에서 유효한 주요 값은 다음과 같다.

| 항목 | 값 |
|---|---:|
| 최대 입력 점 | `120000` |
| 최소 ROI 점 | `150` |
| 최소 RANSAC inlier | `100` |
| voxel 크기 | `0.003 m` |
| RANSAC 거리 | `0.004 m` |
| RANSAC 반복 | `180` |
| 기준 평면 주변 ROI 폭 | `0.30 m` |
| target ROI half width/height | `0.35 / 0.35 m` |
| ZED 대비 최대 법선 차이 | `12 deg` |
| ZED 대비 최대 평면 이동 | `0.25 m` |
| capture timeout | `1.2 s` |
| capture trigger 필수 | `true` |
| `lock_after_refinement` | `false` |

여기서 `0.30 m` ROI와 `0.25 m` shift 허용은 ZED를 전역 길잡이로 두고 D405 측정값을 가까운 작업면의 기준값으로 채택하기 위해 넓게 설정되어 있다.

### 8.3 현재 유효한 샘플 채택 방식

노드 자체에는 여러 공간 샘플을 모아 SVD 평면을 다시 적합하는 코드가 있다. 그러나 최종 perception launch는 `lock_after_refinement=false`로 실행한다. 따라서 현재 실제 동작은 다음과 같다.

- 한 번의 capture에서 RANSAC, inlier, 법선 각도, 평면 shift 검사를 통과하면 즉시 refined plane을 발행한다.
- executor가 만든 여러 prescan pose는 여러 측정을 융합하는 샘플이 아니라, 앞 pose에서 보정에 실패했을 때 시도할 다음 후보 pose다.
- 첫 성공 capture가 해당 보정 단계의 최종 평면이 된다.
- `spatial_samples=3`, `max_spatial_samples=5`, normal spread, fit residual 파라미터는 현재 launch 조합에서는 다중 샘플 fusion에 사용되지 않는다.

이 사실은 외부 재검토가 필요한 핵심 항목이다.

## 9. 법선 갱신 생명주기

### 9.1 Target 단계

1. 사용자가 ZED Raw에서 target을 선택한다.
2. ZED depth RANSAC이 `/perception/target_surface`를 만든다.
3. 웹 UI가 `/refine_target_surface=true`를 발행한다.
4. D405 refiner가 현재 로봇 자세에서 보이는 D405 cloud 한 프레임을 사용해 target plane을 보정한다.
5. 성공 시 `/perception/target_surface_refined`가 이후 Wall Front의 기준이 된다.
6. 실패 또는 timeout이면 UI는 ZED target surface를 사용해 Work Area 단계로 넘어간다.

Target 보정 요청 자체는 로봇을 D405 촬영 자세로 이동시키지 않는다. 현재 자세에서 D405가 target을 보지 못하면 보정이 실패할 수 있다.

### 9.2 Work Area 단계

1. D405 color 영상과 현재 target plane으로 `/perception/wall_front_view`를 만든다.
2. 사용자가 이 정면 뷰에 작업영역을 그린다. 박스를 그리지 않으면 전체 Wall Front를 작업영역으로 사용한다.
3. Wall Front 픽셀 사각형을 평면의 3D 네 모서리로 역매핑한다.
4. `/refine_work_area=true`가 발행된다.
5. `moveit_executor`가 D405 prescan을 시작한다.
6. D405 카메라 중심이 표면에서 약 `0.32 m` 떨어지도록 로봇을 이동한다.
7. 로봇이 정지한 뒤 `1.0 s` settle하고 `/d405/refine_capture=true`를 발행한다.
8. 첫 보정 성공 시 work-area refined plane을 잠근다.

prescan 후보 생성 기준:

| 항목 | 값 |
|---|---:|
| D405 standoff | `0.32 m` |
| 최대 후보 pose | `5` |
| 표면 내 probe offset | `0.06 m` |
| 작업영역 경계 inset | `0.08 m` |
| settle | `1.0 s` |
| executor 측 capture 대기 | `4.0 s` |
| 속도 scaling | `0.025` |
| 첫 pose | collision-aware MoveIt pose plan |
| 이후 pose | collision-aware Cartesian probe |

현재 TCP에 가장 가까운 작업영역 지점을 첫 측정점으로 선택하고, 필요할 때 표면 접선 방향 후보를 순서대로 시도한다. 큰 관절 우회 동작을 막기 위해 총 joint path, 시작-목표 관절 변화, trajectory point 수를 검사한다.

### 9.3 Refined plane 적용

D405 보정 성공 후 다음 일이 일어난다.

1. `wall_projector_node`는 작업영역 네 모서리를 refined plane에 직교 투영한다.
2. 작업영역의 법선을 D405 법선으로 교체한다.
3. 그릴 때 사용한 Wall Front extent와 해상도는 고정해 픽셀-3D 대응이 흔들리지 않게 한다.
4. `moveit_executor`는 D405 refined plane을 lock하고 뒤이어 들어오는 ZED plane 갱신을 무시한다.
5. 새 작업영역이 선택되면 기존 D405 lock을 해제하고 새 보정을 수행한다.

### 9.4 실행 직전 재투영

`Run Robot` 시 fresh D405 plane이 없으면 executor가 다시 prescan을 시도한다. 기본 설정에서는 prescan 실패 시 ZED plane으로 계속할 수 있다.

D405 plane을 사용할 수 있으면 executor는 이미 생성된 경로를 다시 계산한다.

일반 waypoint:

```text
old_surface = old_roller_center - old_normal * 0.031
new_surface = project(old_surface, refined_plane)
new_roller_center = new_surface + refined_normal * 0.031
```

세그먼트 row:

```text
row.surface_point = project(row.surface_point, refined_plane)
row.normal        = refined_normal
row.tangent       = project_to_plane(row.tangent, refined_normal)
```

따라서 스케치의 평면 내 형상은 유지하면서 깊이와 법선을 최신 D405 평면으로 맞춘다.

### 9.5 PAINT 중 법선 정책

도장 실행이 시작된 후에는 ZED 및 D405 surface 갱신을 현재 plan에 반영하지 않는다.

- 실행 중 표면점과 법선은 고정된다.
- 한 작업영역 전체가 하나의 평면과 하나의 법선을 공유한다.
- 곡면의 위치별 법선이나 페인팅 중 연속 법선 추적은 현재 구현하지 않는다.
- 작은 접촉 오차는 TCP Y축 어드미턴스가 흡수하며, 카메라가 온라인으로 자세를 계속 바꾸지는 않는다.

이 정책은 센서 노이즈로 경로와 자세가 순간적으로 바뀌는 것을 막지만, 큰 평면 오차나 곡률을 보상하지는 못한다.

## 10. D405 Wall Front와 픽셀-3D 매핑

`front_view_source=d405`가 최종 설정이다. `wall_projector_node`는 다음 순서로 작업면의 정면 영상을 만든다.

1. D405 image 네 코너에서 광선을 만든다.
2. 각 광선을 현재 target plane과 교차시킨다.
3. 평면 위 right/up 축에서 실제로 보이는 내접 사각 영역을 구한다.
4. 이 3D 사각형을 D405 영상으로 다시 투영한다.
5. OpenCV perspective warp로 `/perception/wall_front_view`를 발행한다.
6. 정면 영상 전체가 대응하는 3D 사각형을 `/perception/wall_front_extent`로 발행한다.

사용자 픽셀 `(u,v)`는 TL/TR/BR/BL 네 모서리의 bilinear interpolation으로 표면점에 매핑된다. 작업영역을 lock한 뒤에는 extent를 고정하므로 D405 refined plane이 약간 이동해도 이미 그린 픽셀 좌표의 의미가 변하지 않는다.

## 11. 웹 스케치 작업 흐름

### 11.1 Target

- 화면: `ZED Raw`
- 입력: 대상 표면을 둘러싼 stroke 또는 box
- 발행: `/target_selection_pixels`, frame `zed_raw`
- 후속 동작: D405 target refinement 요청

### 11.2 Work Area

- 화면: `Wall Front`
- 영상 소스: D405 color를 target plane 기준으로 정면 보정한 영상
- 입력: 도장할 사각 작업영역
- 발행: `/work_area_pixels`, frame `wall_front`
- 후속 동작: 로봇이 D405 prescan pose로 이동하여 work-area plane 보정

### 11.3 Path

- 화면: `Wall Front`
- 입력: 자유 곡선, 선, 여러 개의 독립 stroke 또는 자동 Fill
- 발행: `/sketch_pixels` 또는 `/fill_work_area`
- 결과: `/sketch_waypoints`, `/sketch_eoat_segments`, `/sketch_markers`

브라우저는 각 stroke의 번호를 `Pose.position.z`에 넣는다. `sketch_to_waypoints_node`는 이 번호가 바뀌는 지점에서 stroke를 분리한다. 따라서 시스템은 특정 ㄹ자 한 종류에 고정되지 않는다. 임의의 여러 도장 stroke를 만들 수 있고, 서로 다른 stroke 사이만 비접촉 연결로 변환된다.

Fill은 현재 롤러 길이 `220 mm`와 overlap `30%`를 사용해 번갈아 진행하는 세로 stroke를 생성한다. stroke 자체만 `PAINT`이며 옆 stroke로 넘어가는 연결은 lifted `TRAVEL`이다.

다만 현재 구현에서 브라우저와 `wall_projector_node`의 미리보기는 선택된 work area 안에 Fill을 표시하지만, `sketch_to_waypoints_node`의 실제 Fill 생성은 전체 `wall_front` 폭과 높이를 사용한다. 선택 작업영역과 실제 생성 범위가 일치하는지는 R17의 필수 재검토 항목이다.

### 11.4 Execute와 Run Robot

- `Execute`는 픽셀 경로를 ROS 경로와 세그먼트로 변환한다. 로봇은 아직 움직이지 않는다.
- `Run Robot`은 확인 대화상자 후 `/sketch_execute=true`를 발행한다.
- 실행 전에 RViz의 경로, 법선 방향, work-area plane, 시작 안전 pose를 확인해야 한다.

## 12. 세그먼트 경로 계약

최종 runtime 경로는 `/sketch_eoat_segments`의 `std_msgs/String`에 담긴 version 2 JSON이다.

### 12.1 루트 필드

```json
{
  "version": 2,
  "frame_id": "link0",
  "path_id": "<waypoint stamp in nanoseconds>",
  "point_semantics": "surface_point",
  "contact_offset_m": 0.031,
  "normal_axis": "surface_z",
  "tcp_normal_axis": "+y",
  "tangent_semantics": "paint_motion_direction",
  "preserve_orientation_continuity": true,
  "rows": []
}
```

`path_id`는 함께 발행되는 `/sketch_waypoints`의 stamp와 연결된다. 최종 운용에서는 같은 `path_id`의 version 2 세그먼트가 반드시 있어야 한다.

### 12.2 Row 필드

```text
mode,x,y,z,nx,ny,nz,tx,ty,tz,force_n,offset_m,speed_mps
```

| 필드 | 의미 |
|---|---|
| `mode` | 현재 공정 상태 |
| `x,y,z` | 실제 표면점 |
| `nx,ny,nz` | outward surface normal |
| `tx,ty,tz` | 표면 위 페인팅 진행 방향 |
| `force_n` | 양수 크기로 표현한 목표 접촉력 |
| `offset_m` | `contact_offset_m`에 더하는 추가 이격 |
| `speed_mps` | 해당 motion row의 선속도 상한 |

법선과 tangent는 정규화되고, tangent는 법선 성분을 제거한다. tangent가 법선과 평행하거나 0이면 경로를 거부한다.

## 13. 공정 상태 머신

### 13.1 첫 stroke

```text
Stage 1 safety pose
  -> APPROACH
  -> RAMP_UP
  -> PAINT
```

### 13.2 stroke 사이

```text
PAINT 끝
  -> RAMP_DOWN
  -> RETRACT
  -> TRAVEL
  -> APPROACH
  -> RAMP_UP
  -> 다음 PAINT
```

### 13.3 마지막 stroke

```text
PAINT 끝
  -> RAMP_DOWN
  -> FINISH_RETRACT (추가 10 mm)
  -> final safe retreat (표면 기준 80 mm)
  -> IDLE
```

### 13.4 모드별 실제 동작

| 모드 | pose 동작 | 목표 wrench | 완료 조건 |
|---|---|---|---|
| `APPROACH` | travel clearance에서 명목 contact offset으로 이동 | 0 | Cartesian/FJT 성공 |
| `RAMP_UP` | pose 고정 | 0에서 `force_n`까지 2초 ramp | ramp feedback, 필요 시 contact 확인 |
| `PAINT` | 표면 경로 추종 | `-force_n` on TCP Y | Cartesian/FJT 성공 |
| `RAMP_DOWN` | stroke 끝 pose 유지 | 현재 힘에서 0까지 1초 ramp | ramp feedback |
| `RETRACT` | 법선 바깥쪽으로 10 mm 이동 | 0 | Cartesian/FJT 성공 |
| `TRAVEL` | 10 mm lift를 유지하며 다음 시작점으로 이동 | 0 | Cartesian/FJT 성공 |
| `FINISH_RETRACT` | 마지막 점에서 10 mm 후퇴 | 0 | Cartesian/FJT 성공 |
| `ABORT` | 진행 중 trajectory 취소 | 즉시 0, force disable | operator recovery 필요 |

현재 `APPROACH`는 경로 기반으로 명목 contact offset까지만 이동하며, force로 계속 파고들지 않는다. 접촉 필수 조건은 `RAMP_UP` 완료 시 검사한다.

### 13.5 상태 순서 검증

실행 전에 다음 규칙을 검사한다.

- `PAINT` 이전에 `RAMP_UP`이 있어야 한다.
- `RETRACT` 또는 `TRAVEL` 이전에 `RAMP_DOWN`이 있어야 한다.
- 경로가 force-ready 상태로 끝나면 거부한다.
- 비접촉 모드의 `force_n`은 경고 후 0으로 강제한다.
- 생성 힘이 executor의 최대 힘보다 크면 경로를 거부한다.
- clearance 모드가 최소 `0.005 m`보다 작으면 executor가 경로를 거부한다.
- row normal은 활성 표면 normal과 dot product `0.90` 이상이어야 한다.
- row surface point는 활성 평면에서 `0.030 m` 이내여야 한다.

## 14. EOAT 자세 생성과 180도 flip 방지

각 row의 자세는 surface normal과 path tangent에서 생성한다.

```text
tcp_y = normal
tcp_x = normalize(cross(tcp_y, tangent))
tcp_z = normalize(cross(tcp_x, tcp_y))
```

롤러의 긴 축인 TCP X는 페인팅 진행 tangent와 수직이다. 왕복 stroke에서는 tangent가 반대로 바뀌지만, 이전 waypoint의 TCP X와 dot product가 음수이면 새 TCP X의 부호를 뒤집는다.

```text
if dot(new_tcp_x, previous_tcp_x) < 0:
    new_tcp_x = -new_tcp_x
```

따라서 번갈아 진행하는 Fill에서도 손목이 매 stroke마다 180도 뒤집히지 않는다. D405 prescan 자세도 현재 TCP roll과 가장 가까운 법선 정렬 자세를 선택하여 큰 wrist/base flip을 줄인다.

## 15. MoveIt 실행 구조

`moveit_executor`의 최종 실로봇 backend는 반드시 다음 값이다.

```text
execution_backend = follow_joint_trajectory
```

실행 단계:

1. waypoint와 version 2 segment path를 수신한다.
2. 프레임, `path_id`, geometry, 힘 순서를 검증한다.
3. 필요한 경우 D405 prescan을 수행한다.
4. D405 refined plane으로 waypoint와 segment row를 재투영한다.
5. 첫 표면점에서 outward normal 방향 `0.08 m` safety pose를 만든다.
6. Stage 1은 collision-aware MoveIt pose plan으로 이동한다.
7. 각 motion mode를 `/compute_cartesian_path`로 계획한다.
8. joint limit, Cartesian fraction, 충돌 결과를 검사한다.
9. `/joint_trajectory_controller/follow_joint_trajectory`에 보낸다.
10. action 성공을 받은 뒤에만 다음 세그먼트로 진행한다.
11. 마지막에는 표면 기준 `0.08 m`까지 안전 후퇴한다.

접촉 세그먼트의 Cartesian step은 최대 `0.003 m`, 비접촉 세그먼트는 최대 `0.005 m`를 사용한다. 부분 Cartesian path는 실행하지 않는다.

작업 시작과 종료 시 큰 관절 복귀 동작을 피하기 위해 현재 설정은 다음과 같다.

```text
START_FROM_READY_BEFORE_SKETCH = false
RETURN_TO_READY_AFTER_SKETCH = false
```

## 16. ROS 2 제어 체인

### 16.1 Controller chaining

```text
FollowJointTrajectory goal
  -> joint_trajectory_controller
  -> admittance_controller/<joint> chain references
  -> admittance_controller
  -> rbpodo_hardware position commands
  -> RB10
```

`joint_trajectory_controller.command_joints`가 실제 hardware joint가 아니라 다음 chained interface를 가리킨다.

```text
admittance_controller/base
admittance_controller/shoulder
admittance_controller/elbow
admittance_controller/wrist1
admittance_controller/wrist2
admittance_controller/wrist3
```

JTC가 명목 궤적을 만들고 어드미턴스 컨트롤러가 선택된 Cartesian 축의 순응 변위를 반영한 관절 명령을 hardware에 보낸다. 실제 joint state가 계속 발행되므로 어드미턴스 동작 중 로봇의 변화도 RViz robot state에 보인다.

### 16.2 Painting admittance profile

최종 profile은 `painting_normal_y`이며 TCP Y 하나만 compliant axis다.

```text
selected_axes = [false, true, false, false, false, false]
```

| 축 | M | damping ratio | K | 사용 여부 |
|---|---:|---:|---:|---|
| TCP X | 10 | 2.828427 | 1000 | 비선택 |
| TCP Y | 8 | 4.0 | 1000 | 선택, 표면 법선 |
| TCP Z | 10 | 2.828427 | 1000 | 비선택 |
| Rx/Ry/Rz | 1 | 2.828427 | 100 | 비선택 |

추가 값:

```text
joint_damping = 12.0
F/T controller filter_coefficient = 0.02
```

이 값은 고전적인 별도 `Kp`, `Kd` 쌍이 아니라 다음 어드미턴스 모델의 질량, 감쇠비, 강성이다.

```text
F = M*a + D*v + K*(x - x_ref)
D = 2*zeta*sqrt(M*K)
```

선택된 TCP Y축의 등가 감쇠는 현재 값으로 약 `715.5 N*s/m`다. 유한 강성 `1000 N/m`는 non-contact 센서 잔류값에 의한 무한 drift를 줄이기 위한 설정이다.

## 17. 측정 wrench와 목표 wrench

### 17.1 측정 경로

```text
RB controller eft
  -> rbpodo_hardware
  -> ft_sensor
  -> force_torque_sensor_broadcaster/wrench
  -> admittance_controller measured wrench
  -> painting_force_monitor diagnostics/safety
```

데이터 수집용으로 다음 raw broadcaster도 있다.

```text
/force_torque_sensor_broadcaster_raw/wrench
```

`ft_sensor_raw`는 hardware tare bias는 빠져 있지만 `human_collab` deadband와 clamp 전 값이다. 최종 어드미턴스와 painting safety monitor는 기본적으로 필터된 `/force_torque_sensor_broadcaster/wrench`를 사용한다. raw topic은 현재 monitor가 구독만 하며 안전 판단에는 사용하지 않는다.

### 17.2 목표 wrench 경로

```text
segment mode + force_n
  -> moveit_executor
  -> /painting_admittance/mode
  -> /painting_admittance/desired_force_n
  -> /painting_admittance/enable_force
  -> painting_wrench_reference
  -> /admittance_controller/wrench_reference
```

목표 힘이 실제 컨트롤러에 도달하려면 다음 조건이 모두 맞아야 한다.

1. `painting_wrench_reference`가 `dry_run=false`다.
2. executor가 `painting_force_enabled=true`다.
3. 현재 모드가 `RAMP_UP`, `PAINT`, `CONTACT` 중 힘 허용 모드다.
4. abort가 활성화되지 않았다.

`APPROACH`, `RETRACT`, `TRAVEL`, `FINISH_RETRACT`, `IDLE`, `ABORT`에서는 목표 wrench가 항상 0이다.

## 18. F/T tare, 필터, bias 및 접촉 판단

### 18.1 Hardware 단계

`rbpodo_hardware`는 활성화 시 100개 샘플, 약 1초를 사용해 tare한다. tare가 성공하기 전에는 어드미턴스로 raw bias를 흘리지 않고 wrench를 0으로 유지한다.

`human_collab=true`에서 tare 후 각 wrench 성분에 다음 처리를 한다.

```text
|value| < 1.2       -> 0
value > 30          -> 30
value < -30         -> -30
```

단위는 force 축에서 N, torque 축에서 Nm다.

### 18.2 Painting force monitor 단계

기본 처리 순서:

1. `ft_link` wrench를 `tcp`로 회전 변환한다.
2. 알려진 non-contact mode에서 bias를 추정한다.
3. bias를 뺀다.
4. 시간상수 `0.10 s`인 1차 low-pass filter를 적용한다.
5. force 성분에 `0.5 N` deadband를 적용한다.
6. TCP Y를 normal force로 사용한다.
7. hysteresis와 지속시간으로 contact를 확인한다.

| 파라미터 | 현재 최종 기준 |
|---|---:|
| filter time constant | `0.10 s` |
| monitor force deadband | `0.5 N` |
| bias sample duration | `1.0 s` |
| non-contact bias update | `true` |
| contact detect | `1.5 N` |
| contact release | `0.8 N` |
| confirm duration | `0.10 s` |
| travel unexpected-contact | `3.0 N` |
| warning | `10.0 N` |
| 운영 abort | `15.0 N` |
| code default abort | `20.0 N` |
| stale timeout | `0.20 s` |

접촉 상태는 정확한 0 비교로 결정하지 않는다.

```text
force >= 1.5 N for 0.10 s -> contact true
force <= 0.8 N            -> contact false
```

모드는 접촉 의도를 결정하고 힘은 확인과 안전 판단에만 사용한다.

## 19. 정상 안전 동작과 abort

정상 공정에서는 다음 순서를 강제한다.

- 모든 retract/travel 전에 `RAMP_DOWN` 완료를 기다린다.
- 모든 `PAINT` 전에 `RAMP_UP` 완료를 기다린다.
- `require_contact_before_paint=true`이면 RAMP_UP 완료와 contact 확인이 모두 필요하다.
- ramp feedback timeout은 `5.0 s`다.
- non-contact mode의 목표 wrench는 0이다.
- TRAVEL/RETRACT 중 normal force가 `3.0 N` 이상으로 `0.10 s` 유지되면 abort한다.
- normal force가 운영 abort threshold 이상이면 abort한다.
- wrench가 없거나 `0.20 s` 이상 stale이면 abort한다.
- wrench가 non-finite이거나 `ft_link -> tcp` TF가 없으면 abort한다.

abort 발생 시 현재 구현은 다음을 수행한다.

1. painting mode를 `ABORT`로 설정한다.
2. 목표 wrench를 즉시 0으로 만들고 force enable을 끈다.
3. 진행 중 FollowJointTrajectory goal에 cancel 요청을 보낸다.
4. 상태 머신 timer와 D405 prescan을 정지한다.
5. 새로운 세그먼트 실행을 막는다.
6. 자동 후퇴는 수행하지 않고 작업자 복구를 기다린다.

즉, 정상 종료에는 10 mm retract와 80 mm final retreat가 있지만, 과힘 abort에는 현재 자동 법선 후퇴가 없다.

## 20. 최종 운용 파라미터 기준

### 20.1 경로 및 속도

| 파라미터 | 값 |
|---|---:|
| `contact_offset_m` | `0.031 m` |
| `travel_clearance_m` | `0.010 m` |
| `paint_speed_mps` | `0.020 m/s` |
| `retract_speed_mps` | `0.010 m/s` |
| `approach_speed_mps` | `0.005 m/s` |
| `travel_speed_mps` | `0.030 m/s` |
| minimum accepted clearance | `0.005 m` |
| Stage 1 safety offset | `0.080 m` |
| final retreat offset | `0.080 m` |

### 20.2 힘

| 파라미터 | 값 |
|---|---:|
| generated paint force | `1.6 N` |
| target wrench sign | `-1.0` |
| target axis | TCP `force_y` |
| ramp up | `2.0 s` |
| ramp down | `1.0 s` |
| contact required before PAINT | `true` |
| 운영 command/path cap | `15.0 N` |
| 운영 over-force abort | `15.0 N` |

운영 command 예시는 local force 설정의 `target=1.6 N`, `warn=10 N`, `abort=15 N`에 맞추어 monitor, wrench publisher, executor cap을 일치시킨다.

## 21. 빌드와 실행

### 21.1 빌드

ZED wrapper가 있는 `~/ros2_ws`를 underlay로 사용하고, 최종 로봇 패키지는 `~/sketch_robot_ws`에서 빌드한다.

```bash
cd ~/sketch_robot_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

각 터미널에서 다음 환경을 먼저 source한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/sketch_robot_ws/install/setup.bash
```

### 21.2 터미널 1: 실제 RB10, MoveIt, controller chain, RViz

```bash
ros2 launch sketch_control rb10_moveit_full.launch.py \
  robot_ip:=10.0.2.7 \
  use_fake_hardware:=false \
  use_isaac_sim:=false \
  use_sim_time:=false \
  human_collab:=true \
  use_admittance:=true \
  admittance_use_case:=painting
```

`admittance_use_case:=painting`은 `painting_normal_y` profile을 추가하고, 도장 공정에 필요하지 않은 자동 어드미턴스 helper를 시작하지 않는다.

### 21.3 터미널 2: ZED, D405, 인식, 스케치 변환

```bash
ros2 launch sketch_control rb10_real_perception_sketch.launch.py \
  front_view_source:=d405 \
  zed_camera_model:=zed2i \
  d405_cloud_topic:=/d405/d405/depth/color/points \
  default_paint_force_n:=1.6 \
  paint_speed_mps:=0.020 \
  travel_clearance_m:=0.010 \
  retract_speed_mps:=0.010 \
  approach_speed_mps:=0.005 \
  travel_speed_mps:=0.030 \
  auto_scene_scan_on_start:=true \
  auto_scene_rescan:=false \
  publish_scene_obstacles:=false
```

### 21.4 터미널 3: painting wrench reference와 force monitor

실제 force 명령을 사용하는 최종 운용 명령:

```bash
ros2 launch rbpodo_painting_control painting_admittance_control.launch.py \
  dry_run:=false \
  enable_force:=false \
  desired_contact_force_n:=0.0 \
  max_command_force_n:=15.0 \
  contact_detect_threshold_n:=1.5 \
  contact_release_threshold_n:=0.8 \
  travel_collision_threshold_n:=3.0 \
  over_force_warn_n:=10.0 \
  over_force_abort_n:=15.0
```

초기 `enable_force=false`가 정상이다. executor가 실제 공정 시작 시 `/painting_admittance/enable_force`를 제어한다. 최초 commissioning에서는 같은 구조에서 `dry_run:=true`로 목표 wrench와 mode만 검증한 뒤 실 force를 허용한다.

### 21.5 터미널 4: segment executor

```bash
ros2 run sketch_control moveit_executor --ros-args \
  -p use_sim_time:=false \
  -p execution_backend:=follow_joint_trajectory \
  -p use_eoat_segments:=true \
  -p painting_force_enabled:=true \
  -p require_contact_before_paint:=true \
  -p segment_contact_offset_m:=0.031 \
  -p minimum_travel_clearance_m:=0.005 \
  -p max_paint_force_n:=15.0
```

`execution_backend`를 생략하면 코드 기본값이 실로봇용 backend가 아니므로 반드시 명시한다.

### 21.6 터미널 5: rosbridge

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

### 21.7 터미널 6: 웹 UI

```bash
cd ~/sketch_robot_ws/web
python3 -m http.server 8000
```

브라우저:

```text
http://localhost:8000/index.html
```

## 22. 실로봇 preflight checklist

### 22.1 정지 상태와 물리 안전

- E-stop과 pendant 정지 기능에 즉시 접근할 수 있어야 한다.
- 로봇 주변과 예상 prescan/approach 경로를 비운다.
- 롤러와 AFT200 체결, D405 mount, 케이블 strain relief를 확인한다.
- tare 중 롤러가 표면이나 물체에 닿지 않아야 한다.
- 최초 검증은 저속, 낮은 목표 힘, 작은 작업영역으로 수행한다.

### 22.2 Controller 확인

```bash
ros2 control list_controllers
```

다음 controller가 `active`여야 한다.

```text
joint_state_broadcaster
force_torque_sensor_broadcaster
force_torque_sensor_broadcaster_raw
admittance_controller
joint_trajectory_controller
```

```bash
ros2 action list | rg follow_joint_trajectory
```

다음 action이 있어야 한다.

```text
/joint_trajectory_controller/follow_joint_trajectory
```

### 22.3 카메라와 TF 확인

```bash
ros2 topic hz /zed/zed_node/depth/depth_registered
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
ros2 topic hz /d405/d405/depth/color/points
ros2 topic hz /perception/wall_front_view
```

```bash
ros2 run tf2_ros tf2_echo World zed_left_camera_frame_optical
ros2 run tf2_ros tf2_echo tcp d405_color_optical_frame
ros2 run tf2_ros tf2_echo link0 tcp
ros2 run tf2_ros tf2_echo tcp ft_link
```

프레임 이름은 실제 D405 driver의 prefix 설정과 일치해야 한다.

### 22.4 F/T 확인

```bash
ros2 topic hz /force_torque_sensor_broadcaster/wrench
ros2 topic echo --once /force_torque_sensor_broadcaster/wrench
ros2 topic echo --once /painting_admittance/contact_state
ros2 topic echo --once /painting_admittance/overforce_state
ros2 topic echo --once /painting_admittance/current_mode
```

IDLE에서 기대 상태:

```text
current_mode = IDLE
command_force_tcp_y_n = 0
target wrench = 0
overforce_state = OK
```

필요 시 free-space에서 tare를 다시 수행한다.

```bash
ros2 service call /rbpodo_ft_tare_helper/tare_ft std_srvs/srv/Trigger
```

### 22.5 경로 확인

- `/sketch_waypoints`와 `/sketch_eoat_segments`가 같은 새 경로에서 생성됐는지 확인한다.
- `/sketch_markers`가 작업영역 안에 있는지 RViz에서 확인한다. 자동 Fill은 현재 선택 작업영역과 실제 생성 범위가 다를 수 있으므로 특히 주의한다.
- normal이 표면 밖을 향하는지 확인한다.
- TCP `+Y`가 normal, TCP `-Y`가 누르는 방향인지 확인한다.
- 최종 JSON의 `TRAVEL.offset_m`이 `0.010 m`인지 직접 확인한다. 현재 RViz marker의 connector는 별도 PoseArray 표현을 사용해 `0.030 m`로 보일 수 있다.
- 첫 safety pose와 최종 retreat가 표면에서 `80 mm` 떨어져 있는지 확인한다.

## 23. 주요 토픽

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/target_selection_pixels` | `PoseArray` | ZED Raw 대상 선택 픽셀 |
| `/perception/target_surface` | `PoseStamped` | ZED 초기 대상 평면 |
| `/refine_target_surface` | `Bool` | D405 target 보정 요청 |
| `/perception/target_surface_refined` | `PoseStamped` | D405 보정 대상 평면 |
| `/perception/wall_front_view` | `Image` | D405 기반 정면 작업 영상 |
| `/work_area_pixels` | `PoseArray` | Wall Front 작업영역 픽셀 |
| `/perception/work_area_plane` | `PoseStamped` | 작업영역 평면 |
| `/perception/work_area_corners` | `PoseArray` | 작업영역 3D 네 모서리 |
| `/refine_work_area` | `Bool` | 로봇 이동을 포함한 D405 work-area 보정 요청 |
| `/perception/work_area_plane_refined` | `PoseStamped` | D405 최종 작업 평면 |
| `/sketch_pixels` | `PoseArray` | 사용자 도장 stroke 픽셀 |
| `/fill_work_area` | `Empty` | 자동 Fill 생성 |
| `/sketch_waypoints` | `PoseArray` | RViz/호환용 EOAT waypoint |
| `/sketch_eoat_segments` | `String` | 최종 version 2 공정 경로 |
| `/sketch_execute` | `Bool` | 실제 실행 trigger |
| `/painting_admittance/mode` | `String` | 현재 공정 mode command |
| `/painting_admittance/desired_force_n` | `Float64` | 현재 목표 접촉력 크기 |
| `/painting_admittance/enable_force` | `Bool` | force command interlock |
| `/admittance_controller/wrench_reference` | `WrenchStamped` | 컨트롤러 목표 wrench |
| `/force_torque_sensor_broadcaster/wrench` | `WrenchStamped` | 컨트롤 및 safety용 측정 wrench |
| `/force_torque_sensor_broadcaster_raw/wrench` | `WrenchStamped` | tare 후, deadband/clamp 전 진단 wrench |
| `/painting_admittance/contact_confirmed` | `Bool` | hysteresis 기반 접촉 확인 |
| `/painting_admittance/abort` | `Bool` | force monitor abort |
| `/motion_abort` | `Bool` | 전체 motion cancel |

## 24. 외부 재검토가 필요한 핵심 항목

다음은 별도 제어 구조를 제안하는 목록이 아니라, 위 최종 구조가 실제 로봇에서 안전하고 일관되게 작동하는지 검증해야 할 항목이다.

### R1. 명목 5 mm gap과 1.6 N 목표 힘의 접촉 가능성

현재 롤러 중심의 명목 contact offset은 롤러 반지름보다 `5 mm` 크다. 즉, 경로 기준으로 롤러 표면과 실제 면 사이에 `5 mm` gap이 있다.

TCP Y 강성이 `1000 N/m`이고 목표 힘이 `1.6 N`이면 단순 정적 근사 변위는 다음과 같다.

```text
x = F / K = 1.6 / 1000 = 0.0016 m = 1.6 mm
```

이 근사만 보면 5 mm gap을 닫지 못해 contact가 확인되지 않고 RAMP_UP timeout이 날 가능성이 있다. 다음을 반드시 검증해야 한다.

- `0.031 m`가 실제 CAD/URDF 기준 롤러 중심 위치와 정확히 일치하는가
- MoveIt collision clearance와 실제 contact reference를 같은 값으로 써도 되는가
- stock admittance의 wrench reference 부호와 정적 변위가 예상과 같은가
- 실제 접촉까지 필요한 변위와 목표 힘/강성 조합이 일관적인가

### R2. 세그먼트 경로가 없거나 `path_id`가 다를 때 fail-closed 여부

최종 공정은 segment mode가 접촉/비접촉을 결정해야 한다. 그러나 현재 executor에는 matching segment가 없을 때 일반 PoseArray 실행으로 내려갈 수 있는 코드가 남아 있다. 최종 운용 요구는 다음이어야 한다.

```text
use_eoat_segments=true인 실도장 실행에서는
matching version 2 segment가 없으면 로봇을 움직이지 않는다.
```

현재 코드가 이 조건을 완전히 보장하는지 재검토해야 한다. 이 항목은 비접촉 connector 안전과 직접 연결된다.

### R3. D405 첫 성공 single capture 채택

최종 launch는 `lock_after_refinement=false`이므로 다중 위치의 법선을 평균하거나 SVD로 융합하지 않는다. 최대 25 cm shift와 12도 법선 차이를 허용한 상태에서 첫 성공 RANSAC을 채택한다.

검토할 내용:

- 한 프레임 RANSAC만으로 도장 기준면을 lock해도 되는가
- 최소 inlier `100`, 거리 `4 mm`가 D405 노이즈와 반사 표면에 충분한가
- 여러 prescan 위치의 결과를 모두 모아 안정성 검사한 뒤 채택해야 하는가
- 25 cm shift 허용이 오검출 방어와 양립하는가

### R4. D405 실패 시 ZED fallback 허용

실행 전 D405 보정 실패 시 `D405_PREFLIGHT_REQUIRE_REFINED=false`이므로 ZED 평면으로 계속 실행할 수 있다. 정밀 접촉 도장에 이 fallback을 허용할지, 아니면 실행을 중단할지 명시적인 안전 결정을 내려야 한다.

### R5. PAINT 중 법선 고정과 평면 가정

현재는 한 작업영역에 하나의 법선만 사용하며 PAINT 중 법선을 갱신하지 않는다. 다음 대상에서는 오차가 커질 수 있다.

- 휘어진 벽 또는 곡면
- 넓은 면에서 기울기가 변하는 대상
- D405 보정 이후 대상이나 로봇 base가 움직인 경우
- 롤러 길이 방향으로 면이 비틀린 경우

최종 사용 대상이 충분히 평평하고 고정되어 있다는 운용 조건을 문서화해야 한다.

### R6. 비접촉 mode에서도 measured wrench가 어드미턴스에 들어가는 점

목표 wrench는 `TRAVEL/RETRACT`에서 0이지만, stock admittance controller가 읽는 측정 `ft_sensor`는 mode에 따라 0으로 gate되지 않는다. 따라서 deadband `1.2`, controller filter `0.02`, 유한 강성으로 노이즈 영향을 줄일 뿐, 비접촉 mode에서 힘 입력을 완전히 차단하는 구조는 아니다.

검토할 내용:

- 실제 non-contact 노이즈가 1.2 N deadband를 자주 넘는가
- TRAVEL 중 TCP Y 변위가 허용 범위 안인가
- mode별 compliance enable/disable 또는 측정 wrench gating이 필요한가
- stock controller의 연속 사용이 controller switching보다 안전한지 실측했는가

### R7. 명시적인 Cartesian trim 한계가 없는 점

현재 painting profile은 finite stiffness로 변위를 제한하지만, `max_z_trim_m=5 mm`와 같은 명시적인 Cartesian 누적 한계나 법선 속도 한계는 없다. hardware에는 관절별 현재 상태 대비 `0.1 rad` command clamp가 있지만 이는 Cartesian 5 mm 제한과 같지 않다.

어드미턴스가 센서 bias, 지속 외력, 잘못된 목표 wrench에서 만들 수 있는 최대 TCP Y 변위를 정량 검증해야 한다.

### R8. 목표 1.6 N과 contact threshold 1.5 N의 좁은 여유

목표 힘과 contact detect threshold 차이가 `0.1 N`뿐이다. hardware deadband, controller filter, monitor bias, monitor filter를 모두 거친 뒤 RAMP_UP timeout 5초 안에 안정적으로 contact가 확인되는지 실측해야 한다.

또한 접촉 반력 부호가 TCP `+Y`가 아니면 contact는 영원히 true가 되지 않는다. free-space tare 후 수동으로 롤러를 눌러 부호를 먼저 확인해야 한다.

### R9. Safety monitor가 raw spike를 직접 사용하지 않는 점

monitor는 raw broadcaster를 구독하지만 현재 안전 판단은 필터된/clamped input topic으로 수행한다. 순간 충격을 어느 신호와 시간 조건으로 abort할지 검토해야 한다. 반대로 raw 신호를 직접 사용하면 노이즈 오정지가 늘 수 있으므로 두 신호의 역할을 실측 데이터로 정해야 한다.

### R10. 과힘 abort 후 자동 후퇴가 없는 점

과힘이나 unexpected contact가 발생하면 목표 힘은 0이 되고 trajectory는 취소되지만, 로봇은 자동으로 법선 방향 후퇴하지 않는다. 접촉 상태에서 단순 정지가 안전한지, 제한된 emergency retract가 필요한지 위험 분석이 필요하다.

### R11. APPROACH 중 조기 접촉 처리

현재 APPROACH는 목표 wrench 0으로 명목 contact offset까지 이동하고, contact 필수 확인은 RAMP_UP에서 한다. APPROACH 도중 contact가 먼저 발생했을 때 즉시 멈추고 RAMP_UP으로 전환하는 로직은 없다. 또한 별도의 `max_extra_approach_m` 동작도 없다.

접근 오차 범위와 벽 위치 불확실성을 기준으로 이 동작이 충분한지 검토해야 한다.

### R12. EOAT gravity compensation

painting profile의 gravity compensation은 현재 다음 값이다.

```text
CoG position = [0, 0, 0]
gravity force = 0
```

평평한 수직면에서 자세가 거의 고정될 때는 tare와 bias가 일부 영향을 줄일 수 있지만, 자세가 변하거나 곡면을 따라갈 때 EOAT 중량이 wrench에 섞일 수 있다. 실제 롤러, D405, bracket의 질량과 CoG를 식별해야 한다.

### R13. ZED 동적 obstacle 기본 비활성

현재 `publish_scene_obstacles=false`이므로 ZED residual cloud의 동적 obstacle은 MoveIt planning scene에 기본 등록되지 않는다. 정적 `objects.yaml`, 로봇/EOAT, 활성 작업면 중심으로 충돌 검사를 한다. 사람이나 이동 물체가 들어오는 환경이라면 별도 안전 계층이 필요하다.

### R14. Target D405 refinement에 자동 촬영 자세가 없는 점

Target 단계의 D405 보정은 현재 자세에서 즉시 capture한다. D405가 대상 전체를 보지 못하면 ZED로 fallback한다. Work Area 단계처럼 target 단계도 명시적인 촬영 자세가 필요한지 검토해야 한다.

### R15. UI의 D405 측정 명령 연결 상태

웹 UI의 `Measure D405` 동작은 `/measure_d405_plane`을 발행하지만 현재 workspace에서 이 토픽을 구독하는 backend는 확인되지 않는다. 최종 UI에서 이 버튼을 제거하거나, 실제 보정 trigger와 연결하거나, 비활성 상태를 명확히 해야 한다.

### R16. 임계값의 단일 source of truth

현재 힘 값은 local JSON, perception launch, painting monitor launch, executor parameter에 나뉘어 있다. 이 문서의 명령은 `1.6/10/15 N`으로 맞추지만, 인자를 생략하면 monitor와 executor의 code default가 `20 N`이 될 수 있다.

운영 profile 하나에서 다음 값을 동시에 공급하는 구조가 필요한지 검토해야 한다.

```text
paint force
contact/release threshold
travel collision threshold
warning/abort threshold
command cap
segment parser cap
```

### R17. 자동 Fill의 선택 작업영역 범위 보장

현재 UI와 Wall Front 미리보기는 선택된 work-area corners 안에 Fill stroke를 그린다. 그러나 실제 `sketch_to_waypoints_node._on_fill_work_area()`는 전체 `wall_front` image width/height에서 stroke를 생성하고, 픽셀을 `wall_front_extent` 전체에 매핑한다.

따라서 선택한 초록색 작업영역보다 실제 로봇 경로가 넓게 생성될 가능성이 있다. 다음을 확인하고 fail-closed로 고쳐야 한다.

- Fill 생성 범위를 `/perception/work_area_corners`에 대응하는 픽셀 사각형으로 제한하는가
- 자유 스케치도 선택 작업영역 밖의 픽셀을 거부하거나 clip하는가
- UI 미리보기, RViz marker, version 2 segment가 동일한 3D 범위를 나타내는가

### R18. RViz marker와 최종 segment clearance 불일치

`/sketch_markers`는 `/sketch_waypoints`용 motion entries를 시각화한다. 이 표현의 stroke 사이 lift는 현재 `legacy_noncontact_lift_m=0.030 m`이며, 실제 version 2 segment의 `TRAVEL/RETRACT.offset_m=0.010 m`와 다르다.

따라서 RViz에서 보이는 connector 높이만으로 최종 실행 clearance를 검증할 수 없다. 최종 segment 자체의 mode/offset을 색상과 높이로 표시하는 전용 marker가 필요한지 검토해야 한다.

### R19. D405 refined plane freshness 기준 불일치

`sketch_to_waypoints_node`는 D405 work-area plane을 수신 후 30초 동안만 fresh로 보고, 이후에는 ZED work-area plane으로 돌아간다. 반면 `wall_projector_node`와 `moveit_executor`의 D405 lock은 새 작업영역이 들어올 때까지 유지될 수 있다.

사용자가 work-area 보정 후 30초 이상 지나 경로를 그리면 다음 상태가 생길 수 있다.

- Wall Front와 work-area lock은 D405 기준
- 최초 `/sketch_waypoints`와 marker는 ZED 기준
- Run Robot 직전 segment는 executor에서 다시 D405 평면으로 재투영

최종 실행은 재투영될 수 있지만 실행 전 marker와 실제 경로가 달라질 수 있다. 세 노드가 같은 plane generation ID와 freshness 정책을 공유해야 하는지 검토해야 한다.

## 25. 외부 검토자에게 전달할 요청문

아래 요청과 함께 이 문서 전체를 전달하면 된다.

```text
이 문서는 실제 RB10 롤러 도장 로봇의 현재 최종 구현 사양이다.
대체 시스템을 새로 설계하기 전에, 문서에 적힌 현재 한 경로를 기준으로 다음을 검토해 달라.

1. ZED 초기 평면과 D405 refined 평면의 법선 부호, 좌표 변환, lock/update 순서가 일관적인가?
2. D405 single-capture RANSAC과 허용 오차가 실제 접촉 기준면으로 충분히 안전한가?
3. surface point, roller radius, 5 mm MoveIt clearance, 1.6 N 목표 힘, 1000 N/m 강성이 실제 접촉을 만들 수 있는가?
4. PAINT와 non-contact segment의 상태 순서 및 force-zero 보장이 fail-closed인가?
5. stock ROS 2 admittance를 계속 active로 둔 상태에서 TRAVEL 중 센서 노이즈가 로봇을 움직일 가능성이 있는가?
6. F/T tare, hardware deadband/clamp, controller filter, monitor bias/filter/threshold가 서로 충돌하지 않는가?
7. over-force, stale sensor, TF 실패, trajectory cancel 시 로봇 상태가 안전한가?
8. curved surface, calibration drift, D405 실패, dynamic obstacle에 대한 운용 제한이 충분히 명시됐는가?
9. 자동 Fill과 자유 스케치가 선택 작업영역을 벗어나지 않으며, UI/RViz/segment가 같은 경로를 보여주는가?
10. 구현과 문서가 다른 부분, race condition, 오래된 데이터 사용, topic/QoS mismatch가 있는가?
11. 실로봇 시험 전에 반드시 고쳐야 할 사항을 위험도 순으로 제시해 달라.

각 지적에는 관련 모듈, 실패 시나리오, 재현/검증 방법, 최소 수정 방향을 함께 적어 달라.
```

## 26. 구현 source of truth

| 영역 | 파일 |
|---|---|
| 실제 RB10 + MoveIt + controller launch | `src/sketch_control/launch/rb10_moveit_full.launch.py` |
| 실제 ZED/D405 launch | `src/sketch_control/launch/rb10_real_perception_sketch.launch.py` |
| perception 구성 | `src/sketch_control/launch/rb10_perception_sketch.launch.py` |
| ZED target 평면 | `src/sketch_control/sketch_control/target_selector_node.py` |
| ZED 장면 스캔 | `src/sketch_control/sketch_control/environment_scanner_node.py` |
| D405 평면 보정 | `src/sketch_control/sketch_control/d405_surface_refiner_node.py` |
| Wall Front와 작업영역 | `src/sketch_control/sketch_control/wall_projector_node.py` |
| 픽셀 경로와 segment 생성 | `src/sketch_control/sketch_control/sketch_to_waypoints_node.py` |
| MoveIt, D405 prescan, 상태 머신 | `src/sketch_control/sketch_control/moveit_executor.py` |
| segment schema와 자세 연속성 | `src/rbpodo_painting_control/rbpodo_painting_control/segment_path.py` |
| 목표 wrench | `src/rbpodo_painting_control/rbpodo_painting_control/painting_wrench_reference_node.py` |
| force safety monitor | `src/rbpodo_painting_control/rbpodo_painting_control/painting_force_monitor_node.py` |
| painting support launch | `src/rbpodo_painting_control/launch/painting_admittance_control.launch.py` |
| ros2_control chain | `src/rbpodo_ros2/rbpodo_bringup/config/controllers_admittance.yaml` |
| TCP Y painting profile | `src/rbpodo_ros2/rbpodo_bringup/config/admittance_profiles/painting_normal_y.yaml` |
| F/T tare/deadband/clamp | `src/rbpodo_ros2/rbpodo_hardware/src/rbpodo_hardware_interface.cpp` |
| 웹 UI | `web/index.html`, `web/js/app.js`, `web/style.css` |
| ZED calibration | `zed_d405_apriltag_calibration.json` |
| D405 calibration | `d405_eyeinhand_charuco_calibration.json` |
| 현재 force baseline | `aft200_force_threshold.json` |

## 27. 최종 운용 불변 조건

실제 도장 실행은 최소한 다음 조건을 만족해야 한다.

```text
실로봇 backend = follow_joint_trajectory
controller chain = JTC -> admittance -> rbpodo_hardware
painting profile = TCP Y only
path contract = matching version 2 segments
surface semantics = surface_point
TCP +Y = outward normal
PAINT 전 RAMP_UP 완료 및 contact 확인
TRAVEL 전 RAMP_DOWN 완료
TRAVEL/RETRACT target wrench = 0
fresh하고 검증된 F/T 및 TF
operator가 RViz 경로와 법선을 확인한 뒤 Run Robot
```

위 조건 중 하나라도 확인되지 않으면 실제 도장을 시작하지 않는 것이 최종 운용 원칙이다.
