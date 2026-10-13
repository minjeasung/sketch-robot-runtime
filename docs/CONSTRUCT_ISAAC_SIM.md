# Construct 로봇 — Isaac Sim 6.1

작업 위치: `/home/minjea/robotics/sketch-robot-runtime-main`

GitHub `main`의 `ebc36dd676602ad913a187b32762b3aedd15b88f`에서 새로 복제했습니다.
추가 설정은 로컬 `codex/construct-isaac-sim` 브랜치에 있습니다.
기존 `/home/minjea/robotics/sketch-robot-runtime` 작업은 수정하지 않았습니다.

## 모델

Downloads의 `construct_description (1).zip`, `construct_moveit_config (1).zip`을
각각 `src/construct_description`, `src/construct_moveit_config`에 풀었습니다.
원본 URDF, STL, 초기 자세와 SRDF를 사용합니다. 양팔 각 6축, 헤드 2축입니다.

- `isaac_assets/construct.usd`: 로봇 USD
- `isaac_assets/construct_scene.usda`: 바닥·조명·ROS 그래프를 포함한 씬
- `isaac_assets/construct_manifest.json`: 관절 이름·한계·초기 자세
- `isaac_assets/construct_control.urdf`: 토픽 기반 시뮬레이션 제어 URDF

## 실행

터미널 1:

```bash
cd ~/robotics/sketch-robot-runtime-main
OMNI_KIT_ACCEPT_EULA=YES bash scripts/run_construct_isaac.sh
```

GUI 없이 실행할 때는 끝에 `--no-window`를 붙입니다.
Isaac의 Play 상태에서 시뮬레이션 시계와 관절 상태를 발행합니다. 최초 실행은 렌더링 초기화로 1~2분 걸릴 수 있습니다.
정지→재생 시 초기 자세로 복귀합니다. `--verify-reset` 옵션으로 이 동작을 자동 검사할 수 있습니다.

터미널 2:

```bash
cd ~/robotics/sketch-robot-runtime-main
bash scripts/run_construct_moveit.sh rviz:=true
```

RViz에서 Planning Group을 `left_manipulator`, `right_manipulator`, `robot_head`,
`dual_arm`, `whole_robot`으로 선택할 수 있습니다. 양팔 IK는 원본 KDL 설정을 사용하고
헤드는 관절 목표로 계획합니다. World 프레임과 TF는 원본 URDF 기준입니다.

## 연결과 점검

다른 터미널에서:

```bash
cd ~/robotics/sketch-robot-runtime-main
source scripts/construct_ros_env.sh
ros2 control list_controllers
ros2 topic echo /joint_states --once
python3 scripts/check_construct_sim.py
```

마지막 명령은 **시뮬레이션을 실제로 움직이는 검사**입니다. 양팔과 헤드의 첫 관절을
0.03 rad 왕복하고 MoveIt 계획·실행까지 검사합니다. 결과는 `logs/construct-validation.json`에 저장됩니다.

ROS_DOMAIN_ID 기본값은 105이고 통신 범위는 localhost입니다. 두 실행기의 domain은 같아야 합니다.
`/joint_states`, `/isaac_joint_command`, `/clock`을 사용하며
`left_manipulator_controller`, `right_manipulator_controller`, `robot_head_controller`에
FollowJointTrajectory 액션을 제공합니다. 물리 로봇 IP나 드라이버는 이 실행 경로에서 사용하지 않습니다.

## 다시 생성

```bash
bash scripts/build_construct_sim.sh
OMNI_KIT_ACCEPT_EULA=YES bash scripts/setup_construct_isaac.sh
```

ROS 2 Jazzy 및 MoveIt/ros2_control은 `/opt/ros/jazzy`에 설치되어 있습니다.
Isaac 환경 기본값은 `../.venvs/isaacsim-6.1`이며 `ISAACSIM_VENV`로 바꿀 수 있습니다.
빌드한 파일과 생성한 URDF에는 절대경로가 있으므로 폴더 이동 후 두 명령을 다시 실행하세요.

## 원본 관성값 보정

원본 `right_D435_link`는 질량이 236.6848 kg, 무게중심이 링크에서 2 m 이상 떨어져 있어
시뮬레이션에서 오른팔이 무너졌습니다. 좌우 D435 STL 1,252개 삼각형의 정점이
약 1e-12 m 이내로 일치하는 것을 확인했습니다. 생성 URDF에서는 오른쪽 카메라에
왼쪽의 관성(질량 0.0817263 kg, 로컬 무게중심, 관성텐서)을 복사합니다.
원본 `src/construct_description` 파일과 ZIP은 유지됩니다. 이 값은 대칭 형상에 기반한
시뮬레이션 보정이며 실측 관성 보정이 아닙니다. 내역은 manifest의 `inertia_overrides`에 있습니다.

헤드 원본 설정은 가속도 제한을 비활성화하므로 Jazzy의 시간 매개화(TOTG)가 실패합니다.
생성되는 `moveit_joint_limits.yaml`에만 헤드 가속도 0.5 rad/s², 저크 2.0 rad/s³를
시뮬레이션 값으로 지정합니다. 팔의 원본 계획 제한은 유지하고, Ruckig로 저크 제한을 적용합니다.
14축의 URDF 속도·토크 제한은 PhysX 구동에도 적용합니다.

## 범위

- 기본 실행은 로봇만 구성합니다. 아래 통합 실행기로 ZED 2i stereo, 공식 Outpost와 Michelo 영상/깊이 스트림을 사용합니다. D435와 Helios는 아직 외형만 있습니다.
- PhysX 관절 구동은 동작 검증용 게인입니다. 실제 모터나 접촉 동역학의 교정값이 아닙니다.
- PhysX 내부 self-collision은 비활성화합니다. MoveIt은 원본 충돌 형상과 SRDF 제외쌍으로 경로를 검사합니다.
- 기존 스케치 웹 앱은 RB10/RB20 단일 팔용입니다. RB 도장 워크플로는 유지합니다. Construct에는 아래 별도 로봇 모드로 영상 스케치와 선택한 팔의 경로 추종을 제공합니다.

Isaac 6.x importer 변경은 [NVIDIA 문서](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/urdf_mjcf_importer_exporter_pipeline.html)를 참고했습니다.

## 검증 결과 (2026-10-12)

- 설정 테스트 4개 통과, 14축 USD 변환 및 PhysX 속도 제한 확인
- 양팔·헤드 각 FollowJointTrajectory 왕복 성공
- 양팔·헤드 각 MoveIt 경로 계획·실행·복귀 성공
- 초기 자세 충돌 검사 통과, Ruckig 활성화 확인
- GUI Stop→Play 초기 자세 복귀 오차 최대 0.000338 rad

상세 결과: `logs/construct-validation.json`, `logs/construct-validation.log`,
`logs/construct-scene-final.log`. 화면: `logs/construct-scene.png`.


## EOAT와 ZED / Outpost 확장

원본 좌우 `*_manipulator_link_ee`는 고정 공구입니다. 오른쪽은 긴 토치형,
왼쪽은 짧은 오프셋 공구형 CAD이며 각 손목에 D435가 있습니다. 원본에 용접용
기준점 이름이 있지만 공구 제조사·제품명·구동 인터페이스는 없으므로 특정 제품으로
단정하지 않습니다. 용접 아크·와이어 송급·공구 개폐는 구현하지 않았습니다.
`logs/construct-eoat.png`에 두 공구 메시를 따로 그린 참고 이미지가 있습니다.

ZED 확장은 **ZED SDK가 좌우 영상으로 깊이를 계산**할 수 있도록 stereo 영상을 송신합니다.
공식 Outpost에 추가한 로컬 입력 어댑터가 이 스트림을 받아 Michelo로 전달합니다.
공식 [Stereolabs Isaac 확장](https://docs.stereolabs.com/docs/integrations/isaac-sim)의
stereo streaming 경로를 사용하며 `streamDepth=false`입니다.
`applyZedSim2Real=false`: 해당 잡음 모델은 ZED X에 교정되어 있어 2i의 실측 특성으로
소개하지 않습니다. 조명·재질·이상적인 렌즈와 가상 보정값을 사용하는 시뮬레이션이므로
실물 ZED 2i의 잡음·왜곡·노출·움직임 흐림까지 일치하는 디지털 복제는 아닙니다.

### 공식 서비스 교체 (2026-10-13)

사용자가 받은 `~/Downloads/backend.bridge.linux-main.zip`을
`~/robotics/official-outpost/`에 풀고 ZED 네트워크 입력 어댑터를 추가했습니다.
공식 daemon, HTTP/WS/IPC와 Michelo Library의 프레임 변환·깊이 처리를 사용합니다.
자체 `construct_outpost.py` 서버, Construct용 게이트웨이 실행기,
자체 타임스탬프 필드에 의존하던 ROS 카메라 브리지는 제거했습니다.

- 공식 Outpost 0.1.0 / Michelo Library 0.87.0 / Python 3.11 / ZED SDK 5.3
- 공식 Michelo Backend 0.63.1: SNUCEM에 배포된 source.bundle의
  `ff9716e17bd60d2fa48ed85b97ebb21492445fee` (`../official-michelo-builder/backend`)
- Outpost 콘솔: <http://127.0.0.1:8100/console/>
- Michelo 카메라 시험 UI: <http://127.0.0.1:8101/test/manual/camera/>
- Michelo API: <http://127.0.0.1:8101/docs>

### Isaac 스트림 입력 추가

Library 0.87.0의 기본 `ZedCameraAdapter`는 USB 시리얼만 받으므로,
이를 상속하는 `outpost/zed_stream.py`를 로컬에서 추가했습니다.
`camera_id=stream://127.0.0.1:30000`일 때 SDK `InitParameters.set_from_stream`으로
장치를 열고, 공식 라이브러리의 RGB/깊이 변환과 공식 Outpost/Michelo 전송을 재사용합니다.
실제 USB 카메라는 필요하지 않습니다. 일반 USB 설정은 기존 어댑터를 사용합니다.
이는 **공식 배포에 포함된 기능이 아닌 로컬 확장**이며 Library 0.87.0에 고정되어 있습니다.
라이브러리 업그레이드 시 보호된 메서드와 장치 핸들 구조를 다시 검증해야 합니다.

원본 Outpost의 `adapters.py` 한 파일을 수정하고 어댑터와 테스트 파일을 추가했습니다.
재적용 가능한 변경은 `patches/outpost-zed-stream.patch`에 있으며
`setup_construct_zed.sh`가 중복 적용 없이 설치합니다. 원본 ZIP, Michelo wheel,
공식 Michelo Backend 소스 및 SNUCEM main은 수정하지 않았습니다.

```text
Isaac / ZED 2i stereo → Stereolabs stream :30000
    → 공식 Outpost :8100 + 로컬 ZED 입력 어댑터
    → 공식 Michelo remote_camera / stream_relay :8101
    → 공식 카메라 시험 UI (좌/우 RGB, 깊이)
```

실행:

```bash
cd ~/robotics/sketch-robot-runtime-main
python3 scripts/setup_construct_official.py
bash scripts/setup_construct_zed.sh
OMNI_KIT_ACCEPT_EULA=YES python3 scripts/run_construct_demo.py
```

이 실행기는 Isaac·MoveIt·H형강 장면과 두 공식 서비스를 실행하고, 송신 영상이
갱신되는 것을 확인한 다음 `Construct ZED 2i (Isaac)` 카메라를 자동 등록/연결합니다.
`CONSTRUCT_ZED_READY`가 준비 완료 표시입니다. 공식 카메라 시험 UI에서 해당 카메라를
선택하고 **③ 동작 → 스트림 → rgb / depth / rgb_right / stereo → 시작**을 누릅니다.
공식 서비스는 구독자가 있을 때 송신하고 마지막 구독자가 나가면 송신을 정지합니다.

ZED SDK의 NEURAL_LIGHT 모델은 처음 사용할 때 GPU 최적화에 수 분 걸릴 수 있습니다.
현재 PC에는 최적화 캐시가 생성되어 있습니다. 새 환경의 첫 연결이 시간 초과되면
SDK 최적화 완료를 확인한 뒤 카메라 연결을 다시 시도합니다.
`--no-window`는 Isaac GUI를 생략합니다. Ctrl+C로 실행기가 시작한 프로세스를 종료합니다.
서비스만 따로 실행하려면 각 터미널에서 `bash scripts/run_construct_camera.sh`,
`bash scripts/run_construct_michelo.sh`를 실행합니다.

SDK 5.3은 `../zed-sdk-5.3`, 공식 Outpost의 Python 환경은
`../official-outpost/.venv`에 있습니다. 기존 SDK 5.5 다운로드는 보존하지만
공식 Outpost 실행에는 사용하지 않습니다. 시스템 CUDA·드라이버·udev는 변경하지 않습니다.
Stereolabs Isaac 확장은 `../zed-isaac-sim`의 커밋
`0164268ca123fa4549fc9d2062c1fd55cf7996dc`를 사용합니다.

Michelo 데이터는 `.runtime/construct/michelo-profiles`에 별도로 둡니다.
기본 프로필의 실제 네트워크 PLC 예제를 제거하고 가상 ZED 카메라만 자동 등록합니다.
실제 로봇이나 USB 카메라를 자동 연결하지 않습니다. 카메라 중계용 Michelo 프로세스는
GPU를 사용하지 않으며 Isaac/Outpost와 GPU를 공유하는 비전 기능은 별도 설정 대상입니다.
공식 소스 ZIP과 SNUCEM main은 수정하거나 push하지 않았습니다.

두 공식 lockfile이 Linux Open3D wheel의 `ipywidgets` 의존성을 누락하여,
소스/lock 수정 없이 각 가상환경에 `ipywidgets==8.1.9`를 보충했습니다.
두 환경 모두 `uv pip check`를 통과했습니다. 향후 backend에서 `uv sync`를 다시
실행하면 `uv pip install --python .venv/bin/python ipywidgets==8.1.9`를 적용합니다.
Outpost 설치 스크립트에는 이 보충 설치가 포함되어 있습니다.
공식 backend의 `test-camera --no-browser`로 개발용 시험 화면을 활성화하며,
Windows 전용 Michelo 프런트엔드 설치 프로그램은 실행하지 않았습니다.

검증: 공식 Outpost 테스트 **82개(기존 66 + 추가 16)** 통과.
실제 SDK → Outpost raw IPC → Michelo WebSocket에서 1280×720 좌우 RGB와
uint16 mm 깊이 30프레임을 수신했습니다. 설정 10fps에서 관측 약 8.2fps,
검사한 깊이 프레임의 유효 픽셀은 약 92.8%였습니다. 시퀀스·시각 증가와
JPEG/PNG16 디코딩도 확인했습니다. 실행기를 재시작한 뒤 같은 카메라 ID로
자동 연결되는 것을 확인했습니다. 반복 검사는 다음 명령으로 실행합니다.

```bash
../official-outpost/.venv/bin/python scripts/check_construct_stream.py
```

현재 기록: `logs/official-zed-stream-validation.json`,
`logs/official-services-validation.json`. 수신 이미지: `logs/official-zed-rgb.jpg`,
`logs/official-zed-rgb_right.jpg`, `logs/official-zed-depth.png`(uint16 mm).

카메라 mount는 원본 `zed2i_link`를 따라 움직이고, 공식 USD의 두 광학 중심은
정확히 120mm 간격으로 정규화합니다. 별도 카메라 강체는 제거하고 매 물리 스텝 후
USD와 렌더러 Fabric에 광학 중심의 월드 자세를 동기화합니다. CAD의 +Z전방/−X좌측/−Y상방을
ZED 좌표로 회전하며 determinant=+1을 검사합니다. 이 프로필은 RGB/깊이용이며 IMU는 제공하지 않습니다.
`zed_left_camera_frame`은 x전방/y좌측/z상방,
`zed_left_camera_frame_optical`은 x우측/y하방/z전방입니다.
공식 Library의 프레임 시각은 소프트웨어 Unix 시각이며, 정확한 Isaac 촬영 시각이나
ROS `/clock`으로 매핑한 값이 아닙니다. SDK 영상 시각은 중복 프레임·시계 역행 감지에
사용합니다. 송신 sidecar는 자동 연결 전 송신부 준비 확인에 사용합니다.
ROS 영상/깊이/포인트클라우드 브리지는 이번 입력 확장에 포함하지 않았습니다.
최초 실행 시 MoveIt이 왼팔 첫 관절을 0.6rad로 비켜 시야를 확보한 뒤 헤드 pitch를
0.7rad로 옮깁니다. 두 동작 모두 실제 로봇 메시로 충돌을 검사합니다.

H형강은 x축 방향 길이1.5m, 단면300×300mm, 플랜지20mm, 웨브10mm이고,
중심은 World `(0, -1.30, 1.05)`m입니다. 하단 높이0.90m의 받침대에 고정합니다.
두 플랜지·웨브·받침대 4개를 동일한 7개 box로 PhysX와 MoveIt에 등록합니다.
원본의 숨겨진 testbed 충돌 형상(y≈-1.95m)은 보존합니다.
원본 메시는 제품 CAD이며 이 H형강·받침대는 새 작업물의 단순화된 시뮬레이션 형상입니다.

예전 SDK/ROS 통합 검사와 제거된 호환 서버의 검증 수치는 현재 공식 서비스의
성공 근거로 사용하지 않습니다.
`logs/construct-camera-validation.json` 등 이전 결과는 교체 전 기록입니다.

### 충돌 URI 수정

추가 검증에서 ROS resource_retriever가 bare absolute STL 경로를 거부하는 것을 발견했습니다.
생성되는 MoveIt 전용 `construct_control.urdf`에는 `file://` URI를 쓰도록 수정했고,
Isaac importer용 `construct.urdf`는 로컬 절대경로를 유지합니다. 앞선 robot-only 검사의
충돌 항목은 메시가 누락된 상태였으므로 확장 작업의 재검증 결과로 대체합니다.

### 이전 검증의 범위

2026-10-12의 SDK RGB/깊이·ROS 점군 검사는 제거된 자체 서버 경로에서 수행했습니다.
공식 Outpost 경로는 별도 검증 대상입니다. 로봇 STL 충돌 메시 로딩, 카메라 120mm 간격,
양팔/헤드 계획 실행, H형강/받침대 7개 형상 배치는 이번 서비스 교체에서 유지합니다.


## 기존 스케치 런타임으로 Construct 실행 (2026-10-13)

VNC 바탕화면 안의 터미널에서 실행합니다. DISPLAY를 따로 고정하지 않습니다.

```bash
cd ~/robotics/sketch-robot-runtime-main
bash scripts/run_system_api.sh --construct
```

- 원래 작업관리 화면: <http://127.0.0.1:8081/>
- 원래 minjeasung 스케치 화면: <http://127.0.0.1:8081/sketch/>
- 공식 Michelo 카메라 화면: <http://127.0.0.1:8101/test/manual/camera/>

관리 화면의 `Construct · Isaac Sim` 모드에서 팔을 선택하고 전체 시작합니다.
원래 스케치 순서인 **대상 선택 → 평면 추출 → 선택한 ZED 면 확정 → 작업영역
확정 → 그린 경로 사용 / 자동 도포 경로 생성 → 작업 시작**을 그대로 사용합니다.
완료 후 전체 종료, 터미널 Ctrl+C로 종료합니다.

### 재사용 범위와 장비 차이

별도 Construct 스케치 서버나 UI를 사용하지 않습니다. 원본 `system_api`,
`Supervisor`, `outpost_bridge`, `target_selector`, `wall_projector`,
`sketch_to_waypoints`, `moveit_executor`를 같은 흐름으로 사용합니다.
원본 스케치 HTML/CSS와 조작 흐름을 유지합니다. 작업관리의 실행 모드/모델
선택과 시뮬레이션 안내 문구만 추가합니다.

공식 Michelo가 공식 Outpost의 카메라를 관리하며, Isaac ZED 2i 네트워크 입력은
기존 공식 어댑터 확장을 사용합니다. `construct_camera_keepalive.py`는 공식
Michelo의 local-delivery WebSocket 구독을 유지하는 클라이언트입니다.
새 카메라 서버나 SDK 소유자를 추가하지 않습니다.

Construct 장비 설정은 원래 실행기에 양팔의 그룹·TCP·관절·컨트롤러·SRDF를
제공합니다. 충돌 및 작업 경로 검증은 기존 실행기가 수행합니다. 실제 로봇
프로파일과 드라이버는 그대로 유지하며 Isaac 프로파일에서 실행하지 않습니다.

현재 Construct 프로파일은 **ZED 평면을 이용한 비접촉 경로 이동 검증**입니다.
기본 팔은 왼팔, 표면 이격은 0.30m, 채우기 폭은 0.05m입니다. 기존
`spray_motion_test` 동작을 사용하여 분사 출력은 항상 꺼집니다. 가상 TCP와
장착물은 원본 Construct URDF에서 가져오며, `isaac_assets/construct_*_eoat.json`
프로파일은 시뮬레이션 전용입니다. 실제 노즐 보정값으로 사용하지 않습니다.
D405 접촉 도장·F/T 센서 시뮬레이션을 구현한 것으로 표시하지 않습니다.

Outpost SDK는 벽시계 시각을 사용하고 Isaac TF는 시뮬레이션 시각을 사용하므로,
시뮬레이션의 기존 ROS 브리지는 수신 시점의 ROS 시각으로 메시지를 기록합니다.
노출 시각과 정확히 동기화한 데이터는 아닙니다. 대상 선택·경로 생성 중에는
카메라 헤드를 고정해서 사용합니다. 실장비의 원래 타임스탬프 처리는 유지합니다.
Isaac은 실시간보다 느리게 진행될 수 있어, Construct 실행기의 기존 FJT 결과
대기 여유를 120초로 설정합니다. 유한 시간 제한·상태 신선도·중단 처리는 유지합니다.

### 빌드 및 변경 파일

```bash
bash scripts/build_construct_sim.sh
```

위 명령은 Construct 설명/제어 플러그인과 기존 sketch_control,
rbpodo_painting_control, rbpodo_msgs를 동일한 Construct overlay에 빌드합니다.
시뮬레이터 실행을 관리하는 `run_construct_demo.py`가 카메라 스트림과 초기
카메라 자세까지 확인한 뒤 기존 관리 서버에 준비 상태를 알립니다.
원래 실장비 설정 YAML을 수정하지 않고 Construct용 설정만 병합하여 사용합니다.

이전에 만든 별도 Construct UI와 HTTP 스케치 실행기는 실행 경로에서 제거했고,
로컬 `.runtime/retired-construct-sketch/`에 보관했습니다. 이전 스케치 검증 수치는
그 별도 구현에 대한 것이므로 현재 원본 런타임의 검증 결과로 인용하지 않습니다.

### 원본 런타임 통합 검사 (2026-10-13)

원본 브라우저 화면에서 Isaac ZED 영상 → 평면 추출 → 작업영역 →
스케치 경로 생성 → 원래 MoveIt 실행기의 충돌 검사 및 관절 명령까지 확인했습니다.
H형강의 단색 재질은 스테레오 깊이를 불안정하게 만들어, 동일한 충돌 형상 위에
밀스케일을 표현한 표면 질감을 추가했습니다. 깊이는 계속 실제 ZED SDK가
좌우 영상에서 산출하며 Isaac의 정답 깊이로 대체하지 않습니다. 검사 ROI에서
평면 RMS는 약 1.3mm, World 좌표 높이는 약 1.201m(모델 상면 1.200m),
법선은 약 (0, 0, 1)이었습니다. 이는 해당 장면의 한 측정 결과입니다.

통합 중 확인한 원래 실행기 문제도 수정했습니다. 충돌 장면을 topic과 service로
중복 적용하던 부분은 응답을 확인하는 service만 사용합니다. 진단용 TF 조회는
콜백 처리를 막지 않도록 즉시 조회하여, 기존 300ms 상태 신선도 검사를 유지합니다.
관리 화면에서 시작 중에도 종료가 가능하며, 새 시작 요청과 종료 요청이 경합해
종료 후 프로세스가 다시 시작되지 않도록 회귀 검사했습니다.

검사 로그는 `logs/original-runtime-*-tests.log`에 있습니다. 관리 API 83개,
브라우저 68개, 주요 연결 72개가 통과했고 최종 실행기/장비 검사 49개도 통과했습니다
(검사 범위는 일부 중복됩니다). 전체 기존 실행기 검사에는 원본에서도 재현되는
fixture 실패 및 Construct overlay에 설치하지 않은 RB 패키지 의존 오류가 남아
있으므로 전체 테스트가 통과한 것으로 간주하지 않습니다.

최종 왼팔 통합 실행에서 접근 궤적 79점과 스케치 Cartesian 궤적 14점이 모두
컨트롤러 `Goal reached, success!`로 완료됐고, 원래 실행기는
`all explicit segments complete`를 기록했습니다. 이어 별도 실행 중 원본 UI의
작업 중단을 눌러 컨트롤러 취소 수락과 `running=false`, `motion_abort`를 확인했습니다.
증거: `logs/original-runtime-ui-completed.json`, `logs/original-runtime-final-status.json`,
`logs/construct-moveit.log`, `logs/system_api/executor.log`,
`logs/original-sketch-completed.png`, `logs/original-sketch-cancelled.png`.
오른팔 모델 계약/설정은 검사했지만 브라우저에서 끝까지 이동한 검증은 왼팔 기준입니다.

### 렌더링과 물리 주기 분리 (2026-10-13)

VNC에서 약 0.21배속이던 실행을 개선했습니다. 물리 시간 간격은 0.01초
(시뮬레이션 기준 100Hz)로 유지하고, 카메라/뷰포트 렌더링은 20Hz로 분리합니다.
ROS 관절 상태·명령·시계 그래프는 OnPhysicsStep에서 실행하므로 렌더링을
줄여도 제어 주기를 함께 낮추지 않습니다. 카메라 해상도는 HD720, 깊이는
공식 SDK의 스테레오 계산을 그대로 사용합니다.

실제 VNC 재시작 후 측정은 약 0.65~0.70배속(기존 대비 약 3.1~3.3배)입니다.
ROS로 별도 측정한 9.19초 동안 시뮬레이션은 6.44초 진행했고, 관절 상태는
시뮬레이션 1초당 약 100회였습니다. 좌우 RGB 1280×720과 16bit 깊이도
수신 확인했습니다. 초기 카메라 자세를 위한 팔/헤드 이동은 완료됐습니다.
이는 이 PC/VNC의 현재 장면 측정치이며, 장면/창 크기/부하에 따라 달라집니다.

`logs/construct-performance.json`은 5초마다 실제 진행률과 물리·렌더 비용을
기록합니다. 검증 결과는 `logs/construct-performance-validation.json`,
`logs/construct-performance-camera.json`에 있습니다.
`CONSTRUCT_RENDER_HZ`로 렌더 주기를 조절할 수 있고 기본값은 20입니다.
일시정지 중에도 UI는 갱신되며 재개 시 누적 시간을 급하게 따라잡지 않습니다.

참고: [NVIDIA Physics Simulation Fundamentals](https://docs.isaacsim.omniverse.nvidia.com/latest/physics/simulation_fundamentals.html)의 물리/렌더 주기 분리와 설치된 Isaac Sim의 OnPhysicsStep 시계 테스트를 기준으로 적용했습니다.

## 3D 용접심 이동 시험

기존 시스템관리의 작업 방식을 `용접`으로 선택하면 동일한 ZED 스케치 화면에서 시험 심 선택과 Settings를 사용할 수 있습니다. 실행 순서, 추출기 계약, 통합 검증 방법은 [Isaac 용접 이동 시험](WELDING_ISAAC.md)을 참고하세요.

## 왼팔 뿜칠 v2 EOAT (2026-10-13)

생성 모델의 왼팔을 Downloads의 `EOAT ass_y_ROBOI_Sharing_v2 (2).stp`에서
추출한 강체 EOAT로 교체했다. 원본 Construct URDF와 오른팔은 유지한다.
`assets/construct/spray_v2/mount.json`에 원본 해시, 좌표 변환, 포함·제외 부품과
시뮬레이션 가정을 기록했다. 기존 왼팔 작업툴과 D435는 생성 모델에서 제거한다.

- CAD 로봇 접합면 중심: `(0, -67, 0)` mm.
- CAD `(X,Y,Z)` → 플랜지 `(Y,Z,X)` 축 대응, 출력 단위 m.
  즉 플랜지 좌표는 `(CAD_Z, CAD_X, CAD_Y+67)/1000`이다.
- 플랜지: `left_manipulator_link6` 기준 `xyz=(0,0,0.0543)`, `rpy=(0,0,0)`.
- TCP: 플랜지 기준 `xyz=(-0.110,0,0.2822)`, `rpy=(0,0,0)`.
  `nozzle_1` 전면 Y=215.2 mm를 기준으로 잡은 CAD 명목 TCP이며 실측 보정값은 아니다.
- 분사축: TCP `+Z`, 손목 축과 평행. 기본 자세에서 노즐은 몸통에서 멀어지는 쪽으로
  110 mm 치우친다. 손목 회전에 따라 세계좌표의 방향은 달라진다.
- 센서 AFT200-D80-EN과 장착판은 포함한다. 중복 손목, `spray coat` 분사 원뿔,
  유연 호스 1·2·3은 강체 메시에서 제외했다. 호스 간섭·처짐은 별도 모델이 필요하다.
- 질량 2 kg 및 외접 상자 기반 관성은 시뮬레이션 임시값이다. 재질·실측 질량으로
  교정한 동역학이나 유체 분사, 힘센서 출력은 이번 모델 교체에 포함하지 않는다.
- MoveIt은 생성 `isaac_assets/construct.srdf`를 사용한다. 기존 왼팔 작업툴의
  충돌 제외쌍을 이어받지 않으며, 손목과 EOAT의 인접 제외쌍만 유지한다.

현재 왼팔은 뿜칠 공정으로 사용한다. 용접 시험 토치를 사용하려면 적합한 팔·도구
구성을 선택해야 한다. 기존 용접 토치를 새 분사건 TCP에 덧붙이는 것은 올바른
장착 모델이 아니다.

메시 재추출은 독립 CAD 환경에서 `cadquery-ocp==8.0.1.1.0`, `trimesh`,
`fast-simplification`, `numpy`, `scipy`를 설치하고 실행한다. 기존 체크인된
`mount.json`의 장착 사양을 사용한다.

```bash
python scripts/export_spray_v2_step.py '/path/to/EOAT ass_y_ROBOI_Sharing_v2 (2).stp'
python3 scripts/prepare_construct_assets.py
.venv-api/bin/python scripts/prepare_construct_runtime.py
OMNI_KIT_ACCEPT_EULA=YES bash scripts/setup_construct_isaac.sh
```

USD 재생성 전에는 실행 중인 시뮬레이션을 종료한다. 기존 USD 백업은
`.runtime/backups/before-spray-v2-*`에 있다. ROS와 Isaac은 재생성 후 함께 재시작한다.
