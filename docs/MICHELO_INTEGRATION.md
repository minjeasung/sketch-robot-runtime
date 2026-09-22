# Michelo에서 스케치 UI 열기

## 현재 구현

기존 Outpost 콘솔에 **스케치 작업 ↗** 링크를 붙여 새 탭으로 스케치 UI를 연다.
Michelo 설치 파일, 카메라 데몬, 참고 Git 저장소는 수정하지 않는다.
`8101` 연결 서버가 기존 `8100` 콘솔의 HTTP/WebSocket을 전달하고 HTML 응답에
버튼 스크립트 하나를 삽입한다. 원본 `8100` 화면에는 버튼이 생기지 않는다.
별도 Windows Michelo 메인 프런트엔드를 이식하거나 수정한 것은 아니다.

```text
브라우저 → :8101/console/ → 기존 Outpost :8100
              └─ 스케치 작업 ↗ → :8081/sketch/ (새 탭)
                                  ├─ 시스템 관리 :8081/
                                  └─ rosbridge :9090
ZED/D405 → Outpost → 원본 ZMQ IPC → sketch_outpost_bridge → 기존 인식·경로·실행부
```

화면을 열거나 링크를 누르는 동작은 로봇을 기동하거나 움직이지 않는다.
기존 Michelo Supervisor의 `8080`은 그대로 두고 스케치 API 기본 포트는 `8081`로 구분한다.

## 설치·실행

ROS 런타임 설치는 `PORTABLE_INSTALL.md`를 따른다. Outpost는 기존 설치본을 사용한다.
새 로봇 PC에서는 먼저 참고 저장소의 Outpost 설치 안내에 따라 데몬을 준비한다.
Outpost를 사용할 때 스케치 측 ZED SDK/드라이버 빌드는 필요하지 않다:

```bash
bash scripts/install_runtime.sh --install-deps --without-zed
bash scripts/setup_system_api.sh  # 기존 PC도 이번에 추가된 WS 의존성을 설치
cp config/sketch_runtime.env.example config/sketch_runtime.env  # 최초 설정 때만
```

`config/sketch_runtime.env`에서 실제 카메라 값을 입력한다. 파일은 Git에서 제외된다.

```bash
SKETCH_SUPERVISOR_HOST=127.0.0.1
SKETCH_SUPERVISOR_PORT=8081
SKETCH_MICHELO_HOST=127.0.0.1
SKETCH_MICHELO_PORT=8101
SKETCH_CAMERA_BACKEND=outpost
SKETCH_OUTPOST_HTTP=http://127.0.0.1:8100
SKETCH_OUTPOST_ZED_HW_ID=실제_논리_ID
SKETCH_OUTPOST_ZED_SERIAL=보정한_ZED_시리얼
SKETCH_OUTPOST_D405_HW_ID=실제_논리_ID
SKETCH_OUTPOST_D405_SERIAL=보정한_D405_시리얼
SKETCH_LAUNCH_ZED_DRIVER=false
SKETCH_LAUNCH_D405_DRIVER=false
SKETCH_PROFILE=dry_run
```

관리 화면의 **연결된 카메라 목록 불러오기**에서 카메라를 명시적으로 선택해도 된다.
화면 설정은 해당 서버 실행 동안만 유지된다. 재시작 후에도 유지할 값은 위 파일에 넣는다.
입력 시리얼은 작업에 사용할 보정값과 일치하는지 운영자가 확인한다. 브리지는 입력한
시리얼과 데몬의 실제 장치 시리얼을 대조하며 다른 장치로 자동 대체하지 않는다.

```bash
bash scripts/run_michelo_sketch.sh
```

한 터미널에서 스케치 API와 연결 서버를 함께 실행한다. 둘 중 하나가 종료되면 다른
서버도 종료한다. 이미 스케치 API가 실행 중이면 중복 실행하지 말고 다음만 실행한다:

```bash
bash scripts/run_michelo_gateway.sh
```

브라우저에서 `http://127.0.0.1:8101/console/`에 접속한다.
Michelo에서 ZED/D405 연결 → RGB·깊이 스트리밍 시작 → 스케치 작업 버튼 → 시스템 관리에서
카메라 ID·시리얼 확인 → 전체 시작 → 스케치 순서다. D405는
`camera_type=realsense`, `enable_color=true`, `enable_depth=true`,
`align_color_to_depth=true`로 연결한다. 예: 640×480, 30 FPS 연결 / 15 FPS 스트림.
SDK `point_cloud`는 raw IPC에 제공되며 인코딩 스트림 채널 선택에 추가하지 않는다.

## snucem / Minjea 실행 계정

웹 화면은 OS 계정이 달라도 HTTP로 접속할 수 있다. **원본 IPC는 파일 권한을 따른다.**
이 PC에서 확인한 `/tmp/snucem-xr-outpost-ipc`는 `snucem` 소유 `0700`이므로,
현재 `Minjea`가 실행하는 ROS 브리지는 이를 읽을 수 없다. 이 변경은 권한을 변경하지 않는다.

권장 배치는 Outpost와 스케치 ROS 실행부를 같은 서비스 계정에서 실행하고, 사용자는
자기 계정 브라우저에서 접속하는 것이다. `snucem` 계정에 이 개인 저장소를 별도로
복제·설치하면 참고 저장소의 코드와 기존 설치 파일을 건드리지 않고 운영할 수 있다.
반대로 Outpost를 Minjea 계정에 별도 설치할 수도 있지만, 기존 데몬을 종료하고 카메라
소유권을 넘기는 절차가 필요하다. 두 SDK 프로세스로 동일 카메라를 동시에 열지 않는다.
계정 공유가 반드시 필요하면 소켓 소유자가 전용 그룹/ACL을 명시적으로 설정해야 하며
재연결 때 소켓이 재생성되는 경우도 처리해야 한다. 홈 디렉터리 전체 공개는 필요 없다.

## 원격 PC

두 웹 서버의 HOST를 `0.0.0.0`으로 바꾸고 다음을 각각 설정한다:

- `SKETCH_SUPERVISOR_API_TOKEN`: 시스템 관리 화면의 Bearer 토큰.
- `SKETCH_MICHELO_TOKEN`: 연결 서버의 HTTP Basic 비밀번호. 사용자명은 `sketch`.

`http://로봇PC주소:8101/console/`에서 연다. 링크는 브라우저에서 사용한 호스트와
`SKETCH_SUPERVISOR_PORT`를 사용하므로 원격 PC의 localhost로 이동하지 않는다.
다른 주소/HTTPS 프록시를 사용하는 경우 `SKETCH_PUBLIC_URL`에 전체 스케치 URL을 지정한다.
관리 화면의 Michelo 링크는 기본 `8101`을 사용한다. 포트를 바꾸었다면 연결 서버 주소를
직접 연다. 신뢰하는 LAN에서만 사용한다. 기존 rosbridge `9090`은 API 토큰으로 보호되지
않으며 인터넷에 공개하지 않는다. TLS 및 다중 사용자 제어권 중재는 이번 변경 범위가 아니다.

## 카메라 데이터·실패 처리

브리지는 GET 상태 조회와 ZMQ 구독만 한다. connect/start/stop/disconnect를 보내지 않는다.
카메라 타입·논리 ID·시리얼·내부 파라미터·해상도·generation과 IPC 접근 권한을 검사한다.
프레임은 RGB/깊이 동일 격자, 채널 길이·단위, 타임스탬프·시퀀스를 확인한다.
오래된/역순 프레임을 버리고 3초 동안 유효 프레임이 없으면 종료한다. 재연결/generation,
해상도, 내부 파라미터가 바뀌어도 자동으로 작업을 이어가지 않고 인식을 다시 시작해야 한다.

브리지 종료는 perception launch 전체를 종료한다. Supervisor는 perception 종료를 감지해
의존 executor를 중단하며 기존 abort 절차를 사용한다. 전체 시작 전 카메라 사전 검사를
수행하므로 장치 누락/IPC 권한 오류 때문에 로봇 스택만 먼저 켜지는 것을 막는다.
기존 외부 `controller_manager`/`move_group`가 발견되면 중복 로봇 제어 시작도 거절한다.
별도 ROS domain이나 외부 독립 제어기의 명령까지 중재하는 전역 제어권 장치는 아니다.
Michelo의 다른 로봇 제어 기능과 스케치를 동시에 조작하지 않는다.

| 입력 | ROS 출력 |
|---|---|
| ZED RGB | `/zed/zed_node/rgb/color/rect/image` + `/zed/zed_node/rgb/color/rect/camera_info` |
| ZED depth | `/zed/zed_node/depth/depth_registered` + `/zed/zed_node/depth/camera_info` |
| ZED XYZ | `/zed/zed_node/point_cloud/cloud_registered` |
| D405 RGB | `/d405/d405/color/image_raw` + `/d405/d405/color/camera_info` |
| D405 depth | `/d405/d405/depth/image_rect_raw` + `/d405/d405/depth/camera_info` |
| D405 SDK XYZ | `/d405/d405/depth/color/points` |

영상은 RGB8, 깊이는 32FC1 미터 단위다.
ZED 점군은 광학 좌표계 `zed_left_camera_frame_optical`의 XYZ 미터로 발행한다.
D405는 SDK compact XYZ(mm)를 유효 픽셀에 복원해 미터로 변환하여 소수 mm 정보를 보존한다.
기존 D405 광학 보정 체인 `tcp → d405_link → d405_color_optical_frame`을 발행하며,
D405의 RGB가 깊이 격자에 정렬된다는 전제를 검사한다. URDF의 카메라 외형 위치를 변경하지 않는다.
Windows 미리보기용 JPEG/PNG16 WebSocket 프레임을 정밀 측정 점군으로 대체하지 않는다.

기존 ROS 카메라 방식이 필요하면 `camera_backend=native` 및 각 드라이버 실행 여부를
명시적으로 설정한다. `fake` 프로필에서는 Outpost 브리지를 실행하지 않는다.

## 검증 범위

오프라인 원본 IPC → ROS 메시지 검증은 다음 명령으로 재현할 수 있다. 가짜 ZED/D405를
ROS domain 227에 발행하고 데이터 수신 중단을 확인한다. 실로봇이나 카메라를 열지 않는다.

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH="$PWD/src/sketch_control:$PYTHONPATH" /usr/bin/python3 scripts/verify_outpost_bridge.py
```

원본 프레임 계약, SDK XYZ 정밀도/단위, 프레임 신선도, 카메라 식별, IPC 오류,
네이티브 드라이버 중복 방지, 로봇 시작 전 검사, 콘솔 버튼과 HTTP/WS 전달을 검증한다.
실로봇 이동과 D405 실측은 별도 장비 시험이다. 기존 접촉 도장 테스트 10건과
RB10/RB20 fake MoveIt 종료 오류는 이 변경으로 해결한 것으로 취급하지 않는다.

참고: [SNUCEM Outpost 구조](https://github.com/JongHyunSeo11/SNUCEM_Robot/blob/342b284be828a07c1afd266380087d18c157e998/linux/michelo/README.md),
[D405 원본 데이터 계약](https://github.com/JongHyunSeo11/SNUCEM_Robot/blob/342b284be828a07c1afd266380087d18c157e998/docs/outpost_realsense.md).
