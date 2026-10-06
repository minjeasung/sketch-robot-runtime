# SNUCEM Robot 22.04 스케치 뿜칠 애드온 — Jetson ARM64

기존 **JongHyunSeo11/SNUCEM_Robot_22.04** 설치에 스케치 화면, 측정 평면 선택,
작업영역 지정, 경로 생성·검증·자동 실행을 연결합니다. 대상 환경은
NVIDIA Jetson AGX Orin Developer Kit, **ARM64(aarch64), Ubuntu 22.04,
ROS 2 Humble, Python 3.10, CUDA 12.6, ZED SDK 5.3.1**입니다.

CPU/SDK/CUDA는 사용자가 확인한 실제 로봇컴 기준입니다. JetPack/L4T 세부 버전은
아직 전달되지 않았으므로 특정 버전으로 단정하지 않습니다. ZED SDK는 그 L4T에 맞는
**Jetson용 5.3.1 설치**를 그대로 사용합니다. ROS 애드온은 CPU와 ROS 토픽을 사용하며
CUDA/pyzed에 직접 링크하지 않습니다. GPU 런타임은 기존 카메라 환경에서 담당합니다.
원본의 `research/jammy/Dockerfile`에 있는 x86/CUDA 13.0 이미지를 이 애드온이
빌드하거나 실행하지 않습니다. SDK/CUDA/JetPack, 원본 파일은 설치·업그레이드하지 않습니다.

참고: [NVIDIA JetPack 6.1](https://developer.nvidia.com/embedded/jetpack-sdk-61)은
Ubuntu 22.04와 CUDA 12.6을 사용합니다. 이는 로봇컴의 JetPack 버전을 확인했다는 뜻은 아닙니다.
[Stereolabs 5.3 릴리스 기록](https://docs.stereolabs.com/docs/development/zed-sdk/release-notes/5-x/5-3)과
[공식 Jetson 이미지 목록](https://github.com/stereolabs/zed-docker)을 기준으로 설치 환경을 확인합니다.

지원 인터페이스 기준은 `7b0a2edcc3d1659bdc2dcb495d4dd63f75899f1c`입니다.
애드온은 원본 저장소를 clone/build/patch하지 않습니다. 배포 파일에 원본 소스,
로봇 mesh, 카메라 SDK, ROS 또는 Python 의존성은 포함하지 않습니다.
원본의 인식 클래스를 읽기 전용으로 import하고, 실행 중인 외부 스택의
URDF/SRDF/관절 제한/mesh 해시를 읽습니다. 원본 모델 캐시 생성기도 호출하지 않습니다.

## 설치 전 준비

- 로봇컴에서 `uname -m`이 `aarch64`인지 확인합니다. x86_64와 32비트 ARM은 설치/실행 대상이 아닙니다.
- `cat /etc/nv_tegra_release`, `dpkg-query -W nvidia-l4t-core`로 L4T를 확인합니다.
  기존 카메라를 실행하는 환경에서 `/usr/local/cuda/version.json` 또는 `nvcc --version`으로
  CUDA 12.6을 확인합니다. `nvidia-smi`의 CUDA 표시는 설치된 SDK 버전 확인을 대신하지 않습니다.
  같은 카메라 Python 환경에서 `python -c "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"`로
  ZED SDK 5.3.1을 확인합니다. 애드온 venv에는 pyzed를 설치하지 않습니다.
- 기존 Humble 로봇 드라이버, MoveIt, TF, ZED/Outpost와 실제 보정은 운영자가 준비합니다.
- `/robot_description`, `/move_group/get_parameters`, `/joint_states`,
  `/controller_manager/list_controllers`, MoveIt 서비스, `link0`/`tcp` TF가 필요합니다.
- 물리 실행에서는 `/joint_trajectory_controller/follow_joint_trajectory`를 사용합니다.
  기존 `rb20_spray_executor` 및 같은 액션을 사용하는 다른 실행기는 먼저 종료합니다.
- 원본 `rb20_spray_shared_autonomy` 인식 노드도 종료합니다. 애드온 wrapper가
  동일한 설치된 인식 클래스를 실행하되, 원본 제어 타이머 대신 측정 평면 내보내기를 수행합니다.
  드라이버·MoveIt·카메라는 계속 실행해야 합니다. 애드온은 외부 프로세스를 종료하지 않습니다.
- Python venv/pip, `python3-opencv`, Humble의 `moveit_msgs`, `control_msgs`,
  `controller_manager_msgs`, `tf2_ros`, `visualization_msgs`가 필요합니다.
  rosbridge는 외부 서비스를 사용하거나 `own_rosbridge: true`로 별도 실행합니다.
  후자는 설치된 `rosbridge_server` 패키지가 필요합니다.

## 별도 경로에 설치

아래 경로는 예시입니다. `upstream_root`는 **이미 설치된 원본 경로**로 바꿉니다.
설치·상태 경로는 원본과 분리하고 운영 계정만 쓸 수 있게 설정합니다.
하위 심볼릭 링크와 파일 하드 링크를 통한 쓰기도 차단합니다. 다른 프로세스가 동시에 경로를 바꾸는
공유 쓰기 디렉터리는 지원하지 않습니다.

```bash
source /opt/ros/humble/setup.bash
# 이미 구축된 원본 ROS overlay의 setup.bash도 현재 스택의 실행 절차에 따라 source합니다.
mkdir -p "$HOME/snucem-sketch-bootstrap" "$HOME/.local/state/snucem-sketch"
chmod 700 "$HOME/.local/state/snucem-sketch"
sha256sum -c snucem-spray-humble-arm64-cuda12.6.tar.gz.sha256
tar -xzf snucem-spray-humble-arm64-cuda12.6.tar.gz -C "$HOME/snucem-sketch-bootstrap"
```

`$HOME/.local/state/snucem-sketch/config.json` 예시:

```json
{
  "upstream_root": "/home/robot/SNUCEM_Robot_22.04",
  "install_root": "/home/robot/.local/share/snucem-sketch",
  "state_root": "/home/robot/.local/state/snucem-sketch",
  "calibration_file": "/home/robot/calibration/active-extrinsic.yaml",
  "profile": "preview",
  "model_id": "rb20_1900es",
  "ros_domain_id": 0,
  "rosbridge_url": "ws://127.0.0.1:9090",
  "own_rosbridge": false,
  "image_topic": "/zed/zed_node/left/color/rect/image",
  "camera_info_topic": "/zed/zed_node/left/color/rect/camera_info",
  "points_topic": "/rb/spray/zed/points",
  "api_host": "127.0.0.1",
  "api_port": 8081,
  "api_token": ""
}
```

보정 파일은 현재 외부 스택에서 사용하는 실측 extrinsic 파일을 지정합니다.
애드온이 해당 파일을 보정·수정·적용하지는 않습니다. 보정 해시와 실제 TF를 모두 추적합니다.
카메라 topic, frame, 해상도, ROS domain은 실행 중인 스택과 맞춥니다.
RB10은 `model_id: rb10_1300e_u`를 사용하고 실제 spray URDF와 일치해야 합니다.

```bash
python3 -B "$HOME/snucem-sketch-bootstrap/scripts/snucem_spray.py" \
  --config "$HOME/.local/state/snucem-sketch/config.json" \
  install --bundle "$PWD/snucem-spray-humble-arm64-cuda12.6.tar.gz" --setup-env
```

설치 출력의 `version`과 `version_root`를 사용합니다. 가상환경은
`install_root/envs/<version>`에 생성합니다. ARM64 Python 3.10 wheel만 설치하므로
의존성을 현장에서 소스 컴파일하지 않습니다. 의존성 설치가 실패하면 이전 활성 버전은 유지됩니다.
기존 설정을 덮어쓰지 않습니다. 실행 중인 서비스의 업그레이드는 거절합니다.
배포물 0.2.0은 ARM64/CUDA 12.6/ZED 5.3.1 메타데이터를 검증합니다.
이전 x86 기준 0.1.0 bootstrap 대신 새 압축 파일의 installer를 사용합니다.

```bash
ADDON_ROOT=/home/robot/.local/share/snucem-sketch/versions/<version>
ADDON_PYTHON=/home/robot/.local/share/snucem-sketch/envs/<version>/bin/python
ADDON_CONFIG=/home/robot/.local/state/snucem-sketch/config.json
"$ADDON_PYTHON" -B "$ADDON_ROOT/scripts/snucem_spray.py" --config "$ADDON_CONFIG" doctor
"$ADDON_PYTHON" -B "$ADDON_ROOT/scripts/snucem_spray.py" --config "$ADDON_CONFIG" serve
```

`http://127.0.0.1:8081`에서 추가 기능을 시작한 뒤 스케치 화면을 엽니다.
원격 브라우저에서는 `rosbridge_url`을 브라우저가 접근할 수 있는 주소로 바꿉니다.
API를 외부에 바인딩하려면 24자 이상의 `api_token`이 필요합니다.
토큰은 관리 HTTP 요청만 보호합니다. ROS/rosbridge 접근은 별도로 제한된 운영망에서 관리합니다.

## 실행 단계

| profile | 동작 |
|---|---|
| `preview` | 평면/작업영역/경로 미리보기. 로봇 실행기 생성 안 함 |
| `dry_run` | 실행기를 통한 계획 검증. 물리 명령 전송 차단 |
| `motion_test` | 실제 로봇 이동, 분사 ON 금지. 건을 분리한 시운전용 |
| `spray` | 실제 로봇 이동과 분사. 별도 건 제어기의 실제 상태·명령 ACK 필수 |

모드 변경은 서비스를 종료한 다음 설정을 고치고 재시작합니다. 준비 버튼만으로
로봇이 움직이지 않습니다. 스케치 화면에서 ZED 평면 선택 → 작업영역 지정 →
경로 작성/생성 → 검증 → 작업 시작 순서로 실행합니다. 중단 이후 이전 경로를 자동 재개하지 않습니다.

`spray`는 `/spray_gun/command`의 lease/session/command_id를 처리하고
`/spray_gun/status`로 실제 출력 상태를 응답하는 외부 하드웨어 어댑터가 있어야 합니다.
이 배포물은 장치별 건 드라이버를 생성하지 않으며 준비/응답 상태를 위조하지 않습니다.
기존 프로토콜 구현은 `src/sketch_control/sketch_control/spray_execution.py`에 있습니다.

평면은 원본의 고정된 측정 support에서 유한한 셀로 변환합니다. 추론된 면이나 셀 사이
빈 공간에는 작업영역을 허용하지 않습니다. 잠긴 영역이 새 관측으로 계속 덮이면
동일 영역을 유지하며, 새 측정점 때문에 자동 확대하지 않습니다. 더 넓은 영역을 사용하려면
인식 세션을 다시 준비하고 선택합니다. 평면 소실/실제 형상 변경/보정 변경, 모델 변경,
오래된 관측, 경쟁 실행기 또는 컨트롤러 활성화는 실행을 차단하고 진행 중 작업을 중단합니다.

## 종료·오류 확인

관리 화면의 종료 또는 서버 Ctrl+C는 애드온이 만든 자식 프로세스만 종료합니다.
실행기는 건 OFF와 보유 액션 취소를 요청하고 결과를 제한 시간 동안 처리한 뒤
자신의 scene object만 제거합니다. 네트워크 단절/강제 종료에서는 취소 응답이나
scene 정리를 보장할 수 없으므로 실제 로봇 정지 여부는 외부 스택에서 확인합니다.
하드웨어 건은 lease 만료 시 독립적으로 OFF가 되어야 합니다.

로그: `state_root/logs/`. 실시간 모델: `state_root/model-status.json`.
`EXTERNAL_MODEL_CHANGED`는 종료/재준비/재선택이 필요합니다. 원본 파일 해시 불일치는
지원 인터페이스 변경이므로 코드 검토 후 호환성 목록을 갱신해야 합니다.
프로세스 강제 종료 후 남은 `service-active.json`은 서비스가 실제로 종료됐는지 확인한 후
소유 상태 폴더에서만 제거합니다. 정상 종료에서는 자동 제거됩니다.

## 검증 범위

검증 결과와 미확인 항목은 [검증 기록](SNUCEM_SPRAY_VALIDATION.md)을 참조합니다.
`doctor` 성공은 정적 준비 상태 확인이며 실물 로봇·건의 시운전 통과를 뜻하지 않습니다.
`doctor`의 `camera_environment` 값은 **요구 버전**이며, 자동 감지 결과가 아닙니다.
`verified: false`를 유지하므로 위 명령으로 기존 카메라 환경을 별도 확인합니다.
GitHub의 ARM64 Humble CI는 외부 원본 접근 없이 synthetic model과 실제 ROS 메시지로
노드 생성/차단 경계를 검사합니다. 실제 카메라·MoveIt·FJT 전체 운전 시험은 별도입니다.
