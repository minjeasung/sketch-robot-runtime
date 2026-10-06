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

## Jetson과 Windows에서 실행하는 것

**Jetson은 로봇·카메라·애드온 서버를 실행하고, Windows는 브라우저로 접속합니다.**
스케치 작업 자체에는 Windows용 ROS, 애드온 설치 또는 별도 웹 서버가 필요하지 않습니다.
기존 카메라를 준비하는 데 사용하는 Michelo/Outpost 도구는 그 설치 환경의 절차를 따릅니다.

| 위치 | 실행 항목 | 역할 |
|---|---|---|
| Jetson | JongHyun `stack` | 로봇 모델·드라이버 또는 가상 하드웨어·MoveIt·로봇 TF 제공 |
| Jetson | 기존 Outpost 카메라 + `zed` | 카메라 스트리밍과 ROS 영상·CameraInfo·점군 제공 |
| Jetson | JongHyun `extrinsic` | 실측 카메라 외부보정 TF 제공 |
| Jetson | 애드온 `serve` | 관리 홈페이지와 스케치 홈페이지를 함께 제공 |
| Jetson | rosbridge | Windows 브라우저와 ROS 연결; 기존 서비스 또는 애드온 소유 방식 중 하나 |
| Windows | 브라우저 | 관리 화면에서 준비하고 새 탭의 스케치 화면에서 작업 |

JongHyun 원본은 다시 다운로드하지 않습니다. **다운로드만 완료한 상태와 설치·빌드가
완료된 상태는 다릅니다.** 애드온은 원본의 설치·빌드·카메라 스트리밍을 대신하지 않습니다.
아래 절차는 원본 런타임이 설치된 Jetson 기준입니다.

### main 관리 홈페이지와의 차이

두 화면을 쓰는 구조는 같지만 관리 화면과 관리 API는 동일하지 않습니다.

| 항목 | main | 이 애드온 |
|---|---|---|
| 관리 화면 | 로봇 IP·모드·카메라 선택, 전체/개별 프로세스 관리, 로그 | 애드온 시작·종료, 모드·모델·자식 프로세스 상태 표시 |
| 관리 API | `/configuration`, `/prepare-system`, `/shutdown-system`, `/processes/...`, `/outpost/cameras` | `/prepare`, `/shutdown`, `/status`, `/addon/capabilities`, `/health` |
| 설정 변경 | 관리 화면에서 설정 | `config.json` 수정 후 서버 재시작 |
| 스케치 화면 | 공통 스케치 UI | 같은 UI를 재사용하되 Spray 전용, 설정된 rosbridge·영상 토픽 사용 |

`/status`는 양쪽에 있지만 응답 구조는 다릅니다. 애드온 화면은 main의 카메라 목록
불러오기나 드라이버 전체 시작 기능을 제공하지 않습니다. 외부 로봇·카메라를 준비한 다음
애드온을 시작해야 합니다.

## 1. JongHyun 설치와 장비 정보 확인

원본의 안내와 실행 스크립트는 다음에 있습니다. 비공개 저장소 읽기 권한이 필요할 수 있습니다.

- [최초 설치 안내](https://github.com/JongHyunSeo11/SNUCEM_Robot_22.04/blob/7b0a2edcc3d1659bdc2dcb495d4dd63f75899f1c/INSTALL.md)
- [런타임 설치·빌드 안내](https://github.com/JongHyunSeo11/SNUCEM_Robot_22.04/blob/7b0a2edcc3d1659bdc2dcb495d4dd63f75899f1c/docs/runtime_setup.md)
- [뿜칠 실행 안내](https://github.com/JongHyunSeo11/SNUCEM_Robot_22.04/blob/7b0a2edcc3d1659bdc2dcb495d4dd63f75899f1c/docs/rb20_spray_shared_control.md)
- [실행 진입점](https://github.com/JongHyunSeo11/SNUCEM_Robot_22.04/blob/7b0a2edcc3d1659bdc2dcb495d4dd63f75899f1c/linux/bringup/run_rb20_spray.sh)

위 링크는 애드온이 확인한 원본 리비전에 고정했습니다. 원본 안내에 있는 기존 PC의
IP·시리얼·카메라 보정값을 현장 값으로 간주하지 않습니다. 특히 원본의 x86용 카메라
컨테이너 설치 명령을 Jetson에 그대로 적용하지 말고 이미 준비된 ARM64 카메라 환경을 사용합니다.

Jetson에서 아래 경로를 실제 JongHyun 폴더로 바꿔 확인합니다.

```bash
SPRAY_UPSTREAM="$HOME/SNUCEM_Robot_22.04"
test -x "$SPRAY_UPSTREAM/.venv/bin/python"
test -f "$SPRAY_UPSTREAM/.runtime/ros2_ws/install/local_setup.bash"
```

둘 중 하나라도 실패하면 원본 Python 환경/ROS 빌드 준비부터 확인합니다. 두 파일이
존재하는 것만으로 전체 설치나 실물 운전이 검증되지는 않습니다.
애드온 `doctor`는 원본 인터페이스 파일의 호환성도 검사합니다.
호환성 실패를 피하려고 원본을 강제 초기화하거나 해시 검사를 우회하지 않습니다.

다음 정보를 준비합니다.

| 정보 | 설정/용도 |
|---|---|
| JongHyun 설치 절대 경로 | `upstream_root` |
| Jetson 운영망 IP | Windows의 HTTP·WebSocket 접속 주소 |
| 실제 로봇 모델·로봇 제어기 IP | `stack` 인자와 애드온 `model_id` |
| Outpost 장치 ID·ZED 시리얼 | `zed --hw-id / --camera-id`; 서로 다른 값 |
| 현재 배치의 실측 외부보정 JSON | `extrinsic --calibration`과 애드온 `calibration_file`에 같은 파일 지정 |
| ROS domain | 이 안내는 JongHyun 뿜칠 기본값인 `87` 사용 |

### 모든 Jetson 터미널에서 맞출 ROS 환경

아래 블록을 **원본 프로그램·애드온·별도 rosbridge를 실행하는 각 터미널**에서 먼저 실행합니다.
실제 설치 경로로 수정합니다. 별도 운영 도메인을 이미 사용한다면 `87`과 이후 JSON의
`ros_domain_id`를 함께 바꿉니다. Windows는 브라우저만 사용하므로 ROS domain 설정이 없습니다.

```bash
SPRAY_UPSTREAM="$HOME/SNUCEM_Robot_22.04"
export ROS_DOMAIN_ID=87
source "$SPRAY_UPSTREAM/linux/bringup/ros_env.sh"
export SPRAY_CYCLONEDDS_URI="$CYCLONEDDS_URI"
cd "$SPRAY_UPSTREAM"
```

원본 스크립트는 `ROS_DOMAIN_ID`가 설정되지 않았을 때 87을 사용합니다.
위처럼 명시해야 기존 셸의 domain 0과 섞이지 않습니다. `ros_env.sh`는 원본의
`.runtime/ros2_ws/install/local_setup.bash`를 불러옵니다.
`SPRAY_CYCLONEDDS_URI`는 원본 뿜칠 프로세스도 같은 DDS 설정을 사용하게 합니다.
ROS 통신은 Jetson 안에서 이루어지고, Windows는 HTTP/WebSocket으로 연결합니다.

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

## 2. 애드온 다운로드·설치 — Jetson에서 한 번

### 브랜치 다운로드와 배포 파일 만들기

원본과 다른 폴더에 애드온을 받습니다. 아래 명령은 새 다운로드 기준이며, 같은 폴더가
이미 있다면 다시 clone하지 말고 현재 브랜치와 로컬 변경을 확인합니다.

```bash
mkdir -p "$HOME/projects"
git clone --single-branch --branch codex/snucem-spray-addon \
  https://github.com/minjeasung/sketch-robot-runtime.git \
  "$HOME/projects/sketch-spray-addon"
cd "$HOME/projects/sketch-spray-addon"
git branch --show-current
git log -1 --oneline

python3 -B scripts/package_snucem_spray.py \
  --output "$PWD/dist/snucem-spray-humble-arm64-cuda12.6.tar.gz"
(
  cd dist
  sha256sum -c snucem-spray-humble-arm64-cuda12.6.tar.gz.sha256
)
```

예상 브랜치는 `codex/snucem-spray-addon`입니다. GitHub에서 내려받은 소스로
배포 파일을 만드는 경로이며, 생성한 체크섬은 파일 무결성 확인용입니다.

### Windows 접속용 설정 만들기

애드온 설치·상태 경로는 원본과 분리합니다. 다음 세 값을 실제 값으로 바꿉니다.
`SPRAY_CALIBRATION`은 현재 배치에서 측정한 외부보정 JSON이어야 하며,
원본의 `extrinsic` 실행에도 같은 파일을 사용합니다. 임의의 빈 파일이나 다른 장비의
예제 보정으로 대체하지 않습니다.

```bash
SPRAY_UPSTREAM="$HOME/SNUCEM_Robot_22.04"
SPRAY_CALIBRATION="$HOME/calibration/zed_extrinsic.local.json"
SPRAY_JETSON_IP="JETSON_IP_입력"

mkdir -p "$HOME/.local/share/snucem-sketch" "$HOME/.local/state/snucem-sketch"
chmod 700 "$HOME/.local/share/snucem-sketch" "$HOME/.local/state/snucem-sketch"

python3 - "$SPRAY_UPSTREAM" "$SPRAY_CALIBRATION" "$SPRAY_JETSON_IP" <<'PY'
import ipaddress
import json
from pathlib import Path
import secrets
import sys

upstream = Path(sys.argv[1]).expanduser().resolve()
calibration = Path(sys.argv[2]).expanduser().resolve()
host = str(ipaddress.IPv4Address(sys.argv[3]))
if not upstream.is_dir():
    raise SystemExit(f"원본 설치 경로 확인: {upstream}")
if not calibration.is_file():
    raise SystemExit(f"실측 보정 파일 확인: {calibration}")
state = Path.home() / ".local/state/snucem-sketch"
config = {
    "upstream_root": str(upstream),
    "install_root": str(Path.home() / ".local/share/snucem-sketch"),
    "state_root": str(state),
    "calibration_file": str(calibration),
    "profile": "preview",
    "model_id": "rb20_1900es",
    "ros_domain_id": 87,
    "rosbridge_url": f"ws://{host}:9090",
    "own_rosbridge": True,
    "image_topic": "/zed/zed_node/left/color/rect/image",
    "camera_info_topic": "/zed/zed_node/left/color/rect/camera_info",
    "points_topic": "/rb/spray/zed/points",
    "api_host": "0.0.0.0",
    "api_port": 8081,
    "api_token": secrets.token_urlsafe(32)
}
path = state / "config.json"
with path.open("x", encoding="utf-8") as stream:
    json.dump(config, stream, ensure_ascii=False, indent=2)
path.chmod(0o600)
print(f"설정 생성: {path}")
print("관리 토큰은 이 파일의 api_token에서 확인하세요.")
PY
```

기존 `config.json`이 있으면 이 명령은 덮어쓰지 않고 실패합니다. 이미 설치된 환경은
서비스를 종료한 뒤 기존 파일을 편집합니다. 예시는 RB20 기준입니다.
RB10은 `model_id`를 `rb10_1300e_u`로 바꾸고 아래 `stack`도 RB10으로 맞춥니다.
카메라 토픽·해상도·프레임도 실제 실행 중인 데이터와 맞아야 합니다.

이 예시는 `own_rosbridge: true`로, 「추가 기능 시작」이 rosbridge도 실행합니다.
`rosbridge_server` 패키지가 필요합니다. 이미 rosbridge가 실행 중이라면
`own_rosbridge: false`로 바꾸고 기존 서비스를 사용합니다. 동일 포트에 둘을 실행하지 않습니다.
`rosbridge_url`은 서버를 시작하는 옵션이 아니라 **브라우저가 연결할 주소**입니다.

설치·상태 디렉터리는 운영 계정만 쓸 수 있어야 합니다. 원본과 경로가 겹치거나
하위 심볼릭 링크/파일 하드 링크로 경계를 벗어나는 쓰기는 차단됩니다.

### 전용 가상환경에 설치

[각 터미널의 ROS 환경](#모든-jetson-터미널에서-맞출-ros-환경)을 먼저 적용한 터미널에서 실행합니다.
`python3-venv`, `python3-pip`, `python3-opencv` 및 위 설치 전 준비에 열거한
Humble 패키지가 필요합니다. rosbridge가 없다면 Jetson에서
`sudo apt install ros-humble-rosbridge-server`로 해당 패키지를 준비합니다.

```bash
cd "$HOME/projects/sketch-spray-addon"
python3 -B scripts/snucem_spray.py \
  --config "$HOME/.local/state/snucem-sketch/config.json" \
  install --bundle "$PWD/dist/snucem-spray-humble-arm64-cuda12.6.tar.gz" --setup-env
```

가상환경은 `install_root/envs/<version>`에 생성됩니다. 설치 출력의 `version`은
배포 파일 해시에서 만든 식별자이며 소프트웨어 버전 `0.2.0`을 직접 넣는 자리가 아닙니다.
ARM64 Python 3.10 wheel만 설치하며 현장에서 의존성을 소스 컴파일하지 않습니다.
의존성 설치 실패 시 이전 활성 버전이 유지되고, 실행 중 서비스의 업그레이드는 거절됩니다.
설치가 성공한 뒤 다음 절차로 진행합니다.

### 이미 ARM64 배포 압축 파일을 받은 경우

Git 다운로드·패키징 대신 이 경로를 사용할 수 있습니다. 파일을 받은 폴더에서
체크섬을 확인하고 새 bootstrap 폴더에 풉니다. 위 설정 생성과 ROS 환경 준비는 동일합니다.

```bash
sha256sum -c snucem-spray-humble-arm64-cuda12.6.tar.gz.sha256
mkdir -p "$HOME/snucem-sketch-bootstrap"
tar -xzf snucem-spray-humble-arm64-cuda12.6.tar.gz -C "$HOME/snucem-sketch-bootstrap"
python3 -B "$HOME/snucem-sketch-bootstrap/scripts/snucem_spray.py" \
  --config "$HOME/.local/state/snucem-sketch/config.json" \
  install --bundle "$PWD/snucem-spray-humble-arm64-cuda12.6.tar.gz" --setup-env
```

배포물 0.2.0은 ARM64/CUDA 12.6/ZED 5.3.1 메타데이터를 검증합니다.
이전 x86 기준 0.1.0 bootstrap 대신 새 압축 파일의 installer를 사용합니다.

## 3. JongHyun 기반 프로그램 실행 — 사용할 때마다 Jetson에서

각 터미널에서 앞의 공통 ROS 환경을 적용합니다. 이미 같은 프로그램이 실행 중이면
중복 실행하지 않습니다. 원본 문서의 모든 터미널을 그대로 켜는 것이 아니라,
애드온이 필요로 하는 **stack·카메라 bridge·외부보정**만 준비합니다.

### 터미널 A: 로봇 모델·MoveIt·로봇 TF

처음 연결과 화면을 확인할 때는 가상 하드웨어부터 사용합니다.

```bash
bash linux/bringup/run_rb20_spray.sh stack \
  robot_model:=rb20 real_hardware:=false tool_profile:=spray hitmap:=false
```

RB10이면 `robot_model:=rb10`으로 바꾸고 애드온 `model_id`도 함께 바꿉니다.
`hitmap:=false`는 JongHyun의 별도 히트맵 기능을 이 시작 절차에서 제외합니다.
가상 로봇을 사용해도 실제 카메라 스트리밍과 그 배치의 보정은 별도로 필요합니다.

**실물 로봇 연결은 위 가상 stack을 종료한 뒤 다음 명령으로 교체합니다.**
`ROBOT_IP_입력`을 실제 제어기 IP로 바꾸고 로봇·공구 모델을 실물에 맞춥니다.

```bash
bash linux/bringup/run_rb20_spray.sh stack \
  robot_model:=rb20 real_hardware:=true robot_ip:=ROBOT_IP_입력 \
  tool_profile:=spray hitmap:=false
```

이 명령은 실제 드라이버를 연결합니다. 애드온이 `preview`라는 이유로 외부 드라이버까지
가상 모드가 되는 것은 아닙니다. 반대로 애드온을 `motion_test`로 바꿔도 가상 stack이
실물로 전환되지는 않습니다. 현재 애드온은 실제 spray 공구의 충돌 mesh를 요구하므로
원본의 `tool_profile:=ft_preview`를 동일한 대체 옵션으로 사용하지 않습니다.

### 터미널 B: 카메라 데이터 → ROS

먼저 기존 Jetson용 Outpost 환경에서 해당 ZED의 연결과 스트리밍을 시작합니다.
`zed` 명령은 카메라 자체를 켜는 대신 이미 스트리밍 중인 Outpost 데이터를 읽습니다.
다음 장치 ID와 시리얼은 현재 Outpost 화면/상태에서 확인한 값으로 바꿉니다.

```bash
bash linux/bringup/run_rb20_spray.sh zed \
  --outpost-http http://127.0.0.1:8100 \
  --hw-id OUTPOST_HW_ID_입력 --camera-id ZED_SERIAL_입력
```

스케치 애드온은 왼쪽 영상·CameraInfo·점군을 사용합니다. 원본의 오른쪽 투영까지
사용하는 환경은 원본 안내에 따라 해당 카메라의 실측 `--stereo-calibration`을 추가합니다.
기존 bridge가 이미 같은 토픽을 발행한다면 새로 실행하지 않습니다.
카메라 컨테이너·SDK 실행 명령은 실제 Jetson에 설치된 방식에 따라 달라집니다.

### 터미널 C: 카메라 외부보정 TF

애드온 설정의 보정 파일 경로를 읽어서 같은 파일을 원본 외부보정 실행기에 전달합니다.

```bash
SPRAY_CALIBRATION="$(python3 - <<'PY'
import json
from pathlib import Path
path = Path.home() / ".local/state/snucem-sketch/config.json"
print(json.loads(path.read_text())["calibration_file"])
PY
)"
bash linux/bringup/run_rb20_spray.sh extrinsic --calibration "$SPRAY_CALIBRATION"
```

이 과정은 `link0 ← zed_left_camera_frame_optical`의 실측 변환을 발행합니다.
이미 같은 카메라 TF를 발행 중이면 중복 발행하지 않습니다.
애드온의 `calibration_file` 지정만으로 TF가 적용되지는 않습니다.

### 실행하지 않을 원본 항목

애드온을 사용할 때 JongHyun의 `run_rb20_spray.sh node`, `executor`,
`keyboard`, `haply`를 함께 시작하지 않습니다. 이미 실행 중인 원본
`rb20_spray_shared_autonomy`, `rb20_spray_executor` 및 경쟁 실행기는 먼저 종료합니다.
애드온이 인식 wrapper와 프로필에 따른 실행기를 관리합니다.
기존 원격조작 기능을 위한 전체 실행 스크립트를 켜서 이 항목들을 함께 시작하지 않습니다.

## 4. 애드온 서버 실행 — Jetson 터미널 D

여기서도 공통 ROS 환경을 적용합니다. `preview`에서도 준비 시 실행 중인
`/robot_description`과 MoveIt 모델을 읽으므로 터미널 A가 필요합니다.
`preview`는 로봇 실행기를 만들지 않는 모드이며, 기반 모델 서버까지 불필요한 모드는 아닙니다.

설치가 성공하면 `active-version.json`에 기록된 실제 버전을 읽어 실행합니다.

```bash
ADDON_CONFIG="$HOME/.local/state/snucem-sketch/config.json"
ADDON_ROOT="$(python3 - <<'PY'
import json
from pathlib import Path
path = Path.home() / ".local/state/snucem-sketch/active-version.json"
print(json.loads(path.read_text())["root"])
PY
)"
ADDON_PYTHON="$(python3 - "$ADDON_CONFIG" <<'PY'
import json
from pathlib import Path
import sys
config = json.loads(Path(sys.argv[1]).read_text())
active = json.loads((Path(config["state_root"]) / "active-version.json").read_text())
print(Path(config["install_root"]) / "envs" / active["version"] / "bin/python")
PY
)"

"$ADDON_PYTHON" -B "$ADDON_ROOT/scripts/snucem_spray.py" --config "$ADDON_CONFIG" doctor
```

`"ok": true`와 종료 코드 0을 확인합니다. 실패하면 부족한 의존성·원본 호환성·설정을
해결한 뒤 진행합니다. `doctor`만으로 실행 중 모델이나 실물 전체가 검증되지는 않습니다.

```bash
"$ADDON_PYTHON" -B "$ADDON_ROOT/scripts/snucem_spray.py" --config "$ADDON_CONFIG" serve
```

서버 터미널은 켜둡니다. 실제 자식 프로세스는 다음 관리 화면의 「추가 기능 시작」에서
실행됩니다. `own_rosbridge: true`이면 모델 준비 후 rosbridge도 시작합니다.

## 5. Windows 브라우저에서 관리·스케치 화면 열기

| 연결 | 주소 | 용도 |
|---|---|---|
| 관리 홈페이지 | `http://JETSON_IP:8081/` | 토큰 입력, 애드온 시작·종료, 상태 확인 |
| 스케치 홈페이지 | `http://JETSON_IP:8081/sketch/` | 평면·작업영역·경로 지정 |
| ROS 연결 | `ws://JETSON_IP:9090` | 브라우저의 ROS 통신; 주소창에 열 웹페이지가 아님 |

1. Windows에서 관리 홈페이지에 접속합니다.
2. Jetson `config.json`의 `api_token`을 관리 토큰 칸에 입력합니다.
3. 「추가 기능 시작」을 누르고 자식 프로세스 실행 상태를 확인합니다.
4. 「스케치 화면 열기」를 누르면 새 탭이 열립니다.
5. 평면 선택 → 작업영역 지정 → 경로 작성/생성 → 검증 순서로 진행합니다.

Jetson에서는 브라우저를 열 필요가 없습니다. `serve` 하나가 두 화면을 제공합니다.
Windows에서 Jetson의 TCP 8081·9090에 접근할 수 있는 운영망을 사용합니다.
Windows에서 `127.0.0.1`은 Windows 자신이므로 `rosbridge_url`에도 실제 Jetson IP를
넣습니다. `0.0.0.0`은 서버의 수신 주소이며 브라우저 접속 주소로 쓰지 않습니다.

관리 HTTP API를 외부에 바인딩하려면 24자 이상의 `api_token`이 필요합니다.
관리 토큰은 rosbridge 인증을 대신하지 않으므로 ROS/rosbridge는 제한된 운영망에서
관리하고 이 포트를 인터넷에 공개하지 않습니다.
Jetson 내부에서만 사용할 때는 `api_host: "127.0.0.1"`,
`rosbridge_url: "ws://127.0.0.1:9090"`으로 바꿉니다.

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

작업을 종료할 때는 먼저 관리 화면에서 「추가 기능 종료」를 누르거나 애드온 서버를
Ctrl+C로 종료합니다. 그 뒤 원본 stack·ZED bridge·외부보정·카메라는 각각의 실행
터미널과 기존 카메라 도구에서 종료합니다. Windows 브라우저를 닫는 것만으로
Jetson 서버나 실행 중인 작업이 종료되지는 않습니다.

관리 화면의 종료 또는 서버 Ctrl+C는 애드온이 만든 자식 프로세스만 종료합니다.
실행기는 건 OFF와 보유 액션 취소를 요청하고 결과를 제한 시간 동안 처리한 뒤
자신의 scene object만 제거합니다. 네트워크 단절/강제 종료에서는 취소 응답이나
scene 정리를 보장할 수 없으므로 실제 로봇 정지 여부는 외부 스택에서 확인합니다.
하드웨어 건은 lease 만료 시 독립적으로 OFF가 되어야 합니다.

로그: `state_root/logs/`. 실시간 모델: `state_root/model-status.json`.

| 증상 | 먼저 확인할 것 |
|---|---|
| Windows에서 관리 화면이 안 열림 | Jetson의 `serve` 실행 여부, 실제 IP, `api_host: 0.0.0.0`, TCP 8081 접근 |
| 관리 화면은 열리지만 스케치가 ROS 연결 대기 | `rosbridge_url`이 Jetson IP인지, 추가 기능 준비 성공 여부, rosbridge 실행 및 TCP 9090 접근 |
| 관리 버튼이 401 오류 | 같은 Jetson `config.json`의 `api_token`을 입력했는지 |
| 준비 중 `live model unavailable` | 원본 stack·MoveIt 실행, `model_id` 일치, domain 87과 DDS 환경, `logs/model.log` |
| `doctor`가 원본 호환성 오류 | 설정한 원본 경로와 지원 인터페이스 리비전 확인; 강제 초기화/검사 우회 금지 |
| 카메라 영상 또는 평면이 없음 | Outpost 스트리밍, 실제 장치 ID/시리얼, ZED bridge·영상/점군 토픽·외부보정 TF·관측 상태 |
| rosbridge 포트 충돌 | 기존 서비스가 있으면 `own_rosbridge: false`, 기존 서비스와 애드온 소유 방식 중 하나 선택 |
| 원본 스크립트가 `.venv` 또는 ROS 패키지를 못 찾음 | 원본의 설치·빌드 완료 여부와 `ros_env.sh` 적용 여부 |

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
