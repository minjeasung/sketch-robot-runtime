# 두 PC 설치와 다운로드

## 구성

```text
원격 스케치 PC (Windows/macOS/Linux, 브라우저만 필요)
    ├─ HTTP 8080: 실행 관리·스케치 화면
    └─ WebSocket 9090: 스케치 명령·ROS 상태·카메라 영상
                 ↓ 같은 내부 네트워크
로봇 제어 PC (Ubuntu 24.04 x86_64)
    API Supervisor → ROS Jazzy → RB10 / ZED / D405 / F/T
```

전체 프로그램은 **로봇 PC에만** 설치합니다. 원격 PC에는 ROS, Python, 카메라 SDK,
GitHub 코드 복제가 필요하지 않습니다. 두 PC가 같은 네트워크인 구성을 기준으로 합니다.
한 운영자가 작업하는 구조이며 여러 브라우저의 동시 작업 소유권 중재는 구현하지 않았습니다.

## 1. 소프트웨어 받기

배포 저장소는 **minjeasung/sketch-robot-runtime** 비공개 저장소입니다.
로봇 PC에서 저장소 접근 권한이 있는 GitHub 계정으로 인증한 뒤 받습니다.
GitHub CLI를 사용하는 경우 `gh auth login` 및 `gh auth setup-git`으로 인증을 설정할 수 있습니다.

```bash
git clone https://github.com/minjeasung/sketch-robot-runtime.git ~/sketch_robot_ws
cd ~/sketch_robot_ws
```

기존 `~/sketch_robot_ws`가 있다면 다른 빈 경로를 선택합니다.
원격 스케치 PC에서는 이 저장소를 받을 필요 없이 로봇 PC의 웹 화면에 접속합니다.

소스 압축파일로 전달할 수도 있습니다.
`build/install/log`, 가상환경, `.git`, API 토큰과 PC별 `.env`, 보정 JSON은 제외합니다.
명령 실행 시점의 파일 내용을 포함하므로 미커밋 변경도 포함됩니다.

```bash
python3 scripts/export_runtime.py
```

`dist/sketch-robot-source-날짜.tar.gz`와 `.sha256`을 USB/내부 파일 공유 등으로 전달합니다.
참고용 `JongHyunSeo11/SNUCEM_Robot`은 이 배포 저장소와 별개입니다.
향후 참고 저장소 반영은 별도 브랜치에서 변경·대상 PC 테스트를 거친 뒤 `main`으로 병합합니다.
현재 배포 작업은 참고 저장소에 브랜치를 만들거나 변경사항을 올리지 않습니다.

압축파일을 받은 경우에는 빈 설치 위치에 해제합니다. 기존 작업 폴더 위에 덮어쓰지 않습니다.

```bash
cd ~/Downloads
sha256sum -c sketch-robot-source-날짜.tar.gz.sha256
mkdir -p ~/robot-app
# 압축 안의 최상위 폴더는 sketch_robot_ws입니다.
tar -xzf sketch-robot-source-날짜.tar.gz -C ~/robot-app
cd ~/robot-app/sketch_robot_ws
```

## 2. 로봇 PC 선행 설치

대상 환경은 Ubuntu 24.04 x86_64 / ROS 2 Jazzy / 시스템 Python 3.12입니다.

- [ROS 2 Jazzy 설치](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- 실제 ZED 사용: NVIDIA 드라이버, ZED SDK 5.3과 해당 SDK용 CUDA 개발 환경.
  [ZED 공식 ROS 2 설치 안내](https://www.stereolabs.com/docs/integrations/ros-2)

GPU 드라이버·SDK는 PC의 GPU와 CUDA 조합에 맞춰 먼저 설치합니다.
아래 설치 스크립트는 OS나 GPU 드라이버를 교체하지 않습니다.
원격 브라우저 PC에는 이 단계가 필요 없습니다.

## 3. 설치·빌드·점검

```bash
bash scripts/install_runtime.sh --install-deps
```

이 명령은 apt/rosdep 의존성 설치, API 가상환경 설치, 고정 버전 소스 다운로드,
로봇 SDK와 ROS 패키지 빌드, 기본 설정 생성, 설치 점검을 수행합니다.
apt 설치 단계에서 sudo 권한이 필요합니다. 로봇/카메라를 시작하지 않습니다.
기존 `config/sketch_runtime.env`는 덮어쓰지 않습니다.

시스템 의존성이 이미 준비된 경우 `--install-deps`를 생략합니다.
ZED SDK 없이 fake hardware로 설치 구조부터 확인할 때:

```bash
bash scripts/install_runtime.sh --without-zed --install-deps
```

이 구성은 실제 ZED 운용용이 아닙니다. 이후 SDK 준비 후 옵션 없이 다시 설치·빌드합니다.
이전 fake 설정은 보존되므로 실제 사용할 때 설정을 직접 바꿉니다.
빌드 메모리가 부족하면 `SKETCH_BUILD_JOBS=1`을 앞에 붙입니다.

```bash
bash scripts/check_runtime.sh          # 파일·모듈·패키지 확인
bash scripts/check_runtime.sh --real   # 카메라 패키지와 보정 파일도 확인
```

설치 산출물은 `.runtime/sdk`, `.runtime/ros2`, `.venv-api` 안에 생성됩니다.
새 설치는 다른 `~/rb10_ws`, `~/ros2_ws`에 의존하지 않습니다.
설치 후 폴더를 이동하면 해당 위치에서 다시 빌드합니다. 컴파일된 install/venv를 복사하지 않습니다.
현재 운영 PC의 기존 환경은 새 portable 설치가 성공하기 전까지 그대로 사용할 수 있습니다.

## 4. 같은 로봇 셀의 보정값 이전

같은 로봇·같은 카메라 장착 상태에서 제어 PC만 바꾸는 경우 기존 보정값을 따로 전달합니다.
기존 PC에서:

```bash
python3 scripts/export_runtime.py --calibration-only
```

생성된 calibration 압축의 JSON 네 개는 새 `sketch_robot_ws` 루트에 놓습니다.
API 토큰과 IP 환경파일은 이 압축에 포함되지 않습니다.
보정값은 해당 로봇/카메라 장착에 종속되므로 장착 상태나 장비가 바뀌면 재측정해야 합니다.
설치 점검은 보정 JSON의 존재/형식만 검사하며 보정 정확도를 보장하지 않습니다.

## 5. 같은 네트워크의 원격 PC에 접속 허용

로봇 PC의 `config/sketch_runtime.env`:

```bash
SKETCH_SUPERVISOR_HOST=0.0.0.0
SKETCH_SUPERVISOR_PORT=8080
SKETCH_SUPERVISOR_API_TOKEN=직접_생성한_비밀_토큰
SKETCH_ROBOT_IP=10.0.2.7
SKETCH_PROFILE=dry_run
```

토큰 생성:

```bash
python3 -c 'import secrets; print(secrets.token_urlsafe(32))'
```

로봇 PC에서 실행:

```bash
bash scripts/run_system_api.sh
```

로봇 PC의 `hostname -I`로 원격 PC와 같은 LAN에 속한 IP를 확인합니다.
여러 IP가 표시되면 로봇 컨트롤러 전용 NIC의 `10.0.2.x`가 아니라
원격 PC와 통신할 수 있는 LAN 주소를 선택합니다.
예를 들어 로봇 PC의 LAN 주소가 `192.168.0.20`이면 원격 PC 브라우저에서:

- 관리: `http://192.168.0.20:8080/`
- 스케치: `http://192.168.0.20:8080/sketch/`
- API 문서: `http://192.168.0.20:8080/docs`

관리 화면에 토큰 입력 → 연결 확인 → 전체 시작 → 스케치 화면 열기 순서입니다.
브라우저 즐겨찾기로 등록하면 다음부터 설치·다운로드 없이 접속할 수 있습니다.
로봇 PC의 LAN 주소가 바뀌지 않도록 공유기의 DHCP 예약을 사용하는 편이 편리합니다.

방화벽을 사용한다면 **신뢰하는 원격 PC에서 로봇 PC의 TCP 8080과 9090에 접근**하도록 설정합니다.
자동으로 방화벽을 변경하지 않습니다. rosbridge 9090은 기존 ROS 인터페이스이며 API Bearer 토큰으로
보호되지 않으므로 외부 인터넷 포트포워딩 대상으로 공개하지 않습니다.
스케치의 로봇 실행은 기존 실행기와 인터록을 그대로 사용합니다.

브라우저를 닫는 것은 로봇 정지 명령이 아닙니다. 작업 종료/전체 종료는 UI에서 명시적으로 수행합니다.
네트워크 단절 시 자동 정지와 다중 사용자 제어권 관리는 이번 설치 기능 범위에 포함하지 않습니다.

## 6. 업데이트와 검증 범위

새 버전의 소스를 받아 동일하게 설치/빌드합니다. 장비별 보정값과 `config/sketch_runtime.env`는
별도로 보관합니다. `config/runtime_sources.json`은 현재 검증 환경의 rbpodo SDK와 ZED wrapper
커밋을 고정하고, API Python 패키지는 `config/system_api.requirements.txt`에 고정합니다.
Ubuntu apt 패키지 전체를 고정한 OS 이미지나 완전한 오프라인 설치 묶음은 아닙니다.

설치 스크립트와 배포 압축은 현재 PC에서 검증합니다. 새 OS 설치, GPU SDK 설치,
실제 다른 PC의 네트워크/장비 연결은 대상 PC에서 추가 확인해야 합니다.

화면 없는 로봇 PC에서는 `SKETCH_LAUNCH_RVIZ=false`를 PC별 설정에 추가하면 됩니다.

### 이번 검증 결과 (2026-09-21)

- API·기존 launch 설정·배포 파일 검사: 41개 테스트 통과.
- 소스 압축을 `/tmp`의 별도 경로에 풀고 `--without-zed` 설치 실행:
  SDK 다운로드/빌드, API 설치, ROS 11개 패키지 빌드, 독립 overlay 점검 통과.
- 해당 설치를 ROS domain 219의 fake hardware로 prepare/shutdown 검증 통과.
- 브라우저에서 원격 호스트로 스케치와 Force Setup의 WebSocket 주소를 확인.
- 새 OS의 apt 설치, 새 GPU/ZED SDK 설치, 실제 두 PC 네트워크 연결과 실로봇 작업은 미실행.
