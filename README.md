# Sketch Robot

Michelo Outpost의 ZED/D405 데이터로 작업 평면을 측정하고 웹 스케치로 RB10/RB20의 도장·내화뿜칠 경로를 제어합니다.

**로봇 PC에 설치하고, 같은 네트워크의 원격 PC에서 브라우저로 접속합니다.**
원격 PC에는 ROS나 Python 설치가 필요 없습니다.

## 로봇 PC 설치

Ubuntu 24.04 x86_64 / ROS 2 Jazzy 및 실제 ZED 운용을 위한 GPU·ZED SDK 환경을 먼저 준비합니다.
이 저장소는 비공개이므로 먼저 접근 권한이 있는 GitHub 계정으로 Git 인증을 설정합니다.
기존 작업 폴더가 있다면 복제할 경로를 바꿉니다.

```bash
git clone https://github.com/minjeasung/sketch-robot-runtime.git ~/sketch_robot_ws
cd ~/sketch_robot_ws
bash scripts/install_runtime.sh --install-deps --without-zed
bash scripts/check_runtime.sh
bash scripts/run_michelo_sketch.sh
```

로컬 관리 화면: http://127.0.0.1:8081/ · API 문서: http://127.0.0.1:8081/docs
Michelo 연결 화면: http://127.0.0.1:8101/console/ → **스케치 작업 ↗** 버튼으로 새 탭을 엽니다.
기존 Outpost `8100`은 그대로 사용하며 설치 파일을 수정하지 않습니다.
카메라 ID·시리얼, 보정값과 IPC 접근 계정 설정은 아래 연동 안내를 따릅니다.

원격 접속은 `config/sketch_runtime.env`에 서버 주소와 API 토큰을 설정한 뒤
브라우저에서 `http://로봇PC의LAN주소:8081/`을 엽니다.

- [다운로드·두 PC 설치·접속 안내](docs/PORTABLE_INSTALL.md)
- [Michelo 연결·Outpost 카메라 연동](docs/MICHELO_INTEGRATION.md)
- [RB10/RB20 모델 설정](docs/ROBOT_MODELS.md)
- [EOAT 뿜칠 이동 검증](docs/SPRAY_MOTION_TEST.md)
- [Supervisor API 사용법](docs/SYSTEM_API.md)
- [다중 평면·내화뿜칠](docs/MULTI_PLANE_SPRAY.md)

서버 설치/실행 자체는 로봇 작업을 시작하지 않습니다. 전체 시작 후 스케치 화면에서 작업합니다.
