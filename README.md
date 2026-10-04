# Sketch Robot

Spray requires a confirmed, calibrated EOAT mesh profile. Set the same
`spray_eoat_profile` path for generation and execution; the empty default blocks
Spray while Paint is unchanged. See [EOAT profile setup](docs/SPRAY_EOAT_PROFILE.md).

Michelo Outpost의 ZED/D405 데이터로 작업 평면을 측정하고 웹 스케치로 RB10/RB20의 도장·내화뿜칠 경로를 제어합니다.

도장(Paint)은 기존 ZED+D405 구성을 유지합니다. 뿜칠(Spray)은 시작 설정
`process_mode=spray`로 **ZED만 사용**하며 D405 연결·보정·측정 이동을 요구하지 않습니다.
기본 공정은 Paint이고 `spray_motion_test`는 Spray로 시작합니다. 뿜칠 기본값은
노즐 이격 0.500 m, 폭 0.350 m, 겹침 30%, 속도 0.020 m/s입니다.
[9단계 작업 흐름과 현장 검증 항목](docs/MULTI_PLANE_SPRAY.md)을 먼저 확인하세요.

**로봇 PC에 설치하고, 같은 네트워크의 원격 PC에서 브라우저로 접속합니다.**
원격 PC에는 ROS나 Python 설치가 필요 없습니다.

**설치가 끝났다면 [리눅스 / 윈도우 실행 순서](RUN.md)부터 확인하세요.**

## 로봇 PC 설치

Ubuntu 24.04 x86_64 / ROS 2 Jazzy를 준비합니다. 기본 카메라 입력은 Michelo Outpost이므로
Sketch Runtime 자체에는 ZED SDK/CUDA/`zed_wrapper`가 필요하지 않습니다. ZED 2i를 직접 여는
Outpost 환경에만 ZED SDK가 필요하며, RViz/MoveIt 형상은 SDK와 독립적인 공식
`zed_description`의 ZED 2i mesh를 사용합니다.
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

## 로봇 없이 ZED로 작업 준비

고정 설치된 ZED만으로 대상 평면 선택 → 사각형 작업영역 확정 → 자동 도포 경로
생성·미리보기를 할 수 있습니다. Outpost에서 ZED RGB·깊이 스트리밍을 시작한 뒤,
관리 화면의 실행 모드를 **ZED만 · 경로 생성 (로봇 없음)**으로 바꾸고 설정을 적용하세요.
ZED 장치 ID·시리얼을 선택한 다음 **전체 시작 → 스케치 화면 열기**로 진행합니다.

이 모드(`zed_preview`)는 `perception`과 `rosbridge`만 실행합니다. 로봇 연결,
MoveIt, 컨트롤러, 실행기, 힘센서, D405는 시작하지 않으며 로봇 실행 버튼은 비활성화됩니다.
기존 ZED 외부 보정값으로 표면·경로 좌표를 계산합니다. 카메라를 옮겼다면 보정값을 갱신해야 합니다.
**경로 생성 완료는 로봇 도달성·충돌 검증 완료를 뜻하지 않습니다.** 실제 이동은 전체 종료 후
로봇 모드에서 현재 대상·영역을 다시 선택하고 경로를 검증하여 진행합니다.

매번 이 모드로 시작하려면 PC별 `config/sketch_runtime.env`에 아래 값을 설정합니다.

```bash
SKETCH_PROFILE=zed_preview
SKETCH_PROCESS_MODE=spray
```

코드 갱신 후에는 `sketch_control`을 다시 빌드하고 웹 서버를 재시작해야 새 실행 모드가 적용됩니다.
