# Sketch Robot Supervisor API

여러 터미널에서 실행하던 구성을 FastAPI 프로세스 관리 서버로 통합했습니다.
참고한 구조는 [SNUCEM_Robot Supervisor](https://github.com/JongHyunSeo11/SNUCEM_Robot/blob/main/linux/gateway/rb10_supervisor/api.py)입니다.
참고 저장소는 읽기만 했으며 이 구현은 `sketch_robot_ws`에 추가했습니다.

## 시작

현재 PC에는 API 전용 `.venv-api` 설치와 ROS 패키지 빌드를 완료했습니다.
터미널 하나에서 실행하세요.

```bash
/home/Minjea/sketch_robot_ws/scripts/run_system_api.sh
```

- 관리 화면: http://127.0.0.1:8080/
- API 문서/직접 호출: http://127.0.0.1:8080/docs
- OpenAPI: http://127.0.0.1:8080/openapi.json
- 스케치 화면: http://127.0.0.1:8080/sketch/

서버 기동만으로 로봇이나 카메라가 시작되지는 않습니다. 관리 화면의 **전체 시작**으로
순차 기동하고, **스케치 화면 열기**에서 기존처럼 평면 선택·측정·스케치·도장/뿜칠 작업을 합니다.
작업 후 **전체 종료**를 누릅니다. 각 프로세스의 시작·종료·재시작·로그도 제공됩니다.
ROS 연결이 끊긴 뒤 시스템을 재시작했다면 기존 스케치 탭을 새로고침하세요.

기존 수동 실행 로봇/MoveIt/실행기/rosbridge가 있다면 해당 터미널에서 먼저 종료합니다.
Supervisor는 외부에서 시작한 프로세스를 가져오거나 종료하지 않습니다.
ZED/D405 드라이버를 그대로 사용할 때는 해당 **카메라 시작** 체크를 해제하세요.
`ROS_DOMAIN_ID`는 호출한 환경을 따르며 화면과 `/status`에 표시됩니다.

## 참고 저장소와의 대응

FastAPI + Uvicorn, 등록된 프로세스 ID, HTTP Bearer 인증, 의존성 검사,
prepare 실패 시 이번 요청에서 시작한 항목만 정리, 역순 shutdown을 같은 방식으로 구성했습니다.
원래 ROS 노드와 기존 안전 인터록은 유지합니다.

| 현재 프로세스 ID | 기존 터미널 역할 | 선행 프로세스 |
|---|---|---|
| `robot_control` (화면: robot control) | RB10·MoveIt·controller·RViz | 없음 |
| `perception` | ZED·D405·TF·평면 인식·스케치 변환 | robot_control |
| `force_pipeline` | 목표 wrench·F/T monitor | robot_control |
| `executor` | 경로 실행기·flight recorder | robot_control, perception, force_pipeline |
| `rosbridge` | 기존 웹 UI의 ROS WebSocket, 9090 | 없음 |
| API 서버 자체 | 웹 화면과 실행 관리, 8080 | 위 스크립트로 한 번 기동 |

기존 `rb10_painting_system.launch.py`에 그룹별 실행 스위치만 추가했습니다.
기본값은 기존 통합 실행과 동일합니다. Supervisor는 각 프로세스에 해당 그룹만 켭니다.
카메라는 기존 스케치 프로젝트의 ROS 드라이버를 사용하며 Outpost로 교체하지 않았습니다.
참고 저장소의 Haply/공유제어용 7개 ID나 `start-teleop`은 이 프로젝트에 해당하지 않습니다.
따라서 기존 Windows Michelo 7개 버튼 화면과의 완전한 호환을 뜻하지 않습니다.
현재 제공되는 관리 웹 화면 또는 아래 HTTP API를 사용합니다.

## 실행 모드와 설정

| profile | 로봇 | 경로 실행 | 힘 제어 허용 |
|---|---|---|---|
| `dry_run` (기본) | 실제 연결 | 모의 실행 | 꺼짐 |
| `work` | 실제 연결 | 기존 실행 인터록 통과 후 허용 | 기존 도장 제어 |
| `fake` | 가상 하드웨어 | 모의 실행 | 꺼짐 |

`work`는 기존 통합 launch의 `real_painting_enabled=true`, `dry_run=false`,
`painting_force_enabled=true`를 전달합니다. 서버 시작이나 prepare는 작업 경로를 제출하지 않습니다.
도장/뿜칠 선택은 스케치 화면에서 하며 뿜칠의 F/T 미사용, 50cm 이격,
건 ON/OFF 규칙은 기존 executor가 처리합니다. `fake`의 카메라/RViz 시작 기본값은 false입니다.

설정은 전체 종료 상태에서 관리 화면 또는 `POST /configuration`으로 바꿉니다.
서버 재시작 후 유지할 PC별 설정은 다음 파일을 사용합니다.

```bash
cp -n /home/Minjea/sketch_robot_ws/config/sketch_runtime.env.example \
      /home/Minjea/sketch_robot_ws/config/sketch_runtime.env
```

`config/sketch_runtime.env`는 Git에서 제외됩니다. 로봇 IP, 실행 모드,
ROS domain, 카메라/RViz 사용 여부, 서버 주소/포트를 설정할 수 있습니다.
API 요청으로 임의 shell 명령, 실행 파일, ROS 인자를 전달할 수 없습니다.

## HTTP API

| 요청 | 동작 |
|---|---|
| `GET /healthz` | API 생존 확인 |
| `GET /status` | 프로세스 목록, 설정, 최신 ROS 준비 상태 |
| `GET /events` | 최근 500개 생명주기 이벤트 |
| `GET /configuration` | 실행 설정 |
| `POST /configuration` | 종료 상태에서 설정 변경 |
| `GET /processes/{name}` | 개별 상태 |
| `GET /processes/{name}/logs?lines=100` | 최근 로그, 최대 500줄 |
| `POST /processes/{name}/start` | 의존 프로세스 실행 확인 후 기동 |
| `POST /processes/{name}/stop?cascade=false` | 종료; 의존 항목 실행 중이면 409 |
| `POST /processes/{name}/restart?cascade=false` | 해당 프로세스 재시작 |
| `POST /prepare-system` | 순차 전체 시작; 실패 시 새로 시작한 항목 정리 |
| `POST /shutdown-system` | 의존성 역순 전체 종료 |

`cascade=true`는 의존 항목도 먼저 종료합니다. 재시작 시 의존 항목을 자동으로 재기동하지 않습니다.
GET 상태 조회는 프로세스를 시작하지 않습니다. 실행 중인 항목을 다시 start하면 중복 기동하지 않습니다.
알 수 없는 ID는 404, 인증 실패는 401, 의존성·중복 실행 충돌은 409입니다.

```bash
curl http://127.0.0.1:8080/status

# 가상 하드웨어 모드 설정 (실로봇 연결 없음)
curl -X POST http://127.0.0.1:8080/configuration \
  -H 'Content-Type: application/json' -d '{"profile":"fake"}'

curl -X POST http://127.0.0.1:8080/prepare-system
curl http://127.0.0.1:8080/processes/executor/logs?lines=100
curl -X POST http://127.0.0.1:8080/shutdown-system
```

상태는 `STOPPED / STARTING / RUNNING / STOPPING / EXITED / FAILED`입니다.
`RUNNING`과 `system_prepared`는 OS 프로세스 생존 상태입니다. launch 내부의 모든 ROS 노드,
센서 수신, controller active를 보장하지 않습니다. ROS 그래프와 기존 실행기의
`ros.readiness`를 함께 확인합니다. 2초 이상 갱신되지 않은 ROS 준비 상태는 null로 표시합니다.
선행 프로세스가 비정상 종료되면 의존 프로세스도 종료하며 자동 작업 재개는 하지 않습니다.

## 다른 PC에서 실행 관리

`config/sketch_runtime.env`에 다음을 설정합니다.

```bash
SKETCH_SUPERVISOR_HOST=0.0.0.0
SKETCH_SUPERVISOR_PORT=8080
SKETCH_SUPERVISOR_API_TOKEN=직접_생성한_비밀_토큰
```

토큰 생성 예: `python3 -c 'import secrets; print(secrets.token_urlsafe(32))'`.
설정 후 서버를 재시작하고 다른 PC에서 `http://로봇PC주소:8080/`에 접속합니다.
관리 화면의 API 토큰 입력란에 입력 후 **연결 확인**을 누릅니다. 토큰은 브라우저 메모리에만 둡니다.
직접 호출 시 `Authorization: Bearer <token>` 헤더를 사용합니다. `/docs`의 Authorize도 지원합니다.

루프백 밖에 바인딩할 때 토큰이 없으면 서버가 시작을 거부합니다. 기본은 127.0.0.1입니다.
이 토큰은 Supervisor API 인증이며 기존 rosbridge 9090을 인증하는 기능은 아닙니다.
다른 PC에서 스케치까지 사용하려면 기존 rosbridge 포트도 접근 가능해야 합니다.
현재 HTTP/WS 구성은 신뢰할 수 있는 로봇 LAN용이며 인터넷 공개용 TLS 구성은 포함하지 않습니다.

## 설치·개발·종료

다른 PC에서 최초 설치는 [두 PC 설치 안내](PORTABLE_INSTALL.md)의 선행 환경을 준비한 뒤 수행합니다:

```bash
cd /home/Minjea/sketch_robot_ws
bash scripts/install_runtime.sh --install-deps
bash scripts/check_runtime.sh
bash scripts/run_system_api.sh
```

Jazzy용 Python 3.12와 시스템 ROS 패키지를 유지하고 API 의존성만 `.venv-api`에 설치합니다.
검증한 버전은 `config/system_api.requirements.txt`에 고정했습니다.
API 코드는 workspace 소스에서 읽습니다. ROS 노드/launch 변경은 colcon 빌드가 필요합니다.

로그는 `logs/system_api/{process}.log`, ROS 로그는 기존 `~/.ros/log`입니다.
프로세스 재시작 시 기존 로그가 10MB를 넘으면 `.previous.log` 한 개로 교체합니다.
한 번의 장기 실행 중 파일 크기를 강제로 제한하지는 않습니다.

종료 시 executor에 `/motion_abort=true`, force disable을 먼저 요청하고 소유한 프로세스 그룹에
SIGINT → 필요 시 SIGTERM → SIGKILL 순서로 보냅니다. Ctrl+C도 전체 정리를 수행합니다.
현재 executor의 `/motion_abort`는 하드웨어 motion-inhibit를 래치할 수 있습니다.
executor를 개별 종료·재시작한 뒤에는 기존 제어기의 요구에 따라 전체 종료 후 전체 시작이 필요합니다.
Supervisor는 이 래치를 자동 해제하지 않습니다. 종료 직후 ROS 그래프에서 노드가 사라지기까지
DDS 검색 지연이 있을 수 있으며, 중복 시작 검사는 해당 노드가 사라진 뒤 통과합니다.
외부에서 실행한 프로세스는 종료하지 않습니다. 이 API는 하드웨어 비상정지를 대체하지 않으며,
SIGKILL/전원 차단 시 서버의 정상 정리 코드는 실행될 수 없습니다.
부팅 자동 실행 서비스는 설치하지 않았습니다.

검증 명령:

```bash
PYTHONPATH=src/sketch_control .venv-api/bin/python -m pytest -q \
  src/sketch_control/test/test_system_api.py
```

테스트는 일회용 Python 프로세스를 사용합니다. 실제 로봇 동작·도장·뿜칠은 자동 검증에서 수행하지 않습니다.

### 이번 변경 검증 결과 (2026-09-21)

- Supervisor 테스트 26개 + 기존 painting config/launch 테스트 11개: **37 passed**.
- `sketch_control` colcon 빌드와 통합 launch 인자 조회 통과.
- `ROS_DOMAIN_ID=218`, fake hardware, 카메라/RViz 꺼짐으로 다섯 프로세스
  prepare → 생존·ROS 노드 확인 → shutdown 통과. DDS 검색 지연 후 API 노드만 남음을 확인.
- 브라우저에서 프로세스 목록, fake 설정, 전체 시작 요청, 기존 스케치 스크립트 검증 통과.
- 실로봇 이동·접촉 도장·실제 뿜칠·다른 PC 원격 접속은 수행하지 않았습니다.
