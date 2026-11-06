# RB20 롤러 도장 · Isaac Sim 실행 안내

`isaac_rb20_paint`는 번들 `rb20_1900es`와 롤러 EOAT를 Isaac Sim에서 실행한다.
기존 스케치 화면, ZED/D405 인지, MoveIt, JTC → admittance, force guard,
접촉 전 tare 및 도장 실행기를 사용한다. 관절 상태와 6축 F/T는 물리 측정값,
카메라 입력은 렌더링 RGB/depth다. 로봇 IP, 실물 ZED/RealSense 드라이버,
Michelo/Outpost 연결은 사용하지 않는다.

**현재 검증 범위(2026-10-14):** 하드웨어 플러그인의 실제 DDS 계약 시험과
C++ 계약 시험 7개가 통과했다. Isaac 물리 F/T와 두 카메라 입력을 관측했고,
자유 공간 중력 보정 후 축별 힘 RMS는 약 `[0.0804, 0.0798, 0.0633] N`,
힘 norm RMS는 약 **0.13 N**이었다. DDS 플러그인 시험은 폐기 가능한 상태
발행기를 사용한 별도 시험이다. 별도 물리 하중 시험 8개에서 10 N 축별 힘,
1 Nm 축별 토크, 편심 하중의 측정 평균 오차는 최대 **0.00069 N / 0.00044 Nm**였다.
0.10 m 편심 위치의 10 N 하중은 예상 1 Nm에 대해 **0.99990 Nm**로 측정됐다
([하중 시험](../logs/rb20-painting/ft-physics-validation.json)).
F/T는 500 Hz 물리 반력과 중력 항을 각각 읽고, 연속된 유효 표본 5개의 평균을
100 Hz로 발행한다. 원본 윈도우 끝 step·timestamp를 유지하고 pause/reset,
invalid 표본·step gap에서 평균을 초기화하며, clipping이나 목표 힘 대입은 하지 않는다.
실제 사전 접촉 자세에서 기존 tare 기준을 그대로 적용한 20회 시험은 순간 표본
3/20회, 5-step 평균 20/20회가 통과했다. FIR의 시뮬레이션 시간 기준 군지연은
4 ms이며, 실제 10 N 편심 하중 시험의 90% 응답은 6 ms에서 16 ms로 늘었다.
정상 상태의 하중·모멘트는 9.99785 N / 0.99990 Nm로 유지됐고, 드문 물리
transient는 제거하지 않는다([평균·tare·지연 진단](../logs/rb20-painting/ft-decimation-validation.json)).
구동 강성은 SI 단위 **300000 N·m/rad**, 감쇠는 **30000 N·m·s/rad**로
명시하며, USD의 degree 기준 값에는 `π/180`을 곱해 기록한다. 벽 없는 초기·D405·
사전 접촉 자세 시험에서 최대 TCP 오차는 사전 접촉 **1.16 mm / 0.052°**,
명목 10.01 mm 롤러 간격의 물리 최솟값은 **9.51 mm**였다
([SI 구동 진단](../logs/rb20-painting/servo-si-free-space.json)). 롤러 접촉 콜라이더
하나에만 힘 스프링 **5000 N/m**, 감쇠 **50 N·s/m**를 적용한다. 2 mm/s 실제
전진 시험에서 기존 rigid 접촉은 표면 진입 시 수백 N으로 증가했다. 동일한
기존 구동 설정을 고정하고 롤러 물성만 비교했을 때 1.08/2.08/4.08 mm 침투에
약 9.78/18.89/37.09 N이 측정됐다
([접촉 진단](../logs/rb20-painting/roller-contact-sweep.json)). 다른 콜라이더는
rigid이며 깊은 침투 시 원본 지지대 충돌도 유지된다. 이는 실물 롤러의 물성을
보정한 값이 아닌 시뮬레이션 가정이며, geometry·F/T 원값·안전 임계값은 유지한다.
독립 PhysX `ft_link` pose와 실측 관절의 ROS FK를
비교한 평균 오차는 약 **4 µm / 0.0002°**로 기록되었다. 전체 ROS/Isaac 그래프의
30초 관측에서 신선한 측정값·카메라·controller·좌표 일치 등 건강 검사 9개가
통과했다([측정 보고서](../logs/rb20-painting/explicit-loopback-health.json)).
원본 D405 측정 자세 이동의 정지 후 TCP 오차는 **0.314 mm / 0.0223°**였다
([측정 기록](../logs/rb20-painting/d405-measured-arrival.json)). 브라우저 관리 화면과
스케치 화면의 domain 106 / WebSocket 9091 연결 및 렌더 카메라 표시도 확인했다
([화면 캡처](../logs/rb20-painting/browser/sketch.png)).
새 SI 구동·탄성 롤러 조합에서 **두 획과 최종 후퇴/COMPLETE가 반복 완료**됐다.
두 번째 획의 tare 전에 최대 2초 동안 실측 도착을 기다리며, 기존 3 mm 위치·
7 mm 여유 거리·3° 자세 및 접촉/힘 차단 기준은 그대로 적용한다.
최종 v6 실행은 **엄격 도장 판정과 건강 검사 9개 모두 통과**했다.
획별 평가 구간은 10.22초/10.68초, 평균 **21.39 N / 21.63 N**,
p95 **24.13 N / 27.02 N**, 최대 **25.07 N / 28.06 N**,
18–32 N 비율은 **98.57% / 97.55%**였다. 두 획 모두 힘 유지 기준을 통과했다.
접근·tare·탐색·가압·도장·감압·최종 후퇴를 포함하는 이벤트 76개가 연속 번호
1–76으로 빠짐없이 기록됐다. D405 평면 오차는 **0.0047 mm / 0.00029°**였다
([최종 도장 보고서](../logs/rb20-painting/compliant-manual-v6-report.json)).
같은 v6 실행의 실측 관절 FK로 계산한 마지막 법선 후퇴는 **79.95 mm**였고,
롤러 표면과 벽의 최종 평균 간격은 **80.23 mm**, 접촉력은 **0 N**이었다
([실측 형상 후처리](../logs/rb20-painting/compliant-manual-v6-report-geometry.json)).
동일 JTC 갱신 주기의 원래 경로 기준값과 실측 피드백을 비교한 접선 오차는
획별 RMS **0.80 / 0.82 mm**, p95 **1.27 / 1.18 mm**, 최대 **1.36 / 1.20 mm**였다.
법선 방향 변위 RMS **0.57 / 0.62 mm**에는 compliance와 서보/모델 오차가 함께
포함된다. 별도로 관측한 controller의 직접 법선 보정량은 최대 **1.41 mm**로
설정 한도 20 mm 이내였고 `normal_limit_reached`는 한 번도 참으로 관측되지 않았다.
보정량 스칼라에는 원본 stamp가 없으므로 구간 연결은 수신 당시 State 기준이며,
관측 사이의 모든 제어 갱신을 증명하는 수치는 아니다
([경로·법선 보정 분석](../logs/rb20-painting/v6-controller-reference-analysis.json)).
원본 브라우저의 실제 포인터 조작으로 대상 선택·D405 측정·작업영역 확정,
수동 두 획 검증, 다른 ID의 자동 채우기 경로 검증, tare 승인·Run·PAINT 진입·Stop과
백엔드 중단 응답을 확인했다
([브라우저 실행 기록](../logs/rb20-painting/browser/e2e-2026-10-13T17-06-32-757Z/report.json)).
Stop은 PAINT 진입 직후 다음 이동 궤적 전송 전에 요청되었다. 관측된 명령 차단은
약 9 ms였고 이후 5초 동안 재활성화되지 않았다. 이는 활성 이동 궤적 취소나
정확한 기계 정지 시간의 증거는 아니다
([중단 관측](../logs/rb20-painting/browser-ui-stop-analysis.json)).
실제 타임라인 Pause/Play/Stop/Play 시험도 통과했다. Pause/Stop 중 관절·F/T는
invalid/NaN이고 카메라 준비 상태가 해제되며, 새 렌더 버퍼 이후에만 복구됐다.
Pause/Play는 세션을 유지하고 Stop/Play는 새 세션·초기 step을 발행했다
([타임라인 시험](../logs/rb20-painting/timeline-reset-observer.json)). 이 단독 시험에는
활성 이동 명령을 주입하지 않았으므로 이동 중 취소 시험과 구분한다.
유효한 도장 계획의 SAFETY_APPROACH 궤적이 실제 관절을 움직이는 동안 소유한
시뮬레이터 프로세스만 1초 멈춘 시험도 통과했다. 마지막 수신 상태 이후 약
**198 ms**에 stale을 감지했고, 감지 후 **1.1 ms**에 disabled 명령,
**9.1 ms**에 같은 궤적 UUID의 CANCELED, **19.1 ms**에 취소 서비스 ACK를
관측했다. 입력 복구 뒤 5초 동안 명령이 다시 활성화되지 않았고, 작업 평면과
이전 계획 검증은 무효 상태를 유지했다
([실제 이동 중 입력 단절 시험](../logs/rb20-painting/actual-stale-stop-v2.json)).
이 시험은 물리·카메라·clock의 동시 중단이며 카메라만의 고장을 분리한 시험은
아니다. 물리 엔진도 정지하므로 위 수치를 기계 감속 완료 시간으로 해석하지 않는다.

## 준비

현재 작업 폴더에서 실행한다.

```bash
cd /home/minjea/robotics/sketch-robot-runtime-main
bash scripts/setup_system_api.sh
bash scripts/build_rb20_painting_sim.sh
```

필요 환경은 ROS 2 Jazzy(`/opt/ros/jazzy`), Python 3.12, colcon/CMake,
MoveIt·ros2_control·rosbridge와 Isaac Sim 6.1 및 해당 GPU 렌더링 환경이다.
Isaac 가상환경 기본 경로는 작업 폴더의 상위 `../.venvs/isaacsim-6.1`이며,
다른 설치 위치는 `ISAACSIM_VENV`로 지정한다. Isaac 실행 스크립트는 EULA 동의를
환경에 설정하므로 해당 설치의 이용 조건을 확인해 둔다.

빌드는 `.runtime/rb20_painting/{build,install,log}`에 격리된다. 시스템에
TRAC-IK가 없으면 공식 Jazzy Debian 패키지를 다운로드하여
`.runtime/rb20_painting/deps`에 로컬 추출한다. 이 경우 ROS 패키지 저장소 접근과
`apt-get`, `dpkg-deb`가 필요하다. 빌드 스크립트는 모델·도장 설정과 시뮬레이션
전용 보정 파일을 생성하며, 실장비 보정 파일을 사용하지 않는다.

생성 자산은 `.runtime/rb20_painting/assets/`의 `robot.urdf`,
`robot_physics.urdf`, `robot.srdf`, `robot.usd`, `manifest.json`, `painting.yaml`,
`controllers.yaml`, `scene.json`, `calibration/{zed,d405}.json`이다. USD가 없거나
검증 결과 오래된 경우 시뮬레이터 실행 스크립트가 먼저 재가져오기한다. 최초
시작은 모델 가져오기와 GPU 셰이더 준비 때문에 오래 걸릴 수 있다.

## 관리 화면으로 시작

```bash
SKETCH_ROSBRIDGE_PORT=9091 bash scripts/run_system_api.sh --rb20-paint --port 8070
```

관리 화면은 `http://127.0.0.1:8070/`, 기존 스케치 화면은
`http://127.0.0.1:8070/sketch/`다. 관리 화면의 **RB20 도장 · Isaac Sim**은
모델 `rb20_1900es`, 작업 방식 `paint`, 카메라 입력 `native`로 고정된다.
화면의 native 표기는 기존 ROS 토픽 입력을 뜻하며 실물 카메라 드라이버는
비활성이다. 로봇 IP와 실물 카메라 연결값도 비활성이다.

1. 관리 화면에서 **전체 시작**을 누른다. 데스크톱 디스플레이가 있으면 시뮬레이터
   창이 열리고, 없는 환경에서는 자동으로 headless 렌더링을 시작한다.
2. `simulation → robot_control → perception / force_pipeline → executor` 및
   별도 rosbridge가 준비되는 동안 관리 화면의 로그·ROS 준비 상태를 확인한다.
3. **스케치 화면 열기**에서 실제 렌더 영상을 확인한다. **① 대상 인식**에서
   대상 영역을 그리고 **평면 추출**, **선택한 면 측정**을 진행한다. 측정 완료 후
   **다음: 작업 지정**을 누르고 **② 작업 지정**에서 **작업영역 확정**을 진행한다.
4. 영역 확정 후 **다음: 계획·실행**을 누른다. **③ 계획·실행**에서 경로 방식을
   자동 또는 수동으로 선택하고, 수동이면 두 개의 분리된 획을 완성한다.
   **계획 생성·검증**을 누르고 유효한 D405 측정·계획 identity 및 접촉 전 영점
   조정 승인 등 준비 조건을 충족한다. Settings는 서버 값 확인용이다.
   경로 생성·계획 검사·실행 준비를 따로 표시한다. 계획 검사는 경로·작업영역·공정
   조건 검사이며 이동 구간 검사는 실행 중 수행한다. 영점 조정 승인은 측정 완료가
   아니며, 접촉 전 영점과 접촉력 유지는 실행 중 확인한다.
5. 기존 **작업 시작**을 눌러 실행한다. 진행 단계와 목표/측정 접촉력을 확인하고,
   중단은 **작업 중단**으로 요청한다. 실측 정지 및 취소 결과를 확인한다.
6. 마치면 관리 화면에서 **전체 종료**를 누르고 API 터미널을 Ctrl+C로 종료한다.

응답 수신만으로 단계가 자동 전환되지 않는다. 탭 왕복은 계획을 유지하고 실제 입력 변경은 재검증을 요구한다. **작업 중단**은 모든 단계에서 사용할 수 있다.

**전체 시작은 시스템 준비다. 작업 시작 전에는 도장 궤적과 힘 출력이 활성화되지
않는다.** 기존 엄격 실행 검사를 유지하기 위해 내부 파라미터는
`real_painting_enabled=true`, `dry_run=false`, `painting_force_enabled=true`지만,
하드웨어는 시뮬레이션 플러그인이고 force/compliance의 초기 출력은 OFF다.

전용 ROS domain은 **106**, 탐색 범위는 localhost다. 기존 Construct domain 105와
분리된다. 전용 실행 환경은 `scripts/rb20_painting_dds_env.sh`와
`config/rb20_painting_fastdds.xml`로 UDP 통신·탐색 주소를 `127.0.0.1`에 제한한다.
Jazzy의 `LOCALHOST` 탐색 설정은 공유 메모리 통신을 다시 추가하므로
`ROS_LOCALHOST_ONLY`를 해제하고 `SYSTEM_DEFAULT`와 명시적 XML을 함께 사용한다.
단순 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 설정만으로는 공유 메모리가 꺼지지 않는다.
실행 프로세스의 DDS 스레드와 메모리 매핑으로 적용 여부를 검증한다.
입력 정지 감지 기준은 0.20초로 유지한다. 관절 명령은 원본 State의 정수 timestamp를 그대로 돌려보내며, 시뮬레이터는 해당 물리 표본을 읽은 steady 시각에서 0.20초가 지났으면 명령을 거부한다. 렌더링이나 전달이 늦어진 과거 명령은 도착 시각으로 유효 기간을 갱신할 수 없다.
다른 domain에서 시작한 API는 RB20 선택·기동을 거부한다. 이때 위의
`--rb20-paint` 명령으로 API를 다시 시작한다. API 포트와 WebSocket 포트는
별개다. WebSocket 기본값은 9090이며 위 예시는 기존 서비스를 보존하기 위해
9091을 사용한다. 두 기존 웹 페이지는 API의 `/runtime-connection`에서 포트를
읽은 후 연결한다. 포트 점유 시 기존 서비스를 종료하지 말고 사용 가능한 포트로
API를 다시 시작한다. 한 작업 폴더에는 관리 API 하나만 실행한다.

## 수동 GUI / headless 진단

관리 화면의 **전체 시작**과 수동 프로세스 기동을 동시에 사용하지 않는다.
각 터미널은 같은 작업 폴더에서 실행한다.

```bash
# 터미널 1: GUI
bash scripts/run_rb20_painting_isaac.sh

# 또는 화면 없이 실행
bash scripts/run_rb20_painting_isaac.sh --no-window
```

`.runtime/rb20_painting/managed-ready.json`이 새 측정으로 갱신된 후 별도 터미널에서:

```bash
# 터미널 2: 기존 ROS 체인 전체
bash scripts/run_rb20_painting_ros.sh all
```

ROS의 개별 진단 명령은 `all` 대신 `robot_control`, `perception`,
`force_pipeline`, `executor`를 지정한다. 선행 제어·인지·힘 프로세스를 먼저
준비한다. 수동 `all`에는 별도 관리 대상 rosbridge가 포함되지 않는다.
시뮬레이터의 유한 시간 센서 진단은 다음과 같다. 이 명령은 도장을 시작하지 않는다.

```bash
bash scripts/run_rb20_painting_isaac.sh --no-window --duration 30 \
  --capture logs/rb20-painting/cameras
```

수동 실행은 ROS 터미널부터 Ctrl+C로 종료하고 시뮬레이터도 Ctrl+C로 종료한다.
다른 스택에 광범위한 `pkill`이나 ROS 전체 종료 명령을 사용하지 않는다.

## 측정과 판정

실행 중인 시뮬레이터 및 ROS 체인에 대해 별도 터미널에서:

```bash
source scripts/rb20_painting_ros_env.sh
python3 scripts/check_rb20_painting_isaac.py --duration 15 \
  --output logs/rb20-painting/health-report.json
```

이 검사는 `scope=live_integration_health`로 신선한 관절/F/T, 카메라 스트림,
하드웨어 inhibit, lifecycle 상태 및 필수 활성 controller를 확인한다.
종료 코드 0은 이 범위의 통과만 뜻한다. 사용자 작업을 생성하거나 실행하지 않는다.

### 기존 인터페이스를 통한 CLI 작업 흐름

검사기는 `--workflow`로 기존 화면과 동일한 픽셀·평면 선택·작업영역·경로 요청을
보낼 수 있다. 장면의 벽에 투영한 폴리곤은 사용자 입력 fixture이며, 평면과
D405 측정·경로는 기존 노드가 렌더 depth와 실측 관절에서 계산한다. 직접 평면을
인지 결과로 주입하거나 controller를 우회하지 않는다.

```bash
source scripts/rb20_painting_ros_env.sh

# 대상 평면 선택 및 기존 D405 측정까지만
python3 scripts/check_rb20_painting_isaac.py --workflow select-only --duration 15 \
  --output logs/rb20-painting/selection-report.json

# 작업영역 측정과 두 개의 분리된 수동 획 생성·검증
python3 scripts/check_rb20_painting_isaac.py --workflow manual --duration 15 \
  --output logs/rb20-painting/manual-plan-report.json

# 작업영역 측정과 기존 자동 채우기 경로 생성·검증
python3 scripts/check_rb20_painting_isaac.py --workflow fill --duration 15 \
  --output logs/rb20-painting/fill-plan-report.json
```

한 번에 한 흐름만 요청하고 새 시험 전에는 이전 작업·측정·계획 상태를 정리한다.
D405 측정에는 기존 측정 자세 이동이 포함될 수 있다. `--execute`가 없으면
도장 Run은 요청하지 않는다. 다음 명령은 준비 검사 후 기존 자유 공간 승인과
Run 인터페이스를 통해 **실제 시뮬레이션 도장 실행을 요청한다.** 이 동안 브라우저에서
별도의 Run을 요청하지 않는다.

```bash
python3 scripts/check_rb20_painting_isaac.py --workflow manual --execute --duration 15 \
  --output logs/rb20-painting/manual-painting-report.json
```

자동 채우기 실행은 위 명령의 `manual`을 `fill`로 바꾼다. `--execute`에는
`manual` 또는 `fill`이 필요하며 `select-only`와 함께 사용할 수 없다.
`--duration`은 workflow 처리 후 추가 관측 시간이다. 각 준비·측정·검증·실행
단계에는 별도 제한 시간이 있어 전체 명령 실행 시간은 이 값보다 길 수 있다.
workflow 실패는 중단 요청과 실패 보고서로 남긴다.

### 브라우저 실행을 수동 기록

두 획 시험은 **작업 시작 전** 아래 기록기를 켠 뒤 기존 스케치 화면에서 진행한다.
기록 시간을 전체 접근·tare·탐색·가압·두 획·감압·최종 후퇴가 포함되도록 잡는다.

```bash
source scripts/rb20_painting_ros_env.sh
python3 scripts/check_rb20_painting_isaac.py --record-painting --duration 300 \
  --output logs/rb20-painting/painting-report.json
```

`--record-painting` 또는 `--execute`는 `scope=painting_execution`으로
엄격한 도장 판정을 적용한다. `--execute`에는 `--record-painting`을 추가하지 않아도
자동으로 같은 판정이 적용된다. 두 개의 독립 PAINT 구간,
접근/탐색/가압/감압/후퇴, runtime tare 완료와 최종 COMPLETE를 요구한다.
각 PAINT의 첫 1초 이후 최소 5초의 신선한 실측 구간에서 95% 이상이
18–32 N이어야 한다. 실행 설정은 기존 20 N 목표와 20–30 N hold band다.
측정 누락·오래된 입력·세션 변경·단계 누락·힘 조건 실패는 실패로 남긴다.
JSON 보고서 옆 `*.trace.json`에 상태 이력과 관절/힘 측정 기록을,
`*.samples.csv`에 steady/simulation 시간, step, session, 측정 접촉력, 입력 age 및
validity를 저장한다. 예를 들어 `manual-painting-report.json`과 함께
`manual-painting-report.trace.json`, `manual-painting-report.samples.csv`가 생성된다.
관절 trace에는 실제 측정 위치와 별도로 명령 위치·명령 age·enable 상태를 기록한다.
전환 이력은 `/painting_system/execution_events`의 reliable/volatile 큐에서 수집한다.
동일 JSON과 `execution_sequence`를 실제 전환 시점에 발행하며 과거 실행을 재생하지
않는다. 기존 화면의 최신 상태 토픽 `/painting_system/execution_status`는 유지한다.

전체 인수에는 이 보고서 외에도 렌더 depth 기반 평면 정확도, 실측 TCP 추종,
실제 최종 후퇴 거리, 중단 ACK/정지 시간, 센서 단절·pause/reset·이전 계획 거부와
브라우저 수동/자동 경로 시험 증거가 필요하다. 기록기 통과 하나로 이를 대신하지
않는다. 생성 로그·이미지·성능 기록은 `logs/rb20-painting/`, 프로세스별 관리 로그는
`logs/system_api/`에서 확인한다.

## pause / reset / 오류 후 복구

시뮬레이션 stamp는 `/clock`, 입력 watchdog과 명령 수명은 steady time을 사용한다.
준비 판정은 소유 PID, 실행별 session, 모델/장면 hash, 2초 이내 heartbeat,
진행하는 physics step, 유효 관절/F/T와 두 카메라를 요구한다. 실측 입력이
사라지면 정상 0 N으로 대체하지 않는다.

Pause, Stop/reset, `/clock` 정지, 세션 변경 또는 센서 단절은 새 운동·가압을
차단하고 기존 중단 경로를 요청한다. Play만 눌러 이전 작업을 재개하지 않는다.
관리 화면의 **전체 종료 → 전체 시작**으로 새 스택을 준비하고,
대상/작업영역/D405 측정/경로 승인과 tare를 다시 진행한다. reset은 새 session을
만들며 이전 session의 명령·계획을 재사용할 수 없다. 수동 실행도 ROS 체인을
종료한 뒤 새 simulator session에 맞춰 다시 기동한다.

물리 500 Hz, 제어 100 Hz, 렌더 15 Hz는 설정 목표이며 실제 달성률과 실시간 배율은
측정 로그로 확인해야 한다. RTX 5080의 최종 v6 전체 실행에서는 실측 physics step
약 **363 Hz**, 상태 수신 약 **73 Hz**, 실시간 배율 약 **0.73**이었다.
롤러 접촉 물성은 시험 모델이고, 카메라는 실물의
노이즈·스테레오 오차까지 보정한 모델이 아니다. 시뮬레이션 결과는 실물 RB20의
장착·센서 보정·공정 품질 또는 시운전 완료를 의미하지 않는다.

## 회귀와 검증 범위

관련 Python 검사 91개, 웹 검사 91개, 관리 프로파일 검사 37개와 실제
controller_manager를 사용한 DDS 플러그인 시험이 통과했다. C++는 하드웨어
계약 7개 및 admittance/롤러 균형/플러그인 검사 65개가 통과했다. 이후 변경한
tare 도착 대기·전환 기록·요청 generation은 별도 회귀와 독립 리뷰를 거쳤다.
실행 기록은 `logs/rb20-painting/final-*-regression.log`에 있다.

전체 저장소 테스트가 모두 통과했다는 의미는 아니다. 더 넓은 실행에서 기존
HEAD에서도 재현되는 12개 실패(옛 접촉 mock 10개, 기존 spray/zed2i 소스 계약
2개)를 확인했고, 기존 서비스가 점유한 9090을 빈 포트로 가정하는 API 검사
4개도 분리했다. 관련 관리 API 검사는 테스트용 포트 격리 상태에서 통과했다.

실제 시험에는 이동 중 입력 단절, PAINT 진입 직후 UI Stop, 과력 guard,
Pause/Stop/reset, 복구 후 기존 계획 무효화가 포함된다. 잘못된 세션은 별도
DDS 하드웨어 시험에서 거부했다. 카메라만의 고장, 물리 실행 중 도달 불가
경로, 이동 중인 PAINT 궤적에 대한 UI Stop, 별도의 정지 20 N 가압 및 정확한
기계 감속 완료 시간은 이번 실제 시험에서 개별 검증하지 않았다.

### 중단 후 화면 복구

작업 중단 요청만으로 정지 완료로 표시하지 않는다. 기존 서버의 명시적 안전 해제
(`reset_execution_abort`) 후 새 IDLE 응답과 해제된 준비 상태를 확인하면 편집을 다시
허용하며, 중단 전 계획은 재사용하지 않는다. 서버 전체 재시작으로 복구했다면 로봇의
정지와 시스템 준비를 확인한 뒤 스케치 화면을 새로고침하고 대상을 다시 지정한다.
현재 도장·뿜칠 상태 메시지로는 재연결만으로 새 서버 세션을 판별할 수 없다.
