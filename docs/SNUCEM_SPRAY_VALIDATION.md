# SNUCEM ARM64 Humble 애드온 검증 기록

대상 브랜치: `codex/snucem-spray-addon`. 원본 기준:
`JongHyunSeo11/SNUCEM_Robot_22.04@7b0a2edcc3d1659bdc2dcb495d4dd63f75899f1c`.
원본은 GitHub connector로 읽기만 했으며 원본 소스는 배포물에 포함하지 않았습니다.
현재 배포 대상은 Jetson AGX Orin Developer Kit / ARM64 / CUDA 12.6 / ZED SDK 5.3.1입니다.
JetPack/L4T 세부 버전 및 실제 GPU·카메라 동작은 원격으로 확인하지 못했습니다.

## 로컬 검증

Windows / Python 3.12에서 다음 명령은 **231 passed, 2 skipped**였습니다.
skip은 Windows symlink 권한과 설치되지 않은 rclpy입니다.
의존성의 AnyIO deprecated alias 경고 1개가 있으며 테스트 실패는 아닙니다.

```bash
export PYTHONPATH="$PWD/addons/snucem_spray:$PWD/src/sketch_control:$PWD/src/rbpodo_painting_control"
python -m pytest -q addons/snucem_spray/tests \
  src/sketch_control/test/test_zed_spray_projection.py \
  src/sketch_control/test/test_zed_spray_generation.py \
  src/sketch_control/test/test_polygon_work_area.py \
  src/rbpodo_painting_control/test/test_spray_eoat.py
node --test web/test/*.test.js
```

브라우저 테스트는 **66 passed**. `git diff --check` 통과.
Python 소스는 Python3.10 문법으로 parse 검사했습니다.
번들 재생성 결과가 바이트 단위로 같고, 파일별 해시/경로 검증, 설치 실패/실행 중
업그레이드 거절, 의존성 설치 실패 시 이전 활성 버전 유지, 설정·원본 파일 보존을 검사했습니다.
상태 파일의 symlink와 hardlink도 거절합니다.

전체 `src` suite를 Windows에서 실행한 결과는 665 passed, 18 skipped,
14 failed, 23 errors였습니다. ROS 메시지/ament 미설치, Windows의 SIGKILL 부재,
symlink 권한, 기존 테스트의 기본 cp949 인코딩 등 환경 의존 실패가 포함됩니다.
이를 전체 suite 통과로 표시하지 않습니다. 관련 기능의 집중 회귀 범위는 위와 같습니다.

## 독립 코드 리뷰

읽기 전용 리뷰에서 발견한 세 문제를 수정하고 재검사했습니다.

1. 오래되거나 TF 검증에 실패한 cloud 수신만으로 기존 평면의 freshness가 연장되는 문제:
   원본의 성공한 `_patch_observations` 시각만 사용합니다.
2. 같은 잠긴 평면의 measured support 재표본화/통과 품질값 변화가 작업을 취소하는 문제:
   여전히 관측으로 덮이는 보수적 고정 footprint를 유지하고 품질값을 기하 ID에서 분리했습니다.
3. Humble 기본 SIGINT 핸들러가 취소 전에 ROS context를 종료하는 문제:
   context를 유지해 OFF/취소를 먼저 요청하고 제한 시간 동안 결과를 처리합니다.

최종 리뷰에서 남은 critical/important 지적은 없었습니다.

## ARM64 전환 전 Humble CI 기록

`.github/workflows/snucem-humble.yml`은 `ros:humble-ros-base-jammy`에서
실제 ROS 메시지와 synthetic URDF/SRDF로 모듈 import, 노드 생성,
live 상태 누락 시 물리 dispatch 차단을 검사합니다.
2026-10-06의 [CI 실행](https://github.com/minjeasung/sketch-robot-runtime/actions/runs/37434249227)은
런타임 커밋 `51de13a1933681ff11bc7671d347638312610d51`에서 **전체 절차 성공**입니다.
핵심 회귀 검사는 **285 passed**, 환경 skip 없음, AnyIO 경고 1개입니다.
압축 배포물을 설치한 별도 Python 프로세스에서도 원래 checkout의 모듈을 가져오지 않고
실행기·projector·generator·preview 노드를 생성하고 명령 차단을 확인했습니다.
외부 원본 접근 권한이나 실제 로봇을 사용하는 작업은 아닙니다.

별도로 기존 `test_moveit_executor_fail_closed.py`의 119개를 변경 전
`ebc36dd676602ad913a187b32762b3aedd15b88f` 및 현재 코드에서 같은 Humble 환경으로 실행했습니다.
두 결과 모두 **109 passed, 10 failed**이고 실패한 테스트 이름도 동일합니다.
비교 보고서는 **new_regressions: [], missing: []**입니다.
기존 접촉 도장 fixture 누락 및 오래된 취소 동작 기대값의 실패를 숨기지 않으며,
이 suite 자체를 전체 통과로 표시하지 않습니다. 비교 XML/JSON은 CI artifact에 포함합니다.
경량 배포물 생성·업로드도 성공했습니다.

## ARM64 전환 검증

CI runner를 `ubuntu-22.04-arm`으로 변경했습니다. 같은 Humble 컨테이너를 native ARM64로
실행하며 `uname -m` 및 실제 Python 플랫폼 검사를 통과해야 다음 단계로 진행합니다.
모든 pip 의존성은 `--only-binary=:all:`로 설치합니다. 실제 ROS 메시지/노드 생성,
설치된 경량 배포물의 별도 프로세스 실행, 기존 접촉 도장 비교는 동일하게 유지합니다.
이 CI에는 Jetson GPU·CUDA·ZED 카메라가 없으므로 GPU 운전 검증과 구별합니다.
2026-10-06 [ARM64 CI 실행](https://github.com/minjeasung/sketch-robot-runtime/actions/runs/37435758171)은
런타임 커밋 `05fdc4e3917ca28fdba8d334caf538c6101939ac`에서 **전체 절차 성공**입니다.
로그의 실제 아키텍처는 `aarch64`이며 NumPy 1.26.4, SciPy 1.15.3, Shapely 2.1.2의
CPython 3.10 ARM64 wheel을 설치했습니다. 핵심 회귀 결과는 **298 passed**, skip 없음,
기존 AnyIO 경고 1개입니다. 배포물의 독립 프로세스 실행·노드 생성 검사도 포함합니다.
기존 접촉 도장 suite는 ARM64에서도 변경 전후 모두 **109 passed, 같은 10 failed**이며
`new_regressions: [], missing: []`입니다. 배포물 생성과 artifact 업로드도 성공했습니다.

## 현장 확인

현장에서는 다음 순서의 별도 검증이 필요합니다.

1. 원본 Git 상태 및 파일 hash를 저장하고 설치/종료 후 비교.
2. `doctor`, 실제 URDF/SRDF/mesh 모델 fingerprint, CameraInfo·stamped TF·보정 파일 일치 확인.
3. preview에서 면/작업영역/자유선/자동 채우기/clear/reselect, 셀 사이 빈 공간 거절 확인.
4. 외부 fake hardware stack의 dry_run에서 MoveIt 충돌검증 및 FJT 명령 미전송 확인.
5. 실제 이동이 허용된 시운전 환경의 motion_test에서 정지·취소·경쟁 실행기·관측 소실·모델 변경 차단 확인.
6. 독립 건 OFF 기본값/lease 만료/실제 ACK 검증 후 spray profile 운전.

실제 ROS 그래프에서 원본 wrapper가 동작하는지, MoveIt 전체 경로가 실행되는지,
실물 로봇과 분사 건의 정지·취소가 확인되는지는 로컬 검사와 이번 CI에서 검증하지 못했습니다.
