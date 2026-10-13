# Isaac 용접 이동 시험

기존 minjeasung 스케치 화면, 공식 Michelo·Outpost 카메라 경로, Construct의 MoveIt·FollowJointTrajectory를 사용한다. 이 기능은 3D 심을 따라 토치의 자세와 위빙을 검증하는 시뮬레이션이다. 실제 용접기 출력, 아크·열·용입은 구현하지 않았다.

## 실행

```bash
bash scripts/build_construct_sim.sh
bash scripts/run_system_api.sh --construct
```

기존 시스템관리 화면에서 `Isaac Construct`, `construct_left`, `용접`을 선택하고 준비한다. 이미 다른 공정이 실행 중이면 전체 종료 후 변경한다. 현재 weld 실행은 Construct 시뮬레이션만 지원한다. 기존 실장비 paint/spray 경로는 유지하며, 실제 용접은 검증된 장비·공구·출력 어댑터를 연결해야 활성화할 수 있다.

기존 스케치 화면에서 다음 순서로 사용한다.

1. ZED 원본 영상에서 작업대상을 하나의 사각형 또는 닫힌 스케치로 표시한다.
2. **시험 용접심 불러오기**를 누른다. fixture 모드는 H형강 모서리에 등록된 S1/S2/S3을 반환하며 영상에서 심을 추출하지 않는다.
3. 목록에서 실행할 심을 선택한다. 위쪽 이동 버튼으로 순서를, 방향 버튼으로 진행 방향을 바꾼다.
4. 오른쪽 **Settings**에서 조건을 입력하고 적용한다.
5. **용접 경로 검증**을 누른다. 선택한 모든 심의 접근·진입·용접·이탈이 검증된 후에만 작업 시작이 활성화된다.
6. 기존 **작업 시작/중단** 버튼을 사용한다. Isaac에 중심선, 위빙 경로, 토치 축이 표시된다.

3D 심을 ZED 영상에 투영하는 기능은 이번 범위에서 제외했다. 대상 스케치를 용접 경로로 사용하지 않는다. 대상·심·설정·장면 변경, 카메라 헤드 이동, 스트림 변경, 시뮬레이션 reset은 기존 계획을 무효화한다. 중단 확인 전에는 다음 심이나 새 실행을 시작하지 않는다.

## Settings

용접 속도, 접근/이탈 거리, standoff, 작업각/진행각, 위빙 ON/OFF·전체 폭·주파수·좌우 체류 시간, 전류, XYZ 심의 기준 자세 선택을 제공한다. 기본값은 시뮬레이션 예시이며 사용자가 적용해야 한다. 전류는 가상 공정 상태에만 반영된다.

XYZ 좌표만 받은 경우 **XYZ 심의 기준 자세**에서 등록된 시험 프로파일을 명시적으로 선택하고 적용해야 한다. 기본은 선택 안 함이며, 임의의 표면 법선을 추정하지 않는다. 현재 시험 프로파일의 외측 기준 방향은 World 좌표계 `(0, -1, 0)`이다. 추출기가 자세나 기준 방향을 제공하면 그 값을 우선 사용한다. 실제 작업물에는 그 형상에 맞는 기준 방향·자세가 필요하다.

UI는 mm, mm/s를 표시하고 ROS 계약은 m, m/s, s, deg, A를 사용한다. 위빙 폭은 양 끝 사이의 전체 폭이다. 체류 중에도 중심선 방향으로 전진한다. 작업각은 진행 접선축, 진행각은 음의 횡축 회전이며 입력 자세에 적용한다. 방향을 반전하면 각도의 기준축도 함께 반전한다. 시험 심은 두 면 사이의 45도 기준 자세를 포함한다. 샘플의 관절 속도·가속도를 만족할 수 없으면 용접 조건을 임의로 바꾸지 않고 계획을 거부한다.

도장·뿜칠 Settings는 서버에서 읽은 현재 값을 표시한다. 이 단계에서는 해당 공정의 실행 중 편집 API를 추가하지 않았다. 값을 아직 받지 못했으면 빈 필드로 표시한다.

## 개발 중인 추출기 연결

기본 제공자는 `fixture`다. API 실행 환경에 `WELD_INPUT_PROVIDER=external`을 설정하고 스택을 다시 준비하면 UI는 같은 흐름에서 **용접심 추출 요청**을 보낸다. 개별 launch의 `weld_input_provider:=external`도 지원하며 perception과 executor 양쪽에 동일하게 적용해야 한다.

모든 아래 토픽은 `std_msgs/msg/String` JSON이다.

- `/welding/target_request`: UI의 request_id, target_revision, generation, frame_id, stamp `{sec,nanosec}`, clock_domain, width/height, 원본 픽셀 polygon, provider, server_session_id.
- `/welding/extraction_request`: 입력 노드가 실제 ZED 프레임 정보와 대조한 대상 요청. 추출기는 이 메시지를 구독한다.
- `/welding/extraction_result`: 추출기 결과. 아래 형식으로 응답한다.

```json
{
  "schema_version": 1,
  "request_id": "요청과 동일",
  "target_revision": "요청과 동일",
  "generation": "요청과 동일",
  "source": "external",
  "frame_id": "World",
  "length_unit": "m",
  "seams": [{
    "seam_id": "joint-01",
    "points_xyz": [[-0.30, -1.305, 0.92], [-0.28, -1.305, 0.92]],
    "reference_up_vectors": [[0, -0.7071067811865476, 0.7071067811865476]]
  }]
}
```

`reference_up_vectors`는 외측 기준 방향이며 한 개 또는 점마다 제공한다. 대신 `quaternions_xyzw`로 공구 끝의 자세를 제공할 수 있다. 두 필드를 동시에 사용하지 않는다. 위치만 있는 심은 목록에 표시하며, 실행하려면 Settings에서 기준 자세를 선택하거나 입력에 일치하는 `pose_profile_id`를 제공한다. Quaternion은 xyzw 순서, 심 좌표계에 대한 공구 끝 회전이다. `T_world_tcp = T_world_tip × inverse(T_tcp_tip)`를 적용한다. 완만한 곡선은 입력 점을 잇는 중심선을 유지하며 인접 접선을 보간해 토치 자세와 위빙 축을 연속적으로 구성한다.

`World` 이외의 결과에는 ROS 시계의 `stamp`와 `clock_domain: "ros"`가 필요하다. 대상 영상과 동일한 촬영 시각이어야 하고, 해당 시각의 TF가 없으면 거부하며 최신 TF로 대체하지 않는다. mm 입력, 잘못된 요청 ID, 중복 심 ID, 불연속 점, 과도한 방향 변화, 잘못된 자세는 거부한다. 10 cm 넘는 점 간격이나 45도 넘는 급격한 방향 변화는 추출기에서 분할해야 한다.

추출기는 `/welding/seams`에 직접 발행하지 않는다. 이 토픽은 검증·변환된 내부 결과다. Fixture 파일은 `src/sketch_control/config/construct_weld_fixture.json`, 시험 토치 프로파일은 `weld_sim_profile.json`이다. 프로파일은 실제 용접 토치 보정값으로 사용할 수 없다.

## 상태와 검증

`/welding/status`는 세션, 계획 ID, revision, 계획 시도 ID, 상태, 현재 심/단계와 명령 ACK를 제공한다. 중단한 계획·실행의 늦은 응답은 새 작업에 적용하지 않는다. 충돌 장면 갱신 중에는 실행을 차단하고, 조회한 장면의 토치 부착 형상까지 확인한다. 계획·대기·실행 중 반대쪽 팔 등 비제어 관절이 움직이면 계획을 무효화하거나 중단한다.

`/process_settings/status`는 적용된 서버 설정이다. `/welding/output_status`는 실제 WELD 구간의 컨트롤러 피드백에 맞춘 가상 출력이며 이동·중단·타임아웃에는 OFF다. 출력 어댑터 경계는 `configure`, `start_segment`, `stop_segment`, `status`이고 현재 구현은 `SimWeldOutput`이다.

```bash
source scripts/construct_ros_env.sh
python3 scripts/check_construct_weld.py --case complete
python3 scripts/check_construct_weld.py --case stop --report logs/weld-stop.json
python3 scripts/check_construct_weld.py --case invalid-frame --report logs/weld-invalid-frame.json
python3 scripts/check_construct_weld.py --case unreachable --report logs/weld-unreachable.json
python3 scripts/check_construct_weld.py --case reset --report logs/weld-reset.json
```

`complete`는 S1/S3 순서, 실제 TF 피드백, 위치 5 mm·자세 5도, 속도·폭·위빙 주파수 오차 5%를 확인한다. 시간은 컨트롤러의 시뮬레이션 시각을 사용한다. 결과는 `logs/construct-weld-report.json`과 CSV에 기록한다. 다른 case의 기본 파일명에는 case 이름을 포함하므로 complete 측정 파일을 덮어쓰지 않는다. `reset`은 관리 중인 시뮬레이션 스택을 실제로 종료·재시작하고 과거 실행 명령 거부를 확인한다. 잘못된 프레임/도달 불가 시험에서는 로봇 실행을 요청하지 않는다.

2026-10-13 Construct 왼팔 실제 시뮬레이션 시험 결과:

| 측정 | S1 | S3 |
|---|---:|---:|
| 최대 TCP 위치 오차 | 0.678 mm | 0.672 mm |
| 최대 자세 오차 | 0.0140° | 0.0128° |
| 중심선 진행 속도 (요청 10 mm/s) | 10.0002 mm/s | 9.9990 mm/s |
| 위빙 전체 폭 (요청 4 mm) | 4.010 mm | 3.970 mm |
| 위빙 주파수 (요청 1 Hz) | 1.000 Hz | 1.000 Hz |

S1→S3 실행 및 모든 위 기준을 통과했다. 중단·잘못된 프레임·도달 불가·스택 reset 후 과거 실행 명령 거부도 통과했다. 측정 파일은 `logs/construct-weld-report.json`, `logs/construct-weld-trace.csv`, `logs/weld-tracking.png`, `logs/weld-{stop,invalid-frame,unreachable,reset}.json`이다. 최초 시험에서는 관절 구동 지연으로 폭이 감소했으며, 용접 모드에만 Isaac 위치 구동 강성을 100000으로 설정해 해결했다. 기존 paint/spray 값 50000은 유지한다. 용접 모드의 OMPL 충돌 검사 간격(`longest_valid_segment_fraction`)은 0.0001, 시간 보정의 관절 경로 이탈 허용량(`totg.path_tolerance`)은 0.001로 설정한다. 초기 접근에서 드문 충돌 경로가 최종 검증에 걸리는 문제를 줄이기 위한 설정이며 최종 `ValidateSolution` 검사는 유지한다. 설정 의미는 [MoveIt OMPL 설명](https://moveit.github.io/moveit_tutorials/doc/ompl_interface/ompl_interface_tutorial.html)과 [시간 보정 설명](https://moveit.github.io/moveit_tutorials/doc/time_parameterization/time_parameterization_tutorial.html)에 따른다.

이 수치는 시험 심·토치·설정에서의 시뮬레이션 결과이며 실제 용접 정확도를 의미하지 않는다. 후속 Chrome 브라우저 검사에서 실제 마우스 드래그로 대상 사각형 표시와 S1/S2/S3 불러오기를 확인했다. 이 과정에서 그리기 도구 잠금과 ROSLIB 명령 발행 바인딩 오류를 수정했다. 실제 이동은 공개 ROS 인터페이스로 검증했다.

최종 자동 검증: ROS/Python 관련 검사 269개, API·Outpost 검사 82개, UI 검사 81개 통과. 시스템 Python에서 FastAPI 미설치로 건너뛴 2개는 API 가상환경 검사에 포함해 통과했다. Construct 빌드와 독립 코드 리뷰도 완료했다. 후속 UI 수정 후 UI 검사 83개가 통과했다.
