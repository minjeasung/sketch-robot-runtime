# Isaac 용접 이동 시험

기존 minjeasung 스케치 화면, 공식 Michelo·Outpost 카메라 경로, Construct의 MoveIt·FollowJointTrajectory를 사용한다. 이 기능은 3D 심을 따라 토치의 자세와 위빙을 검증하는 시뮬레이션이다. 실제 용접기 출력, 아크·열·용입은 구현하지 않았다.

## 확정 구성과 적용 상태

2026-10-19 기준 **상용로봇 플랫폼 오른팔의 Hongik V4 용접 EOAT와 레이저 센서**를
사용한다. 왼팔 기본 공구는 내화뿜칠 spray v2다. 대상체는 H_STEEL_USD이다.
양팔 EOAT에 D435는 포함하지 않는다. 레이저 센서는 형상·CAD 프레임까지 포함하며
실제 데이터 수신과 광학 교정 TF는 별도 연결 대상이다.

상용 플랫폼·오른팔·완전 시뮬레이션은 중앙 수평 H형강의 카메라 쪽 웹과
아래 플랜지 윗면이 만나는 접합선으로 데모를 구성했다. 새 초기 자세의 실제
카메라 시야·상태 충돌 검사 및 위빙 포함 전체 실행 계획을 확인했다.
이 배치에서 용접 동작 자체는 아직 실행하지 않았다. 아래 2026-10-19 항목과
[자산 구성](HONGIK_H_STEEL_IMPORT.md)을 기준으로 한다.

## 실행 통합 후 사용할 흐름

```bash
bash scripts/build_construct_sim.sh
bash scripts/run_system_api.sh --construct
```

운영 전환 완료 후 시스템관리 화면에서 상용로봇 플랫폼(내부 ID `Isaac Construct`), 오른팔(`construct_right`), `용접`을 선택하고 준비한다. 이미 다른 공정이 실행 중이면 전체 종료 후 변경한다. 현재 weld 실행은 Construct 시뮬레이션만 지원한다. 기존 실장비 paint/spray 경로는 유지하며, 실제 용접은 검증된 장비·공구·출력 어댑터를 연결해야 활성화할 수 있다.

기존 스케치 화면에서 다음 순서로 사용한다.

1. **① 대상 인식**: ZED 원본 영상에서 작업대상을 하나의 사각형 또는 닫힌 스케치로 표시하고 **시험 용접심 불러오기**를 누른다. fixture 모드는 등록된 시험 심을 반환하며 영상에서 심을 추출하지 않는다. H_STEEL_USD에 맞춘 심 좌표 등록이 선행되어야 한다. 후보 수신 후 **다음: 작업 지정**을 누른다.
2. **② 작업 지정**: 목록에서 실행할 심을 선택한다. 위쪽 이동 버튼으로 순서를, 방향 버튼으로 진행 방향을 바꾼다. 선택 응답 확인 후 **다음: 계획·실행**을 누른다. ZED 영상에 후보선과 선택한 용접선이 표시된다.
3. **③ 계획·실행**: **Settings**에서 조건을 입력하고 적용한다. 선택한 심의 순서·방향 요약을 확인하고 **계획 생성·검증**을 누른다. 모든 심의 접근·진입·용접·이탈 사전 계획과 실행 준비 조건이 충족되면 **작업 시작**이 활성화된다. Isaac에 중심선, 위빙 경로, 토치 축이 표시된다.

응답 수신만으로 단계가 자동 전환되지 않는다. 탭으로 이전 내용을 확인하는 것은 계획을 유지하며, 실제 대상·심·설정 변경은 재검증을 요구한다. 계획 요청·실행·중단 확인 중에는 편집과 새 요청을 잠근다. **작업 중단**은 모든 단계에서 사용할 수 있다. 경로 생성·계획 검사·실행 준비 상태를 별도로 표시한다.

검증된 3D 용접선을 세 단계의 ZED 영상 위에 투영한다. 후보선은 주황색, 선택한 선은 청록색이며 시작·끝점과 진행 방향을 표시한다. 방향 반전 시 시작·끝 표시도 바뀐다. `/welding/seam_overlay`는 해당 영상 시각의 카메라 보정 정보와 TF를 사용하며, 영상 시각·대상 revision·generation이 일치하지 않거나 연결이 끊기면 표시하지 않는다. 현재 시뮬레이션 fixture의 선은 등록된 CAD 접합선이며 자동 영상 추출 결과가 아니다. 대상 스케치를 용접 경로로 사용하지 않는다. 대상·심·설정·장면 변경, 카메라 헤드 이동, 스트림 변경, 시뮬레이션 reset은 기존 계획을 무효화한다. 중단 확인 전에는 다음 심이나 새 실행을 시작하지 않는다.

## Settings

용접 속도, 접근/이탈 거리, standoff, 작업각/진행각, 위빙 ON/OFF·전체 폭·주파수·좌우 체류 시간, 전류, XYZ 심의 기준 자세 선택을 제공한다. 기본값은 시뮬레이션 예시이며 사용자가 적용해야 한다. 전류는 가상 공정 상태에만 반영된다.

XYZ 좌표만 받은 경우 **XYZ 심의 기준 자세**에서 등록된 시험 프로파일을 명시적으로 선택하고 적용해야 한다. 기본은 선택 안 함이며, 임의의 표면 법선을 추정하지 않는다. 기준 방향은 H_STEEL_USD에서 선택한 용접부의 면과 진행 방향에 맞춰 등록한다. 추출기가 자세나 기준 방향을 제공하면 그 값을 우선 사용한다. 실제 작업물에는 그 형상에 맞는 기준 방향·자세가 필요하다.

UI는 mm, mm/s를 표시하고 ROS 계약은 m, m/s, s, deg, A를 사용한다. 위빙 폭은 양 끝 사이의 전체 폭이다. 체류 중에도 중심선 방향으로 전진한다. 작업각은 진행 접선축, 진행각은 음의 횡축 회전이며 입력 자세에 적용한다. 방향을 반전하면 각도의 기준축도 함께 반전한다. 시험 심에는 해당 접합부에 맞는 기준 자세를 함께 등록한다. 샘플의 관절 속도·가속도를 만족할 수 없으면 용접 조건을 임의로 바꾸지 않고 계획을 거부한다.

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

추출기는 `/welding/seams`에 직접 발행하지 않는다. 이 토픽은 검증·변환된 내부 결과다. 통합 시 `src/sketch_control/config/construct_weld_fixture.json`과 `weld_sim_profile.json`을 H_STEEL_USD 및 Hongik 컨택트 팁 TCP에 맞춰 갱신한다. 제공 TCP는 CAD 공칭값이며 실제 용접 토치 보정값으로 사용할 수 없다.

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

새 중앙 수평부재 용접선은 아래와 같이 초기 상태 및 실행 전 계획을 검증했다.
실제 이동 후 완료·중단·궤적 추종 검사는 별도 실행 검증 대상이다.


## 2026-10-19 중앙 수평부재 아래 플랜지–웹 용접 데모

- 구성: 완전 시뮬레이션 / 상용로봇 플랫폼 / 오른팔 / 용접. 왼팔 spray v2와
  오른팔 Hongik V4 용접 EOAT·레이저 센서를 유지한다.
- 시험체: 뿜칠 데모의 배치 `[-0.515, -1.9, 0] m`를 그대로 사용한다.
- 시험 심: `horizontal_lower_front_fillet`. 실제 수평부재 CAD의 +Y 웹 면과
  아래 플랜지 윗면을 교차시켜 만든 약 400 mm 선이다. World 기준 시작점은
  약 `[0.2, -1.496, 0.629] m`, 끝점은 `[-0.2, -1.496, 0.629] m`이다.
  양끝 연결부를 피하도록 부재 양끝에서 각각 100 mm를 남긴다.
  사용자 지정 CAD 시험 심이며 영상 인식 결과나 기존 CAD 용접 비드 추출 결과가 아니다.
- 토치: 웹/플랜지의 바깥쪽 이등분 방향에 대해 45° 접근을 사용하고, 공구 축
  주위로 90° 돌려 옆 기둥과의 본체 간섭을 피한다. 이 자세는 심 quaternion에
  들어 있으므로 Settings 작업각/진행각 보정은 각각 0°다.
- 초기 대기: TCP 약 `[0.2, -1.40266, 0.72234] m`, 용접 시작점 근처에서 대기한다.
  실제 측정은 `[0.199601, -1.402249, 0.721687] m`였고, 12 mm 이격의 기준
  진입점까지 119.835 mm였다. 아래 최종 15 mm 이격 설정에서는 약 117 mm다.
  왼팔 joint1=0.5 rad로 옆에 두고, 오른팔 joint1–6은
  `[-0.80300748, 1.12292951, 2.10950079, -2.55937129, 0.33304508, -1.37688653] rad`로 시작한다.
- ZED: `zed2i_link`의 CAD 장착 렌즈를 유지하고 head joint1=-0.05954050,
  joint2=0.97123631 rad로 아래쪽 접합부를 향한다. 자유 관찰 카메라가 아니다.

검증된 데모 설정은 `config/weld_right_horizontal_demo.json`에 저장했다.
속도 10 mm/s, 접근 120 mm, **이탈 20 mm**, 토치 끝 이격 **15 mm**,
위빙 전체 폭 4 mm·1 Hz, 좌우 체류 0초, 가상 전류 100 A다.
12 mm 이격의 위빙 경로와 120 mm 이탈은 공구/기둥 간섭으로 거부됐으며,
충돌 검사를 끄지 않고 위 설정으로 접근·진입·위빙·이탈 전체 계획을 통과했다.

재시작 후 측정 관절에서 전체 로봇 상태 충돌 검사 통과, 작업선 주변 ZED 광선
81개 가림 0개, 시험체 자체에 의한 작업선 표본 가림 0개를 확인했다.
실제 RGB의 용접선 끝점은 대략 픽셀 `(259,259)`–`(381,253)`에 위치한다.
공개 용접 target/selection/settings/plan 요청을 통한 plan-only 검사는 `READY`와
유효한 plan ID를 반환했다. 이동 명령 및 물리 용접기 출력은 실행하지 않았다.
회귀 검사: `tests/platform_runtime` 401 passed, 16 skipped (40 warnings).

증거는 `/home/minjea/robotics/output/weld-right-demo-20261019/`의
`selected-pose.json`, `verified-report.json`, `zed-view.png`, `plan-final.json`,
`settings.json`, `tests.log`에 보관한다. 초기 자세·심·카메라 변경은
이 상용 플랫폼 오른팔 완전 시뮬레이션 데모에만 적용한다.
