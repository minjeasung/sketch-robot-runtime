# 로봇 팔 선택: RB10 / RB20

관리 화면에서 **로봇 모델**을 선택하고 전체 시작한다. 프로세스가 실행 중이면
모델 변경을 거부한다. EOAT, ZED/D405, 스케치·다중 평면·도장/뿜칠 경로는 공통이다.

| 화면 | model_id | 역할 |
|---|---|---|
| RB10-1300E | `rb10_1300e_u` | 기존 기본 모델과 장착 유지 |
| RB20-1900ES | `rb20_1900es` | 번들 RB20 모델·메시 + 공통 EOAT 기준 TCP |

이 두 모델만 지원한다. 다른 제조사/기종의 자동 호환을 의미하지 않는다.
기존 `rb10_*.launch.py` 파일명은 호환성을 위해 유지하며 실제 모델은 `model_id`로 결정한다.

```bash
# PC별 config/sketch_runtime.env (Git 제외)
SKETCH_MODEL_ID=rb20_1900es
```

API: `POST /configuration`에 `model_id`, `profile`, `robot_ip`를 함께 전달한다.
실행기에서도 읽기 전용 `model_id`로 선택 모델을 기록하고 해당 관절 제한을 사용한다.

## 플랜지와 EOAT 기준

번들 URDF의 `link6 → tcp` 위치는 RB10에서 `[0, -0.1153, 0]`,
RB20에서 `[0, 0, 0.13]`이다. 기존 EOAT는 TCP −Y를 따라 바깥으로 뻗는다.
RB20에 동일한 조립체를 장착하는 **명목 방향**으로 TCP 축을 `Rx(-90°)` 회전하여
TCP −Y가 RB20 플랜지 +Z를 향하도록 한다. TCP +X는 플랜지 +X로 유지한다.
`arm_flange`는 RB20 원래 플랜지 좌표계를 별도로 표시한다.

6개 회전 관절의 위치·축·메시는 원본 RB20 모델을 그대로 사용한다.
TCP 이후 EOAT·롤러·D405의 상대 형상은 기존과 같다. FT 프레임의 회전 관계도
같은 조립체를 기준으로 맞추지만 실제 센서 축·부호·중력 보정은 교체 후 확인해야 한다.
이는 실물 볼트 체결 방향을 자동 측정한 결과가 아니다. 실제 장착의 축 회전과
RViz 형상을 대조하고 필요한 경우 모델의 장착 변환을 수정한 뒤 재보정한다.

따라서 작업 규칙은 두 모델 모두 TCP +Y = 평면 법선, 롤러 긴 축은 획과 수직이다.
현재 EOAT 이동 검증 모드에서는 F/T 보정을 사용하지 않는다.

## 모델별 보정과 충돌 설정

RB10은 기존 작업 폴더의 보정 파일을 사용한다. RB20은 다음 별도 파일을 사용한다.

```text
calibration/rb20_1900es/zed_d405_apriltag_calibration.json
calibration/rb20_1900es/d405_eyeinhand_charuco_calibration.json
```

필요한 pose 키는 각각 `T_world_zed_optical`, `T_d405_optical_to_tcp`이며 기존
보정 도구의 translation/rotation_xyzw 형식이다. 교체한 팔과 정규화된 TCP를 기준으로
보정해야 한다. RB10 파일을 자동 복사하지 않는다. RB20 실장비 perception은
별도 파일이 없거나 변환이 유효하지 않으면 기동을 거부한다. fake 프로필은 이 요구를 제외한다.
소스 배포에는 보정값을 포함하지 않으며 `export_runtime.py --calibration-only`로 별도 전달한다.

RB10에서 가르친 ready/view/calib 관절 자세는 RB20에서 사용하지 않는다.
스케치 접근은 현재 실측 관절 상태에서 계획한다. RB20의 고정 자세 이동이 필요하면
해당 팔에서 검증한 관절 자세를 별도로 등록해야 한다.
RB10 SRDF의 `Never` 충돌 제외는 RB20에 복사하지 않고 인접 링크·고정 EOAT 조립체의
제외만 유지한다. 실행기의 충돌 기준도 MoveIt과 동일한 모델별 SRDF를 사용한다.

## 확인

```bash
bash scripts/check_runtime.sh --model-id rb20_1900es
bash scripts/check_runtime.sh --real --model-id rb20_1900es
```

먼저 fake 프로필에서 RB20·EOAT·TCP 형상과 MoveIt 계획을 확인한다.
실제 RB20에서는 모델/IP, 장착, 보정, 충돌 환경을 확인한 뒤 작은 이동부터 검증한다.
모델 선택 기능의 구현·가상 검증과 실제 RB20의 시운전 완료는 구분한다.

2026-09-22 검증:

- 모델/API/뿜칠/D405 관련 118개 테스트 통과, sketch_control 빌드 통과.
- 기존 RB10 URDF 전체가 변경 전과 동일함을 확인(EOAT·FT 프레임 포함).
- 격리 ROS domain 226에서 fake RB20의 모델/EOAT 로딩, 충돌 없는 관절 상태,
  TCP·플랜지 FK, 충돌 검사 IK, MoveIt 관절 경로 계획 통과. 궤적은 실행하지 않음.
- RB10 fake 모델 로딩·FK도 확인. 실제 로봇에는 연결하지 않음.
- sketch_control 전체 회귀: 375개 통과, 기존 접촉 도장 테스트 10개 실패.
  실패 목록은 이전 검증과 같음.
- 가상 스택 SIGINT 종료 시 `librclcpp` executor 소멸 과정의 MoveIt 충돌을
  RB10과 RB20 양쪽에서 관측. RB20은 SIGKILL 정리가 필요했으며 종료 안정성은 별도 해결 대상.

## 참고한 구조

SNUCEM_Robot의 공통 제어기 재사용 + 명시적인 모델 선택 방식을 참고했다.
참고 저장소는 변경하지 않았다.

- [RB20 model.py](https://github.com/JongHyunSeo11/SNUCEM_Robot/blob/342b284be828a07c1afd266380087d18c157e998/linux/control/rb20_runtime/model.py)
- [RB20Controller](https://github.com/JongHyunSeo11/SNUCEM_Robot/blob/342b284be828a07c1afd266380087d18c157e998/linux/control/rb20_runtime/controller.py)

참고 저장소의 모델 ID는 `rb20_1900es_u`이고 이 프로젝트 번들은 `rb20_1900es`다.
참고 저장소가 보정한 shoulder 원점 `[0, -0.2545, 0]`은 이 번들에 이미 반영되어 있다.
키보드 직접 제어 대신 기존 MoveIt 충돌 계획과 스케치 실행 검증을 유지한다.
