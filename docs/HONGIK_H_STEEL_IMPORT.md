# 상용로봇 플랫폼 최신 구성 — Hongik V4 / H_STEEL_USD

2026-10-15 사용자 확정: **왼팔 내화뿜칠 spray v2 / 오른팔 Hongik V4 용접 EOAT + 레이저 센서**.
용접·뿜칠 작업체는 **H_STEEL_USD**이며, 이 구성을 최신 기본 사양으로 사용한다.
원본 보존, 체크섬 확인, 후보 URDF/충돌 모델 생성까지 수행했다.
**2026-10-15 확인 시 Isaac Sim·관련 ROS 프로세스와 관리 서버(8081)는 종료 상태다. 운영 `isaac_assets`는 아직 교체하지 않았다.**

| 용도 | 입력 | 보관 위치 |
| --- | --- | --- |
| 플랫폼·용접 EOAT | `hongik_urdf_v4_2026-10-13.zip` | `assets/construct/hongik_v4/source/` |
| 용접·뿜칠 작업체 | `h_steel_USD.zip` | `assets/construct/h_steel/source/` |

각 폴더의 상위 `source.json`에 ZIP과 내부 파일 SHA256을 기록했다.
Hongik 패키지의 `SHA256SUMS` 5개도 모두 일치한다. ZIP 명칭은 v4이지만
내부 이름은 `construct_robot_0528_hongik_v2.urdf`, 문서 버전은 v2 r4이다.

## 플랫폼과 용접 공구

이 패키지는 기존 Construct 설명 패키지에 덧붙이는 델타다. 새 URDF와 새 EOAT
표시/충돌 STL 두 개를 제공하고, 몸체·팔·헤드 메시 파일은 기존 패키지를 참조한다.
후보 생성기는 새 패키지의 파일을 우선 사용하고 나머지는 기존 메시로 해결한다.
14개 회전 관절의 origin/axis/limits 정의가 기존과 동일함을 대조했다.

확정 기본 구성은 **왼팔 spray v2 / 오른팔 Hongik V4 용접 EOAT + 레이저 센서**다.
오른팔의 기존 D435와 공구 형상은 제거되고 브라켓·토치·레이저 센서 형상이 들어온다.
레이저 센서의 CAD 프레임만 있으며 광학 교정 TF와 센서 데이터 드라이버는 제공되지 않는다.

| 항목 | 원본에 정의된 값 |
| --- | --- |
| 용접 공구 링크 | `hongik_eoat_base_link` |
| TCP 링크 | `right_manipulator_ee_point` |
| TCP 기준점 | 컨택트 팁 끝면 중심, CAD 공칭 |
| `right_manipulator_link_ee` → TCP 위치 | `(0.206216, 0.118535, -0.300173)` m |
| 같은 변환의 rpy | `(-3.141585, 0.523594, -2.617990)` rad |
| TCP +Z | 와이어/공작물 방향 |
| TCP +Y | 토치 목이 꺾인 평면에 수직, 위빙 기준축 |
| 공구 질량·관성 | 원본은 2 kg payload와 근사 관성. 실측 인증으로 해석하지 않음 |

원본 README에 따르면 노즐 끝은 컨택트 팁 TCP보다 +Z로 1.2 mm 앞이다.
노즐 기준 와이어 돌출을 20 mm로 설정한다면 와이어 끝은 TCP +21.2 mm다.
이것은 계산 예시이며 기본 돌출을 20 mm로 확정한 것은 아니다.
기존 실행기의 150 mm 시험 토치는 새 공구와 함께 사용하면 안 된다.
TCP 자체를 교정하는 방식과 출력 결과를 보정하는 방식을 중복 적용하지 않는다.

공구 메시와 TCP가 `right_manipulator_link6`에서 갈라지는 서로 다른 고정 가지에
있으므로 EOAT 추출기는 손목 아래 고정 하위 트리 전체를 찾되 손목 메시 자체는
제외한다. 이 구조를 반영한 회귀 테스트를 추가했다.

SRDF는 없는 D435 링크 참조를 제거하고 새 공구와 손목의 장착 인접쌍만 추가했다.
원본 새 URDF의 `testbed_link`는 시각 형상 없이 충돌 형상만 남아 있으므로 후보에서는
해당 시각/충돌 형상을 제거했다. 새 H형강 장면과 기존 testbed의 숨은 충돌이 겹치지
않게 하기 위한 처리다. 새로운 전체 자세의 자기충돌 검증은 실행 통합 시 수행해야 한다.

## H형강 작업체

- 원본 `Steel Structure.usdc`, default prim `/SteelMockup`.
- 단위 m, Z-up, 실제 외접 크기 약 **1.03 × 0.80 × 1.516 m**.
- 23개 메시, 1274개 정점, 2352개 삼각형. 기둥·보·플레이트·볼트·용접부가 포함된다.
- 원본 USD의 재질/참조를 유지하고 정적 triangle-mesh 충돌을 추가했다.
  전체를 convex hull로 메워 H형강의 빈 공간을 없애지 않는다.
- 동일 변환을 적용한 삼각형을 `h_steel_collision.json`에 추출하여 향후 MoveIt
  충돌 장면에 사용할 수 있게 했다. 아직 운영 MoveIt 장면에 발행하지 않았다.
- 후보 배치는 `xyz=(-0.515,-1.7,0)` m, 회전 없음이다. 기존 로봇 앞에 놓기 위한
  검토용 값이며 실제 설치 좌표·최종 도달성 검증값이 아니다.

## 생성 결과와 재현

후보 출력: `.runtime/imports/hongik_v4_hsteel/`.

- `construct.urdf`, `construct_control.urdf`, `construct.srdf`, controller/limits YAML
- `construct_manifest.json`, 좌우 EOAT STL/JSON과 equipment 메타데이터
- `h_steel.usda`, `h_steel_collision.json`, `h_steel_manifest.json`
- `h_steel_preview.png`: 추출한 실제 삼각형을 그린 기하 미리보기. Isaac 실행 캡처가 아님

```bash
.venv-api/bin/python scripts/prepare_construct_assets.py \
  --source-model hongik_v4 --output .runtime/imports/hongik_v4_hsteel
```

EOAT 프로필 생성 API:

```python
from pathlib import Path
from scripts.prepare_construct_runtime import prepare
root = Path.cwd()
candidate = root / '.runtime/imports/hongik_v4_hsteel'
prepare(root, candidate, urdf_path=candidate/'construct_control.urdf')
```

H형강 변환은 pxr가 설치된 Isaac Python으로 실행한다.

```bash
/home/minjea/robotics/.venvs/isaacsim-6.1/bin/python scripts/prepare_h_steel.py \
  --output .runtime/imports/hongik_v4_hsteel --translation -0.515 -1.7 0
```

검증: 신규 Hongik/H형강 테스트와 기존 Construct 자산·뿜칠·실행 프로필 테스트
**16개 통과**. USD 크기/삼각형·충돌 일치, 14축 정의 유지, 왼팔 spray v2 공구 유지,
새 오른팔 고정 가지 추출, SRDF 참조를 검사했다. 실제 로봇/용접 실행 시험은 하지 않았다.

## 실행 통합에 남은 작업

1. 후보 URDF로 articulated USD를 생성하고 새 용접 공구의 자기충돌/초기 자세를 확인한다.
2. 운영 USD와 MoveIt 장면 모두에 H형강을 같은 위치로 적용하고 예전 beam/support와
   testbed 충돌을 제거한다. 카메라 시야와 양팔 도달성을 확인한다.
3. 기존 dummy `WeldTorch` 생성/부착 검사를 native Hongik 공구 검사로 교체한다.
   새 TCP와 필요 시 명시한 와이어 돌출을 기준으로 경로를 생성한다.
4. 예전 작업체의 용접 fixture를 무효화한다. 새 H형강의 용접선 또는 실제 카메라에서
   지정한 용접선을 사용하고 과거 경로·장면 식별자를 재사용하지 않는다.
5. 해당 준비가 끝난 후 운영 자산을 전환하고 시스템을 재시작한다.

위 작업은 [구현 계획](superpowers/plans/2026-10-15-platform-eoat-ui.md)에 반영했다.
