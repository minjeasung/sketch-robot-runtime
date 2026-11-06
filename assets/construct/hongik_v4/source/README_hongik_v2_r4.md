# construct_robot_0528_hongik_v2 r4 (홍익대, 2026-10-13)

KIRO `construct_robot_0528.urdf` 를 바탕으로 **오른팔 툴만 실제 장착품으로 바꾼 URDF** 입니다. 몸체, 왼팔, 헤드, 관절, 링크 이름은 원본과 같습니다.
KIRO 9월 용접 로그에서 식별한 실제 구성(장착 방향, 토치 방향)을 반영했습니다. 그림: `hongik_v2_r4_tool.png`.

## 파일
| 파일 | 내용 |
|---|---|
| `construct_description/urdf_0528/construct_robot_0528_hongik_v2.urdf` | URDF (원본과 같은 패키지 구조) |
| `construct_description/meshes_0528/hongik_eoat_bracket1_torch_sensor.stl` | 툴 표시용 메시 (브라켓, 어댑터판·먼지판 5 mm, 센서·센서가드, 토치) |
| `construct_description/meshes_0528/hongik_eoat_bracket1_torch_sensor_collision.stl` | 툴 충돌용 메시 (표시용에서 CAD 와이어 토막만 뺌, 컨택트 팁 끝면은 막음) |
| `hongik_v2_r4_tool.png` | 플랜지 좌표계에서 본 툴, TCP·와이어 끝·센서 광학 원점 |

## 툴 구성
- **장착**: `hongik_flange`(RB 제어기 tcp 플랜지 규약) → `hongik_eoat_base_link` 를 z축으로 **−90°** 돌려 장착 (연구실과 같은 방향).
- **토치**: 클램프 안에서 곧은 몸통 축 둘레로 **180° 돌린** 방향. 토치 목이 센서 **반대쪽**으로 꺾임 (용접 자세에서 토치가 아래, 센서가 위).
  - 돌아간 부품은 토치(몸통, 목, 가스 노즐, 컨택트 팁)뿐입니다. 클램프, 브라켓 팔, 어댑터판·먼지판, 센서, 센서가드는 그대로입니다.
- 노즐·컨택트 팁 치수는 실제와 같습니다 (KIRO 확인).

## TCP (`right_manipulator_ee_point`)
- 부모 `right_manipulator_link_ee`. 기준점 = **컨택트 팁 끝면의 중심** (CAD 공칭). 와이어 끝이 아닙니다.
- 플랜지(RB tcp 규약) → TCP: **(−0.45, 237.86, 297.17) mm, rpy (0, −30, −90)°**.
- `right_manipulator_link_ee` → TCP: xyz (0.206216, 0.118535, −0.300173) m, rpy (−3.141585, 0.523594, −2.617990) rad.
- TCP 축: **+z = 노즐 축** (와이어 방향, 공작물 쪽), **+y = 토치 목이 꺾인 평면에 수직** (용접 때 심을 가로지름, 위빙 축), +x = y × z.
  플랜지 좌표계 기준: x = (0, −0.866, 0.5), y = (1, 0, 0), z = (0, 0.5, 0.866).
- **와이어 끝**: 가스 노즐 끝이 TCP 에서 +z 로 1.2 mm 앞에 있고 (컨택트 팁이 노즐 안으로 1.2 mm), 돌출 길이는 **노즐 끝에서** 잽니다.
  → 와이어 끝 = TCP + (1.2 + 돌출) mm × z_TCP. 돌출 20 mm 이면 TCP + 21.2 mm.

## 캘리브레이션과 보정 (합의 2026-10-13)
- 저희 패키지는 실행 중인 `/robot_description` (이 URDF) 의 **순수 운동학**으로 계획하고, 결과(용접 경로)를 그 모형 기준으로 드립니다.
- KIRO 는 와이어 팁 등으로 미리 캘리브레이션한 뒤 **저희 결과를 KIRO 쪽에서 보정해** 쓰십니다.
- 캘리브레이션 값을 이 URDF (`right_ee_point_joint` origin) 에 직접 넣으시면 저희 출력에도 그 값이 이미 반영됩니다.
  **두 방법 중 하나만** 써 주세요 (URDF 에 넣고 결과도 보정하면 두 번 보정됩니다).
- 참고: TCP 가 플랜지 축에서 238 mm 떨어져 있어, 플랜지 축 둘레 각도 오차 0.1° 가 TCP 에서 약 0.4 mm 가 됩니다.

## 설치
- `urdf_0528/construct_robot_0528_hongik_v2.urdf` 와 `meshes_0528/` 의 두 STL 을 `construct_description` 패키지의 같은 폴더에 넣습니다.
  (이전 판을 받으셨다면 같은 이름으로 바꿉니다.)
- CMakeLists 는 바꾸지 않아도 됩니다 (`meshes_0528`, `urdf_0528` 이 이미 설치 대상). ros2_control xacro 는 관절만 쓰므로 그대로 둡니다.

## MoveIt (SRDF)
- 원본의 `right_D435_link` 가 들어간 항목은 빼 주세요 (오른쪽 D435 는 이 URDF 에 없습니다).
- `hongik_eoat_base_link` ↔ `right_manipulator_link6` 충돌 검사를 꺼 주세요 (어댑터판이 플랜지에 붙어 있어 늘 닿음).
- 나머지 쌍은 MoveIt Setup Assistant 로 충돌 행렬을 다시 만드시는 것을 권합니다.
- 가스 노즐과 컨택트 팁은 `hongik_eoat_base_link` 메시의 일부입니다. 루트 패스에서는 노즐이 판에 거의 닿는 자세가 정상이므로
  (KIRO 확인), 시편을 MoveIt 장면에 넣어 검사하실 때는 노즐과 판의 접근을 고려해 주세요. 저희 패키지는 노즐·컨택트 팁은 판을
  파고들지 않는지만 보고, 그 밖의 부품은 여유 간격을 두고 검사합니다.

## 확인한 것
- `check_urdf` 통과. colcon 빌드 후 URDF 의 메시 경로 28 개가 모두 열립니다.
- 플랜지 → TCP 를 이 URDF 의 FK 로 다시 계산: (−0.45, 237.86, 297.17) mm, rpy (0, −30, −90)° (위 값과 같음).
- 저희 연구실 툴 모형 (같은 브라켓·센서·토치, 같은 장착·토치 방향) 과 툴 메시·TCP·센서 프레임이 일치합니다 (꼭짓점 차이 < 1e-5 mm).
- KIRO 9월 로그의 관절값으로 계산한 툴 위치가 로그에서 식별한 구성과 맞습니다 (9월 22일·15/16일 패스 1 의 와이어 끝, 센서가 토치 위).

## 참고
- 센서 측정 좌표계 `hongik_profiler_optical_frame` 은 URDF 에 없습니다. 저희 launch 가 센서 캘리브레이션 파일 값으로
  `hongik_laser_link` 아래에 정적 TF 로 발행합니다.
- 추가 프레임: `hongik_flange`, `hongik_eoat_base_link`, `hongik_laser_link` (센서 CAD 기준), `hongik_weld_link` (토치 CAD 기준, 토치와 함께 돌아감).
- `hongik_eoat_base_link` 질량 2.0 kg 은 연구실 payload 값이고 관성은 근사입니다. 제어기 payload 설정은 KIRO 설정을 쓰시면 됩니다.
- 출력 좌표 규약 문서 (갱신판) 는 소프트웨어 패키지와 함께 드립니다.
