# 플랫폼·EOAT 실행과 검증

## 상용 플랫폼 공구 장착 규칙 — 2026-10-21

상용 플랫폼은 빈 손목을 기준으로 선택한 팔에만 공정 EOAT를 장착한다.
오른팔·롤러도장은 오른팔 롤러와 왼팔 `none`, 왼팔·내화뿜칠은 왼팔 Spray v2와
오른팔 `none`으로 생성된다. 팔 또는 공정을 바꾸어도 비활성 팔에 이전 공구를
남기지 않는다. 비활성 팔의 공구 형상·충돌·질량·센서는 제거하고 공구 없는
플랜지 좌표계와 로봇 관절은 유지한다. 머리 ZED 카메라는 그대로 유지한다.

기존 양팔 공구 저장값은 서버 시작 시 선택한 팔 기준으로 정규화한다.
이미 적용했던 모델의 기록은 새 적용이 성공할 때까지 보존하므로 관리 화면에서
설정을 적용해야 실제 시뮬레이션 모델이 바뀐다. 관리 화면은 선택한 팔의 공구와
반대팔의 `EOAT 없음` 상태를 함께 표시한다.

## 도장 시작 직후 종료 수정 — 2026-10-21

상용 오른팔 도장 시뮬레이션의 `state_stalled`를 계측한 결과, Python
generation-2 GC가 0.2885초/0.2710초 실행되는 동안 같은 루프의 ROS 처리도
0.2897초/0.2721초 멈췄다. 상태 발행 간격은 최대 0.3125초로 기존
0.20초 lease를 초과했다. 물리 작업 스레드를 줄여도 같은 멈춤이 재현되어
해당 설정 변경은 적용하지 않았다.

`platform_simulation_app.py`는 초기화 완료 후 미사용 객체를 먼저 수거하고
Kit 초기화 객체 그래프를 `gc.freeze()`로 반복 검사에서 제외한다. 실행 중
새로 생기는 순환 참조는 계속 자동 수거하며, 정상/예외 종료 시 `gc.unfreeze()`로
복원한다. 초기화 객체의 순환 참조가 실행 중 불필요해지면 종료까지 메모리에
남을 수 있다. 안전 lease, 물리 주기, 힘 감시, 중단 및 전체 종료 정책은 유지한다.

회귀 테스트는 초기화 그래프의 검사 제외, 새 순환 참조의 수거, 예외 종료 후
복원을 별도 Python 프로세스에서 검증한다. 재실행한 오른팔 도장 런타임은
시작 후 217초까지 카메라/컨트롤러 준비 상태를 유지했고 hardware inhibit는
0회였다. 관측된 상태 발행 간격 최대값은 0.1682초였다. Python 회귀는
2,065 passed / 20 skipped다. 이 검증은 시작 직후 종료 현상에 대한
시험이다. 아래의 실제 도장 획·힘 유지·중단 검증 상태를 완료로 바꾸지 않는다.

이 문서는 2026-10-15 플랫폼 전환 구현의 사용 흐름과 검증 계약을 설명한다.
코드·단위 테스트 통과와 Isaac/실카메라 통합 실행 결과는 별도로 기록한다.
코드 구현 완료, 라이브 검증 일부 미완료 (live verification incomplete)이다. 18개 조합의 실행과
양팔/단일팔 롤러 힘 유지·중단을 완료로 판정하지 않는다.

## 오른팔 단독 GPU 재검증 — 2026-10-16 후속

기존 `robot-system` 평가와 후속 pytest 작업이 끝난 뒤 오른팔을 재실행했다.
정상 시험 시작부터 실패까지 5초 간격 GPU 기록 132개에서 다른 계산 작업은
관측되지 않았다. **단독 GPU에서도 정상 두 획 전체 실행은 미통과**다.

- 첫 획: 9.03 sim s, 반력 18–32 N 비율 100%, 평균 23.68 N으로 힘 판정 통과.
- 두 번째 획: 정착·tare·접촉·가압을 통과했지만 실제 도장 경로를 보내기 직전
  `PAINT PROCESS 11 PAINT:PAINT_NOMINAL_MEASURED_STATE_STALE`로 ABORT.
- 시작점 snapshot 로그의 measured receipt age는 약 13 ms였고, 약 10 ms 뒤
  재검사에서 거부됐다. 마지막 simulator 샘플 시각 273.624012996 s는 ABORT
  이벤트 273.614012995 s보다 10 ms 앞선다. 비동기 clock/source 도착 순서가
  의심되지만 거부한 joint-state의 정확한 age는 기록되지 않아 원인을 확정하지 않는다.
- 접촉 중 중단 시험은 정상 실패 약 48초 뒤 다른 계정 `sungboo`의 GPU 작업이
  새로 관측되어 접촉 전에 소유 시험을 종료했다. loaded cancel 증거는 미완료다.

이번 재검증은 제어 코드를 변경하지 않았다. 이전 GPU 공유 설명만으로 오른팔
실패를 설명할 수 없으며, 다음 조치는 PAINT 명목 시작점 재검사의 source/receipt/
clock 시각을 분리 기록하고 같은 freshness 기준에서 동기화 경계를 확인하는 것이다.
원자료: [오른팔 재검증 요약](../.runtime/platform/revalidation-20261016-right-recheck-summary.json),
[정상 실행](../.runtime/platform/matrix-paint-20261016-right-recheck2/commercial-right-paint/pixel-workflow-report.json),
[GPU 관측](../.runtime/platform/revalidation-20261016-right-recheck-gpu.jsonl).
이번 소유 Isaac/ROS와 GPU 모니터는 종료했고 다른 계정·프로젝트 작업은 변경하지 않았다.

## 최종 체크포인트 — 2026-10-16

| 롤러 구성 | 정상 두 획·감압·후퇴 | 접촉 중 중단 |
| --- | --- | --- |
| 상용 왼팔 | 통과: 6.28/6.26 sim s, 힘 범위 100%/100% | 통과: 힘 명령 차단 24.12 ms |
| RB10 | 최신 TGS 64/1에서 통과: 8.30/8.29 sim s, 100%/99.64% | 통과: 차단 23.65 ms, 정지 확인 4.768초 |
| 상용 오른팔 | 최신 단독 GPU 시험 미통과: 첫 획 통과 후 두 번째 PAINT 진입의 명목 상태 시각 검사에서 거부 | 접촉 중 중단 측정 미완료 |

오른팔 마지막 정상 시험 도중 13:09:45에 다른 계정 `sungboo`의 Isaac 작업이
시작됐으며 같은 시점에 별도의 같은 계정 환경 캡처 작업도 GPU에 관측됐다.
곧이어 `measured state exceeded 0.20 s lease`로 자동 중단됐다. 시간상 중첩은
확인됐지만 모든 이전 실패를 GPU 공유 탓으로 돌리지 않는다. 다음 새 세션도
접촉 전 inhibit로 종료되어 중단 시험 증거로 사용하지 않는다. 다른 작업은
종료하지 않았으며 이번 검증이 시작한 Isaac/ROS만 정리했다.
오른팔의 남은 정상·중단 시험에는 GPU를 단독으로 사용할 수 있는 구간이 필요하다.

최종 행별 상태: [검증 요약](../.runtime/platform/verification-summary.json),
[수정·재시험 원자료 목록](../.runtime/platform/revalidation-20261016-fixes-summary.json),
[GPU 기록](../.runtime/platform/revalidation-20261016-fixes-observations.jsonl).
일부 시뮬레이션의 실제 카메라 9조합과 물리 로봇 실행은 여전히 미검증이다.

최종 주 작업 폴더 회귀는 Python **1,977 passed / 19 skipped**, C++ **13 passed**다.
`pxr`가 있는 Isaac 환경에서 USD solver 검증 **15 passed**를 별도로 확인했다.
관리 UI 8081은 HTTP 200이며 이번 시험의 Isaac/ROS는 종료된 상태다.

## 후속 수정·재검증 — 2026-10-16

기존 실패 원인을 분리하여 아래와 같이 수정했다. 이전 실패 기록은 뒤 절에 남긴다.

- D405 정면화는 기존 작업면 좌표축을 유지하고 실제 카메라 투영 사각형 안에
  포함되는 직사각형을 계산한다. 오른팔 기록의 작업영역 폭은 0.11365 m에서
  0.20358 m로 복원되며, 0.08 m inset·평면 품질·충돌 조건은 유지한다.
- PhysX TGS의 articulation 정지 속도 문제에 대해 기존 physics scene에
  `enableExternalForcesEveryIteration=true`를 적용한다. 보고 속도를 버리거나
  측정 정지 기준을 완화하지 않는다. 적용값은 `simulation-solver-contract.json`에 남긴다.
- 플랫폼 롤러 실행은 public abort를 guard/reference에서 직접 받아 즉시 0을 보낸다.
  재시작까지 중단을 유지하는 옵션은 플랫폼 simulation/mixed paint에만 활성화하며,
  기존 물리 실행의 기본 reset 동작은 유지한다. 시험 도구는 접촉 전에 abort 발행자
  연결을 준비하고, 실제 요청 전후 session/revision 유효성을 계속 검사한다.
- 힘 가압 후 PAINT 진입 시 측정 관절을 새 명목값으로 사용하면 이미 적용된
  admittance 보정이 중복됐다. 최신 JTC reference를 시작점으로 사용하고 기존
  feedback·시각·경로 시작 연속성 검사를 함께 적용한다. 명목 reference 변화는
  왼팔 실측에서 약 0.0017 rad에서 0.00000494 rad로 줄었다.
- 두 번째 획 전 tare는 기존 5초 sampling 제한 안에서 50개 실제 상태 샘플의
  0.0005 rad 정지 조건을 기다린다. 100개 baseline, 별도 25개 검증 샘플,
  1초 verification 제한 및 모든 입력·제어 interlock은 유지한다.

왼팔 접촉 중 중단은 새 세션에서 통과했다. 힘 명령 차단 0.024124초,
관측창 최대 관절 속도 0.007594 rad/s, 위치 변화 0.000560 rad로 기존 기준을
만족했다. 이 중단은 힘 목표/가상 drive 차단이며 접촉 하중의 즉각 소멸이나
자동 후퇴를 뜻하지 않는다. 원자료는
[접촉 중 중단 보고서](../.runtime/platform/matrix-paint-20261016-handoff-cancel/commercial-left-paint/pixel-workflow-report.json)다.

최신 tare를 포함한 왼팔 정상 실행은 두 획과 감압·후퇴를 통과했다.
판정 구간은 6.28/6.26 sim s, 18–32 N 비율은 모두 100%다. RB10도
8.26/8.31 sim s, 99.879/99.880%로 전체 실행을 통과했다. 원자료는
`matrix-paint-20261016-tare/{commercial-left-paint,rb10-single-paint}/pixel-workflow-report.json`이다.

오른팔은 동일 묶음에서 두 획·감압·후퇴를 완료했고 힘 범위·시간도 충족했지만,
두 번째 획의 단일 수신 간격 0.200164초가 기존 0.20초를 넘어 미통과했다.
일치하는 simulator 발행 간격은 0.110687초이며 추가 관측 지연이 확인됐다.
매 callback 뒤 동일 XML을 반복 해석하던 관측 부하를 줄이기 위해 정확히 같은
XML 문자열에 대한 hardware 검증만 최대 4개 캐시한다. 파일은 매번 읽고
session/revision/lease 검사도 매번 수행한다. 단일 지연의 원인이 완전히
입증됐다는 뜻은 아니며 원래 실패를 유지하고 새 세션에서 재시험한다.

오른팔 관측 부하 수정 후 재시험은 첫 획을 완료했으나 두 번째 tare의 TCP
정착 오차 3.005924 mm가 3 mm를 넘어 거부됐다. 초기 오차는 2.967 mm였고
잠깐 범위를 벗어난 뒤 감소 중이었다. tare 시작의 원래 2초 도착 대기 안에서만
위치 오차의 재정착을 허용하고 전체 quiet 시간을 다시 시작하도록 수정했다.
힘·접촉·TF·자세·clearance 검사, 이후 tare 단계의 즉시 거부와 3 mm 기준은 유지한다.
이 수정의 전체 회귀는 1,958 passed / 19 skipped이며 최신 라이브 재검증은 남아 있다.

별도 오른팔 중단 시험은 D405 계획 자체는 성공했지만 최종 관절 상태가
0.20초 freshness를 넘어서 실행 전 거부됐다. 접촉 중 중단을 측정한 것으로
집계하지 않는다. RB10 중단은 힘 명령 차단 24.1 ms를 통과했으나 보고 관절
속도 0.017956 rad/s(위치 기반 0.000155 rad/s)가 기준을 넘어 미통과다.
원자료는 `matrix-paint-20261016-tare-cancel`에 보존한다.

C++ 실제 tare 회귀 4개와 기존 계약 9개도 주 작업 폴더에서 통과했다.

후속 RB10 롤러 시뮬레이션/일부 시뮬레이션은 TGS 위치 반복 64회를 유지하고
속도 반복을 8회에서 1회로 변경했다. 설치된 PhysX 110.3.2의 기본 속도 반복과
4회 초과 TGS 동작 변경 경고를 근거로 한 모델별 설정이며 다른 구성은 64/8이다.
적용한 articulation별 값을 `simulation-solver-contract.json`에 저장한다.
새 설정의 RB10 정상 두 획은 8.30/8.29 sim s, 힘 범위 충족률 100%/99.64%로
통과했다. 별도 접촉 중 중단도 힘 명령 차단 23.65 ms, 관측 최대 속도
0.009600 rad/s, 위치 변화 0.00000381 rad로 통과했다. 정지 확인은 요청 후
4.768초로 기존 5초 제한 안이다. 이는 보고 속도와 위치 기반 속도가 완전히
같다는 뜻은 아니다. 원자료는 `matrix-paint-20261016-final/rb10-single-paint`와
`matrix-paint-20261016-final-cancel/rb10-single-paint`에 있다.

D405 계획 후 상태가 오래되면 원래 경로·seed·scene·token을 유지한 채 최대
1초 steady timer로 새 관절 수신을 기다린다. 새 receipt와 전진한 source stamp,
양쪽 0.20초 freshness를 확인한 뒤 기존 전체 실행 조건을 재검사한다.
중단·컨텍스트 변경·시간초과·관절 이동은 그대로 거부한다. 실제 오른팔 시험에서
208.5 ms의 오래된 상태를 거부한 후 20.8 ms 대기해 새 상태로 실행했다.
해당 시험은 이후 긴 접근이 probe의 720초 관찰 제한을 넘어 종료됐다.
후속 시험은 `--workflow-time-scale 12`로 관찰 시간만 늘리며 로봇 속도·안전
타임아웃·힘·정지 판정은 바꾸지 않는다.

## 재검증 — 2026-10-16

GPU 사용자를 5초마다 기록하며 기존 실패 항목을 재실행했다. 아래 첫 오른팔 시험,
두 번의 RB10 정상 도장, 왼팔 접촉 중 중단 동안 기록상 다른 계정의 GPU 작업은
없었다. 따라서 모든 미완료 원인을 GPU 공유로 설명할 수 없다.

| 시험 | 실제 결과 | 판정 |
| --- | --- | --- |
| 오른팔 기본 ROI 0.3 | 첫 D405 측정·도착 성공. 후속 작업영역 실측 폭 0.11365 m, 길이 0.23406 m. 측정 자세 생성의 양쪽 0.08 m inset을 만족하지 못함 | 도장 전 거부. 로그·영역 꼭짓점 확보 후 소유 시험 종료 |
| RB10 기본/확대 ROI 0.3/0.4 | 두 실행 모두 두 획·감압·후퇴·완료. 확대 시험 두 획의 반력은 100%가 18–32 N, 평균 20.82/21.49 N | 각 판정 구간 3.81/3.82 sim s로 최소 5 s 미달. 힘 유지 통과로 집계하지 않음 |
| 왼팔 접촉 중 중단 | PAINT에서 ABORT, 힘 차단, hardware inhibit 응답 확인 | 미통과: 보고 관절 속도 최대 0.06794 rad/s > 0.01; 힘 차단 확인 0.10643 s > 0.10 |
| 오른팔 확대 ROI 0.5 | 충돌 검사를 통과한 D405 접근 실행 | 다른 계정의 Isaac 재시작을 관측해 소유 시험 종료. 단독 실행 증거로 사용하지 않음 |

왼팔 중단에서 위치 변화는 관측창 내 약 0.0000069 rad였지만 Isaac 보고 속도와
일치하지 않았다. 위치만 사용해 정지로 판정하거나 속도·0.10초 차단 기준을
완화하지 않는다. 추가 구동/계측 원인 분석이 필요하다. RB10 중단은 정상 힘 유지
판정이 통과하지 않아 이번 순차 시험에서 시작하지 않았다.

대상 ROI를 넓혀도 D405 화면에 그리는 선의 상대 길이는 그대로이므로 RB10
관측 시간이 늘지 않았다. 시험 도구에 `--stroke-inset .15`를 추가했다. 이는 기존
작업영역(화면 10–90%) 안에서 수동 스케치 선을 15–85%로 길게 지정한다.
기본값은 기존 25–75%이며 로봇 제어·힘 판정 조건은 변경하지 않는다. 긴 선의
실제 실행은 아직 미검증이다. 수정 후 주 작업 폴더의 전체 Python 회귀는
1,858 passed / 18 skipped다.

원자료: [재검증 요약](../.runtime/platform/revalidation-20261016-summary.json),
[GPU·준비 상태 기록](../.runtime/platform/revalidation-20261016-observations.jsonl),
[왼팔 중단](../.runtime/platform/matrix-paint-left-cancel-20261016/commercial-left-paint/pixel-workflow-report.json),
[RB10 두 획](../.runtime/platform/matrix-paint-20261016-area/rb10-single-paint/pixel-workflow-report.json),
[오른팔 영역 진단](../.runtime/platform/matrix-paint-20261016-quiet/commercial-right-paint/revalidation-diagnostic.json).
소유한 Isaac/ROS는 종료했다. 다른 계정의 작업은 변경하지 않았다.

## 현재 확인된 라이브 증거 — 2026-10-15

코드 지원, 관절 인프라 검증, 전체 공정 실행을 구분한다. 아래 값은 실제 저장된
관측 결과이며 전체 18개 조합 완료를 뜻하지 않는다.

| 범위 | 확인한 결과 | 남은 확인 |
| --- | --- | --- |
| simulation 9개 구성의 관절 인프라 | 각 구성의 초기 상태, 이동·복귀, 취소 후 실측 정지, 비활성 팔 유지 통과 | 새 충돌 모델에서 공정별 전체 동작 재검증 |
| 세 구성의 용접 | 실제 H형강 CAD `IfcFastener_WELD_004`, 상용 양팔 0°/15°, RB10 60°/0°에서 접근·위빙·이탈 및 실측 추종 통과 | 공정 중 별도 취소, 다른 작업선/설정 |
| 세 구성의 내화뿜칠 | Isaac ZED 영상의 대상 지정 → 영역 → 스케치 경로 → 전체 실행·실측 추종 통과 | 다른 영역·경로, 별도 공정 중 취소, 실제 출력 |
| 롤러도장 | 상용 왼팔: ZED/D405 → 두 PAINT 획 → 감압·이탈 통과; 각 획 5.60/5.62 sim s, 실측 반력 100%가 18–32 N | 오른팔·RB10 두 획 및 각 구성의 별도 접촉 중 중단 |
| 실제 8081 관리/스케치 | 구성 적용과 PID/session/revision 준비, 기본/고급/모바일 화면·ZED 표시 확인 | 모든 구성의 UI 작업 실행 |
| mixed 9개 구성·물리 장비 | 코드·회귀 검증 | 실제 카메라 미연결로 `not_run`; 물리 로봇은 움직이지 않음 |
| 소프트웨어 회귀 | 주 작업 폴더 Python 1,852 passed / 18 skipped, JS 140 passed; C++ 9 passed | 이후 변경은 집중 회귀로 별도 기록 |

상용 오른팔 용접의 첫 통과 실행에서는 속도 0.0099955 m/s(요청 0.01 m/s),
위빙 폭 4.0919 mm(요청 4 mm), 약 1 Hz, 최대 위치 오차 0.9854 mm를 관측했다.
양팔 후속 통과 원자료는 `matrix-noncontact-final`에 있다. RB10은 최대 위치 오차
0.443 mm, 위빙 폭 3.9634 mm, 약 1 Hz를 관측했다(`matrix-process-retest`). CAD fixture 기반
시뮬레이션이며 실제 용접 출력이나 실카메라 용접선 인식 검증이 아니다.

관절 정지 probe는 메시지 측정 stamp로 계산한 속도와 실측 velocity를 대조한다.
취소 terminal 이후 기존 5초 제한 내에서 0.6초 관측창을 검사하고,
최소 0.5초 실측 기간·0.01 rad/s·0.001 rad 기준을 유지한다. 정상 이탈과
접촉 중 강제 중단은 다른 검증 항목이며, 관절 인프라 취소를 힘 제어 중단의
증거로 대신하지 않는다.

RB10 원본의 10 N·m effort 제한은 모델 중력 유지 토크(어깨 약 30, 팔꿈치 약
59 N·m)보다 작았다. simulation/mixed의 가상 로봇에만 기존 RB20의 2,000 N·m
force-drive 모델을 사용한다. 실제 RB10 토크 정격을 뜻하지 않으며 원본 URDF와
자기 충돌 규칙은 유지한다. 적용값·출처는 `simulation-drive-contract.json`에 남긴다.

Isaac 6.1 importer는 기존 URDF collision mesh에 지정한 decomposition 옵션을
적용하지 않아 일부 오목한 링크가 단일 convex hull로 남았다. 생성기에서 각
충돌 메시의 convex decomposition을 명시하며 원본 형상·collision enablement·
SRDF의 개별 제외쌍을 유지한다. `robot-collision-contract.json`에 설정과 메시
목록을 기록하고 새 asset revision으로 재생성한다. USD 속성 검증과 실제 물리
동작 검증은 별도로 기록한다.

라이브 원자료는 [최근 비접촉 공정](../.runtime/platform/matrix-noncontact-final/),
[뿜칠·RB10 용접](../.runtime/platform/matrix-process-retest/),
[오른팔 뿜칠](../.runtime/platform/matrix-right-spray-preserved/),
[관절 재시험](../.runtime/platform/matrix-retry/),
[왼팔 롤러 통과](../.runtime/platform/matrix-paint-steady-final/),
[이전 롤러 진단](../.runtime/platform/matrix-paint-final/),
[API 적용](../.runtime/platform/live-api-apply.json)에 보존한다.
[18행 공정 검증 요약](../.runtime/platform/verification-summary.json)은 각 통과의 원자료를
연결한다. 로컬 런타임 산출물은 소스 저장소와 별도로 보존해야 한다. 장비 없는 mixed
항목을 가상 입력으로 대체해 완료 처리하지 않는다.

## 이전 롤러 진단과 재검증 조건 — 2026-10-15

상용 왼팔의 정상 두 획은 `matrix-paint-steady-final`에서 통과했다. 이후 별도
중단 시험(`matrix-paint-left-cancel`)은 PAINT에 들어가기 전 F/T stale 안전 정지로
끝났으므로 접촉 중 사용자 중단의 통과 증거가 아니다. 당시 filtered wrench age
0.209665 s를 관측했으며 기존 0.20 s 제한을 유지한다. 해당 구간에서 simulator 발행은 계속됐지만 수신 측의 측정 step이 갱신되지
않아, 단일 긴 렌더 호출보다 전달/수신 스케줄링 지연을 의심할 근거가 있다.
같은 시점에 다른 계정의
Isaac 작업 3개가 같은 GPU를 사용 중이었다. 동시 실행을 확인한 사실과 모든
실패의 원인을 그 작업으로 확정하는 것은 다르다.

오른팔 `matrix-paint-remaining`은 첫 D405 접근 중 진단 도구의 360초 대기 제한에
걸렸다. RB10은 첫 획 이후 두 번째 접촉 탐색에서 준비 상태 검사에 실패했다.
`matrix-paint-right-preserved`도 관절 probe 시작 전 준비 상태 검사에 실패했다.
이 두 준비 상태 실패는 당시 오류가 원인 필드를 구분하지 않아 원인을 확정할
수 없다. 이후 진단은 실제 거부된 준비 marker와 검사 시각을 저장하고,
`ready=false`와 session/revision 불일치를 구분한다.

실패 정리 중 `/motion_abort`가 일반 준비 상태 검사에 다시 막히던 진단 도구
문제를 수정했다. 중단은 살아 있는 동일 simulator session/revision을 재확인하고
최대 2초의 수신자 대기 후 발행하며, 새 session으로 대체되면 거부한다.
카메라 준비 검사는 메시지 도착 순서가 섞여도 동일 촬영 시각·frame·크기의 네
메시지를 조립하도록 보완했다. 카메라별 최대 16개, 미완성 보관 1초이며 기존
준비 lease 1초를 유지한다. 이 수정이 과거 준비 실패의 원인이었다고 단정하지 않는다.

다음 실험은 GPU/CPU 여유를 확보한 환경에서 오른팔·RB10의 정상 두 획과
각 구성의 별도 접촉 중 중단이다. 오른팔 긴 D405 경로의 진단에는 기존 옵션
`--workflow-time-scale 6`을 사용할 수 있다. 이는 긴 workflow 대기시간만 늘리며
센서 신선도·힘·충돌·정지 판정은 변경하지 않는다. 이번에 소유한 Isaac/ROS는
종료했으며 다른 계정의 프로세스는 변경하지 않았다.

## 실행 선택

| 선택 | 동작 |
| --- | --- |
| 시뮬레이션 | Isaac 로봇과 Isaac 카메라 |
| 일부 시뮬레이션 | 실제 카메라의 측정 입력과 Isaac 로봇 |
| 실환경 | 고급 설정의 기존 실제 로봇·카메라 실행 경로; 기본 설정 적용에서는 차단 |

상용로봇 플랫폼은 왼팔/오른팔 중 한 팔을 실행한다. RB10 베이스는 단일 팔이다.
공정은 내화뿜칠(`spray`), 용접(`weld`), 롤러도장(`paint`)이며 선택 공정에 맞는
EOAT를 해당 팔에 장착한다. 비활성 팔의 장착 상태를 유지하며 양팔 같은 공구도
허용한다. 기본 상용 구성은 왼팔 spray v2, 오른팔 Hongik v4 토치·레이저 형상이다.
레이저 형상과 프레임 포함을 실제 레이저 데이터 수신 완료로 표시하지 않는다.
기존 RB20 도장 프로필은 고급 설정의 별도 모델로 유지한다.

설정 변경은 실행 중 직접 공구를 바꾸는 동작이 아니다. 작업을 종료하고 구성을
적용하면 기존 프로세스를 정리하고, 불변 자산 묶음을 생성·검증한 다음 선택된
구성을 시작한다. 생성/시작 실패 때에는 적용 상태와 오류를 확인하고 다시 적용한다.
실제 적용 구성이 준비되기 전에는 대기 설정을 실행 가능 상태로 취급하지 않는다.
공구·작업면·보정 revision이 바뀌면 기존 경로를 다시 선택/생성해야 한다.

실환경 상용플랫폼과 새 실환경 용접 드라이버는 이 변경의 지원 범위 밖이다.
`dry_run` 또는 fake 제어기를 시뮬레이션 모드로 간주하지 않는다.

## 빌드와 8081 관리 화면 시작

저장소 루트에서 ROS 2 Jazzy, Isaac Sim 6.1 설치와 원본 자산을 준비한다.
분리 worktree에서는 Isaac 경로를 명시한다. 아래 설치 경로는 현재 작업 환경의
예이며 다른 설치를 사용하면 실제 경로로 바꾼다.

```bash
export ISAACSIM_VENV=/home/minjea/robotics/.venvs/isaacsim-6.1
bash scripts/setup_system_api.sh
bash scripts/build_platform_sim.sh
source scripts/platform_ros_env.sh
.venv-api/bin/python -m sketch_control.system_api --workspace "$PWD" --host 127.0.0.1 --port 8081
```

빌드는 `.runtime/platform/build`, `install`, `log`에 통합 ROS overlay를 만든다.
ROS의 system Python에 필요한 mesh decoder는 공통 버전 목록
`config/platform_ros.requirements.txt`에서 `.runtime/platform/python`으로 설치한다.
API 가상환경의 NumPy/SciPy를 ROS 경로에 넣지 않는다. 기존 overlay에서 이 의존성만
보완하려면 `bash scripts/setup_platform_python.sh`를 실행한 뒤
`source scripts/platform_ros_env.sh`로 환경을 다시 읽고 스택을 재시작한다.
기존 운영 `isaac_assets`는 덮어쓰지 않는다. 새 적용 자산은
`.runtime/platform/assets/<revision-instance>/`에 저장하며 각 묶음에는
URDF/SRDF/USD, 공구 프로필, controller 설정, 보정/장면 파일과 생성 파일 해시가
함께 들어 있다. `configuration.json`은 `.runtime/platform/`에 저장된다.
`setup_system_api.sh`는 API 가상환경이 이미 준비된 경우 다시 실행할 필요가 없다.

관리 화면은 <http://127.0.0.1:8081/>, 기존 스케치 화면은
<http://127.0.0.1:8081/sketch/>이다. API 서버 시작만으로 로봇을 움직이지 않는다.
기본 화면에서 실행 모드 → 플랫폼 → 작업 팔 → 공정을 선택하고 저장한 뒤
설정 적용/재시작을 실행한다. 적용은 생성 → simulator/ROS 시작 → 현재 구성의
실측 준비 확인까지 기다린다. 고급 설정에 장치 ID, Outpost 주소, 보정 경로와
로그가 있다. 다른 브라우저가 먼저 저장한 경우 최신 revision을 다시 불러온다.

기본 설정의 **실환경 RB10 적용도 현재 차단**된다. 기존 RB10 실환경 작업은
시스템을 종료한 뒤 고급 설정의 기존 `work` 또는 `spray_motion_test` 프로필을
사용한다. 이 경로의 물리 장비 확인·실행 인터록은 그대로 유지된다. 실환경
선택을 simulator 자산 생성으로 우회하거나 자동 공구 교체 완료로 표시하지 않는다.
기존 RB20은 고급 설정에서 별도 모델로 유지한다.

관리 API는 `GET/POST /runtime/configuration`, `POST /runtime/apply`를 제공한다.
지원 조합은 configuration 응답의 `capabilities`에 포함된다. 저장/적용 요청에는 현재
`expected_revision`이 필요하다. `supported`는 코드 경로 지원 상태이며
`verification=not_run`과 별개다. 관리 서버를 원격 공개하려면 기존
`SKETCH_SUPERVISOR_API_TOKEN` 설정이 필요하다.

이미 생성된 묶음을 관리 UI와 별도로 진단할 때만 다음 stack 진입점을 사용한다.
관리 UI가 실행 중인 동일 stack에 겹쳐 시작하지 않는다.

```bash
bash scripts/run_platform_stack.sh --assets /absolute/path/to/bundle
```

이 명령은 bundle 선택을 읽어 Isaac, controller/MoveIt, 인식기와 rosbridge를
함께 소유한다. rosbridge 기본 포트는 9090이며 `--rosbridge-port` 또는
`SKETCH_ROSBRIDGE_PORT`로 바꿀 수 있다. 이미 사용 중인 포트는 거부하고 해당
프로세스를 강제 종료하지 않는다. Ctrl-C 또는 관리 화면 종료는 소유한 자식
프로세스 그룹을 정리한다. 준비 실패 시 묶음의 `simulation.log`, `ros.log`,
`rosbridge.log`와 관리 화면 오류를 확인하고 원인을 수정한 뒤 다시 적용한다.
오래된 경로·heartbeat 파일만으로 재시작 성공을 판단하지 않는다.

## 기본 장치·대상과 용접 fixture

기본 상용 장착은 왼팔 spray v2, 오른팔 Hongik v4 토치와 레이저 CAD 형상이다.
공정 변경은 선택한 팔만 바꾸며 비활성 팔 공구를 유지한다. 롤러 선택은 기존
AFT200/롤러/D405 조립체를 사용한다. 실물 장착이나 실측 TCP 보정 완료를 뜻하지 않는다.
시뮬레이션은 Isaac 렌더 RGB-D를 공통 ZED/D405 topic으로 보낸다. 물리 SDK 장치나
실물 레이저 수신을 구현한 것으로 표시하지 않는다. 일부 시뮬레이션은 지정한
Outpost 실제 ZED와, 도장의 경우 실제 D405만 사용한다.

시뮬레이션 spray/weld 대상은 원본 H_STEEL_USD와 같은 좌표의 삼각형 충돌 메시다.
모든 조합에서 Isaac과 MoveIt의 바닥도 일치시키며 MoveIt 적용 응답을 기다린다.
용접 fixture는 명명된 `IfcFastener_WELD` CAD 면에서만 생성한다. 노출된 직사각형
면의 끝단 중앙을 연결하고 그 면의 외향 삼각형 법선을 사용한다. 생성된 ZED 카메라의 optical 원점으로 향하는 시선이 다른 메시로 가려지면
제외하며, 면 방향이
모호하면 명시적으로 거부한다. 샘플 간격은 최대 20 mm이고 TCP/standoff 오프셋을
여기서 더하지 않는다. 실제 용접선 인식 결과가 아니며 mixed에서 사용하지 않는다.
`weld_fixture.json`에는 원본 USD/충돌 기하 해시, 면 triangle ID, 선택 팔과
제외 사유가 기록된다. 개발용 **미적용 새 생성 디렉터리**의 simulation-weld
fixture를 CPU에서 생성할 때는 다음을 사용한다. 이미 적용된 불변 자산 디렉터리를
수정하지 않으며, 운영 변경은 관리 UI의 재생성/적용을 거친다.

```bash
.venv-api/bin/python scripts/platform_weld_fixture.py --assets /absolute/path/to/unapplied-simulation-weld-bundle
```

## 일부 시뮬레이션의 카메라와 작업면

실제 카메라 ID, serial, 입력 source session과 보정 revision을 확인한다. 물리적
카메라 좌표를 선택한 가상 로봇의 base frame으로 정합한 보정이 필요하다.
고정 거치 D405에는 물리적 고정 자세를 사용한다. 가상 손목을 움직여 실제 카메라
자세를 갱신하지 않는다. 실제 eye-in-hand 사용에는 실제 관절/시간에 대응하는
카메라 자세가 필요하며, 해당 정보가 없으면 준비를 거부한다.

현재 registration은 `pose_source=fixed_measured`만 지원한다. 다음은 **파일 구조 예**이며
숫자나 serial을 실제 보정값으로 사용할 수 없다. 실제 측정한 변환으로 교체하고
revision을 변경한다. `translation_m`은 물리 optical 원점의 target-frame 위치,
`quaternion_xyzw`는 optical 좌표를 target-frame으로 회전하는 단위 quaternion이다.

```json
{
  "cameras": {
    "zed": {
      "camera_id": "zed",
      "serial": "REPLACE_WITH_ACTUAL_ZED_SERIAL",
      "optical_frame": "physical_zed_optical",
      "target_frame": "World",
      "translation_m": [0.0, 0.0, 0.0],
      "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
      "revision": "REPLACE_WITH_MEASURED_REVISION",
      "pose_source": "fixed_measured"
    }
  }
}
```

RB10의 `target_frame`은 `link0`, 상용플랫폼은 `World`다. 도장은 같은 map에
`d405` 항목을 추가하거나 별도 파일을 사용한다. D405의 `camera_id`는 `d405`,
독립 optical frame은 예를 들어 `physical_d405_optical`이며 실제 D405 serial과
별도로 측정한 변환을 넣는다. 가상 별칭 `zed_left_camera_frame_optical`,
`d405_color_optical_frame`은 실제 registration으로 사용할 수 없다.

고급 설정/API의 `connections`에서 `outpost_http`,
`outpost_zed_hw_id`, `outpost_zed_serial`, `zed_calibration_file`을 지정한다.
도장은 `outpost_d405_hw_id`, `outpost_d405_serial`, `d405_calibration_file`도 필요하다.
두 calibration 경로가 같은 camera map 파일을 가리켜도 된다. 물리 registration은
가상 D405의 `T_d405_optical_to_tcp` hand-eye 파일과 다르다. 실제 움직이는
카메라의 촬영 시점 자세 입력은 현재 구현되지 않았으므로 고정 측정 자세를
제공할 수 없으면 준비 상태를 열지 않는다.

영상 위 선택을 시작하면 표시 중인 RGB-D identity를 고정한다. 최대 30초의
선택 lease 동안에도 현재 카메라의 신선한 입력은 계속 필요하다. 재연결·clock
reset·보정 변경은 선택과 기존 경로를 무효화한다. 용접은 열린 진행선을 그리며,
뿜칠/도장 평면 추출도 이 고정 RGB·depth·intrinsics를 사용하며 입력이 없으면 최신
프레임으로 대체하지 않는다. 용접의 추정 법선은 관측 카메라를 향하는 표면 바깥쪽이며,
평면으로 방향을 확정할 수 없을 때 고급 입력에서 단위 `reference_up` 벡터를
명시한다. 이는 사용자 공정 방향이며 자동 인식/실측 보정을 뜻하지 않는다.

롤러의 D405 work-area refine은 현재 물리적 카메라의 ROI를 새로 캡처한다.
가상 손목 prescan을 실행하지 않으며 캡처 요청 뒤의 accepted refined plane을
기다린다. ROI가 보이지 않거나 새 측정이 오지 않으면 타임아웃으로 실패한다.
고정 카메라를 위한 가상 다중 위치 스캔은 수행하지 않는다.

측정 작업면은 Isaac과 MoveIt에 동일한 메시·변환으로 적용한다. 두 엔진의 같은
revision ACK와 1초 이내 상태가 있어야 실행할 수 있다. 접촉 예외의 대상은
`measured_task_surface`이며 선택한 팔의 롤러 접촉 링크 한 쌍만 허용한다.
D405 refine은 관측된 기존 작업영역을 수락된 측정 평면에 투영하여 같은 유한 메시를
갱신한다. 현재 parent revision이 맞아야 하며 새 revision을 두 엔진이 확인하기
전에는 refined plane을 수락·발행하지 않는다. 실행기는 그 plane의
`task_scene_revision`까지 일치시킨다. 정지한 고정 카메라 캡처 대기는 장면 갱신을
허용하지만 실제 이동·미완료 trajectory 취소·접촉 예외가 있으면 거부한다.
장면 변경·입력 무효화는 계획을 무효화하고 진행 중 동작을 중단한다.
일시정지/재시작·중복 영상의 이전 timestamp를 현재 시각으로 바꾸지 않는다.

## 롤러 힘 제어

측정 경계는 ArmContract의 `sensor_joint`와 `wrench_frame`이다. Isaac의 실제
joint reaction을 센서 좌표로 변환하고, 센서 아래 고정 부품들의 질량·질량중심으로
계산한 중력을 별도 전달한다. 하드웨어 plugin이 중력을 한 번 보상하고, 기존
free-space tare와 독립 잔차 확인을 거친다. 목표 힘을 센서 값으로 만들지 않는다.

각 팔은 별도 state/command topic과 force-control namespace를 사용한다.
6축 payload 순서는 선택 팔의 `joints` 순서다. session/revision 불일치, 다른 팔의
관절 이름, 잘못된 wrench frame, 비유한 값, pause와 0.20초를 넘긴 상태는 거부한다.
`real_painting_enabled=true`, `dry_run=false`, `painting_force_enabled=true`는
기존 힘 제어 interlock이다. UI 모드는 이 플래그로 추론하지 않으며 시뮬레이션은
선택 관절이 `IsaacPaintingSystem`에 연결되었는지 별도로 확인한다.

최종 힘 검증은 두 PAINT 획 각각에 다음을 요구한다.

- 접근 → free-space tare 완료 → 접촉 탐색 → 가압 → PAINT → 감압 → 후퇴.
- 기본 목표 20 N, 제어 hold band 20–30 N.
- 각 획의 첫 1초를 제외하고 최소 5초 동안 신선한 실측 샘플 유지.
- 해당 구간의 95% 이상이 18–32 N. 누락/오래된 샘플과 session 변경은 실패.
- 중단 명령의 제어기 취소 응답과 실제 정지 확인. 접촉 해제 뒤 원래 ACM 복원.

모델 표시, 관절 명령 전송, 정상적인 0 N 측정만으로 이 검증을 통과시키지 않는다.

## 읽기 전용 검사

저장된 자산과 실행 프로세스의 heartbeat를 읽는 기본 명령이다. 로봇 이동,
공구 교체, 자산 생성, 서비스 시작을 수행하지 않는다.

```bash
python3 scripts/check_platform_runtime.py \
  --mode simulation --platform commercial --arm left --process paint \
  --assets .runtime/platform/assets/ASSET_REVISION \
  --output .runtime/platform/reports/left-paint-readiness.json
```

`--assets`를 생략하면 저장된 runtime configuration의 `bundle_directory`를 사용한다.
`bundle.json`, `simulation-ready.json`, `managed-ready.json`의 선택, session,
asset/runtime/scene revision, PID, 1초 이내 heartbeat와 센서 유효성을 확인한다.
준비가 정상이어도 최상위 실행 판정은 `not_run`이다. `readiness.status`만 별도로
`passed`가 된다. 프로세스 종료 시 남은 heartbeat로 실행 성공을 주장하지 않는다.

모든 보고서는 simulation/mixed × commercial left/right 또는 RB10 single ×
spray/weld/paint의 18행 `verification_matrix`를 포함한다. 증거 없는 행은
`not_run`이며 passed 집계에 들어가지 않는다. real 모드는 검증기에서 허용하지 않는다.

## 시뮬레이션 기반 연결·소량 이동 probe

```bash
source scripts/platform_ros_env.sh
/usr/bin/python3 scripts/check_platform_motion.py --assets /absolute/path/to/bundle
# 시뮬레이션에서만 실제 소량 이동/복귀/취소 시험을 허용할 때:
/usr/bin/python3 scripts/check_platform_motion.py --assets /absolute/path/to/bundle --execute
```

기본 모드는 현재 simulator/managed marker, live MoveGroup의 simulator hardware,
전체 로봇 `GetStateValidity`와 충돌 contact를 검사한다. `--execute`는 선택한 팔의
첫 관절을 약 0.02 rad 계획·실행 후 복귀하고, 느린 이동 중 취소 응답과 실제
정지 표본 및 비활성 관절 drift를 확인한다. `motion-probe.json`의 종류는
`infrastructure_probe`다. 이것을 spray/weld/paint 공정 완료나 롤러 힘 유지의
증거로 계산하지 않는다. mixed/real은 이 probe에서 거부한다.

## 기록된 실행 증거 평가

```bash
python3 scripts/check_platform_runtime.py \
  --mode mixed --platform rb10 --arm single --process paint \
  --observations /path/to/measured-observations.json \
  --output .runtime/platform/reports/rb10-mixed-paint.json
```

관찰 파일은 `selection`, `identity`, `executed`, `command_accepted`,
`terminal_state`, `camera`, `controller`, `joint_samples`, `geometry`, `stop`을 갖는다.
paint에는 `events`와 `force_samples`, weld에는 `weaving`을 추가한다.
정확한 최소 사례는 `tests/platform_runtime/test_report_contract.py`에 있다.
이 파일의 값은 단위 테스트용이며 실제 실행 증거가 아니다.

- `identity`: session_id, asset_revision, runtime_revision, tool_revision,
  scene_revision, mixed의 calibration_revision.
- `camera`: connected, source, camera_id, frame_id, age_s. source는
  `isaac_camera`, `live_camera`, `recording`, `fixture`를 구별한다.
  실카메라가 없으면 mixed는 `not_run`, 녹화/fixture를 실카메라로 평가하면 실패한다.
- `joint_samples`: 관절 이름 6개, 실제 positions, 같은 순서의 desired_positions,
  원 timestamp, state_age_s, valid, session/scene revision, 측정 source.
  command echo를 실제 관절 추종으로 계산하지 않는다. 추종 허용 오차는 0.02 rad이다.
- `geometry`: 실측 normal_error_deg와 offset_error_m. 허용 오차는 2도와 5 mm이다.
- `stop`: last_input, detected, disabled, cancel_ack, stopped의 같은 steady clock 값.
  취소 요청 전송만으로 실제 정지를 인정하지 않는다.
- `force_samples`: 원 step/sim_time, session/scene revision, state/force age,
  measured_contact_force_n과 `source=isaac_measured_reaction`.
- `weaving`: `source=measured_tcp`, 실제 위치 오차 및 최소 두 주기 관측.

관찰 파일의 출처와 수집 도구 로그를 함께 보존한다. 이 검증기는 외부 JSON의
진위를 인증하지 않으며 측정값·출처·revision의 계약을 평가한다.

## 기존 계획의 명시적 실행

`--execute --plan /path/to/validated-plan.json`을 추가해야 실행 요청을 보낸다.
새 시험 경로를 만들거나, free-space 확인을 임의로 설정하거나, 실환경으로
fallback하지 않는다. 계획이 없거나 오래된 경우 요청 자체를 거부한다.

계획 envelope는 `validated=true`, 현재 `selection`, `identity`, `plan_id`를 가진다.
paint/spray에는 실제 executor의 `path_id`를 plan_id로 사용하고 `plan_hash`도
일치해야 한다. weld에는 현재 `server_session_id`와 `expected` identity가 필요하다.
시작 직전에 실제 ROS 준비 상태와 같은 plan identity인지 다시 확인한 뒤 기존
`/sketch_execute` 또는 `/welding/execute` 요청을 한 번 보낸다.

수집기는 controller reference/feedback, PAINT 이벤트, 실제 센서·카메라 입력을
기록한다. 타임아웃에는 `/motion_abort`를 요청한다. 현재 수집기는 표면 오차,
별도 중단 시험의 취소/정지 시간, TCP weaving 오차를 자동 산출하지 않는다.
이 증거가 없으면 명령이 완료되어도 전체 실행 보고서는 통과하지 않는다.
해당 항목은 같은 identity로 수집한 계측 관찰 파일로 평가해야 한다.

## 구현 검증과 남은 통합 검증

선택 팔 state/command 계약, namespace, 센서 변환, ACM 복원, 측정 작업면 ACK,
고정 D405 capture, 보고서 판정은 CPU/ROS 단위 테스트로 검증한다. 동적 작업면은
`task_scene` 또는 `perception` registry로 선언하여 명목 box 없이 시작하며,
실측면이 없으면 작업 준비를 열지 않는다. 경로 점을 측정면으로 역생성하지 않는다.
최근 canonical 넓은 회귀는 1,689개 통과/8 skip이었다. 이후 수정의 집중 회귀와
겹치므로 합산하지 않으며 현재 모든 변경의 최종 전체 suite 결과로 확대하지 않는다. Isaac에서의
실제 접촉력 유지와 UI 취소, H형강 도달성·카메라 시야, 실카메라 기반 세 공정 실행은
별도 실행 보고서가 필요하다. 미연결 장비와 미실행 조합을 이 문서에서 완료로
바꾸지 않는다. 관리 UI의 지원 여부와 실제 검증 결과를 구분해 확인한다.

## 시뮬레이터 pixel workflow probe

`scripts/check_platform_workflow.py --assets <bundle.directory>`는 기본적으로
준비 상태, live MoveGroup의 simulator URDF, 전체 로봇 충돌 상태, ZED 영상에
투영한 작업면 pixel ROI만 검사한다. 작업면 선택도 D405 손목 이동을 일으킬 수
있으므로 기본 모드에서는 workflow topic을 publish하지 않는다. 보고서의
`read_only_pixel_plan`은 MoveIt 경로 검증이나 실제 도장 완료를 의미하지 않는다.

명시적으로 `--execute`를 지정하면 기존 UI와 같은 target select → D405 측정 →
work area → 재측정 → 두 stroke sketch → plan validation → Run 순서를 사용한다.
`--workflow fill`은 동일한 work area에 기존 fill 요청을 보낸다. 현재 paint만
지원하며 mixed/real은 거부한다. `--roi-fraction` 기본값은 벽면 크기의 0.3이다.
실행 중에는 별도 probe나 UI로 동작을 겹쳐 요청하지 않는다.

선택 팔 namespace, session/revision, 실제 상태·카메라·controller를 검증한다.
벽면 앞뒤와 pixel 범위를 검사하며, free-space 확인은 연속된 실제 wrench에서
중력을 제외한 힘/토크, 관측 속도, TCP와 벽면 거리로 확인한 경우에만 보낸다.
최종 JSON 외에 `.trace.json`과 `.samples.csv`로 실행 이벤트와 실측 힘을 남긴다.
두 PAINT stroke 각각 시뮬레이션 시간으로 첫 1초를 제외한 5초 이상에서
18–32 N 범위 비율 95% 이상,
접근·tare·접촉 탐색·ramp·retreat 순서를 모두 만족해야 도장 검사를 통과한다.
보고서에는 시뮬레이션 시간과 실제 경과시간을 함께 남기고, 상태 freshness는
계속 실제 경과시간으로 판단한다. 이 검사는 별도 취소·정지 시험을 대신하지 않는다.

`--execute --cancel-during-paint`는 힘이 걸린 첫 PAINT에서 public `/motion_abort`를
한 번 요청하는 별도 중단 시험이다. 해당 실행의 중단 이벤트, 실제 전달된 영점
힘 명령, 하드웨어 inhibit와 실측 정지를 함께 확인한다. 이 동작은 정상 감압·
후퇴가 아니라 재시작이 필요한 강제 정지이며 자동 후퇴를 수행했다고 기록하지 않는다.
시뮬레이션 스택은 inhibit 즉시 준비 상태를 해제하고, 정지 후 관측을 위해 최대
5초간 센서·ROS를 유지한 뒤 종료한다. inhibit는 이 시간 동안 해제되지 않는다.

`simulation-phase-timing.json`과 `PLATFORM_SLOW_STEP` 로그는 physics, rendering,
카메라 readback, 상태 발행과 스케줄링 지연을 기록한다. 외부 GPU 작업과 겹친
상태 전달 지연을 정상적인 접촉력 검증으로 집계하지 않으며, 진단을 위해
0.2초 상태 유효시간이나 힘 보호 기준을 늘리지 않는다.

느린 렌더링 시뮬레이터용 `--workflow-time-scale` 기본값은 3이다. D405 target 및
work-area 측정 대기는 각각 360초, 최종 도장 완료 대기는 900초가 된다. 센서/상태
freshness, readiness/catalog/plan 확인 제한시간과 실행 중 identity guard는
늘리지 않는다. 기존 RB20 checker의 기본 제한시간은 유지한다.

## 시뮬레이터 CAD weld workflow probe

`scripts/check_platform_weld.py --assets <bundle.directory>`는 기본적으로
현재 simulator/session, live MoveGroup URDF, ZED frame identity, 생성된 CAD
fixture seam ID를 읽기만 한다. `--plan-only`는 public target/selection/settings/
plan topic으로 계획만 요청하며 실행하지 않는다. `--execute`만 실제 시뮬레이터
용접 경로 실행을 요청한다. `--seam <ID>`를 반복해 fixture의 실제 seam 순서를
지정할 수 있고 생략하면 첫 두 seam을 선택한다. `--settings <json>`으로 기존
용접 settings 계약에 맞는 값을 요청한다.

검증용 H형강 작업선과 설정은 다음과 같다. 기본 0°/0°가 모든 로봇 배치에서
도달·충돌 조건을 만족하는 것은 아니며, 설정 변경 뒤 반드시 계획을 다시 생성한다.

| 로봇 | 작업선 | 작업각 / 진행각 | 현재 증거 |
| --- | --- | --- | --- |
| 상용 왼팔·오른팔 | `IfcFastener_WELD_004` 정방향 | 0° / 15° | 전체 시뮬레이션 실행·위빙 통과 |
| RB10 | `IfcFastener_WELD_004` 정방향 | 60° / 0° | 전체 시뮬레이션 실행·위빙 통과 |

시뮬레이션 RB10 용접은 원본 H형강 전체를 `[-1.5,0,0]`에 강체 이동하고 카메라를
함께 배치한다. 상용 왼팔 뿜칠은 H형강을 기존 위치에서 Y 방향 +0.40 m 이동하며,
RB10 뿜칠은 대상체를 유지하고 낮은 작업면이 보이도록 카메라 시점만 변경한다.
원본 형상·TCP·팔 길이는 바꾸지 않으며 일부 시뮬레이션의 실제 측정 대상에는
이 가상 배치를 적용하지 않는다. 배치와 출처는 생성된 `scene.json`에 기록한다.

기본 planning timeout은 540초, execution timeout은 900초다. 실행 중 readiness
identity/freshness 확인은 유지한다. `.trace.json`에는 실제 base→TCP TF와 용접
output의 simulator 시작 시각, `.trace.csv`에는 seam별 실측 trace를 남긴다.
기존 `report_segment`가 tracking/speed/weave를 판정하며 관측 부재는 통과시키지
않는다. 이 도구는 CAD fixture 검증이며 카메라 seam 인식이나 별도 cancel 시험을
완료했다고 표시하지 않는다.

Commercial right/Hongik CAD fixture의 `IfcFastener_WELD_004`는 기본 work/travel
각도 0/0에서 약 70.7% 지점에 움직이는 팔과 주차된 반대 팔의 충돌이 확인됐다.
동일 장면의 `{"work_angle_deg":0,"travel_angle_deg":15}` 설정은 이후 전체 접근·
용접·후퇴 실행과 실측 위빙 판정까지 통과했다. 수치는 위 최신 증거 표와
`matrix-final/commercial-right-weld/weld-probe.json`을 따른다. 기본값이나 충돌 규칙은
변경하지 않았다. 상세 근거는
`.superpowers/sdd/2026-10-15-platform-eoat-ui/weld-geometry-audit.json`에 남긴다.

## 시뮬레이터 spray public-workflow probe

```bash
source scripts/platform_ros_env.sh
.venv-api/bin/python scripts/check_platform_spray.py --assets .runtime/platform/matrix-live/commercial-left-spray
# 선택·경로 생성·로봇 실행을 명시적으로 허용할 때만:
.venv-api/bin/python scripts/check_platform_spray.py --assets .runtime/platform/matrix-live/commercial-left-spray --execute
```

기본 모드는 simulator/session/revision, 실제 MoveGroup 전체 모델, 전체 관절 충돌과
카메라에 보이는 H형강 CAD ROI만 검사한다. `--execute`는 기존 target 선택 → ZED
plane lock → work-area → `/painting_system/zed_sketch_request` → 경로 검증 →
`/sketch_execute` 흐름을 사용한다. 실제 ZED 추출면이 선택한 CAD 면과 맞지 않으면
거부한다. `--cad-prim`으로 기존 CAD primitive를 지정할 수 있으며 새 형상이나 측정
평면을 주입하지 않는다. 실제 카메라/실환경과 paint/weld 구성은 거부한다.

이 probe는 live `spray_motion_test=true`와 `dry_run=false`를 확인한다. 물리 건 출력은
비활성 상태이며 도막/분사량 검사가 아니다. 출력 JSON은 CAD triangle/hash 출처,
현재 path identity, 전체 모델 충돌 결과, 비활성 팔 변화량과 실측 관절 TF의 TCP를
기록한다. TCP에 native nozzle offset을 한 번 적용해 SPRAY 구간의 노즐 경로를
비교하며 최대 위치 오차 10 mm, 축 오차 5도, 경로 범위 90% 이상, 시뮬레이션
sample 간격 0.2초 이하와 비활성 관절 변화 0.005 rad 이하가 필요하다.
`.trace.json`에는 실제 실행 이벤트·TCP·관절 샘플이 남는다.

상용 왼팔·오른팔·RB10의 라이브 전체 실행·추종 검증을 통과했다. 최대 노즐
위치 오차는 각각 2.869 mm, 1.533 mm, 0.818 mm이며 경로 범위는 99.62%,
99.94%, 99.81%다. 완료 이벤트만으로 통과시키지 않는다. 이 검사는 별도의 취소/
정지 시험, 물리 카메라 입력, 실제 spray output 또는 전체 18조합 검증을 대신하지 않는다.

첫 public weld 실행은 모든 phase를 완료했지만 실측 weave 폭 3.427 mm가 요청
4 mm보다 14.3% 작아 5% 기준을 통과하지 못했다. 실측 1 Hz 성분의 fitted 폭
3.3866 mm는 기존 가상 drive의 D/K=0.1 s 응답 예측 3.3869 mm와 일치했다.
따라서 simulator의 spray/weld는 K=300000 Nm/rad, D=6000 Nms/rad를 사용한다.
Paint의 K=300000, D=30000과 기존 torque/velocity 한계는 유지한다.
`simulation-drive-contract.json`에 process, gain 및 가상 구동계 provenance를
기록하며 실제 actuator 사양으로 해석하지 않는다. 변경 gain은 이후 위 오른팔 CAD seam 실행의 실측 위빙 판정을 통과했다.
다른 구성의 통과를 추정하지 않으며 weave/tracking 판정 기준은 완화하지 않는다.

모든 공정의 simulator physics는 500 Hz, 실제 joint state 및 `/clock` 발행은
100 Hz, 렌더링은 15 Hz다. 비접촉 공정의 짧은 가상 servo 시정수를 더 작은
물리 timestep으로 계산하며 paint의 기존 주기와 F/T 5-sample window는 유지한다.
정지 판정은 실제 simulator joint velocity와 위치 기반 속도를 모두 검사한다.
둘의 차이가 크더라도 실제 velocity를 버리거나 정지 허용치를 늘리지 않는다.

시뮬레이션 paint의 알려진 벽은 Isaac과 MoveIt에서 같은 전체 위치·크기를 사용한다.
생성된 `objects.yaml`의 `collision_padding_m: 0.0`과
`collision_geometry_source: shared_simulation_scene_wall`은 이 정확한 공유 형상을
명시한다. 기존 설정의 기본 20 mm padding은 유지되며, 명시한 padding이 음수나
비유한 값이면 설정을 거부한다. 접촉 허용 충돌 쌍과 힘 안전 한계는 바꾸지 않는다.

롤러는 원본 고정 조인트를 유지한 cylinder sliding 근사 모델이다. 롤러와 벽의
마찰계수는 각각 static 0.4/dynamic 0.3/restitution 0으로 같게 설정하며,
PhysX 평균 결합에서 dynamic 0.3을 유지한다. 5000 N/m, 50 Ns/m의 compliant
contact는 롤러에만 적용한다. Mixed paint의 측정 삼각형 표면도 같은 rigid
마찰 재질을 받지만 표면 좌표·삼각형·frame 검증은 유지하고 두께를 추가하지 않는다.
`simulation-contact-contract.json`에 모델·재질과 적용된 측정 표면 revision을 기록한다.
이 근사는 passive roller bearing의 실측 검증을 대신하지 않는다.

세 공정의 OMPL 충돌 edge fraction은 0.0001, TOTG path tolerance는
0.001 rad다. D405 접근을 포함해 경로 시간화 중 형상 변형과 충돌 검사 간격을 제한하며 최종
ValidateSolution, 전체 충돌 규칙, 속도·가속도 제한은 유지한다.

최종 관리 UI는 revision 3의 대기 설정(시뮬레이션·상용 왼팔·내화뿜칠,
왼팔 spray v2/오른팔 Hongik v4)으로 복원했다. 8081 API와 스케치 페이지는
HTTP 200을 확인했으며 simulator는 시작하지 않았다. 실행하려면 구성 적용이 필요하다.


## ZED 장착 시점 수정 · 2026-10-18

현재 상용 플랫폼·왼팔 뿜칠의 완전 시뮬레이션 ZED는 Hongik v4 `zed2i_link`에 연결한다.
이번 변경은 해당 데모 구성에 한정하며 다른 공정의 카메라 장착 시점은 별도 전환·검증 대상이다.
이전 작업별 `eye/target` 관찰 시점은 실제 양팔 사이 장착을 재현하지 않았으므로
센서 입력에 사용하지 않는다. Isaac 관찰용 뷰포트 카메라는 별도로 유지한다.

렌즈 기준은 STL의 왼쪽 렌즈 중심 `[-0.060, 0, 0] m`, 링크 기준 회전 identity이다.
이는 CAD 명목 핀홀 기준이며 실측 광학 중심·내부 파라미터 보정값이 아니다.
센서 월드 자세는 측정된 헤드 강체 자세와 고정 장착 변환으로 매 프레임 계산하며,
ROS optical TF도 동일한 `zed2i_link` 아래 고정 변환으로 발행한다.

왼팔 뿜칠 시뮬레이션의 초기 데모 자세는 H형강 수평부재 윗면을 바라보도록
헤드 joint1=-0.05929017 rad, joint2=0.83027438 rad로 설정한다(2026-10-19).
시험체를 0.60 m 멀리 이동한 배치에서 World [0, -1.5, 1.016] m를 향하도록
조정한 값이다. 이전 2026-10-18 자세는 joint1=-0.16302702, joint2=1.35081407 rad였다.
왼팔은 노즐 TCP World `[0.35, -1.5, 1.65] m`의 분사 OFF 대기 자세로 시작한다.
수평부재 윗면의 예정 진입점 `[0.24, -1.514, 1.516] m`에서 약 0.174 m 떨어진
위치다. 왼팔 joint1–6은 `[1.61565899, -0.11542290, -1.91754763,
0.00000366, -1.10862947, 0.10417195] rad`이다. 재시작 후 측정 관절에서도
전체 로봇 충돌 검사 통과 및 작업면으로 향하는 ZED 광선 81개 가림 0개를 확인했다.
실제 RGB에서도 작업면이 가림 없이 보인다. 시작점과 획 끝까지의 Cartesian
계획 fraction=1.0을 확인했으며, 이 대기 자세에서 전체 작업 실행은 별도 검증 대상이다.
이 초기 자세는 실제 로봇으로 전송하는 명령이 아니며, 실환경 보정을 대체하지 않는다.
이전 임의 시점에 의존한 시야·공정 검증 기록은 이 장착 시점의 검증 증거가 아니다.


### 2026-10-18 왼팔 뿜칠 UI 경로 실행 재검증

장착 ZED 시점으로 선택한 H형강 수평 부재 윗면의 약 489 mm 단일 획을
두 번 실행하여 COMPLETE를 확인했다. 원뿔 분사 축의 0°/180° 회전과 두 IK
초기 추정값 중 전체 Cartesian 경로가 충돌 없이 완성되는 자세만 접근 후보로
사용한다. 기존 충돌 검사와 반대팔·머리의 측정 상태는 유지한다.
마지막 실행의 이격은 499.71–499.92 mm, 평균 속도는 sim-time 20.03 mm/s,
전체 이동 중 상태 충돌 표본 1,035회에서 검출 0회였다.

준비 상태 및 분사 출력 lease timer는 steady clock을 사용한다. 다만 분사
시각화 활성 구간의 카메라 프레임 갱신이 최대 3.61초 지연되어 runtime의
카메라 freshness가 만료됐다. **동작 성공과 데모 표시 안정화 완료를 구분한다.**
상세 측정값과 미검증 범위는 `SPRAY_MOTION_TEST.md`의 같은 날짜 항목을 따른다.

### 2026-10-19 분사 표시를 카메라와 공유

사용자 요청에 따라 분사 원뿔·도포 영역을 시뮬레이션 ZED RGB/depth에도 표시한다.
센서 촬영 때 표시를 숨기거나 render product를 껐다 켜지 않으며,
예약된 갱신마다 `world.render()`를 한 번 호출하여 카메라와 뷰포트를 함께 갱신한다.
완성되지 않은 카메라 프레임은 다음 예약 갱신에서 수신하고 추가 렌더로 기다리지 않는다.
RGB/depth의 프레임·시간 일치 및 실제 새 프레임 여부 검사는 유지한다.
표시용 메시가 depth에도 포함되므로 이 영상은 설명용 표시를 포함한 시뮬레이션 입력이다.

같은 초기 자세에서 분사 표시를 켠 별도 정지 시뮬레이션의 A/B 비교 결과,
렌더 처리 평균 144.0→21.0 ms, ZED 발행 4.08→8.41 Hz,
실시간 배율 0.272→0.561이었다. 실시간 성능 달성이나 전체 이동 중 검증을 뜻하지 않는다.
비교 스크립트의 시각화 강제 활성은 별도 ROS domain 206의 진단 실행에만 사용했으며
일반 실행의 계획/분사 lease 검증은 유지한다. 증거와 범위는 `SPRAY_MOTION_TEST.md` 참조.
