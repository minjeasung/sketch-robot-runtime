# 2026-09-22 반영 및 검증

## 이번 main 반영 범위

- RB10-1300E / RB20-1900ES 모델 선택, 모델별 URDF·관절 제한·SRDF·보정 경로.
- RB20에서 RB10 전용 저장 자세 사용 차단. 실장 EOAT 회전 방향과 보정은 별도 확인 필요.
- 현재 EOAT로 실제 뿜칠 경로를 검증하는 `spray_motion_test`: 분사 항상 OFF, 힘 제어 미사용.
- Michelo Outpost 콘솔에 새 탭 링크를 추가하는 독립 연결 서버. 참고 저장소/설치본 수정 없음.
- Outpost raw IPC → 기존 ZED/D405 ROS 토픽, 장치 식별·신선도·접근 권한 검사.
- 기본 카메라 입력을 Outpost로 변경. 기존 ROS 드라이버는 명시적 선택으로 유지.
- API 기본 포트 8081, 연결 서버 8101, 두 서비스를 함께 실행하는 스크립트와 설치 안내.

기존 main에 반영된 다중 평면·D405 측정 위치/순서 최적화·UI 정리도 유지한다.

## 확인 결과

- sketch_control 전체 단위 테스트: **396 통과 / 기존 접촉 도장 테스트 10 실패**.
- 새 Outpost 계약/연결 서버 및 API 테스트: **50 통과**.
- Outpost/API/RB20/뿜칠 관련 선택 테스트: **75 통과**.
- `colcon build --packages-select sketch_control --symlink-install`: 성공.
- 최상위 launch 인자 조회 및 모든 장치 그룹을 끈 launch 로딩: 성공.
- 가상 ZED/D405 raw IPC → ROS RGB·깊이·CameraInfo·점군: 성공.
  D405 SDK 소수 mm 보존, 광학 좌표계, 미터 단위와 3초 프레임 중단 실패 확인.
- 실제 Outpost 콘솔 HTTP·이벤트 WS·영상 binary WS 전달: 성공.
- 브라우저에서 스케치 새 탭·캔버스 표시, 원격 호스트 링크, 카메라 목록 선택,
  Outpost 모드에서 SDK 드라이버 비활성화: 성공.

10건은 기존 test_moveit_executor_fail_closed.py의 fixture 누락 4건과 접촉 후 취소 기대값
불일치 6건이다. 기존 RB10/RB20 fake MoveIt 종료 오류도 해결 완료로 간주하지 않는다.
이번 검증은 실제 로봇 이동/분사를 실행하지 않았다.

## 현장 검증에 남은 조건

확인 당시 `snucem` Outpost에는 ZED만 스트리밍 중이고 D405는 등록되지 않았다.
`Minjea`는 `snucem`의 raw IPC 디렉터리에 접근할 수 없다. 따라서 새 콘솔과 스케치 UI는
접속 가능하지만, 두 카메라를 통한 실측은 D405 연결 및 실행 계정/IPC 권한 준비 후 검증해야 한다.
장치 ID·시리얼과 해당 로봇·장착에 맞는 보정값은 현장 설정이며 Git에 포함하지 않는다.
