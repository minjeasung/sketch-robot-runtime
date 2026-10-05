# 평면 추출 버전

2026-10-05 현장 비교용으로 저장한 **버전별 스냅샷**이다. GitHub에서는 `plane-extraction/*` 보관 브랜치로,
이 PC에서는 동일한 이름의 로컬 태그와 복구용 번들로 보관한다.
번호가 높다고 성능이 좋은 것은 아니다. 현재 선택한 실행 버전은 **v0 (RGB 경계 우선)**이다.

| 이름 | 보관 브랜치 / 로컬 태그 | 평면 추출 방식 |
|---|---|---|
| base | `plane-extraction/base` | Git `f0f5fc9`의 원본. 선택 영역 전체 깊이에 반복 RANSAC 적용. RGB 경계 분할 없음. |
| v0 | `plane-extraction/v0` | RGB 경계 우선. RGB 직선과 깊이 연결 영역으로 나눈 뒤 RANSAC 피팅. |
| v1 | `plane-extraction/v1` | H형강 보강. 국소 법선 후보, 좁은 면의 지지점 배정, 잔여 조각 억제. |
| v2 | `plane-extraction/v2` | 깊이 RANSAC 우선. RGB는 기존 면 사이 경계만 보정. 깊이 일치 진단 표시. |

`v0`는 프로젝트 최초 버전이 아니다. 원본 코드는 `base`로 구분한다.
v0/v1은 수정 직전의 실제 파일 백업에서, v2는 사용자가 비교한 실행 폴더에서 복원했다.
각 버전은 공통 기반 `origin/main`(`af71ff3`)에 해당 평면 추출 파일과 관리 도구를 결합한 스냅샷이다.
따라서 보관 브랜치나 태그가 과거 시점의 프로젝트 전체 상태를 뜻하지는 않는다.
평면 추출과 무관한 미완료 로컬 변경은 공개 스냅샷에 섞지 않는다.
현재 실행 폴더의 그 변경은 전환 시에도 유지한다.

## 확인·전환

저장소 루트에서 실행한다.

```bash
python3 scripts/plane_versions.py list
python3 scripts/plane_versions.py status
python3 scripts/plane_versions.py switch v0 --dry-run
python3 scripts/plane_versions.py switch v0
# 비교할 때 v0 대신 base, v1, v2를 사용한다.
```

전환 범위는 `config/plane_extraction_versions.json`에 기록된 평면 관련 파일뿐이다.
다른 작업 파일, 현재 브랜치, Git 스테이징 상태는 유지한다.
브라우저 UI는 공통으로 유지한다. 대상 영역의 되돌리기·모두 지우기는 평면 목록과 선택도 지우고,
취소된 요청의 늦은 응답을 무시한다. 알고리즘 전환은 이 삭제 동작을 되돌리지 않는다.
작업영역·경로 단계의 되돌리기는 해당 스케치만 수정하며 대상 평면은 유지한다.
로컬 태그가 없으면 `origin/plane-extraction/*` 보관 브랜치를 읽는다.
기록되지 않은 수정이나 혼합 상태가 있으면 전환을 거부한다. 먼저 새 버전을 저장해야 한다.
버전에 없는 실험 파일은 전환 시 제거되지만 해당 태그에 복원 가능한 원본이 남는다.

명령은 **디스크의 코드만** 바꾼다. 실행 중인 노드는 기존 코드를 유지하므로
시스템 화면에서 인식(perception)을 다시 시작한 후, 브라우저를 새로고침하고 대상을 다시 추출한다.
프로세스 재시작은 기존 면 선택·작업영역·경로를 무효화할 수 있다.
이 도구는 로봇이나 카메라 프로세스를 자동으로 시작하지 않는다.

```bash
git diff --stat plane-extraction/base plane-extraction/v0
git diff plane-extraction/v1 plane-extraction/v2 -- src/sketch_control/sketch_control/
```

GitHub에서 보관 브랜치를 가져오면 모든 버전을 사용할 수 있다.

```bash
git fetch origin 'refs/heads/plane-extraction/*:refs/remotes/origin/plane-extraction/*'
```

관리 시점의 로컬 복구용 번들은 `.runtime/plane-extraction-versions.bundle`에도 보관한다.
다른 복사본에서 태그를 복원하려면 이 번들을 가져온다.

```bash
git fetch /path/to/plane-extraction-versions.bundle 'refs/tags/plane-extraction/*:refs/tags/plane-extraction/*'
```

앞으로 새 알고리즘은 새 보관 브랜치와 태그(v3 이후)로 저장한 다음 같은 RGB/깊이 녹화와 같은 ROI로 비교한다.
기존 보관 브랜치와 태그는 덮어쓰지 않는다. 픽셀 영역, 실제 면의 대응, 누락·오분리와 처리 시간을 함께 기록하고,
테스트 통과나 후보 면 개수만으로 현장 성능이 좋아졌다고 판단하지 않는다.
