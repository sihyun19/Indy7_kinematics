# Indy7 Kinematics & 3D Web Simulator

## Directory Structure

- `config/`
  - `indy7_dimensions.py`: CAD 도면 기반 링크 치수(지면~J1 0.0775 m, Link1 0.2225 m 분할 합산 0.3 m), 편심 오프셋, RPY 방위각, 단위벡터 및 관절 기하학 정의 (`Joint` 데이터클래스)
- `model/`
  - `Indy7_v1_Layout.pdf`: 뉴로메카 공식 Indy7 치수 도면
  - `indy7.urdf`: 공식 Indy7 URDF 로봇 모델
  - `meshes/`: 각 링크의 visual 및 collision STL 메쉬 파일 (`Indy7_0.stl` ~ `Indy7_6.stl`)
- `src/`
  - `Indy7_FK.py`: SE(3) 동차변환행렬 누적 기반 정기구학(FK) 계산, Roll-Pitch-Yaw 추출 및 영점 자세 검증
  - `viewer_3d.py`: 멀티스레드 로컬 웹서버(`ThreadingHTTPServer`) 및 Three.js 규격(Column-major) 변환행렬·TCP 좌표를 제공하는 `/api/fk` REST API
- `web/`
  - `index.html`: 6축 관절 한계각 슬라이더, 원클릭 프리셋(`Home`, `Test Pose`), 실시간 TCP 위치/자세 모니터링 UI
  - `viewer.js`: Three.js 씬 구성(Z-up, 조명, 그리드), 7개 STL 메쉬 로드, Python FK 전역 변환행렬 직접 주입(`matrixAutoUpdate = false`) 렌더링 파이프라인
- `docs/`: 작업영역 치수 및 기구학 검증 캡처 자료
- `data/`: 궤적 및 로봇 토크 로그 데이터 저장소 (예정)
- `notebooks/`: 데이터 분석 및 프로토타이핑 주피터 노트북 (예정)

---

## Quick Start

### 1. 정기구학(FK) 단독 검증
```powershell
python src/Indy7_FK.py
```

### 2. 3D 웹 시뮬레이터 실행
```powershell
python src/viewer_3d.py
```
* 웹 브라우저(`http://localhost:8000/web/index.html`) 자동 실행 및 실시간 인터랙션
