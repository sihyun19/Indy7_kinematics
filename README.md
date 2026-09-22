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

## Requirements

* **OS**: Windows / Linux / macOS
* **Python**: Python 3.10 이상 (Python 3.11 권장)
* **Python Dependencies**:
  * `numpy` (필수: 기구학 및 동차변환행렬 연산)
  * `scipy`, `matplotlib` (선택: 데이터 분석 및 시각화용)
  ```powershell
  pip install -r requirements.txt
  ```
* **Web Environment**:
  * WebGL을 지원하는 모던 웹 브라우저 (Chrome, Edge 등)
  * 외부 CDN(Three.js, STLLoader) 로드를 위한 인터넷 연결 환경

---

## Quick Start

모든 명령어는 반드시 **프로젝트 루트 디렉토리(`Indy7_kinematics_sihyun/`)**를 현재 작업 디렉토리(Current Working Directory)로 설정한 상태에서 실행해야 합니다.

### 1. 프로젝트 루트 이동 및 가상환경 활성화
```powershell
# 프로젝트 루트 디렉토리로 이동
cd "d:\yonsei univ\2026-2\학부연구\Indy7_kinematics_sihyun"

# 가상환경 활성화 (Windows PowerShell 기준)
.\.venv\Scripts\Activate.ps1
```

### 2. 의존성 패키지 설치
```powershell
pip install -r requirements.txt
```

### 3. 정기구학(FK) 콘솔 단독 검증
영점 자세(Zero Pose) 및 임의 관절각에 대한 TCP 위치/자세 출력을 확인합니다.
```powershell
python src/Indy7_FK.py
```

### 4. 3D 웹 시뮬레이터 실행
로컬 HTTP 서버를 구동하면 기본 브라우저가 자동 실행되며 3D 뷰어 화면이 열립니다.
```powershell
python src/viewer_3d.py
```
* 웹 브라우저 수동 접속 주소: `http://localhost:8000/web/index.html`
* 서버 종료: 터미널에서 `Ctrl + C`
