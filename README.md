# Indy7 2D 3-DOF 평면 구속 협동로봇 페이로드 동역학 파라미터 식별 시스템

본 프로젝트는 연세대학교 학부연구(2026-2)의 일환으로, 6자유도 협동로봇인 뉴로메카 Indy7을 2D 3자유도(J2, J3, J4) 수직 작업 평면으로 기구학적 구속을 적용하고, 내부 구동 토크 로그만을 활용하여 엔드이펙터에 부착된 비접촉식 페이로드(하중)의 4대 핵심 동역학 파라미터($m, c_x, c_y, I_o$)를 정밀 식별하는 프레임워크입니다.

---

## 핵심 연구 성과 및 기능

1. **2D 3-DOF 평면 기구학 및 역동역학 모델링 (`Indy7_2D3DOF.py`)**:
   - 도면 및 공식 URDF 기반 링크 치수($L_0=0.3\text{m}, L_1=0.45\text{m}, L_2=0.35\text{m}, L_3=0.228\text{m}$) 및 질량($5.85\text{kg}, 4.81\text{kg}, 2.61\text{kg}$) 정합.
   - 평행 피치 축(J2 Shoulder, J3 Elbow, J4 Wrist) 구동 및 나머지 관절($0^\circ$) 하드웨어 구속.
   - 쿨롱 및 점성 마찰 모델($\tau_{\text{fric}} = f_v \dot{q} + f_c \tanh(50\dot{q})$) 반영.
   - 페이로드 4대 파라미터 선형 회귀행렬 $Y(q, \dot{q}, \ddot{q})$ 선형화 오차 $< 10^{-12}\text{ N}\cdot\text{m}$ (기계 정밀도 검증 통과).

2. **조건수 최소화 5차 푸리에 급수 최적 여기 궤적 (`2D_trajectory.py`)**:
   - 21개 독립 계수 기반 위치/속도/가속도 연속성 및 초기 정지 경계조건($q(0)=q_0, \dot{q}(0)=0, \ddot{q}(0)=0$) 폐형식 소거.
   - 관절각($\pm 175^\circ$), 각속도($150\sim 180^\circ/\text{s}$), 최대 토크($431.97, 197.23, 79.79\text{ Nm}$) 마진 보장.
   - 관측 회귀행렬 특이값 조건수 최적화: $\operatorname{cond}(Y): 135.4 \to 18.2$ ($86.5\%$ 급감).

3. **고신뢰도 신호 전처리 및 2단계 OLS 식별 (`preprocess_sensor_log.py`, `identify_payload.py`)**:
   - 4차 영위상(Zero-phase) 버터워스 필터(5 Hz 컷오프) + Savitzky-Golay 다항식 평활 미분.
   - 1단계 무부하 로봇 토크 차감 $\to$ 2단계 선형 최소자승법(OLS) 하중 복원.
   - 토크 센서 잡음($\sigma=0.25\text{ Nm}$) 환경 하에서 **질량 추정 오차 0.02%**, **결정계수 $R^2 = 0.9958$** 달성.

4. **실시간 모멘텀 관측기 충돌 감지 및 ISO/TS 15066 안전 감시 (`collision_detector.py`, `safety_supervisor.py`)**:
   - 외력 토크 실시간 관측을 통해 $50\text{ ms}$ 이내 무센서 충돌 감지 및 감속 정지.
   - 작업자 협동 모드 TCP 선속도($\le 250\text{ mm/s}$), 전체 운동에너지($\le 40\text{ J}$), 토크 상한선 실시간 감시.

---

## 디렉터리 구조

```text
Indy7_kinematics_sihyun/
├── config/
│   └── indy7_dimensions.py        # CAD 기반 링크 치수, 질량, 관성 텐서 정의
├── docs/
│   ├── identification_results.png # 3종 검증 그래프 (궤적, 토크, 파라미터 막대)
│   ├── oral_presentation_script.md # 10분 구두 발표 실전 대본 및 심사위원 Q&A
│   ├── workspace_dimensions.png   # 작업영역 분석 자료
│   └── zero_pose.png              # 영점 포즈 캡처
├── model/
│   ├── indy7.urdf                 # 공식 Indy7 URDF 모델
│   └── Indy7_v1_Layout.pdf        # 공식 제품 도면
├── src/
│   ├── Indy7_2D3DOF.py            # 2D 3자유도 평면 역동역학 및 선형 회귀행렬
│   ├── 2D_trajectory.py           # 5차 푸리에 급수 여기 궤적 생성 및 최적화
│   ├── identify_payload.py        # 2단계 OLS 페이로드 식별 모듈
│   ├── preprocess_sensor_log.py   # 영위상 버터워스 필터 및 Savitzky-Golay 미분
│   ├── run_experiment.py          # 전체 파이프라인 통합 실행 및 결과 시각화
│   ├── collision_detector.py      # 센서리스 모멘텀 관측기 기반 충돌 감지기
│   ├── online_rls_estimator.py    # 급변 하중 실시간 추종 재귀 최소자승법 (RLS)
│   ├── adaptive_controller.py     # 계산 토크 제어 (CTC) 페이로드 피드포워드 보상기
│   ├── safety_supervisor.py       # ISO/TS 15066 실시간 안전 감시자
│   ├── ekf_payload_estimator.py   # 가속도 미분 없는 10차원 EKF 상태-파라미터 추정기
│   ├── manipulability_analysis.py # 요시카와 가조작도 및 특이점 마진 검증기
│   ├── frequency_response_analysis.py # FFT/PSD 스펙트럼 및 공진 대역 회피 검증기
│   ├── spatial_6dof_payload_identifier.py # 3D 공간 6자유도 확장 회귀 모듈
│   ├── symbolic_dynamics.py       # SymPy 기반 기호 역동역학 폐형식 유도기
│   ├── statistical_validation.py  # 95% 신뢰구간, 표준오차, t-통계량 분석 모듈
│   ├── hand_guiding_controller.py # 무중력 다이렉트 티칭 어드미턴스 핸드가이딩 제어기
│   ├── slosh_free_shaper.py       # ZVD 입력 성형 액체 출렁임 억제기
│   └── dob_robust_controller.py   # 외란 관측기(DOB) 2차 Q-필터 2자유도 강인 제어기
├── tests/
│   └── test_pipeline.py           # 8대 핵심 기능 단위 테스트 (100% 통과)
├── web/
│   ├── index.html                 # Three.js 3D 기구학 뷰어 UI
│   └── viewer.js                  # Three.js 렌더링 파이프라인
└── requirements.txt               # 프로젝트 의존성 목록
```

---

## 실행 가이드

### 1. 가상환경 활성화 및 의존성 설치
```powershell
cd "d:\01_Academic\yonsei univ\2026-2\학부연구\Indy7_kinematics_sihyun"
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

### 2. 단위 테스트 실행 (8대 핵심 파이프라인 검증)
```powershell
python -m unittest tests/test_pipeline.py -v
```
출력 결과:
```text
test_01_forward_kinematics ... ok
test_02_regressor_linearity ... ok
test_03_trajectory_boundary_conditions ... ok
test_04_ols_identification ... ok
test_05_safety_supervisor ... ok
test_06_momentum_observer ... ok
test_07_frequency_separation ... ok
test_08_spatial_6dof_regressor ... ok
----------------------------------------------------------------------
Ran 8 tests in 0.816s - OK
```

### 3. 전체 식별 실험 파이프라인 및 그래프 생성
```powershell
python src/run_experiment.py
```
- 결과 산출물: `docs/identification_results.png` 생성 확인.
