# 🦾 Indy7-Kinematics-Payload-Estimator

> **Sensorless End-Effector Payload Estimation via Parametric Kinematics and Motor Torque Feedback for Neuromeka Indy7**  
> *연세대학교 공과대학 기계공학부 생산공학연구실 (Advanced Manufacturing Lab) 2026-2학기 학부연구 프로젝트*

[![Python](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![NumPy](https://img.shields.io/badge/NumPy-Linear_Algebra-013243.svg)](https://numpy.org/)
[![Validation](https://img.shields.io/badge/Validation-MATLAB_Simscape-orange.svg)](https://www.mathworks.com/products/simscape.html)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## 📌 Executive Summary (연구 개요)

협동로봇 현장에서 고가의 6축 F/T(Force/Torque) 센서 없이 로봇 끝단(End-Effector)에 장착된 물체의 무게를 추정하는 것은 안전 제어 및 비용 절감 측면에서 매우 중요한 과제입니다.

본 프로젝트는 **뉴로메카(Neuromeka) Indy7 (6-DOF 협동로봇)**을 대상으로, **2·3·5번 축(Pitch) 평면 3자유도 운동 구속** 하에서 추가 센서 없이 **모터 구동 토크 피드백(Current/Torque Log)** 및 정역학 수식($\tau = J^T F$)을 활용하여 엔드이펙터의 페이로드(하중)를 실시간 추정하는 파이프라인을 구축합니다.

특히, 연구실 기존 레거시 시스템인 **MATLAB Simscape Multibody 물리 모델**과의 수치적 정합성($10^{-9}$ 수준의 오차 일치)을 확보함과 동시에, 하드웨어 사양 변경에 즉각 대응할 수 있는 **데이터 주도형 매개변수화(Parametric) Python 기구학 엔진**을 구현합니다.

---

## 🎯 Key Engineering Highlights (핵심 기술적 특징)

1. **Zero-Magic-Number 파라메트릭 기구학 엔진 (Python/NumPy)**:
   - 기존 MATLAB 코드의 하드코딩된 행렬 수치(`0.0775`, `-0.450`, `-0.267` 등)를 완전히 탈피.
   - CAD 도면 치수(`Indy7Geometry`)와 $SE(3)$ 변환 루프(`compute_fk`)를 직교 분리하여, 툴(그리퍼) 길이 변경 및 링크 변경 시 단 1줄의 설정 수정으로 즉각 재계산 가능.
2. **선배 레거시와의 무위험 투트랙 하이브리드 검증 (Two-Track Arbitrage)**:
   - MATLAB Simscape의 복잡한 물리·충돌 모델(`indy7_sim.slx`)을 정답(Ground Truth)으로 삼아, Python 기구학/자코비안 결과와 1:1 단위 테스트(`assert_allclose`) 수행.
   - `scipy.io.loadmat` 브릿지를 통해 선배들이 생성한 최적 가진 궤적(`.mat`) 데이터를 무손실 추출 및 분석.
3. **정역학 평형 및 토크 잔차(Residual) 기반 하중 역산**:
   - 가상일의 원리(Principle of Virtual Work)에 기반한 관절 토크 $\leftrightarrow$ 외력 간 관계 유도: $\tau = J^T F$.
   - 무부하 중력 토크($\tau_{gravity}$) 및 비선형 관절 마찰($\tau_{friction}$)을 제거한 잔차 토크에 무어-펜로즈 유사역행렬($(J^T)^\dagger$)을 적용해 무게($m = F_z / g$) 추정.

---

## 📐 Mathematical Formulation (수학적 모델링)

### 1. 전진 기구학 (Forward Kinematics) & 자코비안 (Geometric Jacobian)
각 관절 $i$의 국소 동차변환행렬 $T_i$는 고정 회전(RPY) 및 고정 오프셋 치수($p_{xyz}$)와 회전 관절각 $q_i$의 결합으로 정의됩니다:

$$T_i(q_i) = \text{Trans}(p_{xyz}) \cdot \text{Rot}_{rpy}(\phi, \theta, \psi) \cdot \text{Rot}_z(q_i)$$

베이스부터 엔드이펙터(TCP)까지의 총 변환행렬 $T_{0}^{tcp}$는 체인 곱으로 합성됩니다:

$$T_{0}^{tcp}(q) = \left( \prod_{i=1}^{n} T_i(q_i) \right) \cdot T_{tcp\_offset}$$

기하 자코비안 $J(q) = \begin{bmatrix} J_v \\ J_\omega \end{bmatrix} \in \mathbb{R}^{6 \times n}$는 각 축의 회전 방향 벡터 $z_i$와 끝단 위치 벡터 $p_e$, 관절 원점 $p_i$의 외적으로 도출됩니다:

$$J_v^{(i)} = z_i \times (p_e - p_i), \quad J_\omega^{(i)} = z_i$$

### 2. 정역학 평형과 하중 역추정 (Payload Inversion)
가상일의 원리($\delta W = \tau^T \delta q - F_{ext}^T \delta x = 0$)에 의해, 미소 변위 관계 $\delta x = J \delta q$를 대입하면 정역학 평형식이 유도됩니다:

$$\tau = J(q)^T F_{ext}$$

실제 구동 모터에서 측정된 토크 $\tau_{meas}$는 링크 자중에 의한 중력 토크 $\tau_g(q)$, 비선형 마찰 토크 $\tau_f(\dot{q})$, 그리고 엔드이펙터 하중 토크 $\tau_{ext}$의 합입니다:

$$\tau_{meas} = \tau_g(q) + \tau_f(\dot{q}) + J(q)^T F_{ext}$$

정지 상태($\dot{q} = 0$) 또는 준정적(Quasi-static) 등속 구간에서, 사전에 식별된 자중 및 마찰 모델을 감산하여 **잔차 토크($\tau_{net}$)**를 계산하고 하중을 역산합니다:

$$\tau_{net} = \tau_{meas} - (\hat{\tau}_g(q) + \hat{\tau}_f(\dot{q}))$$

$$\hat{F}_{ext} = (J^T)^\dagger \tau_{net} \quad \Longrightarrow \quad \hat{m}_{payload} = \frac{-\hat{F}_z}{g}$$

---

## 📂 System Architecture & Directory Layout

```text
Indy7-Kinematics-Payload-Estimator/
├── config/
│   └── indy7_dimensions.py      # [Zero-Magic-Number] CAD 도면 기반 링크 치수 및 오프셋 명세
├── src/
│   ├── __init__.py
│   ├── kinematics.py            # 순수 NumPy 기반 N-자유도 파라메트릭 FK 및 기하 자코비안
│   ├── payload_estimator.py     # 토크 잔차 분리 및 의사역행렬 기반 질량 역추정 알고리즘
│   └── data_bridge.py           # MATLAB .mat 궤적 및 CSV 로그 데이터 로더
├── tests/
│   ├── test_fk_numerical.py     # 수치 미분 vs 해석적 자코비안 정합성 검증
│   └── test_matlab_crossval.py  # 선배 MATLAB Simscape 모델과의 수치 일치(Assert) 테스트
├── data/
│   ├── raw_matlab/              # 선배의 .mat 가진 궤적 및 시뮬링크 출력 로그
│   └── processed/               # 파이썬 전처리 및 토크 프로파일 데이터
├── notebooks/
│   ├── 01_kinematics_validation.ipynb
│   └── 02_torque_payload_estimation.ipynb
├── requirements.txt
├── LICENSE
└── README.md
```

---

## ⚙️ Installation & Quick Start

### 1. 환경 설정
```bash
git clone https://github.com/your-username/Indy7-Kinematics-Payload-Estimator.git
cd Indy7-Kinematics-Payload-Estimator
pip install -r requirements.txt
```

### 2. MATLAB 레거시 정합성 단위 테스트 실행 (Sanity Check)
선배의 `indy7_sim_fk.m` 및 Simscape 모델의 홈 포즈 좌표 `[0, -0.1865, 1.3275]`와 오차 $10^{-9}$ 수준으로 일치하는지 자동 검증합니다.
```bash
python -m unittest tests/test_matlab_crossval.py
```

### 3. 파이썬 파라메트릭 기구학 예제 실행
```python
import numpy as np
from config.indy7_dimensions import Indy7Geometry
from src.kinematics import Indy7Kinematics

# 1. 도면 치수 파라미터 로드 (그리퍼 길이 변경 시 파라미터만 수정)
geo = Indy7Geometry(l_tcp=0.060)
robot = Indy7Kinematics(geometry=geo)

# 2. 홈 포즈 및 임의 관절 각도 계산
q_home = np.zeros(6)
T_tcp, J = robot.forward_kinematics(q_home)

print("TCP Position (m):\n", np.round(T_tcp[:3, 3], 4))
print("Geometric Jacobian Condition Number:", np.linalg.cond(J))
```

---

## 📈 14-Week Research Roadmap (2026-2학기 학부연구 마일스톤)

| 주차 | 기간 | 핵심 마일스톤 | 산출물 (GitHub Artifacts) |
| :---: | :---: | :--- | :--- |
| **W1~2** | 09/01 ~ 09/18 | 도면 치수 분석, 파라메트릭 FK 및 자코비안 구현 | `config/indy7_dimensions.py`, `src/kinematics.py`, 교차검증 테스트 |
| **W3~4** | 09/19 ~ 10/02 | 선배 MATLAB 시뮬레이션 데이터 브릿지 연동 | `data_bridge.py`, 가진 궤적 재현 플롯, 수치 미분 검증 스크립트 |
| **W5~6** | 10/03 ~ 10/16 | 2·3·5축 평면 구속 정역학 모델링 및 자중/마찰 영점화 | `payload_estimator.py`, 시뮬레이션 무부하 토크 프로파일 모델 |
| **W7~9** | 10/17 ~ 11/06 | Indy7 실기기 하중 실험 데이터 수집 및 오차 필터링 | 무게추(0.5kg~2kg) 추정 결과 그래프, 노이즈 감쇄 LPF 필터 |
| **W10~12**| 11/07 ~ 11/27 | 자세별 추정 오차 민감도 분석 및 Sim2Real 갭 고찰 | 최종 실험 분석 주피터 노트북, 학기말 연구 보고서 초안 |
| **W13~14**| 11/28 ~ 12/18 | 최종 보고서 교수님 제출 및 연구실 성과 발표 | 최종 학부연구 보고서(PDF), 클린 코드 배포 |

---

## 👨‍💻 Author & Acknowledgement

* **연구자**: **박시현 (Park Si-hyun)**  
  * 연세대학교 공과대학 기계공학부 학사과정 (2026-2학기 학부연구생)  
  * 연세대학교 생산공학연구실 (Advanced Manufacturing Lab)
* **지도 및 자문**: 
  * 김민성 연구원 (석사과정, 기구학 모델링 및 학부연구 지도)
  * 조성한 연구원 (Indy7 파라미터 식별 및 Simscape 시뮬레이션 환경 제공)
