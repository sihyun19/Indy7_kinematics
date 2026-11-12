# Indy7 Kinematics

학부연구(2026-2). 뉴로메카 Indy7의 기구학·동역학 모델링과, 평면 3자유도로 구속한 모델에서의 엔드이펙터 페이로드 식별용 궤적 최적화.

## 진행 현황

| 단계 | 내용 | 상태 |
|---|---|---|
| 1 | 순기구학(FK), 자코비안, 3D 웹 뷰어 | 완료 |
| 2 | 강체 동역학(관성·코리올리·중력) | 완료 |
| 3 | 2D 3자유도(Indy7의 J2, J3, J5 기준) 토크 모델, 페이로드 4개 파라미터(m, mc_x, mc_y, I_o) 선형 회귀행렬 | 완료 |
| 4 | 푸리에 급수 궤적 + SLSQP 조건 수 최적화 | 구현 완료, 변수 시작값 난수화 예정 |
| 5 | 2단계 OLS 페이로드 식별 | 작성 중(`identify_payload.py` 비어 있음) |

## 구성

```text
config/indy7_dimensions.py   링크 치수, 질량, 관성, 관절 한계 (URDF 기반)
model/                       indy7.urdf, 메쉬, 제품 도면
src/
├── Indy7_FK.py              순기구학
├── Indy7_Jacobian.py        TCP·링크 무게중심 자코비안
├── indy7_dynamics.py        6자유도 동역학
├── Indy7_2D3DOF.py          2D 3자유도 토크 모델, 회귀행렬
├── 2D_trajectory.py         푸리에 궤적, 조건 수 목적함수, 제약조건, SLSQP
├── identify_payload.py      페이로드 식별 (미구현)
└── viewer_3d.py             3D 뷰어 서버 (web/ 사용, 최적화 궤적 .npz 재생)
web/                         Three.js 뷰어
tests/test_pipeline.py       구버전 테스트, 현재 코드와 맞지 않음
```

## 궤적 최적화 방식

- 관절당 5개 고조파의 유한 푸리에 급수, 주기 10 s.
- 시작 시 속도·가속도 경계조건을 해석적으로 만족시켜 변수를 30개에서 21개로 축소.
- 목적함수: 회귀행렬을 쌓은 행렬의 조건 수.
- 제약조건: 관절 위치·속도·토크가 한계치의 80% 이내(안전율 1.25).

## 실행

```powershell
pip install -r requirements.txt
cd src
python Indy7_2D3DOF.py     # 선형화 정합성 확인
python 2D_trajectory.py    # 궤적 최적화, 결과를 optimized_trajectory.npz/.txt 로 저장
cd ..
python src/viewer_3d.py    # 3D 뷰어
```