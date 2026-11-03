import numpy as np
from Indy7_2D3DOF import INDY7_2D_JOINTS, compute_regressor

#뭔가 궤적을 만들어내는 함수는 2d trajectory에서 만든 후 불러올것

#궤적에서 데이터 받아서 토크, 회귀행렬 쌓는 함수

# 최소자승법 연산 (이론: theta_hat = np.linalg.inv(W.T @ W) @ W.T @ Tau)
theta_hat, residuals, rank, s = np.linalg.lstsq(Y_all, Torque_all, rcond=None)
