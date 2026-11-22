'''
미지 파라미터 식별
'''
import numpy as np
from typing import Tuple
from Indy7_2D3DOF import INDY7_2D_JOINTS, fk_2D, compute_torque, compute_regressor
from Trajectory_2D import trajectory_evaluator
import csv, re, datetime
from pathlib import Path

#토크 측정값 가져오기
def get_loaded_data() -> Tuple:


    
    return loaded_time, loaded_q, loaded_qdot, loaded_qddot, loaded_torque

def get_unloaded_data() -> Tuple:


    
    return unloaded_time, unloaded_q, unloaded_qdot, unloaded_qddot, unloaded_torque

#레그레서 행렬 역연산 
def inverse_regressor( 
):
    loaded_time, loaded_q, loaded_qdot, loaded_qddot, loaded_torque = get_loaded_data()
    unloaded_time, unloaded_q, unloaded_qdot, unloaded_qddot, unloaded_torque = get_unloaded_data()
    if not len(loaded_q) == len(unloaded_q) or not np.allclose(loaded_time, unloaded_time) :
        raise ValueError
    N= len(loaded_q)
    Y_all = np.zeros((3 * N, 4))
    for i in range(N):
        Y_all[3 * i : 3 * i + 3, :] = compute_regressor(loaded_q[i], loaded_qdot[i], loaded_qddot[i])
    
    delta_tau = (loaded_torque - unloaded_torque).flatten()
    theta_hat = np.linalg.inv((Y_all.T @ Y_all)) @ Y_all.T @ delta_tau

    mass_hat = theta_hat[0]
    cx_hat = theta_hat[1]/mass_hat
    cy_hat = theta_hat[2]/mass_hat
    IO_hat = theta_hat[3]
    Icom_hat = IO_hat - mass_hat*(cx_hat**2+cy_hat**2)

    return mass_hat, cx_hat, cy_hat, IO_hat, Icom_hat

if __name__ == "__main__":
    pass

    

    

# 모주코를 이용한 계산값과 비교