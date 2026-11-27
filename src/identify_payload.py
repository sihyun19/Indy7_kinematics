'''
미지 파라미터 식별
'''
import numpy as np
from typing import Tuple
from Indy7_2D3DOF import INDY7_2D_JOINTS, fk_2D, compute_torque, compute_regressor
from Trajectory_2D import trajectory_evaluator
import csv, re, datetime
from pathlib import Path
import sys
sys.path.append(str(Path(__file__).resolve().parent.parent))
TRAJ_DIR = Path(__file__).resolve().parent.parent / "data" / "trajectories"
RESULT_DIR = Path(__file__).resolve().parent.parent / "data" / "results"

#토크 측정값 가져오기
def get_sim_loaded_data() -> Tuple:
    with np.load(RESULT_DIR / f"trajectory_{traj_id:03d}" / f"sim_loaded.npz") as loaded_data:
        return loaded_data["time"], loaded_data["q"], loaded_data["qdot"], loaded_data["qddot"], loaded_data["torque"]

def get_sim_unloaded_data() -> Tuple:
    with np.load(RESULT_DIR / f"trajectory_{traj_id:03d}" / f"sim_unloaded.npz") as unloaded_data:
        return unloaded_data["time"], unloaded_data["q"], unloaded_data["qdot"], unloaded_data["qddot"], unloaded_data["torque"]

#레그레서 행렬 역연산 
def inverse_regressor():
    loaded_time, loaded_q, loaded_qdot, loaded_qddot, loaded_torque = get_sim_loaded_data()
    unloaded_time, unloaded_q, unloaded_qdot, unloaded_qddot, unloaded_torque = get_sim_unloaded_data()
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
    traj_id = int(sys.argv[1])
    mass_hat, cx_hat, cy_hat, IO_hat, Icom_hat = inverse_regressor()
    print(f"추정 파라미터: mass={mass_hat:.4f}, cx={cx_hat:.4f}, cy={cy_hat:.4f}, IO={IO_hat:.4f}, Icom={Icom_hat:.4f}")

    

    

# 모주코를 이용한 계산값과 비교