from dataclasses import dataclass, field
from typing import List
import numpy as np
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parent.parent))
from config.indy7_dimensions import Joint, INDY7_JOINTS, TCP_OFFSET, TCP_DIR

#파라미터 정리
@dataclass
class Joint2D:
    name: str
    lenth: float
    lenth_dir: np.ndarray | None = None
    l_com: float=0.5
    mass: float = 0.0
    inertia_zz : float = 2.0

INDY7_2D_JOINTS = [
    # base->joint1
    Joint2D(
        name = "joint1",
        lenth = INDY7_JOINTS[0].len_val + INDY7_JOINTS[1].len_val
    ),
    Joint2D(
            name = "joint2",
            lenth = INDY7_JOINTS[2].len_val,
            mass = INDY7_JOINTS[2].mass 
        ),
    Joint2D(
            name = "joint3",
            lenth =  INDY7_JOINTS[3].len_val + INDY7_JOINTS[4].len_val,
            mass = INDY7_JOINTS[3].mass + INDY7_JOINTS[4].mass
        ),
    Joint2D(
            name = "tcp",
            lenth = INDY7_JOINTS[5].len_val + TCP_OFFSET,
            mass = INDY7_JOINTS[5].mass
        )
]

print(f"base= {INDY7_2D_JOINTS[0].lenth}, L1= {INDY7_2D_JOINTS[1].lenth}, L2= {INDY7_2D_JOINTS[2].lenth}, L3= {INDY7_2D_JOINTS[3].lenth}")

def compute_torque(
    q: np.ndarray, 
    qdot: np.ndarray, 
    qddot: np.ndarray, 
    payload_params: np.ndarray | None = None, # mass, c_x, c_y, Izz_com 순서 
    friction_params: np.ndarray | None = None
) -> np.ndarray:
    """
    2D 3자유도 평면 로봇 + 페이로드 + 마찰의 전체 역동역학 토크 계산
    
    입력:
      q, qdot, qddot     : (3,) 각도, 각속도, 각가속도 [rad, rad/s, rad/s^2]
      payload_params : (4,) [m, c_x, c_y, I_O] (미입력 시 0으로 처리)
      friction_params: (6,) [fv1, fc1, fv2, fc2, fv3, fc3] (미입력 시 0으로 처리)
      
    출력:
      tau_total      : (3,) 각 관절에 요구되는 전체 토크 [N*m]
    """
    phi = np.cumsum(q) 
    phi_dot = np.cumsum(qdot) 

    L  = np.array([j.lenth for j in INDY7_2D_JOINTS[1:]])
    Lc = np.array([j.l_com for j in INDY7_2D_JOINTS[1:]])
    m  = np.array([j.mass for j in INDY7_2D_JOINTS[1:]])
    I  = np.array([j.inertia_zz for j in INDY7_2D_JOINTS[1:]])

    T = np.column_stack([-np.sin(phi), np.cos(phi)]) 
    N = np.column_stack([-np.cos(phi), -np.sin(phi)])
    u, uc = L[:, None] * T, Lc[:, None] * T
    u_dot, uc_dot = (phi_dot * L)[:, None] * N, (phi_dot * Lc)[:, None] * N

    Jv_list = []
    Jv_dot_list = []
    for i in range(3)
        Jv_i = np.zeros((2, 3))
        Jv_dot_i = np.zeros((2, 3))
        for k in range(i):
            Jv_i[:, k] = np.sum(u[k:i], axis=0) + uc[i]
            Jv_dot_i[:, k] = np.sum(u_dot[k:i], axis=0) + uc_dot[i]
        Jv_i[:, i] = uc[i]
        Jv_list.append(Jv_i)
        Jv_dot_i[:, i] = uc_dot[i]
        Jv_dot_list.append(Jv_dot_i)

    #로봇
    M_robot = np.zeros((3,3))
    C_robot = np.zeros((3,3))
    g_robot = np.zeros(3)
    for i in range(3):
        M_robot += m[i] * ( Jv_list[i].T @ Jv_list[i])
        M_robot[:i + 1, :i + 1] += I[i]                              #관성항
        C_robot += m[i] * (Jv_list[i].T @ Jv_dot_list[i])            #코리올리항
        g_robot += Jv_list[i].T @ (m[i] * np.array([0.0, 9.80665])) # 중력항

    M_payload = np.zeros((3, 3))
    if payload_params is not None:
        u_payload = payload_params[1] * T[3] +  payload_params[2] * N[3]
        Jv_payload = Jv_list[3] + np.column_stack([u_payload, u_payload, u_payload])
        m_payload = payload_params[0]
        I_payload = payload_params[3]
        # 페이로드 관성 구현 진행예정
    M_total = M_robot + M_payload

