from dataclasses import dataclass, field
from typing import List, Tuple, Optional
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
    q_limit_upper : float = 0.0
    q_limit_lower : float = 0.0 
    qdot_limit : float = 0.0 
    effort : float = 0.0 

INDY7_2D_JOINTS = [
    Joint2D(
        name = "joint0",#base
        lenth = INDY7_JOINTS[0].len_val + INDY7_JOINTS[1].len_val
    ),
    Joint2D(
        name = "joint1",
        lenth = INDY7_JOINTS[2].len_val,
        mass = INDY7_JOINTS[2].mass,
        q_limit_upper = 3.0543261909900767,
        q_limit_lower = -3.0543261909900767,
        qdot_limit = 2.6179938779914944,
        effort = 431.97
    ),
    Joint2D(
        name = "joint2",
        lenth =  INDY7_JOINTS[3].len_val + INDY7_JOINTS[4].len_val,
        mass = INDY7_JOINTS[3].mass + INDY7_JOINTS[4].mass,
        q_limit_upper = 3.0543261909900767,
        q_limit_lower = -3.0543261909900767,
        qdot_limit = 2.6179938779914944,
        effort = 197.23
    ),
    Joint2D(
        name = "joint3",
        lenth = INDY7_JOINTS[5].len_val + TCP_OFFSET,
        mass = INDY7_JOINTS[5].mass,
        q_limit_upper = 3.0543261909900767,
        q_limit_lower = -3.0543261909900767,
        qdot_limit = 3.141592653589793,
        effort = 79.79
    )
]

DEFAULT_FRICTION_PARAMS = np.array([1.25, 1.80, 0.85, 1.20, 0.45, 0.65])


def forward_kinematics(q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    2D 3자유도 순기구학 (각 관절 및 엔드이펙터 2D 좌표 반환)
    기준 좌표계: Base 관절 위치 (0, L0)
    """
    L0 = INDY7_2D_JOINTS[0].lenth
    L = np.array([j.lenth for j in INDY7_2D_JOINTS[1:]])
    phi = np.cumsum(q)
    T = np.column_stack([-np.sin(phi), np.cos(phi)])

    p0 = np.array([0.0, L0])
    p1 = p0 + L[0] * T[0]
    p2 = p1 + L[1] * T[1]
    p3 = p2 + L[2] * T[2]
    return p1, p2, p3

def compute_torque(
    q: np.ndarray, 
    qdot: np.ndarray, 
    qddot: np.ndarray, 
    payload_params: np.ndarray | None = None, 
    friction_params: np.ndarray | None = None
) -> np.ndarray:
    """
    2D 3자유도 평면 로봇 + 페이로드 + 마찰의 전체 역동역학 토크 계산
    
    입력:
      q, qdot, qddot     : (3,) 각도, 각속도, 각가속도 [rad, rad/s, rad/s^2]
      payload_params : (4,) [m, c_x, c_y, I_o] (미입력 시 0으로 처리)
      friction_params: (6,) [fv1, fc1, fv2, fc2, fv3, fc3] (미입력 시 0으로 처리)
      
    출력:
      Torque      : (3,) 각 관절에 요구되는 전체 토크 [N*m]
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
    for i in range(3):
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
    
    #페이로드   
    M_payload = np.zeros((3, 3))
    C_payload = np.zeros((3, 3))
    g_payload = np.zeros(3)
    if payload_params is not None:
        u_payload = payload_params[1] * T[2] +  payload_params[2] * N[2]
        u_dot_payload = phi_dot[-1] * ( payload_params[1] * N[2] - payload_params[2] * T[2] )
        Jv_payload = Jv_list[2] + np.column_stack([-uc[2]+u[2]+u_payload, -uc[2]+u[2]+u_payload, -uc[2]+u[2]+u_payload])
        Jv_dot_payload = Jv_dot_list[2] + np.column_stack([-uc_dot[2]+u_dot[2]+u_dot_payload, -uc_dot[2]+u_dot[2]+u_dot_payload, -uc_dot[2]+u_dot[2]+u_dot_payload])
        m_payload = payload_params[0]
        I_payload = payload_params[3] - m_payload * (payload_params[1]**2 + payload_params[2]**2)

        M_payload = m_payload * ( Jv_payload.T @ Jv_payload ) + I_payload * np.ones((3, 3))
        C_payload = m_payload * (Jv_payload.T @ Jv_dot_payload )
        g_payload = Jv_payload.T @ (m_payload * np.array([0.0, 9.80665]))

    M_total = M_robot + M_payload
    C_total = C_robot + C_payload
    g_total = g_robot + g_payload

    Torque = M_total @ qddot + C_total @ qdot + g_total
    return Torque

def compute_regressor(
    q: np.ndarray, 
    qdot: np.ndarray, 
    qddot: np.ndarray, 
) -> np.ndarray:
    """          
        출력:
          Y               : (3, 4) 파라미터 m, mc_x,mc_y, I_o)에 대한 선형 회귀행렬
    """
    Y = np.zeros((3,4))
    torque_robot = compute_torque( q, qdot, qddot)
    Y[:,0] = compute_torque( q, qdot, qddot, np.array([1, 0, 0, 0]) )- torque_robot
    Y[:,1] = compute_torque( q, qdot, qddot, np.array([1, 1, 0, 0]) )- Y[:,0] - torque_robot
    Y[:,2] = compute_torque( q, qdot, qddot, np.array([1, 0, 1, 0]) )- Y[:,0] - torque_robot
    Y[:,3] = compute_torque( q, qdot, qddot, np.array([0, 0, 0, 1]) )- torque_robot
    return Y

if __name__ == "__main__":
    q, qd, qdd = np.array([0.2, 0.4, -0.3]), np.array([0.5, -0.2, 0.8]), np.array([1.2, -0.6, 0.4])
    params = np.array([2.5, 0.04, -0.02, 0.015])  # m, cx, cy, Io

    tau_pay_direct = compute_torque(q, qd, qdd, params) - compute_torque(q, qd, qdd)
    Y = compute_regressor(q, qd, qdd)
    theta = np.array([params[0], params[0]*params[1], params[0]*params[2], params[3]])
    tau_pay_linear = Y @ theta

    error = np.max(np.abs(tau_pay_direct - tau_pay_linear))
    print(f"선형화 정합성 최대 오차: {error:.2e} N*m (통과 여부: {error < 1e-12})")