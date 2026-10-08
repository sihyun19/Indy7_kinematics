import numpy as np
from config.indy7_dimensions import Joint, INDY7_JOINTS
from src.Indy7_FK import fk
from src.Indy7_Jacobian import link_jacobian

def inertia_mat(
    q:np.ndarray,
    joints: list[Joint] = INDY7_JOINTS, 
)-> np.ndarray:
    """
    로봇 전체 질량/관성 행렬 M(q) 계산
    반환값: M_q
    """
    _, _, _, _, T_links = fk(q)
    M_q =np.zeros((6, 6))
    J = link_jacobian(q)
    for i, j in enumerate(joints):

        R_i=T_links[i+1][:3,:3]
        J_vi= J[i][:3,:]
        J_wi = J[i][3:,:]
        I_world_i =  R_i @ j.inertia @ R_i.T
        M_q += j.mass * J_vi.T @ J_vi +  J_wi.T @ I_world_i @ J_wi
    return M_q

def coriolis_vec(
        q:np.ndarray,
        q_dot:np.ndarray,
        joints: list[Joint] = INDY7_JOINTS
)-> np.ndarray:
    """
    코리올리 벡터 C(q, q_dot) 계산
    반환값: C_q_qdot
    """
    C_q_qdot = np.zeros(6)
    
    dM_dq = np.zeros((6, 6, 6))
    for k in range(6):

        q_plus = q.copy()
        q_minus = q.copy()
        q_plus[k] += 1e-6
        q_minus[k] -= 1e-6

        M_q_plus = inertia_mat(q_plus, joints)
        M_q_minus = inertia_mat(q_minus, joints)

        dM_dq[k,:,:] = (M_q_plus - M_q_minus) / (2 * 1e-6)

    for i in range(6):
        for j in range(6):
            for k in range(6):
                c_ijk = 0.5 * (dM_dq[k, i, j] + dM_dq[j, i, k] - dM_dq[i, j, k]) * q_dot[j] * q_dot[k]
                C_q_qdot[i] += c_ijk
    return C_q_qdot

def gravity_vec(
        q:np.ndarray,
        joints: list[Joint] = INDY7_JOINTS
)-> np.ndarray:
    """
    중력 벡터 g(q) 계산
    반환값: g_q
    """
    g_q = np.zeros(6)
    J = link_jacobian(q)
    for i, j in enumerate(joints):
        J_vi= J[i][:3,:]
        g_q += j.mass * J_vi.T @ np.array([0, 0, 9.80665])  

    return g_q

def torque_vec(
        q:np.ndarray,
        q_dot:np.ndarray,
        q_ddot:np.ndarray,
        joints: list[Joint] = INDY7_JOINTS
)-> np.ndarray:
    """
    관절 토크 벡터 계산
    반환값: tau
    """
    M_q = inertia_mat(q, joints)
    C_q_qdot = coriolis_vec(q, q_dot, joints)
    g_q = gravity_vec(q, joints)

    tau = M_q @ q_ddot + C_q_qdot + g_q
    return tau