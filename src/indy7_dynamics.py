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



