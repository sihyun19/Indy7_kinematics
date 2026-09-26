import numpy as np
from src.Indy7_FK import fk

def tcp_jacobian(q: np.ndarray) -> np.ndarray:
    """tcp기준 6*6 자코비안 계산 함수"""
    T_tcp, p_joints, z_axes, p_com, T_links = fk(q)
    p_tcp = T_tcp[:3, 3]  # TCP 위치
    
    J = np.zeros((6, 6))
    for j in range(6):
        z_j = z_axes[:, j]
        p_j = p_joints[:, j]
        J[:3, j] = np.cross(z_j, p_tcp - p_j) #선속도 v=rw
        J[3:, j] = z_j #각속도 w
    return J

def link_jacobian(q:np.ndarray) -> list[np.ndarray]:
    """
    각 링크 별 무게 중심에 대한 6*6 자코비안 계산 함수
    반환값: 
        J_links: 6개 링크에 대한 6*6 자코비안 리스트
    """
    T_tcp, p_joints, z_axes, p_com, T_links = fk(q)
    J_links = []
    for i in range(6):

        p_ci = p_com[:, i]  # 링크 무게중심 위치
        J_i = np.zeros((6, 6))

        for j in range(i+1):
            z_j = z_axes[:, j]
            p_j = p_joints[:, j]

            J_i[:3, j] = np.cross(z_j, p_ci - p_j) #선속도 v=rw
            J_i[3:, j] = z_j #각속도 w
        J_links.append(J_i)
    return J_links