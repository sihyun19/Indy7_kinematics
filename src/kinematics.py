"""
Indy7 Parametric Forward Kinematics and Geometric Jacobian Engine
순수 NumPy 기반 동차변환행렬 및 기하 자코비안 연산 엔진
"""

import numpy as np
from typing import Tuple
import sys
import os

# Add parent directory to sys.path for config import
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.indy7_dimensions import Indy7Geometry


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    """URDF fixed-axis roll-pitch-yaw 회전행렬 (R = Rz(y) * Ry(p) * Rx(r))"""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
        [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
        [    -sp,                 cp * sr,                 cp * cr]
    ])


def htf(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    """3x3 회전행렬과 3x1 위치벡터로 4x4 동차변환행렬 생성"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


class Indy7Kinematics:
    def __init__(self, geometry: Indy7Geometry = None):
        """
        geometry: Indy7Geometry 인스턴스 (생략 시 기본 CAD 치수 적용)
        """
        self.geo = geometry if geometry is not None else Indy7Geometry()
        self.RPY = self.geo.get_rpy_matrix()
        self.XYZ = self.geo.get_xyz_matrix()
        self.TCP_XYZ = self.geo.get_tcp_xyz()

    def update_geometry(self, geometry: Indy7Geometry):
        """링크 치수나 툴 오프셋 변경 시 파라미터 갱신"""
        self.geo = geometry
        self.RPY = self.geo.get_rpy_matrix()
        self.XYZ = self.geo.get_xyz_matrix()
        self.TCP_XYZ = self.geo.get_tcp_xyz()

    def forward_kinematics(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        q: 6x1 관절 각도 벡터 (rad)
        Returns:
            T: 4x4 동차변환행렬 (World -> TCP frame)
            J: 6x6 기하 자코비안 (World 좌표계 기준, 상위 3행: 선속도, 하위 3행: 각속도)
            axes: 6x6 관절 회전축 정보 (상위 3행: 관절 원점, 하위 3행: Z 회전축 방향)
        """
        q = np.asarray(q, dtype=np.float64).flatten()
        assert len(q) == 6, f"Indy7은 6자유도입니다. 입력 관절 수: {len(q)}"

        M = np.eye(4)
        orig = np.zeros((3, 6))
        zax = np.zeros((3, 6))

        for i in range(6):
            # 1. 고정 오프셋 변환 (이전 관절 -> 현재 관절 원점)
            R_fixed = rpy_matrix(self.RPY[i, 0], self.RPY[i, 1], self.RPY[i, 2])
            M = M @ htf(R_fixed, self.XYZ[i])

            # 2. 관절 원점 및 회전축 저장
            orig[:, i] = M[:3, 3]
            zax[:, i] = M[:3, 2]

            # 3. 능동 관절 회전 (Z축 기준 q_i 회전)
            c, s = np.cos(q[i]), np.sin(q[i])
            R_joint = np.array([
                [c, -s, 0.0],
                [s,  c, 0.0],
                [0.0, 0.0, 1.0]
            ])
            M = M @ htf(R_joint, np.zeros(3))

        # 4. 엔드이펙터(TCP) 고정 변환
        T = M @ htf(np.eye(3), self.TCP_XYZ)

        # 5. 기하 자코비안 계산: J_v = z_i x (p_e - p_i), J_w = z_i
        pe = T[:3, 3]
        J = np.zeros((6, 6))
        for i in range(6):
            J[:3, i] = np.cross(zax[:, i], pe - orig[:, i])
            J[3:, i] = zax[:, i]

        axes = np.vstack([orig, zax])
        return T, J, axes
