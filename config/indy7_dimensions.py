"""
Indy7 Kinematic Dimensions & Physical Parameters Specification
CAD 도면 및 URDF 명세 기반 치수 정의 (Zero-Magic-Number)
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class Indy7Geometry:
    """
    Indy7 6축 협동로봇의 CAD 도면 기반 기하학적 치수 파라미터.
    치수가 변경되거나 추가 엔드이펙터(그리퍼 등)를 장착할 때 이 파라미터만 수정하면 됩니다.
    """
    # 1. 고정 회전 상수 (URDF xacro pi/2)
    h: float = 1.570796327

    # 2. 베이스 및 링크 기하 치수 (단위: m)
    h_base1: float = 0.0775      # Joint 1 높이 (베이스 바닥 ~ 1번 축 모터)
    d_shoulder: float = -0.109   # Joint 2 숄더 좌우 편심 오프셋
    h_base2: float = 0.222       # Joint 2 숄더 수직 높이
    l_arm1: float = -0.450       # [가변] Link 2 상완 길이 (2번축 ~ 3번축)
    d_elbow: float = -0.0305     # Joint 3 엘보우 축 오프셋
    l_arm2: float = -0.267       # [가변] Link 3 하완 길이 (3번축 ~ 4번축)
    d_wrist1: float = -0.075     # Joint 4 손목 오프셋 1
    d_wrist2: float = -0.114     # Joint 5 손목 오프셋 2
    h_wrist: float = 0.083       # Joint 5 손목 높이
    l_flange: float = -0.168     # Joint 6 플랜지 링크 오프셋
    h_flange: float = 0.069      # Joint 6 플랜지 높이

    # 3. 툴(엔드이펙터) 오프셋 (기본 60mm)
    l_tcp: float = 0.060         # [가변] 6번 축 플랜지 면 ~ TCP 끝단 거리

    def get_rpy_matrix(self) -> np.ndarray:
        """관절 간 고정 RPY 회전각 (rad)"""
        h = self.h
        return np.array([
            [ 0.0,  0.0,  0.0],
            [   h,    h,  0.0],
            [ 0.0,  0.0,  0.0],
            [  -h,  0.0,    h],
            [   h,    h,  0.0],
            [  -h,  0.0,    h]
        ])

    def get_xyz_matrix(self) -> np.ndarray:
        """관절 간 고정 XYZ 변위 오프셋 (m)"""
        return np.array([
            [ 0.0,           0.0,             self.h_base1 ],
            [ 0.0,           self.d_shoulder, self.h_base2 ],
            [ self.l_arm1,   0.0,             self.d_elbow ],
            [ self.l_arm2,   0.0,             self.d_wrist1],
            [ 0.0,           self.d_wrist2,   self.h_wrist ],
            [ self.l_flange, 0.0,             self.h_flange]
        ])

    def get_tcp_xyz(self) -> np.ndarray:
        """플랜지 좌표계 기준 TCP 끝단 오프셋"""
        return np.array([0.0, 0.0, self.l_tcp])
