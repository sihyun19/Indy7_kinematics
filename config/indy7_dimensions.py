from dataclasses import dataclass, field
from typing import List
import numpy as np

# 3차원 6방향 단위벡터 및 영벡터
X_POS = np.array([ 1.,  0.,  0.])
X_NEG = np.array([-1.,  0.,  0.])
Y_POS = np.array([ 0.,  1.,  0.])
Y_NEG = np.array([ 0., -1.,  0.])
Z_POS = np.array([ 0.,  0.,  1.])
Z_NEG = np.array([ 0.,  0., -1.])
ZERO  = np.array([ 0.,  0.,  0.])

H = 1.570796327  # 90deg (rad)

@dataclass(frozen=True)
class Joint:

    #조인트 기하학 파라미터
    #- len_val, len_dir   : 주 링크 길이(m) 및 방향 단위벡터
    #- off_val, off_dir   : 관절 편심 단차(m) 및 방향 단위벡터
    #- rpy                : 조립 방위각 (roll, pitch, yaw)
    #- axis               : 회전축 단위벡터
    #- sign               : 모터 엔코더 회전 부호 (+1 또는 -1)
    
    name: str
    len_val: float
    len_dir: np.ndarray
    off_val: float
    off_dir: np.ndarray
    rpy: np.ndarray
    axis: np.ndarray = field(default_factory=lambda: Z_POS.copy())
    sign: int = 1
    com: np.ndarray = field(default_factory=lambda: ZERO.copy())
    mass: float = 0.0

    @property
    def origin(self) -> np.ndarray:
        #길이와 단차 벡터를 더해서 3차원 상대 위치(XYZ) 반환
        return self.len_val * self.len_dir + self.off_val * self.off_dir

# Indy7 파라미터 (base, link1 ~ link6, tcp)
INDY7_JOINTS = [
    # Joint 1: base -> link1
    Joint(
        name="joint1",
        len_val=0.0775, len_dir=Z_POS, #베이스 위치 정보
        off_val=0.0,    off_dir=ZERO,
        rpy=np.array([0., 0., 0.]),
        axis=Z_POS, sign=1,
        com= np.array([1.0e-06, -0.038646, 0.150736]), mass=11.44444535
    ),
    # Joint 2: link1 -> link2
    Joint(
        name="joint2",
        len_val=0.2225, len_dir=Z_POS, #link1 길이
        off_val=0.1090, off_dir=Y_NEG,
        rpy=np.array([H, H, 0.]),
        axis=Z_POS, sign=1,
        com=np.array([-0.248250, -2.0e-06, 0.076528]), mass=5.84766553
    ),
    # Joint 3: link2 -> link3
    Joint(
        name="joint3",
        len_val=0.4500, len_dir=X_NEG, #link2 길이
        off_val=0.0305, off_dir=Z_NEG,
        rpy=np.array([0., 0., 0.]),
        axis=Z_POS, sign=1,
        com=np.array([-0.129304, -1.3988e-10, -0.072209]), mass=2.68206064
    ),
    # Joint 4: link3 -> link4
    Joint(
        name="joint4",
        len_val=0.2670, len_dir=X_NEG, #link3 길이
        off_val=0.0750, off_dir=Z_NEG,
        rpy=np.array([-H, 0., H]),
        axis=Z_POS, sign=1,
        com=np.array([-2.0e-06, -0.035728, 0.051948]), mass=2.12987371
    ),
    # Joint 5: link4 -> link5
    Joint(
        name="joint5",
        len_val=0.0830, len_dir=Z_POS, #link4 길이
        off_val=0.1140, off_dir=Y_NEG,
        rpy=np.array([H, H, 0.]),
        axis=Z_POS, sign=1,
        com=np.array([-0.045433, 3.0e-06, 0.062349]), mass=2.22412271
    ),
    # Joint 6: link5 -> link6
    Joint(
        name="joint6",
        len_val=0.1680, len_dir=X_NEG, #link5 길이
        off_val=0.0690, off_dir=Z_POS,
        rpy=np.array([-H, 0., H]),
        axis=Z_POS, sign=1,
        com=np.array([-1.5085e-10, -0.000421, 0.031452]), mass=0.38254932
    ),
]

# Link 6 -> TCP (고정 툴)
TCP_OFFSET = 0.0600  # 60 mm
TCP_DIR = Z_POS
TCP_MASS = 0.0       
TCP_COM = ZERO.copy() # TCP 좌표계 기준 툴 무게중심 오프셋 (m)