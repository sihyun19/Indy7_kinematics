import numpy as np
from config.indy7_dimensions import Joint, INDY7_JOINTS, TCP_OFFSET, TCP_DIR


def htf(R: np.ndarray, p: np.ndarray | None = None) -> np.ndarray: 
    """3x3 회전행렬 R과 3x1 위치벡터 p를 4x4 동차변환행렬로 합성"""
    H = np.eye(4)
    H[:3, :3] = R
    if p is not None:
        H[:3, 3] = p
    return H

def rpy_rot(rpy: np.ndarray) -> np.ndarray:
    """
    URDF RPY 회전각 -> 3x3 회전행렬 R
    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr],
        [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr],
        [   -sp,                  cp * sr,                  cp * cr]
    ])

def rot_z(theta: float) -> np.ndarray:
    """Z축 기준 theta(rad) 회전에 대한 3x3 회전행렬"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0.],
        [s,  c, 0.],
        [0., 0., 1.]
    ])


def fk(
    q: np.ndarray, 
    joints: list[Joint] = INDY7_JOINTS, 
    tcp_offset: float = TCP_OFFSET, 
    tcp_dir: np.ndarray = TCP_DIR
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Indy7 정기구학 (Forward Kinematics) 계산 함수
    
    매개변수:
        q: 6차원 관절각 벡터 (rad)
        joints: 6개 조인트 파라미터 리스트
        tcp_offset: 툴 끝단 길이 (m)
        tcp_dir: 툴 끝단 연장 방향 단위벡터
        
    반환값:
        T: 베이스 기준 최종 TCP의 4x4 행렬
        orig: 각 조인트의 3차원 월드 위치 (3x6 행렬)
        zax: 각 조인트 회전축의 3차원 월드 방향 (3x6 행렬)
    """
    # 1. 누적 변환행렬 초기화 (베이스 좌표계)
    M = np.eye(4)
    
    # 2. 자코비안 계산용
    orig = np.zeros((3, 6))
    zax  = np.zeros((3, 6))
    
    # 3. 6개 링크 순차 누적 루프
    for i, j in enumerate(joints):
        # 이전 관절 -> 이번 관절 좌표 변환
        T_fixed = htf(rpy_rot(j.rpy), j.origin)
        M = M @ T_fixed
        
        # 이번 관절의 월드 좌표계 위치 및 회전축 추출
        orig[:, i] = M[:3, 3]  # 조인트 중심 위치
        zax[:, i]  = M[:3, 2]  # 조인트 z축 방향 (3번째 열)
        
        # (3) 모터 회전 변환 적용 (엔코더 부호 반영)
        theta = j.sign * q[i]
        M = M @ htf(rot_z(theta))

    # 4. 최종 TCP 고정 변환 결합
    T_tcp = htf(np.eye(3), tcp_offset * tcp_dir)
    T = M @ T_tcp
    
    return T, orig, zax

if __name__ == "__main__":
    # Sanity Check: q = [0, 0, 0, 0, 0, 0]일 때 TCP 위치 확인
    q_zero = np.zeros(6)
    T_res, orig_res, zax_res = fk(q_zero)
    tcp_pos = T_res[:3, 3]
    print("=== Indy7 FK Sanity Check ===")
    print("TCP 위치 (XYZ, m):", np.round(tcp_pos, 4))
    print("기대값            : [ 0.     -0.1865  1.3275]")
    
    # 오차 검증
    assert np.allclose(tcp_pos, [0.0, -0.1865, 1.3275], atol=1e-4), "INdy7 규격과 다름. 수치 점검 필요."
    print("위치 확인 완료.")