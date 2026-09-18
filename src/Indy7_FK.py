import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

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

def rot_rpy(R: np.ndarray) -> np.ndarray:
    """3x3 회전행렬 R -> RPY(Roll, Pitch, Yaw, rad) 추출"""
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    if np.abs(np.cos(pitch)) > 1e-6: #특이점 여부 판단
        roll  = np.arctan2(R[2, 1], R[2, 2])
        yaw   = np.arctan2(R[1, 0], R[0, 0])
    else: #특이점이 맞으면 y를 0으로 두고 r을 계산
        roll = np.arctan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return np.array([roll, pitch, yaw])

def fk(
    q: np.ndarray, 
    joints: list[Joint] = INDY7_JOINTS, 
    tcp_offset: float = TCP_OFFSET, 
    tcp_dir: np.ndarray = TCP_DIR
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray]]:
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

    # 3. 각 링크 변환행렬 별도 저장->시뮬레이션 구축용
    link_transforms = [M.copy()]
    
    # 4. 6개 링크 순차 누적 루프
    for i, j in enumerate(joints):
        # 이전 관절 -> 이번 관절 좌표 변환
        T_fixed = htf(rpy_rot(j.rpy), j.origin)
        M = M @ T_fixed
        
        # 이번 관절의 월드 좌표계 위치 및 회전축 추출
        orig[:, i] = M[:3, 3]  # 조인트 중심 위치
        zax[:, i]  = M[:3, 2]  # 조인트 z축 방향 (3번째 열)
        
        # 모터 회전 변환 적용 (엔코더 부호 반영)
        theta = j.sign * q[i]
        M = M @ htf(rot_z(theta))
        link_transforms.append(M.copy())#회전 적용 이후의 링크 변환행렬 저장(시뮬레이션용)

    # 5. 최종 TCP 고정 변환 결합
    T_tcp = htf(np.eye(3), tcp_offset * tcp_dir)
    T = M @ T_tcp
    
    return T, orig, zax, link_transforms

def evaluate_fk(q_deg: list[float] | np.ndarray, name: str = "Pose") -> dict:
    """도(deg) 단위 각도를 받아 FK 계산 후 결과 출력"""
    q_rad = np.radians(q_deg)
    T, orig, zax, link_transforms = fk(q_rad)
    tcp_pos = T[:3, 3]
    tcp_rpy = rot_rpy(T[:3, :3])
    
    print(f"\n=== {name} ===")
    print("TCP 위치 (XYZ, mm):", np.round(tcp_pos * 1000.0, 2))
    print("TCP 회전 (RPY, deg):", np.round(np.degrees(tcp_rpy), 2))
    
    return {
        "T": T,
        "tcp_pos": tcp_pos,
        "tcp_rpy": tcp_rpy,
        "orig": orig,
        "zax": zax,
        "link_transforms": link_transforms
    }

if __name__ == "__main__":
    # Sanity Check: q = [0, 0, 0, 0, 0, 0]일 때 TCP 위치 확인
    q_zero = np.zeros(6)
    T_res, orig_res, zax_res, link_transforms_res = fk(q_zero)
    tcp_pos = T_res[:3, 3]
    print("=== Indy7 FK Sanity Check ===")
    print("TCP 위치 (XYZ, m):", np.round(tcp_pos, 4))
    print("기대값            : [ 0.     -0.1865  1.3275]")
    print(f"저장된 링크 변환행렬 개수: {len(link_transforms_res)}개 (Base + Link)")
    
    # 오차 검증
    assert np.allclose(tcp_pos, [0.0, -0.1865, 1.3275], atol=1e-4), "INdy7 규격과 다름. 수치 점검 필요."
    print("위치 확인 완료.")

    print("=== Indy7 FK 각도 입력===")
    print("6개 관절 각도(deg)를 띄어쓰기로 구분하여 입력 후 q 입력")
    print("입력 예시: 0 -30 60 0 45 0\n")
    while True:
        user_input = input(">> 각도 입력 [J1 J2 J3 J4 J5 J6]: ").strip()
        if user_input.lower() == 'q':
            print("종료합니다.")
            break
        
        try:
            q_list = [float(val) for val in user_input.split()]
            if len(q_list) != 6:
                print(f"각도는 정확히 6개여야 합니다. (현재 입력: {len(q_list)}개)")
                continue
            evaluate_fk(q_list, name="입력된 자세")
        except ValueError:
            print("[오류] 올바른 숫자 형식으로 입력해주세요.")