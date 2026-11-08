from typing import Tuple, Optional, Dict, Any
import numpy as np
from Indy7_2D3DOF import INDY7_2D_JOINTS, compute_torque, compute_regressor, forward_kinematics

def trajectory_evaluator(
    fourier_coeffs_21 : np.ndarray,
    #[ 관절 1 (7개) ]
    #  index  0 ~  3 : a_1,1 ~ a_1,4 (관절 1의 sin 계수 4개)
    # index  4 ~  6 : b_1,1 ~ b_1,3 (관절 1의 cos 계수 3개)
    #[ 관절 2 (7개) ]
    # index 7 ~ 10 : a_2,1 ~ a_2,4 (관절 2의 sin 계수 4개)
    # index 11 ~ 13 : b_2,1 ~ b_2,3 (관절 2의 cos 계수 3개)
    #[ 관절 3 (7개) ]
    # index 14 ~ 17 : a_3,1 ~ a_3,4 (관절 3의 sin 계수 4개)
    # index 18 ~ 20 : b_3,1 ~ b_3,3 (관절 3의 cos 계수 3개)
    time_step : float = 0.01,
    period : float = 10.0, #기본 각주파수 기본 2pi/(10s)= 0.628rad/s
    q0 : Optional[np.ndarray] = None
)-> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    if q0 is None:
        q0 = np.array([0.0, 0.5, -0.5])
    
    N = int(period/time_step) #샘플링 수
    Time_vec = np.linspace(0, period, N, endpoint=False)
    omega = 2 * np.pi / period
    q = np.zeros((N, 3)) + q0
    qdot = np.zeros((N, 3))
    qddot = np.zeros((N, 3))

    fourier_coeffs = np.zeros(30)
    for i in range(3):
        fourier_coeffs[10*i : 10*i+4] = fourier_coeffs_21[7*i : 7*i+4]
        fourier_coeffs[10*i+4] = -np.sum(fourier_coeffs_21[7*i : 7*i+4])
        fourier_coeffs[10*i+5 : 10*i+8] = fourier_coeffs_21[7*i+4 : 7*i+7]
        b1, b2, b3 = fourier_coeffs_21[7*i+4], fourier_coeffs_21[7*i+5], fourier_coeffs_21[7*i+6]
        fourier_coeffs[10*i+8] = float(-(32.0/3.0)*b1 - (14.0/3.0)*b2 - (64.0/27.0)*b3)
        fourier_coeffs[10*i+9] = float((25.0/3.0)*b1 + (10.0/3.0)*b2 + (35.0/27.0)*b3)
    

    for i in range (5):
        q[:,0] += (fourier_coeffs[i]/((i+1)*omega))*np.sin((i+1) * omega * Time_vec)-(fourier_coeffs[i+5]/((i+1)*omega))*np.cos((i+1) * omega * Time_vec)
        q[:,1] += (fourier_coeffs[i+10]/((i+1)*omega))*np.sin((i+1) * omega * Time_vec)-(fourier_coeffs[i+15]/((i+1)*omega))*np.cos((i+1) * omega * Time_vec)
        q[:,2] += (fourier_coeffs[i+20]/((i+1)*omega))*np.sin((i+1) * omega * Time_vec)-(fourier_coeffs[i+25]/((i+1)*omega))*np.cos((i+1) * omega * Time_vec)

        qdot[:,0] += (fourier_coeffs[i])*np.cos((i+1) * omega * Time_vec)+(fourier_coeffs[i+5])*np.sin((i+1) * omega * Time_vec)
        qdot[:,1] += (fourier_coeffs[i+10])*np.cos((i+1) * omega * Time_vec)+(fourier_coeffs[i+15])*np.sin((i+1) * omega * Time_vec)
        qdot[:,2] += (fourier_coeffs[i+20])*np.cos((i+1) * omega * Time_vec)+(fourier_coeffs[i+25])*np.sin((i+1) * omega * Time_vec)

        qddot[:,0] += (-fourier_coeffs[i]*((i+1)*omega))*np.sin((i+1) * omega * Time_vec)+(fourier_coeffs[i+5]*((i+1)*omega))*np.cos((i+1) * omega * Time_vec)
        qddot[:,1] += (-fourier_coeffs[i+10]*((i+1)*omega))*np.sin((i+1) * omega * Time_vec)+(fourier_coeffs[i+15]*((i+1)*omega))*np.cos((i+1) * omega * Time_vec)
        qddot[:,2] += (-fourier_coeffs[i+20]*((i+1)*omega))*np.sin((i+1) * omega * Time_vec)+(fourier_coeffs[i+25]*((i+1)*omega))*np.cos((i+1) * omega * Time_vec)
    
    return Time_vec, q, qdot, qddot   

def compute_condition_number(
    fourier_coeffs: np.ndarray,
    time_step: float = 0.01, 
    period: float = 10.0   
) -> float:

    _, q, qdot, qddot = trajectory_evaluator(
        fourier_coeffs, 
        time_step, 
        period
    )
    N= len(q)

    Y_all = np.zeros((3 * N, 4))
    for i in range(N):
        Y_all[3 * i : 3 * i + 3, :] = compute_regressor(q[i], qdot[i], qddot[i])

    eigen_values = np.linalg.eigvalsh(Y_all.T @ Y_all) #고유값 자동 정렬 작은거->큰거
    sigmas = np.sqrt(np.clip(eigen_values, 0.0, None))
    if sigmas[0] < 1e-12:
        return 1e6
    else: pass
    cond_Y_all = sigmas[-1] / sigmas[0]
    return float(cond_Y_all)

# 제약조건
def evaluate_constraints(
    fourier_coeffs_21: np.ndarray,
    time_step: float = 0.02,
    period: float = 10.0,
    test_payload: Optional[np.ndarray] = None
) -> Dict[str, float]:
    """
    로봇 기계적 제약 조건 마진 계산
    """
    if test_payload is None:
        test_payload = np.array([2.5, 0.04, -0.02, 0.015])

    _, q, qdot, qddot = trajectory_evaluator(fourier_coeffs_21, time_step, period)
    N = len(q)

    joints = INDY7_2D_JOINTS[1:]
    q_max = np.array([j.q_limit_upper for j in joints])
    qdot_max = np.array([j.qdot_limit for j in joints])
    effort_max = np.array([j.effort for j in joints])

    margin_q = np.min(q_max - np.max(np.abs(q), axis=0))
    margin_qdot = np.min(qdot_max - np.max(np.abs(qdot), axis=0))

    torques = np.zeros((N, 3))
    for i in range(N):
        torques[i] = compute_torque(q[i], qdot[i], qddot[i], payload_params=test_payload)
    margin_effort = np.min(effort_max - np.max(np.abs(torques), axis=0))

    y_coords = []
    for i in range(N):
        _, _, p3 = forward_kinematics(q[i])
        y_coords.append(p3[1])
    margin_floor = float(np.min(y_coords) - 0.05)

    return {
        "margin_q": float(margin_q),
        "margin_qdot": float(margin_qdot),
        "margin_effort": float(margin_effort),
        "margin_floor": float(margin_floor),
        "is_valid": bool(margin_q > 0 and margin_qdot > 0 and margin_effort > 0 and margin_floor > 0)
    }


if __name__ == "__main__":
    coeffs = np.array([
        0.042, -0.038,  0.025, -0.012,  0.035, -0.028,  0.018,
        0.035,  0.045, -0.022,  0.015, -0.028,  0.032, -0.014,
       -0.028,  0.032,  0.038, -0.016,  0.022, -0.025,  0.012
    ])
    t, q, qdot, qddot = trajectory_evaluator(coeffs)
    
    # q를 수치 미분한 값과 수식으로 구한 qdot 비교
    qdot_numerical = np.gradient(q[:, 0], t)
    diff = np.max(np.abs(qdot[:, 0] - qdot_numerical))
    print(f"속도 수식 최대 오차: {diff:.2e}")

    cond = compute_condition_number(coeffs)
    print(f"조건수: {cond:.2f}")

    margin_info = evaluate_constraints(coeffs)
    print(f"제약조건 충족 여부: {margin_info['is_valid']}")