'''
SLSQP솔버
'''
import numpy as np
from Indy7_2D3DOF import INDY7_2D_JOINTS, compute_torque, compute_regressor
from scipy.optimize import minimize

# 푸리에 계수에 따른 q, qdot, qddot 행렬 생성
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
    q0 : np.ndarray | None = None
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

# 목적함수 역할
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

# 제약조건, 다운샘플링 후 12*50=600 벡터  출력
def evaluate_constraints(
    fourier_coeffs_21: np.ndarray,
    time_step: float = 0.2,
    period: float = 10.0,
) -> np.ndarray:
  
    _, q, qdot, qddot = trajectory_evaluator(fourier_coeffs_21, time_step, period)
    N = len(q)

    q_max = np.array([0.8*j.q_max for j in INDY7_2D_JOINTS[1:]])
    q_min = np.array([0.8*j.q_min for j in INDY7_2D_JOINTS[1:]])
    qdot_max = np.array([0.8*j.qdot_max for j in INDY7_2D_JOINTS[1:]])
    torque_max = np.array([0.8*j.effort for j in INDY7_2D_JOINTS[1:]])

    margin_q_upper = ((q_max - q) / q_max).flatten()
    margin_q_lower = ((q - q_min) / np.abs(q_min)).flatten()
    margin_qdot    = ((qdot_max - np.abs(qdot)) / qdot_max).flatten()
    torque_all =   np.zeros((N, 3))
    for i in range(N):
        torque_all[i,:] = compute_torque(q[i], qdot[i], qddot[i])
    margin_torque  = ((torque_max - np.abs(torque_all)) / torque_max).flatten()
        
    return np.concatenate([margin_q_upper, margin_q_lower, margin_qdot, margin_torque])

# SLSQP솔버 조립
def slsqp(
    coeffs_21: np.ndarray | None = None,
    maxiter: int = 100,
    eps: float = 1e-4,
    ftol: float = 1e-4
):
    if coeffs_21 is None:
        coeffs_21 = np.array([
                0.042, -0.038,  0.025, -0.012,  0.035, -0.028,  0.018,
                0.035,  0.045, -0.022,  0.015, -0.028,  0.032, -0.014,
               -0.028,  0.032,  0.038, -0.016,  0.022, -0.025,  0.012
            ])
    t, q, qdot, qddot = trajectory_evaluator(coeffs_21)

    count = 0
    def print_count(xk):
        nonlocal count
        count += 1
        current_cond = compute_condition_number(xk)
        print(f"  [반복 {count:2d}] 현재 조건수: {current_cond:.4f}")

    result = minimize(
        fun = compute_condition_number,
        x0 = coeffs_21,
        method = 'SLSQP',
        constraints = { 'type' : 'ineq', 'fun' : evaluate_constraints },
        callback = print_count,
        options= {
            'maxiter': maxiter,
            'disp': True,
            'eps': eps,
            'ftol': ftol
        }
    )
    return result
    
if __name__ == "__main__":
    print(f"--------test--------")
    coeffs_test = np.array([
        0.042, -0.038,  0.025, -0.012,  0.035, -0.028,  0.018,
        0.035,  0.045, -0.022,  0.015, -0.028,  0.032, -0.014,
       -0.028,  0.032,  0.038, -0.016,  0.022, -0.025,  0.012
    ])
    t_test, q_test, qdot_test, qddot_test = trajectory_evaluator(coeffs_test)
    
    
    # q를 수치 미분한 값과 수식으로 구한 qdot 비교
    qdot_test_numerical = np.gradient(q_test[:, 0], t_test)
    diff = np.max(np.abs(qdot_test[:, 0] - qdot_test_numerical))
    print(f"속도 수식 최대 오차: {diff:.2e}")

    cond = compute_condition_number(coeffs_test)
    print(f"조건수: {cond:.2f}")

    # 제약조건 벡터 검증
    c_vals_test = evaluate_constraints(coeffs_test)
    print(f"제약조건 벡터 형태: {c_vals_test.shape}") # (2400,)
    print(f"최소 마진(음수면 위반): {np.min(c_vals_test):.4f}")

    print(f"--------실행--------")
    res= slsqp(coeffs_test)
    print(f"성공 여부      : {res.success}")
    print(f"--------결과--------")
    if res.success:
        
        
        print(f"반복횟수       : {res.nit:.f}")
        print(f"최적화 벡터     : {res.x:.f}")
        print(f"최적화 조건수   : {res.fun:.f}")
    else:
        print(f"최적화 실패: {res.message}")