import numpy as np
from Indy7_2D3DOF import INDY7_2D_JOINTS, compute_regressor

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
    period : float = 10.0 #기본 각주파수 기본 2pi/(10s)= 0.628rad/s
)-> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    
    N = int(period/time_step) #샘플링 수
    Time_vec = np.linspace(0, period, N, endpoint=False)
    q0 = np.array([0.0, 0.5, -0.5])
    omega = 2 * np.pi / period
    q = np.zeros((N, 3)) + q0
    qdot = np.zeros((N, 3))
    qddot = np.zeros((N, 3))

    fourier_coeffs = np.zeros(30)
    for i in range(3):
        fourier_coeffs[10*i : 10*i+4] = fourier_coeffs_21[7*i : 7*i+4]
        fourier_coeffs[10*i+4] = -np.sum(fourier_coeffs_21[7*i : 7*i+4])
        fourier_coeffs[10*i+5 : 10*i+8] = fourier_coeffs_21[7*i+4 : 7*i+7]
        fourier_coeffs[10*i+8] = float((100/9)*omega*q0[i] - (32/3)*fourier_coeffs_21[7*i+4] - (14/3)*fourier_coeffs_21[7*i+5] - (64/27)*fourier_coeffs_21[7*i+6] )
        fourier_coeffs[10*i+9] = float( -(80/9)*omega*q0[i] + (25/3)*fourier_coeffs_21[7*i+4] + (10/3)*fourier_coeffs_21[7*i+5] + (35/27)*fourier_coeffs_21[7*i+6] )
    

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
        Y_all[3*i:3*i+3,:] = compute_regressor(q[i], qdot[i], qddot[i])

    eigen_values = np.linalg.eigvalsh(Y_all.T @ Y_all) #고유값 자동 정렬 작은거->큰거
    sigmas = np.sqrt(np.clip(eigen_values, 0.0, None))
    if sigmas[0] < 1e-12:
        return 1e6
    else: pass
    cond_Y_all = sigmas[-1] / sigmas[0]
    return float(cond_Y_all)

#제약조건
def evaluate_constraints(
        
)

#메인함수(위에것들 묶어서 최적화된 궤적 생성)
    # 매 스탭 제약 조건에서 위반시 해당 조건에 대해 음수 잔차 반환-> 경사강하법으로 재생성
    # t=0일때, sin=0, cos=1이고, b에 대한 식 3개 나옴-> 2개 결정하면 3개 자동 결정->3x3=9개 소거하고 21개만 지정하면 됨

if __name__ == "__main__":
    coeffs = np.ones(21)
    t, q, qdot, qddot = trajectory_evaluator(coeffs)
    
    # q를 수치 미분한 값과 수식으로 구한 qdot 비교
    qdot_numerical = np.gradient(q[:, 0], t)
    diff = np.max(np.abs(qdot[:, 0] - qdot_numerical))
    print(f"속도 수식 최대 오차: {diff:.2e}")  # 1e-3 이하로 일치해야 정상

    cond = compute_condition_number(coeffs)
    print(f"조건수: {cond:.2f}")