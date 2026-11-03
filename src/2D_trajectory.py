import numpy as np
from Indy7_2D3DOF import INDY7_2D_JOINTS, compute_regressor

def trajectory_evaluator(
    fourier_coeffs : np.ndarray,
    #[ 관절 1 (10개) ]
    #  index  0 ~  4 : a_1,1 ~ a_1,5 (관절 1의 sin 계수 5개)
    # index  5 ~  9 : b_1,1 ~ b_1,5 (관절 1의 cos 계수 5개)
    #[ 관절 2 (10개) ]
    # index 10 ~ 14 : a_2,1 ~ a_2,5 (관절 2의 sin 계수 5개)
    # index 15 ~ 19 : b_2,1 ~ b_2,5 (관절 2의 cos 계수 5개)
    #[ 관절 3 (10개) ]
    # index 20 ~ 24 : a_3,1 ~ a_3,5 (관절 3의 sin 계수 5개)
    # index 25 ~ 29 : b_3,1 ~ b_3,5 (관절 3의 cos 계수 5개)
    Time_steps : float = 0.01,
    period : float = 10.0 #기본 각주파수 기본 2pi/(10s)= 0.628rad/s
)-> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    
    N = int(period/Time_steps) #샘플링 수
    Time_vec = np.linspace(0, period, N, endpoint=False)
    q0 = np.array([0.0, 0.5, -0.5])
    omega = 2 * np.pi / period
    q = np.zeros((N, 3)) + q0
    qdot = np.zeros((N, 3))
    qddot = np.zeros((N, 3))

    for i in range (5):
        q[:,0] += (fourier_coeffs[i]/((i+1)*omega))*np.sin((i+1) * omega * Time_vec)-(fourier_coeffs[i+5]/((i+1)*omega))*np.cos((i+1) * omega * Time_vec)
        q[:,1] += (fourier_coeffs[i+10]/((i+1)*omega))*np.sin((i+1) * omega * Time_vec)-(fourier_coeffs[i+15]/((i+1)*omega))*np.cos((i+1) * omega * Time_vec)
        q[:,2] += (fourier_coeffs[i+20]/((i+1)*omega))*np.sin((i+1) * omega * Time_vec)-(fourier_coeffs[i+25]/((i+1)*omega))*np.cos((i+1) * omega * Time_vec)

        qdot[:,0] += (fourier_coeffs[i])*np.sin((i+1) * omega * Time_vec)+(fourier_coeffs[i+5])*np.cos((i+1) * omega * Time_vec)
        qdot[:,1] += (fourier_coeffs[i+10])*np.sin((i+1) * omega * Time_vec)+(fourier_coeffs[i+15])*np.cos((i+1) * omega * Time_vec)
        qdot[:,2] += (fourier_coeffs[i+20])*np.sin((i+1) * omega * Time_vec)+(fourier_coeffs[i+25])*np.cos((i+1) * omega * Time_vec)

        qddot[:,0] += -(fourier_coeffs[i]*((i+1)*omega))*np.sin((i+1) * omega * Time_vec)+(fourier_coeffs[i+5]*((i+1)*omega))*np.cos((i+1) * omega * Time_vec)
        qddot[:,1] += -(fourier_coeffs[i+10]*((i+1)*omega))*np.sin((i+1) * omega * Time_vec)+(fourier_coeffs[i+15]*((i+1)*omega))*np.cos((i+1) * omega * Time_vec)
        qddot[:,2] += -(fourier_coeffs[i+20]*((i+1)*omega))*np.sin((i+1) * omega * Time_vec)+(fourier_coeffs[i+25]*((i+1)*omega))*np.cos((i+1) * omega * Time_vec)
    
    return Time_vec, q, qdot, qddot   

#목적함수/조건수 계산
def compute_condition_number(
        
)

#제약조건

#메인함수(위에것들 묶어서 최적화된 궤적 생성)


if __name__ == "__main__":
    coeffs = np.ones(30)
    t, q, qdot, qddot = trajectory_evaluator(coeffs)
    
    # q를 수치 미분한 값과 수식으로 구한 qdot 비교
    qdot_numerical = np.gradient(q[:, 0], t)
    diff = np.max(np.abs(qdot[:, 0] - qdot_numerical))
    print(f"속도 수식 정합성 최대 오차: {diff:.2e}")  # 1e-3 이하로 일치해야 정상