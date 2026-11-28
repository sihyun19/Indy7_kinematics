'''
SLSQP솔버
'''
import os
import numpy as np
from Indy7_2D3DOF import INDY7_2D_JOINTS, R_BASE_2D, fk_2D, compute_torque, compute_regressor
from scipy.optimize import minimize
import csv, re, datetime
from pathlib import Path

# 실측 외곽 390mm(폭) × 296mm(높이) 기준 3-원 근사 (대칭 보수 반경 r=0.075m)
PAYLOAD_CIRCLES = [
    (0.245,  0.000, 0.075),  # 1) 상단 추 (축방향 최대 0.320m 커버 ≥ 0.296m)
    (0.165,  0.155, 0.075),  # 2) 우측 추 (횡방향 최대 +0.230m 커버)
    (0.165, -0.155, 0.075),  # 3) 좌측 끝단 (장착 방향 무관 보수 반경)
]

# 실측치 [4.4902, 0.1691, -0.0555, 0.2031]에 안전 여유 5%를 둔 상한
MAX_PAYLOAD1 = np.array([4.6, 0.175, -0.060, 0.215])
MAX_PAYLOAD2 = np.array([4.6, 0.175, 0.060, 0.215])

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

# 제약조건, 다운샘플링 후 *200 벡터  출력
def evaluate_constraints(
    fourier_coeffs_21: np.ndarray,
    time_step: float = 0.05,
    period: float = 10.0,
) -> np.ndarray:
  
    _, q, qdot, qddot = trajectory_evaluator(fourier_coeffs_21, time_step, period)
    N = len(q)

    q_max = np.array([0.8*j.q_max for j in INDY7_2D_JOINTS])
    q_min = np.array([0.8*j.q_min for j in INDY7_2D_JOINTS])
    qdot_max = np.array([0.4*j.qdot_max for j in INDY7_2D_JOINTS])
    torque_max = np.array([0.8*j.effort for j in INDY7_2D_JOINTS])

    margin_q_upper = ((q_max - q) / q_max).flatten()
    margin_q_lower = ((q - q_min) / np.abs(q_min)).flatten()
    margin_qdot    = ((qdot_max - np.abs(qdot)) / qdot_max).flatten()
    torque_all =   np.zeros((N, 3))
    for i in range(N):
        tau_free = compute_torque(q[i], qdot[i], qddot[i])
        tau_max_1  = compute_torque(q[i], qdot[i], qddot[i], MAX_PAYLOAD1)
        tau_max_2  = compute_torque(q[i], qdot[i], qddot[i], MAX_PAYLOAD2)
        tau_max = np.maximum(np.abs(tau_max_1), np.abs(tau_max_2))
        torque_all[i, :] = np.maximum(np.abs(tau_free), np.abs(tau_max))
    margin_torque  = ((torque_max - np.abs(torque_all)) / torque_max).flatten()

    #절대좌표 마진
    z_min  = 0.1
    margin_z = []
    for i in range(N):
        p1, p2, p3 = fk_2D(q[i])
        phi = np.sum(q[i])
        z_payload = [p3[1] + t_off * np.cos(phi) - n_off * np.sin(phi) - r for t_off, n_off, r in PAYLOAD_CIRCLES]
        margin_z.extend([p1[1], p2[1], p3[1], *z_payload])
    margin_z = (np.array(margin_z) - z_min)

    #충돌 마진
    margin_col = []
    for i in range(N):
        margin_col.extend(collision(q[i]))
    margin_col = np.array(margin_col)

        
    return np.concatenate([margin_q_upper, margin_q_lower, margin_qdot, margin_torque, margin_z, margin_col ])

def collision(
    q : np.ndarray
) -> np.ndarray :
    base = np.array([0.0, 0.0])
    p0= np.array([0.0, INDY7_2D_JOINTS[0].len_val])
    p1, p2, p3 = fk_2D(q)
    phi = np.sum(q)
    CLEARANCE = 0.02
    R1, R2, R3 = (j.R for j in INDY7_2D_JOINTS)
    #링크간 충돌
    d0_sq = link_link_distance(base, p0, p1, p2)
    d1_sq = link_link_distance(base, p0, p2, p3)
    d2_sq = link_link_distance(p0, p1, p2, p3)
    margins = [
        (d0_sq - (R_BASE_2D + R2 + CLEARANCE)**2) / (R_BASE_2D + R2 + CLEARANCE)**2,
        (d1_sq - (R_BASE_2D + R3 + CLEARANCE)**2) / (R_BASE_2D + R3 + CLEARANCE)**2,
        (d2_sq - (R1 + R3 + CLEARANCE)**2)        / (R1 + R3 + CLEARANCE)**2,
    ]
    #링크-페이로드 충돌
    links = [(base, p0, R_BASE_2D), (p0, p1, R1), (p1, p2, R2)]
    for t_off, n_off, r in PAYLOAD_CIRCLES:
        p_pay = np.array([
            p3[0] - t_off * np.sin(phi) - n_off * np.cos(phi),
            p3[1] + t_off * np.cos(phi) - n_off * np.sin(phi),
        ])
        for link_start, link_end, r_link in links:
            d_sq = point_link_distance(p_pay, link_start, link_end)
            d_req_sq = (r + r_link + CLEARANCE) ** 2
            margins.append((d_sq - d_req_sq) / d_req_sq)

    
    return np.array(margins)

def point_link_distance(p, link_start, link_end):
    link = link_end - link_start
    t = np.clip(np.dot(p - link_start, link) / np.dot(link, link), 0.0, 1.0)
    diff = p - (link_start + t * link)
    return float(np.dot(diff, diff))

def link_link_distance(l1_start, l1_end, l2_start, l2_end):
    def side(p, start, end):
        link = end -start
        p_start_vec = p - start
        return link[0] * p_start_vec[1] - link[1] * p_start_vec[0]
    l2_cross_l1 = side(l2_start, l1_start, l1_end ) * side(l2_end, l1_start, l1_end) < 0
    l1_cross_l2 = side(l1_start, l2_start, l2_end) * side(l1_end, l2_start, l2_end) < 0
    d_end_square = min(point_link_distance(l1_start, l2_start, l2_end),
               point_link_distance(l1_end,   l2_start, l2_end),
               point_link_distance(l2_start, l1_start, l1_end),
               point_link_distance(l2_end,   l1_start, l1_end))
    if l2_cross_l1 and l1_cross_l2:
        return -d_end_square
    return d_end_square
    

# SLSQP솔버 조립
def slsqp(
    coeffs_21: np.ndarray | None = None,
    maxiter: int = 100,
    eps: float = 1e-4,
    ftol: float = 1e-3
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
        print(f"  [반복 {count:2d}] 현재 조건수: {current_cond:.4f}, " f"최소 마진: {evaluate_constraints(xk).min():+.4f}")
        np.save("temp_coeffs.npy", xk)

    result = minimize(
        fun = compute_condition_number,
        x0 = coeffs_21,
        method = 'SLSQP',
        constraints = { 'type' : 'ineq', 'fun' : evaluate_constraints },
        callback = print_count,
        bounds = [(-2.0, 2.0)] * 21,
        options= {
            'maxiter': maxiter,
            'disp': True,
            'eps': eps,
            'ftol': ftol
        }
    )
    return result


TRAJ_DIR = Path(__file__).resolve().parent.parent / "data" / "trajectories"

def next_id(out_dir):
    ids = [int(m.group(1)) for f in Path(out_dir).glob("optimized_coeffs_*.npy")
           if (m := re.search(r"_(\d+)\.npy$", f.name))]
    return max(ids, default=0) + 1

def run_random(scale=0.1, maxiter=100, max_try=100):
    TRAJ_DIR.mkdir(parents=True, exist_ok=True)
    for _ in range(max_try):
        coeffs = np.random.uniform(-scale, scale, 21)
        if np.min(evaluate_constraints(coeffs)) >= 0:
            break
    else:
        raise RuntimeError(f"만족하는 seed를 찾지 못했습니다.")
    print(f"coeffs={coeffs}")
    return coeffs

def save(optimized_coeffs, out_dir: str | Path  =".", tag =""):
    out_dir = Path(out_dir)
    np.save(out_dir / f"optimized_coeffs{tag}.npy", optimized_coeffs)
    np.savetxt(out_dir / f"optimized_coeffs{tag}.txt", optimized_coeffs, fmt="%.8f")
    print(f"최적화 계수가 'optimized_coeffs{tag}.npy' 및 '.txt'에 저장되었습니다.")
    Time_vec, q, qdot, qddot  = trajectory_evaluator(optimized_coeffs, time_step=0.00025)
    N = len(q)
    
    torque =   np.zeros((N, 3))
    for i in range(N):
        torque[i,:] = compute_torque(q[i], qdot[i], qddot[i])
    np.savez(out_dir / f"optimized_trajectory{tag}.npz", time=Time_vec, q=q, qdot=qdot, qddot=qddot, torque=torque)
    trajectory_data = np.column_stack([Time_vec[:, None], q, qdot, qddot,torque])
    header_str = "time, q1, q2, q3, qdot1, qdot2, qdot3, qddot1, qddot2, qddot3, torque1, torque2, torque3"
    np.savetxt(out_dir / f"optimized_trajectory{tag}.txt", trajectory_data, fmt="%.8f", delimiter=", ", header=header_str)
    print(f"최적화 경로가 'optimized_trajectory{tag}.npz' 및 '.txt'에 저장되었습니다.")
    print(f"q_min        : {np.min(q, axis=0)}")
    print(f"q_max        : {np.max(q, axis=0)}")
    print(f"|qdot|_max   : {np.max(np.abs(qdot), axis=0)}")
    print(f"|torque|_max : {np.max(np.abs(torque), axis=0)}")

    z = np.array([[p[1] for p in fk_2D(q_t)] for q_t in q])
    print(f"높이 최소(p1,p2,p3) : {np.min(z, axis=0)}")
    
    is_valid = np.all(evaluate_constraints(optimized_coeffs, 0.01) >= 5 * -1e-3)
    print(f"제약조건 만족 여부(여유 20%): {is_valid}")
    if not is_valid:
        report_violations(optimized_coeffs, 0.01)

def report_violations(coeffs, time_step=0.01, period=10.0):
    c = evaluate_constraints(coeffs, time_step, period)
    N = int(period / time_step)
    specs = [
        ("q_upper", ["joint2", "joint3", "joint5"], "%", 100),
        ("q_lower", ["joint2", "joint3", "joint5"], "%", 100),
        ("qdot",    ["joint2", "joint3", "joint5"], "%", 100),
        ("torque",  ["joint2", "joint3", "joint5"], "%", 100),
        ("z",       ["p1", "p2", "p3", "pay_top", "pay_R", "pay_L"], "m", 1),
        ("col",     ["base-L2", "base-L3", "L1-L3", "top-base", "top-L1", "top-L2", "R-base", "R-L1", "R-L2", "L-base", "L-L1", "L-L2"], "%", 100),
    ]
    print("---- 제약 위반 리포트 ----")
    any_violation = False
    offset = 0
    for name, labels, unit, scale in specs:
        k = len(labels)
        block = c[offset : offset + N * k].reshape(N, k)
        offset += N * k
        for j, label in enumerate(labels):
            m = block[:, j]
            if m.min() >= 0:
                continue
            any_violation = True
            i = m.argmin()
            print(f"{name:8s} {label:8s}: 최대 위반 {m.min() * scale:+.2f} {unit}, " f"t = {i * time_step:.2f} s, 위반 샘플 {np.sum(m < 0)}/{N}")
    if not any_violation:
        print("위반 없음")

if __name__ == "__main__":
    print(f"--------test--------")
    ##
    # coeffs_test = np.array([
    #    -0.022, -0.001,  0.048,  0.046,  0.022,  0.004, -0.022,
    #    -0.034,  0.047,  0.002, -0.038,  0.012,  0.028,  0.011,
    #    0.042, -0.046,  0.003, -0.004, -0.044,  0.014,  0.035,
    #])
    ##

    coeffs_test = run_random()
    t_test, q_test, qdot_test, qddot_test = trajectory_evaluator(coeffs_test)
    
    
    # q를 수치 미분한 값과 수식으로 구한 qdot 비교
    qdot_test_numerical = np.gradient(q_test[:, 0], t_test)
    diff = np.max(np.abs(qdot_test[:, 0] - qdot_test_numerical))
    print(f"속도 수식 최대 오차: {diff:.2e}")

    cond = compute_condition_number(coeffs_test)
    print(f"조건수: {cond:.2f}")

    # 제약조건 벡터 검증
    c_vals_test = evaluate_constraints(coeffs_test)
    print(f"제약조건 벡터 형태: {c_vals_test.shape}") # (1500,)
    print(f"최소 마진(음수면 위반): {np.min(c_vals_test):.4f}")

    print(f"--------실행--------")
    res= slsqp(coeffs_test)
    if res.success:
        TRAJ_DIR.mkdir(parents=True, exist_ok=True)
        save(res.x, TRAJ_DIR, f"_{next_id(TRAJ_DIR):03d}")
        
    else:
        print(f"최적화 실패: {res.message}")