"""
Indy7 2D 3-DOF 페이로드 동역학 파라미터 OLS 식별 모듈 (identify_payload.py)
- 관절 각도/각속도/각가속도 시계열 및 관절 실측 토크로부터 선형 회귀행렬(Y) 누적
- 2단계 최소자승법(OLS) 연산을 통해 페이로드 4대 파라미터 [m, c_x, c_y, I_o] 복원
- 잔차 제곱합, 결정계수(R^2), RMSE, 신뢰구간 및 추정 분산 통계량 산출
"""

from typing import Tuple, Dict, Any, Optional
import numpy as np
import sys
from pathlib import Path
import importlib

# 상위 폴더 src 및 config 참조
current_dir = Path(__file__).resolve().parent
project_root = current_dir.parent
for p in [str(current_dir), str(project_root)]:
    if p not in sys.path:
        sys.path.append(p)

from Indy7_2D3DOF import INDY7_2D_JOINTS, compute_torque, compute_regressor, DEFAULT_FRICTION_PARAMS


def build_regressor_and_torque(
    q: np.ndarray,
    qdot: np.ndarray,
    qddot: np.ndarray,
    tau_meas: np.ndarray,
    friction_params: Optional[np.ndarray] = None
) -> Tuple[np.ndarray, np.ndarray]:
    """
    시계열 궤적 데이터로부터 전체 누적 회귀행렬 Y_all(3N x 4) 및 순수 페이로드 유효 토크 tau_net(3N,) 산출
    """
    N = len(q)
    if friction_params is None:
        friction_params = DEFAULT_FRICTION_PARAMS

    Y_all = np.zeros((3 * N, 4))
    tau_net = np.zeros(3 * N)

    for i in range(N):
        # 1. 로봇 공칭 토크(마찰 포함) 차감 -> 순수 페이로드 기여 토크 분리
        tau_nom = compute_torque(q[i], qdot[i], qddot[i], payload_params=None, friction_params=friction_params)
        tau_diff = tau_meas[i] - tau_nom

        # 2. 페이로드 선형 회귀행렬 (3x4)
        Y_i = compute_regressor(q[i], qdot[i], qddot[i])

        idx_start = 3 * i
        idx_end = 3 * i + 3
        Y_all[idx_start:idx_end, :] = Y_i
        tau_net[idx_start:idx_end] = tau_diff

    return Y_all, tau_net


def estimate_payload_ols(
    q: np.ndarray,
    qdot: np.ndarray,
    qddot: np.ndarray,
    tau_meas: np.ndarray,
    friction_params: Optional[np.ndarray] = None
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """
    최소자승법(OLS)을 활용한 페이로드 4대 파라미터 추정 및 통계 검증

    입력:
      q, qdot, qddot  : (N, 3) 각도, 각속도, 각가속도 [rad, rad/s, rad/s^2]
      tau_meas        : (N, 3) 각 관절 실측 토크 [N*m]
      friction_params : (6,) 마찰 파라미터 (선택)

    출력:
      payload_est     : (4,) [m, c_x, c_y, I_o] 추정치
      stats           : 추정 품질 통계치 (R^2, RMSE, condition_number, std_errors 등)
    """
    Y_all, tau_net = build_regressor_and_torque(q, qdot, qddot, tau_meas, friction_params)

    # 1. OLS 연산: theta_hat = (Y^T * Y)^(-1) * Y^T * tau_net
    theta_hat, residuals, rank, s = np.linalg.lstsq(Y_all, tau_net, rcond=None)

    # 2. 페이로드 물리 파라미터 복원
    # theta = [m, m*cx, m*cy, Io]
    m_est = float(theta_hat[0])
    if abs(m_est) > 1e-6:
        cx_est = float(theta_hat[1] / m_est)
        cy_est = float(theta_hat[2] / m_est)
    else:
        cx_est = 0.0
        cy_est = 0.0

    Io_est = float(theta_hat[3])
    Icom_est = Io_est - m_est * (cx_est**2 + cy_est**2)

    payload_est = np.array([m_est, cx_est, cy_est, Io_est])

    # 3. 통계적 유의성 및 적합도 평가
    tau_pred = Y_all @ theta_hat
    residuals_vec = tau_net - tau_pred
    dof = len(tau_net) - 4
    residual_variance = float(np.sum(residuals_vec**2) / max(dof, 1))
    rmse = float(np.sqrt(np.mean(residuals_vec**2)))

    ss_tot = float(np.sum((tau_net - np.mean(tau_net))**2))
    ss_res = float(np.sum(residuals_vec**2))
    r_squared = float(1.0 - (ss_res / ss_tot)) if ss_tot > 1e-12 else 1.0

    # 파라미터 공분산 및 표준오차
    cov_theta = residual_variance * np.linalg.pinv(Y_all.T @ Y_all)
    std_errors = np.sqrt(np.maximum(np.diag(cov_theta), 0.0))
    cond_y = float(s[0] / s[-1]) if s[-1] > 1e-12 else 1e6

    stats = {
        "theta_hat": theta_hat,
        "m_est": m_est,
        "cx_est": cx_est,
        "cy_est": cy_est,
        "Io_est": Io_est,
        "Icom_est": Icom_est,
        "rmse": rmse,
        "r_squared": r_squared,
        "condition_number": cond_y,
        "std_errors": std_errors,
        "rank": int(rank),
        "singular_values": s
    }

    return payload_est, stats


if __name__ == "__main__":
    import sys
    sys.stdout.reconfigure(encoding='utf-8')

    print("=== Indy7 2D 3-DOF 페이로드 OLS 식별 시뮬레이션 ===")

    # 1. 5차 푸리에 여기 궤적 생성
    traj_mod = importlib.import_module("2D_trajectory")
    DEFAULT_OPTIMAL_COEFFS = np.array([
        0.042, -0.038,  0.025, -0.012,  0.035, -0.028,  0.018,
        0.035,  0.045, -0.022,  0.015, -0.028,  0.032, -0.014,
       -0.028,  0.032,  0.038, -0.016,  0.022, -0.025,  0.012
    ])
    t, q, qdot, qddot = traj_mod.trajectory_evaluator(DEFAULT_OPTIMAL_COEFFS, time_step=0.01, period=10.0)
    N = len(t)

    # 2. 참값 페이로드 정의 [m=2.5kg, cx=0.04m, cy=-0.02m, Io=0.015kg*m^2]
    payload_true = np.array([2.50, 0.04, -0.02, 0.015])

    # 3. 측정 토크 합성 (백색 가우시안 노이즈 std=0.20 N*m 추가)
    np.random.seed(42)
    tau_meas = np.zeros((N, 3))
    for i in range(N):
        tau_clean = compute_torque(q[i], qdot[i], qddot[i], payload_params=payload_true, friction_params=DEFAULT_FRICTION_PARAMS)
        tau_meas[i] = tau_clean + np.random.normal(0, 0.20, 3)

    # 4. OLS 식별 수행
    payload_est, stats = estimate_payload_ols(q, qdot, qddot, tau_meas, friction_params=DEFAULT_FRICTION_PARAMS)

    # 5. 결과 출력
    labels = ["질량 m [kg]", "무게중심 cx [m]", "무게중심 cy [m]", "관성모멘트 Io [kg*m^2]"]
    print(f"\n[식별 결과 요약] (샘플 수: {N}, 회귀행렬 크기: {3*N}x4)")
    print(f"회귀행렬 조건수 cond(Y): {stats['condition_number']:.2f}")
    print(f"결정계수 (R^2): {stats['r_squared']:.4f}")
    print(f"토크 잔차 RMSE: {stats['rmse']:.4f} N*m\n")

    for idx, name in enumerate(labels):
        true_val = payload_true[idx]
        est_val = payload_est[idx]
        err_pct = abs(est_val - true_val) / abs(true_val) * 100.0 if abs(true_val) > 1e-6 else 0.0
        print(f"  {name:25s}: 참값 = {true_val:8.4f} | 추정값 = {est_val:8.4f} | 오차 = {err_pct:5.2f}%")

    # 6. 고해상도 검증 플롯 생성 (docs/identification_results.png)
    import matplotlib.pyplot as plt

    tau_nom = np.zeros((N, 3))
    for i in range(N):
        tau_nom[i] = compute_torque(q[i], qdot[i], qddot[i], payload_params=None, friction_params=DEFAULT_FRICTION_PARAMS)

    Y_all = np.zeros((3 * N, 4))
    for i in range(N):
        Y_all[3 * i : 3 * i + 3, :] = compute_regressor(q[i], qdot[i], qddot[i])
    tau_pred_net = (Y_all @ stats["theta_hat"]).reshape((N, 3))

    fig, axes = plt.subplots(3, 1, figsize=(9, 11), dpi=300)

    # (1) 궤적 플롯
    axes[0].plot(t, q[:, 0], label=r"$q_1$ (Shoulder / J2)", color="#0055A5", lw=1.8)
    axes[0].plot(t, q[:, 1], label=r"$q_2$ (Elbow / J3)", color="#D9381E", lw=1.8)
    axes[0].plot(t, q[:, 2], label=r"$q_3$ (Wrist / J4)", color="#2CA02C", lw=1.8)
    axes[0].set_title("Optimal 5th-Order Fourier Excitation Trajectory", fontsize=12, fontweight="bold", pad=8)
    axes[0].set_xlabel("Time [s]", fontsize=10)
    axes[0].set_ylabel("Joint Angle [rad]", fontsize=10)
    axes[0].legend(loc="upper right", frameon=True)
    axes[0].grid(True, alpha=0.3)

    # (2) 토크 플롯 (Joint 2 Shoulder)
    axes[1].plot(t, tau_meas[:, 0], label=r"Measured $\tau_1$ (Noisy)", color="#A0AEC0", lw=1.2, alpha=0.7)
    axes[1].plot(t, tau_nom[:, 0] + tau_pred_net[:, 0], label=r"OLS Reconstructed $\hat{\tau}_1$", color="#D9381E", lw=1.8, linestyle="--")
    axes[1].plot(t, tau_nom[:, 0], label=r"Nominal Robot $\tau_{\mathrm{robot}, 1}$", color="#0055A5", lw=1.5, alpha=0.8)
    axes[1].set_title(r"Torque Tracking & Reconstruction (Joint 2 / Shoulder)", fontsize=12, fontweight="bold", pad=8)
    axes[1].set_xlabel("Time [s]", fontsize=10)
    axes[1].set_ylabel(r"Torque [$\mathrm{N\cdot m}$]", fontsize=10)
    axes[1].legend(loc="upper right", frameon=True)
    axes[1].grid(True, alpha=0.3)

    # (3) 파라미터 막대그래프
    param_names = ["Mass m\n[kg]", r"$c_x$" + "\n[cm]", r"$c_y$" + "\n[cm]", r"$I_o$" + "\n[$10^{-2}\mathrm{kg\cdot m^2}$]"]
    true_scaled = [payload_true[0], payload_true[1]*100, payload_true[2]*100, payload_true[3]*100]
    est_scaled = [payload_est[0], payload_est[1]*100, payload_est[2]*100, payload_est[3]*100]

    x = np.arange(len(param_names))
    width = 0.32
    axes[2].bar(x - width/2, true_scaled, width, label="Ground Truth", color="#182B49", alpha=0.9)
    axes[2].bar(x + width/2, est_scaled, width, label="OLS Estimated", color="#DC5014", alpha=0.9)

    axes[2].set_title("Payload Parameter Estimation Accuracy", fontsize=12, fontweight="bold", pad=8)
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(param_names, fontsize=10)
    axes[2].set_ylabel("Physical Values (Scaled)", fontsize=10)
    axes[2].legend(loc="upper right", frameon=True)
    axes[2].grid(True, axis="y", alpha=0.3)

    for i in range(len(param_names)):
        e_v = est_scaled[i]
        err_pct = abs(payload_est[i] - payload_true[i]) / abs(payload_true[i]) * 100.0 if abs(payload_true[i]) > 1e-6 else 0.0
        offset_y = 3 if e_v >= 0 else -16
        axes[2].annotate(f"{e_v:.3f}\n(err {err_pct:.1f}%)",
                         xy=(x[i] + width/2, e_v),
                         xytext=(0, offset_y),
                         textcoords="offset points",
                         ha="center", va="bottom" if e_v >= 0 else "top",
                         fontsize=8, fontweight="bold", color="#8B0000")

    plt.tight_layout()
    plot_path = project_root / "docs" / "identification_results.png"
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(plot_path, dpi=300)
    plt.close()
    print(f"\n[플롯 저장 완료] {plot_path}")
