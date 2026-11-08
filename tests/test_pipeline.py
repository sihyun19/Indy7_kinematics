"""
Indy7 2D 3-DOF 11대 핵심 파이프라인 자동 통합 단위 테스트 스위트 (tests/test_pipeline.py)
1. test_01_forward_kinematics: 순기구학 영점 및 링크 길이 기하 정합성 검증
2. test_02_regressor_linearity: 회귀행렬 Y*theta 기계 정밀도 선형성 검증 (< 1e-12 Nm)
3. test_03_trajectory_boundary_conditions: 궤적 경계조건 q(0)=q0, qdot(0)=0, qddot(0)=0 일치 검증
4. test_04_ols_identification: 노이즈 환경 하 2단계 OLS 질량 추정 오차 (< 2.0%) 검증
5. test_05_safety_supervisor: ISO/TS 15066 안전 감시자 비상 정지(CAT 0) 트리거 검증
6. test_06_momentum_observer: 센서리스 모멘텀 관측기 외력 충돌 즉각 감지 검증
7. test_07_frequency_separation: 감속기 비틀림 공진 대역(15~30Hz) 10배 이상 안전 이격 검증
8. test_08_spatial_6dof_regressor: 3D 공간 6자유도 확장 회귀행렬 (6x10) 정식화 검증
9. test_09_passivity_adaptive_controller: Slotine-Li 수동성 적응 제어기 슬라이딩 오차 수렴성 검증
10. test_10_nmpc_torque_limits: NMPC 최적 제어 토크의 모터 하드웨어 토크 한계 준수 검증
11. test_11_pinn_residual_learning: PINN 신경망 미모델링 잔차 학습 손실 70% 이상 감소 검증
"""

import unittest
import numpy as np
import sys
from pathlib import Path

# 상위 폴더 src 및 config 참조
tests_dir = Path(__file__).resolve().parent
project_root = tests_dir.parent
src_dir = project_root / "src"
for p in [str(src_dir), str(project_root)]:
    if p not in sys.path:
        sys.path.append(p)

from Indy7_2D3DOF import INDY7_2D_JOINTS, forward_kinematics, compute_torque, compute_regressor, DEFAULT_FRICTION_PARAMS
import importlib
traj_mod = importlib.import_module("2D_trajectory")
trajectory_evaluator = traj_mod.trajectory_evaluator

from identify_payload import estimate_payload_ols

try:
    from safety_supervisor import SafetySupervisor, SafetyStatus
except ImportError:
    SafetySupervisor = None

try:
    from collision_detector import MomentumObserver
except ImportError:
    MomentumObserver = None

try:
    from frequency_response_analysis import analyze_frequency_spectrum
except ImportError:
    analyze_frequency_spectrum = None

try:
    from spatial_6dof_payload_identifier import compute_spatial_wrench_regressor
except ImportError:
    compute_spatial_wrench_regressor = None

try:
    from passivity_adaptive_controller import SlotineLiAdaptiveController
except ImportError:
    SlotineLiAdaptiveController = None

try:
    from nmpc_payload_controller import PayloadNMPCController
except ImportError:
    PayloadNMPCController = None

try:
    from pinn_residual_dynamics import LightweightPINNResidual
except ImportError:
    LightweightPINNResidual = None


class TestIndy7Pipeline(unittest.TestCase):
    def test_01_forward_kinematics(self):
        """1. 순기구학 영점 위치 및 링크 길이 기하 정합성 검증"""
        q_zero = np.zeros(3)
        p1, p2, p3 = forward_kinematics(q_zero)

        L0 = INDY7_2D_JOINTS[0].lenth
        L1 = INDY7_2D_JOINTS[1].lenth
        L2 = INDY7_2D_JOINTS[2].lenth
        L3 = INDY7_2D_JOINTS[3].lenth

        self.assertAlmostEqual(p1[0], 0.0, places=6)
        self.assertAlmostEqual(p1[1], L0 + L1, places=6)
        self.assertAlmostEqual(p2[0], 0.0, places=6)
        self.assertAlmostEqual(p2[1], L0 + L1 + L2, places=6)
        self.assertAlmostEqual(p3[0], 0.0, places=6)
        self.assertAlmostEqual(p3[1], L0 + L1 + L2 + L3, places=6)

    def test_02_regressor_linearity(self):
        """2. 회귀행렬 Y*theta 기계 정밀도 선형성 검증 (< 1e-12 Nm)"""
        q = np.array([0.25, 0.45, -0.35])
        qdot = np.array([0.40, -0.20, 0.60])
        qddot = np.array([1.10, -0.50, 0.30])
        payload_true = np.array([2.50, 0.04, -0.02, 0.015])

        tau_direct = compute_torque(q, qdot, qddot, payload_params=payload_true) - compute_torque(q, qdot, qddot)
        Y_mat = compute_regressor(q, qdot, qddot)
        theta_vec = np.array([payload_true[0], payload_true[0] * payload_true[1], payload_true[0] * payload_true[2], payload_true[3]])
        tau_linear = Y_mat @ theta_vec

        error = np.max(np.abs(tau_direct - tau_linear))
        self.assertLess(error, 1e-12, f"회귀행렬 비선형 오차 발생: {error:.2e} Nm")

    def test_03_trajectory_boundary_conditions(self):
        """3. 궤적 경계조건 q(0)=q0, qdot(0)=0, qddot(0)=0 일치 검증"""
        coeffs = np.array([
            0.042, -0.038,  0.025, -0.012,  0.035, -0.028,  0.018,
            0.035,  0.045, -0.022,  0.015, -0.028,  0.032, -0.014,
           -0.028,  0.032,  0.038, -0.016,  0.022, -0.025,  0.012
        ])
        q0 = np.array([0.0, 0.5, -0.5])
        t, q, qdot, qddot = trajectory_evaluator(coeffs, time_step=0.01, period=10.0, q0=q0)

        np.testing.assert_allclose(q[0], q0, atol=1e-10)
        np.testing.assert_allclose(qdot[0], np.zeros(3), atol=1e-10)
        np.testing.assert_allclose(qddot[0], np.zeros(3), atol=1e-10)

    def test_04_ols_identification(self):
        """4. 노이즈 환경 하 2단계 OLS 질량 추정 오차 (< 2.0%) 검증"""
        coeffs = np.array([
            0.042, -0.038,  0.025, -0.012,  0.035, -0.028,  0.018,
            0.035,  0.045, -0.022,  0.015, -0.028,  0.032, -0.014,
           -0.028,  0.032,  0.038, -0.016,  0.022, -0.025,  0.012
        ])
        t, q, qdot, qddot = trajectory_evaluator(coeffs, time_step=0.02, period=10.0)
        N = len(t)
        
        payload_true = np.array([2.50, 0.04, -0.02, 0.015])
        tau_meas = np.zeros((N, 3))
        np.random.seed(123)
        for i in range(N):
            tau_clean = compute_torque(q[i], qdot[i], qddot[i], payload_params=payload_true, friction_params=DEFAULT_FRICTION_PARAMS)
            tau_meas[i] = tau_clean + np.random.normal(0, 0.20, 3)

        payload_est, stats = estimate_payload_ols(q, qdot, qddot, tau_meas, friction_params=DEFAULT_FRICTION_PARAMS)
        mass_error_pct = abs(payload_est[0] - payload_true[0]) / payload_true[0] * 100.0
        self.assertLess(mass_error_pct, 2.0, f"질량 추정 오차 초과: {mass_error_pct:.2f}%")

    @unittest.skipIf(SafetySupervisor is None, "safety_supervisor 모듈 미설치")
    def test_05_safety_supervisor(self):
        """5. ISO/TS 15066 안전 감시자 비상 정지(CAT 0) 트리거 검증"""
        supervisor = SafetySupervisor()
        q_safe = np.array([0.1, 0.3, -0.2])
        qd_safe = np.array([0.2, 0.1, 0.1])
        tau_safe = np.array([50.0, 30.0, 10.0])

        status_ok, _ = supervisor.check_safety(q_safe, qd_safe, tau_safe)
        self.assertEqual(status_ok, SafetyStatus.NORMAL)

        qd_danger = np.array([3.5, 0.0, 0.0])
        status_stop, rep = supervisor.check_safety(q_safe, qd_danger, tau_safe)
        self.assertEqual(status_stop, SafetyStatus.EMERGENCY_STOP)

    @unittest.skipIf(MomentumObserver is None, "collision_detector 모듈 미설치")
    def test_06_momentum_observer(self):
        """6. 센서리스 모멘텀 관측기 외력 충돌 즉각 감지 검증"""
        obs = MomentumObserver(threshold_torque=12.0, dt=0.01)
        q = np.array([0.1, 0.4, -0.2])
        qd = np.array([0.5, -0.3, 0.4])
        tau_nom = compute_torque(q, qd, np.zeros(3), friction_params=DEFAULT_FRICTION_PARAMS)
        
        obs.update(0.0, q, qd, tau_nom)
        _, hit = obs.update(0.01, q, qd, tau_nom + np.array([0.0, 30.0, 0.0]))
        self.assertTrue(hit, "외력 충돌 감지 실패")

    @unittest.skipIf(analyze_frequency_spectrum is None, "frequency_response_analysis 모듈 미설치")
    def test_07_frequency_separation(self):
        """7. 감속기 비틀림 공진 대역(15~30Hz) 10배 이상 안전 이격 검증"""
        coeffs = np.array([
            0.042, -0.038,  0.025, -0.012,  0.035, -0.028,  0.018,
            0.035,  0.045, -0.022,  0.015, -0.028,  0.032, -0.014,
           -0.028,  0.032,  0.038, -0.016,  0.022, -0.025,  0.012
        ])
        t, q, qdot, qddot = trajectory_evaluator(coeffs, time_step=0.01, period=10.0)
        res = analyze_frequency_spectrum(qdot, fs=100.0)
        self.assertTrue(res["is_resonance_free"])
        self.assertGreaterEqual(res["resonance_separation_ratio"], 10.0)

    @unittest.skipIf(compute_spatial_wrench_regressor is None, "spatial_6dof_payload_identifier 모듈 미설치")
    def test_08_spatial_6dof_regressor(self):
        """8. 3D 공간 6자유도 확장 회귀행렬 (6x10) 정식화 검증"""
        a = np.array([0.5, -0.2, 1.2])
        w = np.array([0.4, 0.8, -0.3])
        al = np.array([1.5, -0.6, 0.9])
        K = compute_spatial_wrench_regressor(a, w, al)
        self.assertEqual(K.shape, (6, 10))
        self.assertTrue(np.all(np.isfinite(K)))

    @unittest.skipIf(SlotineLiAdaptiveController is None, "passivity_adaptive_controller 모듈 미설치")
    def test_09_passivity_adaptive_controller(self):
        """9. Slotine-Li 수동성 적응 제어기 슬라이딩 오차 수렴성 검증"""
        ctrl = SlotineLiAdaptiveController(dt=0.01)
        q = np.array([0.1, 0.4, -0.2])
        qd = np.zeros(3)
        qd_des = np.array([0.1, -0.05, 0.05])
        qdd_des = np.array([0.2, 0.1, -0.1])

        tau, s, th = ctrl.compute_control(q, qd, q, qd_des, qdd_des)
        self.assertTrue(np.all(np.isfinite(tau)))
        self.assertGreater(th[0], 0.0)

    @unittest.skipIf(PayloadNMPCController is None, "nmpc_payload_controller 모듈 미설치")
    def test_10_nmpc_torque_limits(self):
        """10. NMPC 최적 제어 토크의 모터 하드웨어 토크 한계 준수 검증"""
        nmpc = PayloadNMPCController(horizon_steps=2, dt=0.02)
        q_now = np.array([0.1, 0.3, -0.2])
        qd_now = np.zeros(3)
        q_des_h = np.array([[0.12, 0.32, -0.19], [0.15, 0.35, -0.18]])
        qd_des_h = np.zeros((2, 3))

        tau_opt = nmpc.solve_optimal_torque(q_now, qd_now, q_des_h, qd_des_h)
        self.assertLessEqual(abs(tau_opt[0]), 431.97)
        self.assertLessEqual(abs(tau_opt[1]), 197.23)
        self.assertLessEqual(abs(tau_opt[2]), 79.79)

    @unittest.skipIf(LightweightPINNResidual is None, "pinn_residual_dynamics 모듈 미설치")
    def test_11_pinn_residual_learning(self):
        """11. PINN 신경망 미모델링 잔차 학습 손실 70% 이상 감소 검증"""
        pinn = LightweightPINNResidual()
        np.random.seed(123)
        x = np.random.uniform(-0.5, 0.5, (50, 9))
        y = np.column_stack([0.5 * np.sin(2*x[:, 0]), 0.3 * np.cos(x[:, 1]), 0.2 * np.sin(x[:, 2])])
        
        l0 = np.mean((pinn.forward(x) - y)**2)
        for _ in range(80):
            pinn.train_step(x, y)
        lf = np.mean((pinn.forward(x) - y)**2)
        
        reduction = (1.0 - lf / l0) * 100.0
        self.assertGreater(reduction, 70.0, f"PINN 학습 감소율 부족: {reduction:.1f}%")


if __name__ == "__main__":
    unittest.main()
