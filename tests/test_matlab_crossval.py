"""
Unit Tests: Cross-Validation with Senior's MATLAB Simscape Baseline
선배의 indy7_sim_fk.m 및 Simscape 모델의 기준값과의 수치 정합성 검증 테스트
"""

import unittest
import numpy as np
import sys
import os

# Add parent directory to sys.path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config.indy7_dimensions import Indy7Geometry
from src.kinematics import Indy7Kinematics


class TestIndy7MatlabCrossValidation(unittest.TestCase):
    def setUp(self):
        self.geo = Indy7Geometry()
        self.robot = Indy7Kinematics(self.geo)

    def test_home_pose_exact_match(self):
        """
        선배의 indy7_sim_fk.m 주석 기준:
        indy7_sim_fk(zeros(6,1)) puts the TCP at [0; -0.1865; 1.3275].
        오차 1e-6 m 이내 일치 검증
        """
        q_zero = np.zeros(6)
        T, J, axes = self.robot.forward_kinematics(q_zero)
        tcp_pos = T[:3, 3]

        expected_pos = np.array([0.0, -0.1865, 1.3275])
        np.testing.assert_allclose(
            tcp_pos, expected_pos, atol=1e-6,
            err_msg="홈 포즈 TCP 위치가 선배의 MATLAB Simscape 기준값과 일치하지 않습니다!"
        )
        print(f"\n[PASS] Home Pose TCP Position: {np.round(tcp_pos, 4)} == Expected: {expected_pos}")

    def test_parametric_tcp_offset_flexibility(self):
        """
        파라메트릭 유연성 검증:
        TCP 툴 길이를 60mm에서 120mm (+60mm)로 확장했을 때,
        엔드이펙터 끝단 Z좌표가 정확히 +0.060m 이동하는지 검증
        """
        custom_geo = Indy7Geometry(l_tcp=0.120)
        custom_robot = Indy7Kinematics(custom_geo)

        q_zero = np.zeros(6)
        T_custom, _, _ = custom_robot.forward_kinematics(q_zero)
        pos_custom = T_custom[:3, 3]

        expected_z = 1.3275 + 0.060  # 1.3875m
        self.assertAlmostEqual(pos_custom[2], expected_z, places=5)
        print(f"[PASS] Parametric Tool Offset: {pos_custom[2]:.4f}m == Expected: {expected_z:.4f}m")

    def test_jacobian_shape_and_finite(self):
        """자코비안 행렬 형태(6x6) 및 유효성(NaN/Inf 없음) 검증"""
        q_test = np.deg2rad([10, -20, 30, 0, 45, -15])
        T, J, _ = self.robot.forward_kinematics(q_test)

        self.assertEqual(J.shape, (6, 6))
        self.assertFalse(np.isnan(J).any())
        self.assertFalse(np.isinf(J).any())
        print(f"[PASS] Jacobian 6x6 Matrix validated. Rank = {np.linalg.matrix_rank(J)}")


if __name__ == "__main__":
    unittest.main()
