"""
Unit tests for WholeBodyController.

Requires: roboticstoolbox-python, qpsolvers[osqp]
Install:  pip install -r src/moma_paper_demo/requirements.txt
"""
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

pytest.importorskip('roboticstoolbox', reason='roboticstoolbox-python not installed')
pytest.importorskip('qpsolvers', reason='qpsolvers not installed')

from moma_paper_demo.whole_body_controller import (
    ArmJointVelocity,
    BaseVelocity,
    WholeBodyController,
)

# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

# Realistic fixed transform: base_footprint → ur_base_link
# (arm is mounted ~0.35 m above and 0.18 m forward on MiR, values are illustrative)
_BTA = np.array([
    [1, 0, 0, 0.18],
    [0, 1, 0, 0.00],
    [0, 0, 1, 0.35],
    [0, 0, 0, 1.00],
], dtype=float)

# Home configuration for UR10 (from RTB-P model)
_JOINT_NAMES = [
    'ur_shoulder_pan_joint', 'ur_shoulder_lift_joint', 'ur_elbow_joint',
    'ur_wrist_1_joint', 'ur_wrist_2_joint', 'ur_wrist_3_joint',
]


@pytest.fixture
def ctrl():
    c = WholeBodyController(
        pos_tol=0.02,
        ori_tol=0.05,
        beta=1.5,
        k_eps=0.5,
        k_a=0.01,
        y_gain=0.01,
        ps=0.1,
        pi_influence=0.9,
        eta=1.0,
        max_base_linear_vel=0.3,
        max_base_angular_vel=0.5,
        joint_names=_JOINT_NAMES,
    )
    c.bTa = _BTA.copy()
    return c


def _identity_base() -> np.ndarray:
    return np.eye(4)


def _oTe_from_xyz(x, y, z) -> np.ndarray:
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def _q_home(ctrl) -> np.ndarray:
    """UR10 home configuration from RTB-P model."""
    return ctrl._arm.qr.copy()


# ---------------------------------------------------------------------------
# Goal detection
# ---------------------------------------------------------------------------

def test_goal_not_reached_when_far(ctrl):
    oTe = _oTe_from_xyz(0.0, 0.0, 0.5)
    oTe_des = _oTe_from_xyz(2.0, 1.0, 1.2)
    assert not ctrl.is_goal_reached(oTe, oTe_des)


def test_goal_reached_within_tolerance(ctrl):
    oTe_des = _oTe_from_xyz(2.0, 1.0, 1.2)
    oTe = _oTe_from_xyz(2.01, 1.0, 1.2)   # 0.01 m < pos_tol=0.02
    assert ctrl.is_goal_reached(oTe, oTe_des)


def test_goal_not_reached_just_outside_tolerance(ctrl):
    oTe_des = _oTe_from_xyz(2.0, 1.0, 1.2)
    oTe = _oTe_from_xyz(2.03, 1.0, 1.2)   # 0.03 m > pos_tol=0.02
    assert not ctrl.is_goal_reached(oTe, oTe_des)


# ---------------------------------------------------------------------------
# QP solve: return types and structure
# ---------------------------------------------------------------------------

def test_compute_returns_correct_types(ctrl):
    oTb = _identity_base()
    oTe = _oTe_from_xyz(0.5, 0.0, 1.0)
    oTe_des = _oTe_from_xyz(2.0, 0.0, 1.2)
    q_a = _q_home(ctrl)
    base_cmd, arm_cmd = ctrl.compute(oTb, oTe, oTe_des, q_a)
    assert isinstance(base_cmd, BaseVelocity)
    assert isinstance(arm_cmd, ArmJointVelocity)
    assert arm_cmd.velocities.shape == (6,)
    assert arm_cmd.joint_names == _JOINT_NAMES


def test_compute_returns_zeros_without_bTa(ctrl):
    ctrl.bTa = None
    oTb = _identity_base()
    oTe = _oTe_from_xyz(0.0, 0.0, 1.0)
    oTe_des = _oTe_from_xyz(2.0, 0.0, 1.2)
    base_cmd, arm_cmd = ctrl.compute(oTb, oTe, oTe_des, np.zeros(6))
    assert base_cmd.vx == 0.0 and base_cmd.wz == 0.0
    assert np.all(arm_cmd.velocities == 0.0)


# ---------------------------------------------------------------------------
# QP solve: velocity direction and limits
# ---------------------------------------------------------------------------

def test_base_moves_forward_for_goal_ahead(ctrl):
    """Goal directly ahead → base should move forward (+x in world = +vx)."""
    oTb = _identity_base()
    oTe = oTb @ _BTA @ _oTe_from_xyz(0.3, 0.0, 0.5)   # EE above arm base
    oTe_des = _oTe_from_xyz(3.0, 0.0, 1.0)             # goal far ahead
    q_a = _q_home(ctrl)
    base_cmd, _ = ctrl.compute(oTb, oTe, oTe_des, q_a)
    assert base_cmd.vx > 0.0


def test_base_velocity_within_limits(ctrl):
    """QP bounds should clamp base velocities to configured limits."""
    oTb = _identity_base()
    oTe = _oTe_from_xyz(0.0, 0.0, 1.0)
    oTe_des = _oTe_from_xyz(100.0, 0.0, 1.0)   # huge error
    q_a = _q_home(ctrl)
    base_cmd, _ = ctrl.compute(oTb, oTe, oTe_des, q_a)
    assert abs(base_cmd.vx) <= ctrl._qdlim_base[1] * 1.41 + 1e-6   # ×1.4 scaling factor
    assert abs(base_cmd.wz) <= ctrl._qdlim_base[0] * 1.41 + 1e-6


def test_arm_velocity_within_limits(ctrl):
    """All arm joint velocities must stay within UR10e limits."""
    oTb = _identity_base()
    oTe = _oTe_from_xyz(0.0, 0.0, 1.0)
    oTe_des = _oTe_from_xyz(0.0, 0.0, 100.0)   # huge vertical error
    q_a = _q_home(ctrl)
    _, arm_cmd = ctrl.compute(oTb, oTe, oTe_des, q_a)
    qdlim = ctrl._QDLIM_ARM
    assert np.all(np.abs(arm_cmd.velocities) <= qdlim * 1.41 + 1e-6)


# ---------------------------------------------------------------------------
# QP solve: velocity scaling
# ---------------------------------------------------------------------------

def test_velocity_scaling_near_vs_far(ctrl):
    """Commands near goal (et < 0.5) should scale differently from far (et > 0.5)."""
    oTb = _identity_base()
    q_a = _q_home(ctrl)

    oTe_far = _oTe_from_xyz(0.0, 0.0, 1.0)
    oTe_des = _oTe_from_xyz(1.0, 0.0, 1.0)   # 1 m away (et > 0.5 → scale 0.7/et)

    oTe_near = _oTe_from_xyz(0.85, 0.0, 1.0)  # 0.15 m away (et < 0.5 → scale 1.4)

    base_far, _ = ctrl.compute(oTb, oTe_far, oTe_des, q_a)
    base_near, _ = ctrl.compute(oTb, oTe_near, oTe_des, q_a)

    # When far: 0.7/1.0 = 0.7×; when near: 1.4×. Near should have higher speed per unit error.
    speed_per_error_far = abs(base_far.vx) / 1.0
    speed_per_error_near = abs(base_near.vx) / 0.15
    assert speed_per_error_near > speed_per_error_far


# ---------------------------------------------------------------------------
# Holistic Jacobian
# ---------------------------------------------------------------------------

def test_holistic_jacobian_shape(ctrl):
    oTb = _identity_base()
    oTe = _oTe_from_xyz(0.5, 0.0, 1.2)
    oTa = oTb @ _BTA
    q_a = _q_home(ctrl)
    J = ctrl._holistic_jacobian(oTb, oTe, oTa, q_a)
    assert J.shape == (6, 8)


def test_holistic_jacobian_base_rotation_column(ctrl):
    """Column 0 of J (rotation virtual joint) must have correct structure."""
    oTb = np.eye(4)
    oTe = _oTe_from_xyz(1.0, 0.5, 1.0)   # EE at (1, 0.5, 1) in world
    oTa = oTb @ _BTA
    q_a = _q_home(ctrl)
    J = ctrl._holistic_jacobian(oTb, oTe, oTa, q_a)

    dp = oTe[:3, 3] - oTb[:3, 3]
    expected_lin = np.array([-dp[1], dp[0], 0.0])
    np.testing.assert_allclose(J[:3, 0], expected_lin, atol=1e-10)
    np.testing.assert_allclose(J[3:, 0], [0, 0, 1], atol=1e-10)


def test_holistic_jacobian_base_forward_column(ctrl):
    """Column 1 (forward translation) must point in base heading direction."""
    yaw = math.pi / 4   # 45° base heading
    oTb = np.eye(4)
    oTb[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    oTe = _oTe_from_xyz(1.0, 1.0, 1.0)
    oTa = oTb @ _BTA
    q_a = _q_home(ctrl)
    J = ctrl._holistic_jacobian(oTb, oTe, oTa, q_a)

    expected_lin = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    np.testing.assert_allclose(J[:3, 1], expected_lin, atol=1e-10)
    np.testing.assert_allclose(J[3:, 1], [0, 0, 0], atol=1e-10)


# ---------------------------------------------------------------------------
# PBS
# ---------------------------------------------------------------------------

def test_p_servo_zero_error_gives_zero(ctrl):
    oTe = _oTe_from_xyz(1.0, 0.5, 1.2)
    v = ctrl._p_servo(oTe, oTe)
    np.testing.assert_allclose(v, np.zeros(6), atol=1e-12)


def test_p_servo_translation_direction(ctrl):
    oTe = _oTe_from_xyz(0.0, 0.0, 1.0)
    oTe_des = _oTe_from_xyz(1.0, 0.0, 1.0)
    v = ctrl._p_servo(oTe, oTe_des)
    assert v[0] > 0.0   # positive x velocity
    assert abs(v[1]) < 1e-10  # no y
    assert abs(v[2]) < 1e-10  # no z
