"""
Haviland et al. 2022 — holistic reactive QP controller.

Reference: "A Holistic Approach to Reactive Mobile Manipulation"
           IEEE RA-L Vol. 7, No. 2, April 2022.

See ALGORITHM_NOTES.md for full mathematical derivation and parameter guide.
"""
import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np
import scipy.sparse as sp
from scipy.spatial.transform import Rotation

try:
    import roboticstoolbox as rtb
    _HAS_RTB = True
except ImportError:
    _HAS_RTB = False

try:
    from qpsolvers import solve_qp
    _HAS_QP = True
except ImportError:
    _HAS_QP = False


@dataclass
class Pose3D:
    """Goal pose as loaded from YAML. Converted to SE3 by the node."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0


@dataclass
class BaseVelocity:
    """
    Non-holonomic base velocity command.
    vx → Twist.linear.x  (= q̇_b[1], forward virtual joint)
    wz → Twist.angular.z (= q̇_b[0], rotation virtual joint)
    """
    vx: float = 0.0
    wz: float = 0.0


@dataclass
class ArmJointVelocity:
    """Arm joint velocity command, published as JointJog to MoveIt Servo."""
    velocities: np.ndarray = field(default_factory=lambda: np.zeros(6))
    joint_names: list = field(default_factory=list)


class WholeBodyController:
    """
    Holistic QP controller for a non-holonomic mobile manipulator.

    The mobile base is modelled with 2 virtual joints (δθ, δd). The arm
    uses the UR10 model from roboticstoolbox-python. A QP is solved at
    each timestep to simultaneously command base and arm.

    Usage
    -----
    1. Instantiate with tuning parameters.
    2. Set `bTa` (4×4 base→arm-base transform) once TF is available.
    3. Call `compute()` each control cycle.

    Coordinate convention: all 4×4 SE3 matrices are in the map/world frame
    unless noted otherwise.
    """

    # Conservative UR10e joint velocity limits [rad/s].
    # All joints: UR10e datasheet max = 2.094 rad/s (120°/s).
    _QDLIM_ARM = np.full(6, 2.094)

    def __init__(
        self,
        pos_tol: float = 0.02,
        ori_tol: float = 0.05,
        beta: float = 1.5,
        k_eps: float = 0.5,
        k_a: float = 0.01,
        y_gain: float = 0.01,
        ps: float = 0.1,
        pi_influence: float = 0.9,
        eta: float = 1.0,
        max_base_linear_vel: float = 0.3,
        max_base_angular_vel: float = 0.5,
        joint_names: Optional[list] = None,
    ) -> None:
        if not _HAS_RTB:
            raise ImportError(
                'roboticstoolbox-python is required. '
                'Install: pip install roboticstoolbox-python'
            )
        if not _HAS_QP:
            raise ImportError(
                'qpsolvers[osqp] is required. '
                'Install: pip install qpsolvers[osqp]'
            )

        # RTB-P UR10 model (close to UR10e; differences are minor for a demo).
        self._arm = rtb.models.UR10()

        self.pos_tol = pos_tol
        self.ori_tol = ori_tol
        self.beta = beta
        self.k_eps = k_eps
        self.k_a = k_a
        self.y_gain = y_gain
        self.ps = ps
        self.pi = pi_influence
        self.eta = eta

        # q̇ bounds for base virtual joints: [rot_max, fwd_max]
        self._qdlim_base = np.array([max_base_angular_vel, max_base_linear_vel])

        self.joint_names: list = joint_names or []

        # Set by the ROS node after the static TF lookup (base_footprint → ur_base_link).
        self.bTa: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def is_goal_reached(self, oTe: np.ndarray, oTe_desired: np.ndarray) -> bool:
        """True when EE is within pos_tol of the desired pose."""
        return float(np.linalg.norm(oTe_desired[:3, 3] - oTe[:3, 3])) < self.pos_tol

    def compute(
        self,
        oTb: np.ndarray,       # 4×4: base pose in map frame
        oTe: np.ndarray,       # 4×4: current EE pose in map frame
        oTe_desired: np.ndarray,  # 4×4: desired EE pose in map frame
        q_a: np.ndarray,       # (6,): current arm joint angles [rad]
    ) -> tuple[BaseVelocity, ArmJointVelocity]:
        """
        Solve the holistic QP (Eq. 6) and return velocity commands.

        Returns zero commands if bTa is not yet set or the QP is infeasible.
        """
        zero_base = BaseVelocity()
        zero_arm = ArmJointVelocity(np.zeros(6), self.joint_names)

        if self.bTa is None:
            return zero_base, zero_arm

        n_b, n_a = 2, 6
        n = n_b + n_a   # 8 total joints

        # Position error magnitude for adaptive gain scaling.
        et = max(float(np.linalg.norm(oTe_desired[:3, 3] - oTe[:3, 3])), 1e-6)

        # --- Desired EE spatial velocity via PBS (Eq. 17) ---
        v_des = self._p_servo(oTe, oTe_desired)  # (6,) in map frame

        # --- Holistic Jacobian in map frame: 6×8 ---
        oTa = oTb @ self.bTa
        J = self._holistic_jacobian(oTb, oTe, oTa, q_a)

        # --- Q matrix: quadratic cost (Eq. 10) ---
        Q_mat = np.eye(n + 6)
        Q_mat[:n, :n] *= self.y_gain
        Q_mat[:2, :2] *= 1.0 / et         # base joints: cheaper when far
        Q_mat[n:, n:] = (1.0 / et) * np.eye(6)   # slack: tighter near goal

        # --- c vector: linear cost (Eq. 11) ---
        c = np.zeros(n + 6)
        c[n_b:n] = -self._manipulability_jacobian(q_a)    # arm manipulability
        bTe = np.linalg.inv(oTb) @ oTe
        theta_e = math.atan2(bTe[1, 3], bTe[0, 3])
        c[0] = -self.k_eps * theta_e      # base orientation toward EE (Eq. 13)

        # --- Equality constraint: J̃ x = v_des (Eq. 8) ---
        Aeq = np.c_[J, np.eye(6)]   # 6×14
        beq = v_des                  # (6,)

        # --- Inequality constraint: joint limit avoidance (Eq. 15–16) ---
        Ain, bin_ = self._joint_velocity_damper(q_a, n)

        # --- Velocity bounds ---
        lb = -np.r_[self._qdlim_base, self._QDLIM_ARM, 10.0 * np.ones(6)]
        ub = np.r_[self._qdlim_base, self._QDLIM_ARM, 10.0 * np.ones(6)]

        # --- Solve QP ---
        # OSQP requires sparse inputs; passing dense arrays generates a warning.
        try:
            sol = solve_qp(
                sp.csc_matrix(Q_mat.astype(np.float64)),
                c.astype(np.float64),
                G=sp.csc_matrix(Ain.astype(np.float64)),
                h=bin_.astype(np.float64),
                A=sp.csc_matrix(Aeq.astype(np.float64)),
                b=beq.astype(np.float64),
                lb=lb.astype(np.float64),
                ub=ub.astype(np.float64),
                solver='osqp',
            )
        except Exception:
            sol = None

        if sol is None:
            return zero_base, zero_arm

        qd = sol[:n].copy()

        # Velocity scaling from paper Section VI
        if et > 0.5:
            qd *= 0.7 / et
        else:
            qd *= 1.4

        # q̇[0] = δθ̇ → Twist.angular.z
        # q̇[1] = δḋ  → Twist.linear.x
        base_cmd = BaseVelocity(vx=float(qd[1]), wz=float(qd[0]))
        arm_cmd = ArmJointVelocity(
            velocities=np.array(qd[n_b:], dtype=float),
            joint_names=self.joint_names,
        )
        return base_cmd, arm_cmd

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _p_servo(self, oTe: np.ndarray, oTe_des: np.ndarray) -> np.ndarray:
        """
        Position-based servoing (Eq. 17): desired EE spatial velocity in map frame.

        v[:3] = β * translation error in map frame
        v[3:] = β * 1.3 * axis-angle orientation error
        """
        v = (oTe_des[:3, 3] - oTe[:3, 3]) * self.beta
        R_err = oTe_des[:3, :3] @ oTe[:3, :3].T
        omega = Rotation.from_matrix(R_err).as_rotvec() * self.beta * 1.3
        return np.r_[v, omega]

    def _holistic_jacobian(
        self,
        oTb: np.ndarray,   # base pose in map (4×4)
        oTe: np.ndarray,   # EE pose in map (4×4)
        oTa: np.ndarray,   # arm base pose in map (4×4)
        q_a: np.ndarray,   # arm joints (6,)
    ) -> np.ndarray:
        """
        Build the 6×8 holistic Jacobian in the map/world frame.

        Columns [0] and [1] correspond to the base virtual joints (Eq. 3).
        Columns [2:8] correspond to the UR10e arm joints.
        """
        # --- Base virtual joints in world frame ---
        # Column 0: rotation δθ at base position (z-axis at base origin)
        dp = oTe[:3, 3] - oTb[:3, 3]   # EE relative to base in world frame
        J_rot = np.array([-dp[1], dp[0], 0.0, 0.0, 0.0, 1.0])

        # Column 1: forward translation δd (base heading direction in world)
        base_yaw = math.atan2(oTb[1, 0], oTb[0, 0])
        J_fwd = np.array([math.cos(base_yaw), math.sin(base_yaw), 0.0, 0.0, 0.0, 0.0])

        J_base = np.column_stack([J_rot, J_fwd])   # 6×2

        # --- Arm Jacobian, rotated to world frame ---
        # arm.jacob0(q) gives the 6×6 Jacobian in the arm's base frame.
        # To express it in the world frame, rotate each velocity vector by oRa.
        oRa = oTa[:3, :3]
        J_arm_local = self._arm.jacob0(q_a)         # 6×6 in ur_base_link frame
        R_blk = np.block([[oRa, np.zeros((3, 3))],
                          [np.zeros((3, 3)), oRa]])  # 6×6 rotation block
        J_arm = R_blk @ J_arm_local                  # 6×6 in map frame

        return np.hstack([J_base, J_arm])   # 6×8

    def _manipulability_jacobian(self, q_a: np.ndarray) -> np.ndarray:
        """
        Numerical gradient of arm manipulability metric w.r.t. joint angles.
        Returns (6,) vector.
        """
        m0 = float(self._arm.manipulability(q_a))
        Jm = np.zeros(6)
        dq = 1e-6
        for i in range(6):
            q_p = q_a.copy()
            q_p[i] += dq
            Jm[i] = (float(self._arm.manipulability(q_p)) - m0) / dq
        return Jm

    def _joint_velocity_damper(
        self, q_a: np.ndarray, n: int
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Velocity dampers for joint limit avoidance (Eq. 15–16).

        Applied to arm joints only (base virtual joints have no limits).
        Returns Ain (n+6 × n+6) and bin_ (n+6,) for: Ain @ x ≤ bin_.
        """
        Ain = np.zeros((n + 6, n + 6))
        bin_ = np.zeros(n + 6)
        n_b = 2

        qlim = self._arm.qlim   # shape (2, 6): row 0 = lower, row 1 = upper

        for i in range(6):
            qi = float(q_a[i])
            q_lower = float(qlim[0, i])
            q_upper = float(qlim[1, i])
            idx = n_b + i

            rho_upper = q_upper - qi
            rho_lower = qi - q_lower

            # Damper fires when within pi of a limit; safety margin is ps.
            if rho_upper < self.pi and rho_upper > self.ps:
                Ain[idx, idx] = 1.0
                bin_[idx] = self.eta * (rho_upper - self.ps) / (self.pi - self.ps)
            elif rho_lower < self.pi and rho_lower > self.ps:
                Ain[idx, idx] = -1.0
                bin_[idx] = self.eta * (rho_lower - self.ps) / (self.pi - self.ps)

        return Ain, bin_
