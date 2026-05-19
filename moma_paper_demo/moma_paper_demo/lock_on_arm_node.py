"""Arm-only Jacobian controller that keeps the EE locked on a world-frame pose.

Demo concept
------------
The MiR drives forward (world X) while oscillating laterally (world Y) in a sine
wave.  The arm tracks a fixed (Y, Z) position AND a fixed orientation in the
world frame — keeping the EE on a straight horizontal line, pointing the same
way regardless of how the base rotates — while X is left unconstrained so the
arm rides forward with the robot naturally.

Only the tracked axes contribute rows to the Jacobian, so the damped-least-squares
solution minimises joint motion while correcting only the controlled dimensions.

A secondary null-space task pulls the arm back toward a reference posture (the
configuration at the moment of enabling) in the redundant DOF, preventing the arm
from drifting into self-colliding configurations over time.

Data exchange
-------------
The arm base pose (map → ur_base_link) changes every cycle as the MiR moves.
Reading it from TF is the mechanism by which the cobot consumes the AMR's live
position — demonstrating holistic data exchange between the two robots.

Output
------
Publishes JointTrajectory directly to /ur_manipulator_controller/joint_trajectory.
Each cycle integrates the Jacobian velocity solution one time-step forward and
sends a 1-point trajectory.  The controller interpolates smoothly between points.

Enable via:
  ros2 param set /lock_on_arm_node enabled true

If the arm drifts into a bad configuration, recover via:
  ros2 param set /lock_on_arm_node go_home true
"""
import math
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

from builtin_interfaces.msg import Duration

import rclpy
import rclpy.time
from rclpy.node import Node
import tf2_ros
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    import roboticstoolbox as rtb
    _HAS_RTB = True
except ImportError:
    _HAS_RTB = False

# UR10e joint names — must match /joint_states and MoveIt SRDF (ur_ prefix).
_UR10E_JOINT_NAMES = [
    'ur_shoulder_pan_joint',
    'ur_shoulder_lift_joint',
    'ur_elbow_joint',
    'ur_wrist_1_joint',
    'ur_wrist_2_joint',
    'ur_wrist_3_joint',
]

# SRDF "Home" state for ur_manipulator — used as default reference posture and
# for go_home recovery.  In order: pan, lift, elbow, wrist1, wrist2, wrist3.
_HOME_JOINTS = np.array([-0.7853, -1.5000, 2.3561, 0.0, 0.0, 0.0])


class LockOnArmNode(Node):
    """Keeps the UR10e EE on a fixed world-frame line by controlling only selected axes.

    At each control cycle the node:
      1. Reads current arm joint angles from /joint_states.
      2. Looks up the arm base pose (map → ur_base_link) from TF.
      3. Builds the n×6 Jacobian for the tracked axes only (n ≤ 3):
           J_tracked = (oRa · J_arm_base[:3, :])[tracked_axes, :]
      4. Solves for joint velocities via damped least-squares (primary task):
           q̇_p = Jᵀ (J Jᵀ + λ² I)⁻¹ · k_p · e_tracked
      5. Adds null-space posture control (secondary task — does not affect EE):
           q̇   = q̇_p + (I − J† J) · k_ns · (q_ref − q)
      6. Clamps per-joint velocities and publishes a 1-point JointTrajectory.

    Default: track_x=false, track_y=true, track_z=true.
    The EE rides forward (X) with the robot while the arm cancels the sine-wave
    lateral oscillation (Y) and holds fixed height (Z).
    """

    def __init__(self) -> None:
        super().__init__('lock_on_arm_node')

        if not _HAS_RTB:
            self.get_logger().fatal(
                'roboticstoolbox-python not found. '
                'Install: pip install roboticstoolbox-python'
            )
            raise ImportError('roboticstoolbox-python required')

        self._declare_parameters()

        self._target = np.array([
            self.get_parameter('target.x').value,
            self.get_parameter('target.y').value,
            self.get_parameter('target.z').value,
        ])
        # Target orientation as a 3x3 rotation matrix.  Identity until the first
        # auto-capture (or first control cycle if auto_capture is off).
        self._target_R = np.eye(3)
        self._k_p = self.get_parameter('k_p').value
        self._k_p_rot = self.get_parameter('k_p_rot').value
        self._k_ns = self.get_parameter('k_ns').value
        self._damping = self.get_parameter('damping').value
        self._max_joint_vel = self.get_parameter('max_joint_vel').value
        self._pos_hold_tol = self.get_parameter('pos_hold_tol').value
        self._rot_hold_tol = self.get_parameter('rot_hold_tol').value
        self._rate = self.get_parameter('control_rate').value
        self._map_frame = self.get_parameter('map_frame').value
        self._arm_base_frame = self.get_parameter('arm_base_frame').value
        self._ee_frame = self.get_parameter('ee_frame').value
        self._enabled = self.get_parameter('enabled').value

        # Indices of the world-frame position axes to control (0=X, 1=Y, 2=Z).
        self._tracked_axes = [
            i for i, flag in enumerate([
                self.get_parameter('track_x').value,
                self.get_parameter('track_y').value,
                self.get_parameter('track_z').value,
            ]) if flag
        ]

        # Indices of the world-frame orientation axes to control (0=Rx, 1=Ry, 2=Rz).
        self._tracked_rot_axes = [
            i for i, flag in enumerate([
                self.get_parameter('track_roll').value,
                self.get_parameter('track_pitch').value,
                self.get_parameter('track_yaw').value,
            ]) if flag
        ]

        if not self._tracked_axes and not self._tracked_rot_axes:
            raise ValueError(
                'At least one of track_x/y/z or track_roll/pitch/yaw must be true.'
            )

        self._auto_capture = self.get_parameter('auto_capture_target').value

        # Reference posture for null-space control.  Updated to the actual joint
        # configuration at the moment 'enabled' flips to true, so the arm is
        # pulled back toward wherever the user positioned it before enabling.
        self._q_ref = _HOME_JOINTS.copy()

        # RTB UR10 model — used for Jacobian computation only.
        self._arm = rtb.models.UR10()

        # Current arm joint angles — updated from /joint_states.
        self._q_a = np.zeros(6)
        self._q_a_ready = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10
        )
        self._traj_pub = self.create_publisher(
            JointTrajectory, '/ur_manipulator_controller/joint_trajectory', 10
        )

        self.add_on_set_parameters_callback(self._on_parameter_event)

        self._timer = self.create_timer(1.0 / self._rate, self._control_loop)

        pos_labels = ['X', 'Y', 'Z']
        rot_labels = ['Rx', 'Ry', 'Rz']
        tracked_pos = [pos_labels[i] for i in self._tracked_axes]
        tracked_rot = [rot_labels[i] for i in self._tracked_rot_axes]
        self.get_logger().info(
            f'LockOnArmNode ready. '
            f'Position axes: {tracked_pos}. Orientation axes: {tracked_rot}. '
            f'Target pos: ({self._target[0]:.3f}, {self._target[1]:.3f}, {self._target[2]:.3f}) m [map]. '
            f'enabled={self._enabled}. '
            f'Tip: ros2 param set /lock_on_arm_node go_home true  to recover from bad posture.'
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _joint_states_cb(self, msg: JointState) -> None:
        for i, name in enumerate(_UR10E_JOINT_NAMES):
            if name in msg.name:
                self._q_a[i] = msg.position[msg.name.index(name)]
        self._q_a_ready = True

    def _on_parameter_event(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'enabled':
                if p.value and not self._enabled and self._auto_capture:
                    if not self._snapshot_current_ee_as_target():
                        self.get_logger().warning(
                            'Cannot enable: TF map → ur_tool0 not available yet. '
                            'Try again in a moment.'
                        )
                        return SetParametersResult(
                            successful=False,
                            reason='TF not available for auto target capture',
                        )
                self._enabled = p.value
                if not p.value:
                    self._publish_stop()
                self.get_logger().info(
                    f'LockOnArmNode {"ENABLED — tracking target." if p.value else "DISABLED."}'
                )

            elif p.name == 'go_home':
                if p.value:
                    if self._enabled:
                        self.get_logger().warning(
                            'Disabling lock-on to go home. '
                            'Re-enable after arm reaches home.'
                        )
                        self._enabled = False
                    self._send_home()

            elif p.name == 'k_p':
                self._k_p = p.value
            elif p.name == 'k_p_rot':
                self._k_p_rot = p.value
            elif p.name == 'k_ns':
                self._k_ns = p.value
            elif p.name == 'damping':
                self._damping = p.value
            elif p.name == 'max_joint_vel':
                self._max_joint_vel = p.value
            elif p.name == 'pos_hold_tol':
                self._pos_hold_tol = p.value
            elif p.name == 'rot_hold_tol':
                self._rot_hold_tol = p.value

        return SetParametersResult(successful=True)

    def _snapshot_current_ee_as_target(self) -> bool:
        """Snapshot current EE pose (position + orientation) as the lock-on target,
        and current joints as the null-space posture reference."""
        oTe_tf = self._lookup_transform(self._map_frame, self._ee_frame)
        if oTe_tf is None:
            return False
        self._target = np.array([
            oTe_tf.transform.translation.x,
            oTe_tf.transform.translation.y,
            oTe_tf.transform.translation.z,
        ])
        self._target_R = Rotation.from_quat([
            oTe_tf.transform.rotation.x,
            oTe_tf.transform.rotation.y,
            oTe_tf.transform.rotation.z,
            oTe_tf.transform.rotation.w,
        ]).as_matrix()
        # Also capture the current joint configuration as the null-space reference.
        # This prevents the arm from drifting away from the posture the user chose.
        if self._q_a_ready:
            self._q_ref = self._q_a.copy()

        pos_labels = ['X', 'Y', 'Z']
        rot_labels = ['Rx', 'Ry', 'Rz']
        tracked_pos = [pos_labels[i] for i in self._tracked_axes]
        tracked_rot = [rot_labels[i] for i in self._tracked_rot_axes]
        rpy = Rotation.from_matrix(self._target_R).as_euler('xyz', degrees=True)
        self.get_logger().info(
            f'Captured EE target: '
            f'pos=({self._target[0]:.3f}, {self._target[1]:.3f}, {self._target[2]:.3f}) m, '
            f'rpy=({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}) deg [map]. '
            f'Posture reference locked. Pos axes: {tracked_pos}. Rot axes: {tracked_rot}.'
        )
        return True

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if not self._enabled:
            return

        if not self._q_a_ready:
            self.get_logger().warning(
                'Waiting for /joint_states...', throttle_duration_sec=2.0
            )
            return

        # Arm base rotation in world frame (changes as MiR moves — the data-exchange link).
        oTa_tf = self._lookup_transform(self._map_frame, self._arm_base_frame)
        if oTa_tf is None:
            return

        oRa = Rotation.from_quat([
            oTa_tf.transform.rotation.x,
            oTa_tf.transform.rotation.y,
            oTa_tf.transform.rotation.z,
            oTa_tf.transform.rotation.w,
        ]).as_matrix()

        # Current EE pose (position + orientation) in world frame.
        oTe_tf = self._lookup_transform(self._map_frame, self._ee_frame)
        if oTe_tf is None:
            return

        ee_pos = np.array([
            oTe_tf.transform.translation.x,
            oTe_tf.transform.translation.y,
            oTe_tf.transform.translation.z,
        ])
        ee_R = Rotation.from_quat([
            oTe_tf.transform.rotation.x,
            oTe_tf.transform.rotation.y,
            oTe_tf.transform.rotation.z,
            oTe_tf.transform.rotation.w,
        ]).as_matrix()

        # --- Compute errors ---
        # Position error in world frame (m).
        e_pos_full = self._target - ee_pos
        e_pos = e_pos_full[self._tracked_axes] if self._tracked_axes else np.zeros(0)
        pos_norm = float(np.linalg.norm(e_pos)) if len(e_pos) else 0.0

        # Orientation error: rotvec of the rotation that maps current → target,
        # expressed in the world frame (rad).
        R_err = self._target_R @ ee_R.T
        e_rot_full = Rotation.from_matrix(R_err).as_rotvec()
        e_rot = e_rot_full[self._tracked_rot_axes] if self._tracked_rot_axes else np.zeros(0)
        rot_norm = float(np.linalg.norm(e_rot)) if len(e_rot) else 0.0

        # Hold if both position and orientation errors are below their tolerances.
        if pos_norm < self._pos_hold_tol and rot_norm < self._rot_hold_tol:
            self.get_logger().info(
                f'EE on target (pos={pos_norm*1000:.1f} mm, '
                f'rot={np.degrees(rot_norm):.2f} deg). Holding.',
                throttle_duration_sec=2.0,
            )
            self._publish_stop()
            return

        # --- Build combined task Jacobian and stacked error vector ---
        J_full = self._arm.jacob0(self._q_a)         # 6×6 in ur_base_link frame
        J_world_pos = oRa @ J_full[:3, :]            # 3×6 linear in world frame
        J_world_rot = oRa @ J_full[3:, :]            # 3×6 angular in world frame

        rows = []
        task_err = []
        if self._tracked_axes:
            rows.append(J_world_pos[self._tracked_axes, :])
            task_err.append(self._k_p * e_pos)
        if self._tracked_rot_axes:
            rows.append(J_world_rot[self._tracked_rot_axes, :])
            task_err.append(self._k_p_rot * e_rot)

        J_task = np.vstack(rows)                      # m×6, m = pos_axes + rot_axes
        x_dot = np.concatenate(task_err)              # m

        # --- Primary task: damped least-squares ---
        m = J_task.shape[0]
        lam2 = self._damping ** 2
        JJT = J_task @ J_task.T + lam2 * np.eye(m)
        q_dot = J_task.T @ np.linalg.solve(JJT, x_dot)

        # --- Secondary task: null-space posture control ---
        if self._k_ns > 0.0:
            J_pinv = J_task.T @ np.linalg.solve(JJT, np.eye(m))   # 6×m
            null_proj = np.eye(6) - J_pinv @ J_task                # 6×6
            q_dot_posture = self._k_ns * (self._q_ref - self._q_a)
            q_dot = q_dot + null_proj @ q_dot_posture

        q_dot = self._clamp_velocity(q_dot)
        self._publish_arm_cmd(q_dot)

        self.get_logger().info(
            f'err: pos={pos_norm*1000:.1f} mm, rot={np.degrees(rot_norm):.2f} deg  '
            f'|q̇|_max={np.max(np.abs(q_dot)):.3f} rad/s  '
            f'posture_err={np.linalg.norm(self._q_ref - self._q_a):.2f} rad',
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_arm_cmd(self, q_dot: np.ndarray) -> None:
        """Integrate velocity one step and send as a 1-point JointTrajectory."""
        dt = 1.0 / self._rate
        q_next = self._q_a + q_dot * dt

        msg = JointTrajectory()
        # Zero stamp → controller uses reception time as reference, avoids
        # sim-time deadline races.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = _UR10E_JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = q_next.tolist()
        pt.velocities = q_dot.tolist()
        pt.time_from_start = Duration(sec=0, nanosec=int(dt * 1e9))

        msg.points = [pt]
        self._traj_pub.publish(msg)

    def _publish_stop(self) -> None:
        """Send the current joint position as a hold trajectory."""
        msg = JointTrajectory()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = _UR10E_JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = self._q_a.tolist()
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)  # 100 ms

        msg.points = [pt]
        self._traj_pub.publish(msg)

    def _send_home(self) -> None:
        """Send arm to the SRDF Home configuration over 5 seconds."""
        if not self._q_a_ready:
            self.get_logger().warning('Cannot go home: /joint_states not yet received.')
            return

        msg = JointTrajectory()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.joint_names = _UR10E_JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = _HOME_JOINTS.tolist()
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=5, nanosec=0)

        msg.points = [pt]
        self._traj_pub.publish(msg)
        self.get_logger().info(
            f'Homing arm to: {np.round(_HOME_JOINTS, 3).tolist()} rad over 5 s. '
            f'Enable tracking after it arrives.'
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _clamp_velocity(self, q_dot: np.ndarray) -> np.ndarray:
        """Scale the entire velocity vector down if any joint exceeds the limit."""
        max_v = float(np.max(np.abs(q_dot)))
        if max_v > self._max_joint_vel:
            q_dot = q_dot * (self._max_joint_vel / max_v)
        return q_dot

    def _lookup_transform(self, target: str, source: str) -> Optional[object]:
        try:
            return self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warning(
                f'TF {source} → {target}: {e}', throttle_duration_sec=2.0
            )
            return None

    def _declare_parameters(self) -> None:
        self.declare_parameter('target.x', 0.5)
        self.declare_parameter('target.y', 0.0)
        self.declare_parameter('target.z', 1.2)
        self.declare_parameter('auto_capture_target', True)
        # Position axes to control in the world frame.
        self.declare_parameter('track_x', False)
        self.declare_parameter('track_y', True)
        self.declare_parameter('track_z', True)
        # Orientation axes to control in the world frame (Rx, Ry, Rz).
        self.declare_parameter('track_roll', True)
        self.declare_parameter('track_pitch', True)
        self.declare_parameter('track_yaw', True)
        # Position gain [1/s] — converts position error (m) into linear velocity (m/s).
        self.declare_parameter('k_p', 1.0)
        # Orientation gain [1/s] — converts angular error (rad) into angular velocity (rad/s).
        self.declare_parameter('k_p_rot', 1.0)
        # Null-space gain — pulls arm toward reference posture without affecting EE.
        # Set to 0.0 to disable null-space control.
        self.declare_parameter('k_ns', 0.5)
        self.declare_parameter('damping', 0.05)
        self.declare_parameter('max_joint_vel', 1.0)
        # Hold tolerances — once below these, the node sends a stop trajectory.
        self.declare_parameter('pos_hold_tol', 0.002)   # 2 mm
        self.declare_parameter('rot_hold_tol', 0.02)    # ~1.1 deg
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('arm_base_frame', 'ur_base_link')
        self.declare_parameter('ee_frame', 'ur_tool0')
        self.declare_parameter('enabled', False)
        # Write-only trigger: set to true to move arm to SRDF Home over 5 s.
        self.declare_parameter('go_home', False)


def main() -> None:
    rclpy.init()
    node = LockOnArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
