import math
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
import rclpy.time
from rclpy.node import Node
import tf2_ros
from control_msgs.msg import JointJog
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState

from moma_paper_demo.whole_body_controller import (
    ArmJointVelocity,
    BaseVelocity,
    Pose3D,
    WholeBodyController,
)

# UR10e joint names — must match /joint_states and MoveIt SRDF (ur_ prefix).
UR10E_JOINT_NAMES = [
    'ur_shoulder_pan_joint',
    'ur_shoulder_lift_joint',
    'ur_elbow_joint',
    'ur_wrist_1_joint',
    'ur_wrist_2_joint',
    'ur_wrist_3_joint',
]


class ControllerNode(Node):
    """
    ROS 2 wrapper for WholeBodyController.

    Reads current poses from TF and arm joint angles from /joint_states,
    solves the holistic QP, and publishes:
      /cmd_vel                        geometry_msgs/Twist    → MiR base
      /servo_node/delta_joint_cmds    control_msgs/JointJog  → MoveIt Servo (arm)
    """

    def __init__(self) -> None:
        super().__init__('controller_node')
        self._declare_parameters()

        goal = Pose3D(
            x=self.get_parameter('goal_pose.x').value,
            y=self.get_parameter('goal_pose.y').value,
            z=self.get_parameter('goal_pose.z').value,
            qx=self.get_parameter('goal_pose.qx').value,
            qy=self.get_parameter('goal_pose.qy').value,
            qz=self.get_parameter('goal_pose.qz').value,
            qw=self.get_parameter('goal_pose.qw').value,
        )
        self._goal_oTe = self._pose3d_to_matrix(goal)

        self._controller = WholeBodyController(
            pos_tol=self.get_parameter('pos_tol').value,
            ori_tol=self.get_parameter('ori_tol').value,
            beta=self.get_parameter('beta').value,
            k_eps=self.get_parameter('k_eps').value,
            k_a=self.get_parameter('k_a').value,
            y_gain=self.get_parameter('y_gain').value,
            ps=self.get_parameter('ps').value,
            pi_influence=self.get_parameter('pi_influence').value,
            eta=self.get_parameter('eta').value,
            max_base_linear_vel=self.get_parameter('max_base_linear_vel').value,
            max_base_angular_vel=self.get_parameter('max_base_angular_vel').value,
            joint_names=UR10E_JOINT_NAMES,
        )

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._ee_frame = self.get_parameter('ee_frame').value
        self._arm_base_frame = self.get_parameter('arm_base_frame').value
        self._control_rate = self.get_parameter('control_rate').value
        self._enabled = self.get_parameter('enabled').value

        # Current arm joint angles (updated from /joint_states).
        self._q_a = np.zeros(6)
        self._q_a_ready = False

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10
        )
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._joint_jog_pub = self.create_publisher(
            JointJog, '/servo_node/delta_joint_cmds', 10
        )

        self.add_on_set_parameters_callback(self._on_parameter_event)

        self._timer = self.create_timer(1.0 / self._control_rate, self._control_loop)

        self.get_logger().info(
            f'WholeBodyController ready. '
            f'Goal EE: ({goal.x:.2f}, {goal.y:.2f}, {goal.z:.2f}) [map]. '
            f'enabled={self._enabled}'
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _joint_states_cb(self, msg: JointState) -> None:
        """Extract UR10e joint positions from the merged /joint_states topic."""
        for i, name in enumerate(UR10E_JOINT_NAMES):
            if name in msg.name:
                self._q_a[i] = msg.position[msg.name.index(name)]
        self._q_a_ready = True

    def _on_parameter_event(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'enabled':
                self._enabled = p.value
                self.get_logger().info(
                    f'Controller {"ENABLED — publishing /cmd_vel and arm commands" if p.value else "DISABLED — idle, Nav2 owns /cmd_vel"}'
                )
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if not self._enabled:
            return

        # Cache the static base → arm transform once.
        if self._controller.bTa is None:
            bTa_tf = self._lookup_transform(self._base_frame, self._arm_base_frame)
            if bTa_tf is None:
                return
            self._controller.bTa = self._tf_to_matrix(bTa_tf)
            self.get_logger().info('Arm base transform cached. Controller active.')

        if not self._q_a_ready:
            self.get_logger().warning(
                'Waiting for /joint_states...', throttle_duration_sec=2.0
            )
            return

        oTb_tf = self._lookup_transform(self._map_frame, self._base_frame)
        oTe_tf = self._lookup_transform(self._map_frame, self._ee_frame)
        if oTb_tf is None or oTe_tf is None:
            return

        oTb = self._tf_to_matrix(oTb_tf)
        oTe = self._tf_to_matrix(oTe_tf)

        if self._controller.is_goal_reached(oTe, self._goal_oTe):
            self.get_logger().info('Goal reached. Stopping controller.')
            self._stop()
            return

        base_cmd, arm_cmd = self._controller.compute(
            oTb, oTe, self._goal_oTe, self._q_a.copy()
        )
        self._publish_base_cmd(base_cmd)
        self._publish_arm_cmd(arm_cmd)

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_base_cmd(self, cmd: BaseVelocity) -> None:
        msg = Twist()
        msg.linear.x = cmd.vx
        msg.angular.z = cmd.wz
        self._cmd_vel_pub.publish(msg)

    def _publish_arm_cmd(self, cmd: ArmJointVelocity) -> None:
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.joint_names = cmd.joint_names
        msg.velocities = cmd.velocities.tolist()
        msg.duration = 1.0 / self._control_rate
        self._joint_jog_pub.publish(msg)

    def _stop(self) -> None:
        self._timer.cancel()
        if not rclpy.ok():
            return
        self._cmd_vel_pub.publish(Twist())
        empty_jog = JointJog()
        empty_jog.joint_names = UR10E_JOINT_NAMES
        empty_jog.velocities = [0.0] * 6
        self._joint_jog_pub.publish(empty_jog)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _lookup_transform(self, target: str, source: str) -> Optional[object]:
        try:
            return self._tf_buffer.lookup_transform(target, source, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warning(
                f'TF {source} → {target}: {e}', throttle_duration_sec=2.0
            )
            return None

    @staticmethod
    def _tf_to_matrix(tf) -> np.ndarray:
        """Convert TF TransformStamped.transform to 4×4 SE3 numpy array."""
        t = tf.transform.translation
        r = tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    @staticmethod
    def _pose3d_to_matrix(p: Pose3D) -> np.ndarray:
        """Convert goal Pose3D (from YAML params) to 4×4 SE3 numpy array."""
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([p.qx, p.qy, p.qz, p.qw]).as_matrix()
        T[:3, 3] = [p.x, p.y, p.z]
        return T

    def _declare_parameters(self) -> None:
        self.declare_parameter('goal_pose.x', 2.0)
        self.declare_parameter('goal_pose.y', 1.0)
        self.declare_parameter('goal_pose.z', 1.2)
        self.declare_parameter('goal_pose.qx', 0.0)
        self.declare_parameter('goal_pose.qy', 0.0)
        self.declare_parameter('goal_pose.qz', 0.0)
        self.declare_parameter('goal_pose.qw', 1.0)
        self.declare_parameter('pos_tol', 0.02)
        self.declare_parameter('ori_tol', 0.05)
        self.declare_parameter('beta', 1.5)
        self.declare_parameter('k_eps', 0.5)
        self.declare_parameter('k_a', 0.01)
        self.declare_parameter('y_gain', 0.01)
        self.declare_parameter('ps', 0.1)
        self.declare_parameter('pi_influence', 0.9)
        self.declare_parameter('eta', 1.0)
        self.declare_parameter('max_base_linear_vel', 0.3)
        self.declare_parameter('max_base_angular_vel', 0.5)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('ee_frame', 'ur_tool0')
        self.declare_parameter('arm_base_frame', 'ur_base_link')
        self.declare_parameter('enabled', False)


def main() -> None:
    rclpy.init()
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
