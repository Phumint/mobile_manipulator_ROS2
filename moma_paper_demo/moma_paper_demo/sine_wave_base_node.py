"""Drives the MiR base in a sine wave trajectory via open-loop feedforward cmd_vel.

The node computes the exact angular velocity feedforward required for a non-holonomic
robot to trace y(x) = A * sin(2π * x / λ) in the world frame, given constant forward
speed v.  Both nodes in this demo (this one and lock_on_arm_node) read the TF tree,
demonstrating that the AMR and cobot continuously share pose data.

Prerequisites:
  1. ros2 launch moma_bringup moma_system.launch.py      use_sim:=<true|false>
  2. ros2 launch moma_bringup moma_nav_moveit.launch.py  use_sim:=<true|false>  map:=<path>
  3. ros2 launch moma_paper_demo holistic_demo.launch.py use_sim:=<true|false>
  4. ros2 param set /sine_wave_base_node enabled true   (when ready to move)
"""
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist


class SineWaveBaseNode(Node):
    """Publishes cmd_vel to make the MiR trace a sine wave path.

    Forward velocity is constant (linear_vel).  Angular velocity is the exact
    curvature feedforward:
        ω(t) = -v · A · ω_s² · sin(ω_s · t) / (v² + (A · ω_s · cos(ω_s · t))²)
    where ω_s = 2π · v / λ  (temporal angular frequency for the given forward speed).
    """

    def __init__(self) -> None:
        super().__init__('sine_wave_base_node')
        self._declare_parameters()

        self._v = self.get_parameter('linear_vel').value
        self._A = self.get_parameter('amplitude').value
        self._lambda = self.get_parameter('wavelength').value
        self._rate = self.get_parameter('control_rate').value
        self._enabled = self.get_parameter('enabled').value

        # ω_s = 2π · v / λ  — temporal frequency of the sine oscillation
        self._omega_s = 2.0 * math.pi * self._v / self._lambda

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.add_on_set_parameters_callback(self._on_parameter_event)

        self._start_time = self.get_clock().now()
        self._timer = self.create_timer(1.0 / self._rate, self._control_loop)

        self.get_logger().info(
            f'SineWaveBaseNode ready. '
            f'v={self._v} m/s, A={self._A} m, λ={self._lambda} m, '
            f'ω_s={self._omega_s:.3f} rad/s, enabled={self._enabled}'
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_parameter_event(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'enabled':
                self._enabled = p.value
                if p.value:
                    self._start_time = self.get_clock().now()
                    self.get_logger().info('SineWaveBaseNode ENABLED — driving sine wave.')
                else:
                    self.get_logger().info('SineWaveBaseNode DISABLED — publishing zero cmd_vel.')
                    self._publish_stop()
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if not self._enabled:
            return

        t = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

        # Lateral velocity and its time derivative along the ideal sine path
        vy = self._A * self._omega_s * math.cos(self._omega_s * t)
        dvy_dt = -self._A * self._omega_s ** 2 * math.sin(self._omega_s * t)

        # Exact heading-rate feedforward: dθ/dt = v · dvy/dt / (v² + vy²)
        denom = self._v ** 2 + vy ** 2
        angular_z = self._v * dvy_dt / denom if denom > 1e-9 else 0.0

        msg = Twist()
        msg.linear.x = self._v
        msg.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish_stop(self) -> None:
        self._cmd_vel_pub.publish(Twist())

    def _declare_parameters(self) -> None:
        self.declare_parameter('linear_vel', 0.2)
        self.declare_parameter('amplitude', 0.4)
        self.declare_parameter('wavelength', 4.0)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('enabled', False)


def main() -> None:
    rclpy.init()
    node = SineWaveBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
