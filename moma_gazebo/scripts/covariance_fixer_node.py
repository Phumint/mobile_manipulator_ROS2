#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math

class CovarianceFixer(Node):
    def __init__(self):
        super().__init__('covariance_fixer_node')

        # --- IMU ---
        # Values derived from your URDF stddev parameters
        # Covariance = stddev^2 (variance)
        self.imu_angular_vel_cov  = [8e-06, 0.0,   0.0,
                                     0.0,   8e-06, 0.0,
                                     0.0,   0.0,   3e-07]  # matches your z-axis stddev^2

        self.imu_linear_accel_cov = [5e-05,  0.0,    0.0,
                                     0.0,    1e-04,  0.0,
                                     0.0,    0.0,    1.3e-04]  # matches your URDF

        # Orientation: gz-sim doesn't provide reliable covariance,
        # set -1 to signal "no orientation data" to EKF
        self.imu_orientation_cov  = [-1.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0]

        # --- Odometry ---
        # Typical values for a differential drive in simulation
        self.odom_pose_cov   = [1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
                                0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
                                0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,
                                0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,
                                0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,
                                0.0,  0.0,  0.0,  0.0,  0.0,  1e-3]

        self.odom_twist_cov  = [1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
                                0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
                                0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,
                                0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,
                                0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,
                                0.0,  0.0,  0.0,  0.0,  0.0,  1e-3]

        self.imu_sub  = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.imu_pub  = self.create_publisher(Imu, '/imu/fixed', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom/fixed', 10)

        self.get_logger().info("Covariance Fixer Node started.")

    def imu_cb(self, msg: Imu):
        msg.orientation_covariance       = self.imu_orientation_cov
        msg.angular_velocity_covariance  = self.imu_angular_vel_cov
        msg.linear_acceleration_covariance = self.imu_linear_accel_cov
        self.imu_pub.publish(msg)

    def odom_cb(self, msg: Odometry):
        msg.pose.covariance  = self.odom_pose_cov
        msg.twist.covariance = self.odom_twist_cov
        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(CovarianceFixer())
    rclpy.shutdown()

if __name__ == '__main__':
    main()