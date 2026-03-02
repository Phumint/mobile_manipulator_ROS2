#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from threading import Lock


class JointStateMerger(Node):

    def __init__(self):
        super().__init__('joint_state_merger')

        self.lock = Lock()

        self.mir_msg = None
        self.ur_msg = None

        self.create_subscription(
            JointState,
            '/mir/joint_states',
            self.mir_callback,
            10
        )

        self.create_subscription(
            JointState,
            '/ur/joint_states',
            self.ur_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.get_logger().info("Joint State Merger Started")

    def mir_callback(self, msg):
        with self.lock:
            self.mir_msg = msg
            self.publish_merged()

    def ur_callback(self, msg):
        with self.lock:
            self.ur_msg = msg
            self.publish_merged()

    def publish_merged(self):
        if self.mir_msg is None and self.ur_msg is None:
            return

        merged = JointState()
        merged.header.stamp = self.get_clock().now().to_msg()

        names = []
        positions = []
        velocities = []
        efforts = []

        if self.mir_msg:
            names += self.mir_msg.name
            positions += self.mir_msg.position
            velocities += self.mir_msg.velocity
            efforts += self.mir_msg.effort

        if self.ur_msg:
            names += self.ur_msg.name
            positions += self.ur_msg.position
            velocities += self.ur_msg.velocity
            efforts += self.ur_msg.effort

        merged.name = names
        merged.position = positions
        merged.velocity = velocities
        merged.effort = efforts

        self.publisher.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()