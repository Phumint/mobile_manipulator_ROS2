#!/usr/bin/env python3
"""Republish /odom as the odom -> base_footprint TF.

Mirrors what the MiR driver does on real hardware: the driver publishes
both the /odom topic and the corresponding TF. In sim, the OdometryPublisher
gazebo plugin publishes the topic; this node publishes the matching TF.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.cb, 50)

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomTfPublisher())


if __name__ == '__main__':
    main()
