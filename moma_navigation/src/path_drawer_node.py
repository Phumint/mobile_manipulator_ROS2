#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import json
import math

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowPath

class PathDrawerNode(Node):
    def __init__(self):
        super().__init__('path_drawer_node')
        
        # Subscriber to listen to the web UI
        self.subscription = self.create_subscription(
            String,
            '/web_drawn_path_raw',
            self.path_callback,
            10)
            
        # Action Client to talk to Nav2's Controller
        self.action_client = ActionClient(self, FollowPath, '/follow_path')
        
        # Optional: Publisher to visualize the generated path in RViz
        self.rviz_path_pub = self.create_publisher(Path, '/web_drawn_path_rviz', 10)
        
        self.get_logger().info("Path Drawer Node started. Waiting for paths from Web UI...")

    def get_quaternion_from_yaw(self, yaw):
        """Convert a yaw angle (in radians) to a geometry_msgs Quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def path_callback(self, msg):
        self.get_logger().info("Received new path from Web UI! Processing...")
        
        try:
            points = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse JSON path data.")
            return

        if len(points) < 2:
            self.get_logger().warning("Path needs at least 2 points. Ignoring.")
            return

        nav_path = Path()
        nav_path.header.frame_id = 'map'
        nav_path.header.stamp = self.get_clock().now().to_msg()

        # Iterate through points to build Poses
        for i in range(len(points)):
            pose = PoseStamped()
            pose.header = nav_path.header
            pose.pose.position.x = float(points[i]['x'])
            pose.pose.position.y = float(points[i]['y'])
            pose.pose.position.z = 0.0

            # Calculate orientation (yaw) pointing to the next coordinate
            if i < len(points) - 1:
                dx = points[i+1]['x'] - points[i]['x']
                dy = points[i+1]['y'] - points[i]['y']
                yaw = math.atan2(dy, dx)
            else:
                # For the last point, keep the same yaw as the previous point
                dx = points[i]['x'] - points[i-1]['x']
                dy = points[i]['y'] - points[i-1]['y']
                yaw = math.atan2(dy, dx)

            pose.pose.orientation = self.get_quaternion_from_yaw(yaw)
            nav_path.poses.append(pose)

        # Publish for RViz visualization
        self.rviz_path_pub.publish(nav_path)
        
        # Send to Nav2 Action Server
        self.send_path_to_nav2(nav_path)

    def send_path_to_nav2(self, nav_path):
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Nav2 FollowPath Action Server not available!")
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = nav_path
        # You can specify a specific controller here if you have multiple configured in Nav2
        # goal_msg.controller_id = 'FollowPath' 

        self.get_logger().info(f"Sending path with {len(nav_path.poses)} poses to Nav2...")
        self.action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathDrawerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()