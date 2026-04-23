#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseWithCovarianceStamped
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
import math
import time

class MomaIntegrationDemo(Node):
    """
    A demonstration node that coordinates Nav2 and MoveIt2.
    It first moves the MiR base, then moves the UR arm.
    """
    def __init__(self):
        # Initialize with use_sim_time=True as specified in your setup
        super().__init__('moma_integration_demo', 
                         parameter_overrides=[
                             rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
                         ])
        
        # 1. Initialize Action Clients
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._move_client = ActionClient(self, MoveGroup, 'move_action')
        
        # 2. Initial Pose Publisher (to trigger Nav2/AMCL TF)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            'initialpose', 
            10)
        
        self.get_logger().info('MOMA Integration Demo Node initialized and waiting for servers...')

    def set_initial_pose(self, x=0.0, y=0.0, yaw=0.0):
        """Publishes an initial pose to AMCL to start the map->odom TF."""
        self.get_logger().info('Waiting for AMCL to subscribe to /initialpose...')
        
        # Wait until there is at least one subscriber to the initialpose topic
        while self._initial_pose_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        self.get_logger().info('AMCL detected. Sending initial pose...')
        
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Small covariance to tell AMCL we are certain
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[35] = 0.05
        self._initial_pose_pub.publish(msg)
        self.get_logger().info('Initial pose sent.')

    def send_nav_goal(self, x, y, yaw):
        """Sends a navigation goal to Nav2."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 Action Server not available!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        
        # Convert yaw to quaternion (Z-axis rotation)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Sending Nav2 Goal: x={x}, y={y}...')
        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 Goal Rejected!')
            return False

        self.get_logger().info('Navigation in progress...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Navigation Succeeded.')
        return True

    def send_moveit_pose_goal(self, x, y, z):
        """Sends a Cartesian pose goal to MoveIt2 MoveGroup."""
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup Action Server not available!')
            return False

        goal_msg = MoveGroup.Goal()
        
        # Configure Motion Plan Request
        request = MotionPlanRequest()
        request.group_name = 'ur_manipulator' # Matches the controller in your launch context
        request.num_planning_attempts = 20
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        
        # 1. Position Constraint: Define a small region around the target [x, y, z]
        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'ur_tool0'
        
        # Define the target region as a small bounding box (1cm)
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.05, 0.05, 0.05]
        
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        target_pose.orientation.w = 1.0 # Default orientation (looking forward/up)
        
        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0

        # 2. Orientation Constraint: Define tolerances for rotation
        oc = OrientationConstraint()
        oc.header.frame_id = 'base_link'
        oc.link_name = 'ur_tool0'
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.5
        oc.absolute_y_axis_tolerance = 0.5
        oc.absolute_z_axis_tolerance = 0.5
        oc.weight = 1.0
        
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(pc)
        goal_constraints.orientation_constraints.append(oc)
        request.goal_constraints.append(goal_constraints)
        
        goal_msg.request = request
        
        self.get_logger().info(f'Sending MoveIt2 Pose Goal to link ur_tool0: x={x}, y={y}, z={z}...')
        send_goal_future = self._move_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('MoveIt2 Goal Rejected!')
            return False

        self.get_logger().info('Planning and Moving Arm...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('MoveIt2 Motion Finished.')
        return True

if __name__ == '__main__':
    rclpy.init()
    node = MomaIntegrationDemo()

    try:
        # Step 0: Tell Nav2 where we are (triggers the 'map' frame)
        node.set_initial_pose(0.0, 0.0, 0.0)
        
        if node.send_nav_goal(0.5, 0.0, 0.0):
            node.send_moveit_pose_goal(0.6, 0.0, 1.3)
    except Exception as e:
        node.get_logger().error(f'Demo failed: {str(e)}')

    node.destroy_node()
    rclpy.shutdown()