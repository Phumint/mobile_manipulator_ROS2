"""
Launch the holistic data-exchange demo.

This launch file is Step 3 of the demo startup sequence:
  1. ros2 launch moma_bringup moma_system.launch.py      use_sim:=<true|false>
  2. ros2 launch moma_bringup moma_nav_moveit.launch.py  use_sim:=<true|false>  map:=<path>
  3. ros2 launch moma_paper_demo holistic_demo.launch.py use_sim:=<true|false>

Nodes started by this file:
  - sine_wave_base_node: open-loop feedforward cmd_vel for the MiR sine wave path
  - lock_on_arm_node   : Jacobian arm controller, publishes JointTrajectory directly
                         to /ur_manipulator_controller/joint_trajectory (no Servo needed)

Both nodes start with enabled:=false (safe default — does not conflict with Nav2).
Enable them when ready:
  ros2 param set /lock_on_arm_node    enabled true
  ros2 param set /sine_wave_base_node enabled true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')

    demo_params = PathJoinSubstitution(
        [FindPackageShare('moma_paper_demo'), 'config', 'holistic_demo_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='true → Gazebo simulation (use_sim_time=true), false → real hardware',
        ),

        # Sine wave base driver — simulation
        Node(
            package='moma_paper_demo',
            executable='sine_wave_base_node',
            name='sine_wave_base_node',
            output='screen',
            parameters=[demo_params, {'use_sim_time': True}],
            condition=IfCondition(use_sim),
        ),
        # Sine wave base driver — real hardware
        Node(
            package='moma_paper_demo',
            executable='sine_wave_base_node',
            name='sine_wave_base_node',
            output='screen',
            parameters=[demo_params, {'use_sim_time': False}],
            condition=UnlessCondition(use_sim),
        ),

        # Arm lock-on tracker — simulation
        Node(
            package='moma_paper_demo',
            executable='lock_on_arm_node',
            name='lock_on_arm_node',
            output='screen',
            parameters=[demo_params, {'use_sim_time': True}],
            condition=IfCondition(use_sim),
        ),
        # Arm lock-on tracker — real hardware
        Node(
            package='moma_paper_demo',
            executable='lock_on_arm_node',
            name='lock_on_arm_node',
            output='screen',
            parameters=[demo_params, {'use_sim_time': False}],
            condition=UnlessCondition(use_sim),
        ),
    ])
