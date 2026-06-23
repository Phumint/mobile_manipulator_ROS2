"""
Launch the holistic data-exchange demo.

This launch file is Step 3 of the demo startup sequence:
  1. ros2 launch moma_bringup moma_system.launch.py      use_sim:=<true|false>
  2. ros2 launch moma_bringup moma_nav_moveit.launch.py  use_sim:=<true|false>  map:=<path>
  3. ros2 launch moma_paper_demo holistic_demo.launch.py use_sim:=<true|false>

Nodes started by this file:
  - sine_wave_base_node: open-loop feedforward cmd_vel for an optional, repeatable
                         scripted sine-wave path — NOT used by the default demo.
  - lock_on_arm_node   : Jacobian arm controller, publishes JointTrajectory directly
                         to /joint_trajectory_controller/joint_trajectory (no Servo needed)

Default demo: enable lock_on_arm_node only, then drive the MiR by hand with a
teleop tool (e.g. teleop_twist_keyboard) publishing directly to /cmd_vel. The EE
stays locked on its fixed odom-frame pose while you drive the base around freely.
  ros2 param set /lock_on_arm_node enabled true

Optional scripted-path testing: also enable sine_wave_base_node instead of (or
in addition to) manual teleop, for a repeatable open-loop sine trajectory.
Both nodes start with enabled:=false (safe default — does not conflict with Nav2).
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
