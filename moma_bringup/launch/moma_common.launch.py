import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    use_sim = LaunchConfiguration('use_sim')

    # Real Hardware IP Arguments
    mir_ip = LaunchConfiguration('mir_ip')
    ur_robot_ip = LaunchConfiguration('ur_robot_ip')

    # =========================================================
    # SIMULATION ONLY NODES (Conditioned on IfCondition)
    # =========================================================

    # Start the Dual Laser Merger (Leveraging mir_ws!)
    # CONDITION: Only run this if we are in simulation. 
    # The real MiR hardware provides a unified /scan natively.
    laser_merger_node = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        name='dual_laser_merger_node',
        output='screen',
        condition=IfCondition(use_sim),
        parameters=[
            PathJoinSubstitution([FindPackageShare('mir_gazebo'), 'config', 'laser_merger_params.yaml']),
            {'use_sim_time': use_sim}
        ],
        remappings=[
        ('/merged', '/scan')
        ]   
    )

    # EKF intentionally removed for sim parity with real hardware. In sim, the
    # OdometryPublisher gazebo plugin publishes ground-truth-physics /odom on
    # behalf of the gazebo "base" (analogous to MiR's onboard pre-fused odom),
    # and odom_tf_publisher emits the matching odom -> base_footprint TF.

    # =========================================================
    # REAL HARDWARE DRIVERS (Conditioned on UnlessCondition)
    # =========================================================

    # 1. MiR Real Hardware Driver
    # MiR Real Hardware Driver
    mir_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('mir_driver_bridge'), 'launch', 'mir.launch.py'])
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            'mir_ip': mir_ip,
            'start_rsp': 'false',  # <--- This kills the rogue publisher!
        }.items()
    )

    # 2. UR Real Hardware Driver
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            'robot_ip': ur_robot_ip,
            'ur_type': 'ur10e',
            'tf_prefix': 'ur_',
            'use_fake_hardware': 'false',
            # Use the same controller name as simulation so MoveIt config is identical
            'initial_joint_controller': 'ur_manipulator_controller',
            # Pass your unified URDF to the UR driver so it knows about the MiR base
            'description_package': 'moma_description',
            'description_file': 'moma.urdf.xacro',
            'kinematics_config': PathJoinSubstitution([
            FindPackageShare('moma_description'), 'config', 'ur10e_calibration.yaml'
        ]),
        }.items()
    )

    # Controller Spawners (sim only — real hardware UR driver activates these itself)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        condition=IfCondition(use_sim)
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur_manipulator_controller',
                   '--controller-manager', '/controller_manager'],
        condition=IfCondition(use_sim)
    )

    # =========================================================
    # DELAY ACTIONS (Using TimerAction to stagger startup)
    # =========================================================

    # Delay sim nodes
    delay_laser_merger = TimerAction(period=2.0, actions=[laser_merger_node])

    # Controllers need /controller_manager up (hosted inside Gazebo via ign_ros2_control).
    # moma_common itself is already delayed 5 s from moma_system, so 3 s here = 8 s total.
    delay_controllers = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner, arm_controller_spawner]
    )

    # Delay hardware drivers
    delay_mir_driver = TimerAction(period=6.0, actions=[mir_driver_launch])
    delay_ur_driver = TimerAction(period=12.0, actions=[ur_driver_launch])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        DeclareLaunchArgument('use_sim', default_value='false', description='Is this a simulation?'),
        DeclareLaunchArgument('mir_ip', default_value='192.168.12.20', description='IP of the real MiR'),
        DeclareLaunchArgument('ur_robot_ip', default_value='192.168.12.120', description='IP of the real UR arm'),

        delay_laser_merger,
        delay_controllers,
        delay_mir_driver,
        delay_ur_driver
    ])