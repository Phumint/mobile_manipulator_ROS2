import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
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
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
        ('/merged', '/scan')
        ]   
    )

    # Start the EKF Localization Node
    # CONDITION: Only run this in simulation. The real MiR provides pre-fused /odom.
    ekf_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('moma_navigation'), 'launch', 'localization.launch.py'])
        ),
        condition=IfCondition(use_sim),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # =========================================================
    # REAL HARDWARE DRIVERS (Conditioned on UnlessCondition)
    # =========================================================

    # 1. MiR Real Hardware Driver
    mir_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('mir_driver_bridge'), 'launch', 'mir.launch.py'])
        ),
        condition=UnlessCondition(use_sim),
        launch_arguments={
            'mir_ip': mir_ip,
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
            # Pass your unified URDF to the UR driver so it knows about the MiR base
            'description_package': 'moma_description',
            'description_file': 'moma.urdf.xacro',
            'kinematics_config': PathJoinSubstitution([
            FindPackageShare('moma_description'), 'config', 'ur10e_calibration.yaml'
        ]),
        }.items()
    )

    # twist_mux (used in both sim and real) would NOT have an IfCondition.

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        DeclareLaunchArgument('use_sim', default_value='true', description='Is this a simulation?'),
        DeclareLaunchArgument('mir_ip', default_value='192.168.12.20', description='IP of the real MiR'),
        DeclareLaunchArgument('ur_robot_ip', default_value='192.168.12.120', description='IP of the real UR arm'),
        laser_merger_node,
        ekf_localization_launch,
        mir_driver_launch,
        ur_driver_launch
    ])