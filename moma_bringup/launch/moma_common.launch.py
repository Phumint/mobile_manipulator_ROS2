import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim = LaunchConfiguration('use_sim')

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
        ]
    )

    # In the future, other common nodes go here. For example:
    # twist_mux (used in both sim and real) would NOT have an IfCondition.

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        DeclareLaunchArgument('use_sim', default_value='true', description='Is this a simulation?'),
        laser_merger_node
    ])