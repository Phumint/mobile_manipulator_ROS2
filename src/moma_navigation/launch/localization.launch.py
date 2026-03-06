import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moma_nav_dir = FindPackageShare('moma_navigation')
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ekf_config_path = PathJoinSubstitution([moma_nav_dir, 'config', 'ekf.yaml'])

    # The hardware-agnostic EKF Node
    start_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': use_sim_time}],
        # Remap the output so it doesn't clash with the raw /odom topic
        remappings=[('odometry/filtered', '/odometry/filtered')]
    )

    return LaunchDescription([
        start_ekf_node
    ])