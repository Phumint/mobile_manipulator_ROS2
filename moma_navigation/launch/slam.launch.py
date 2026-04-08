import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    moma_nav_dir = FindPackageShare('moma_navigation')
    slam_toolbox_dir = FindPackageShare('slam_toolbox')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Include the official SLAM Toolbox launch file
    start_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_toolbox_dir, 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=PathJoinSubstitution([moma_nav_dir, 'config', 'slam_toolbox.yaml']),
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
        ),
        start_slam_toolbox
    ])