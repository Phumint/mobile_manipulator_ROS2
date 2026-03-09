import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # --- Paths ---
    moma_nav_dir = FindPackageShare('moma_navigation')
    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    # --- Launch Configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')

    # --- The "Agnostic" Magic: RewrittenYaml ---
    # This automatically intercepts the YAML file and overwrites 'use_sim_time' 
    # and 'autostart' so you don't have to maintain separate YAMLs for Sim vs Real Hardware.
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # --- Includes ---
    # We call the official, heavily tested Nav2 bringup, but feed it our custom MOMA variables.
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': configured_params
        }.items()
    )

    return LaunchDescription([
        # Declare arguments so they can be overridden via CLI
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([moma_nav_dir, 'config', 'moma_nav2.yaml']), description='Full path to the ROS2 parameters file to use'),
        # Require a map to be passed in
        DeclareLaunchArgument('map', default_value='', description='Full path to map yaml file to load'),
        
        bringup_cmd
    ])