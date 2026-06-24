import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # --- The "Agnostic" Magic: RewrittenYaml ---
    # This automatically intercepts the YAML file and overwrites 'use_sim_time'
    # and 'autostart' so you don't have to maintain separate YAMLs for Sim vs Real Hardware.
    # Same mechanism resolves the custom behavior trees (no <Spin/> recovery,
    # unsafe for the MiR100's rectangular footprint) to an absolute path.
    bt_xml_dir = PathJoinSubstitution([moma_nav_dir, 'config', 'behavior_trees'])
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'default_nav_to_pose_bt_xml': PathJoinSubstitution(
            [bt_xml_dir, 'navigate_to_pose_w_replanning_and_recovery_no_spin.xml']),
        'default_nav_through_poses_bt_xml': PathJoinSubstitution(
            [bt_xml_dir, 'navigate_through_poses_w_replanning_and_recovery_no_spin.xml']),
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

    # --- RViz2 Node ---
    rviz_config_file = PathJoinSubstitution([moma_nav_dir, 'rviz', 'rviz_config.rviz'])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # Declare arguments so they can be overridden via CLI
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([moma_nav_dir, 'config', 'moma_nav2.yaml']), description='Full path to the ROS2 parameters file to use'),
        # Require a map to be passed in
        DeclareLaunchArgument('map', default_value='', description='Full path to map yaml file to load'),
        
        bringup_cmd,
        rviz_node
    ])