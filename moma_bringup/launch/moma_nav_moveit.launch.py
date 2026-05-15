import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Arguments & Configurations
    use_sim = LaunchConfiguration('use_sim')

    # Build MoveIt Config (shared by MoveGroup and RViz)
    moveit_config = MoveItConfigsBuilder("moma_robot", package_name="moma_moveit_config").to_moveit_configs()
    moveit_params = moveit_config.to_dict()
    moveit_params.update({'use_sim_time': use_sim})

    # 2. Path to Navigation Launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('moma_navigation'), 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim,
            'use_rviz': 'false'
        }.items()
    )

    # 3. MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params]
    )

    # 4. Combined RViz Node
    # Uses your custom moveit.rviz config but allows Nav2 plugins to be added
    rviz_config_file = os.path.join(
        get_package_share_directory("moma_bringup"),
        "rviz",
        "nav_moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true', description='Use simulation time if true'),
        nav2_launch,
        move_group_node,
        rviz_node,
    ])
