import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    moma_gz_dir = FindPackageShare('moma_gazebo')
    bridge_config = LaunchConfiguration('bridge_config')

    # Get MiR models path
    mir_pkg_share = get_package_share_directory('mir_description')
    mir_models_path = os.path.dirname(mir_pkg_share)

    # ADDED: Get the MOMA models path (where room_map_demo lives)
    moma_models_path = os.path.join(get_package_share_directory('moma_gazebo'), 'models')

    # ADDED: Combine both paths with a colon ':' separator
    combined_resource_paths = f"{mir_models_path}:{moma_models_path}"

    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=combined_resource_paths
    )

    world_path = PathJoinSubstitution([
        moma_gz_dir,
        'worlds',
        'object_demo.world.sdf'
    ])

    # Start Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )

    # Spawn Robot
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'moma_robot', '-z', '0.1'],
        output='screen'
    )

    # Parameter Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('bridge_config', default_value=PathJoinSubstitution([moma_gz_dir, 'config', 'ros_gz_bridge.yaml'])),
        set_ign_resource_path,
        gz_sim,
        gz_spawn_entity,
        gz_bridge,
    ])