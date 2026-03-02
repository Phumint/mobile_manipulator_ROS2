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

    # --- DYNAMIC RESOURCE PATH SETUP ---
    # get_package_share_directory returns: /.../install/mir_description/share/mir_description
    # We need the parent directory so Gazebo can resolve "model://mir_description/..."
    mir_pkg_share = get_package_share_directory('mir_description')
    mir_models_path = os.path.dirname(mir_pkg_share)

    set_ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=mir_models_path
    )
    
    # Adding GZ_SIM_RESOURCE_PATH for future-proofing in case you upgrade to Gazebo Harmonic/Garden
    set_gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=mir_models_path
    )
    # -----------------------------------

    # Start Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
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

    # controller_manager = Node(
    #     package = 'controller_manager',
    #     executable = 'spawner',
    #     arguments=['joint_state_broadcaster',
    #            '--controller-manager', '/controller_manager'],
    #     remappings=[
    #     ('/joint_states', '/ur/joint_states'),  # ← key remap
    #     ],

    #     output='screen',
    # )

    # delayed_controller = TimerAction(
    #     period=5.0,
    #     actions=[controller_manager]
    # )

    return LaunchDescription([
        DeclareLaunchArgument('bridge_config', default_value=PathJoinSubstitution([moma_gz_dir, 'config', 'ros_gz_bridge.yaml'])),
        set_ign_resource_path,
        set_gz_resource_path,
        gz_sim,
        gz_spawn_entity,
        gz_bridge,
        # delayed_controller
    ])