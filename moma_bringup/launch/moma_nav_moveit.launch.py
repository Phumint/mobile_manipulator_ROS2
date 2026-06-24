import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

# Snap's core20 base injects an old Ubuntu 20.04 libpthread.so.0 into the
# dynamic linker path. Ubuntu 22.04 glibc removed __libc_pthread_init (merged
# into libc.so.6), so rviz2 crashes on startup with a symbol lookup error.
# Stripping /snap paths from LD_LIBRARY_PATH for the rviz2 process fixes this.
_rviz_ld_path = ':'.join(
    p for p in os.environ.get('LD_LIBRARY_PATH', '').split(':')
    if p and '/snap/' not in p
)

def generate_launch_description():
    # 1. Arguments & Configurations
    use_sim = LaunchConfiguration('use_sim')
    map_yaml_file = LaunchConfiguration('map')

    # Build MoveIt Config (shared by MoveGroup and RViz).
    # Both OMPL (collision-aware, probabilistic) and Pilz (analytical IK,
    # minimal joint travel) are loaded so RViz can switch between them.
    moveit_config = (
        MoveItConfigsBuilder("moma_robot", package_name="moma_moveit_config")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    moveit_params.update({
        'use_sim_time': use_sim,
        'current_state_monitor_wait_time': 2.0,
        'trajectory_execution.allowed_start_tolerance': 0.05,
    })

    # 2. Path to Navigation Launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('moma_navigation'), 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim,
            'use_rviz': 'false',
            'map': map_yaml_file,
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
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_params],
        # VS Code snap mounts core20 libs into the sandbox. Snap's libpthread.so.0
        # (glibc 2.31) references __libc_pthread_init which was removed in glibc 2.34.
        # LD_PRELOAD forces the system stub to bind first, blocking the snap version.
        additional_env={
            'LD_LIBRARY_PATH': _rviz_ld_path,
            'LD_PRELOAD': '/lib/x86_64-linux-gnu/libpthread.so.0',
        },
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true', description='Use simulation time if true'),
        DeclareLaunchArgument('map', default_value='', description='Full path to map yaml file to load'),
        nav2_launch,
        move_group_node,
        rviz_node,
    ])
