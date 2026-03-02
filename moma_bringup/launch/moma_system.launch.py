from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Global Arguments
    use_sim = LaunchConfiguration('use_sim')

    # 1. Start Robot State Publisher (from moma_description)
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('moma_description'), 'launch', 'rsp.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim}.items()
    )

    # 2. Start Gazebo Environment (from moma_gazebo)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('moma_gazebo'), 'launch', 'moma_sim.launch.py'])
        ),
        condition=IfCondition(use_sim) # Only launches if we are in simulation mode
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true', description='Launch Gazebo simulation'),
        rsp_launch,
        gazebo_launch
    ])