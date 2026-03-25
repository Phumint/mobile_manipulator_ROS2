import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

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

    # 3. Common Launch (Drivers, Mergers, EKF)
    common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('moma_bringup'), 'launch', 'moma_common.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim,
            'use_sim': use_sim  # Pass the flag down so common.launch knows what to spin up
        }.items()
    )

    # 4. Controller Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        parameters= [{'use_sim_time': use_sim}]
    )

    # =========================================================
    # DELAY ACTIONS (Staggering the bringup process)
    # =========================================================
    
    # Let RSP start immediately, but give Gazebo 2 seconds to ensure the URDF is ready
    delay_gazebo = TimerAction(period=2.0, actions=[gazebo_launch])
    
    # Give Gazebo/Hardware 5 seconds to boot up before firing the common nodes
    delay_common = TimerAction(period=5.0, actions=[common_launch])
    
    # Controllers require the /controller_manager to be fully active (usually hosted by Gazebo or the hardware driver). 
    # Delaying this by 8 seconds ensures it doesn't crash trying to find a missing service.
    delay_jsb_spawner = TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner])


    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true', description='Launch Gazebo simulation'),
        
        rsp_launch,          # 0.0s delay
        delay_gazebo,        # 2.0s delay
        delay_common,        # 5.0s delay
        delay_jsb_spawner,   # 8.0s delay
    ])