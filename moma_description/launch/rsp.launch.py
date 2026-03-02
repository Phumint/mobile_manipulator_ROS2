import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Args
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Process Xacro (Passing arguments dynamically)
    moma_desc_path = FindPackageShare('moma_description').find('moma_description')
    urdf_file = os.path.join(moma_desc_path, 'urdf', 'moma.urdf.xacro')
    
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf_file
    ])

    # Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim time if true'),
        robot_state_publisher_node
    ])