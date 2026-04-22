import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Build the config
    moveit_config = MoveItConfigsBuilder("moma_robot", package_name="moma_moveit_config").to_moveit_configs()

    # Manually locate the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory("moma_moveit_config"),
        "config",
        "moveit.rviz"
    )

    # Extract ALL parameters to a dictionary and force use_sim_time
    rviz_parameters = moveit_config.to_dict()
    rviz_parameters.update({'use_sim_time': True})

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[rviz_parameters],
    )

    return LaunchDescription([rviz_node])
