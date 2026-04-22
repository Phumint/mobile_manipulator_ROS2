from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Build the config
    moveit_config = MoveItConfigsBuilder("moma_robot", package_name="moma_moveit_config").to_moveit_configs()
    
    # Extract parameters to a dictionary and force use_sim_time
    move_group_params = moveit_config.to_dict()
    move_group_params.update({'use_sim_time': True})

    # Manually define the node to guarantee the parameter applies
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params]
    )

    return LaunchDescription([move_group_node])
