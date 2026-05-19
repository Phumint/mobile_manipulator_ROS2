import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """
    Launch the holistic whole-body controller and its MoveIt Servo backend.

    Prerequisites (start in this order in separate terminals):
      1. ros2 launch moma_bringup moma_system.launch.py      use_sim:=<true|false>
      2. ros2 launch moma_bringup moma_nav_moveit.launch.py  use_sim:=<true|false>  map:=<path>
      3. ros2 launch moma_paper_demo demo.launch.py          use_sim:=<true|false>

    Step 3 (this file) starts two nodes:
      - servo_node   : MoveIt Servo converts JointJog → JointTrajectory → ur_manipulator_controller
      - controller_node : QP whole-body controller, publishes to /cmd_vel and /servo_node/delta_joint_cmds
    """
    use_sim = LaunchConfiguration('use_sim')

    # MoveIt config needed by servo for kinematics and collision checking.
    moveit_config = (
        MoveItConfigsBuilder("moma_robot", package_name="moma_moveit_config")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Humble's ServoParameters::makeServoParameters() declares every servo param
    # under the "moveit_servo" sub-namespace (e.g. moveit_servo.move_group_name).
    # Load the flat YAML and wrap it here so parameters land at the right level.
    _servo_yaml_path = os.path.join(
        get_package_share_directory('moma_moveit_config'),
        'config', 'servo_params.yaml',
    )
    with open(_servo_yaml_path) as _f:
        servo_params = {'moveit_servo': yaml.safe_load(_f)}

    demo_params = PathJoinSubstitution(
        [FindPackageShare('moma_paper_demo'), 'config', 'demo_params.yaml']
    )

    common_servo_kwargs = dict(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
    )

    common_ctrl_kwargs = dict(
        package='moma_paper_demo',
        executable='controller_node',
        name='controller_node',
        output='screen',
    )

    # Servo starts paused in Humble — call start_servo after a short delay.
    start_servo = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/servo_node/start_servo',
                     'std_srvs/srv/Empty', '{}'],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='true → Gazebo simulation (use_sim_time=true), false → real hardware',
        ),

        # MoveIt Servo — simulation
        Node(
            **common_servo_kwargs,
            parameters=[moveit_config.to_dict(), servo_params, {'use_sim_time': True}],
            condition=IfCondition(use_sim),
        ),
        # MoveIt Servo — real hardware
        Node(
            **common_servo_kwargs,
            parameters=[moveit_config.to_dict(), servo_params, {'use_sim_time': False}],
            condition=UnlessCondition(use_sim),
        ),

        start_servo,

        # Whole-body controller — simulation
        Node(
            **common_ctrl_kwargs,
            parameters=[demo_params, {'use_sim_time': True}],
            condition=IfCondition(use_sim),
        ),
        # Whole-body controller — real hardware
        Node(
            **common_ctrl_kwargs,
            parameters=[demo_params, {'use_sim_time': False}],
            condition=UnlessCondition(use_sim),
        ),
    ])
