import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
import yaml

sys.path.insert(0, os.path.dirname(__file__))

from table_scene_utils import (
    get_scene_object_collision_parameter_list,
    load_table_scene_config,
)


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r', encoding='utf-8') as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    arm_id = LaunchConfiguration('arm_id')
    hand = LaunchConfiguration('hand')
    load_table = LaunchConfiguration('load_table')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_delay = LaunchConfiguration('rviz_delay')
    table_scene_config = load_table_scene_config()
    scene_object_parameters = get_scene_object_collision_parameter_list(table_scene_config)

    robot_xacro_file = os.path.join(
        get_package_share_directory('panda_gazebo_bringup'),
        'urdf',
        'panda_gazebo.urdf.xacro',
    )
    robot_description_config = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            robot_xacro_file,
            ' arm_id:=',
            arm_id,
            ' hand:=',
            hand,
        ]
    )
    robot_description = {
        'robot_description': robot_description_config,
        'use_sim_time': True,
    }

    semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_moveit_config'),
        'srdf',
        'panda_arm.srdf.xacro',
    )
    robot_description_semantic_config = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            semantic_xacro_file,
            ' arm_id:=',
            arm_id,
            ' hand:=',
            hand,
        ]
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config,
    }

    kinematics_yaml = load_yaml('franka_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml,
    }

    # RViz's local RobotInteraction IK lookup checks per-group parameters such
    # as `panda_manipulator.kinematics_solver_timeout` before it looks inside
    # `robot_description_kinematics.*`. Keep exposing the minimal nested
    # structure to avoid the Humble MotionPlanning reload conflict, but also
    # provide the full per-group parameters so the interactive goal marker can
    # solve pose IK locally inside RViz.
    rviz_group_kinematics = kinematics_yaml

    rviz_kinematics_yaml = {}
    for group_name, group_config in kinematics_yaml.items():
        rviz_group_config = {}
        if 'kinematics_solver' in group_config:
            rviz_group_config['kinematics_solver'] = group_config['kinematics_solver']
        if 'kinematics_solver_ik_links' in group_config:
            rviz_group_config['kinematics_solver_ik_links'] = group_config[
                'kinematics_solver_ik_links'
            ]
        if rviz_group_config:
            rviz_kinematics_yaml[group_name] = rviz_group_config
    rviz_robot_description_kinematics = {
        'robot_description_kinematics': rviz_kinematics_yaml,
    }

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
            'default_planner_request_adapters/ResolveConstraintFrames '
            'default_planner_request_adapters/FixWorkspaceBounds '
            'default_planner_request_adapters/FixStartStateBounds '
            'default_planner_request_adapters/FixStartStateCollision '
            'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('franka_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        'panda_gazebo_bringup',
        'config/moveit_controllers.yaml',
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'use_sim_time': True,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'use_sim_time': True,
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    scene_collision_object_nodes = [
        Node(
            package='panda_gazebo_bringup',
            executable='apply_table_collision_object.py',
            name=f'{scene_object["object_id"]}_collision_object_publisher',
            output='screen',
            condition=IfCondition(load_table),
            parameters=[scene_object],
        )
        for scene_object in scene_object_parameters
    ]

    rviz_config = os.path.join(
        get_package_share_directory('panda_gazebo_bringup'),
        'rviz',
        'moveit.rviz',
    )
    rviz_node = TimerAction(
        period=rviz_delay,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                namespace='/',
                output='log',
                arguments=['-d', rviz_config],
                condition=IfCondition(use_rviz),
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    ompl_planning_pipeline_config,
                    rviz_robot_description_kinematics,
                    rviz_group_kinematics,
                    {'use_sim_time': True},
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    'Launching MoveIt + RViz only. '
                    'Use this with panda_gazebo_bringup/gazebo.launch.py, not with gazebo_moveit.launch.py.'
                )
            ),
            DeclareLaunchArgument(
                'arm_id',
                default_value='panda',
                description='Arm ID used in the Panda URDF and MoveIt configuration.',
            ),
            DeclareLaunchArgument(
                'hand',
                default_value='true',
                description='Load the Franka hand geometry in the MoveIt robot model.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Launch RViz with the Panda MoveIt configuration.',
            ),
            DeclareLaunchArgument(
                'load_table',
                default_value='true',
                description='Add the configured scene objects to the MoveIt planning scene.',
            ),
            DeclareLaunchArgument(
                'rviz_delay',
                default_value='6.0',
                description='Delay RViz startup so MoveIt and the simulated arm settle before the MotionPlanning display initializes.',
            ),
            move_group,
            *scene_collision_object_nodes,
            rviz_node,
        ]
    )
