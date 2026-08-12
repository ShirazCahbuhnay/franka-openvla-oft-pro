import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, os.path.dirname(__file__))

from table_scene_utils import generate_table_world_file, load_table_scene_config


def generate_launch_description():
    arm_id = LaunchConfiguration('arm_id')
    hand = LaunchConfiguration('hand')
    gz_args = LaunchConfiguration('gz_args')
    initialize_home_pose = LaunchConfiguration('initialize_home_pose')
    bridge_scene_camera = LaunchConfiguration('bridge_scene_camera')
    use_cbf_controller = LaunchConfiguration('use_cbf_controller')
    load_table = LaunchConfiguration('load_table')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_delay = LaunchConfiguration('rviz_delay')
    gui_config_file = os.path.join(
        get_package_share_directory('panda_gazebo_bringup'),
        'config',
        'gazebo_gui.config',
    )
    table_scene_config = load_table_scene_config()
    table_world_file = generate_table_world_file(table_scene_config)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('panda_gazebo_bringup'), 'launch', 'gazebo.launch.py']
                )
            ]
        ),
        launch_arguments={
            'arm_id': arm_id,
            'hand': hand,
            'gz_args': gz_args,
            'initialize_home_pose': initialize_home_pose,
            'bridge_scene_camera': bridge_scene_camera,
            'use_cbf_controller': use_cbf_controller,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('panda_gazebo_bringup'), 'launch', 'moveit.launch.py']
                )
            ]
        ),
        launch_arguments={
            'arm_id': arm_id,
            'hand': hand,
            'load_table': load_table,
            'use_rviz': use_rviz,
            'rviz_delay': rviz_delay,
        }.items(),
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    'Launching Gazebo + MoveIt + RViz together. '
                    'Do not run panda_gazebo_bringup/moveit.launch.py in parallel with this launch.'
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
                description='Load the Franka hand geometry and finger joints.',
            ),
            DeclareLaunchArgument(
                'gz_args',
                default_value=f'-r {table_world_file} --gui-config {gui_config_file}',
                description='Extra arguments forwarded to Gazebo Sim.',
            ),
            DeclareLaunchArgument(
                'initialize_home_pose',
                default_value='false',
                description='Optionally send a one-shot home trajectory after the arm controller becomes active.',
            ),
            DeclareLaunchArgument(
                'bridge_scene_camera',
                default_value='true',
                description='Bridge the fixed scene camera image and camera_info topics into ROS when ros_gz_bridge is installed.',
            ),
            DeclareLaunchArgument(
                'use_cbf_controller',
                default_value='false',
                description='Use cbf_joint_trajectory_controller for panda_arm_controller.',
            ),
            DeclareLaunchArgument(
                'load_table',
                default_value='true',
                description='Add the configured scene objects to the MoveIt planning scene.',
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Launch RViz with the MoveIt configuration.',
            ),
            DeclareLaunchArgument(
                'rviz_delay',
                default_value='6.0',
                description='Delay RViz startup so the simulated arm finishes its startup motion before MotionPlanning initializes.',
            ),
            gazebo_launch,
            moveit_launch,
        ]
    )
