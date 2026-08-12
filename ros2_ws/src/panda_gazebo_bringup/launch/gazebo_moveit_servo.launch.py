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
    auto_start_servo = LaunchConfiguration('auto_start_servo')
    launch_keyboard_teleop = LaunchConfiguration('launch_keyboard_teleop')
    launch_controller_teleop = LaunchConfiguration('launch_controller_teleop')
    keyboard_frame_id = LaunchConfiguration('keyboard_frame_id')
    keyboard_speed = LaunchConfiguration('keyboard_speed')
    keyboard_turn = LaunchConfiguration('keyboard_turn')
    controller_config_file = LaunchConfiguration('controller_config_file')
    use_ps5_6dof_teleop = LaunchConfiguration('use_ps5_6dof_teleop')
    controller_6dof_config_file = LaunchConfiguration('controller_6dof_config_file')
    launch_gripper_teleop = LaunchConfiguration('launch_gripper_teleop')
    gripper_teleop_config_file = LaunchConfiguration('gripper_teleop_config_file')
    controller_device_id = LaunchConfiguration('controller_device_id')
    controller_deadzone = LaunchConfiguration('controller_deadzone')
    controller_autorepeat_rate = LaunchConfiguration('controller_autorepeat_rate')
    controller_frame_id = LaunchConfiguration('controller_frame_id')
    gui_config_file = os.path.join(
        get_package_share_directory('panda_gazebo_bringup'),
        'config',
        'gazebo_gui.config',
    )
    table_scene_config = load_table_scene_config()
    table_world_file = generate_table_world_file(table_scene_config)

    gazebo_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('panda_gazebo_bringup'),
                        'launch',
                        'gazebo_moveit.launch.py',
                    ]
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
            'load_table': load_table,
            'use_rviz': use_rviz,
            'rviz_delay': rviz_delay,
        }.items(),
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('panda_gazebo_bringup'),
                        'launch',
                        'servo.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'arm_id': arm_id,
            'hand': hand,
            'auto_start_servo': auto_start_servo,
            'launch_keyboard_teleop': launch_keyboard_teleop,
            'launch_controller_teleop': launch_controller_teleop,
            'keyboard_frame_id': keyboard_frame_id,
            'keyboard_speed': keyboard_speed,
            'keyboard_turn': keyboard_turn,
            'controller_config_file': controller_config_file,
            'use_ps5_6dof_teleop': use_ps5_6dof_teleop,
            'controller_6dof_config_file': controller_6dof_config_file,
            'launch_gripper_teleop': launch_gripper_teleop,
            'gripper_teleop_config_file': gripper_teleop_config_file,
            'controller_device_id': controller_device_id,
            'controller_deadzone': controller_deadzone,
            'controller_autorepeat_rate': controller_autorepeat_rate,
            'controller_frame_id': controller_frame_id,
        }.items(),
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    'Launching Gazebo + MoveIt + MoveIt Servo together. '
                    'This minimal teleop path uses the existing Panda joint trajectory controller.'
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
                description='Launch RViz with the Panda MoveIt configuration.',
            ),
            DeclareLaunchArgument(
                'rviz_delay',
                default_value='6.0',
                description='Delay RViz startup so the simulated arm settles before MotionPlanning initializes.',
            ),
            DeclareLaunchArgument(
                'auto_start_servo',
                default_value='true',
                description='Call /servo_node/start_servo automatically after the node is up.',
            ),
            DeclareLaunchArgument(
                'launch_keyboard_teleop',
                default_value='false',
                description='Open teleop_twist_keyboard in a separate gnome-terminal window.',
            ),
            DeclareLaunchArgument(
                'launch_controller_teleop',
                default_value='false',
                description='Start game_controller_node and teleop_twist_joy for a gamepad.',
            ),
            DeclareLaunchArgument(
                'keyboard_frame_id',
                default_value='panda_link0',
                description='Frame ID stamped onto keyboard Twist commands.',
            ),
            DeclareLaunchArgument(
                'keyboard_speed',
                default_value='0.10',
                description='Default linear speed for teleop_twist_keyboard in m/s.',
            ),
            DeclareLaunchArgument(
                'keyboard_turn',
                default_value='0.40',
                description='Default angular speed for teleop_twist_keyboard in rad/s.',
            ),
            DeclareLaunchArgument(
                'controller_config_file',
                default_value=os.path.join(
                    get_package_share_directory('panda_gazebo_bringup'),
                    'config',
                    'ps5_servo_teleop.config.yaml',
                ),
                description='YAML config for teleop_twist_joy. Defaults to the PS5 preset; switch to xbox_servo_teleop.config.yaml for Xbox-style mapping.',
            ),
            DeclareLaunchArgument(
                'use_ps5_6dof_teleop',
                default_value='false',
                description='Use the PS5-specific 6-DOF mapper instead of teleop_twist_joy. Hold L2 for roll/pitch/yaw rotation mode.',
            ),
            DeclareLaunchArgument(
                'controller_6dof_config_file',
                default_value=os.path.join(
                    get_package_share_directory('panda_gazebo_bringup'),
                    'config',
                    'ps5_6dof_servo_teleop.config.yaml',
                ),
                description='YAML config for the PS5-specific 6-DOF Servo teleop mapper.',
            ),
            DeclareLaunchArgument(
                'launch_gripper_teleop',
                default_value='true',
                description='Start gamepad gripper action teleop when controller teleop is enabled.',
            ),
            DeclareLaunchArgument(
                'gripper_teleop_config_file',
                default_value=os.path.join(
                    get_package_share_directory('franka_moveit_config'),
                    'config',
                    'gamepad_gripper_teleop.config.yaml',
                ),
                description='YAML config for gamepad gripper open, close, homing, and stop buttons.',
            ),
            DeclareLaunchArgument(
                'controller_device_id',
                default_value='0',
                description='Joystick device_id passed to joy_node.',
            ),
            DeclareLaunchArgument(
                'controller_deadzone',
                default_value='0.10',
                description='Deadzone passed to joy_node.',
            ),
            DeclareLaunchArgument(
                'controller_autorepeat_rate',
                default_value='50.0',
                description='Continuous Joy re-publish rate in Hz while holding the enable button.',
            ),
            DeclareLaunchArgument(
                'controller_frame_id',
                default_value='panda_link0',
                description='Frame ID stamped onto controller Twist commands.',
            ),
            gazebo_moveit_launch,
            servo_launch,
        ]
    )
