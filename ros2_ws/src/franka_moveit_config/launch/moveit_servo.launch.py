import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    use_rviz = LaunchConfiguration('use_rviz')
    use_cbf_controller = LaunchConfiguration('use_cbf_controller')
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
    arm_home_config_file = LaunchConfiguration('arm_home_config_file')
    controller_device_id = LaunchConfiguration('controller_device_id')
    controller_deadzone = LaunchConfiguration('controller_deadzone')
    controller_autorepeat_rate = LaunchConfiguration('controller_autorepeat_rate')
    controller_frame_id = LaunchConfiguration('controller_frame_id')

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('franka_moveit_config'),
                'launch',
                'moveit.launch.py',
            )
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': fake_sensor_commands,
            'use_rviz': use_rviz,
            'use_cbf_controller': use_cbf_controller,
        }.items(),
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('franka_moveit_config'),
                'launch',
                'servo.launch.py',
            )
        ),
        launch_arguments={
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': fake_sensor_commands,
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
            'arm_home_config_file': arm_home_config_file,
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
                    'Launching the real Panda MoveIt + MoveIt Servo stack together. '
                    'Do not run franka_moveit_config/moveit.launch.py in parallel with this launch.'
                )
            ),
            DeclareLaunchArgument(
                'robot_ip',
                default_value='',
                description='Hostname or IP address of the robot. Leave empty only with fake hardware.',
            ),
            DeclareLaunchArgument(
                'use_fake_hardware',
                default_value='false',
                description='Use fake hardware instead of connecting to the robot.',
            ),
            DeclareLaunchArgument(
                'fake_sensor_commands',
                default_value='false',
                description="Fake sensor commands. Only valid when 'use_fake_hardware' is true.",
            ),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Launch RViz with the Panda MoveIt configuration.',
            ),
            DeclareLaunchArgument(
                'use_cbf_controller',
                default_value='false',
                description='Use cbf_joint_trajectory_controller for panda_arm_controller.',
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
                default_value='0.05',
                description='Default linear speed for teleop_twist_keyboard in m/s.',
            ),
            DeclareLaunchArgument(
                'keyboard_turn',
                default_value='0.20',
                description='Default angular speed for teleop_twist_keyboard in rad/s.',
            ),
            DeclareLaunchArgument(
                'controller_config_file',
                default_value=os.path.join(
                    get_package_share_directory('franka_moveit_config'),
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
                    get_package_share_directory('franka_moveit_config'),
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
                'arm_home_config_file',
                default_value=os.path.join(
                    get_package_share_directory('franka_moveit_config'),
                    'config',
                    'ps5_arm_home.config.yaml',
                ),
                description='YAML config for gamepad arm homing node.',
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
            moveit_launch,
            servo_launch,
        ]
    )
