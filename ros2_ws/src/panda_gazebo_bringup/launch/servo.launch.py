import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r', encoding='utf-8') as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def require_package(package_name):
    try:
        get_package_share_directory(package_name)
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "Required package '%s' is not installed. "
            "Install it first, for example: sudo apt-get install -y ros-humble-%s"
            % (package_name, package_name.replace('_', '-'))
        ) from exc


def generate_launch_description():
    require_package('moveit_servo')
    require_package('joy')
    require_package('teleop_twist_joy')

    arm_id = LaunchConfiguration('arm_id')
    hand = LaunchConfiguration('hand')
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
    legacy_controller_condition = IfCondition(
        PythonExpression(
            [
                "'",
                launch_controller_teleop,
                "'.lower() == 'true' and '",
                use_ps5_6dof_teleop,
                "'.lower() != 'true'",
            ]
        )
    )
    gripper_teleop_condition = IfCondition(
        PythonExpression(
            [
                "'",
                launch_controller_teleop,
                "'.lower() == 'true' and '",
                launch_gripper_teleop,
                "'.lower() == 'true' and '",
                hand,
                "'.lower() == 'true'",
            ]
        )
    )
    gripper_namespace = PythonExpression(["'/' + '", arm_id, "' + '_gripper'"])
    ps5_6dof_controller_condition = IfCondition(
        PythonExpression(
            [
                "'",
                launch_controller_teleop,
                "'.lower() == 'true' and '",
                use_ps5_6dof_teleop,
                "'.lower() == 'true'",
            ]
        )
    )

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

    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(
            'franka_moveit_config',
            'config/kinematics.yaml',
        ),
    }

    servo_yaml = load_yaml('panda_gazebo_bringup', 'config/servo.yaml')
    servo_params = {'moveit_servo': servo_yaml}

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            servo_params,
            {'use_sim_time': True},
        ],
    )

    start_servo = Node(
        package='panda_gazebo_bringup',
        executable='start_servo.py',
        name='start_servo_client',
        output='screen',
        parameters=[
            {
                'service_name': '/servo_node/start_servo',
                'wait_timeout_sec': 30.0,
                'call_timeout_sec': 30.0,
                'use_sim_time': True,
            }
        ],
        condition=IfCondition(auto_start_servo),
    )

    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='gnome-terminal --',
        parameters=[
            {
                'stamped': True,
                'frame_id': keyboard_frame_id,
                'speed': keyboard_speed,
                'turn': keyboard_turn,
            }
        ],
        remappings=[('cmd_vel', '/servo_node/delta_twist_cmds')],
        condition=IfCondition(launch_keyboard_teleop),
    )

    controller_joy = Node(
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
        output='screen',
        parameters=[
            {
                'device_id': controller_device_id,
                'deadzone': controller_deadzone,
                'autorepeat_rate': controller_autorepeat_rate,
                'use_sim_time': True,
            }
        ],
        condition=IfCondition(launch_controller_teleop),
    )

    controller_teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[
            controller_config_file,
            {
                'publish_stamped_twist': True,
                'frame': controller_frame_id,
                'use_sim_time': True,
            },
        ],
        remappings=[('/cmd_vel', '/servo_node/delta_twist_cmds')],
        condition=legacy_controller_condition,
    )

    controller_6dof_teleop = Node(
        package='franka_moveit_config',
        executable='ps5_6dof_servo_teleop.py',
        name='ps5_6dof_servo_teleop_node',
        output='screen',
        parameters=[
            controller_6dof_config_file,
            {
                'frame': controller_frame_id,
                'use_sim_time': True,
            },
        ],
        remappings=[('cmd_vel', '/servo_node/delta_twist_cmds')],
        condition=ps5_6dof_controller_condition,
    )

    gripper_teleop = Node(
        package='franka_moveit_config',
        executable='gamepad_gripper_teleop.py',
        name='gamepad_gripper_teleop_node',
        output='screen',
        parameters=[
            gripper_teleop_config_file,
            {
                'gripper_ns': gripper_namespace,
                'use_sim_time': True,
            },
        ],
        condition=gripper_teleop_condition,
    )

    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    'Launching MoveIt Servo for the Panda arm. '
                    'This reuses /joint_states and publishes trajectories to '
                    '/panda_arm_controller/joint_trajectory.'
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
            LogInfo(
                condition=IfCondition(launch_keyboard_teleop),
                msg=(
                    'Keyboard teleop provides x/y/z translation plus yaw only. '
                    'Roll, pitch, gripper commands, and JointJog are not part of this first setup.'
                ),
            ),
            LogInfo(
                condition=legacy_controller_condition,
                msg=(
                    'Controller teleop publishes continuous TwistStamped commands via '
                    'game_controller_node and teleop_twist_joy to /servo_node/delta_twist_cmds. '
                    'Default preset: hold L1 to enable, hold R1 for turbo, left stick x/y, '
                    'right stick z/yaw; with L1 held, Cross closes the gripper, Square opens, '
                    'Triangle homes, and Circle stops.'
                )
            ),
            LogInfo(
                condition=ps5_6dof_controller_condition,
                msg=(
                    'PS5 6-DOF controller teleop publishes TwistStamped commands to '
                    '/servo_node/delta_twist_cmds. Hold L1 to enable, hold R1 for turbo, '
                    'use left stick x/y and right stick z/yaw normally, and hold L2 for roll/pitch/yaw. '
                    'With L1 held, Cross closes the gripper, Square opens, Triangle homes, and Circle stops.'
                )
            ),
            servo_node,
            start_servo,
            keyboard_teleop,
            controller_joy,
            controller_teleop,
            controller_6dof_teleop,
            gripper_teleop,
        ]
    )
