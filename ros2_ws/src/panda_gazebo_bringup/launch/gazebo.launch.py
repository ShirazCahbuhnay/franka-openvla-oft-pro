import os
import sys

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
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
    gui_config_file = os.path.join(
        get_package_share_directory('panda_gazebo_bringup'),
        'config',
        'gazebo_gui.config',
    )

    robot_xacro_file = os.path.join(
        get_package_share_directory('panda_gazebo_bringup'),
        'urdf',
        'panda_gazebo.urdf.xacro',
    )
    default_ros2_controllers_file = PathJoinSubstitution(
        [FindPackageShare('panda_gazebo_bringup'), 'config', 'ros2_controllers.yaml']
    )
    cbf_ros2_controllers_file = PathJoinSubstitution(
        [FindPackageShare('panda_gazebo_bringup'), 'config', 'ros2_controllers_cbf.yaml']
    )
    ros2_controllers_file = IfElseSubstitution(
        use_cbf_controller,
        if_value=cbf_ros2_controllers_file,
        else_value=default_ros2_controllers_file,
    )
    table_scene_config = load_table_scene_config()
    scene_camera_config = table_scene_config.get('scene_camera', {})
    table_world_file = generate_table_world_file(table_scene_config)
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            robot_xacro_file,
            ' arm_id:=',
            arm_id,
            ' hand:=',
            hand,
            ' ros2_controllers_file:=',
            ros2_controllers_file,
        ]
    )
    robot_description = {
        'robot_description': robot_description_content,
        'use_sim_time': True,
    }

    resource_paths = [
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        os.path.dirname(get_package_share_directory('franka_description')),
        os.path.dirname(get_package_share_directory('panda_gazebo_bringup')),
    ]
    gz_resource_path = os.pathsep.join([path for path in resource_paths if path])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
                )
            ]
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    map_to_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
        parameters=[{'use_sim_time': True}],
    )

    clock_bridge_actions = []
    scene_camera_tf_nodes = []
    scene_camera_bridge_actions = []
    ros_gz_bridge_available = False
    try:
        get_package_share_directory('ros_gz_bridge')
        ros_gz_bridge_available = True
        clock_bridge_actions = [
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='clock_bridge',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            )
        ]
    except PackageNotFoundError:
        clock_bridge_actions = [
            LogInfo(
                msg=(
                    'ros_gz_bridge is not installed, so /clock will not be bridged into ROS. '
                    'Nodes using use_sim_time, including MoveIt Servo teleop, will stay at time zero.'
                )
            )
        ]

    if scene_camera_config.get('enabled', False):
        scene_camera_tf_nodes = [
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='scene_camera_frame_broadcaster',
                output='screen',
                arguments=[
                    '--x',
                    str(scene_camera_config['position'][0]),
                    '--y',
                    str(scene_camera_config['position'][1]),
                    '--z',
                    str(scene_camera_config['position'][2]),
                    '--roll',
                    str(scene_camera_config['orientation_rpy'][0]),
                    '--pitch',
                    str(scene_camera_config['orientation_rpy'][1]),
                    '--yaw',
                    str(scene_camera_config['orientation_rpy'][2]),
                    '--frame-id',
                    'world',
                    '--child-frame-id',
                    scene_camera_config['frame_id'],
                ],
                parameters=[{'use_sim_time': True}],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='scene_camera_optical_frame_broadcaster',
                output='screen',
                arguments=[
                    '--roll',
                    str(-1.5707963267948966),
                    '--pitch',
                    '0.0',
                    '--yaw',
                    str(-1.5707963267948966),
                    '--frame-id',
                    scene_camera_config['frame_id'],
                    '--child-frame-id',
                    scene_camera_config['optical_frame_id'],
                ],
                parameters=[{'use_sim_time': True}],
            ),
        ]

        if ros_gz_bridge_available:
            scene_camera_bridge_actions = [
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='scene_camera_bridge',
                    output='screen',
                    condition=IfCondition(bridge_scene_camera),
                    parameters=[{'use_sim_time': True}],
                    # Humble's parameter_bridge reliably supports topic arguments plus ROS remapping.
                    arguments=[
                        (
                            f"{scene_camera_config['gz_topic']}"
                            '@sensor_msgs/msg/Image[gz.msgs.Image'
                        ),
                        (
                            f"{scene_camera_config['gz_camera_info_topic']}"
                            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
                        ),
                        '--ros-args',
                        '-r',
                        (
                            f"{scene_camera_config['gz_topic']}:="
                            f"{scene_camera_config['ros_image_topic']}"
                        ),
                        '-r',
                        (
                            f"{scene_camera_config['gz_camera_info_topic']}:="
                            f"{scene_camera_config['ros_camera_info_topic']}"
                        ),
                    ],
                )
            ]
        else:
            scene_camera_bridge_actions = [
                LogInfo(
                    condition=IfCondition(bridge_scene_camera),
                    msg=(
                        'Fixed scene camera is enabled, but ros_gz_bridge is not installed. '
                        "Gazebo will still publish native topics '%s' and '%s'. "
                        "Install ros_gz_bridge to expose ROS topics '%s' and '%s'."
                        % (
                            scene_camera_config['gz_topic'],
                            scene_camera_config['gz_camera_info_topic'],
                            scene_camera_config['ros_image_topic'],
                            scene_camera_config['ros_camera_info_topic'],
                        )
                    ),
                )
            ]

    joint_state_combiner = Node(
        package='panda_gazebo_bringup',
        executable='joint_state_combiner.py',
        name='joint_state_combiner',
        output='screen',
        parameters=[
            {
                'arm_joint_states_topic': '/arm/joint_states',
                'finger_joint_names': [
                    'panda_finger_joint1',
                    'panda_finger_joint2',
                ],
                'gripper_joint_states_topic': ['/', arm_id, '_gripper/joint_states'],
                'finger_position': 0.026,
                'publish_rate_hz': 60.0,
                'use_sim_time': True,
            },
        ],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '120'],
        output='screen',
    )

    panda_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_arm_controller', '--controller-manager-timeout', '120'],
        output='screen',
    )

    panda_hand_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_hand_controller', '--controller-manager-timeout', '120'],
        output='screen',
        condition=IfCondition(hand),
    )

    fake_gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py']
                )
            ]
        ),
        launch_arguments={
            'robot_ip': 'fake',
            'use_fake_hardware': 'true',
            'arm_id': arm_id,
            'trajectory_command_topic': ['/', arm_id, '_hand_controller/joint_trajectory'],
            'visual_max_width': '0.052',
        }.items(),
    )

    startup_home_goal = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'action',
            'send_goal',
            '/panda_arm_controller/follow_joint_trajectory',
            'control_msgs/action/FollowJointTrajectory',
            '{trajectory: {joint_names: [panda_joint1, panda_joint2, panda_joint3, '
            'panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: '
            '[0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, '
            '1.5707963267948966, 0.7853981633974483], velocities: [0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0], time_from_start: {sec: 4}}]}}',
        ],
        output='screen',
        condition=IfCondition(initialize_home_pose),
    )

    return LaunchDescription(
        [
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
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
            gazebo,
            *clock_bridge_actions,
            map_to_world,
            *scene_camera_tf_nodes,
            robot_state_publisher,
            joint_state_combiner,
            fake_gripper,
            spawn,
            *scene_camera_bridge_actions,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn,
                    on_exit=[joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[panda_arm_controller, panda_hand_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=panda_arm_controller,
                    on_exit=[startup_home_goal],
                )
            ),
        ]
    )
