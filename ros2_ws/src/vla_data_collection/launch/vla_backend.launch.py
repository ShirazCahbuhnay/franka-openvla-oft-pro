import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Paths to other launch files
    vla_share = get_package_share_directory('vla_data_collection')
    franka_share = get_package_share_directory('franka_moveit_config')
    
    camera_launch_path = os.path.join(vla_share, 'launch', 'camera.launch.py')
    moveit_servo_launch_path = os.path.join(franka_share, 'launch', 'moveit_servo.launch.py')
    
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.16.0.2',
        description='IP address of the Franka robot.'
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware (simulation) instead of a real robot.'
    )
    launch_controller_teleop_arg = DeclareLaunchArgument(
        'launch_controller_teleop',
        default_value='true',
        description='Start gamepad controller teleop nodes.'
    )
    use_ps5_6dof_teleop_arg = DeclareLaunchArgument(
        'use_ps5_6dof_teleop',
        default_value='true',
        description='Enable 6-DOF teleop with PS5 controller.'
    )
    
    # Include Camera Launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_path)
    )
    
    # Include MoveIt Servo Launch
    moveit_servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_servo_launch_path),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'launch_controller_teleop': LaunchConfiguration('launch_controller_teleop'),
            'use_ps5_6dof_teleop': LaunchConfiguration('use_ps5_6dof_teleop'),
        }.items()
    )
    
    return LaunchDescription([
        robot_ip_arg,
        use_fake_hardware_arg,
        launch_controller_teleop_arg,
        use_ps5_6dof_teleop_arg,
        camera_launch,
        moveit_servo_launch
    ])
