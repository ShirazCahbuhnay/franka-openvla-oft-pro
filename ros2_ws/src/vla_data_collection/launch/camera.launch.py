import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your config file
    config_dir = os.path.join(get_package_share_directory('vla_data_collection'), 'config')
    param_file = os.path.join(config_dir, 'camera_params.yaml')

    # Define the usb_cam node
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='vla_front_camera',
        parameters=[param_file],
        output='screen'
    )

    return LaunchDescription([
        camera_node
    ])
