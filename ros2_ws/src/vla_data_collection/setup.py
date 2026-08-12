import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vla_data_collection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seb',
    maintainer_email='lzeghib@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'instruction_node = vla_data_collection.instruction_node:main',
            'rosbag2_trim_sync = vla_data_collection.rosbag2_trim_sync:main',
            'rosbag2_to_rlds = vla_data_collection.rosbag2_to_rlds:main',
            'rosbag2_folder_to_rlds = vla_data_collection.rosbag2_folder_to_rlds:main',
            'rosbag2_to_rlds_folder_noop = '
            'vla_data_collection.rosbag2_to_rlds_folder_noop:main',
            'rlds_merge = vla_data_collection.rlds_merge:main',
            'rlds_rosbag_report = vla_data_collection.rlds_rosbag_report:main',
            'collect_episode = vla_data_collection.collect_episode:main',
            'openvla_oft_inference_node = vla_data_collection.openvla_oft_inference_node:main',
            'openvla_oft_inference_node_more_action = '
            'vla_data_collection.openvla_oft_inference_node_more_action:main',
        ],
    },
)
