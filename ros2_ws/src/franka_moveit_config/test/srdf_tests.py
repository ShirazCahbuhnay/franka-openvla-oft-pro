#  Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from os import path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro

panda_xacro_file_name = path.join(get_package_share_directory('franka_moveit_config'), 'srdf',
                                  'panda_arm.srdf.xacro')


def test_load():
    urdf = xacro.process_file(panda_xacro_file_name).toxml()
    assert urdf.find('panda_rightfinger') != -1


def test_load_without_gripper():
    urdf = xacro.process_file(panda_xacro_file_name,
                              mappings={'hand': 'false'}).toxml()
    assert urdf.find('panda_rightfinger') == -1


def test_load_with_arm_id():
    urdf = xacro.process_file(panda_xacro_file_name,
                              mappings={'arm_id': 'totally_different_arm'}).toxml()
    assert urdf.find('totally_different_arm_joint1') != -1


def test_hand_open_state_matches_supported_width():
    urdf = xacro.process_file(panda_xacro_file_name).toxml()
    root = ET.fromstring(urdf)
    open_state = root.find("./group_state[@name='open'][@group='hand']")
    assert open_state is not None

    finger_1 = open_state.find("./joint[@name='panda_finger_joint1']")
    finger_2 = open_state.find("./joint[@name='panda_finger_joint2']")
    assert finger_1 is not None
    assert finger_2 is not None
    assert float(finger_1.attrib['value']) == 0.026
    assert float(finger_2.attrib['value']) == 0.026


if __name__ == '__main__':
    pass
