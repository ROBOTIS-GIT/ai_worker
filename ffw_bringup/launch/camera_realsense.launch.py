# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in
# rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_multi_camera_launch.py \
#     camera_name1:=D400 \
#     device_type1:=d4 \
#     device_type2:=l5

"""Launch realsense2_camera node."""
import copy
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml

# Add realsense2_camera/launch to sys.path using ROS package discovery
realsense2_camera_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'),
                                            'launch')
sys.path.append(realsense2_camera_launch_dir)
import rs_launch  # noqa: E402, I100


# Utility function to load YAML as dict
def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


# Read serial numbers from rs_serial.yaml
serials_path = os.path.join(get_package_share_directory('ffw_bringup'), 'config', 'common',
                            'rs_serial.yaml')
serials = yaml_to_dict(serials_path)
serial1 = serials.get('camera1_serial')
serial2 = serials.get('camera2_serial')

local_parameters = [{'name': 'camera_name1', 'default': 'camera_left',
                     'description': 'camera1 unique name'},
                    {'name': 'camera_name2', 'default': 'camera_right',
                     'description': 'camera2 unique name'},
                    {'name': 'camera_namespace1', 'default': 'camera_left',
                     'description': 'camera1 namespace'},
                    {'name': 'camera_namespace2', 'default': 'camera_right',
                     'description': 'camera2 namespace'},
                    {'name': 'serial_no1', 'default': serial1,
                     'description': 'choose device1 by serial number'},
                    {'name': 'serial_no2', 'default': serial2,
                     'description': 'choose device2 by serial number'},
                    {'name': 'depth_module.depth_profile1', 'default': '480,270,30',
                     'description': 'depth stream profile for camera1'},
                    {'name': 'depth_module.depth_profile2', 'default': '480,270,30',
                     'description': 'depth stream profile for camera2'},
                    {'name': 'depth_module.color_profile1', 'default': '424,240,30',
                     'description': 'Depth module color stream profile for d405 camera1'},
                    {'name': 'depth_module.color_profile2', 'default': '424,240,30',
                     'description': 'Depth module color stream profile for d405 camera2'},
                    {'name': 'colorizer.enable1', 'default': 'true',
                     'description': 'enable colorizer filter for camera1'},
                    {'name': 'colorizer.enable2', 'default': 'true',
                     'description': 'enable colorizer filter for camera2'},
                    ]


def set_configurable_parameters(local_params):
    return {param['original_name']: LaunchConfiguration(param['name'])
            for param in local_params}


def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params


def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        rs_launch.declare_configurable_parameters(params2) +
        [
            OpaqueFunction(
                function=rs_launch.launch_setup,
                kwargs={
                    'params': set_configurable_parameters(params1),
                    'param_name_suffix': '1'
                }
            ),
            OpaqueFunction(
                function=rs_launch.launch_setup,
                kwargs={
                    'params': set_configurable_parameters(params2),
                    'param_name_suffix': '2'
                }
            ),
        ]
    )
