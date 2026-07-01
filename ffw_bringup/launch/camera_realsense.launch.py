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
import re
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import yaml

# Add realsense2_camera/launch to sys.path using ROS package discovery
realsense2_camera_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'),
                                            'launch')
sys.path.append(realsense2_camera_launch_dir)
import rs_launch  # noqa: E402, I100


# Utility function to load YAML as dict
def yaml_to_dict(path_to_yaml):
    if not os.path.exists(path_to_yaml):
        return {}
    with open(path_to_yaml, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader) or {}


def serials_from_realsense_context():
    try:
        import pyrealsense2 as rs
    except ImportError:
        return []

    serials = []
    try:
        context = rs.context()
    except RuntimeError:
        return []

    for device in context.query_devices():
        try:
            serials.append(device.get_info(rs.camera_info.serial_number))
        except RuntimeError:
            continue
    return serials


def serials_from_rs_enumerate_devices():
    try:
        result = subprocess.run(
            ['rs-enumerate-devices'],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return []

    if result.returncode != 0:
        return []

    return re.findall(r'Serial Number\s*:\s*([0-9]+)', result.stdout)


def discover_realsense_serials():
    seen = set()
    serials = []
    for serial in serials_from_realsense_context() or serials_from_rs_enumerate_devices():
        if serial and serial not in seen:
            seen.add(serial)
            serials.append(serial)
    return serials


def format_serial_for_launch(serial):
    serial = str(serial).strip()
    if serial.startswith("'") and serial.endswith("'"):
        return serial
    return f"'{serial}'"


def serials_to_launch_dict(serials):
    return {
        f'camera{index}_serial': format_serial_for_launch(serial)
        for index, serial in enumerate(serials, start=1)
    }


def write_serials_yaml(path_to_yaml, serials):
    directory = os.path.dirname(path_to_yaml)
    if directory:
        os.makedirs(directory, exist_ok=True)
    serials_dict = serials_to_launch_dict(serials)
    with open(path_to_yaml, 'w') as f:
        f.write('# Auto-generated RealSense serial numbers for this robot.\n')
        f.write('# Delete this file to re-detect connected cameras on the next bringup.\n')
        for key, value in serials_dict.items():
            f.write(f'{key}: "{value}"\n')


def load_realsense_serials():
    persistent_path = os.environ.get('FFW_RS_SERIAL_PATH', '/workspace/config/rs_serial.yaml')
    fallback_path = os.path.join(
        get_package_share_directory('ffw_bringup'), 'config', 'common', 'rs_serial.yaml')

    persistent_serials = yaml_to_dict(persistent_path)
    if persistent_serials:
        print(f'[camera_realsense] Using RealSense serials from {persistent_path}')
        return persistent_serials

    discovered_serials = discover_realsense_serials()
    if discovered_serials:
        try:
            write_serials_yaml(persistent_path, discovered_serials)
            print(f'[camera_realsense] Saved RealSense serials to {persistent_path}')
        except OSError as exc:
            print(f'[camera_realsense] Could not save RealSense serials to '
                  f'{persistent_path}: {exc}')
        return serials_to_launch_dict(discovered_serials)

    print(f'[camera_realsense] Could not auto-detect RealSense serials. '
          f'Using fallback serials from {fallback_path}')
    return yaml_to_dict(fallback_path)


serials = load_realsense_serials()
serial1 = serials.get('camera1_serial', '')
serial2 = serials.get('camera2_serial', '')
serial3 = serials.get('camera3_serial', '')  # d455 head, only used when enable_head_camera=true

local_parameters = [{'name': 'camera_name1', 'default': 'camera_left',
                     'description': 'camera1 unique name'},
                    {'name': 'camera_name2', 'default': 'camera_right',
                     'description': 'camera2 unique name'},
                    {'name': 'camera_name3', 'default': 'camera_head',
                     'description': 'camera3 unique name'},
                    {'name': 'camera_namespace1', 'default': 'camera_left',
                     'description': 'camera1 namespace'},
                    {'name': 'camera_namespace2', 'default': 'camera_right',
                     'description': 'camera2 namespace'},
                    {'name': 'camera_namespace3', 'default': 'camera_head',
                     'description': 'camera3 namespace'},
                    {'name': 'serial_no1', 'default': serial1,
                     'description': 'choose device1 by serial number'},
                    {'name': 'serial_no2', 'default': serial2,
                     'description': 'choose device2 by serial number'},
                    {'name': 'serial_no3', 'default': serial3,
                     'description': 'choose device3 by serial number'},
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
    params3 = duplicate_params(rs_launch.configurable_parameters, '3')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_head_camera',
                default_value='false',
                choices=['true', 'false'],
                description='Launch the d455 head camera in addition to the two d405 wrist '
                            'cameras. Set to true on f2 (3-camera setup), false on '
                            'sg2/bg2/sh5/bh5 (2-camera setup with zed head).'
            ),
        ] +
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        rs_launch.declare_configurable_parameters(params2) +
        rs_launch.declare_configurable_parameters(params3) +
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
            OpaqueFunction(
                function=rs_launch.launch_setup,
                kwargs={
                    'params': set_configurable_parameters(params3),
                    'param_name_suffix': '3'
                },
                condition=IfCondition(LaunchConfiguration('enable_head_camera'))
            ),
        ]
    )
