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
# Use this launch file to launch 3 devices (2x d405 + 1x d455).
# Configured for RGB-only operation: depth, infrared, IMU, pointcloud, and related
# features are disabled by default to minimize USB bandwidth and CPU load.
# To re-enable any stream, override the corresponding parameter at launch time, e.g.:
#   ros2 launch ffw_bringup camera_realsense.launch.py enable_depth3:=true
#
# The Parameters available for definition in the command line for each camera are described in
# rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.

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
serial3 = serials.get('camera3_serial')  # d455

local_parameters = [
                    {'name': 'camera_name1', 'default': 'camera_left',
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
                    # Disable depth streams to reduce USB 2.0 bandwidth
                    {'name': 'enable_depth1', 'default': 'false',
                     'description': 'enable/disable depth stream for camera1'},
                    {'name': 'enable_depth2', 'default': 'false',
                     'description': 'enable/disable depth stream for camera2'},
                    {'name': 'enable_depth3', 'default': 'false',
                     'description': 'enable/disable depth stream for camera3'},
                    # Disable infrared streams to reduce USB 2.0 bandwidth
                    {'name': 'enable_infra11', 'default': 'false',
                     'description': 'enable/disable infra1 stream for camera1'},
                    {'name': 'enable_infra12', 'default': 'false',
                     'description': 'enable/disable infra1 stream for camera2'},
                    {'name': 'enable_infra13', 'default': 'false',
                     'description': 'enable/disable infra1 stream for camera3'},
                    {'name': 'enable_infra21', 'default': 'false',
                     'description': 'enable/disable infra2 stream for camera1'},
                    {'name': 'enable_infra22', 'default': 'false',
                     'description': 'enable/disable infra2 stream for camera2'},
                    {'name': 'enable_infra23', 'default': 'false',
                     'description': 'enable/disable infra2 stream for camera3'},
                    # Color-only profiles
                    {'name': 'depth_module.color_profile1', 'default': '640,480,15',
                     'description': 'Depth module color stream profile for d405 camera1'},
                    {'name': 'depth_module.color_profile2', 'default': '640,480,15',
                     'description': 'Depth module color stream profile for d405 camera2'},
                    {'name': 'rgb_camera.color_profile3', 'default': '640,480,15',
                     'description': 'RGB camera color stream profile for d455 camera3'},
                    # Disable colorizer (no depth to colorize)
                    {'name': 'colorizer.enable1', 'default': 'false',
                     'description': 'enable/disable colorizer filter for camera1'},
                    {'name': 'colorizer.enable2', 'default': 'false',
                     'description': 'enable/disable colorizer filter for camera2'},
                    {'name': 'colorizer.enable3', 'default': 'false',
                     'description': 'enable/disable colorizer filter for camera3'},
                    # Disable pointcloud (depth-based, not needed for RGB-only)
                    {'name': 'pointcloud.enable1', 'default': 'false',
                     'description': 'enable/disable pointcloud for camera1'},
                    {'name': 'pointcloud.enable2', 'default': 'false',
                     'description': 'enable/disable pointcloud for camera2'},
                    {'name': 'pointcloud.enable3', 'default': 'false',
                     'description': 'enable/disable pointcloud for camera3'},
                    # Disable depth-color alignment (no depth to align)
                    {'name': 'align_depth.enable1', 'default': 'false',
                     'description': 'enable/disable depth-color alignment for camera1'},
                    {'name': 'align_depth.enable2', 'default': 'false',
                     'description': 'enable/disable depth-color alignment for camera2'},
                    {'name': 'align_depth.enable3', 'default': 'false',
                     'description': 'enable/disable depth-color alignment for camera3'},
                    # Disable depth/color sync (only relevant when both streams are on)
                    {'name': 'enable_sync1', 'default': 'false',
                     'description': 'enable/disable depth-color sync for camera1'},
                    {'name': 'enable_sync2', 'default': 'false',
                     'description': 'enable/disable depth-color sync for camera2'},
                    {'name': 'enable_sync3', 'default': 'false',
                     'description': 'enable/disable depth-color sync for camera3'},
                    # Disable RGBD topic (requires depth)
                    {'name': 'enable_rgbd1', 'default': 'false',
                     'description': 'enable/disable rgbd topic for camera1'},
                    {'name': 'enable_rgbd2', 'default': 'false',
                     'description': 'enable/disable rgbd topic for camera2'},
                    {'name': 'enable_rgbd3', 'default': 'false',
                     'description': 'enable/disable rgbd topic for camera3'},
                    # Disable IMU streams on d455 (camera3) — only d455 has IMU
                    {'name': 'enable_gyro3', 'default': 'false',
                     'description': 'enable/disable gyro stream for d455 camera3'},
                    {'name': 'enable_accel3', 'default': 'false',
                     'description': 'enable/disable accel stream for d455 camera3'},
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
                }
            ),
        ]
    )
