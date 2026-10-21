#!/usr/bin/env python3
#
# Copyright 2026 ROBOTIS CO., LTD.
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

"""Connect two SICK picoScan lidars to the AI Worker scan topics."""

import ipaddress
import os
import socket

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_sensors(context):
    """Give each driver its own arguments, UDP ports, topics, and layer TF."""
    def value(name):
        return LaunchConfiguration(name).perform(context)

    launch_file = os.path.join(
        get_package_share_directory('sick_scan_xd'), 'launch',
        'sick_picoscan_120.launch' if value('sick_model') == 'picoscan120'
        else 'sick_picoscan.launch',
    )
    ports = [int(value(f'sick_udp_port{i}')) for i in range(2)]
    # Each driver also binds the next port for its UDP loopback check.
    if (any(port < 1024 or port > 65534 for port in ports)
            or len({ports[0], ports[0] + 1, ports[1], ports[1] + 1}) != 4):
        raise ValueError('SICK UDP ports must be 1024..65534 and at least 2 apart')
    if value('sensorip0') == value('sensorip1'):
        raise ValueError('Left and right SICK sensor IP addresses must be different')

    actions = []
    for index, side in [(0, 'left'), (1, 'right')]:
        sensor_ip = str(ipaddress.IPv4Address(value(f'sensorip{index}')))
        receiver_ip = value('udp_receiver_ip')
        if not receiver_ip:
            # Ask the routing table for our source IP; no UDP packet is sent.
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.connect((sensor_ip, 2111))
                receiver_ip = sock.getsockname()[0]
        receiver_address = ipaddress.IPv4Address(receiver_ip)
        if receiver_address.is_unspecified or receiver_address.is_multicast:
            raise ValueError('udp_receiver_ip must be the receiving computer IPv4 address')

        frame = value(f'sick_frame_id{index}')
        topic = '/' + value(f'output_topic{index}').lstrip('/')
        namespace = f'sick_{side}'
        actions.append(LogInfo(msg=(
            f'SICK {side}: {sensor_ip} -> {receiver_ip}:{ports[index]}, '
            f'{topic}, frame {frame}_1'
        )))
        # The upstream Python launch forwards global sys.argv. Pass the XML
        # configuration directly to each process to keep the two sensors separate.
        actions.append(Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            namespace=namespace,
            output='screen',
            arguments=[
                launch_file,
                f'hostname:={sensor_ip}',
                f'udp_receiver_ip:={receiver_ip}',
                f'udp_port:={ports[index]}',
                f'check_udp_receiver_port:={ports[index] + 1}',
                f'publish_frame_id:={frame}',
                f'publish_laserscan_fullframe_topic:={topic}',
                'publish_laserscan_segment_topic:=scan_segment',
                'imu_enable:=0',
                'verbose_level:=0',
                'tf_publish_rate:=0',
                'host_FREchoFilter:=2',
                'custom_pointclouds:=cloud_unstructured_fullframe',
                'cloud_unstructured_fullframe:=coordinateNotation=0 updateMethod=0 '
                'echos=0,1,2 layers=1 reflectors=0,1 infringed=0,1 '
                f'rangeFilter=0,999,0 topic=cloud frameid={frame} publish=1',
            ],
        ))
        # picoScan has one scan layer; sick_scan_xd appends "_1" to its frame.
        # The existing robot URDF owns base_link -> lidar_[lr]_link.
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'sick_{side}_layer_tf',
            arguments=['--frame-id', frame, '--child-frame-id', f'{frame}_1'],
        ))
    return actions


def generate_launch_description():
    """Launch two picoScan drivers for the existing AI Worker scan merger."""
    return LaunchDescription([
        DeclareLaunchArgument('sensorip0', default_value='192.168.6.3',
                              description='Left picoScan IPv4 address'),
        DeclareLaunchArgument('sensorip1', default_value='192.168.6.4',
                              description='Right picoScan IPv4 address'),
        DeclareLaunchArgument('output_topic0', default_value='scan_left'),
        DeclareLaunchArgument('output_topic1', default_value='scan_right'),
        DeclareLaunchArgument('udp_receiver_ip', default_value='',
                              description='Host IPv4; empty uses the route to each sensor'),
        DeclareLaunchArgument('sick_udp_port0', default_value='2115',
                              description='Left scan UDP port; the next port is also reserved'),
        DeclareLaunchArgument('sick_udp_port1', default_value='2117',
                              description='Right scan UDP port; the next port is also reserved'),
        DeclareLaunchArgument('sick_frame_id0', default_value='lidar_l_link'),
        DeclareLaunchArgument('sick_frame_id1', default_value='lidar_r_link'),
        DeclareLaunchArgument('sick_model', default_value='picoscan',
                              choices=['picoscan', 'picoscan120'],
                              description='SICK configuration: picoScan150 or picoScan120'),
        OpaqueFunction(function=launch_sensors),
    ])
