#!/usr/bin/env python3
# Copyright 2025 ROBOTIS CO., LTD.
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
#
# Author: Woojin Wie

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


MODEL_DIRECTORIES = {
    'bg2': 'ffw_bg2_rev4_follower',
    'sg2': 'ffw_sg2_rev1_follower',
    'f1': 'ffw_f1_follower',
    'f2': 'ffw_f2_follower',
}


def launch_setup(context):
    model = LaunchConfiguration('model').perform(context)
    description_share = Path(get_package_share_directory('ffw_description'))
    config_share = Path(get_package_share_directory('ffw_moveit_config'))
    model_config = Path('config') / model
    urdf = (description_share / 'urdf' / MODEL_DIRECTORIES[model]
            / f'ffw_{model}_follower.urdf.xacro')
    use_sim = LaunchConfiguration('use_sim')

    moveit_config = (
        MoveItConfigsBuilder(robot_name=f'ffw_{model}_follower',
                             package_name='ffw_moveit_config')
        .robot_description(file_path=urdf, mappings={'use_sim': use_sim.perform(context)})
        .robot_description_semantic(file_path=model_config / 'robot.srdf')
        .robot_description_kinematics(file_path='config/common/kinematics.yaml')
        .joint_limits(file_path=model_config / 'joint_limits.yaml')
        .trajectory_execution(file_path='config/common/moveit_controllers.yaml',
                              moveit_manage_controllers=False)
        .planning_pipelines(default_planning_pipeline='ompl',
                            pipelines=['ompl', 'pilz_industrial_motion_planner', 'chomp'])
        .pilz_cartesian_limits(file_path='config/common/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )
    # Jazzy's builder searches only config/ for pipeline files. Load shared
    # overrides explicitly while retaining the standard OMPL planner definitions.
    for pipeline in moveit_config.planning_pipelines['planning_pipelines']:
        with (config_share / 'config/common' / f'{pipeline}_planning.yaml').open() as stream:
            moveit_config.planning_pipelines[pipeline].update(yaml.safe_load(stream))
    warehouse = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': LaunchConfiguration('warehouse_sqlite_path'),
    }
    clock = {'use_sim_time': ParameterValue(use_sim, value_type=bool)}
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), warehouse, clock, {
            'publish_robot_description_semantic': ParameterValue(
                LaunchConfiguration('publish_robot_description_semantic'), value_type=bool),
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2_moveit', output='log',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        arguments=['-d', str(config_share / 'config/common/moveit.rviz')],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics, moveit_config.planning_pipelines,
                    moveit_config.joint_limits, warehouse, clock],
    )
    return [move_group, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='bg2', choices=list(MODEL_DIRECTORIES),
                              description='Follower model used for motion planning.'),
        DeclareLaunchArgument('start_rviz', default_value='true',
                              description='Whether to execute RViz.'),
        DeclareLaunchArgument('use_sim', default_value='false',
                              description='Whether to use simulation time.'),
        DeclareLaunchArgument('warehouse_sqlite_path',
                              default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
                              description='Path to the warehouse database.'),
        DeclareLaunchArgument('publish_robot_description_semantic', default_value='true',
                              description='Whether to publish the semantic robot description.'),
        OpaqueFunction(function=launch_setup),
    ])
