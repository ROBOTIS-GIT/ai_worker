import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_navigation = get_package_share_directory('ffw_navigation')
    os.environ.setdefault("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + pkg_navigation + "/gazebo_models/"

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz',
        default_value = 'true',
        description = 'Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value = 'navigation.rviz',
        description = 'RViz config file'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value = 'True',
        description = 'Flag to enable use_sim_time'
    )

    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    localization_params_path = os.path.join(
        pkg_navigation,
        'config',
        'amcl_localization.yaml'
    )

    navigation_params_path = os.path.join(
        pkg_navigation,
        'config',
        'navigation.yaml'
    )

    map_file_path = os.path.join(
        pkg_navigation,
        'maps',
        'map.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_navigation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': localization_params_path,
                'map': map_file_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            params_file': navigation_params_path,
        }.items()
    )

    launchDescriptionObject = LaunchDescription()
    launchDescriptionObject.add_action(rviz_launch_arg)
    launchDescriptionObject.add_action(rviz_config_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(rviz_node)
    launchDescriptionObject.add_action(localization_launch)
    launchDescriptionObject.add_action(navigation_launch)

    return launchDescriptionObject
