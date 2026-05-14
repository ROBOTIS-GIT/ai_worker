^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ffw
^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.1 (2026-05-14)
------------------
* Added dual_laser_merger for SH5
* Added unified ffw_pack.launch file
* Removed bg2 pack launch file
* Add pack position yaml files in each robot type config folder
* Contributors: Howon Kim, Hyungyu Kim

1.3.0 (2026-05-04)
------------------
* Improved 3-wheel swerve steering limit using flip logic
* Extended neck downward angle by 10 degrees
* Contributors: Sungho Woo

1.2.2 (2026-04-27)
------------------
* Unified s6-overlay ROS 2 longrun services into ai_worker_bringup and avatar_bringup
* Simplified docker/container.sh and removed noVNC-oriented Docker Compose workflow
* Contributors: Hyungyu Kim

1.2.1 (2026-04-09)
------------------
* Fixed palm joint name for HX5
* Contributors: Hyunwoo Nam

1.2.0 (2026-04-01)
------------------
* Added support for SH5, BH5
* Contributors: Hyunwoo Nam

1.1.21 (2026-03-26)
-------------------
* Added docker-compose.novnc.yml
* Contributors: Wonho Yun

1.1.20 (2026-03-12)
-------------------
* Fixed version of docker image in docker-compose.yml
* Added a notice in container.sh when update is available
* Contributors: Hyungyu Kim

1.1.19 (2026-02-19)
-------------------
* Added MPPI navigation mode
* Merged AMCL params into navigation.yaml and added navigation_mppi.yaml
* Added BT tree with IsPathLengthUnder
* Added a console entry point for the mobile teleop node
* Contributors: Yongjun Kwon

1.1.18 (2026-02-09)
-------------------
* Use sigint signal to shutdown s6-overlay services
* Contributors: Hyungyu Kim

1.1.17 (2026-01-27)
-------------------
* Supported System Manager
* Added s6-overlay services
* Contributors: Hyungyu Kim

1.1.16 (2026-01-20)
-------------------
* Added dual_laser_merger for SG2
* Configured ros_gz_bridge via YAML
* Updated Gazebo world and spawn parameters
* Added Lidar support (URDF & Gazebo config)
* Corrected inertial parameters for stability
* Optimized wheel collision geometry
* Initial release of ffw_navigation
* Added Nav2, SLAM Toolbox, and AMCL config
* Added default map and RViz setup
* Contributors: Sungho Woo, HyunKyu Kim, Yongjun Kwon

1.1.15 (2025-12-09)
-------------------
* Fixed Position and Velocity Unit for lift
* Added Support for FFW SG2 lakibeam lidar
* Added head eef tracker feature
* Fixed deprecated fake_hardware to mock_hardware
* Updated for new realtime_tools::RealtimePublisher API
* Contributors: Woojin Wie, Wonho Yun

1.1.14 (2025-10-14)
-------------------
* Corrected the battery monitoring feature when exceeding the maximum voltage
* Contributors: Woojin Wie

1.1.13 (2025-10-14)
-------------------
* Added support for battery monitoring for ffw_sg2_rev1
* Contributors: Woojin Wie

1.1.12 (2025-09-17)
-------------------
* Updated docker compose file to mount zed resources
* Contributors: Woojin Wie

1.1.11 (2025-08-13)
-------------------
* Updated URDF for ZED camera
* Fix velocity limit variable for ffw_swerve_drive_controller
* Added tactile switch feature to joystick controller
* Applied Dynamic Brake for Dynamixel Y
* Contributors: Woojin Wie, Wonho Yun

1.1.10 (2025-07-22)
--------------------
* Added start/pause feature for ffw_joint_trajectory_command_broadcaster
* Merged two joint_trajectory_command_broadcaster into one for ffw_lg2_leader
* Added ffw_robot_manager package in meta package
* Added URDF for ZED camera
* Modified MoveIt files
* Updated Dockerfile
* Contributors: Wonho Yun, Woojin Wie

1.1.9 (2025-07-18)
------------------
* Added robot manager ros2 controller
* Added Swerve Steer initialization feature
* Fixed Gazebo simulation issue
* Added error code reading feature for Dynamixel
* Contributors: Woojin Wie

1.1.8 (2025-07-14)
------------------
* Modified joystick controller to enable lift control while in swerve mode
* Refactored joystick controller
* Contributors: Wonho Yun

1.1.7 (2025-07-11)
------------------
* Added slow start feature for joint_trajectory_command_broadcaster
* Contributors: Woojin Wie

1.1.6 (2025-07-07)
------------------
* Updated ros2_control xacro file for ffw_bg2_rev4_follower
* Fix header file name for tf2
* Contributors: Woojin Wie

1.1.5 (2025-06-30)
------------------
* Updated ros2_control xacro file for ffw_bg2_rev2_follower
* Contributors: Woojin Wie

1.1.4 (2025-06-27)
------------------
* Added ROS_DOMAIN_ID to the Dockerfile
* Contributors: Woojin Wie

1.1.3 (2025-06-26)
------------------
* Modified jog scale for ffw_lg2_leader
* Added dependencies to the package.xml file for image_transport_plugins
* Contributors: Woojin Wie

1.1.2 (2025-06-26)
------------------
* Added dependencies to the package.xml file
* Contributors: Woojin Wie

1.1.1 (2025-06-26)
------------------
* Reordered pip install order in Dockerfile to fix the numpy version issue
* Added Current Limit parameter to the ros2_control xacro file for ffw_sg2_rev1
* Contributors: Woojin Wie

1.1.0 (2025-06-16)
------------------
* Add installation of some ROS 2 packages for physical AI tools in Dockerfile
* Add an alias command in Dockerfile for running the physical AI server
* Support ffw_sg2_rev1 Model
* Add swerve drive controller package for ffw_sg2_rev1
* Modify joystick controller to support swerve mode
* Contributors: Kiwoong Park, Woojin Wie, Geonhee Lee, Wonho Yun

1.0.9 (2025-06-09)
------------------
* Updated urdf files for ffw_bg2_rev4
* Modified Gazebo launch file
* Contributors: Woojin Wie, Wonho Yun

1.0.8 (2025-06-02)
------------------
* Updated Model files for ffw_bg2_rev4
* Contributors: Woojin Wie

1.0.7 (2025-05-30)
------------------
* Updated Model files for ffw_bg2_rev4
* Contributors: Woojin Wie

1.0.6 (2025-05-28)
------------------
* Modified Docker volume mapping
* Created RealSense and ZED launch files
* Adjusted joint names
* Improved file structure
* Removed deprecated files
* Contributors: Woojin Wie

1.0.5 (2025-05-09)
------------------
* Fixed Dockerfile
* Updated Camera URDF
* Contributors: Woojin Wie

1.0.4 (2025-05-08)
------------------
* Fixed Dockerfile
* Updated ros2 control xacro file to support async
* Contributors: Woojin Wie

1.0.3 (2025-04-28)
------------------
* Added support for Joystick controller
* Added ffw_spring_actuator_controller
* Contributors: Woojin Wie

1.0.2 (2025-04-16)
------------------
* Added support for ROBOTIS RH Gripper
* Added differentiation between slow and fast versions
* Updated codebase to comply with flake8 linting standards
* Contributors: Wonho Yun

1.0.1 (2025-04-07)
------------------
* Modified the profile velocity parameters for enhanced arm and hand teleoperation performance
* Modified the README file to reflect usage instructions for this package
* Removed unused files and redundant comments to streamline the codebase
* Contributors: Wonho Yun, Pyo

1.0.0 (2025-04-06)
------------------
* Added the initial version of the FFW ROS package
* Added arm and hand teleoperation support for FFW
* Added integrated controller compatibility for Inspire Robot Hand
* Contributors: Sungho Woo, Woojin Wie, Wonho Yun, Pyo

0.1.0 (2025-03-27)
------------------
* Added bringup scripts for system initialization
* Added robot description files for visualization and planning
* Added base controller functionalities
* Added MoveIt for motion planning support
* Contributors: Sungho Woo, Woojin Wie
