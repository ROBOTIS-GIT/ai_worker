^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ffw_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2026-06-30)
------------------
* Fixed zenohd alias command to run rmw_zenoh_cpp rmw_zenohd
* Contributors: kimtaehyeong99

2.0.0 (2026-06-17)
------------------
* Set rmw_zenoh_cpp as the default RMW in Docker images
* Contributors: Hyungyu Kim

1.3.2 (2026-06-02)
------------------
* Updated wheel radius parameters for SH5, SG2
* Contributors: Howon Kim

1.3.1 (2026-05-14)
------------------
* Added dual_laser_merger for SH5
* Added unified ffw_pack.launch file
* Removed bg2 pack launch file
* Add pack position yaml files in each robot type config folder
* Contributors: Howon Kim, Hyungyu Kim

1.3.0 (2026-05-04)
------------------
* Bumped package version to 1.3.0

1.2.2 (2026-04-27)
------------------
* None

1.2.1 (2026-04-09)
------------------
* None

1.2.0 (2026-04-01)
------------------
* Added support for SH5, BH5
* Contributors: Hyunwoo Nam

1.1.21 (2026-03-26)
-------------------
* None

1.1.20 (2026-03-12)
-------------------
* None

1.1.19 (2026-02-19)
-------------------
* None

1.1.18 (2026-02-09)
-------------------
* None

1.1.17 (2026-01-27)
-------------------
* None

1.1.16 (2026-01-20)
-------------------
* Added dual_laser_merger for SG2
* Configured ros_gz_bridge via YAML
* Updated Gazebo world and spawn parameters
* Contributors: Yongjun Kwon

1.1.15 (2025-12-09)
-------------------
* Added head eef tracker feature
* Added support for FFW SG2 lakibeam lidar
* Fixed deprecated fake_hardware to mock_hardware
* Contributors: Woojin Wie, Wonho Yun

1.1.14 (2025-10-14)
-------------------
* None

1.1.13 (2025-10-14)
-------------------
* None

1.1.12 (2025-09-17)
-------------------
* None

1.1.11 (2025-08-13)
-------------------
* None

1.1.10 (2025-07-22)
-------------------
* Merged two joint_trajectory_command_broadcaster into one
* Contributors: Wonho Yun

1.1.9 (2025-07-18)
------------------
* Added Swerve Steer initialization feature
* Fixed Gazebo simulation issue
* Contributors: Woojin Wie

1.1.8 (2025-07-14)
------------------
* None

1.1.7 (2025-07-11)
------------------
* None

1.1.6 (2025-07-07)
------------------
* None

1.1.5 (2025-06-30)
------------------
* None

1.1.4 (2025-06-27)
------------------
* None

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
* None

1.1.0 (2025-06-16)
------------------
* Support ffw_sg2_rev1 Model
* Contributors: Woojin Wie

1.0.9 (2025-06-09)
------------------
* Modified Gazebo launch file
* Contributors: Wonho Yun

1.0.8 (2025-06-02)
------------------
* None

1.0.7 (2025-05-30)
------------------
* None

1.0.6 (2025-05-28)
------------------
* Added RealSense and ZED launch files
* Contributors: Woojin Wie

1.0.5 (2025-05-09)
------------------
* None

1.0.4 (2025-05-08)
------------------
* None

1.0.3 (2025-04-28)
------------------
* None

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
