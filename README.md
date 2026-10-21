# ROBOTIS AI Worker ROS 2 Packages

This repository contains the official ROS 2 packages for the ROBOTIS AI Worker platform. These packages provide the necessary interfaces and tools to control the robot, integrate with its sensors, and develop physical AI applications, including those utilizing frameworks like LeRobot. For detailed usage instructions, please refer to the documentation below.
  - [Documentation for AI Worker](https://ai.robotis.com/)

To learn more about the Physical AI Tools, visit:
  - [Physical AI Tools](https://github.com/ROBOTIS-GIT/physical_ai_tools)

To explore our open-source platforms in a simulation environment, visit:
  - [Simulation Models](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)

For usage instructions and demonstrations of the AI Worker, check out:
  - [Tutorial Videos](https://www.youtube.com/@ROBOTISOpenSourceTeam)

To access datasets and pre-trained models for our open-source platforms, see:
  - [AI Models & Datasets](https://huggingface.co/ROBOTIS)

To use the Docker image for running ROS packages and Physical AI tools with the AI Worker, visit:
  - [Docker Images](https://hub.docker.com/r/robotis/ros/tags)

## SICK picoScan LiDAR

The [official SICK driver](https://github.com/SICKAG/sick_scan_xd) is available as
`ros-jazzy-sick-scan-xd`. Both Dockerfiles install it and build the local
`ffw_bringup` package. Run Docker builds from the repository root so that the
local launch files are included. Existing containers need the package installed:

```bash
docker exec ai_worker bash -c \
  'apt-get update && apt-get install -y --no-install-recommends ros-jazzy-sick-scan-xd'
```

The running container may mount a different checkout; check `docker inspect
ai_worker` before building. To install this checkout's bringup package in a
separate workspace inside an existing container, run from the repository root:

```bash
docker exec ai_worker mkdir -p /opt/ai_worker_sick/src
docker cp ffw_bringup ai_worker:/opt/ai_worker_sick/src/
docker exec ai_worker bash -c \
  'source /root/ros2_ws/install/setup.bash && cd /opt/ai_worker_sick && colcon build --packages-select ffw_bringup --symlink-install'
docker exec -it ai_worker bash
source /opt/ai_worker_sick/install/setup.bash
```

Source this overlay in each shell used to launch the new driver. Container
recreation removes this manual installation; rebuild the Docker image to
include it permanently. For a native ROS workspace, build `ffw_bringup` from
this checkout and source its `install/setup.bash` instead.

AI Worker uses `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and `ROS_DOMAIN_ID=30`.
If the Zenoh router is not already running, start it in a separate terminal:

```bash
docker exec -it ai_worker bash -ic 'ros2 run rmw_zenoh_cpp rmw_zenohd'
```

Start just the two sensors:

```bash
ros2 launch ffw_bringup lidar_dual.launch.py lidar_driver:=sick_picoscan
```

| Position | Sensor IP | LaserScan topic | LaserScan frame | UDP ports |
| --- | --- | --- | --- | --- |
| Left | `192.168.6.3` | `/scan_left` | `lidar_l_link_1` | 2115 (scan), 2116 (check) |
| Right | `192.168.6.4` | `/scan_right` | `lidar_r_link_1` | 2117 (scan), 2118 (check) |

The receiving host IP is selected from the route to each sensor. To specify it,
add `udp_receiver_ip:=<host-IP>`. Use `sensorip0` / `sensorip1` to change the left /
right addresses and `sick_udp_port0` / `sick_udp_port1` to change ports. Each scan
port reserves the next port for the driver's check, so the ports must be at
least two apart. Docker requires host networking, as in `docker/docker-compose.yml`.

The default SICK configuration is `sick_picoscan.launch` (picoScan150).
For picoScan120, add `sick_model:=picoscan120`. IMU streaming is disabled.
Point clouds are published separately on `/sick_left/cloud` and `/sick_right/cloud`.
The driver configures the scan destination and starts measurement on launch;
Ctrl+C stops the driver and scan streaming.

For robot integration, add `lidar_driver:=sick_picoscan` to the existing follower
or mobile-base bringup command, for example:

```bash
ros2 launch ffw_bringup ffw_mobile_base_ai.launch.py lidar_driver:=sick_picoscan
```

Use that command only when ready to start the robot's usual hardware bringup.
It includes the existing merger that combines `/scan_left` and `/scan_right`
into `/scan`. Use either the standalone sensor launch or the robot bringup,
so each sensor has only one driver. The default driver remains `lakibeam`.

The SICK launch adds identity transforms from `lidar_l_link` / `lidar_r_link` to
their `_1` scan-layer frames and disables the driver's default `map` transform.
The robot URDF must provide the actual mounting poses of both sensors; verify
these when replacing the original LiDARs. The standalone sensor launch does not
start the robot state publisher or scan merger. All consumers must use the same
ROS domain and middleware as the driver.

Check the output with:

```bash
ros2 topic hz /scan_left
ros2 topic hz /scan_right
ros2 topic echo /scan_left --once --field header
```

Validation on 2026-10-21 with the official Jazzy 3.9.0 binary and both sensors:
LaserScan and PointCloud2 reception and the scan-layer transforms were verified.
Both scans arrived at approximately 24 Hz; occasional UDP loss was observed on
the current Wi-Fi route. The robot mounting calibration and full navigation
pipeline were not tested.

Known limitation in the tested container: on shutdown, the driver acknowledges
`ScanDataEnable 0` but then aborts during ROS resource cleanup. This occurred
with both Fast DDS and Zenoh. Runtime reception works, but clean shutdown is
not yet verified; the upstream driver source has not been modified.
