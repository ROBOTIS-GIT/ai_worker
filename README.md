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

## Dual LiDAR (SICK picoScan / Lakibeam)

Sensor drivers have separate launch files; both publish `/scan_left` and
`/scan_right` for the existing `dual_laser_merger`.

| Launch file | Role |
| --- | --- |
| `lidar_sick_dual.launch.py` | Two SICK picoScan drivers and their scan-layer TFs |
| `lidar_lakibeam_dual.launch.py` | Two Lakibeam drivers (requires `lakibeam1`) |
| `lidar_dual_merged.launch.py` | Directly selects one sensor launch, adds mounting TFs and the merger |

The merged launch accepts `lidar_driver:=lakibeam` (default) or
`lidar_driver:=sick_picoscan`. Shared arguments are `sensorip0` / `sensorip1`,
`output_topic0` / `output_topic1`, and `frame_id0` / `frame_id1` (left / right).
Driver-specific arguments are declared in the respective sensor launch and can
also be passed through the merged launch. `frame_id0` / `frame_id1` replace the
previous SICK-only `sick_frame_id0` / `sick_frame_id1` arguments.

### SICK driver installation

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

For a standalone LiDAR run with a merged `/scan` topic, start the sensors,
mounting transforms, and scan merger together:

```bash
ros2 launch ffw_bringup lidar_dual_merged.launch.py lidar_driver:=sick_picoscan
```

This publishes `/scan` (`sensor_msgs/msg/LaserScan`) and `/scan_cloud` in
`base_link`, in addition to the two input scans. It does not start controllers,
motors, or navigation. `dual_laser_merger` is included in the AI Worker image;
native installations can use `ros-jazzy-dual-laser-merger`.

The same standalone pipeline supports Lakibeam when `lakibeam1` is installed:

```bash
ros2 launch ffw_bringup lidar_dual_merged.launch.py lidar_driver:=lakibeam
```

`lidar_dual_merged.launch.py` replaces `lidar_sick_merged.launch.py`; launch only
one pipeline at a time. All merger arguments and defaults are declared in this
common launch, for both standalone and full hardware bringups. `merged_scan_time`
defaults to 0.04 s for SICK (25 Hz), or `1 / scanfreq` for Lakibeam (30 Hz by
default), and can be overridden. It sets the output `LaserScan.scan_time` field;
it does not limit the publishing rate. The average and shadow filters default to
disabled; `enable_average_filter` and `enable_shadow_filter` control them.

The default mounting poses match the mobile-base URDF (also used by SG2/F2/SH5):
left `-0.111881 0.240 0.34432 0 0 1.57079632679`, right
`-0.111881 -0.240 0.34432 0 0 -1.57079632679` (x y z roll pitch yaw, m/rad).
Override `left_pose` and `right_pose` for different mounts. If the robot state
publisher already supplies the mounting transforms, use `publish_mount_tf:=false`.
If the two sensor drivers are already running, use `launch_sensors:=false`.
Do not run this merger alongside the merger in the robot's full bringup.

To start only the two sensors, without a merged `/scan` topic, run the sensor
launch directly. For SICK:

```bash
ros2 launch ffw_bringup lidar_sick_dual.launch.py
```

For Lakibeam:

```bash
ros2 launch ffw_bringup lidar_lakibeam_dual.launch.py
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
The mobile-base, F2, SG2, and SH5 hardware bringups include `lidar_dual_merged.launch.py`,
which starts the selected sensor drivers and one merger together. They retain
the 20-second delay with `init_position:=true`, or 10 seconds with
`init_position:=false`. `launch_lidar:=false` disables both drivers and the merger.
The robot's `robot_state_publisher` supplies mounting TFs, so these bringups pass
`publish_mount_tf:=false` and forward `use_sim` as `use_sim_time`.

The hardware bringups only enable or disable the LiDAR pipeline with
`launch_lidar`; they do not declare or override merger settings. Sensor selection,
scan timing, and filter settings belong to the LiDAR launches. Thus full hardware
bringups now use the same sensor-based scan time and disabled filters as the
standalone pipeline, replacing the previous 0.1 s / enabled-filter defaults.
Gazebo-specific launches continue to merge simulated scans without hardware drivers.
Use either the standalone pipeline or the robot bringup, so each sensor has only
one driver and `/scan` has one merger. The default driver remains `lakibeam`.

The SICK launch adds identity transforms from `lidar_l_link` / `lidar_r_link` to
their `_1` scan-layer frames and disables the driver's default `map` transform.
The mounting transforms must match the actual poses of both sensors. The
sensor-only launch does not start a merger or publish mounting transforms;
use `lidar_dual_merged.launch.py lidar_driver:=sick_picoscan` for the complete
standalone pipeline. Both standalone and full hardware bringups use
`lidar_dual_merged.launch.py`, which directly includes either
`lidar_sick_dual.launch.py` or `lidar_lakibeam_dual.launch.py` and starts the
existing `dual_laser_merger` implementation.
All consumers must use the same ROS domain and middleware as the driver.

Check the output with:

```bash
ros2 topic hz /scan_left
ros2 topic hz /scan_right
ros2 topic hz /scan
ros2 topic echo /scan_left --once --field header
ros2 topic echo /scan --once --field header
```

Validation on 2026-10-21 with the official Jazzy 3.9.0 binary and both sensors:
LaserScan and PointCloud2 reception and the scan-layer transforms were verified.
Both scans arrived at approximately 23–24 Hz; occasional UDP loss was observed
on the current Wi-Fi route. After splitting the sensor launches, the common
merged launch was verified using Zenoh on domain 30: 411 merged scans in
20 seconds including startup, frame `base_link`, 960 rays, both input clouds
included in `/scan_cloud`, and exactly one publisher per scan topic. Launch
selection, shared arguments, driver-specific overrides, and merger-only mode
were also checked without starting sensor processes. All four hardware bringup
launch graphs were checked with process execution disabled: the 10/20-second
timer settings, one merger, no duplicate mounting TF publishers, forwarding of
simulation time, and disabling the complete pipeline with `launch_lidar:=false`.
Lakibeam hardware reception
was not tested because `lakibeam1` is absent from the current container. The robot
mounting calibration and full navigation pipeline were not tested.

Known limitation in the tested container: on shutdown, the driver acknowledges
`ScanDataEnable 0` but then aborts during ROS resource cleanup. This occurred
with both Fast DDS and Zenoh. Runtime reception works, but clean shutdown is
not yet verified; the upstream driver source has not been modified.
