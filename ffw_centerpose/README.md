# ffw_centerpose

CenterPose-based pick-and-place for the AI Worker (bottle / box / shoe). This package takes detections from Isaac ROS CenterPose, converts them into grasp poses, and drives the left arm via MoveL.

This setup spans three machines/containers: the AI Worker (robot), and an Isaac ROS container + a worker container on a separate GPU user PC, bridged over the network with Zenoh RMW.

## Package Contents

- `centerpose_camera.py` — relays the ZED's raw image/CameraInfo under a fixed topic name that Isaac ROS CenterPose consumes, converting BGRA8/RGBA8/RGB8 to BGR8 if needed.
- `centerpose_pointcloud.py` — crops the point cloud to the robot's reachable area, fits the table plane, and publishes the bbox markers used to visualize each detection.
- `centerpose_bottle.py` / `centerpose_box.py` / `centerpose_shoe.py` — one pick-and-place node per object type. Each subscribes to CenterPose detections, converts the detection into a grasp pose (position via depth sampling + calibrated offset, orientation via the object's yaw), and exposes `~/capture`, `~/execute`, `~/cancel` services to run the pick-and-place sequence. All three share the common motion/TF/quaternion logic in `pick_place_base.py` and differ mainly in their calibration YAML and place sequence.
- `pick_place_base.py` — shared base class + `load_camera_topics()` helper used by all of the above.

## Environment Setup

### Isaac ROS CenterPose

Follow NVIDIA's [Isaac ROS CenterPose quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_centerpose/index.html) to install the Isaac ROS dev container and download the Triton model repository.

Pretrained CenterPose models for additional object categories can be downloaded from NVIDIA's [CenterPose model catalog](https://catalog.ngc.nvidia.com/orgs/nvidia/tao/models/centerpose/deployable_dla34) — place them under `isaac_ros_assets/models/triton/` in your Isaac ROS workspace, which is mounted inside the container as `/workspaces/isaac_ros-dev`.

### Zenoh RMW Bridge (User PC ↔ AI Worker)

Because the Isaac ROS container and the AI Worker are on different machines, this setup uses Zenoh as the RMW implementation instead of the default DDS, with the user PC acting as a Zenoh client connecting to the AI Worker's Zenoh router.

Install the Zenoh RMW package (it also bundles the `zenohd` router) inside the container/workspace on the user PC side:

```bash
sudo apt install ros-$ROS_DISTRO-rmw-zenoh-cpp
```

Then export the following before launching *any* ROS 2 node that needs to talk to the AI Worker (Isaac ROS container terminal **and** worker container terminal):

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_DOMAIN_ID=30
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=false;mode="client";connect/endpoints=["tcp/robot_ip:7447"]'
```

- `ROS_DOMAIN_ID=30` must match on both the AI Worker and the user PC.
- Replace `robot_ip` with your AI Worker's actual address.

> For a stable camera feed, connect the AI Worker's rear USB 3.0 port to the user PC with a wired cable.

## Start Guide

### Step 1: Bring Up the AI Worker

> Before bringup, set the ZED camera's `depth.depth_mode` to `'ULTRA'` in `ffw_bringup/config/common/common_stereo.yaml`, and `general.grab_resolution` to `'HD720'` in `ffw_bringup/config/common/zedm.yaml`. CenterPose's depth sampling needs both to get reliable detections.

```bash
# AI Worker terminal 1
zenohd
```

```bash
# AI Worker terminal 2
ros2 launch ffw_bringup ffw_sg2_follower_ai.launch.py
```

### Step 2: Launch CenterPose Detection (Isaac ROS Container)

Make sure the Zenoh env vars above are exported in this terminal, then pick the launch matching the object you're picking.

```bash
# Bottle
ros2 launch isaac_ros_centerpose isaac_ros_centerpose_triton.launch.py \
  model_name:=centerpose_bottle \
  model_repository_paths:="['/workspaces/isaac_ros-dev/isaac_ros_assets/models/triton']" \
  input_image_width:=1280 input_image_height:=720 score_threshold:=0.3
```

```bash
# Box
ros2 launch isaac_ros_centerpose isaac_ros_centerpose_triton.launch.py \
  model_name:=centerpose_box \
  model_repository_paths:="['/workspaces/isaac_ros-dev/isaac_ros_assets/models/triton']" \
  input_image_width:=1280 input_image_height:=720 score_threshold:=0.3
```

```bash
# Shoe
ros2 launch isaac_ros_centerpose isaac_ros_centerpose_triton.launch.py \
  model_name:=centerpose_shoe \
  model_repository_paths:="['/workspaces/isaac_ros-dev/isaac_ros_assets/models/triton']" \
  input_image_width:=1280 input_image_height:=720 score_threshold:=0.3
```

### Step 3: Run the Worker Container

All of the commands below run in separate terminals inside the same `ffw_centerpose` workspace container on the user PC (the "worker container"). Terminals 1–3 are common to every object:

```bash
# Worker container — terminal 1
ros2 launch ffw_centerpose centerpose_initial_pose.launch.py
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py controller_type:=movel
```

```bash
# Worker container — terminal 2
ros2 run ffw_centerpose centerpose_camera
```

```bash
# Worker container — terminal 3
ros2 launch ffw_centerpose centerpose_pointcloud.launch.py
```

Then, in terminal 4, pick the commands matching the object from Step 2:

<details>
<summary>Bottle</summary>

```bash
# Worker container — terminal 4
ros2 launch ffw_centerpose centerpose_bottle.launch.py execute_motion:=true
```

```bash
# Worker container — terminal 5
ros2 service call /centerpose_bottle/capture std_srvs/srv/Trigger {}
ros2 service call /centerpose_bottle/execute std_srvs/srv/Trigger {}
ros2 service call /centerpose_bottle/cancel std_srvs/srv/Trigger {}
```

</details>

<details>
<summary>Box</summary>

```bash
# Worker container — terminal 4
ros2 launch ffw_centerpose centerpose_box.launch.py execute_motion:=true
```

```bash
# Worker container — terminal 5
ros2 service call /centerpose_box/capture std_srvs/srv/Trigger {}
ros2 service call /centerpose_box/execute std_srvs/srv/Trigger {}
ros2 service call /centerpose_box/cancel std_srvs/srv/Trigger {}
```

</details>

<details>
<summary>Shoe</summary>

```bash
# Worker container — terminal 4
ros2 launch ffw_centerpose centerpose_shoe.launch.py execute_motion:=true
```

```bash
# Worker container — terminal 5
ros2 service call /centerpose_shoe/capture std_srvs/srv/Trigger {}
ros2 service call /centerpose_shoe/execute std_srvs/srv/Trigger {}
ros2 service call /centerpose_shoe/cancel std_srvs/srv/Trigger {}
```

</details>

> `~/capture` takes a single snapshot of the object's position at that moment — it isn't continuous tracking, so make sure the object is visible and settled when you call it. It only succeeds if there is a fresh detection (within `detection_timeout`), a valid `CameraInfo`, and a resolvable camera→base TF. If it fails, check that `isaac_ros_centerpose` and `centerpose_camera` are both publishing, and that the Zenoh bridge is actually up (`ros2 topic hz /centerpose/detections` from the worker container).

### Why the Calibration Offsets

Each object node loads a per-object calibration YAML (`grasp_position_offset`, `*_depth_center_offset`, `fixed_grasp_z`, etc.). These aren't arbitrary tuning — the ZED's depth readout comes out tilted across the frame rather than flat/uniform, so raw CenterPose position + depth alone isn't reliable enough for a stable grasp. The offsets correct for that measured bias instead of trusting the raw depth value directly.

### Optional: Freeze the Table Plane

`freeze_table_plane` computes the table plane once and keeps republishing it, useful once the camera/table setup is fixed and you don't want per-frame RANSAC noise:

```bash
ros2 launch ffw_centerpose centerpose_pointcloud.launch.py freeze_table_plane:=true
```

Launch this with the table still empty, so the RANSAC fit only sees the bare table plane. Once it's frozen, place the object on the table — that keeps the captured point cloud clean instead of picking up the object itself as part of the table fit.
