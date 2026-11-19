# A13 leader

A13 combines the A1 arms with left and right A3 hands on one serial bus.
The original A1 and standalone A3 models are unchanged.

- Both joint-7 origins relative to joint 6: `(0, 0, -0.042)` m (42 mm downward).
- Joint-7 zero-pose frames remain aligned with joint 6 (`rpy="0 0 0"`);
  their rotation axes are `1 0 0`, making the downward hands swing sideways
  in the YZ plane rather than twist around the finger direction.
- Left A3 base relative to left joint 7: `(-0.0148853, -0.00957761, -0.025)` m.
- Right A3 base relative to right joint 7: `(-0.0148853, +0.00957761, -0.025)` m.
- The complete mount offset and orientation are rotated so previous +X becomes -Z.
  Mount RPY is `(180°, 0°, -90°)` left and `(180°, 0°, +90°)` right.
  At zero joint angles, all four non-thumb finger shafts hang downward (-Z),
  with palms facing each other (left toward -Y, right toward +Y).
- No triggers, joystick sensors, or vibration motors are included.
- One hardware system reads 54 positions at 2 Mbaud on `/dev/serial/by-role/leader_a1`.
  The default update rate is 60 Hz. All joints are torque-disabled.
- Arm IDs are 31–37 (left) and 1–7 (right), retaining A1 drive modes and homing offsets.
- Finger IDs are 141–164 (left) and 111–134 (right), skipping every fifth vibration-motor ID.
- Both hands explicitly use zero-offset encoder conversion.

After building and sourcing the workspace:

```bash
# Model preview only (no serial connection)
ros2 launch ffw_description ffw_a13.launch.py

# Complete bringup with mock hardware
ros2 launch ffw_bringup ffw_a13_leader_ai.launch.py use_mock_hardware:=true start_rviz:=true

# Hardware bringup; uses the same serial role as A1
ros2 launch ffw_bringup ffw_a13_leader_ai.launch.py start_rviz:=true
```

The A13 port and baud rate are fixed in its ros2_control Xacro.
The launch accepts an `update_rate` override; pedal connection settings remain independent.

The hardware launch uses the `leader` namespace by default (`namespace:=...` overrides it).
The following topics carry `sensor_msgs/msg/JointState`, in radians:

- `/leader/arm_l_state_broadcaster/joint_states`: `arm_l_joint1` through `arm_l_joint7`.
- `/leader/arm_r_state_broadcaster/joint_states`: `arm_r_joint1` through `arm_r_joint7`.
- `/leader/hand_l_state_broadcaster/joint_states`: `finger_l_joint1` through `finger_l_joint20`.
- `/leader/hand_r_state_broadcaster/joint_states`: `finger_r_joint1` through `finger_r_joint20`.
- `/leader/joint_states`: all 54 measured joints, plus the unencoded base-height `joint` at zero for TF.

Connect the separate hand retargeting node to the two hand state topics. Match positions
by joint name; no `a3_` joint prefix is added. The retargeter owns the follower trajectories:
`/leader/joint_trajectory_command_broadcaster_left_hand/joint_trajectory` and
`/leader/joint_trajectory_command_broadcaster_right_hand/joint_trajectory` (SH5 inputs).
This bringup does not publish arm or hand command trajectories or launch teleoperation.

RViz displays the existing mesh-free link frames. TF frame names have the `leader_` prefix;
the root is `leader_world`. The description-only preview exposes the manual base-height
joint in its GUI; the hardware launch assumes zero height.

Validation (inside a sourced ROS workspace):

```bash
python3 src/ai_worker/ffw_description/test/test_a13_description.py
python3 src/ai_worker/ffw_description/test/test_a13_mock_bringup.py
```

The runtime check uses mock hardware on ROS domain 191 and shuts down its own launch.

## Hardware timing check — 2026-11-18

Tested all 54 devices on `/dev/ttyACM0` at 2 Mbps in `ai_worker_a3_leader`.
The selected default is **60 Hz**, the highest useful rate in the 5 Hz sweep below.
Use `update_rate:=50` for additional timing margin under heavier host load.

- **50 Hz:** 40-second measurement sustained 50.00 hardware reads/s. Read duration
  averaged 15.376 ms, maximum 15.746 ms, leaving 4.254 ms of the 20 ms cycle.
  Both 7-joint arm topics and both 20-joint hand topics remained at 50 Hz with finite
  positions. All controllers and hardware were active, with no runtime cycle warnings.
- **60 Hz:** 20-second measurement sustained 60.00 hardware reads/s. Read duration
  averaged 15.400 ms, maximum 15.780 ms, leaving only 0.887 ms of its cycle.
- **100 Hz:** packet receive errors and hardware/controller deactivation; the 10 ms
  timeout is shorter than the full bus read. Topic publication rate alone is misleading.

Startup Fast Bulk Read probing failed; the driver automatically switched to normal
Bulk Read and then operated successfully at 50/60 Hz. The failed probe produced an
initial communication-error log entry in each measured run; this was not a steady-state
failure at those rates. The driver's read path uses the controller period as its packet
timeout, explaining the 100 Hz failure.

No follower commands were sent. Tests ended with their hardware launch stopped.
Raw logs and JSON measurements are in
`docker/workspace/a3_leader/a13_hardware_validation_20261118/` at the repository root.

### Maximum-rate sweep (5 Hz steps)

A follow-up hardware sweep tested 60, 65, and 70 Hz. The default was then set to 60 Hz.

- **60 Hz**, 25 seconds: 60.00 actual read attempts/s, all successful; zero skipped
  async triggers and zero read failures. Worst read duration was 15.819 ms.
- **65 Hz**, 25 seconds: remained active with zero read failures, but skipped 728
  async triggers and achieved only about 35.90 actual reads/s. Publishing joint states
  at 65 Hz does not make those repeated samples fresh.
- **70 Hz**: repeated receive errors caused hardware and controller deactivation;
  the test stopped after joint-state publication ceased.

Thus **60 Hz is the highest useful frequency tested in 5 Hz increments**. 65 Hz can
stay running, but delivers fewer fresh readings. These are bounded measurements on
this host/bus, not a long-duration reliability guarantee. Test logs and JSON summaries
are in `docker/workspace/a3_leader/a13_hardware_validation_20261118/max_rate/`.

```bash
ros2 launch ffw_bringup ffw_a13_leader_ai.launch.py update_rate:=60 start_rviz:=true
```
