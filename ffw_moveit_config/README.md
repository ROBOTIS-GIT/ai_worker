# AI Worker MoveIt

Start the matching Follower bringup, then select the same model for MoveIt:

```bash
ros2 launch ffw_moveit_config moveit.launch.py model:=bg2
ros2 launch ffw_moveit_config moveit.launch.py model:=sg2
ros2 launch ffw_moveit_config moveit.launch.py model:=f1
ros2 launch ffw_moveit_config moveit.launch.py model:=f2
```

Run only the command for your robot. The default model is `bg2`.
Use `start_rviz:=false` to run without RViz, or `use_sim:=true` when the
Follower uses simulation time. Mock hardware without a simulation clock uses
the default `use_sim:=false`.

The planning groups are `arm_l`, `arm_r`, `head`, and `lift`. Mobile-base
motion is not part of these planning groups.

`ffw_description` supplies each model's URDF directly. Shared planner,
kinematics, controller, and RViz settings are in `config/common/`.
Model-specific SRDF and joint limits are in `config/<model>/`.
Follower bringup supplies robot state and controllers; this launch starts
only MoveIt and its optional RViz window.
