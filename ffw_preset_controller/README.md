# Hand preset controller

This node maps raw LG2 or A2 leader gripper positions to YAML-defined release
and grasp poses for SH5/BH5 hands. Each hand command contains 20 joint positions.

## Bringup

With the follower hand controllers running, enable presets in either leader launch:

```bash
ros2 launch ffw_bringup ffw_lg2_leader_ai.launch.py use_hand_preset:=true
ros2 launch ffw_bringup ffw_a2_leader_ai.launch.py use_hand_preset:=true
```

Use one leader launch. `use_hand_preset` defaults to `false`, preserving normal
gripper operation. When enabled, the leader arm trajectory broadcaster omits
gripper joints while `/leader/joint_states` retains the raw gripper positions
used by this node. A2 keeps the common leader pose-reference generation and
follower-side teleoperation architecture; it does not start another teleoperation
controller on the leader.

The default preset file is `ffw_bringup/config/common/hand_presets.yaml`.
To use a custom file and initial selection, append:

```bash
hand_presets_file:=/absolute/path/to/my_hand_presets.yaml hand_initial_preset:=pinch
```

Leader-specific gripper calibration and topics live in
`ffw_bringup/config/ffw_lg2_leader/ffw_lg2_hand_preset_controller.yaml` and
`ffw_bringup/config/ffw_a2_leader/ffw_a2_hand_preset_controller.yaml`.
The preset file is shared between leaders. The node publishes to the existing
SH5/BH5 left/right hand trajectory topics under `/leader`.

## Editing presets

Each named entry under `presets` defines `release` and `grasp` mappings with
`thumb`, `index`, `middle`, `ring`, and `little`, each containing four joint
positions in radians. Values use the right-hand convention. `mirrored_joints`
contains one-based joint indices to negate for the left hand; optional
`left_release` and `left_grasp` mappings override that mirroring.
`default_preset` selects the initial pose unless overridden at launch.

The node interpolates between release and grasp, clamps the input ratio to
0–1, and blends preset changes over `preset_switch_duration`. It pauses a hand
when its leader gripper state becomes stale. With `clamp_to_joint_limits` enabled,
it applies limits received on `robot_description_topic`; before receiving those
limits it warns and publishes unclamped commands.

Select or reload presets on a running system. The YAML file is the preset catalog;
the controller does not define a second custom discovery service. Each preset has
a required unique `id` in 1–65535 used with the shared
`robotis_interfaces/srv/SetPreset` API. The shipped IDs are 1 (`envelope`),
2 (`pinch`), and 3 (`3finger`). Keep IDs stable when reordering the YAML.

```bash
ros2 service call /hand_preset_controller/set_preset robotis_interfaces/srv/SetPreset "{target_arm: both, left_preset_id: 2, right_preset_id: 2}"
ros2 service call /hand_preset_controller/reload_presets std_srvs/srv/Trigger '{}'
```

`target_arm` accepts `left`, `right`, or `both`; only selected arms' preset IDs
are used, and each hand can select a different preset. Invalid requests leave both
hands unchanged. The response's `accepted` acknowledges selection;
`transition_id` increments for each accepted request within the node's lifetime.
It does not report completion of the configured blend.
The hand service is namespaced separately from Cyclo's `/set_preset` arm service.

Invalid reloads retain the previous presets. Editing the source default requires
rebuilding bringup unless the install
is symlinked; an absolute custom file can be edited and reloaded directly.

The standalone launch requires an explicit preset path, keeping this package
independent of bringup:

```bash
ros2 launch ffw_preset_controller hand_preset_controller.launch.py \
  presets_file:=/absolute/path/to/my_hand_presets.yaml \
  params_file:=/absolute/path/to/leader_parameters.yaml
```
