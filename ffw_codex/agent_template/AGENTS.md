# Robot workspace

You control a physical ROBOTIS AI Worker SG2. The robot moves only through the 'robot_*' tools. A host process owns the robot connection, checks every action, and records the episode.

Read `context/robot_contract.md` before your first action. It has the frame, units, limits and what each result means.

## Episode

- One episode, one task. The task is in the first message and in the `robot_start` packet.
- Call `robot_start` once. It returns the first observation.
- The physical scene can change while you think. A previously issued command may remain active between tool calls. Refresh the observation before acting on an outdated scene.
- When a new observation shows that the task is done, call `task_complete`. That ends the episode. There is no way to give up.

## Robot tools

| Tool | Arguments | Effect |
|---|---|---|
| `robot_start` | none | First observation. Only once per episode |
| `robot_observe` | none | New observation. Sends no motion command |
| `robot_move` | `request_id`, `reason`, `arm`, `x`, `y`, `z`, `roll`, `pitch`, `yaw` | Move one hand to an absolute pose |
| `robot_gripper` | `request_id`, `reason`, `arm`, `value` | Open (0) or close (1) one gripper |
| `robot_head` | `request_id`, `reason`, `head_joint1`, `head_joint2` | Point the head camera |
| `robot_lift` | `request_id`, `reason`, `position_m` | Raise or lower the body |
| `robot_base` | `request_id`, `reason`, `vx`, `vy`, `wz`, `duration_s` | Drive the mobile base for a short time, then stop |
| `robot_execute_plan` | `request_id`, `reason`, `steps` | Execute a short sequence of actions in one call; steps run sequentially |
| `task_complete` | `request_id`, `reason` | Declare the task done. Ends the episode |

- Call them inside `exec` as `tools.<name>(arguments)`. Action calls wait for completion detection and an attempt to capture a new observation before returning.
- The result is one string. Line 1 is a JSON packet. If `packet.observation` is not null, each following line is a camera image as a `data:` URL, in the order head, wrist_left, wrist_right. Pass each `data:` line to `image()` to see it.
- `action_executed` in the packet says whether a robot command was sent. When it is false, `status`, `rejection` and `reason` say why.
- Every action needs `request_id` from the latest packet (`action_context.request_id`, also in `next_call`) and a `reason`.
- Run one robot tool at a time.
- After every base command, inspect the new images and `result.motion.odom_measured` before choosing the next action. If the response has no observation, call `robot_observe`.

## Action plans

- `robot_execute_plan` runs several `robot_move`, `robot_gripper`, `robot_head` and `robot_lift` steps in one call. Each step takes the same arguments as the single tool, without `request_id`. The host checks every step as it would a separate call, but nobody looks at the intermediate images.
- Use a plan when the intermediate images would not change the next steps. Examples: raising or retracting a hand through space you have already seen to be clear, withdrawing a held object back along the path the hand came in, moving back along a path you just came.
- Do not use a plan where the next step depends on what you see: aligning with an object, the final approach, checking a grasp or a release. End the plan before that point and look at the new images.
- Each step obeys the same limits as a single call. A hand step is measured from where the previous step ended. `robot_head`, `robot_lift` and every `robot_gripper` command may only be the last step. Opening a gripper can release an object, so inspect the resulting images before any retraction.
- The plan stops at the first step that does not end `arrived` with a valid observation. `result.steps` lists what ran, `result.remaining_steps` what did not, and `result.stop_reason` says why. A completed plan is not a completed task; inspect the final images.
- `robot_base` cannot be part of a plan. Call it separately.

## Files

Your working directory is `agent/` of this run. `workspace.json` has the absolute paths.

| Path | Access | Contents |
|---|---|---|
| `AGENTS.md`, `context/`, `workspace.json` | read | These instructions, the robot contract, paths |
| `notes/NOTES.md` | write | Your notes for this episode |
| `scratch/` | write | Your scripts, crops, computed images |
| `../public/observations/NNN/` | read | `head.jpg`, `wrist_left.jpg`, `wrist_right.jpg`, `observation.json` of observation NNN |
| `../public/history.jsonl` | read | Every packet of this episode, without images |

- Shell commands run without network access. Use `python3`; `numpy` and `cv2` (OpenCV) are installed.
- To look closely, save a crop or a marked-up image in `scratch/` and open it with `view_image`.
- Nothing else on the machine is readable. When something is blocked, do not look for a way around it.

## Do not

- Reach the robot, ROS or the simulator in any way other than the `robot_*` tools.
- Edit files outside `notes/` and `scratch/`.
