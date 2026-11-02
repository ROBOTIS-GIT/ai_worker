# Task rules

## Goal

Do the task in the first message as written. When a new observation shows that it is done, call `task_complete` with that observation's `request_id` and the visible evidence as `reason`. The host records the final state, and a person checks every claim against it: a claim the evidence does not support counts as a failure. Do not add conditions to the task.

## Evidence

- Base each conclusion on observations: the images and the measured state.
- `arrived` means the hand reached the commanded pose. It does not mean the hand touched anything. A closed gripper does not mean a grasp. Check contact and grasp in the images of a later observation.
- If the evidence is unclear, keep the conclusion uncertain. When a later observation contradicts an earlier conclusion, drop the earlier conclusion.
- You are not given object positions, object sizes or the table height. Estimate them from the images and the measured poses, and write down how you estimated them.

## Acting

- Choose the method yourself.
- Give every action a `reason`: the visible evidence and the purpose.
- Analyze as much as you need before acting: crop, compute, compare observations. An observation is valid for `action_context.max_age_s` seconds. When it has expired, the action is rejected; call `robot_observe`, check the scene again and decide again.
- A rejected action did not run, and nothing is retried for you.
- Objects and the robot can move while you think. The pre-action check compares the robot's measured state; it cannot establish that objects in the scene stayed still.
- A command whose outcome cannot be confirmed, such as a hand still moving after the time limit, ends the episode. A hand or gripper that stopped short (`stopped`) did run; look at the new observation to see why.
- Before grasping, placing, or moving the base, check the object, finger alignment, destination, or path from more than one viewpoint. Moving the head or either arm changes its camera view; inspect the new images before deciding.

## Clearance

- Before approaching, find what bounds the space around the object: the support below it and anything above, behind or beside it. In the reason of the first approach action, state which sides are open and which are blocked.
- Approach from an open side. The whole hand must fit along the path: the fingers and the camera body at its offset.
- Clearance is the gap between an obstacle and the nearest part of the hand, the camera body, or the held object, not the gap above the object alone.
- After closing, choose the smallest motion that frees the object from its support. If that needs a rise, keep it within the clearance above.
- The reverse of the approach is the default way out, but the hand entered empty and leaves with an object. Before withdrawing, check that the hand, the camera body and the held object together fit along that path; if they do not, choose another clear direction.
- A grasp check does not need a lift. The object leaving its place and moving with the hand in any direction is the evidence.

## Grasping and placing

- Before reaching to grasp or place, position the base and adjust the lift so that the wrist remains comfortably inside its reach from the shoulder. Account for hand orientation when deriving the wrist position. A top-down hand usually requires the base to be closer, or the lift lower, than a forward-pointing hand for the same EE target.
- If a move stops short, first use the images, position and orientation errors, and the wrist-to-shoulder vector to distinguish a reach or posture limit from contact, an obstacle, or self-collision.
- When the wrist is near full extension and no obstruction explains the stop, do not extend farther. Retract through known-clear space to the base-safe posture. Reduce mainly horizontal separation with the base and mainly vertical separation with the lift, then inspect a fresh observation and retry. Check hand, held-object, and surrounding clearance before moving the lift.
- For grasping tasks, assess both position and orientation before the final approach. Include the orientation choice and its visible basis in the action reason, including when keeping the current orientation.
- For elongated objects, choose a grasp where the finger closing direction crosses the object's long axis. Being between the fingers alone does not establish orientation alignment.
- To preserve orientation while translating, reuse the last issued move target rpy for that arm (`result.target`), not measured drift. Before its first move, use the measured rpy as the initial reference. In a plan, use the previous target rpy for that arm. Do not reset the angles to zero unless deliberately choosing the zero orientation.
- If a move stops short or its orientation error exceeds the 5° arrival tolerance, assess the images and both position and orientation errors before the final approach. These errors alone do not identify contact. Do not repeat a blocked target unchanged; reposition or deliberately choose a reachable orientation in clear space.
- After rotating the wrist, re-estimate alignment from the new wrist image before the final approach. Do not reuse earlier pixel mappings unchanged or equate an image-space angle directly with base_link yaw.
- If closure visibly pushes or rotates the object, reassess alignment before repeating the same approach.
- Before closing, check that the part of the object you want to hold is at the finger contact area, in both height and depth. Contact or a moved object shows the hand reached the object, not that it is held where the fingers close. The contact area is behind the reported EE point; see "Gripper geometry" in the contract.
- Choose the grasp point before the final approach. Prefer a stable part of the object's body. Choose a cap, a neck or a thin edge only when the images show it will hold. In the reason of the closing action, name the part the fingers are on and the evidence for it.
- Opening a gripper while holding an object is an irreversible release. Immediately before opening, use the latest observation to verify from at least two useful camera views that the intended object is still held, the intended destination is the surface directly below it, and the object is low enough to be supported rather than dropped. Coordinates, an `arrived` result, or a destination visible in only one ambiguous view are not sufficient evidence.
- If any of the object, the gap to the support, the destination boundary, or intervening obstacles are occluded or visually ambiguous, do not open the gripper. Keep holding the object, move the head or the holding arm to obtain a clearer view, inspect the new images, and realign or descend as needed. Do not treat time pressure or a plausible pose estimate as permission to release.
- After opening, inspect a new observation before retracting. Confirm that the object is visibly supported at the intended destination and no longer moves with the fingers. If that evidence is unclear, do not claim completion.

## Keep going

- Continue until the task is done. A failed grasp, a rejection or low odds are not reasons to stop. Find the likely cause, change the approach and try again. There is no way to give up; if the task is not done, keep working until the budget runs out.
- Ending your turn does not end the episode. You will be asked to continue.
- `progress.actions_remaining` and `progress.seconds_remaining` show the budget. Rejections and observations do not use actions.

## Notes

Keep `notes/NOTES.md` current: your estimates and how you got them, what you tried, what happened, what you will change. `../public/history.jsonl` has every earlier packet if you need to look back.
