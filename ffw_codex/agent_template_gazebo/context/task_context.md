# Task rules

## Goal

Do the task in the first message as written. When a new observation shows it is done, call `task_complete` with that observation's `request_id` and visible evidence as `reason`. A person checks every claim; unsupported claims fail. Do not add conditions to the task.

## Evidence

- Base conclusions on images and measured state.
- `arrived` means the hand reached its pose, not that it touched anything. A closed gripper does not prove a grasp. Check contact and grasp in a later observation.
- If evidence is unclear, keep the conclusion uncertain. Drop an earlier conclusion when a later observation contradicts it.
- Object positions, sizes, and table height are not provided. Estimate them from images and measured poses, and note how.

## Acting

- Choose the method yourself. Give every action a `reason` containing the visible evidence and purpose.
- Analyze before acting: crop, compute, and compare observations. If an observation expires, call `robot_observe`, inspect the scene again, and decide again.
- A rejected action did not run and is not retried automatically.
- The pre-action check sees only robot state, not object motion. The simulation keeps running while you think.
- An unconfirmed command outcome ends the episode. A hand or gripper that stopped short (`stopped`) did run; inspect the new observation.
- Before grasping, placing, or moving the base, check the relevant alignment, destination, or path from more than one viewpoint. Move the head or an arm and inspect the new images when needed.

## Reach before grasping or placing

- Before reaching to grasp or place, position the base and adjust the lift so that the wrist remains comfortably inside its reach from the shoulder. Account for hand orientation when deriving the wrist position. A top-down hand usually requires the base to be closer, or the lift lower, than a forward-pointing hand for the same EE target.
- If a move stops short, first use the images, position and orientation errors, and the wrist-to-shoulder vector to distinguish a reach or posture limit from contact, an obstacle, or self-collision.
- When the wrist is near full extension and no obstruction explains the stop, do not extend farther. Retract through known-clear space to the base-safe posture. Reduce mainly horizontal separation with the base and mainly vertical separation with the lift, then inspect a fresh observation and retry. Check hand, held-object, and surrounding clearance before moving the lift.

## Clearance

- Before approaching, find what bounds the space around the object: the support below it and anything above, behind or beside it. In the reason of the first approach action, state which sides are open and which are blocked.
- Approach from an open side. The whole hand must fit along the path: the fingers and the camera body at its offset.
- Clearance is the gap between an obstacle and the nearest part of the hand, the camera body, or the held object, not the gap above the object alone.
- After closing, choose the smallest motion that frees the object from its support. If that needs a rise, keep it within the clearance above.
- The reverse of the approach is the default way out, but the hand entered empty and leaves with an object. Before withdrawing, check that the hand, the camera body and the held object together fit along that path; if they do not, choose another clear direction.
- A grasp check does not need a lift. The object leaving its place and moving with the hand in any direction is the evidence.

## Grasping

- Before the final approach, assess grasp position and orientation. State the orientation choice and visible basis in the action reason.
- For elongated objects, make the finger closing direction cross the long axis. Being between the fingers alone does not establish alignment.
- To preserve orientation, reuse that arm's last issued move target rpy (`result.target`), not measured drift. Before its first move, use measured rpy. In a plan, use the previous target rpy. Do not reset angles to zero accidentally.
- After a stopped move or orientation error over 5 degrees, assess images and both errors before the final approach. Do not repeat a blocked target unchanged; reposition or deliberately choose a reachable orientation in clear space.
- After wrist rotation, re-estimate from the new wrist image. Do not reuse old pixel mappings or equate image angle with base_link yaw.
- If closing pushes or rotates the object, reassess before repeating the approach.
- Before closing, check that the part of the object you want to hold is at the finger contact area, in both height and depth. The contact area is behind the reported EE point; see "Gripper geometry" in the contract.
- Choose the grasp point before the final approach. Prefer a stable part of the object's body. Choose a cap, a neck or a thin edge only when the images show it will hold. In the reason of the closing action, name the part the fingers are on and the evidence for it.

## Placing

- Opening a gripper while holding an object is irreversible. Immediately before opening, verify in the latest observation and from at least two useful views that the intended object remains held, the intended support is directly below it, and the object is low enough to be supported rather than dropped. Coordinates, `arrived`, or one ambiguous view are insufficient.
- If the object, support gap, destination boundary, or obstacles are unclear or occluded, do not open. Keep holding; change the head or arm view, inspect the new images, and realign or descend. Time pressure and a plausible pose estimate do not justify release.
- After opening, inspect a new observation before retracting. Confirm the object is supported at the destination and no longer moves with the fingers. Otherwise do not claim completion.

## Keep going

- Continue until done. After a failed grasp, rejection, or low odds, change the approach and retry until the budget ends. Ending your turn does not end the episode.
- `progress.actions_remaining` and `progress.seconds_remaining` show the budget. Rejections and observations do not use actions.

## Notes

Keep `notes/NOTES.md` current with estimates, evidence, attempts, results, and next changes. Earlier packets are in `../public/history.jsonl`.
