# Pilz collision validation and contact trial

Observation window: 2026-08-27 15:14–15:40 Asia/Shanghai.

The new result is **collision rejection verified, control sequence verified,
physics grasp false**.

The unique EdgeGrasp MoveGroup overlay now injects Pilz
`default_planning_response_adapters/ValidateSolution` while keeping direct
MoveGroup execution disabled. The previously dangerous mid-approach candidate
was recomputed but rejected: indices 51–71 of an 84-state candidate collided
between `edgegrasp_target_cube` and `gripper_link`. MoveIt returned
`INVALID_MOTION_PLAN` / error `99999`, and the read-only probe executed no
trajectory. The robot and cube stayed at their initial poses.

A farther approach passed the same validator with the gripper both closed and
open. Its first execution attempt was canceled fail-closed after a transient
`safety_signal_stale`; downstream cancellation was observed and the cube did
not move. A single bounded retry completed through
`PlanTarget → ExecuteTrajectory → FollowJointTrajectory`, with action status
`SUCCEEDED`, FJT error code `0`, and no cube displacement.

Before intentional contact, only `edgegrasp_target_cube` was removed from the
MoveIt PlanningScene. `edgegrasp_table` remained required and confirmed. Both
descend and lift then passed Pilz `ValidateSolution` in read-only probes.

Task `collision-validated-physics-039` completed all four correlated stages:

- `APPROACH`
- `DESCEND`
- `CLOSE_GRIPPER`
- `LIFT`

Every stage reached a matching typed terminal and downstream FJT success. The
independent physics observer saw 1860 gripper/cube contacts, but the matched
robot entity was `gripper_link`, not a distal pad. Cube Z rose from
`0.204999999902 m` to only `0.207267687902 m` (about 2.27 mm), below the 20 mm
gate, then returned to table height. Its final XY position was about 60 mm from
the initial position. No retention window was reached, so
`physics_grasp_verified=false`.

The current Gazebo server log contains zero tracked DART mesh-construction
diagnostics, the robot stayed world-anchored, and all three controllers remained
active. After cleanup there was no matching project process and `/clock` had no
publisher. MoveGroup still exited `-11` on SIGINT, Gazebo reported `-2`, and the
interface probe hit a teardown-time rclpy conversion exception; those shutdown
defects are recorded separately from runtime motion evidence.

Machine-readable commands, poses, digests, log paths, collision entities, and
cleanup facts are in `2026-08-27-pilz-collision-contact-runtime.json`.
