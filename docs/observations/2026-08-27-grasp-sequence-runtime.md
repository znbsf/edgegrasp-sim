# Four-stage typed sequence runtime observation

Observed from 2026-08-27 03:26:59 to 03:40:24 Asia/Shanghai in the isolated
Ubuntu 24.04 WSL2 / ROS 2 Jazzy / Gazebo 8.11 DART workspace.

## Result

`grasp-sequence-runtime-0331` completed the correlated sequence:

```text
APPROACH_PLAN -> APPROACH_EXEC
DESCEND_PLAN  -> DESCEND_EXEC
CLOSE_GRIPPER_EXEC
LIFT_PLAN     -> LIFT_EXEC
COMPLETE
```

The controller log reported three successful arm FJT goals and one successful
gripper FJT goal. The final GraspSequence result was wrapper status `4`,
`sequence_completed=true`, terminal phase `COMPLETE`, and last correlated
command `grasp-sequence-runtime-0331|lift|3`.

This is sequence evidence, not grasp-physics evidence. The task used poses
below the table as a conservative integration smoke test. No cube pose/contact
observer was attached, the optional cube was not loaded into MoveIt, and the
result correctly remained `physics_grasp_verified=false`.

## Runtime fix validated

The first implementation asked the adapter to snapshot the current
end-effector orientation independently for approach, descend, and lift. Small
controller/model drift therefore changed the IK constraint between immutable
stage positions. The action now carries one explicit normalized
`arm_orientation`; all three arm goals copy that exact quaternion. A zero or
non-normalized quaternion is rejected before planning. The WSL package test
collected and passed 22/22 tests after this interface change.

## Fail-closed repeat

A later task, `grasp-sequence-runtime-0333`, completed approach and descend but
detected `joint_state_stale` while the gripper goal was active. It requested
gripper cancellation, entered `SAFE_STOP`, and issued no lift. This validates
the runtime fail-closed branch but also means repeat-run reliability and the
requested 10-run consistency metric remain unverified.

## Evidence boundary

- MoveIt planned only; every trajectory went through typed
  `/edgegrasp/execute_trajectory` before the upstream FJT action.
- The three upstream controllers were active and the robot model remained at
  world pose zero.
- The current Gazebo server log contains zero prior DART mesh-construction and
  geometry-creation diagnostics.
- Only the separately recorded named base-proxy contact is behaviorally
  verified; full-link collision fidelity is not.
- Shutdown still exposed upstream MoveGroup SIGINT segmentation and Gazebo
  signal exit behavior. The project-owned planning-scene double-shutdown path
  was observed and fixed. The final package-scoped Jazzy regression passed
  74/74 tests, including 22/22 warning-free sequence-package tests.

Machine-readable details and exact IDs are in
`2026-08-27-grasp-sequence-runtime.json`.
