# Correlated grasp trial: sequence complete, physics false

Observation time: 2026-08-27 07:24:27–07:24:47 Asia/Shanghai.

This run is the first live end-to-end trial in this project where a collision-aware
MoveIt plan flowed through the EdgeGrasp adapter, typed trajectory gate and the
SO-101 FollowJointTrajectory controllers for all four sequence stages. It is **not**
a successful simulated grasp.

## Observed result

- Safe preparation opened the gripper and moved the arm to
  `[-0.05, 0.8, -1.4, -1.4, 0.0]` through `/edgegrasp/execute_trajectory`.
  Both wrapper actions reached `STATUS_SUCCEEDED` and both downstream FJT results
  returned error code `0`.
- Collision-aware `GetPositionIK` calls returned success (`error_code=1`) for the
  selected approach and descend poses. These probes sent no motion.
- The correlated sequence reached `APPROACH → DESCEND → CLOSE_GRIPPER → LIFT → COMPLETE`.
  Its final correlated command was `grasp-physics-trial-20260827-003|lift|3`.
- The independent physics observer saw 132 cube-pose samples, but zero configured
  gripper contacts. Cube Z stayed at `0.424999999902001 m`; lift and retention were
  false. Therefore `physics_grasp_verified=false`.
- SO-101 remained anchored at zero pose. The cube remained at
  `[0.2, 0.0, 0.425]`. The current Gazebo log contains zero copies of the tracked
  DART mesh-construction and geometry-creation diagnostics.

## Observer clock fault

The observer aborted with `clock_rollback`. `/clock` had exactly one publisher,
`/ros_gz_bridge`. Code review found a wrapper race: concurrent callbacks sampled
the ROS clock before taking the shared ordering lock, so an older sample could be
committed after a newer one. The implementation now serializes the read and
comparison, but this file deliberately records the pre-fix runtime result. A clean
runtime rerun is required before calling that fix runtime-verified.

## Evidence boundary

This run verifies the target-to-plan-to-gate-to-FJT sequence contract. It does not
verify gripper contact, cube lift, retention, or a simulated physical grasp. The
unchanged cube pose and zero gripper contacts are direct negative evidence, not a
missing-data success.

Machine-readable details, commands, digests and log paths are in
`2026-08-27-grasp-trial-sequence-complete-physics-false.json`.
