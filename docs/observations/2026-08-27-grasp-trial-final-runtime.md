# Final correlated grasp-trial runtime

Observation window: 2026-08-27 07:32–07:54 Asia/Shanghai.

The final scoped result is **sequence success, physics grasp false**. Task006
completed `APPROACH → DESCEND → CLOSE_GRIPPER → LIFT → COMPLETE`; every stage
ended with a matching typed gate result, action `STATUS_SUCCEEDED`, and
`FollowJointTrajectory` error code `0`. The SO-101 model stayed at world pose
zero. The cube stayed at `[0.2, 0.0, 0.425]`, with no configured gripper-cube
contact, lift, or retention evidence.

## What the three reruns established

- Task004 still produced `clock_rollback` after only serializing the ROS clock
  read. That disproved the first, incomplete diagnosis.
- Task005 re-sampled time under the same lock as each strict-core operation.
  The clock fault disappeared, but the MoveIt adapter canceled lift because its
  wall-clock result deadline expired before a slow Gazebo simulation reached the
  downstream terminal.
- Task006 used a bounded 15-second adapter wall guard while the typed gate kept
  ROS-clock motion safety. All four stages completed and no clock rollback or
  premature lift cancellation recurred.

The independent physics observer collected 2781 cube-pose samples and 11408
table-contact samples. It collected zero gripper contacts, and baseline, peak,
and final cube Z were all `0.424999999902001 m`. It eventually returned
`observation_wall_timeout` because the 30 simulated-second observation window
did not finish inside its 60 wall-second guard. This is a bounded fail-closed
observer terminal, not evidence of a grasp.

Before the accepted task006 request, a 120-second observation request was
rejected because the configured maximum is 30 seconds. No sequence or motion
started for that rejected request.

## Evidence boundary

This verifies the live target/MoveIt plan-only/typed gate/FJT/sequence chain for
the chosen fixed poses. It also supplies direct negative physics evidence. It
does not verify a physical simulated grasp, cube retention, every collision
proxy, or real hardware.

Machine-readable values, full digests, log paths, cleanup state, and the task004
and task005 progression are in
`2026-08-27-grasp-trial-final-runtime.json`.
