# Grasp physics observer: bounded-clock live negative path

Time: 2026-08-27 06:38:11–06:38:15 Asia/Shanghai
Environment: WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.11.0, DART
Machine-readable record: `2026-08-27-grasp-physics-negative-runtime-v2.json`

## Outcome

The live observer accepted a read-only evidence goal and processed 75
timestamped cube-pose samples. Gazebo pose and `/clock` use separate bridge
callbacks, so the wrapper now temporarily queues only same-domain samples whose
source stamp is ahead of the locally observed ROS clock by no more than the
configured 200 ms freshness bound. It submits them to the unchanged strict core
only after the ROS clock catches up. Larger future skew remains fail-closed.

No correlated sequence terminal, gripper/cube contact, lift, or retention was
present. The action therefore produced the expected negative terminal:

```text
action status: STATUS_ABORTED (6)
terminal phase: FAULT
reason: observation_sim_timeout
sequence_completed: false
gripper_contact_observed: false
lift_observed: false
retention_observed: false
physics_grasp_verified: false
```

The generated evidence digest was
`8be99a6c090967bcc0513015cadcb7623df455527e7b700f3ab2154a76056821`.

## Teardown and claim boundary

The launch reported all three expected controllers active and the current
Gazebo log had zero matching DART mesh-construction diagnostics. On Ctrl-C the
observer exited cleanly without the earlier unobserved `Future`/`Destroyable`
warning. A post-shutdown check found no matching ROS/Gazebo process and ROS 2
reported `/clock` as unknown.

This is live negative-path and teardown evidence. It is not a physics-grasp
result. Positive evidence still requires the same task identity to bind a
successful sequence terminal, an actual gripper/cube collision pair, cube lift
above the table, and an uninterrupted retention window.
