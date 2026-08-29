# DART proxy pre-anchor runtime observation — failed

- Observation time: 2026-08-26 21:11–21:22 Asia/Shanghai
- Environment: Ubuntu 24.04 WSL2, Gazebo Sim 8.11.0, gz-physics 7.6.0, DART 6.13.2, ROS 2 Jazzy
- Gazebo log: `/home/edgegrasp/.gz/sim/log/2026-08-26T21:11:56.752523438/server_console.log`
- Launch: `ros2 launch edgegrasp_ros edgegrasp_proxy_gazebo.launch.py use_camera:=false launch_edgegrasp_nodes:=false`
- Evidence status: **FAILED_BEHAVIOR_PROBE; superseded implementation snapshot**

This run used the first generated primitive-collision URDF, before the
EdgeGrasp world-to-base fixed anchor was added. It is retained as negative
evidence and must not be reported as a collision-proxy pass.

Observed interface facts:

- The DART engine plugin loaded.
- `joint_state_broadcaster`, `arm_controller`, and `gripper_controller` were active.
- `/joint_states` contained the six expected joints.
- Both expected `FollowJointTrajectory` actions were present.
- The 13 earlier DART mesh-construction diagnostics and associated
  `geometry ... couldn't be created` messages were absent from this log.

Those observations did **not** establish physical correctness. The behavior
probe failed:

```text
PROBE_SCOPE=one_base_proxy_contact_only
INITIAL_POSE z=0.000395
SETTLED_POSE_T_PLUS_3S z=0.003515
RETENTION_POSE_T_PLUS_5S z=0.005390
```

A later settled-state query returned:

```text
edgegrasp_base_proxy_probe: xyz ~= [0, 0, 0.007499]
so101: xyz = [0.016581, 0.000068, 0.043666]
       rpy = [-0.000003, 0.689333, -0.000962]
```

The probe landed on the ground rather than the expected base-proxy top near
`z ~= 0.078 m`, and the robot moved and tilted. Two defects were identified:

1. `ros_gz_sim create -file` supplied a default factory pose, so the model pose
   embedded in the probe SDF was not the effective spawn pose.
2. The pinned `use_gazebo:=true` xacro expansion has `base_link` as a free root;
   the `world` link and fixed `world_joint` exist only in its non-Gazebo branch.

The generated EdgeGrasp copy is therefore being changed to inject and validate
the fixed `world -> base_link` anchor, and the probe command now supplies its
XYZ explicitly. A new clean world and a new probe entity are required for
revalidation; this run cannot be reused.

The contact sensor also exposed a schema placement issue: the sensor-level
topic was ignored and Gazebo published the world-scoped default contact topic.
The project SDF now places the requested topic under `<contact>` and requires a
fresh runtime check.

The older mesh log remains classified as
`DART_COLLISION_CONSTRUCTION_DIAGNOSTIC_UNRESOLVED`: disappearance of the text
alone is not a collision-behavior result.
