# Grasp physics observer: live negative-path observation

Time: 2026-08-27 06:11:49–06:11:51 Asia/Shanghai
Environment: WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8.11.0, DART
Machine-readable record: `2026-08-27-grasp-physics-negative-runtime.json`

## Outcome

The read-only `GraspPhysicsEvidence` action accepted an observation request in
the live Gazebo graph and received 76 timestamped target-cube pose samples. No
matching grasp-sequence terminal, gripper/cube contact, lift, or retention
evidence was supplied. It therefore terminated fail-closed with:

```text
action status: STATUS_ABORTED (6)
terminal phase: FAULT
reason: observation_sim_timeout
sequence_completed: false
physics_grasp_verified: false
exit code: 0 (the expected negative contract was observed)
```

The observer node had no ActionClient and no motion-command publisher. Its only
application publisher was `/edgegrasp/grasp_physics_status`.

## Live graph evidence

- `joint_state_broadcaster`, `arm_controller`, and `gripper_controller` were active.
- Both pinned FJT actions were present.
- `/clock` had one publisher, `/ros_gz_bridge`.
- The SO-101 model remained at XYZ `[0, 0, 0]`, RPY `[0, 0, 0]`.
- The current Gazebo server log contained zero matching DART mesh-construction diagnostics.
- `/edgegrasp/target_cube_pose` carried a valid sim stamp and frame
  `edgegrasp_table_cube`; the observed cube centre z was
  `0.424999999902001 m`.
- `/edgegrasp/target_cube_contacts` identified
  `target_cube::target_cube_link::collision` against
  `table::table_link::collision` at approximately `z=0.4 m`.

## Reproduction

With the proxy Gazebo launch already running:

```bash
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash
python3 /home/edgegrasp/ros2_ws/src/edgegrasp-sim/scripts/check_grasp_physics_observer.py
```

Launch command used:

```bash
ros2 launch edgegrasp_ros edgegrasp_proxy_gazebo.launch.py \
  use_camera:=false \
  launch_edgegrasp_nodes:=false \
  launch_physics_observer:=true
```

## Evidence boundary

This proves the live raw pose/contact bridge and the observer's negative,
read-only fail-closed path. It does **not** prove gripper/cube contact, cube
lift, retention, or a physics grasp. A positive claim remains blocked on all
three independent observations: correlated successful sequence terminal,
gripper/cube contact, and cube lift retained above the table.
