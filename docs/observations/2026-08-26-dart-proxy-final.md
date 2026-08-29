# DART primitive proxy runtime observation

- Observation window: 2026-08-26 21:29–21:32 Asia/Shanghai
- ROS launch log: `/home/edgegrasp/.ros/log/2026-08-26-21-29-57-252889-DESKTOP-MGV04IT-4208`
- Gazebo log: `/home/edgegrasp/.gz/sim/log/2026-08-26T21:29:58.66319727/server_console.log`
- Runtime: Ubuntu 24.04 WSL2, ROS 2 Jazzy, Gazebo Sim 8.11.0,
  gz-physics 7.6.0, DART 6.13.2
- Scope result: **SPECIFIC_BASE_PROXY_CONTACT_PASS**

Launch:

```bash
ros2 launch edgegrasp_ros edgegrasp_proxy_gazebo.launch.py \
  use_camera:=false launch_edgegrasp_nodes:=false
```

The generated robot input passed `xacro` and `check_urdf` before launch. Its
generation report recorded 13 replacements, zero remaining collision meshes,
18 collision boxes, 17 visual meshes, one `world` root, and one fixed
`world -> base_link` joint.

Runtime interface observations:

```text
joint_state_broadcaster active
arm_controller          active
gripper_controller      active

/arm_controller/follow_joint_trajectory
  control_msgs/action/FollowJointTrajectory
/gripper_controller/follow_joint_trajectory
  control_msgs/action/FollowJointTrajectory

joint names:
elbow_flex, gripper, shoulder_lift, shoulder_pan, wrist_flex, wrist_roll
```

`/tf_static` had one publisher, `robot_state_publisher`. The SO-101 Gazebo
model pose remained XYZ `[0,0,0]`, RPY `[0,0,0]` before and after the probe.
Both `/edgegrasp/table_contacts` and the dynamically created
`/edgegrasp/base_proxy_probe_contacts` were visible. The table contact message
identified `table::table_top::collision` against
`target_cube::cube_link::collision`.

Behavior probe command:

```bash
bash /home/edgegrasp/ros2_ws/src/edgegrasp-sim/scripts/probe_base_collision.sh \
  edgegrasp_table_cube \
  /home/edgegrasp/ros2_ws/install/edgegrasp_ros/share/edgegrasp_ros/worlds/base_proxy_probe.sdf \
  edgegrasp_base_proxy_probe_v4
```

Machine-checked result, exit code 0:

```text
PROBE_SPAWN_XYZ=0.0,0.0,0.16
SETTLED_Z=0.077600
RETENTION_Z=0.077600
collision1=edgegrasp_base_proxy_probe_v4::probe_link::probe_collision
collision2=so101::base_link::base_link_fixed_joint_lump__edgegrasp_proxy_base_link_collision_1_collision_1
contact surface z ~= 0.07009999
contact normal z ~= 1.0
RESULT=SPECIFIC_BASE_PROXY_CONTACT_PASS
```

The probe parser intentionally accepts Gazebo's
`base_link_fixed_joint_lump__..._collision_1` name decoration while separately
requiring the `so101::base_link::` identity and the contracted
`edgegrasp_proxy_base_link_collision_1` name.

The current Gazebo log loaded the DART plugin and contained:

```text
Mesh construction diagnostic count: 0
geometry ... couldn't be created count: 0
```

This result verifies DART contact behavior for exactly one generated base box
proxy plus the table/cube primitive contact. It does **not** verify every arm
proxy, exact mesh fidelity, MoveIt collision rejection, trajectory collision
safety, grasp contact/retention, or real hardware. The older mesh diagnostic
remains historical evidence; its disappearance is supporting evidence here,
not the sole pass criterion.
