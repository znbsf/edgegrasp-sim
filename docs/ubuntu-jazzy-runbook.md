# Ubuntu 24.04 / ROS 2 Jazzy runbook

This runbook keeps the pinned SO-101 source, the EdgeGrasp monorepo adapter,
and every generated ROS artifact isolated from the Windows checkout. Commands
that bypass the EdgeGrasp trajectory gate are labeled explicitly.

## Evidence levels

1. Package inventory proves only that binaries are installed.
2. `rosdep`, `colcon`, xacro, and `check_urdf` prove build/model parsing.
3. Controller/action/topic observations prove a running interface.
4. A successful MoveIt response proves planning only.
5. `PlanTarget -> MoveGroup(plan_only) -> EdgeGrasp gate -> FJT` plus controller
   result proves the tested simulated command path.
6. A full `GraspSequence.action` result proves protocol completion only when
   each matching PlanTarget/ExecuteTrajectory/FJT terminal succeeds.
7. None of the above proves collision fidelity, grasp success, or real hardware
   safety.

## Windows host boundary

Safe read-only inventory:

```powershell
cmd.exe /c ver
wsl.exe --status
wsl.exe --version
wsl.exe --list --verbose
wsl.exe --list --online
powershell -NoProfile -File scripts\audit_environment.ps1 -IncludeWsl
```

Never unregister the existing `Ubuntu-22.04`. `wsl --update`, `wsl --shutdown`,
`wsl --install -d Ubuntu-24.04`, optional Windows features, BIOS
virtualization, GPU/vGPU drivers, SFC/DISM, and reboot are host state changes.
They require explicit user authorization and, where applicable, an
administrator terminal. A failed distro start is recorded as `UNVERIFIED`; do
not auto-repair it from an audit script.

Official references: [WSL install](https://learn.microsoft.com/en-us/windows/wsl/install),
[WSL unexpected-failure troubleshooting](https://learn.microsoft.com/en-us/windows/wsl/troubleshooting#the-error-code-0x8000ffff-unexpected-failure),
and [WSLg GUI applications](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps).

## Ubuntu guest prerequisites

Use Ubuntu 24.04 Noble, ROS 2 Jazzy, and Gazebo Harmonic. After configuring the
official ROS apt source, the project dependency set is:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools python3-rosdep python3-pytest python3-venv
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-moveit
sudo apt install ros-jazzy-rosbag2-storage-mcap liburdfdom-tools rsync
```

Ubuntu `sudo` is guest-root permission, not Windows administrator permission.
The confirmed Jazzy package is `ros-jazzy-gz-ros2-control`; still verify the
installed ROS package after installation.

```bash
source /opt/ros/jazzy/setup.bash
ros2 doctor --report
gz sim --versions
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix ros_gz_bridge
ros2 pkg prefix controller_manager
ros2 pkg prefix gz_ros2_control
ros2 pkg prefix moveit_ros_move_group
ros2 pkg prefix moveit_setup_assistant
```

Package prefixes are not SO-101 runtime evidence. See the official
[ROS/Gazebo installation guidance](https://gazebosim.org/docs/harmonic/ros_installation/),
[ROS 2 Jazzy binary install](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html),
[gz_ros2_control Jazzy docs](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html),
and [MoveIt binary install](https://moveit.ai/install-moveit2/binary/).

## Isolated workspace and exact sources

The upstream launch writes generated files under hard-coded `~/ros2_ws`
paths. Use a dedicated Ubuntu user/distro. If `~/ros2_ws` already belongs to
another project, do not merge or delete it; create a fresh distro/user.

```bash
source /opt/ros/jazzy/setup.bash
SRC=/mnt/c/Users/huang/Documents/githubsss/workspaces/forks/so101_ros2
test "$(git -C "$SRC" rev-parse HEAD)" = 0305e03ab54e64aae9263fcbf339622e654012f3
test -z "$(git -C "$SRC" status --porcelain)"
mkdir -p "$HOME/ros2_ws/src"
rsync -a --exclude .git "$SRC/" "$HOME/ros2_ws/src/so101_ros2/"
rsync -a --exclude .git --exclude .venv --exclude .venv-mujoco314 \
  --exclude build --exclude install --exclude log --exclude __pycache__ \
  --exclude '*.pyc' --exclude .pytest_cache --exclude .ruff_cache \
  /mnt/c/Users/huang/Documents/githubsss/incubator/edgegrasp-sim/ \
  "$HOME/ros2_ws/src/edgegrasp-sim/"
cd "$HOME/ros2_ws"
colcon list
```

`edgegrasp_core` is deliberately a monorepo adapter. Keep the entire
`edgegrasp-sim` layout so `src/edgegrasp` remains three parents above
`ros_ws/src/edgegrasp_core/setup.py`. Copying only that ROS subpackage fails
with an intentional diagnostic.

Run dependency resolution before applying any upstream metadata patch:

```bash
sudo rosdep init  # once per distro; omit when already initialized
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
```

On the audited pin, clean rosdep reproduced unresolved metadata keys
`ament_python` and `moveit_planners_pilz_industrial_motion_planner`. After
confirming their actual packages are installed, the bounded workaround used
for this disposable workspace was:

```bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y \
  --skip-keys "ament_python moveit_planners_pilz_industrial_motion_planner"
```

Build and test:

```bash
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --event-handlers console_direct+
source "$HOME/ros2_ws/install/setup.bash"
python3 -c 'import edgegrasp; print(edgegrasp.DEFAULT_TARGET_FRAME)'
colcon test --test-result-base "$HOME/ros2_ws/test_results/edgegrasp_scoped" \
  --packages-select edgegrasp_core edgegrasp_interfaces \
  edgegrasp_ros edgegrasp_moveit_adapter edgegrasp_grasp_sequence \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result \
  --test-result-base "$HOME/ros2_ws/test_results/edgegrasp_scoped" \
  --all --verbose
```

On the latest validated snapshot, the five EdgeGrasp packages collected 113
tests: core 1, ROS safety/gate/observer 49, MoveIt adapter 21, and grasp
sequence 42, with zero failures/errors/skips; the interface package has no
tests. A separate
whole-workspace test also exposed upstream-only lint failures in the pinned
checkout; do not patch the clean third-party source merely to hide them.

Write xacro output only to `/tmp`:

```bash
ros2 run xacro xacro \
  "$HOME/ros2_ws/src/so101_ros2/so101_description/urdf/robots/so101.urdf.xacro" \
  use_gazebo:=true use_camera:=true > /tmp/so101.urdf
check_urdf /tmp/so101.urdf
```

Do not use `so101_bringup/scripts/so101_gazebo_and_moveit.sh` for the first
run; it contains fixed sleeps and broad `pkill -9` cleanup.

## A. Controller-only runtime gate

```bash
ros2 launch so101_gazebo so101.gazebo.launch.py \
  robot_name:=so101 world_file:=empty.world use_camera:=false use_rviz:=false \
  use_robot_state_pub:=true use_sim_time:=true load_controllers:=true \
  spawn_delay:=2.0 controller_load_delay:=5.0
```

In a second sourced shell:

```bash
ros2 control list_controllers -c /controller_manager
ros2 topic echo --once /joint_states
ros2 action list -t
ros2 topic info /arm_controller/joint_trajectory
ros2 topic info /gripper_controller/joint_trajectory
```

Expected runtime contract: `joint_state_broadcaster`, `arm_controller`, and
`gripper_controller` active; six joint states; two
`control_msgs/action/FollowJointTrajectory` actions. Joint order is exactly:

```text
arm: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll (radians)
gripper: gripper (radians)
```

Launch the EdgeGrasp safety boundary, not a duplicate MoveIt config:

```bash
ros2 launch edgegrasp_ros edgegrasp_mock.launch.py \
  backend:=gazebo require_camera:=false use_sim_time:=true
```

The preferred project boundary is the typed
`/edgegrasp/execute_trajectory` action. The following request topics are
legacy, uncorrelated gate diagnostics retained for low-level simulation smoke
tests; sequence and MoveIt adapter code must not use them:

```bash
ros2 topic pub --once /edgegrasp/arm_joint_trajectory_request \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], points: [{positions: [0.1, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}"
ros2 topic pub --once /edgegrasp/gripper_joint_trajectory_request \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [gripper], points: [{positions: [0.2], time_from_start: {sec: 2}}]}"
```

The following commands are **UNSAFE/BYPASS DIAGNOSTICS, SIMULATION ONLY**.
They do not test EdgeGrasp safety and are never sent automatically:

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], points: [{positions: [0.1, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 3}}]}}"
ros2 action send_goal /gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [gripper], points: [{positions: [0.2], time_from_start: {sec: 2}}]}}"
```

Inspect the current start state and use conservative positions before either
diagnostic.

## B. Camera/world gate

Stop the controller-only simulator, then:

```bash
ros2 launch so101_gazebo so101.gazebo.launch.py \
  robot_name:=so101 world_file:=pick_and_place.world use_camera:=true \
  use_rviz:=false use_sim_time:=true
ros2 topic echo --once /clock
ros2 topic hz /camera_head/color/image_raw
ros2 topic hz /camera_head/depth/image_rect_raw
ros2 topic echo --once /camera_head/depth/camera_info
```

The first run may download Gazebo Fuel assets. The upstream simulated camera
is not evidence of a physical D405. The original mesh run emitted unresolved
DART collision-construction diagnostics. In later generated-primitive runs the
tracked diagnostic count was zero, and separate behavior probes observed the
table-cube pair plus one specifically named base proxy. Diagnostic absence
alone is not collision proof; full-link fidelity and grasp behavior remain
unverified.

## C. MoveIt plan-only diagnostics

After the controllers are active, start MoveIt through the unique EdgeGrasp
thin overlay. It reuses the pinned upstream MoveIt configuration, adds only the
two project-owned distal contact boxes, injects Pilz `ValidateSolution`, and
disables direct MoveGroup execution:

```bash
ros2 launch edgegrasp_ros edgegrasp_proxy_move_group.launch.py \
  robot_name:=so101 use_camera:=false use_gazebo:=true \
  use_sim_time:=true use_rviz:=false
ros2 param get /move_group allow_trajectory_execution
ros2 param get /move_group disable_capabilities
ros2 interface show moveit_msgs/srv/GetMotionPlan
ros2 interface show moveit_msgs/msg/MotionPlanRequest
ros2 service list -t | grep plan_kinematic_path
```

Require `allow_trajectory_execution=false`, the two disabled execute
capabilities, and a startup log showing Pilz loaded
`default_planning_response_adapters/ValidateSolution`. Starting the pinned
upstream `move_group.launch.py` directly remains a lower-level diagnostic; it is
not the EdgeGrasp collision-validated control path.

`GetMotionPlan` contains only `motion_plan_request`; it has no
`PlanningOptions`. Use the service only after the runtime endpoint/type is
confirmed:

```bash
ros2 service call /plan_kinematic_path moveit_msgs/srv/GetMotionPlan \
"{motion_plan_request: {start_state: {joint_state: {name: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], position: [0.0, 0.0, 0.0, 0.0, 0.0]}, is_diff: true}, goal_constraints: [{joint_constraints: [{joint_name: shoulder_pan, position: 0.05, tolerance_above: 0.005, tolerance_below: 0.005, weight: 1.0}, {joint_name: shoulder_lift, position: -0.05, tolerance_above: 0.005, tolerance_below: 0.005, weight: 1.0}, {joint_name: elbow_flex, position: 0.05, tolerance_above: 0.005, tolerance_below: 0.005, weight: 1.0}, {joint_name: wrist_flex, position: -0.05, tolerance_above: 0.005, tolerance_below: 0.005, weight: 1.0}, {joint_name: wrist_roll, position: 0.0, tolerance_above: 0.005, tolerance_below: 0.005, weight: 1.0}]}], pipeline_id: ompl, group_name: arm, num_planning_attempts: 1, allowed_planning_time: 2.0, max_velocity_scaling_factor: 0.2, max_acceleration_scaling_factor: 0.2}}"
```

Success requires `error_code.val == 1`, a nonempty exact-order trajectory,
finite vectors, increasing time, and empty fixed-base multi-DOF points. It is
planning evidence only and must never auto-execute the response.

### C1. Load and confirm the shared PlanningScene

Keep the same Gazebo graph running; do not start `edgegrasp_scene.launch.py` as
a second simulator. In a new sourced shell, start the EdgeGrasp-owned thin
loader after move_group is ready:

```bash
ros2 launch edgegrasp_ros planning_scene.launch.py \
  use_sim_time:=true clock_domain:=ros_sim clock_epoch:=0 \
  target_frame:=base_link include_optional_cube:=false
```

The default `scene_config` is the installed `edgegrasp_ros/config/scene.json`;
the launch argument may name an explicit complete-project copy when auditing a
different generated scene. Readiness is fail closed and requires the service
to echo the expected names and geometry:

```bash
ros2 service list -t | grep -F '/get_planning_scene [moveit_msgs/srv/GetPlanningScene]'
ros2 topic echo --once --full-length /edgegrasp/planning_scene_status
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene \
  "{components: {components: 24}}"
```

Mask `24` is `WORLD_OBJECT_NAMES | WORLD_OBJECT_GEOMETRY`. Before proceeding,
the status must say `ready=true`, `reason=confirmed`, frame `base_link`, the
expected scene digest, and object `edgegrasp_table`. A periodic requery keeps
the previously confirmed state only while that same geometry remains within
the adapter receive-time TTL; query timeout, service loss, geometry drift, or
clock rollback revokes readiness. Bool alone does not prove cross-node epoch
propagation, so reset all participating nodes together or restart the isolated
graph.

The checked-in `worlds/table_cube.sdf` is generated from the same JSON and a
test requires byte equality. After changing the JSON, rerun the generator and
tests before launching. `include_optional_cube:=true` adds
`edgegrasp_target_cube` to MoveIt. The later 15:14 runtime confirmed both
table-plus-cube and table-only modes from the same digest. Disable the optional
cube only at an explicit intentional-contact boundary; never remove the table
or use target removal to hide a non-target collision.

## D. EdgeGrasp target-to-controller path

The adapter is separate from upstream MoveIt configuration:

```text
PlanTarget immutable pose
  -> timestamped tf2 to base_link
  -> /compute_ik with fresh joint-state seed
  -> /move_action with planning_options.plan_only=true
  -> RobotTrajectory validator
  -> /edgegrasp/execute_trajectory (controller=arm_controller)
  -> trajectory_gate
  -> /arm_controller/follow_joint_trajectory
```

Start it only after A and C are healthy:

```bash
ros2 interface show edgegrasp_interfaces/action/PlanTarget
ros2 interface show moveit_msgs/action/MoveGroup
ros2 launch edgegrasp_moveit_adapter moveit_adapter.launch.py \
  use_sim_time:=true clock_domain:=ros_sim planning_frame:=base_link
```

The bundled one-shot client can derive a target from the current end-effector
pose. A zero offset is a wiring smoke test; a meaningful displacement should
use a known reachable pose and conservative limits:

```bash
ros2 run edgegrasp_moveit_adapter plan_target_client --ros-args \
  -p use_sim_time:=true -p clock_domain:=ros_sim \
  -p target_frame:=base_link -p use_current_end_effector_pose:=true \
  -p planning_timeout_s:=0.15 -p velocity_scaling:=0.1 \
  -p acceleration_scaling:=0.1
```

The first milestone accepts only group `arm`; gripper execution remains a
separate typed `ExecuteTrajectory` request. A target without orientation keeps
the fresh current end-effector orientation for IK. Unreachable 5-DOF pose
requests fail with no trajectory publication.

Physical position limits originate in the pinned URDF; velocity/acceleration
planning caps originate in MoveIt's `joint_limits.yaml`. EdgeGrasp applies a
narrower project-owned admission profile and 0.1-0.2 scaling. Inspect the final
merged robot model at runtime rather than treating the YAML alone as position
limits.

The 2026-08-27 scoped table cases used the client above and immutable
`task_id|stage|sequence_no` command IDs:

| Case | Target/parameter change | Required result |
| --- | --- | --- |
| A | current end-effector pose, zero offset | one typed arm command; wrapper and FJT status 4; FJT error 0 |
| B | `[0.2, 0.0, 0.38]` m, inside the table box | IK/MoveIt failure and zero dispatch |
| C | FK pose of joints `[0,-1.5,0,0,0]`, requiring a table crossing/detour | accepted collision-valid detour or failure with zero dispatch; never accept a nonempty trajectory whose MoveIt error is not SUCCESS |
| D | `[2.0, 0.0, 2.0]` m | unreachable and zero dispatch |
| E | wrong epoch or planning beyond source freshness | reject/cancel and zero dispatch |

The earlier recorded C result was `ValidateSolution: INVALID_MOTION_PLAN` / error
`99999`; although the plan-only service response contained a candidate
trajectory, EdgeGrasp published none. Exact results are in
`docs/observations/2026-08-27-planning-scene-runtime.json`. This proves the
required table rejection only. The later optional-cube run rejected an
84-state mid approach at indices 51–71 for target-cube versus `gripper_link`
collision, then accepted a farther approach without cube motion. Exact results
are in
`docs/observations/2026-08-27-pilz-collision-contact-runtime.json`. Minimum
clearance and whole-robot collision fidelity remain unmeasured.

Official schemas: [GetMotionPlan](https://raw.githubusercontent.com/moveit/moveit_msgs/2.7.1/srv/GetMotionPlan.srv),
[MotionPlanRequest](https://raw.githubusercontent.com/moveit/moveit_msgs/2.7.1/msg/MotionPlanRequest.msg),
[RobotTrajectory](https://raw.githubusercontent.com/moveit/moveit_msgs/2.7.1/msg/RobotTrajectory.msg),
[MoveItErrorCodes](https://raw.githubusercontent.com/moveit/moveit_msgs/2.7.1/msg/MoveItErrorCodes.msg),
and [JointTrajectory](https://raw.githubusercontent.com/ros2/common_interfaces/jazzy/trajectory_msgs/msg/JointTrajectory.msg).

### D1. Four-stage correlated sequence

After the typed target publisher, safety monitor, interface probe, PlanningScene
loader, MoveIt adapter, and trajectory gate are healthy, start the orchestrator:

```bash
ros2 interface show edgegrasp_interfaces/action/GraspSequence
ros2 launch edgegrasp_grasp_sequence grasp_sequence.launch.py \
  use_sim_time:=true clock_domain:=ros_sim clock_epoch:=0 \
  target_frame:=base_link
```

The one-shot client snapshots a matching fresh `TrackedTarget`. It does not
silently perform a TF orientation lookup: first inspect the current
`base_link -> gripper_frame_link` transform, then pass an explicit normalized
approach quaternion and an explicit normalized grasp quaternion. Approach uses
the first; descend and lift share the second. Both remain immutable for the
task, and either zero quaternion is rejected. Validate every pose and the
current gripper start state before sending it.

The following is the exact conservative integration-smoke payload that
completed on 2026-08-27 after a separately gated +30 mm z preposition. Do not
reuse these absolute values from a different start state; re-snapshot and
revalidate them:

```bash
ros2 run edgegrasp_grasp_sequence grasp_sequence_client --ros-args \
  -p use_sim_time:=true -p clock_domain:=ros_sim -p clock_epoch:=0 \
  -p task_id:=grasp-sequence-runtime-0331 -p target_id:=ros-target \
  -p approach_position_m:="[0.391231968, -0.001571668, 0.256520737]" \
  -p descend_position_m:="[0.391231968, -0.001571668, 0.251520737]" \
  -p lift_position_m:="[0.391231968, -0.001571668, 0.261520737]" \
  -p approach_orientation_xyzw:="[0.017007859, 0.706463960, 0.013976791, 0.707406570]" \
  -p grasp_orientation_xyzw:="[0.017007859, 0.706463960, 0.013976791, 0.707406570]" \
  -p gripper_closed_position_rad:=0.2 \
  -p planning_timeout_s:=1.0 -p result_timeout_s:=60.0 \
  -p velocity_scaling:=0.1 -p acceleration_scaling:=0.1
```

The recorded task returned wrapper status 4 and `sequence_completed=true`.
It used positions below the table as a wiring/ordering test; it did not interact
with the cube and correctly returned `physics_grasp_verified=false`. An early
repeat entered SAFE_STOP on `joint_state_stale` while the gripper stage was
active, requested cancellation, and sent no lift. That failure remains recorded
rather than being hidden by the later accepted batch.

The wrapper issues exactly one stage at a time and advances only after matching
typed terminal evidence. An invalid command ID/digest/status retains the
downstream slot and refuses reset; client cancellation is serialized ahead of
any late success callback. Even a successful result means
`sequence_completed=true` and `physics_grasp_verified=false` until contact,
cube lift, and retention are independently observed.

### D2. Cube-scoped sequence plus read-only physics evidence

Launch the proxy graph with `target_id:=target_cube` and mock target coordinates
equal to the scene cube center (`0.2, 0.0, 0.425`), then start MoveGroup, the
PlanningScene loader, MoveIt adapter, and sequence node as above. Confirm one
`/clock` publisher, three active controllers, `/edgegrasp/planning_scene_ready`,
and all typed actions before motion.

The accepted task006 first used the typed gate—not direct FJT—to open the
gripper and move to the validated pre-position:

```bash
cd /home/edgegrasp/ros2_ws/src/edgegrasp-sim
python3 scripts/prepare_so101_trial.py \
  --task-id grasp-trial-preparation-006 --target-id target_cube \
  --gripper 1.5 --arm -0.05 0.8 -1.4 -1.4 0.0 --timeout-s 30.0 \
  --ros-args -p use_sim_time:=true
```

Then the one-shot client correlated the same target snapshot with an independent
read-only physics-evidence action:

```bash
ros2 run edgegrasp_grasp_sequence grasp_trial_client --ros-args \
  -p use_sim_time:=true -p clock_domain:=ros_sim -p clock_epoch:=0 \
  -p task_id:=grasp-physics-trial-20260827-006 -p target_id:=target_cube \
  -p approach_position_m:="[0.218593782915783,0.008984635830313,0.403477545364772]" \
  -p descend_position_m:="[0.230058188982453,0.009558463149666,0.382321322136899]" \
  -p lift_position_m:="[0.218593782915783,0.008984635830313,0.403477545364772]" \
  -p approach_orientation_xyzw:="[0.000140847932006,-0.212962906352545,0.048190781189021,0.975871113051375]" \
  -p grasp_orientation_xyzw:="[0.000140847932006,-0.212962906352545,0.048190781189021,0.975871113051375]" \
  -p gripper_closed_position_rad:=0.2 \
  -p planning_timeout_s:=2.0 -p sequence_timeout_s:=120.0 \
  -p physics_timeout_s:=130.0 -p observation_timeout_s:=30.0 \
  -p velocity_scaling:=0.1 -p acceleration_scaling:=0.1
```

Do not raise `observation_timeout_s` above the observer's configured 30-second
maximum; task006 first demonstrated that such a request is rejected before the
sequence starts. The accepted run completed all four typed/FJT stages but exited
`11`, which means sequence success with physics unverified. It observed zero
gripper contacts and unchanged cube Z; the observer reached its bounded wall
timeout because this Gazebo run was slower than 0.5 real time. Treat this as
direct negative grasp evidence, not as a successful pick. Exact results are in
`docs/observations/2026-08-27-grasp-trial-final-runtime.json`.

The later collision-validated trial used scene cube center
`[0.24695465627174787, 0.1218646928533295, 0.205]`. First keep the optional cube
enabled and execute only the farther approach through `PlanTarget`:

```bash
ros2 run edgegrasp_moveit_adapter plan_target_client --ros-args \
  -p use_sim_time:=true -p clock_domain:=ros_sim -p clock_epoch:=0 \
  -p task_id:=far-approach-runtime-037 -p stage:=approach \
  -p sequence_no:=0 -p target_id:=target_cube \
  -p target_x_m:=0.27859814485128837 \
  -p target_y_m:=0.1733906932169842 \
  -p target_z_m:=0.29693801242225025 \
  -p orientation_x:=0.3450298741758752 \
  -p orientation_y:=0.6172118344266647 \
  -p orientation_z:=0.6332767577280262 \
  -p orientation_w:=0.3145862131297726 \
  -p pipeline_id:=pilz_industrial_motion_planner -p planner_id:=PTP \
  -p planning_timeout_s:=2.0 -p result_timeout_s:=20.0 \
  -p velocity_scaling:=0.2 -p acceleration_scaling:=0.2
```

Confirm FJT success and unchanged cube pose. Only then request the target-contact
scene transition and wait for a confirmed status:

```bash
ros2 service call /edgegrasp/set_optional_cube_collision \
  std_srvs/srv/SetBool "{data: false}"
ros2 topic echo --once --full-length /edgegrasp/planning_scene_status
```

The status must name expected `edgegrasp_table`, forbidden
`edgegrasp_target_cube`, `ready=true`, and `reason=confirmed`. With the sequence
and physics observer already running, task039 used:

```bash
ros2 run edgegrasp_grasp_sequence grasp_trial_client --ros-args \
  -p use_sim_time:=true -p clock_domain:=ros_sim -p clock_epoch:=0 \
  -p task_id:=collision-validated-physics-039 -p target_id:=target_cube \
  -p approach_position_m:="[0.27859814485128837,0.1733906932169842,0.29693801242225025]" \
  -p descend_position_m:="[0.2538169071284581,0.15643670690409664,0.2166595364540212]" \
  -p lift_position_m:="[0.2537915860804717,0.15641927732806185,0.2567110773505267]" \
  -p approach_orientation_xyzw:="[0.3450298741758752,0.6172118344266647,0.6332767577280262,0.3145862131297726]" \
  -p grasp_orientation_xyzw:="[0.3450298741758752,0.6172118344266647,0.6332767577280262,0.3145862131297726]" \
  -p gripper_closed_position_rad:=0.77 \
  -p pipeline_id:=pilz_industrial_motion_planner -p planner_id:=PTP \
  -p planning_timeout_s:=2.0 -p sequence_timeout_s:=90.0 \
  -p physics_timeout_s:=100.0 -p observation_timeout_s:=30.0 \
  -p velocity_scaling:=0.1 -p acceleration_scaling:=0.1
```

Task039 completed every typed/FJT stage and observed gripper/cube contact, but
the contact entity was `gripper_link`, peak lift was only 2.27 mm, the cube
moved about 60 mm laterally, and retention was false. Exit `11` remains the
expected client code for sequence success with physics unverified. The approach
preposition and manual target-contact scene transition are separate steps; the
orchestrator does not yet automate that policy.

### D3. Candidate003 strict distal-pad trial

Candidate002's negative tool-axis approach is now a required fail-closed probe:
Pilz rejected states 57–68 for cube versus `gripper_link`, and OMPL returned
`GOAL_STATE_INVALID`. Do not execute it. Candidate003 keeps the corrected
descend geometry but uses this collision-validated far approach:

```bash
ros2 run edgegrasp_moveit_adapter plan_target_client --ros-args \
  -p use_sim_time:=true -p clock_domain:=ros_sim -p clock_epoch:=0 \
  -p task_id:=candidate003-approach-041 -p stage:=approach \
  -p sequence_no:=0 -p target_id:=target_cube \
  -p target_x_m:=0.2709963789421751 \
  -p target_y_m:=0.16819000116779706 \
  -p target_z_m:=0.2967142812839486 \
  -p orientation_x:=0.3450298741758752 \
  -p orientation_y:=0.6172118344266647 \
  -p orientation_z:=0.6332767577280262 \
  -p orientation_w:=0.3145862131297726 \
  -p pipeline_id:=pilz_industrial_motion_planner -p planner_id:=PTP \
  -p planning_timeout_s:=2.0 -p result_timeout_s:=30.0 \
  -p velocity_scaling:=0.1 -p acceleration_scaling:=0.1
```

Require `moveit_error_code=1`, matching typed/downstream terminal status 4,
FJT error 0, `moveit_direct_execution_used=false`, and an unchanged Gazebo cube
pose. Then apply and confirm the same table-only MoveIt contact policy shown
above. Before motion, run plan-only approach-to-descend and descend-to-lift
probes from the observed start states; the recorded candidate003 run returned
51/50 and 22/21 source/gate-ready points. Only then send:

```bash
ros2 run edgegrasp_grasp_sequence grasp_trial_client --ros-args \
  -p use_sim_time:=true -p clock_domain:=ros_sim -p clock_epoch:=0 \
  -p task_id:=candidate003-physics-042 -p target_id:=target_cube \
  -p approach_position_m:="[0.2709963789421751,0.16819000116779706,0.2967142812839486]" \
  -p descend_position_m:="[0.2462364349184608,0.15125054509958977,0.21671404504175756]" \
  -p lift_position_m:="[0.2462364349184608,0.15125054509958977,0.25671404504175754]" \
  -p approach_orientation_xyzw:="[0.3450298741758752,0.6172118344266647,0.6332767577280262,0.3145862131297726]" \
  -p grasp_orientation_xyzw:="[0.3450298741758752,0.6172118344266647,0.6332767577280262,0.3145862131297726]" \
  -p gripper_closed_position_rad:=0.77 \
  -p pipeline_id:=pilz_industrial_motion_planner -p planner_id:=PTP \
  -p planning_timeout_s:=2.0 -p sequence_timeout_s:=90.0 \
  -p physics_timeout_s:=100.0 -p observation_timeout_s:=30.0 \
  -p velocity_scaling:=0.1 -p acceleration_scaling:=0.1
```

The current observer accepts only collisions containing
`edgegrasp_grasp_proxy_fixed_finger_pad` or
`edgegrasp_grasp_proxy_moving_finger_pad`; broad palm/link tokens are rejected.
The recorded trial completed all four FJT stages but observed zero distal-pad
contacts, only 2.43 mm peak lift, no retention and 52.3 mm lateral motion. Exit
11 is therefore expected negative physics evidence. See
`docs/observations/2026-08-27-candidate003-runtime.json`.

### D4. Candidate004 retained-cube selective-contact gate

Candidate003's whole-target removal is preserved only as historical evidence.
For current work, keep the table and cube in MoveIt and permit target contact
only for the two EdgeGrasp-owned distal-pad child links:

```bash
ros2 service call /edgegrasp/set_target_pad_contacts \
  std_srvs/srv/SetBool "{data: true}"
ros2 topic echo --once --full-length /edgegrasp/planning_scene_status
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene \
  "{components: {components: 152}}"
```

Require `ready=true`, `reason=confirmed`, both `edgegrasp_table` and
`edgegrasp_target_cube`, and an ACM target row with exactly these task-specific
permissions:

```text
edgegrasp_fixed_finger_pad_link:  true
edgegrasp_moving_finger_pad_link: true
gripper_link:                     false
moving_jaw_so101_v1_link:         false
```

Do not allow either parent link merely to make a grasp path pass. With the
candidate004 pose from `so101_side_grasp_candidate.json`, the read-only probes
are:

```bash
python3 scripts/probe_so101_joint_plan.py \
  --label candidate004_approach_to_descend_pad_only \
  --start -0.6000000524768894 -0.4249611950204053 \
    -0.04600904468898089 0.47097024253202446 -1.5708000280600944 \
  --start-gripper 1.5 \
  --target-position 0.2462364349184608 0.15125054509958977 \
    0.21671404504175756 \
  --target-orientation 0.3450298741758752 0.6172118344266647 \
    0.6332767577280262 0.3145862131297726 \
  --pipeline-id pilz_industrial_motion_planner --planner-id PTP \
  --allowed-planning-time-s 2 --velocity-scaling .1 \
  --acceleration-scaling .1

python3 scripts/probe_so101_joint_plan.py \
  --label candidate004_descend_to_lift_pad_only \
  --start -0.6000000442835862 -0.8693352717379274 \
    0.8640595639082524 0.005275710769147334 -1.5708000280690093 \
  --start-gripper 0.60 \
  --target-position 0.2462364349184608 0.15125054509958977 \
    0.25671404504175754 \
  --target-orientation 0.3450298741758752 0.6172118344266647 \
    0.6332767577280262 0.3145862131297726 \
  --pipeline-id pilz_industrial_motion_planner --planner-id PTP \
  --allowed-planning-time-s 2 --velocity-scaling .1 \
  --acceleration-scaling .1
```

The recorded run rejected approach-to-descend at states 27-42/51 for cube
versus `gripper_link`, and accepted the 22-point closed-gripper lift control.
Both commands are service-only probes: they publish no trajectory and perform
no execution. Candidate004 must remain blocked until a new descend pose passes
with the same retained-cube and forbidden-parent contract. See
`docs/observations/2026-08-27-candidate004-selective-acm-plan-only.json`.

### D5. Candidate005 proxy-MoveGroup typed trial

Candidate005 replaces only the approach route/orientation; descend and lift
keep the candidate004 grasp orientation. Start MoveGroup with the EdgeGrasp
proxy overlay, never the pinned package directly:

```bash
ros2 launch edgegrasp_ros edgegrasp_proxy_move_group.launch.py \
  robot_name:=so101 use_camera:=false use_gazebo:=true \
  use_sim_time:=true use_rviz:=false
ros2 launch edgegrasp_ros planning_scene.launch.py \
  use_sim_time:=true clock_domain:=ros_sim clock_epoch:=0 \
  target_frame:=base_link include_optional_cube:=true
```

Before motion, require one `/clock` publisher, all three controllers active,
both FJT actions, all four EdgeGrasp actions, both distal-pad links in
MoveGroup's `robot_description`, `ValidateSolution` loaded, one PlanningScene
loader, and a confirmed selective ACM:

```bash
ros2 service call /edgegrasp/set_target_pad_contacts \
  std_srvs/srv/SetBool "{data: true}"
ros2 topic echo --once --full-length /edgegrasp/planning_scene_status
```

Open the gripper through the typed gate; this is not a direct FJT diagnostic:

```bash
python3 scripts/prepare_so101_trial.py --gripper-only --gripper 1.5 \
  --target-id target_cube --task-id candidate005-prep-UNIQUE --timeout-s 20
```

Only after those gates pass, the exact simulation-only candidate payload is:

```bash
ros2 run edgegrasp_grasp_sequence grasp_trial_client --ros-args \
  -p use_sim_time:=true -p clock_domain:=ros_sim -p clock_epoch:=0 \
  -p task_id:=candidate005-UNIQUE -p target_id:=target_cube \
  -p approach_position_m:="[0.18606933614192925,0.11976423272744036,0.38721872836424076]" \
  -p descend_position_m:="[0.2462364349184608,0.15125054509958977,0.21671404504175756]" \
  -p lift_position_m:="[0.2462364349184608,0.15125054509958977,0.25671404504175754]" \
  -p approach_orientation_xyzw:="[0.19946281649811778,0.38019026690132723,0.8150060012713882,0.38914671228183106]" \
  -p grasp_orientation_xyzw:="[0.3450298741758752,0.6172118344266647,0.6332767577280262,0.3145862131297726]" \
  -p gripper_closed_position_rad:=0.60 \
  -p pipeline_id:=pilz_industrial_motion_planner -p planner_id:=PTP \
  -p planning_timeout_s:=2.0 -p sequence_timeout_s:=90.0 \
  -p physics_timeout_s:=100.0 -p observation_timeout_s:=30.0 \
  -p velocity_scaling:=0.1 -p acceleration_scaling:=0.1
```

One clean attempt observed a 1 ms `/clock` rollback and correctly SAFE_STOPped;
do not reset or retry inside that latched graph. Exact-PID cleanup followed by
one fresh-domain retry completed the command sequence. That retry recorded 35
samples containing both pad tokens but only 0.332 mm peak cube lift and no
retention, so exit 11 and `physics_grasp_verified=false` are the correct
outcome. Same-sample contact is not force closure. See
`docs/observations/2026-08-27-candidate005-simultaneous-proxy-runtime.json`.

For the instrumented replay of the same candidate, use a fresh domain and an
empty artifact directory. The script validates exact target ID/scene position,
waits for `motion_allowed=true` and `interface_ready=true`, sends preparation
and sequence goals only through typed EdgeGrasp actions, and cleans only exact
`ROS_DOMAIN_ID` processes:

```bash
cd /home/edgegrasp/ros2_ws/src/edgegrasp-sim
bash scripts/run_candidate005_contact_quality.sh 51 \
  /home/edgegrasp/ros2_ws/test_results/candidate007_contact_timing_UNIQUE \
  candidate007-contact-timing-UNIQUE
```

Candidate007 measured 39 same-sample two-pad messages over 0.099 simulated
seconds. Their final timestamp was 1.747 seconds before sequence completion;
the moving pad's later one-sided contact ended 1.400 seconds before completion.
Cube lift was 0.359 mm, retention was false, and exit 11 remained correct. See
`docs/observations/2026-08-27-candidate007-contact-timing-runtime.json`.

Do not merely substitute another `gripper_closed_position_rad` in this command.
The current candidate profile's OBB overlap is locked to 0.60 rad. Generate and
statically validate a new profile for a new angle first, then change exactly one
runtime variable. The helper below may be used for scoped cleanup only after
checking its printed exact-domain PID list:

Candidate008 followed that rule. Its checked-in profile was generated from the
fixed-SHA kinematic contract at 0.50 rad and the harness was given both the
matching profile basename and command value:

```bash
python3 scripts/generate_grasp_geometry_candidate.py \
  --source-profile ros_ws/src/edgegrasp_ros/config/so101_grasp_geometry.json \
  --proxy-contract ros_ws/src/edgegrasp_ros/config/so101_collision_proxies.json \
  --gripper-position-rad 0.50 \
  --name edgegrasp_so101_target_cube_grasp_geometry_candidate008_q0p50

bash scripts/run_candidate005_contact_quality.sh 52 \
  /home/edgegrasp/ros2_ws/test_results/candidate008_q0p50_20260827_2105 \
  candidate008-q0p50-001 \
  so101_grasp_geometry_candidate008_q0p50.json \
  0.50
```

The generator prints JSON to stdout and does not write a profile implicitly.
The checked-in profile is guarded by a byte-equivalent JSON test and exact OBB
preflight. The runtime produced 98 simultaneous pad samples over 0.252
simulated seconds and 2.667 mm peak lift, but contact ended 1.788 seconds before
sequence completion and retention failed. See
`docs/observations/2026-08-27-candidate008-q0p50-runtime.json`. Do not interpret
the improved transient metrics as a grasp or automatically try a still tighter
angle; the measured lateral drift calls for a pose/contact-symmetry or bounded
friction study first.

### D6. Chained candidate plan-only admission and the 0.45/0.40 controls

Before executing any newly generated geometry profile, run the isolated
three-segment admission harness in a fresh ROS domain and artifact directory:

```bash
cd /home/edgegrasp/ros2_ws/src/edgegrasp-sim
bash scripts/run_grasp_candidate_plan_only.sh \
  61 \
  /home/edgegrasp/ros2_ws/test_results/candidate010_q0p45_plan_only_UNIQUE \
  candidate010-q0p45-plan-only \
  so101_grasp_geometry_candidate010_q0p45.json \
  10 0.0
```

This harness launches Gazebo, the EdgeGrasp proxy MoveGroup overlay, and the
confirmed shared PlanningScene only. It deliberately does not launch the
MoveIt adapter, trajectory gate, or grasp sequence. Its Python probe calls only
`/plan_kinematic_path`, validates and discards each returned RobotTrajectory,
and chains the terminal joint state through home-to-approach,
approach-to-descend, and descend-to-lift. It reports zero trajectory
publication, ExecuteTrajectory goals, FJT goals, and execution attempts.

Candidate009 used the optional final argument to scan tool-X offsets while all
other parameters remained fixed. The zero-offset control passed all three
segments 10/10 with one digest per segment. The first -0.1 mm step and every
sample through -0.9 mm failed approach-to-descend with `NO_IK_SOLUTION` and
zero execution. Do not execute that static symmetry optimum. See
`docs/observations/2026-08-27-candidate009-reachability-scan.json`.

The next two one-variable profiles kept offset `0.0` and changed only the
generated close geometry/command:

```bash
# Plan-only first; both recorded profiles passed every segment 10/10.
bash scripts/run_grasp_candidate_plan_only.sh 61 \
  /home/edgegrasp/ros2_ws/test_results/candidate010_q0p45_plan_only_UNIQUE \
  candidate010-q0p45-plan-only \
  so101_grasp_geometry_candidate010_q0p45.json 10 0.0

bash scripts/run_grasp_candidate_plan_only.sh 63 \
  /home/edgegrasp/ros2_ws/test_results/candidate011_q0p40_plan_only_UNIQUE \
  candidate011-q0p40-plan-only \
  so101_grasp_geometry_candidate011_q0p40.json 10 0.0
```

Only after those gates, the existing typed runtime harness was invoked with
the matching basename and close command (`0.45` then `0.40`). Candidate010
reached 3.323 mm peak lift and candidate011 reached 3.711 mm, but both returned
the cube to table height and failed retention. Candidate011 bilateral contact
lasted 0.463 simulated seconds and ended 1.737 seconds before completion. Stop
the close-angle search here; the next bounded experiment changes pad
friction/contact material or distal-pad proxy geometry while preserving the
same plan-only and 20 mm plus retention gates. See
`docs/observations/2026-08-27-candidate010-q0p45-runtime.json` and
`docs/observations/2026-08-27-candidate011-q0p40-runtime.json`.

Candidate012 freezes that 0.40 rad geometry and changes only the generated
finger-pad friction profile. Always run the no-execution matrix first:

```bash
bash scripts/run_candidate012_plan_only.sh 70 /tmp/c012-implicit \
  candidate012-implicit-plan implicit_default
bash scripts/run_candidate012_plan_only.sh 71 /tmp/c012-control-plan \
  candidate012-control-plan candidate012_control_mu1p0
bash scripts/run_candidate012_plan_only.sh 72 /tmp/c012-treatment-plan \
  candidate012-treatment-plan candidate012_treatment_mu1p5
```

Only after every row reports 10/10 and all motion-boundary counters remain zero,
run the explicit paired rows in fresh domains and unique artifact directories:

```bash
bash scripts/run_candidate012_pad_friction.sh 81 \
  /home/edgegrasp/ros2_ws/test_results/c012-control-UNIQUE \
  c012-control-UNIQUE candidate012_control_mu1p0
bash scripts/run_candidate012_pad_friction.sh 82 \
  /home/edgegrasp/ros2_ws/test_results/c012-treatment-UNIQUE \
  c012-treatment-UNIQUE candidate012_treatment_mu1p5
```

The proxy launch starts `/clock`, safety, and the trajectory gate before a
3-second delayed synthetic target publisher. This prevents a startup callback
ordering race without weakening future-target rejection. Abort if any unrelated
Gazebo process is present; it can starve the observer's 200 ms evidence window.
The recorded paired run reached 3.392/3.468 mm peak lift for `mu=1.0/1.5`, and
neither row retained the cube. See
`docs/observations/2026-08-28-candidate012-pad-friction-runtime.json`.

### D7. Candidate024 face-aligned control and stale-baseline recovery

Candidate024 is the current fixed simulation control. It was derived from the
measured Candidate022 contact geometry: rotate the cube by 37.243 degrees so a
face is aligned with the horizontal closing-axis projection, then translate it
9.7 mm to restore a 1.002 mm fixed-pad pre-close gap. The reachable arm stage
poses, 0.40 rad close command, explicit `mu=mu2=1.0`, zero distal extension,
and -0.02 Nm simulated preload remain fixed.

Do not execute it before the isolated three-segment plan-only gate. Use a fresh
domain, task ID, and artifact directory every time:

```bash
cd /home/edgegrasp/ros2_ws/src/edgegrasp-sim
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash

bash scripts/run_grasp_candidate_plan_only.sh \
  151 \
  /home/edgegrasp/ros2_ws/test_results/candidate024_plan_only_UNIQUE \
  candidate024-face-aligned-plan-only \
  so101_grasp_geometry_candidate024_face_aligned_q0p40.json \
  10 0.0 candidate012_control_mu1p0 0.0 \
  so101_side_grasp_candidate024_face_aligned.json \
  scene_candidate024_face_aligned.json \
  table_cube_candidate024_face_aligned.sdf
```

Only after that artifact reports 10/10 attempts, 30/30 accepted segments, and
zero trajectory publication, ExecuteTrajectory goals, FJT goals, and execution
may one bounded typed runtime be started:

```bash
bash scripts/run_candidate005_contact_quality.sh \
  152 \
  /home/edgegrasp/ros2_ws/test_results/candidate024_runtime_UNIQUE \
  candidate024-runtime-UNIQUE \
  so101_grasp_geometry_candidate024_face_aligned_q0p40.json \
  0.40 true candidate012_control_mu1p0 0.0 \
  effort_pid_preload -0.02 \
  so101_side_grasp_candidate024_face_aligned.json \
  scene_candidate024_face_aligned.json \
  table_cube_candidate024_face_aligned.sdf
```

The trial client binds physics observation and sequence execution to one exact
target snapshot. After the five-sample read-only baseline it rechecks that
snapshot against a 100 ms admission limit; it does not weaken the core 200 ms
deadline. An older snapshot is never sent. The observer may be rebuilt at most
twice, and only after cancel acceptance plus a correlated CANCELED wrapper
terminal whose reason, task, target, source timestamp, clock domain, and epoch
all match. Timeout, rejection, exception, late feedback from an older attempt,
or any identity mismatch terminates with zero motion.

The accepted observation contains 10 scoped physics-grasp results across 11
runs; the one failure was the pre-fix stale-target SAFE_STOP before motion, and
post-fix r03-r11 passed 9/9. r04 exercised the actual correlated observer
restart. Successful retained lift was 28.858-29.128 mm with both configured
distal-pad contacts and a 0.5 s retention window. This is evidence for one
fixed target pose in the primitive-proxy Gazebo model, not physics
determinism, full collision fidelity, force closure, or hardware grasp. See
`docs/observations/2026-08-29-candidate024-face-aligned-runtime.json`.

```bash
bash scripts/cleanup_ros_domain.sh 51          # read-only listing
bash scripts/cleanup_ros_domain.sh 51 --terminate
```

### D8. Candidate024 distinct-target plan-only matrix

This matrix is the next admission layer, not a batch execution command. It
generates ten local targets through the pinned shoulder-pan joint transform,
four deliberately invalid scene contracts, and six on-table distant targets.
Every valid case gets a fresh ROS domain and an isolated Gazebo/MoveIt graph;
all returned trajectories are validated and discarded.

```bash
cd /home/edgegrasp/ros2_ws/src/edgegrasp-sim
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash

python3 scripts/run_candidate024_target_matrix_plan_only.py \
  --matrix ros_ws/src/edgegrasp_ros/config/candidate024_target_matrix.json \
  --artifact-dir /home/edgegrasp/ros2_ws/test_results/candidate024_target_matrix_UNIQUE \
  --ros-domain-id-start 180
```

The starting domain plus 15 must remain at most 232. The artifact path must be
fresh and remain under `/home/edgegrasp/ros2_ws/test_results`. A passing result
requires `PLAN_ONLY_MATRIX_PASS`, 20 matched outcomes, zero false accepts,
false rejects, unverified cases, and motion-side-effect violations. The
2026-08-29 accepted artifact met those gates with 10/10 positive targets,
4/4 scene-contract rejections, and 6/6 MoveIt rejections. See
`docs/observations/2026-08-29-candidate024-target-matrix-plan-only.json`.

Do not send all ten accepted trajectories. The next bounded step is to choose
only three representative positives (both range endpoints and one near-control
case), regenerate their exact scene/candidate/world files, then route each once
through the existing typed sequence. Plan-only acceptance is not execution,
contact, lift, retention, or grasp evidence.

For repeatability, do not background `ros2 run`: that owns a CLI wrapper rather
than reliably owning the child action-server process. The checked script
resolves and starts the installed console-script entry points directly, uses a
unique action endpoint per run, and rejects any dirty shutdown:

```bash
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash
cd /home/edgegrasp/ros2_ws/src/edgegrasp-sim
bash scripts/run_grasp_sequence_repetitions.sh 10 \
  /home/edgegrasp/ros2_ws/test_results/grasp_repeat_direct_final_20260827_0520 \
  grasp-direct-accept \
  '[0.391231968, -0.001571668, 0.256520737]' \
  '[0.391231968, -0.001571668, 0.251520737]' \
  '[0.391231968, -0.001571668, 0.261520737]' \
  '[0.017007859, 0.706463960, 0.013976791, 0.707406570]' \
  0.2
```

The accepted 2026-08-27 run produced 10/10 `sequence_completed=true`, ten
clean per-run sequence shutdowns, and zero physics-success claims. Its summary
SHA-256 is
`8658ff43a9974edec6f6c69934f37052e530ebff6fb1891a32c899268d38c4da`.
Because all ten final trajectory digests differ, report this only as protocol
outcome repeatability. Exact retained failure/invalid batches are in
`docs/observations/2026-08-27-grasp-sequence-repeatability.json`.

## E. MCAP record/replay modes

Recording intentionally includes raw inputs plus derived/status/command topics,
cube pose/contact, sequence terminal, and physics status for later comparison.
Default replay excludes every derived/evidence/command topic. Sim recording
must use one sim clock:

```bash
# Gazebo is the sole /clock authority; waits for /clock and adds --use-sim-time.
bash scripts/record_mcap.sh /path/to/bag ros_sim

# Mock/system-clock graph; does not add --use-sim-time.
bash scripts/record_mcap.sh /path/to/bag system

ros2 bag info /path/to/bag
python3 scripts/validate_mcap_time_domain.py /path/to/bag
```

Default `SAFETY_REPLAY` is raw-input-only:

1. Stop Gazebo, MoveIt, controllers, and every other `/clock` publisher.
2. Start wrapper/gate only: `ros2 launch edgegrasp_ros edgegrasp_replay.launch.py`.
3. In another shell run `bash scripts/replay_mcap.sh /path/to/bag`.

The script preflights `/clock`, aborts if a publisher exists, lets rosbag be
the sole clock authority, and plays only `/joint_states`, target, and three raw
camera topics. Recorded permission, status, interface, and trajectory-request
topics are intentionally excluded. Jazzy `ros2 bag play --clock` excludes the
bag's recorded `/clock` from that same player; the conflict to prevent is an
external publisher such as Gazebo.

Restart `edgegrasp_replay` or explicitly coordinate a new epoch before playing
the same bag again. Rewinding the bag clock into an unchanged wrapper correctly
latches clock rollback. This replay has no planner, MoveIt adapter, backend, or
physics.

`GAZEBO_RUN` uses Gazebo as sole `/clock` authority and never combines it with
`ros2 bag play --clock`. `OUTPUT_INSPECTION` may play all topics only manually
in an isolated graph with no live monitor, gate, planner, controller, or
backend; it is never the default closed-loop mode.

## Observation log template

For each runtime layer, record:

```text
command:
started_at (timezone):
ended_at (timezone):
exit_code:
stdout/stderr artifact:
expected:
observed:
status: PASS | FAIL | PARTIAL | UNVERIFIED
claim boundary:
```

Do not upgrade a static path, package prefix, or launch-file presence into a
runtime result.
