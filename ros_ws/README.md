# EdgeGrasp ROS 2 overlay

This workspace contains only EdgeGrasp-owned adapters. It does not copy the
pinned SO-101 Gazebo or MoveIt packages.

## Package layout

- `edgegrasp_core`: monorepo ament adapter for the single source tree at
  `../../src/edgegrasp` relative to `ros_ws/src/edgegrasp_core`. It is not a
  standalone-copyable package.
- `edgegrasp_interfaces`: atomic `TrackedTarget.msg`, immutable
  `PlanTarget.action`, correlated `ExecuteTrajectory.action`, and immutable
  `GraspSequence.action` payloads.
- `edgegrasp_ros`: mock target, ROS-clock safety monitor, interface probe,
  FollowJointTrajectory command gate, launches, and table/cube SDF.
- `edgegrasp_moveit_adapter`: tf2 + bounded IK + MoveGroup `plan_only` bridge
  that submits only to the typed EdgeGrasp trajectory-gate action.
- `edgegrasp_grasp_sequence`: dependency-free core adapter that serializes
  approach, descend, separate gripper close, and lift through the two typed
  actions and waits for exact terminal correlation before advancing.

Keep or copy the entire `edgegrasp-sim` project into a ROS workspace. Copying
only `edgegrasp_core` intentionally fails because it would lose the canonical
core source. The exact build, isolation, diagnostics, MoveIt, and MCAP commands
are in the [Ubuntu/Jazzy runbook](../docs/ubuntu-jazzy-runbook.md).

## Verified build/test commands

```bash
cd "$HOME/ros2_ws"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
python3 -c 'import edgegrasp; print(edgegrasp.DEFAULT_TARGET_FRAME)'
colcon test --test-result-base "$HOME/ros2_ws/test_results/edgegrasp_scoped" \
  --packages-select edgegrasp_core edgegrasp_interfaces \
  edgegrasp_ros edgegrasp_moveit_adapter edgegrasp_grasp_sequence \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result \
  --test-result-base "$HOME/ros2_ws/test_results/edgegrasp_scoped" \
  --all --verbose
```

These commands ran in Ubuntu 24.04 WSL2 with ROS 2 Jazzy. The latest exact
counts and limitations are in the [validation report](../docs/validation-report.md).

## Safety and planning paths

```text
/edgegrasp/target_3d
  -> safety_monitor (source stamp + local stream watchdog)
  -> /edgegrasp/motion_allowed

PlanTarget.action immutable pose
  -> timestamped tf2 to base_link
  -> /compute_ik with fresh start state
  -> /move_action, planning_options.plan_only=true
  -> separate RobotTrajectory validator
  -> /edgegrasp/execute_trajectory (controller=arm_controller)
  -> trajectory_gate
  -> /arm_controller/follow_joint_trajectory
```

Gripper commands stay separate:

```text
/edgegrasp/execute_trajectory (controller=gripper_controller)
  -> trajectory_gate
  -> /gripper_controller/follow_joint_trajectory
```

The action-level orchestrator is:

```text
/edgegrasp/grasp_sequence
  -> immutable normalized approach orientation
  -> immutable normalized grasp orientation for descend/lift
  -> approach PlanTarget terminal
  -> descend PlanTarget terminal
  -> gripper ExecuteTrajectory terminal
  -> lift PlanTarget terminal
  -> COMPLETE (protocol terminal only)
  -> independent GraspPhysicsEvidence terminal
```

`GraspSequence` never declares physics success. The independent observer may
set `physics_grasp_verified=true` only after the matching lift command, both
configured distal-pad contacts, the configured lift threshold, and the full
retention window all correlate.

The two `*_joint_trajectory_request` topics are retained only as legacy
diagnostic adapters. New planning and sequence code must use the typed action
so task/command/target/stage/sequence, trajectory digest, source timestamp,
domain, epoch, and the downstream terminal result remain correlated.

The adapter never calls MoveIt execution and never writes directly to the
upstream controller action. Core `MotionPlan` remains a Cartesian deterministic
model; MoveIt `RobotTrajectory` remains a ROS joint-space model.

Both gates require `base_link`, explicit `ros_system` or `ros_sim` clock
policy, local epoch, recent permission, recent target stream, recent exact
joint state, exact joint order, finite arrays, conservative project position,
duration, velocity, and acceleration limits, and ready downstream interfaces.
Cancellation failure, or an accepted cancel without a terminal result, latches
an error, retries at most three times, and then requires controller-level stop
escalation; a cancel request is not claimed as a guaranteed physical stop.

`PointStamped` cannot carry clock domain/epoch. The wrapper assigns a local
epoch; cross-node epoch propagation remains a future shared-interface task.
`PlanTarget.action` carries the immutable domain/epoch snapshot, but receiving
wrappers still own reset coordination.

## Launches

ROS-only mock/system clock:

```bash
ros2 launch edgegrasp_ros edgegrasp_mock.launch.py \
  backend:=mock require_camera:=false use_sim_time:=false
```

Pinned Gazebo proxy overlay after upstream is built:

```bash
ros2 launch edgegrasp_ros edgegrasp_proxy_gazebo.launch.py \
  use_camera:=false use_rviz:=false launch_edgegrasp_nodes:=true
```

Start pinned MoveGroup through the unique EdgeGrasp thin overlay. It disables
direct MoveGroup execution, keeps the upstream arm collision model, adds two
distal contact boxes, and ensures Pilz calls `ValidateSolution`:

```bash
ros2 launch edgegrasp_ros edgegrasp_proxy_move_group.launch.py \
  robot_name:=so101 use_camera:=false use_gazebo:=true \
  use_sim_time:=true use_rviz:=false
ros2 launch edgegrasp_ros planning_scene.launch.py \
  use_sim_time:=true clock_domain:=ros_sim clock_epoch:=0 \
  target_frame:=base_link include_optional_cube:=true
```

MoveIt adapter after controllers, move_group, and confirmed PlanningScene are
healthy:

```bash
ros2 launch edgegrasp_moveit_adapter moveit_adapter.launch.py \
  use_sim_time:=true clock_domain:=ros_sim planning_frame:=base_link
```

Sequence wrapper after the typed target publisher, monitor, interface probe,
adapter, and trajectory gate are healthy:

```bash
ros2 launch edgegrasp_grasp_sequence grasp_sequence.launch.py \
  use_sim_time:=true clock_domain:=ros_sim target_frame:=base_link
```

This launch has package/fake-action evidence and scoped runtime evidence against
Gazebo, pinned MoveIt, the typed gate, and both FJT controllers. The corrected
direct-entrypoint harness completed 10/10 repeated protocol runs with clean
per-run sequence shutdown. The runs did not observe cube contact, lift, or
retention; their ten trajectory digests differed. This is protocol-outcome
repeatability, not byte-identical planning or a physics-grasp claim. See
`../docs/observations/2026-08-27-grasp-sequence-repeatability.json`.

The later collision-validated task039 observed `gripper_link`/cube contact but
only 2.27 mm peak lift, no retention, and about 60 mm lateral cube displacement.
It remains `physics_grasp_verified=false`; see
`../docs/observations/2026-08-27-pilz-collision-contact-runtime.json`.

Candidate003 then tightened the observer to exact distal-pad proxy names. Its
far approach kept the cube fixed and all four correlated stages reached FJT
success, but the observer found zero distal-pad contacts, only 2.43 mm peak
lift, no retention, and about 52.3 mm lateral displacement. See
`../docs/observations/2026-08-27-candidate003-runtime.json`. This is sequence
success plus stronger negative physics evidence, not a grasp.

Candidate004 replaces the earlier whole-target removal with a selective ACM:
the target cube stays in MoveIt, only
`edgegrasp_fixed_finger_pad_link` and
`edgegrasp_moving_finger_pad_link` may contact it, and the two parent links
remain forbidden. Runtime inspection confirmed that exact matrix. Pilz then
rejected the current approach-to-descend path for cube versus `gripper_link`
collision, while the closed-gripper descend-to-lift control passed plan-only.
Nothing was executed. See
`../docs/observations/2026-08-27-candidate004-selective-acm-plan-only.json`.

Candidate005 uses a distinct collision-routing approach quaternion and keeps
the grasp quaternion for descend/lift. In a clean proxy-MoveGroup runtime it
completed all four correlated stages. The observer counted 1,125 fixed-pad and
188 moving-pad contacts, including 35 samples containing both configured pad
tokens. Peak lift was only 0.332 mm and no retention window completed, so this
is sequence success plus simultaneous-contact evidence—not force closure or a
successful simulated grasp. The preceding attempt's 1 ms `/clock` rollback
correctly latched SAFE_STOP. See
`../docs/observations/2026-08-27-candidate005-simultaneous-proxy-runtime.json`.

Candidate006 kept those parameters and added per-pad contact depth, Gazebo/DART
wrench maxima, and `/joint_states` gripper effort. Candidate007 added source
timestamps for each pad, same-sample bilateral contact, sequence completion,
and effort-at-completion. In the clean candidate007 graph, bilateral contact
lasted 0.099 simulated seconds and ended 1.747 seconds before the sequence
terminal; peak cube lift was only 0.359 mm and retention was false. The exact
records are
`../docs/observations/2026-08-27-candidate006-contact-quality-runtime.json` and
`../docs/observations/2026-08-27-candidate007-contact-timing-runtime.json`.
These fields are diagnostic only: they do not establish calibrated force,
force closure, or physics grasp success.

Candidate008 used a separately generated 0.50 rad geometry profile and the
matching close command. It increased simultaneous contact to 98 samples over
0.252 simulated seconds and transient peak lift to 2.667 mm, but bilateral
contact ended 1.788 seconds before the sequence terminal, the cube returned to
table height, and retention remained false. See
`../docs/observations/2026-08-27-candidate008-q0p50-runtime.json`. The result is
a stronger negative/control comparison; it is not a physics grasp.

The generic isolated preflight is:

```bash
bash scripts/run_grasp_candidate_plan_only.sh \
  61 /home/edgegrasp/ros2_ws/test_results/plan_only_UNIQUE \
  so101_grasp_geometry_candidate010_q0p45.json
```

It launches only Gazebo, proxy MoveGroup, and the confirmed PlanningScene. Its
probe calls `/plan_kinematic_path` for the chained home-to-approach,
approach-to-descend, and descend-to-lift segments. It does not launch the
adapter, trajectory gate, or sequence and cannot publish a trajectory or send
ExecuteTrajectory/FJT goals. Candidate009 used this gate to reject every
sampled negative tool-X offset (-0.1 through -0.9 mm) with
`NO_IK_SOLUTION`, while the unchanged zero-offset control passed all segments
10/10. See
`../docs/observations/2026-08-27-candidate009-reachability-scan.json`.

Candidates010/011 kept that reachable zero-offset arm path, generated matching
0.45/0.40 rad gripper profiles, and each passed the isolated three-segment gate
10/10 before one typed execution. Candidate011 is the strongest transient
result: 171 bilateral samples over 0.463 simulated seconds and 3.711 mm peak
lift. It still lost contact 1.737 seconds before completion, returned the cube
to the table, and failed retention. The close-angle search is stopped; next use
a bounded friction/contact-material or distal-pad proxy-geometry experiment.
See
`../docs/observations/2026-08-27-candidate010-q0p45-runtime.json` and
`../docs/observations/2026-08-27-candidate011-q0p40-runtime.json`.

Candidate012 keeps Candidate011 geometry and uses the generated material
profiles `candidate012_control_mu1p0` and `candidate012_treatment_mu1p5`.
All three material rows passed the plan-only chain 10/10; paired typed runtime
then completed both sequences but produced only 3.392/3.468 mm peak lift and
no retention. The treatment delta was +0.076 mm, so the bounded friction sweep
does not establish friction as the current bottleneck. The proxy launch delays
the mock target by 3 s so the `/clock` consumer is initialized before the first
sample; future timestamps are still denied. See
`../docs/observations/2026-08-28-candidate012-pad-friction-runtime.json`.

Candidate024 aligns a cube face with the measured closing axis while retaining
the known-reachable arm poses, 0.40 rad close command, explicit `mu=mu2=1.0`,
and -0.02 Nm simulated preload. It passed the isolated three-segment plan-only
gate 10/10 with zero execution. Across r01-r11, 10 runtime attempts met the
strict bilateral-contact, 20 mm lift, and 0.5 s retention contract; r02 stopped
before motion because its target became 208 ms old at the planning boundary.
The corrected trial client now rechecks the exact target at 100 ms before
submission. A stale baseline can restart only after a correlated CANCELED
observer terminal; r04 exercised that path and the post-fix r03-r11 set passed
9/9. The current package-scoped Jazzy result is core 1/1, interfaces 0 tests,
ROS 49/49, MoveIt adapter 23/23, and sequence 45/45. See
`../docs/observations/2026-08-29-candidate024-face-aligned-runtime.json`.
This remains one proxy scene and is not hardware grasp evidence.

Raw-input safety replay, with Gazebo/controllers stopped and no `/clock`
publisher:

```bash
ros2 launch edgegrasp_ros edgegrasp_replay.launch.py
# separate shell
bash scripts/replay_mcap.sh /path/to/bag
```

`edgegrasp_replay` contains only the safety monitor and trajectory gate. It has
no planner, MoveIt adapter, backend, controller, or physics. Evidence/status
topics may be recorded but are excluded from default playback.

Keep the three clock modes separate: `SAFETY_REPLAY` is the raw-input command
above with rosbag as sole `/clock` publisher; `GAZEBO_RUN` gives Gazebo sole
clock authority and never uses playback `--clock`; `OUTPUT_INSPECTION` permits
manual all-topic playback only in an isolated graph with no live monitor, gate,
planner, controller, or backend.

## Pinned external interface

```text
arm joints: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll
gripper joints: gripper
feedback: /joint_states
arm action: /arm_controller/follow_joint_trajectory
gripper action: /gripper_controller/follow_joint_trajectory
type: control_msgs/action/FollowJointTrajectory
```

Direct controller actions and probable JTC topics bypass EdgeGrasp safety and
are diagnostic-only. Never use them as stale-target, collision, planning, or
grasp evidence.

## Workspace isolation

Do not source adoodevv and legalaspro overlays together; they reuse SO-101
package names. Keep one disposable Jazzy Gazebo workspace for the pinned
adoodevv source plus the complete EdgeGrasp project, and a different future
workspace for legalaspro mock/real evaluation.
