# EdgeGrasp / SO-101 simulation plan

## 1. Evidence vocabulary and current state

This project uses five non-interchangeable states:

- **Verified locally**: the named command ran on this machine and the result was
  inspected.
- **Structurally verified**: dependency-free parsing, import, packaging, or
  configuration checks passed, but the target runtime did not run.
- **Runtime verified (scoped)**: the named process or interface ran; only the
  explicitly observed scope is claimed.
- **Written, runtime unverified**: implementation exists but its external
  runtime or integration test was unavailable.
- **Hardware-only**: simulation cannot establish the claim.

Snapshot on 2026-08-30. The core and simulator counts below remain from the
2026-08-29 snapshot; the 2026-08-30 addition is limited to the in-process
fail-closed artifact in the MoveIt matrix:

| Capability | Status | Defensible claim |
| --- | --- | --- |
| Target model, predictor, state machine, safety/controller contracts, mock backend | Verified locally | Pure Python plus project structure passed 332 current CPython 3.14.5 project tests; all three 100-run replay scenarios passed |
| Core deterministic replay | Verified locally | 100 identical replays for each synthetic 0/20/40 mm/s scenario |
| Four-stage grasp sequence | Runtime verified (scoped) | 60 dependency-free tests plus 45 Jazzy action/client tests pass; the fixed Candidate024 control produced 10 simulation-physics successes across 11 attempts, and three selected distinct target poses each completed one bounded typed run |
| Standard Python install/import | Verified locally | The root package installs into a clean target and imports without source `PYTHONPATH` |
| ROS packages and safety/trajectory gate | Runtime verified (scoped) | Jazzy colcon plus 49 ROS tests; typed target publication/correlation, selective PlanningScene/ACM confirmation, all-pad physics observation, and dependency-injected fake-clock/permission/watchdog/cancel paths observed |
| Correlated target observation | Runtime verified (mock scope) | `TrackedTarget` and legacy `PointStamped` were emitted with matching stamp/point; target ID/domain/epoch were preserved |
| MoveIt plan-only adapter | Runtime verified (scoped) | 23 Jazzy tests with fake IK/MoveGroup failure dependencies, plus separately classified real-process normal planning and historical-negative evidence |
| Gazebo SO-101 control | Runtime verified (scoped) | Three active controllers, six joint states, two FJT endpoints and gated results observed |
| Pinned upstream MoveIt configuration | Static and runtime planning verified | The EdgeGrasp thin overlay uses pinned move_group, adds Pilz `ValidateSolution`, exposes two distal-pad links, disables direct MoveGroup execution, and completed candidate005 through the typed path |
| Shared Gazebo/MoveIt table/cube scene | Runtime verified (scoped) | One contract generates the Gazebo SDF; MoveIt confirmed table+cube and a selective ACM that permits target contact only for two dedicated pad child links while keeping both parent links forbidden |
| MCAP single-clock record/replay | Runtime verified (wrapper scope) | ros_sim record, time-domain check, raw-input replay, and clock-conflict guard ran |
| Pinned SO-101 MJCF | Original XML/joint/actuator/mesh structure verified | Windows MuJoCo runtime blocked before model-specific execution |
| Camera topics | Runtime verified (topic scope) | Simulated color/depth/info published; no calibration/localization claim |
| Primitive-proxy contact | Runtime verified (narrow scope) | Generated primitives replaced 13 collision meshes; Candidate024's 14 fixed-plus-selected runtime logs had zero tracked DART mesh/geometry diagnostics, and 13 successful runs retained both configured pad contacts through lift |
| Physics grasp observer | Runtime verified (scoped positive evidence) | Candidate024 produced 13 correlated `physics_grasp_verified=true` results: ten at the fixed control and one at each of three selected distinct poses, with 28.858-29.128 mm retained lift and the configured 0.5 s retention; one fixed-control pre-fix attempt stopped before motion on stale target |
| Full collision fidelity and real hardware | Unverified/future | Three selected shoulder-pan-symmetry target poses and conservative proxy geometry are covered; per-pose repeatability, workspace-wide behavior, all-link fidelity, force calibration, real stops, and hardware remain unverified |

MoveIt runtime evidence uses three additional non-interchangeable classes. The
two fail-closed tracks are shown explicitly because their results answer
different questions:

| Evidence class | Status | Defensible claim |
| --- | --- | --- |
| `REAL_MOVEGROUP_NORMAL` | `PASS_SCOPED` | The Candidate024 target matrix used the real MoveGroup process under the EdgeGrasp proxy overlay for normal `GetMotionPlan`; 20/20 outcomes matched, accepted trajectories were validated/discarded, and all execution counters were zero. This is normal planning only, not cancel health. |
| `B · INJECTED_CANCEL_TIMEOUT` | `summary.status=PASS`, `overall_pass=true` | The 2026-08-30 artifact `/home/edgegrasp/ros2_ws/test_results/injected_moveit_fail_closed_20260830T001525` ran two existing tests with an in-process fake MoveGroup ActionServer: 2 passed in 2.04 s. Explicit cancel and goal-response timeout stayed fail closed; the late accepted fake goal was canceled, and trajectory/fake-gate goals were zero. This is not real MoveGroup, controller, or hardware evidence. `accepted_goal_result_timeout_verified=false` remains unverified. |
| `A · REAL_MOVEGROUP_CANCEL_RACE_NEGATIVE_EXISTING_ARTIFACT` | `summary.status=UPSTREAM_PROCESS_EXITED`, `overall_pass=false` | The two frozen 2026-08-29 P2 runs kept `edgegrasp_fail_closed=true` and sent zero actual motion goals, but each real MoveGroup run recorded one SIGSEGV and exit `-11` in `PlanExecution::stop()`; `move_group_survived=false`. `accepted_goal_result_timeout_verified=false` and `strong_move_group_request_id_correlation=false` remain boundaries. Preserve only as historical negative evidence; do not reproduce OS-process interruption. |

Exact P2 hashes and claim flags are in the
[MoveIt evidence matrix](observations/2026-08-29-moveit-evidence-matrix.json).
The 2026-08-30 safe artifact is a separate WSL result path in the table above;
it must remain classified as in-process fake-dependency evidence.

The dependency-free sequence core is now wrapped by `GraspSequence.action`.
A task freezes one finite normalized approach orientation for collision routing
and one finite normalized grasp orientation shared by descend and lift.
Re-snapshotting either orientation between immutable stage positions is not
allowed. A four-stage run cannot safely reuse one original
`source_timestamp_ns`: both
the MoveIt adapter and trajectory gate enforce the 200 ms source-freshness
contract. The wrapper therefore consumes `TrackedTarget`, binds every stage to
a fresh observation of the same immutable target, rejects position drift
outside an explicit tolerance, and verifies the resulting trajectory digest
through `PlanTarget` and `ExecuteTrajectory`; increasing the timeout is not an
accepted substitute. The interface and failure branches are verified with fake
ROS action servers. A complete real Gazebo/MoveIt/controller sequence and its
fail-closed follow-up are recorded in
`docs/observations/2026-08-27-grasp-sequence-runtime.json`; the corrected
10-run result plus invalid and failed predecessor batches are in
`docs/observations/2026-08-27-grasp-sequence-repeatability.json`. These runs
used safe integration poses below the table and are not object-grasp evidence.
The later cube-scoped task004-task006 progression is recorded in
`docs/observations/2026-08-27-grasp-trial-final-runtime.json`: task006 completed
all four typed stages without clock rollback or premature adapter cancellation,
but the cube did not move and no gripper contact was observed.
The later
`docs/observations/2026-08-27-pilz-collision-contact-runtime.json` records the
Pilz `ValidateSolution` fix: the unsafe mid approach was rejected for
target-cube/`gripper_link` collision, a farther approach executed without cube
motion, and task039 completed the typed sequence with contact but without the
required lift or retention. Candidate003 then required exact distal-pad proxy
tokens, completed the typed sequence, and produced stronger negative evidence:
zero pad contact, 2.43 mm peak lift, no retention, and 52.3 mm lateral motion.
Candidate004 fixes the static AABB false positive with exact OBB SAT and keeps
the cube collision-active. Runtime ACM inspection confirmed only the two pad
child links are allowed to touch it. Pilz then rejected approach-to-descend at
states 27-42/51 for cube versus `gripper_link`, both with gripper 1.5 and 0.60;
the closed-gripper descend-to-lift control passed plan-only. No candidate004
trajectory was executed.

Candidate005 added a distinct routed approach orientation while keeping the
candidate004 grasp orientation. The first clean proxy-MoveGroup attempt
observed a 1 ms ROS simulation-clock rollback and correctly latched SAFE_STOP.
After exact-PID cleanup, one fresh-domain retry completed the four correlated
stages through `PlanTarget -> ExecuteTrajectory -> FJT`. Both configured pad
tokens occurred in the same contact sample 35 times, but the cube rose only
0.332 mm versus the required 20 mm and never retained. This is stronger contact
evidence and explicit negative grasp evidence, not successful grasp evidence.
See
`docs/observations/2026-08-27-candidate005-simultaneous-proxy-runtime.json`.

Candidate006 kept the exact candidate005 geometry and measured per-pad contact
depth/wrench maxima plus gripper joint effort. Candidate007 added source-time
correlation without changing motion parameters. Its two pad tokens overlapped
for only 0.099 simulated seconds; the final simultaneous sample preceded the
correlated sequence terminal by 1.747 seconds, and even the later moving-pad
contact ended 1.400 seconds before completion. Gripper effort was near zero at
sequence completion and cube lift stayed below 0.36 mm. These observations are
consistent with transient contact followed by unload, slip, or geometry loss,
but do not distinguish the mechanism or prove force closure. See
`docs/observations/2026-08-27-candidate006-contact-quality-runtime.json` and
`docs/observations/2026-08-27-candidate007-contact-timing-runtime.json`.

Candidate008 used the fixed-SHA kinematics to generate a matching moving-pad
OBB/AABB profile for a single 0.50 rad closure change. It increased simultaneous
contact from 39 to 98 samples, contact span from 0.099 s to 0.252 s, and peak
lift from 0.359 mm to 2.667 mm. The cube nevertheless returned to table height,
drifted about 16.57 mm in XY, and was not retained. This comparison supports a
pose/contact-symmetry or bounded friction study next; it does not support a
grasp-success claim. See
`docs/observations/2026-08-27-candidate008-q0p50-runtime.json`.

Candidate009 tested a static -0.9 mm tool-X symmetry selection without sending
motion. An isolated helper called only `/plan_kinematic_path`, chained each
validated terminal joint state into the next start state, and launched no
adapter, trajectory gate, sequence node, ExecuteTrajectory client, or FJT
client. The unchanged zero-offset control passed home-to-approach,
approach-to-descend, and descend-to-lift 10/10; every sampled negative offset
from -0.1 through -0.9 mm failed approach-to-descend with
`NO_IK_SOLUTION`. All execution counters stayed zero. This rejects the
translation-only symmetry optimum at the current IK boundary; it is not a
failed motion attempt. See
`docs/observations/2026-08-27-candidate009-reachability-scan.json`.

Candidates010/011 kept the known-reachable arm poses and changed one coupled
profile/command variable: close angle 0.45 then 0.40 rad. Each generated
profile passed exact geometry checks and all three isolated Pilz segments 10/10
before one typed execution. Candidate010 produced 144 simultaneous samples over
0.368 s and 3.323 mm peak lift. Candidate011 produced 171 samples over 0.463 s
and 3.711 mm peak lift. Both returned the cube to table height, lost bilateral
contact about 1.7 s before completion, and failed retention. The angle search
now stops: further closing would tune proxy penetration without addressing
holding. See
`docs/observations/2026-08-27-candidate010-q0p45-runtime.json` and
`docs/observations/2026-08-27-candidate011-q0p40-runtime.json`.

Candidate012 kept the Candidate011 geometry and every motion input fixed, then
mapped explicit isotropic pad friction to the generated SDF collisions. The
implicit context, explicit `mu=1.0` control, and explicit `mu=1.5` treatment all
passed the isolated three-segment plan-only gate 10/10 with zero motion. On the
same final code snapshot, control and treatment both completed the correlated
sequence but failed lift and retention. Treatment changed peak lift by only
0.076 mm and simultaneous-contact span by 13 ms; this does not identify
friction as the bottleneck or prove how DART applied the configured coefficient.
The bounded friction sweep therefore stops. See
`docs/observations/2026-08-28-candidate012-pad-friction-runtime.json`.

Candidate013 tested +5 mm and +2 mm tool-frame Y offsets only in isolated
plan-only graphs. Both failed approach-to-descend with `NO_IK_SOLUTION`, sent
zero motion, and were discarded. Candidate014 instead kept every arm pose,
close command, scene, planner setting, and explicit `mu=mu2=1.0` fixed while
extending only the moving distal-pad simulation box by 12 mm. The first apparent
plan rejection exposed a harness ordering bug: the selective ACM service had
returned `success=False` before the base scene was confirmed, and the old
harness ignored it. The corrected harness now requires base-scene confirmation,
successful ACM service response, and a later confirmed status before planning.
It passed all three chained segments in 3/3 attempts with zero command or FJT
goals. One typed runtime then completed the sequence, but achieved only 3.680 mm
peak lift and 0.462 s bilateral contact, ending contact 1.744 s before completion
and failing retention. This is effectively unchanged from Candidate011 and does
not support contact-patch length as the bottleneck. See
`docs/observations/2026-08-28-candidate014-moving-pad-runtime.json`.

Candidate024 followed the measured geometry rather than continuing parameter
sweeps. Candidate022's bilateral contacts were separated by about 50.022 mm,
but its closing axis was 37.243 degrees from the nearest cube-face normal. The
new scene rotates the cube by that measured angle and translates it 9.7 mm to
restore a 1.002 mm fixed-pad pre-close gap; the reachable arm-stage poses,
0.40 rad close command, explicit `mu=mu2=1.0`, and -0.02 Nm simulated preload
stay fixed. The isolated three-segment plan-only graph passed 10/10 with stable
per-segment trajectory digests and zero execution. The first runtime succeeded.
The second exposed a different scheduling fault: the observer baseline aged the
same target from fresh to 208 ms at the planning boundary, and the existing
200 ms gate correctly stopped before any trajectory/FJT side effect.

The trial client now checks the exact bound target immediately before sequence
submission against a 100 ms admission limit. An older snapshot can only cause a
bounded, read-only observer restart after a correlated CANCELED terminal with
reason `observer_cancelled` and matching task, target, source timestamp, domain,
and epoch; late feedback is generation-filtered. Runtime r04 exercised that
path (242 ms rejected, correlated cancel confirmed, replacement admitted at
52 ms). Post-fix r03-r11 passed 9/9. Across all 11 runtime attempts, 10 met the
strict bilateral-contact, 20 mm lift, and 0.5 s retention contract; retained
lift was 28.858-29.128 mm. This is a successful simulated grasp for one fixed
proxy scene, not proof of force closure, physics determinism, or hardware
performance. See
`docs/observations/2026-08-29-candidate024-face-aligned-runtime.json`.

See [validation-report.md](validation-report.md) for commands and
[environment-audit.md](environment-audit.md) for the host/WSL boundary.

## 2. Architecture and contracts

The stable project architecture is:

```text
sensor/input
  -> Target3D
  -> constant-velocity tracker/predictor
  -> EndpointWorkspaceGate
  -> controller/executor
  -> safety boundary
  -> backend(mock | gazebo | real)
```

The same upper data and result contracts remain replaceable across backends.
Only the synchronous mock backend executes in the current core. Gazebo and real
selection are disabled placeholders until an explicit adapter is attached.

### Target and coordinate contract

`Target3D` carries:

- target ID;
- position in metres;
- integer source timestamp in nanoseconds;
- explicit frame ID;
- confidence;
- clock domain and epoch.

The first-phase planning frame is `base_link`. The pure core intentionally has
no tf2 implementation, so mismatched frames fail closed. Mock coordinates are
authored directly in `base_link`. The ROS MoveIt adapter implements a
timestamped, bounded tf2 transform into `base_link`; missing, extrapolated,
stale, or failed transforms reject planning. A Gazebo `world` target may enter
only through that explicit transform, never through an implicit frame rename.

### Source freshness versus stream liveness

Two independent timers are intentional:

1. **Source freshness** compares `Target3D.timestamp_ns` with the current time.
   Exactly 200 ms is allowed; greater than 200 ms and any future timestamp are
   rejected.
2. **Local stream liveness** records when a fresh target was received.
   `EdgeGraspController.tick(now_ns)` stops after more than 200 ms without a
   new fresh receive. Repeated ticks after the latch are idempotent.

This prevents a source timestamp alone from masquerading as continued message
liveness. The pure controller tests cover tracking, planning, and synchronous
executing interruption. The ROS timer ran under Gazebo and MCAP replay; target
dropout canceled an active simulated arm goal. A cancel request remains weaker
than proof of physical stop.

### Clock and epoch contract

Within one epoch, receive, planning-boundary, and watchdog times must be
non-negative, integer, and monotonically non-decreasing. A rollback latches
`SAFE_STOP` or `ERROR`; recovery requires an explicit new epoch reset.

The core requires an explicit `execute_now_ns` after planning. It cannot
default to the earlier receive time, so a target that becomes stale during slow
planning never reaches `backend.execute`.

ROS policy is explicit:

| ROS setting | EdgeGrasp domain |
| --- | --- |
| `use_sim_time=false` | `ros_system` |
| `use_sim_time=true` | `ros_sim` |

Launch passes one domain, frame, and initial epoch to publisher, monitor, and
trajectory gate. Nodes reject domain/use-sim-time mismatch and latch on clock
rollback. `geometry_msgs/PointStamped` carries neither identity, domain, nor
epoch, so legacy receiving wrappers assign local epochs. `TrackedTarget` now
carries target ID, source-stamped point, domain, and epoch atomically and its
mock dual-publication contract ran under Jazzy; safety/gate nodes do not yet
consume it. Cross-node reset coordination therefore remains **UNVERIFIED**.
The separate `PlanTarget.action` carries an immutable source
time/domain/epoch and request ID, but receiving wrappers still own epoch
resets. The ROS wrapper does not invoke the pure
`EdgeGraspController`; equivalent ROS gates are tested and reported separately.

### Planner and execution contract

`EndpointWorkspaceGate` checks only:

- endpoint inside a conservative Cartesian box;
- endpoint outside configured blocked AABBs;
- exact target frame.

It emits one Cartesian waypoint for the mock backend. It is not IK,
swept-volume/path collision checking, robot geometry, a planning scene, or
MoveIt. A blocked result is `ENDPOINT_BLOCKED / endpoint_blocked`; this project
does not use the obsolete or misleading `planning_collision` term.

A plan crosses the core execution boundary only if all are true:

- `success is True`;
- `reason is PlanReason.PLANNED`;
- the plan is a valid `MotionPlan`;
- target ID, source timestamp, request timestamp, frame, domain, epoch, and
  final waypoint match the prediction.

Contradictory plan results fail closed. The core backend contract is synchronous:
`execute` returns only when execution has completed or stopped. Re-entrant
targets while `EXECUTING` are rejected as busy. Reset from planning/executing
must stop first; a stop failure cannot transition to `IDLE`.

## 3. ROS motion-command boundary

The overlay implements two connected safety paths:

```text
PointStamped target
  -> safety_monitor
  -> /edgegrasp/motion_allowed

PlanTarget immutable pose
  -> timestamped tf2 to base_link
  -> /compute_ik with a fresh joint-state seed
  -> MoveGroup action with planning_options.plan_only=true
  -> RobotTrajectory validator
  -> /edgegrasp/execute_trajectory (controller=arm_controller)
  -> trajectory_gate
  -> /arm_controller/follow_joint_trajectory

/edgegrasp/execute_trajectory (controller=gripper_controller)
  -> trajectory_gate
  -> /gripper_controller/follow_joint_trajectory
```

`trajectory_gate` rejects unknown, false, stale, or rolled-back permission. It
accepts only the two pinned controller names/actions, validates the exact arm or
gripper joint order, verifies every point's vector sizes, finite values, and
strictly increasing positive time, rejects a concurrent goal per controller,
and requests cancellation on permission loss. Its local pure permission and
trajectory-contract validators plus source/config structure are tested. Jazzy
colcon collected and passed the fake FJT integration and node-level
cancel/send/result failure tests. Live Gazebo accepted gated arm and gripper
goals; target dropout requested cancellation and the simulated arm returned a
canceled result. Cancel rejection, transport exception, response timeout, and
an accepted cancel without a terminal result are bounded to three attempts;
exhaustion latches an error that requires controller-level stop escalation.

`edgegrasp_moveit_adapter` is arm-only and does not copy upstream MoveIt
configuration. It transforms the immutable target, performs bounded IK, sends
one MoveGroup `plan_only` request, validates the returned joint trajectory,
rechecks safety/freshness/interface/start state, and submits exactly once to
the typed arm gate. Wrong group/order, malformed or out-of-limit trajectory,
stale/concurrent request, unavailable dependency, and IK/MoveIt failure all
produce zero gate publication. Gripper sequencing remains separate.

SO-101 arm joint order and units:

1. `shoulder_pan` — radians
2. `shoulder_lift` — radians
3. `elbow_flex` — radians
4. `wrist_flex` — radians
5. `wrist_roll` — radians

The gripper contract is one joint, `gripper`, in radians.

A conservative request through the legacy EdgeGrasp topic adapter is:

```bash
ros2 topic pub --once /edgegrasp/arm_joint_trajectory_request trajectory_msgs/msg/JointTrajectory "{joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], points: [{positions: [0.0, -0.2, 0.4, -0.2, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}"
ros2 topic pub --once /edgegrasp/gripper_joint_trajectory_request trajectory_msgs/msg/JointTrajectory "{joint_names: [gripper], points: [{positions: [0.2], time_from_start: {sec: 2, nanosec: 0}}]}"
```

It forwards only while permission, target liveness, interface readiness, and
joint state remain fresh. Both arm and gripper paths ran in simulation. New
sequence/adapter code uses `ExecuteTrajectory` instead so immutable command
identity, source timestamp, digest, and terminal FJT evidence are preserved.

The following commands are **UNSAFE/BYPASS diagnostics**. They bypass the
EdgeGrasp gate and are only low-level, simulation-only controller checks after
the start state, limits, exact action types, and controller state have been
inspected:

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], points: [{positions: [0.0, -0.2, 0.4, -0.2, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}}"
ros2 action send_goal /gripper_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [gripper], points: [{positions: [0.2], time_from_start: {sec: 2, nanosec: 0}}]}}"
```

A successful diagnostic proves only a controller path, not planner safety,
collision avoidance, grasping, or readiness for real hardware.

### MCAP timing and replay contract

Gazebo/`ros_sim` recording waits for `/clock` and uses `--use-sim-time`; the
system/mock profile does not. Raw inputs and derived/status/command topics are
recorded intentionally for comparison, but default playback allowlists only
`/joint_states`, `/edgegrasp/target_3d`, and the raw camera topics.

Default `SAFETY_REPLAY` requires Gazebo, MoveIt, controllers, and every other
clock publisher to be stopped. `edgegrasp_replay.launch.py` starts only the
safety monitor and trajectory gate; rosbag `--clock` is the sole authority.
Replaying recorded permission or trajectory requests would duplicate live
decisions and is forbidden. Rewinding the same bag requires a restarted wrapper
or coordinated new epoch; otherwise clock rollback correctly latches
fail-closed. This mode contains no planner, backend, controller, or physics.

`GAZEBO_RUN` instead gives Gazebo sole `/clock` authority and never uses
`ros2 bag play --clock`. `OUTPUT_INSPECTION` is a manual all-topic playback
mode allowed only in an isolated graph with no live monitor, gate, planner,
controller, or backend. It is never the default closed-loop path.

## 4. Claims versus pinned upstream repository state

The machine-readable source of truth is
[upstream-manifest.json](upstream-manifest.json). Every repository blob link is
pinned to a 40-character SHA. Reproduction steps include explicit working
directories.

### adoodevv/so101_ros2

Pin: `0305e03ab54e64aae9263fcbf339622e654012f3`,
BSD-3-Clause.

Use for Gazebo Harmonic, `gz_ros2_control`, SO-101 URDF/D435/world/bridge,
controllers, and its complete-looking MoveIt configuration. At this pin:

- the clean detached local checkout contains SRDF, `move_group`, controller
  mapping, joint limits, kinematics, planning pipelines, and a resolving
  Gazebo+MoveIt bringup path;
- the MoveIt README still describes planned/incomplete work and a
  ForwardCommandController/Float64MultiArray gripper path, which conflicts with
  the pinned `ros2_controllers.yaml` and `moveit_controllers.yaml`; EdgeGrasp
  follows the two FollowJointTrajectory YAML mappings;
- `robot_state_publisher.launch.py` expands templates and writes into hard-coded
  `~/ros2_ws` source/install paths, so it must only run in a disposable,
  inspected workspace;
- system tests are TODO shells and real hardware is not implemented;
- `pick_and_place.world` needs Gazebo Fuel resources on first use.

Evidence:

- [README status claims](https://github.com/adoodevv/so101_ros2/blob/0305e03ab54e64aae9263fcbf339622e654012f3/README.md#L30-L39)
- [Gazebo commands and expected active controllers](https://github.com/adoodevv/so101_ros2/blob/0305e03ab54e64aae9263fcbf339622e654012f3/README.md#L80-L100)
- [MoveIt README documentation state](https://github.com/adoodevv/so101_ros2/blob/0305e03ab54e64aae9263fcbf339622e654012f3/so101_moveit_config/README.md#L3-L14)
- [MoveIt README planned items](https://github.com/adoodevv/so101_ros2/blob/0305e03ab54e64aae9263fcbf339622e654012f3/so101_moveit_config/README.md#L75-L80)
- [pinned move_group launch](https://github.com/adoodevv/so101_ros2/blob/0305e03ab54e64aae9263fcbf339622e654012f3/so101_moveit_config/launch/move_group.launch.py)
- [pinned MoveIt controller mapping](https://github.com/adoodevv/so101_ros2/blob/0305e03ab54e64aae9263fcbf339622e654012f3/so101_moveit_config/config/so101/moveit_controllers.yaml)

The earlier finding that these MoveIt files were absent is retracted in the
machine-readable audit history. Static presence alone did not prove runtime.
The project subsequently ran the pin in a dedicated Jazzy WSL workspace:
controller-only `empty.world`, both FJT actions, move_group, joint-space
planning, camera topic publication, and the EdgeGrasp plan-only-to-gate path
passed within their named scopes. A later generated-proxy run removed the
earlier DART mesh diagnostics and proved one named base-proxy contact only; one
move_group Ctrl-C shutdown segmentation fault remains a recorded limitation.

### TheRobotStudio/SO-ARM100

Pin: `7629d2ad9853d10fb903093a33ef6114099d97e5`,
Apache-2.0.

Use for URDF/MJCF/mesh and calibration-convention cross-checks only. It is not a
ROS workspace and has no Gazebo/MoveIt command. Base collision meshes were
removed, relative mesh paths require care, and the LeRobot gripper convention
`0=closed, 100=open` is not encoded.

Evidence:

- [simulation overview](https://github.com/TheRobotStudio/SO-ARM100/blob/7629d2ad9853d10fb903093a33ef6114099d97e5/Simulation/README.md#L18-L23)
- [relative meshes and removed base collision](https://github.com/TheRobotStudio/SO-ARM100/blob/7629d2ad9853d10fb903093a33ef6114099d97e5/Simulation/SO101/README.md#L7-L9)
- [gripper mapping caveat](https://github.com/TheRobotStudio/SO-ARM100/blob/7629d2ad9853d10fb903093a33ef6114099d97e5/Simulation/SO101/README.md#L24-L31)

Collision geometry and gripper conversion must be resolved explicitly before
using these assets as planning/control evidence.

The clean local detached checkout was statically checked: `scene.xml` resolves
to `so101_new_calib.xml`, the six joint/actuator names and order match, and all
13 declared meshes exist. Runtime remains unverified. Python MuJoCo 3.3.7 and
3.12.0 failed loading bundled Windows plugins, the checksum-verified official
3.12.0 `testspeed` failed even on its own bundled model, and 2.3.7 rejected the
unchanged asset's newer `kv` schema. These are host/runtime blockers, not a
successful dynamics or SO-101 stepping result.

### legalaspro/so101-ros-physical-ai

Pin: `58318c905a2c61289fa907de85cb8473322fbe68`,
Apache-2.0. Its `legalaspro/feetech_ros2_driver` submodule is pinned to
`4c0fdbfe16c84c686f8ace09526c52d98d0110ca`.

Use as a design reference for complete MoveIt configuration, mock/real
`ros2_control`, controllers, camera TF, MCAP recording, and hardware safety.
At this pin it supports `hardware_type=mock|real`; MuJoCo is planned and
Gazebo is absent.

Evidence:

- [MoveIt command, real/mock boundary, and MuJoCo limitation](https://github.com/legalaspro/so101-ros-physical-ai/blob/58318c905a2c61289fa907de85cb8473322fbe68/README.md#L426-L457)
- [hardware caveat](https://github.com/legalaspro/so101-ros-physical-ai/blob/58318c905a2c61289fa907de85cb8473322fbe68/docs/hardware.md#L1-L11)
- [calibration and EEPROM requirements](https://github.com/legalaspro/so101-ros-physical-ai/blob/58318c905a2c61289fa907de85cb8473322fbe68/docs/hardware.md#L124-L155)
- [pinned Feetech submodule commit](https://github.com/legalaspro/feetech_ros2_driver/commit/4c0fdbfe16c84c686f8ace09526c52d98d0110ca)

No upstream code is copied. EdgeGrasp keeps a thin integration adapter and does
not duplicate the pinned upstream MoveIt configuration. Its differentiated
evidence is the safety command boundary, timestamp/frame/watchdog contracts,
deterministic replay, packaging, and metrics.

## 5. Quantified acceptance

| Gate | Current core result | Required ROS/Gazebo evidence |
| --- | --- | --- |
| 100 deterministic replays | Passed for 0/20/40 mm/s | MCAP repeatability remains a future metric; wrapper playback ran once per fresh epoch |
| No-update watchdog | >200 ms calls stop and latches; boundary/idempotence tested | Fake `/clock` test allows exactly 200 ms and cancels by the next 50 Hz tick (220 ms); live dropout also canceled an active simulated action; physical stop unverified |
| Source freshness | 200 ms allowed; 200 ms + 1 ns and future rejected | Replay denied short callback-order future samples until clock catch-up, then allowed |
| Prediction error | Noiseless linear synthetic error is zero/roundoff after two samples | Record P50/P95 sensor-to-command latency and 3D error |
| Unreachable/blocked endpoint | Endpoint proxy rejects | Runtime table-inside, invalid table-crossing candidate, far-unreachable, stale, and epoch-mismatch cases all produced zero trajectory dispatch |
| Planning/backend/stop failures | Fail closed in core and ROS fake-server tests | Live unreachable IK failed with zero trajectory publication |
| Target-to-controller | Not a core claim | Reachable pose produced IK, MoveGroup plan-only, gate publication, FJT status 4 and matching joint feedback |
| Four-stage protocol repeatability | Not a core claim | Corrected direct-entrypoint harness completed 10/10 runs; all wrapper statuses were 4 and all per-run sequence processes stopped cleanly; ten distinct trajectory digests mean planner-byte determinism is unproven |
| MCAP time domain | Not applicable | 27 target records, receive-minus-header range -1 ms to +1 ms, zero violations at 1 s guard |
| Grasp success | Not a core claim | Candidate024 produced 10 scoped simulation-physics successes; each successful observer result required both configured pad contacts, at least 20 mm lift, and 0.5 s retention. Post-fix r03-r11 passed 9/9; hardware remains unverified |

The 100-run result is core replay determinism. It is not simulator, dynamics,
controller, or grasp determinism.

## 6. Completed ladder and next runtime gates

Completed on the dedicated Ubuntu 24.04 WSL2/Jazzy workspace:

1. Preserved Ubuntu 22.04 and created a separate Ubuntu 24.04 distro/user.
2. Installed and inventoried Jazzy, Harmonic, ros2_control, MoveIt, and MCAP.
3. Copied the exact clean adoodevv pin into disposable `~/ros2_ws`; built the
   upstream plus complete EdgeGrasp monorepo layout.
4. Passed xacro/check_urdf, controller/joint/action inspection, conservative
   direct diagnostics, gated arm/gripper commands, watchdog cancellation,
   MoveIt plan service, target-to-gate adapter, camera topics, and MCAP
   record/raw replay.
5. Completed a zero-execution distinct-target matrix. Ten targets generated
   through the pinned `shoulder_pan` joint origin/axis passed all 30 chained
   arm segments; four invalid scenes and six distant targets were rejected at
   their declared layer. All 20 outcomes matched with zero mis-forward,
   unverified, or motion-side-effect counts. See
   [the matrix observation](observations/2026-08-29-candidate024-target-matrix-plan-only.json).
6. Executed only the selected near-control pose and both declared range
   endpoints once each. All 12 correlated typed commands reached matching FJT
   success terminals; all three independent observer results retained both pad
   contacts, 28.965-29.095 mm lift, and the 0.5 s retention window. See
   [the typed-runtime observation](observations/2026-08-29-candidate024-distinct-target-typed-runtime.json).

Remaining gates, in order:

1. Preserve Candidate024 as the fixed simulation control. Do not continue
   blind friction, close-angle, or pad-length sweeps; any new scene or grasp
   pose must again pass the isolated plan-only gate before typed execution.
2. Keep real MoveGroup validation to normal plan-only/zero-execution artifacts.
   Exercise cancel, goal-response timeout, and late-result contracts only with
   in-process dependency injection or a dedicated fake MoveGroup ActionServer.
   Retain the two real-process P2 cancel-race crashes as negative artifacts and
   do not reproduce OS-process interruption.
3. If a per-pose repeatability claim is needed, pre-register a small repeat
   count and rerun the three selected poses; the current one-run-per-pose result
   must not be converted into a workspace success percentage.
4. Investigate the move_group Ctrl-C shutdown segmentation fault and the
   `use_camera=false` SRDF torso warnings.
5. Begin a real follower-arm bring-up only with calibration, conservative
   unloaded motion, explicit stop/release checks, and no autonomous grasp until
   the hardware frame/timestamp/gripper mapping is measured.

The full command sequence is in
[ubuntu-jazzy-runbook.md](ubuntu-jazzy-runbook.md).

A loaded model, visible action server, active controller, or successful direct
trajectory is not a successful grasp.

## 7. Hardware purchase gate

The simulation gate for buying one SO-101 follower arm is now met. Candidate024
has 10 scoped physics-success artifacts and 9/9 post-fix repeatability, so the
highest-value next work is measuring the sim-to-real gap: calibration, gripper
mapping/contact, stop/release behavior, and real timing. Purchase one follower
only, start with conservative unloaded joint-space tests, and keep autonomous
grasp disabled until those checks pass. Delay the camera until joint/gripper
calibration is stable enough for frame/extrinsic validation, and delay an edge
GPU until profiling demonstrates a deployment bottleneck.
