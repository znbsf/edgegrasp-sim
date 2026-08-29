# EdgeGrasp Sim

EdgeGrasp Sim is a safety-first SO-101 simulation portfolio project. It starts
with a deterministic ROS-independent core, then adds a ROS 2 command gate and
a MoveIt plan-only adapter without copying the upstream robot configuration.

## Current evidence

Snapshot date: 2026-08-29, Asia/Shanghai.

| Layer | Observed result | Claim boundary |
| --- | --- | --- |
| Deterministic core | 332 Windows CPython 3.14.5 project tests pass; 0/20/40 mm/s scenarios replay identically 100 times | Core replay determinism, not physics determinism |
| Python/ROS packaging | Root distribution import, complete-monorepo ament adapter, negative isolated-copy test, and Jazzy colcon build pass | `edgegrasp_core` is intentionally monorepo-layout dependent |
| ROS safety boundary | `edgegrasp_ros` collects and passes 49 Jazzy tests, including typed target publication, ExecuteTrajectory correlation, fake FJT, simulated-clock watchdog, PlanningScene/ACM confirmation, all-pad physics observation, and fail-closed late/cancel/send/result paths | A cancellation request is not a guaranteed physical stop; XML has zero failures/errors/skips, and an earlier intermittent rclpy teardown warning remains tracked |
| MoveIt adapter | `edgegrasp_moveit_adapter` collects and passes 23 Jazzy tests | Fake IK/MoveGroup tests prove failure contracts; pinned runtime evidence is listed separately |
| Gazebo controllers | Three controllers active, six joint states observed, arm and gripper FJT actions accepted conservative gated goals | Controller execution is not grasp success |
| MoveIt | The unique EdgeGrasp overlay loaded OMPL/Pilz/STOMP, injected Pilz `ValidateSolution`, disabled direct MoveGroup execution, and exposed both distal-pad links in the runtime model; candidate005 then completed through that proxy planning model | These are scoped path/execution checks, not minimum-clearance or whole-robot collision fidelity |
| Shared PlanningScene | The scene contract generated Gazebo's table/cube world; MoveIt runtime-confirmed both objects and a selective ACM where only the two distal-pad child links may contact the retained cube | The selective transition is still manual; parent links remain forbidden and no target-to-pad physics claim follows from ACM configuration |
| Target-to-controller path | A reachable immutable pose ran through tf2, `/compute_ik`, MoveGroup `plan_only`, trajectory validation, EdgeGrasp gate, and arm FJT; controller returned status 4 | One simulated arm trajectory, not autonomous pick-and-place |
| Four-stage sequence and physics observer | 60 dependency-free sequence tests and 45 Jazzy action/client tests pass. Candidate024 passed 10/10 isolated three-segment plan-only attempts, then produced 10 correlated simulation-physics grasp successes across 11 runtime attempts; the sole failure was a pre-motion stale-target SAFE_STOP. After the scheduling fix, r03-r11 passed 9/9 | Success is limited to one fixed proxy scene and target pose; it is not physics determinism or hardware grasp evidence |
| Distinct-target plan-only matrix | A fixed Candidate024-derived matrix matched 20/20 hypotheses: ten shoulder-pan-symmetry targets passed all 30 arm segments, four invalid scenes were rejected before ROS startup, and six distant targets were rejected by MoveIt. False accept, false reject, unverified, and motion-side-effect counts were all zero | Each distinct target ran once and every trajectory was discarded; this is planning coverage, not execution or grasp evidence |
| Camera/world/contact | Pinned world published color/depth/camera-info; generated primitives replaced 13 collision meshes. Candidate024's 11 runtime logs contained zero tracked DART mesh/geometry-construction diagnostics, and every successful run had both configured distal-pad contacts through lift retention | Log absence alone is not collision proof; only one base proxy has isolated behavior evidence and the robot still uses conservative primitive collision proxies |
| MCAP | Jazzy ros_sim record and raw-input replay ran; 27 target records matched bag receive time within ±1 ms | Wrapper/gate replay only; no planner/backend/physics replay |
| Real hardware | Not run | Hardware, calibration, camera extrinsics, and grasp success remain unverified |

Exact commands, interpreter versions, counts, and failures are in the
[validation report](docs/validation-report.md). The staged design is in the
[simulation plan](docs/simulation-plan.md), the reproducible ROS procedure is
the [Ubuntu/Jazzy runbook](docs/ubuntu-jazzy-runbook.md), and source pins and
licenses are in the machine-readable
[upstream manifest](docs/upstream-manifest.json). The latest table collision
cases are preserved in the machine-readable
[PlanningScene runtime observation](docs/observations/2026-08-27-planning-scene-runtime.json),
the [sequence runtime observation](docs/observations/2026-08-27-grasp-sequence-runtime.json).
The corrected 10-run harness, retained failures, summary hash, and strict
claim boundary are in the
[sequence repeatability observation](docs/observations/2026-08-27-grasp-sequence-repeatability.json).
The task004-task006 fix progression and final live negative grasp evidence are
in the
[final grasp-trial observation](docs/observations/2026-08-27-grasp-trial-final-runtime.json).
The later Pilz response-adapter fix, identical-path collision rejection, safe
approach, explicit target-contact scene policy, and contact-without-lift result
are in the
[Pilz collision/contact observation](docs/observations/2026-08-27-pilz-collision-contact-runtime.json).
Candidate002's fail-closed path results and candidate003's accepted approach are
in the [candidate plan-only observation](docs/observations/2026-08-27-candidate002-003-plan-only.json).
The latest typed sequence and stricter zero-distal-contact physics result are in
the [candidate003 runtime observation](docs/observations/2026-08-27-candidate003-runtime.json).
The retained-cube selective ACM and candidate004 plan-only rejection/acceptance
boundary are in the
[candidate004 observation](docs/observations/2026-08-27-candidate004-selective-acm-plan-only.json).
The current end-to-end candidate005 record—including the retained 1 ms clock
rollback SAFE_STOP, correct proxy MoveGroup retry, 35 same-sample two-pad
contacts, and failed 20 mm lift/retention gate—is in the
[candidate005 simultaneous-contact observation](docs/observations/2026-08-27-candidate005-simultaneous-proxy-runtime.json).
The unchanged candidate geometry was then instrumented for solver contact
quality and gripper effort in the
[candidate006 quality observation](docs/observations/2026-08-27-candidate006-contact-quality-runtime.json).
The latest fresh-domain run correlates those contacts with the sequence terminal
in the
[candidate007 timing observation](docs/observations/2026-08-27-candidate007-contact-timing-runtime.json):
same-sample bilateral contact ended 1.747 simulated seconds before sequence
completion, while cube lift remained below 0.36 mm. This is evidence of
transient contact loss, not a simulated grasp.
Candidate008 then changed one controlled variable: a reproducibly generated
moving-pad geometry profile and matching `0.50 rad` close command. The
[candidate008 observation](docs/observations/2026-08-27-candidate008-q0p50-runtime.json)
records 98 simultaneous samples over 0.252 simulated seconds and 2.667 mm peak
lift. Those are clear improvements over candidate007, but bilateral contact
still ended 1.788 seconds before completion, the cube returned to the table,
and retention failed. It remains a stronger negative/control result, not a
simulated grasp.
Candidate009 selected a static -0.9 mm tool-X symmetry optimum, but the new
isolated [chained plan-only gate](docs/observations/2026-08-27-candidate009-reachability-scan.json)
showed that every sampled negative offset down to -0.1 mm failed
approach-to-descend with `NO_IK_SOLUTION`; the unchanged zero-offset control
passed all three Pilz segments 10/10. No adapter, gate, trajectory publication,
ExecuteTrajectory goal, FJT goal, or execution was present in those graphs.
The translation-only symmetry search was therefore rejected rather than tried
on the robot.
Candidate010 and Candidate011 kept the reachable arm poses and changed only the
generated gripper close geometry/command to 0.45 and 0.40 rad. Both passed the
same 10/10 plan-only gate before one controlled execution. Their
[0.45 rad observation](docs/observations/2026-08-27-candidate010-q0p45-runtime.json)
and [0.40 rad observation](docs/observations/2026-08-27-candidate011-q0p40-runtime.json)
record increasing transient bilateral contact (0.368/0.463 s) and peak lift
(3.323/3.711 mm), but both returned the cube to table height and failed the
20 mm plus retention contract. The close-angle search is now stopped; these
are controlled negative results, not grasp success.
Candidate012 then held Candidate011 geometry, scene, planner, scaling, and typed
execution fixed while changing only generated finger-pad `mu=mu2` from 1.0 to
1.5. All three material rows passed 10/10 plan-only; the paired runtime rows
both completed the sequence. Treatment changed peak lift by only +0.076 mm
(3.392 to 3.468 mm) and extended simultaneous contact by 13 ms, while both
cubes returned to table height and failed retention. The
[Candidate012 paired observation](docs/observations/2026-08-28-candidate012-pad-friction-runtime.json)
therefore closes this bounded friction sweep as negative evidence; the next
useful change is contact geometry/kinematics, not a larger unbounded coefficient.
Candidate013 then tested only +5 mm and +2 mm tool-frame Y offsets in isolated
plan-only graphs. Both failed approach-to-descend with `NO_IK_SOLUTION`, sent
zero motion, and were rejected without retaining a runtime profile or API.
Candidate014 instead preserved the reachable arm poses and extended only the
moving distal-pad simulation box by 12 mm along local Y. After fixing a real
PlanningScene/ACM harness race, all three chained segments passed 3/3 and one
typed execution completed. Peak lift was 3.680 mm, bilateral contact lasted
0.462 s, and retention still failed—essentially Candidate011's result. The
[Candidate014 observation](docs/observations/2026-08-28-candidate014-moving-pad-runtime.json)
therefore rejects contact-patch length as the demonstrated bottleneck; the
extended box is EdgeGrasp-owned simulation geometry, not an upstream or
hardware-validated design.
Candidate024 then changed the target pose from measured Candidate022 geometry,
not by blind sweep: it aligned one cube face with the observed closing axis and
translated the cube 9.7 mm to restore a 1.0 mm fixed-pad pre-close gap while
keeping the reachable arm-stage poses unchanged. It passed the isolated
three-segment plan-only chain 10/10 with zero execution. Runtime r01 succeeded;
r02 then exposed a real client scheduling flaw where five physics-baseline
samples aged the same target to 208 ms at the planning boundary. The safety
gate correctly sent zero motion. The client now rechecks that exact snapshot
against a 100 ms pre-send bound and, if stale, may rebuild the read-only
observer only after a correlated CANCELED/`observer_cancelled` terminal. r04
exercised that restart at runtime (242 ms rejected, 52 ms replacement), and
r03-r11 completed 9/9 after the fix. Across all 11 attempts, 10 independently
met bilateral-pad contact, at least 20 mm lift, and 0.5 s retention; retained
lift ranged 28.858-29.128 mm. See the machine-readable
[Candidate024 observation](docs/observations/2026-08-29-candidate024-face-aligned-runtime.json).
This is a scoped simulated grasp in one proxy scene, not real-robot evidence.
The next distinct-target experiment first disproved a naive Cartesian generator:
all ten rigid translations failed first-stage IK while the unchanged control
passed. Inspection of the pinned URDF showed that `shoulder_pan` is offset from
the base origin and its local +Z maps to base-frame -Z. The corrected generator
uses that exact joint origin and axis. Its
[20-case plan-only observation](docs/observations/2026-08-29-candidate024-target-matrix-plan-only.json)
records 10/10 distinct reachable hypotheses, 10/10 matched rejection
hypotheses, and zero motion side effects. The superseded translation result is
kept in the
[diagnostic observation](docs/observations/2026-08-29-candidate024-translation-matrix-plan-only.json)
rather than being hidden or relabelled.
The preceding histogram run is retained as physical-contact history but is
explicitly excluded from proxy-planning evidence because it launched pinned
upstream MoveGroup by operator mistake.

## Quick start: deterministic core

Python 3.10 or newer:

```powershell
git clone https://github.com/znbsf/edgegrasp-sim.git
cd edgegrasp-sim
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
powershell -NoProfile -File scripts\check.ps1 -ReplayRuns 100 `
  -PythonExecutable .\.venv\Scripts\python.exe
```

Ubuntu/CI:

```bash
python3 -m pip install -e '.[dev]'
bash scripts/check.sh 100
```

A standalone clone intentionally does not vendor the pinned SO-101 checkout.
The private-release preflight therefore passed 330 tests and skipped two
checkout-dependent collision-contract tests. The documented full development
layout, with the exact clean `so101_ros2` pin available outside this repository,
passed 332/332. Both modes run the same deterministic replay and structure
gates; a skip is never reported as runtime simulator evidence.

Useful focused checks:

```powershell
python -m pytest --collect-only -q
python -m pytest -q
python -m edgegrasp replay --runs 100
python scripts\benchmark_replay.py --runs 100 --horizon-ms 100
python scripts\validate_project.py
powershell -NoProfile -File scripts\audit_environment.ps1 -IncludeWsl
```

The README uses `bash scripts/...` so commands remain portable when a Windows
tool or archive loses executable mode bits. Git Bash `bash -n` is a syntax
check, not Ubuntu/ROS execution evidence.

## Architecture

The deterministic core remains independent of ROS:

```text
sensor/input
  -> Target3D(position, source timestamp, base_link, domain, epoch)
  -> constant-velocity predictor
  -> EndpointWorkspaceGate
  -> EdgeGraspController + watchdog + state machine
  -> synchronous backend(mock | disabled gazebo | disabled real)
```

`EndpointWorkspaceGate` checks only endpoint AABB reachability/blocking. It is
not IK, swept-path, geometry, or collision-safe planning; the rejection reason
is `endpoint_blocked`.

The ROS motion path is deliberately separate:

```text
PointStamped target -> safety_monitor -> /edgegrasp/motion_allowed

PlanTarget.action immutable pose
  -> timestamped tf2 to base_link
  -> confirmed shared PlanningScene table readiness
  -> bounded /compute_ik with fresh /joint_states seed
  -> moveit_msgs/action/MoveGroup, planning_options.plan_only=true
  -> RobotTrajectory validator
  -> /edgegrasp/execute_trajectory (typed arm command)
  -> trajectory_gate with exact identity, digest, and source-stamp echo
  -> /arm_controller/follow_joint_trajectory
```

Gripper commands use the same typed gate action with
`controller=gripper_controller` and their own FJT action. The legacy request
topics remain diagnostic adapters, not the correlated sequence path. The
adapter never calls MoveIt execution, never publishes a combined
`arm_with_gripper` trajectory, and never sends directly to a controller.
MoveIt `RobotTrajectory` is not coerced into the core Cartesian `MotionPlan`.

The typed sequence path composes those interfaces without guessing from topic
order:

```text
GraspSequence.action immutable task
  -> immutable approach orientation for collision routing
  -> immutable grasp orientation shared by descend and lift
  -> APPROACH: PlanTarget -> matching ExecuteTrajectory terminal
  -> DESCEND: PlanTarget -> matching ExecuteTrajectory terminal
  -> CLOSE_GRIPPER: ExecuteTrajectory(controller=gripper_controller)
  -> LIFT: PlanTarget -> matching ExecuteTrajectory terminal
  -> COMPLETE (sequence_completed=true; protocol result only)
  -> independent GraspPhysicsEvidence terminal
```

Every stage uses the latest fresh `TrackedTarget` observation for the same
immutable target and rejects geometry drift. A stage advances only after the
outer wrapper action is terminal, the exact task/command/stage/sequence/digest
matches, and the downstream FJT has a legal terminal status. Untrusted terminal
evidence retains the downstream slot and blocks reset.

The independent physics observer requires every configured pad token in the
same contact sample before contact can refresh the retention clock. Separate
hits at different times are insufficient. Even same-sample contact remains
insufficient without at least 20 mm cube lift and a complete retention window.
Candidate024 is the first case where that independent observer returned
`physics_grasp_verified=true`; `GraspSequence` itself still never makes the
physics claim.
It also records contact depth/wrench maxima, gripper joint effort, and
first/last source timestamps as diagnostics. Those fields do not weaken the
success gate and must not be interpreted as calibrated force or force closure.

## Implemented fail-closed contracts

- `execute_now_ns` is required after planning; there is no unsafe receive-time
  default. Slow planning, clock rollback, or a target older than 200 ms blocks
  execution.
- Core `tick(now_ns)` independently stops on target-stream silence greater
  than 200 ms; the exact 200 ms boundary is allowed and repeated ticks are
  idempotent.
- Core and ROS reject invalid/future/out-of-order timestamps, wrong frame,
  wrong domain/epoch, low confidence, malformed plans, incorrect joint order,
  nonfinite arrays, non-increasing time, stale start state, and conservative
  position/velocity/acceleration violations.
- The first target frame is `base_link`. Mock targets are already expressed in
  that frame. ROS MoveIt requests use timestamped tf2 with bounded failure;
  no implicit `world` or camera conversion exists.
- ROS gate admission requires recent permission, target receive liveness,
  joint states, interface readiness, and the two exact pinned FJT endpoints.
- Permission loss, watchdog expiry, result timeout, or clock fault requests
  cancellation. Cancel rejection/exception/timeout—and an accepted cancel that
  never reaches a terminal result—latches an error, retries at most three
  times, then requires controller-level stop escalation. The project does not
  equate “cancel requested” with “robot stopped.”
- The pure backend contract is synchronous. The ROS action adapters are
  explicitly asynchronous and track one in-flight request with correlation,
  cancellation, and late-result suppression.

`PointStamped` cannot transport target identity or clock domain/epoch, so the
legacy safety wrapper still owns a local epoch. The mock publisher also
publishes `TrackedTarget` with atomic target ID, source stamp, domain, and
epoch; the sequence wrapper consumes this typed stream and is fake-action
runtime-tested. Coordinated reset/epoch propagation across all nodes remains
future work.

## MCAP single-clock contract

Recording profiles:

```bash
# Gazebo/ros_sim: waits for /clock and records with --use-sim-time.
bash scripts/record_mcap.sh /path/to/bag ros_sim

# Mock/system clock: does not use --use-sim-time.
bash scripts/record_mcap.sh /path/to/bag system
```

Derived status, command-request, cube-pose/contact, sequence-terminal, and
physics-status topics are intentionally recorded for comparison. Default
safety replay never replays them:

```bash
# Stop Gazebo/MoveIt/controllers and confirm no existing /clock publisher.
ros2 launch edgegrasp_ros edgegrasp_replay.launch.py
# In a separate sourced shell:
bash scripts/replay_mcap.sh /path/to/bag
```

The three modes are deliberately separate:

- `SAFETY_REPLAY`: stop Gazebo/controllers, launch only the wrapper/gate, and
  replay the raw allowlist with rosbag as the sole `/clock` authority.
- `GAZEBO_RUN`: Gazebo alone owns `/clock`; never combine it with playback
  `--clock`.
- `OUTPUT_INSPECTION`: manual all-topic playback only in an isolated graph with
  no live monitor, gate, planner, controller, or backend.

The default playback allowlist is only `/joint_states`,
`/edgegrasp/target_3d`, and three raw camera topics. `edgegrasp_replay` has no
planner, MoveIt adapter, backend, or physics. Restart it or coordinate a new
epoch before rewinding the same bag; an unchanged wrapper correctly treats
rewind as clock rollback.

## Upstream composition

- `adoodevv/so101_ros2` @
  `0305e03ab54e64aae9263fcbf339622e654012f3`, BSD-3-Clause: pinned Gazebo,
  ros2_control, camera, controllers, and complete-looking MoveIt configuration.
  The tree and selected runtime paths are verified. Its README is stale about
  both MoveIt completeness and the gripper controller; the actual pinned YAML
  maps both controllers to FollowJointTrajectory.
- `TheRobotStudio/SO-ARM100` @
  `7629d2ad9853d10fb903093a33ef6114099d97e5`, Apache-2.0: geometry,
  URDF/MJCF, mesh, and calibration cross-check. Base collision meshes are
  missing and gripper 0/100 semantics are not encoded. Windows MuJoCo runtime
  remains independently blocked.
- `legalaspro/so101-ros-physical-ai` @
  `58318c905a2c61289fa907de85cb8473322fbe68`, Apache-2.0, Feetech submodule
  `4c0fdbfe16c84c686f8ace09526c52d98d0110ca`: remote pinned reference for
  mock/real, MoveIt, recording, and hardware safeguards. It was not cloned or
  run and has no Gazebo/MuJoCo backend.

No upstream source is copied into this project. Checkouts stay under
`workspaces/` and overlapping SO-101 workspaces are never sourced together.

## Layout

```text
src/edgegrasp/                         deterministic core + pinned contract
tests/                                 Windows/CI core and structural tests
scripts/                               checks, audit, benchmark, MCAP tools
ros_ws/src/edgegrasp_core/             monorepo ament adapter
ros_ws/src/edgegrasp_interfaces/       TrackedTarget + four typed actions
ros_ws/src/edgegrasp_ros/              monitor, probe, FJT gate, launches
ros_ws/src/edgegrasp_moveit_adapter/   IK + MoveGroup plan-only bridge
ros_ws/src/edgegrasp_grasp_sequence/   correlated four-stage action wrapper
docs/                                  runbook, plan, manifest, evidence report
```

## Hardware decision

Buying one SO-101 follower arm is now justified as the next validation tool,
not as proof that the project is finished. Candidate024 supplies 10 scoped
simulation-physics successes, including 9/9 after the freshness-scheduling fix,
with bilateral pad contact, 28.858-29.128 mm retained lift, and a 0.5 s
retention window. The highest-value next unknowns are now hardware calibration,
the real gripper mapping/contact geometry, stop behavior, and sim-to-real
timing. Start with one follower and conservative unloaded/joint-space tests;
do not immediately buy an edge GPU. Add a camera after joint and gripper
calibration is stable enough to validate timestamp, frame, and extrinsics on
real data.

This phase deliberately excludes VLA, GraspNet, FoundationPose, custom
TensorRT kernels, and reinforcement learning. It contains no company-private
code, SDK, model, or data. EdgeGrasp-owned code is MIT licensed; upstreams
retain their licenses recorded in the manifest.
