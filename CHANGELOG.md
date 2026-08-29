# Changelog

## Unreleased

- Split MoveIt evidence into three non-interchangeable classes:
  `REAL_MOVEGROUP_NORMAL` for scoped plan-only/zero-execution responses,
  `INJECTED_CANCEL_TIMEOUT` for fake/in-process cancellation and timeout
  contracts, and `REAL_MOVEGROUP_CANCEL_RACE_NEGATIVE_EXISTING_ARTIFACT` for
  the frozen P2 upstream-process failures.
- Record the actually executed WSL P2 probe/runner hashes only as historical
  artifact provenance. Both 2026-08-29 runs kept EdgeGrasp fail closed with
  zero actual motion goals but have `summary.status=UPSTREAM_PROCESS_EXITED`, not
  PASS: each
  real MoveGroup run recorded one SIGSEGV and exit `-11` in
  `PlanExecution::stop()`, so `overall_pass=false` and
  `move_group_survived=false`. The signal-injection scripts are not active
  release commands.
- Stop rerunning OS-process interruption. Add an injected-only runner using the
  existing in-process fake MoveGroup ActionServer; two existing tests cover
  explicit cancel, goal-response timeout, late accepted-goal cancel, and zero
  trajectory/gate publication without increasing test count. Accepted-goal
  result-future timeout remains unverified.
- Record the 2026-08-30 safe in-process artifact at
  `/home/edgegrasp/ros2_ws/test_results/injected_moveit_fail_closed_20260830T001525`:
  `summary.status=PASS`, `overall_pass=true`, and 2 tests passed in 2.04 s, covering
  explicit cancel and goal-response timeout; the late accepted fake goal was
  canceled and trajectory/fake-gate goal counts were zero. This is not real
  MoveGroup, controller, or hardware evidence.
- Fix a late send-future cancellation race in the MoveIt adapter. A goal
  accepted after the wrapper has already failed closed is now canceled even
  when request cleanup has not yet cleared the active request identity; apply
  the same rule to a late accepted typed-gate goal.
- Add a zero-runtime-dependency deterministic grasp-pipeline core.
- Add timestamped `Target3D`, constant-velocity prediction, a 200 ms stale
  source gate, active receive-stream watchdog, explicit frame/clock epoch,
  endpoint-only workspace blocking, strict plan protocol, and fail-closed
  mock/Gazebo/real backend selection.
- Add deterministic static, 20 mm/s, and 40 mm/s scenarios and repeatable
  replay/benchmark tooling.
- Add pytest coverage for prediction, source/receive freshness, clock rollback,
  frame/confidence gates, contradictory plans, reset/stop failures, synchronous
  execution, endpoint limitations, packaging, and 100-run core replay.
- Add a built and scoped-runtime-tested ROS 2 overlay with a packaged core
  dependency, mock target publisher, ROS-clock safety monitor, fail-closed dual
  arm/gripper FJT gate, endpoint probe, table/cube SDF, Gazebo upstream overlay,
  and single-clock MCAP tooling.
- Pin and validate three upstream repositories in a machine-readable manifest.
- Add a packaged pinned SO-101 joint/controller/action contract, pure contract
  validator, local-checkout structural audit, and collected Jazzy fake-action
  tests for send/result/cancel/watchdog failure paths.
- Retract the incorrect missing-MoveIt finding after inspecting the clean pinned
  adoodevv checkout; preserve static-versus-runtime evidence boundaries.
- Add an original-asset SO-ARM100 MJCF static validator and record the Windows
  MuJoCo host-runtime failures without claiming model stepping.
- Add `PlanTarget.action` and an arm-only tf2/IK/MoveGroup `plan_only` adapter
  that validates `RobotTrajectory` independently and publishes only through the
  EdgeGrasp FJT gate.
- Verify scoped Jazzy/Harmonic runtime: controller activation, joint states,
  both FJT paths, MoveIt planning, one reachable target-to-gate arm trajectory,
  camera topics, and raw-input MCAP replay; retain explicit collision/grasp and
  hardware exclusions.
- Add a single-source table/cube scene contract, generated Gazebo SDF, and a
  service-confirmed MoveIt PlanningScene loader. Gate the plan-only adapter on
  recent scene readiness, fix periodic-requery false pulses, and preserve
  machine-readable runtime A-E evidence: one reachable gated FJT success plus
  zero-dispatch table-inside, invalid table-crossing, unreachable, stale, and
  epoch-mismatch rejections.
- Add a dependency-free four-stage grasp-sequence controller and a typed ROS
  `GraspSequence.action` wrapper. Serialize approach, descend, separate gripper
  close, and lift; require exact task/command/stage/digest and legal FJT terminal
  evidence; retain stop-uncertain slots on contradictions; make client cancel
  win terminal-success races; and keep `physics_grasp_verified=false` pending
  contact, cube lift, and retention evidence.
- Freeze explicit normalized approach and grasp orientations per GraspSequence
  task, reuse the grasp orientation for descend/lift, reject either fail-closed
  zero default, and validate one complete scoped
  Gazebo/MoveIt run through three arm FJT goals plus one gripper FJT goal.
  Preserve `physics_grasp_verified=false` and record a later joint-state-stale
  repeat as SAFE_STOP rather than presenting one success as deterministic.
- Make the Jazzy grasp-sequence test harness wait for ActionServer executor
  quiescence before destroying nodes, surface retained callback exceptions,
  and close the prior unfetched `InvalidHandle` teardown warning. That
  package-scoped snapshot was 106/106 across all five EdgeGrasp packages, with zero
  XML failures, errors, or skips; the separate `Destroyable` warning remains.
- Replace the repeat harness's `ros2 run` wrapper ownership with direct installed
  entry points, give every run an isolated action endpoint, detect traceback or
  lingering-process shutdown failures, and guard rclpy context teardown. Retain
  the earlier 4/10 failure, invalid wrapper-PID batch, and 1/2 shutdown-race
  batch; the corrected runtime artifact completes 10/10 typed protocol runs
  with clean per-run sequence shutdown while keeping
  `physics_grasp_verified=false`.
- Add a read-only `GraspPhysicsEvidence` observer that correlates immutable task,
  target, scene, clock, sequence terminal, cube pose, and contact evidence while
  exposing no motion client. Re-sample ROS time under the same lock as every
  strict-core operation after a clock-read-only fix was disproved at runtime.
- Bound the MoveIt adapter's post-dispatch wall wait with a 15-second guard so a
  slow Gazebo real-time factor cannot preempt the typed gate's ROS-clock safety
  contract. Final task006 completed approach, descend, gripper close, and lift;
  the independent observer measured zero gripper contacts and unchanged cube Z,
  so the delivered result remains sequence success and physics grasp false.
- Add one EdgeGrasp-owned MoveGroup launch overlay that keeps the pinned
  upstream arm model, adds only the two distal contact boxes, disables direct
  MoveGroup execution, and injects Pilz `ValidateSolution`. Runtime rejected the
  previously accepted mid approach as `INVALID_MOTION_PLAN` for target-cube
  versus `gripper_link` collision while a farther approach executed through the
  typed gate without moving the cube.
- Confirm table-plus-cube and table-only PlanningScene modes from the same scene
  digest. Task039 completed the correlated four-stage sequence after a manually
  staged target-contact transition and observed gripper/cube contact, but peak
  lift was only 2.27 mm, the cube moved about 60 mm laterally, and retention was
  false. Preserve this as physics-failure evidence, not a grasp claim.
- Replace the task039 palm-contact interpretation with an exact distal-pad
  observer contract. Candidate002 is rejected with the cube present;
  candidate003 passes the far approach and table-only descend/lift gates, then
  completes all four typed/FJT stages. Zero distal-pad contacts, only 2.43 mm
  peak lift, no retention, and 52.3 mm lateral displacement remain explicit
  negative physics evidence.
- Replace the candidate003 AABB-only pad preflight with exact OBB SAT, move the
  Gazebo pad collisions onto two dedicated child links, and add a
  service-confirmed selective MoveIt ACM that keeps the target cube while
  allowing only those pads to contact it. Candidate004 plan-only runtime
  correctly rejects approach-to-descend for cube versus `gripper_link` and
  accepts the closed-gripper lift control; no trajectory or grasp was executed.
- Let an already-confirmed PlanningScene loader supersede its own in-flight
  periodic read-only revalidation when applying the selective pad policy, while
  preserving fail-closed refusal before the first confirmation.
- Require every configured gripper-pad token in the same contact sample before
  refreshing the physics-retention clock. Separate one-sided contacts no longer
  compose into bilateral evidence; that Jazzy package-scoped snapshot passed
  106/106 (`core=1`, `interfaces=0`, `ROS=49`, `adapter=21`, `sequence=35`).
- Preserve a candidate005 clean-graph 1 ms simulation-clock rollback as
  SAFE_STOP evidence, then perform one fresh-domain retry under the actual
  EdgeGrasp proxy MoveGroup overlay. The correlated sequence completed and 35
  samples contained both pad tokens, but peak cube lift was only 0.332 mm and
  retention failed; keep `physics_grasp_verified=false`.
- Correct the earlier candidate005 histogram record: it accidentally launched
  pinned upstream MoveGroup. Its Gazebo contact counts remain physical evidence,
  but it is explicitly excluded from EdgeGrasp proxy-planning evidence.
- Extend the read-only physics observer with per-pad penetration/wrench maxima,
  exact gripper-joint effort, first/last source timestamps, same-sample contact
  timing, and sequence-terminal snapshots while leaving the lift/retention
  success gate unchanged. Candidate006 measured contact quality; candidate007
  showed only 0.099 simulated seconds of bilateral contact ending 1.747 seconds
  before sequence completion, 0.359 mm peak lift, and no retention.
- Add a reusable candidate contact-quality/timing harness and an exact
  `ROS_DOMAIN_ID` cleanup helper. The harness now requires true safety and
  interface samples, validates target ID/scene coordinates, avoids a
  `pipefail`/SIGPIPE log-selector trap, and preserves both pre-motion aborts as
  negative evidence rather than motion trials.
- Record cube pose/contact, table contact, sequence terminal, and physics status
  in both MCAP record profiles while keeping every derived/evidence/command
  topic out of the default raw-input replay allowlist.
- Add a fixed-SHA, dependency-free generator for gripper-angle-specific moving
  pad OBB/AABB profiles and route the selected profile through the runtime
  client/harness. Candidate008 changed only the generated geometry and close
  command from 0.60 to 0.50 rad: simultaneous contact rose from 39 samples over
  0.099 s to 98 over 0.252 s and peak lift rose from 0.359 mm to 2.667 mm, but
  contact still ended 1.788 s before completion and retention failed. Preserve
  this as a stronger negative/control result with `physics_grasp_verified=false`.
- Add an isolated, chained `GetMotionPlan` candidate-admission harness. It
  launches no adapter, gate, sequence, ExecuteTrajectory client, or FJT client,
  validates and discards all three RobotTrajectories, and records explicit
  zero-execution counters. Candidate009's -0.9 mm static symmetry selection and
  every sampled negative offset down to -0.1 mm failed approach-to-descend with
  `NO_IK_SOLUTION`; the unchanged zero-offset control passed 10/10.
- Generate and validate 0.45/0.40 rad Candidate010/011 profiles while keeping
  the reachable arm poses fixed. Both passed the isolated three-segment gate
  10/10 before typed execution. Candidate011 improved transient bilateral
  contact to 171 samples over 0.463 simulated seconds and peak lift to 3.711
  mm, but the cube returned to the table and retention remained false. Stop the
  angle search and keep `physics_grasp_verified=false`; next study one bounded
  pad material/friction or distal-proxy geometry variable.
- Add the Candidate012 machine-readable pad-material contract, bounded
  `mu=mu2` profiles, URDF-to-SDF validator, plan-only wrapper, and paired
  runtime harness. Both explicit rows complete the typed sequence on the same
  final launch snapshot, but `mu=1.0`/`1.5` reach only 3.392/3.468 mm peak
  lift and both fail retention. Stop the friction sweep and preserve
  `physics_grasp_verified=false`.
- Add a bounded Candidate014 moving-pad simulation attachment that preserves
  the proximal edge while extending only distal-pad local-Y length by 12 mm.
  The corrected 3-attempt chained plan-only gate passes with zero execution;
  one typed runtime completes all four stages but reaches only 3.680 mm lift,
  0.462 s bilateral contact, and no retention. Preserve it as negative evidence
  and do not treat the attachment as upstream or hardware geometry.
- Fix a real candidate-harness race by requiring base PlanningScene
  confirmation, `SetBool success=True`, and a confirmed selective-ACM status
  before planning. The superseded rejection is retained as harness evidence,
  not misclassified as a geometry failure.
- Delay the synthetic target publisher by 3 s behind the Gazebo `/clock` and
  fail-closed consumer startup. This removes a startup ordering race without
  changing the future-target safety gate. Preserve the rejected pre-motion
  attempts and one observer-baseline failure caused by a separately identified
  orphan diagnostic Gazebo process.
- Historical 21:07 snapshot: Windows CPython 3.10.11 passed
  286/286 project tests and all three 100-run core replays; the isolated Jazzy
  result base passes 112/112 package tests (`core=1`, `interfaces=0`, `ROS=49`,
  `adapter=21`, `sequence=41`) with zero XML errors, failures, or skips.
- Revalidate the Candidate011 snapshot: Windows CPython 3.10.11 passes 308/308
  project tests and all three 100-run core replays; CPython 3.10/3.14
  compileall, Ruff, 35 JSON parses, 9 Git Bash syntax checks, and 4 PowerShell
  parses pass. The isolated Jazzy result base passes 113/113 package tests
  (`core=1`, `interfaces=0`, `ROS=49`, `adapter=21`, `sequence=42`) with zero
  XML errors, failures, or skips.
- Revalidate the Candidate012 snapshot: Windows CPython 3.10.11 passes 330/330
  project tests and all three 100-run core replays; full-tree Ruff, 38 JSON
  parses, 11 Git Bash syntax checks, and 4 PowerShell parses pass. The final
  sequential Jazzy result base passes 113/113 package tests with zero XML
  errors, failures, or skips.
- Revalidate the Candidate014 snapshot without adding test cases: Windows
  CPython 3.10.11 remains 330/330 with all three 100-run replays; CPython
  3.10/3.14 compileall, whole-tree Ruff, 40 JSON parses, 11 Git Bash syntax
  checks, and 4 PowerShell parses pass. Five Jazzy packages build; the isolated
  final result base is 113/113 after one existing PlanningScene test timed out
  once, passed alone, and then passed in the package rerun.
- Add Candidate024 from measured Candidate022 contact geometry: rotate the cube
  face 37.243 degrees into the gripper closing axis and translate it 9.7 mm to
  restore a 1.002 mm fixed-pad pre-close gap without changing the reachable arm
  stage poses. Its isolated three-segment plan-only gate passes 10/10 with zero
  execution and stable per-segment digests.
- Diagnose the Candidate024 r02 stale-target SAFE_STOP as client scheduling,
  not random MoveIt failure. Recheck the exact target immediately before
  sequence submission with a 100 ms admission bound; rebuild a stale read-only
  baseline at most twice and only after a correlated CANCELED /
  `observer_cancelled` terminal with matching identity; generation-filter late
  feedback. Runtime r04 exercises the 242 ms reject/cancel and 52 ms replacement
  path.
- Record Candidate024 r01-r11: 10 scoped simulation-physics grasp successes,
  one pre-motion stale rejection, and post-fix r03-r11 at 9/9. Successful
  retained lift spans 28.858-29.128 mm with both configured distal-pad contacts
  and a 0.5 s retention window. Preserve the proxy-scene and hardware claim
  boundaries.
- Permit an empty lift command ID only on an unsuccessful sequence terminal so
  the observer reports `sequence_not_completed` instead of a construction
  exception; successful terminals still require the exact lift command ID.
- Revalidate the Candidate024 snapshot: Windows CPython 3.14.5 passes 332/332
  and all three 100-run core replays; the five EdgeGrasp Jazzy packages build
  and pass 118/118 package-scoped tests (`core=1`, `interfaces=0`, `ROS=49`,
  `adapter=23`, `sequence=45`) with zero XML errors, failures, or skips.
- Add one Candidate024 target-matrix contract and one zero-execution runner
  without adding test cases. The retained first run disproves rigid Cartesian
  translation as a positive-case generator: 10/10 proposals fail first-stage
  IK while the unchanged control passes all three segments.
- Correct the generator from the pinned URDF: `shoulder_pan` is offset from the
  base origin and its local +Z maps to base-frame -Z. Applying that exact joint
  transform yields 10/10 distinct three-segment plan-only passes; four invalid
  scenes and six distant targets are rejected as declared. The 20-case matrix
  has zero false accepts, false rejects, unverified cases, motion dispatches,
  tracked DART mesh diagnostics, or geometry-construction failures.
- Execute only the preselected near-control target and both shoulder-pan range
  endpoints once each through `PlanTarget -> ExecuteTrajectory -> FJT`. All 12
  correlated typed terminals return wrapper status 4 and FJT error code 0; the
  independent observer verifies bilateral pad contact, 28.965-29.095 mm
  retained lift, and the 0.5 s retention window in all three runs. Preserve the
  one-run-per-pose, proxy-physics, collision-fidelity, and hardware boundaries.
