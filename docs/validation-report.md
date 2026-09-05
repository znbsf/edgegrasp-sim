# EdgeGrasp validation report

Baseline snapshot: 2026-08-30, Asia/Shanghai (`+08:00`). The 2026-09-05
focused injected update is recorded separately below. Simulator runtime
counts remain from 2026-08-29; baseline package results are historical.

This report separates the ROS-independent core, EdgeGrasp ROS packages,
pinned-upstream tests, scoped simulator observations, and unverified claims.
An earlier success count is never reused as current evidence.

## 2026-09-05 focused injected source-provenance verification

The [new observation](observations/2026-09-05-injected-source-provenance-runtime.json)
records a fresh Ubuntu-24.04/Jazzy run of the prescribed in-process runner:
12 tests passed in 6.00 s, 12 JSONL records, and all 15 required facets verified.
The pytest process imported the installed adapter from the WSL workspace;
its SHA-256 matched the declared Windows checkout source. JUnit, event count,
and artifact hashes were independently checked. Missing/mismatched provenance
regressions have Windows coverage; this Jazzy run exercised the matching path.

The initial launcher failed before pytest; the successful runner returned 0,
but its outer launcher exit command encountered a carriage return. Both launch
errors are retained and described in the observation. This is injected evidence
only, not a new five-package, real MoveGroup, controller, physics, or hardware
verification. The historical snapshots below remain unchanged.

## 0. Historical 2026-08-30 snapshot

### Windows core, structure, replay, and syntax

The prior fully pinned Windows command started at
`2026-08-29T03:29:58.2221980+08:00` and ended at
`2026-08-29T03:30:22.1056596+08:00`. Pytest reported 21.64 s; the timestamped
wrapper interval was 23.883 s and collected/passed 332/332 from one stable
filesystem snapshot.

```powershell
scripts\check.ps1 -ReplayRuns 100
py -3.10 -m compileall -q src tests scripts ros_ws
py -3.14 -m compileall -q src tests scripts ros_ws
C:\Users\huang\scoop\shims\ruff.exe check .
py -3.14 -c "# parse every project JSON under docs, src, and ros_ws/src"
# Git Bash bash -n over all 11 project .sh files
# PowerShell Parser.ParseFile over all four project .ps1 files
```

Observed:

```text
CPython 3.14.5 / pytest 9.1.1 / pluggy 1.6.0
collected: 332
passed:    332
failed:    0
skipped:   0
check.ps1 pytest wall time: 21.64 s
timestamped wrapper interval: 23.883 s
CPython 3.10 compileall: PASS
CPython 3.14 compileall: PASS
Ruff whole tree: PASS
project JSON parser: 55/55 PASS
project structure: PASS
Git Bash syntax: 11/11 scripts PASS (static only)
PowerShell parser: 4/4 scripts PASS
check.ps1 exit: 0
```

One first JSON audit command named nonexistent `scene_contract.json` instead
of the checked-in `scene.json` and exited 1 before parsing that list. The
corrected directory-enumerating command parsed all 21 artifacts in that older
snapshot; the current final parser covered 55/55. This is recorded as an
operator-command error, not a project parse failure.

The same `check.ps1` run replayed the 0/20/40 mm/s scenarios 100 times each.
All were deterministic, accepted 20/20 cycles, rejected zero, and produced
digests `0a6ba6f7...`, `854cf2da...`, and `da7fcd92...` respectively. This is
core replay determinism only.

The current P12d worktree was rerun on Windows on 2026-08-30 after the SDF
source pins and ROS integration tests changed. `scripts/check.ps1` collected
332 tests: 330 passed and two were skipped solely because the optional pinned
local SO-101 checkout was absent; it exited zero, structural validation passed,
and all three 100-run replay digests remained deterministic. The explicit
post-run gates also passed: Ruff over the full tree, 66/66 unignored
project-source JSON files, 12/12 Bash scripts, 4/4 PowerShell scripts, and
`git diff --check`. A 67th JSON under ignored `build/lib` was also parseable but
is excluded from the reproducible source-file count. This current
checkout result does not replace the older fully pinned 332/332 artifact; it
states the exact dependency availability of this worktree.

### Ubuntu 24.04 / ROS 2 Jazzy packages

The complete project layout already existed in the disposable guest. This turn
synchronized the exact changed adapter, integration test, runner, and two SDF
contract JSON files with no broad delete; the P12d artifact verified the exact
adapter/test/runner SHA-256 values. The adapter package was rebuilt before the
current package-scoped P12d result:

```text
/home/edgegrasp/ros2_ws/test_results/edgegrasp_all_packages_20260830T121257P12d
```

```bash
colcon build --packages-select edgegrasp_moveit_adapter --symlink-install \
  --event-handlers console_direct+
colcon test --packages-select \
  edgegrasp_core edgegrasp_interfaces edgegrasp_ros \
  edgegrasp_moveit_adapter edgegrasp_grasp_sequence \
  --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/edgegrasp_all_packages_20260830T121257P12d \
  --event-handlers console_cohesion+ --return-code-on-test-failure
colcon test-result --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/edgegrasp_all_packages_20260830T121257P12d \
  --all --verbose
```

The P12d package-scoped verification completed all five selected packages with
129/129 tests passed, zero failures/errors/skips, under the same disposable
Ubuntu/Jazzy package environment. Its package breakdown is:

| Package | Collected | Passed | Failed | Errors | Skipped |
| --- | ---: | ---: | ---: | ---: | ---: |
| `edgegrasp_core` | 1 | 1 | 0 | 0 | 0 |
| `edgegrasp_interfaces` | 0 | 0 | 0 | 0 | 0 |
| `edgegrasp_ros` | 49 | 49 | 0 | 0 | 0 |
| `edgegrasp_moveit_adapter` | 34 | 34 | 0 | 0 | 0 |
| `edgegrasp_grasp_sequence` | 45 | 45 | 0 | 0 | 0 |

EdgeGrasp total: 129/129, zero XML failure/error/skip. Results were queried from
the P12d package artifact and each package's exact build result base. The
standalone adapter result at
`/home/edgegrasp/ros2_ws/test_results/edgegrasp_moveit_adapter_20260830T121050P12d`
also passed 34/34 after one stale reason assertion was corrected. Whole-workspace
`colcon test-result --all` is not used as the EdgeGrasp verdict because the same
workspace retains older pinned-upstream lint results. Earlier intermittent ROS
teardown/timing observations remain historical evidence below; the corrected
P12d package run was green, and no timeout or safety policy was relaxed.

One earlier wrapper invocation enabled shell nounset before
sourcing `/opt/ros/jazzy/setup.bash`; the ROS setup script referenced the
unset probe variable `AMENT_TRACE_SETUP_FILES` and exited before build or test.
The authoritative run sourced ROS first and did not relax any project check.
One subsequent attempted variable-based `rsync` command was expanded by the
Windows native-command boundary into a root self-scan. It used no `--delete`,
reported permission errors, and was interrupted before any build or test. A
literal-path dry run then showed only the intended project files, and the
literal-path sync plus the build/test result above supersede that operator
error.

### MoveIt evidence classes and delivery addendum

The following evidence classes are non-interchangeable. Package-test success
does not expand the real MoveGroup runtime claim:

| Evidence class | Result | Evidence and boundary |
| --- | --- | --- |
| `REAL_MOVEGROUP_NORMAL` | `PASS_SCOPED` | The independent Candidate024 target-matrix artifact ran the real MoveGroup process under the EdgeGrasp proxy overlay for normal `GetMotionPlan`. It matched 20/20 declared outcomes, validated and discarded 30 accepted trajectories, and recorded zero trajectory publications, ExecuteTrajectory goals, FJT goals, or execution attempts. This proves only normal planning/validation, not cancel or process health. |
| `B · INJECTED_CANCEL_TIMEOUT` | `summary.status=PASS`, `overall_pass=true` | `/home/edgegrasp/ros2_ws/test_results/injected_moveit_fail_closed_20260830T121122P12d` ran 12 focused integration tests with in-process fake MoveGroup and typed-gate ActionServers. All 12 tests, 12 JSONL records, and 15/15 required facets passed with zero failures/errors/skips and validator error count zero. It directly observes accepted result-future timeout, strong request/generation/UUID correlation, delayed MoveGroup and typed-gate goal-response cancellation, gate `get_result_async()->None` and result exception, terminal confirmation/non-confirmation, success-after-cancel fail-closed behavior, MoveGroup result-future unavailability, old-generation late-CANCELED isolation, old-generation late-SUCCEEDED isolation, and explicit cancel. The MoveGroup unavailable-future case is injected at the adapter seam before a real ClientGoalHandle result future exists; it is not real MoveGroup evidence. Injected-scoped positives are `accepted_goal_result_timeout_verified_injected`, `accepted_goal_result_future_timeout_verified_injected`, `strong_move_group_request_id_correlation_verified_injected`, `gate_delayed_goal_response_cancel_verified_injected`, `gate_result_future_unavailable_verified_injected`, `gate_result_exception_verified_injected`, `move_group_result_future_unavailable_verified_injected`, and `old_generation_late_success_isolation_verified_injected`; unqualified, real MoveGroup, controller, simulation-physics, and hardware fields remain false. |
| `A · REAL_MOVEGROUP_CANCEL_RACE_NEGATIVE_EXISTING_ARTIFACT` | `summary.status=UPSTREAM_PROCESS_EXITED`, `overall_pass=false` | The two frozen 2026-08-29 P2 artifacts kept `edgegrasp_fail_closed=true` with zero actual motion goals, but each real MoveGroup run recorded one SIGSEGV and exit `-11` in `libmoveit_plan_execution.so.2.12.4` `PlanExecution::stop()` at address `0x5c`; both have `move_group_survived=false`. They are historical negative artifacts and the process-interruption injection is not rerun. |

Track A consists of these two read-only WSL artifacts:

```text
/home/edgegrasp/ros2_ws/test_results/real_moveit_fail_closed_explicit_cancel_20260829T2250P2
/home/edgegrasp/ros2_ws/test_results/real_moveit_fail_closed_goal_response_timeout_20260829T2302P2
```

The current P12d safe artifact started at
`2026-08-30T12:11:24,512653509+08:00` and ended at
`2026-08-30T12:11:31,390345731+08:00`. Its summary SHA-256 is
`11aefd9dbc2ead97b7445e35bf38e9df6cf9df8581e022aa9a1f977f99e8492d`;
focused 12-test JUnit SHA-256 is
`b1565f63afe8759e0c6b3313ec7248ac0858763fbdcb0dad59b44cf1682385ec`;
12-record JSONL ledger SHA-256 is
`b4dcdc675bb174c3f96edcf8e2d9aef5144bca0bfa4202361ab2d8bbaa5a19a4`;
pytest log SHA-256 is
`fd762f3022eed4ff751d9663899773eedef7525321375d83167a138005e827e2`;
`start.txt` SHA-256 is
`7bf2ebaa853dd06de11449a722729d0c2a4b90dad13fd77ad373edeb2b94564c`;
and `end.txt` SHA-256 is
`6c3941628ef71009fe5ec3887b6630faf8ef855552b85200a79172db98a7b113`.
The runner checked all 15 required facets and reported no validation errors.
The source snapshot hashes are: adapter
`065bed61a7141efe22d75c5da29c583d0ad166c96a1f144f9ba0ba71c2c90672`
(3,243 lines), integration test
`c6954f1fa18715f523833da30312790498c8c9c0f6fb73b462d0b71d5a20f54f`
(2,604 lines), and runner
`6f4e613157cea3dd39d130de1b1a5540d79f6754b57bd1dcd28bb476bb660333`
(2,297 lines). The exact accepted MoveGroup client handle, its result future,
the monotonic attempt generation, the `edgegrasp:<request_id>` constraint,
and canonical client/server UUIDs are joined before the injected-scoped
strong-correlation flag can become true. Cancel acceptance is not treated as
terminal completion. P12d also directly tests MoveGroup result-future
unavailability at the adapter seam and old-generation late-SUCCEEDED isolation;
the former is not a real MoveGroup ClientGoalHandle observation.
The superseded P11 artifact remains historical lineage at
`/home/edgegrasp/ros2_ws/test_results/injected_moveit_fail_closed_20260830T023710P11`
with 10 tests, 10 records, and 13 facets; its exact hashes remain in the P11
observation and are not current P12d provenance.
The complete machine-readable current record is the
[P12d injected observation](observations/2026-08-30-injected-moveit-result-timeout-correlation-v4-runtime.json).

The current source-contract byte pins use the checked-in LF SDF bytes:
Candidate012 `table_cube.sdf` is
`8a4b436cd4863a0801602d15bfafebecaadfbe104d4b7db93e4a5f33415bb4c4`, and
Candidate024 `table_cube_candidate024_face_aligned.sdf` is
`058e3237b430c56dbcfcde1091573bf8ce6827e6322114c094011a16095af0ba`.
Historical observations that captured CRLF working copies retain their
original hashes and remain historical provenance, not current canonical-LF
source pins.

The historical P10 safe artifact started at
`2026-08-30T02:29:45.815253993+08:00` and ended at
`2026-08-30T02:29:50.904413574+08:00`. Its summary SHA-256 is
`47a6d7194589b83c3efc67613b37aa8a4568a6a1075f711801f2965a57b823b1`;
focused 10-test JUnit SHA-256 is
`d6547c72a0eecab4e536d54d558f086fcf1398fdcdeec36d0e21e78561ba24f5`;
10-record JSONL ledger SHA-256 is
`b1862752eafb44ef9cc42fb56e2c11d3ced9fadd302cd3e51ce13a8d962c5389`;
pytest log SHA-256 is
`ed49f7a1f5ef55b1d99a0398acdd028e1267f6eb1b98a497dd968163fa1413d8`;
`start.txt` SHA-256 is
`5cd1ad960b11da45a673a0874d212b9bf2942235e9f86680970720e28b03aae9`;
and `end.txt` SHA-256 is
`21b3027eb7cff5198e5c0adc3a8be81e89b7603c54f1bbc8a8aa3d1c51406170`.
The runner checked all 13 required facets and reported no validation errors.
The source snapshot hashes are: adapter
`477bb7b95b41f5019e60020d6d7e6830da2040104edc5de508d383f5a4124b4d`
(3,239 lines), integration test
`d1b21f86b2a7dd4c18feb7d4de172c05995a50a6b365d61efd5fa9d36003a6e5`
(2,293 lines), and runner
`4458fd4c384313d3990bf57d07da86eb1fa09589ca41cdb2169863dc7d62af3e`
(1,862 lines). The exact accepted MoveGroup client handle, its result future,
the monotonic attempt generation, the `edgegrasp:<request_id>` constraint,
and canonical client/server UUIDs are joined before the injected-scoped
strong-correlation flag can become true. Cancel acceptance is not treated as
terminal completion.

The earlier complete-package result remains a historical 23/23 result for its
earlier source, with `pytest.xml` SHA-256
`4d3a5bee82ee9c27c006d3a70c5d87fc8ef9433fc3224f86b8be1ef7cfac46cf`. The P11
artifact is also historical lineage at the path recorded above; its exact
source and artifact hashes remain in the P11 observation. P11 directly covered
the accepted-result timeout, gate unavailable/exception, and late-CANCELED
generation cases available at that snapshot, but did not directly claim the
MoveGroup unavailable-future seam or stale late-SUCCEEDED isolation now covered
by P12d. Current injected-scoped positives and all unqualified/real,
controller, simulation-physics, and hardware boundaries are defined by the
P12d summary above.
The superseded P8 baseline remains archived at
`/home/edgegrasp/ros2_ws/test_results/injected_moveit_fail_closed_20260830T021211P8`
with 8 tests and 11 facets; it is not used for the current claim.

The P2 capture manifests record byte identity between the WSL capture and a
Windows incubator recheck. This standalone repository preserves the actually
executed probe and runner only as historical provenance with exact SHA-256
`39f8615370cf73199699a8ddb0905ab55f8dd28d3d74ae41094ba50dbbde87cd`
(983 lines) and
`f67d55539780c70b2c5ab51e2d1b212bc4691b9e9ab8693e4207250a845d77e2`
(778 lines); the capture adapter SHA is
`1b238393b12f86d076c82303970f9d35824931bbf38d3a8af054cf07b0ab2658`.
The executable historical signal-injection files are deliberately excluded;
the current standalone checkout uses the in-process safe runner. Both P2 runs
report `edgegrasp_fail_closed=true`, all three actual-motion goal counts
zero, joint drift `6.2449966191115345e-19` /
`9.714448921889736e-19` rad, and late MoveGroup status 4 occurring
14,464,841 / 25,449,221 ns after the wrapper terminal. Cleanup succeeded with
zero remaining scoped processes and `/clock` unknown. Correlation is explicitly
fresh-graph runtime inference, so within those historical real-process P2
artifacts `strong_move_group_request_id_correlation=false`; accepted-goal
result timeout is also unverified. See the machine-readable
[MoveIt evidence matrix](observations/2026-08-29-moveit-evidence-matrix.json)
and [frozen negative observation](observations/2026-08-29-real-moveit-fail-closed-runtime.json).

The separate 212 ms `stale_target` rejection against the unchanged 200 ms gate
and `planning_scene:safety_false` after the MoveGroup crash/service loss are
correct fail-closed outcomes, not false positives. Neither the 200 ms stale
gate nor frame, epoch, future-skew, or typed-gate constraints were relaxed.

### Candidate024 face-aligned simulation grasp and freshness fix

Candidate024 is the first scoped positive simulation-physics grasp result. It
was derived from Candidate022's observed contact geometry: the gripper closing
axis was 37.243 degrees from the nearest cube-face normal. The scene rotates
the cube by that measured angle and translates it 9.7 mm to restore a 1.002 mm
fixed-pad pre-close gap. The reachable arm stage poses, 0.40 rad gripper close,
explicit `mu=mu2=1.0`, zero moving-pad extension, and -0.02 Nm simulated preload
remain fixed.

The isolated plan-only artifact is:

```text
/home/edgegrasp/ros2_ws/test_results/
  candidate024_face_aligned_plan_only_10x_20260829T0225
```

All 10 attempts and all 30 chained arm segments returned MoveIt error code 1,
passed trajectory validation, and produced one stable digest per segment.
Trajectory publication, ExecuteTrajectory goals, FJT goals, and execution were
all zero.

The first typed runtime (`r01`) met the strict observer contract. The immediate
repeat (`r02`) did not: its target source stamp was `27,600,000,000` ns, the
approach dispatched at age 181 ms, and the adapter reached its planning boundary
at age 208 ms. The 200 ms source gate correctly returned
`plan_or_execution_failed:stale_target` before any trajectory, typed execute,
or FJT side effect. The cause was in the trial client: it checked freshness,
then waited for five read-only physics-baseline samples and submitted the same
snapshot without a second check.

The corrected client checks the exact snapshot immediately before sequence
submission against a 100 ms admission limit, leaving at least 100 ms before the
unchanged 200 ms core deadline. A stale baseline may be rebuilt at most twice,
but only after cancel acceptance plus a correlated CANCELED terminal with
reason `observer_cancelled` and matching task, target, source timestamp, clock
domain, and epoch. Late feedback is filtered by observation generation. r04
exercised this branch in the live graph: attempt 1 was 242 ms old; its cancel
terminal and identity matched; attempt 2 used a new target at 52 ms and then
completed the grasp. A cancel timeout, exception, rejection, wrong terminal, or
identity mismatch remains a zero-motion error.

Runtime results across r01-r11:

| Metric | Observed |
| --- | ---: |
| Attempts | 11 |
| Physics-success results | 10 |
| Pre-motion stale SAFE_STOP | 1 |
| Post-fix r03-r11 | 9/9 physics success |
| Successful retained lift | 28.858-29.128 mm |
| Retained-lift P50 / P95 | 28.936 / 29.124 mm |
| Post-fix pre-sequence target-age P50 / P95 / max | 73 / 89 / 95 ms |
| Plan dispatch to execution-start P50 / P95 | 51 / 76 ms, 30 arm stages |
| FJT execution P50 / P95 | 3618 / 7179.1 ms, mixed arm stages |
| Gripper close execution P50 / P95 | 2532.5 / 2548.85 ms |
| Runtime logs with tracked DART mesh/geometry diagnostics | 0/11 |
| Exact cleanup with no remaining domain process and unknown `/clock` | 11/11 |

Every successful result required both configured distal-pad contacts, at least
20 mm lift, the matching lift-command terminal, and a 0.5 s retention window.
The observer digests and every artifact directory are listed in
`docs/observations/2026-08-29-candidate024-face-aligned-runtime.json`. This is
one target pose in an EdgeGrasp primitive-proxy model. It is not force
calibration, dynamics determinism, full collision fidelity, or hardware grasp
evidence. The harness deliberately opens the gripper after evidence capture;
the later final Gazebo pose is therefore post-release and is not used as the
retention endpoint.

The same investigation corrected a diagnostic-only failure path: an
unsuccessful `GraspSequenceTerminal` may legitimately have an empty lift command
ID. The physics core now accepts that shape only when
`sequence_completed=false` and reports `sequence_not_completed` instead of a
wrapper construction exception. A successful completion still requires the
exact non-empty lift command ID.

### Candidate024 distinct-target plan-only matrix

Recorded 2026-08-29 19:45-19:59 Asia/Shanghai in the isolated Ubuntu 24.04
Jazzy/Harmonic WSL workspace. The final command was:

```bash
cd /home/edgegrasp/ros2_ws/src/edgegrasp-sim
python3 scripts/run_candidate024_target_matrix_plan_only.py \
  --matrix ros_ws/src/edgegrasp_ros/config/candidate024_target_matrix.json \
  --artifact-dir /home/edgegrasp/ros2_ws/test_results/candidate024_target_matrix_pan_plan_only_20260829T2000 \
  --ros-domain-id-start 210
```

The first proposed positive generator was deliberately retained as a failed
hypothesis. It translated the cube and all three arm positions by the same XYZ
delta while keeping full orientations fixed. All ten proposed positives failed
`home_to_approach` with MoveIt's explicit `NO_IK_SOLUTION`, while an immediate
unchanged Candidate024 control passed all three segments. The error was not a
random planner or environment failure: the pinned URDF places the
`shoulder_pan` origin at `[0.0388353, -8.97657e-09, 0.0624]` m, and its joint
frame maps local +Z onto base-frame -Z. Rotating about the base origin with the
same sign therefore did not follow the robot's FK manifold.

The corrected matrix applies the exact pinned shoulder-pan origin and axis to
the cube and every arm position/orientation. It keeps target-to-gripper
geometry, planner, scaling, gripper command, collision proxies, and contact
policy fixed.

| Metric | Observed |
| --- | ---: |
| Matrix cases | 20 |
| Distinct reachable hypotheses | 10/10 plan-only PASS |
| Accepted positive arm segments | 30/30 |
| Rejection hypotheses | 10/10 matched |
| Scene-contract rejections | 4 |
| MoveIt distant-target rejections | 6 |
| False accepts / false rejects | 0 / 0 |
| Unverified cases | 0 |
| Motion-side-effect violations | 0 |
| Positive planning P50 / P95 | 0.975 / 2.370 ms |
| Positive service-response wall P50 / P95 | 8.480 / 18.507 ms |
| Unique positive trajectory digests | 30/30 segments |
| Tracked DART mesh / geometry diagnostics | 0 / 0 across 16 runtime cases |

Each runtime case used a fresh ROS domain and graph. The graph contained no
EdgeGrasp motion node, and the probe created no publisher or action client.
Trajectory publication, ExecuteTrajectory goals, FJT goals, and execution were
zero. Generated install-share files were removed after every case; the final
process check found no matching ROS/Gazebo process. This is distinct-target
scene/IK/MoveIt validation, not typed execution or physics-grasp evidence.

The authoritative records are
`docs/observations/2026-08-29-candidate024-target-matrix-plan-only.json` and the
retained failed-method record
`docs/observations/2026-08-29-candidate024-translation-matrix-plan-only.json`.

### Candidate024 representative distinct-target typed runtime

Recorded 2026-08-29 20:11:56-20:22:15 Asia/Shanghai in the same isolated
Ubuntu 24.04 Jazzy / Gazebo Sim 8.11 WSL workspace. Only three accepted matrix
cases were selected: near-control `reachable_pan_p005` and the two declared
range endpoints `reachable_pan_n040` and `reachable_pan_p040`. Each exact
candidate/scene/world triplet was checked against the plan-only artifact hash,
installed for one run, then removed by exact path.

Each run used the existing `run_candidate005_contact_quality.sh` harness with
ROS domains 226-228, the Candidate024 q0.40 geometry, the fixed
`candidate012_control_mu1p0` material profile, and the bounded -0.02 N m
effort-PID preload. No test case or alternate motion path was added.

| Metric | Observed |
| --- | ---: |
| Selected targets | 3 |
| Sequence completions | 3/3 |
| Independent physics observer passes | 3/3 |
| Correlated typed command terminals | 12/12 |
| FJT `error_code=0` | 12/12 |
| Missing command events | 0 |
| Retained cube lift | 28.965-29.095 mm |
| Bilateral configured-pad contact | 3/3 |
| Configured 0.5 s retention | 3/3 |
| Tracked DART mesh / geometry diagnostics | 0 / 0 |
| Post-run matching processes / `/clock` publishers | 0 / 0 |

Across the 12 stage commands, source-observation-to-stage-dispatch latency was
22.5 ms P50 and 83.1 ms P95. Stage-dispatch-to-correlated-terminal was
3106.0 ms P50 and 7226.85 ms P95. These percentiles use linear interpolation
at `(n-1)*p`; the latter arm interval includes MoveIt planning, typed-gate
admission, and FJT execution, so it is not pure controller latency or a stable
performance benchmark.

The sequence action intentionally reports `physics_unverified`; only the
separate read-only observer may promote the result after correlated terminal,
bilateral contact, at least 20 mm lift, and retention all pass. Its three
results returned `physics_grasp_verified=true` with reason
`contact_lift_retention_verified`. The MCAP analyzer independently keeps its
generic claim boundary false rather than converting derived statistics into an
observer result; that is not a contradiction.

This is scoped simulation-physics evidence for three selected poses, each run
once. It is not a per-pose repeatability result, a 3/3 estimate of workspace
success probability, full collision fidelity, physics determinism, or
real-hardware grasp evidence. Machine-readable inputs, command IDs, trajectory
digests, observer metrics, MCAP hashes, Gazebo log paths, and cleanup evidence
are in
`docs/observations/2026-08-29-candidate024-distinct-target-typed-runtime.json`.

### Candidate009-011 admission and physics runtime

Candidate009 introduced an isolated chained plan-only harness. The graph
contained Gazebo, the EdgeGrasp proxy MoveGroup overlay, and the confirmed
PlanningScene, but no adapter, trajectory gate, grasp sequence,
ExecuteTrajectory client, or FJT client. Returned RobotTrajectories were
validated and discarded. Every record therefore reports zero trajectory
publication, zero ExecuteTrajectory/FJT goals, and zero execution attempts.

- The selected -0.9 mm tool-X symmetry offset passed home-to-approach but
  failed approach-to-descend with `NO_IK_SOLUTION` in ROS domain 53.
- A fixed-parameter reachability scan then showed that the unchanged
  zero-offset control passed all three chained Pilz PTP segments 10/10, with
  one trajectory digest per segment. The first -0.1 mm step and sampled -0.2,
  -0.3, -0.5, -0.7, -0.8, and -0.9 mm offsets all failed the same descend IK
  gate on the first attempt.
- Every exact domain cleaned to zero matching processes and `/clock` was absent
  afterward. The correct decision was to stop the translation-only search and
  not execute Candidate009.

Candidate010 kept the zero-offset arm poses and changed only the generated
moving-pad profile plus matching close command to 0.45 rad. Domain 61 passed
all three plan-only segments 10/10 with stable point counts and one digest per
segment. The controlled domain-62 execution then completed the correlated
PlanTarget/ExecuteTrajectory/FJT sequence but returned the defined client code
11 because the independent physics gate failed:

- 144 simultaneous two-pad samples over 0.368 simulated seconds;
- 3.323 mm peak lift versus 20 mm required;
- bilateral contact ended 1.744 s before sequence completion;
- final XY drift about 14.12 mm; cube returned to table height; no retention.

Candidate011 repeated the same admission discipline at 0.40 rad. Domain 63
passed every plan-only segment 10/10 with zero execution. The domain-64 typed
run completed the sequence and supplied the strongest transient result in the
0.50/0.45/0.40 series, but it still failed the physics contract:

- 171 simultaneous two-pad samples over 0.463 simulated seconds;
- 3.711 mm peak lift, final XY drift about 13.56 mm;
- bilateral contact ended 1.737 s before completion;
- cube returned to table height; retention false;
- robot remained anchored, all controllers/actions were present, the scoped
  Gazebo log had zero tracked DART construction diagnostics, and exact-domain
  cleanup left no process or `/clock` publisher.

This evidence stops the close-angle search. Further closing would increasingly
tune proxy penetration while the success gate remains unmet. The next bounded
experiment is one pad friction/contact-material variable or one distal-pad
proxy-geometry revision, again admitted through the isolated plan-only gate.
Exact hashes, timings, action correlation, and cleanup records are in:

- `docs/observations/2026-08-27-candidate009-plan-only-runtime.json`;
- `docs/observations/2026-08-27-candidate009-reachability-scan.json`;
- `docs/observations/2026-08-27-candidate010-q0p45-plan-only.json`;
- `docs/observations/2026-08-27-candidate010-q0p45-runtime.json`;
- `docs/observations/2026-08-27-candidate011-q0p40-plan-only.json`;
- `docs/observations/2026-08-27-candidate011-q0p40-runtime.json`.

### Candidate012 paired finger-pad friction runtime

Candidate012 retained Candidate011's generated geometry, `0.40 rad` close
command, table/cube scene, target, Pilz PTP pipeline, 0.1 scaling, typed gate,
and observer success contract. A machine-readable experiment contract allowed
only generated finger-pad `mu=mu2` to differ between the paired rows. Static
URDF-to-SDF validation found exactly two mapped pad collisions, zero remaining
collision meshes, and the world anchor in each generated model.

Before motion, the implicit context, explicit `mu=1.0` control, and explicit
`mu=1.5` treatment each passed the isolated chained three-segment plan-only
harness 10/10. Those graphs published no trajectory and contained no
ExecuteTrajectory or FJT goal. Runtime used the same final launch snapshot for
the two paired rows:

```bash
bash scripts/run_candidate012_pad_friction.sh 81 \
  /home/edgegrasp/ros2_ws/test_results/candidate012_control_runtime_clean_20260828_1425 \
  candidate012-control-runtime-clean candidate012_control_mu1p0
bash scripts/run_candidate012_pad_friction.sh 80 \
  /home/edgegrasp/ros2_ws/test_results/candidate012_treatment_runtime_clean_20260828_1415 \
  candidate012-treatment-runtime-clean candidate012_treatment_mu1p5
```

| Metric | `mu=1.0` control | `mu=1.5` treatment |
| --- | ---: | ---: |
| Sequence terminal | COMPLETE | COMPLETE |
| Peak lift | 3.392 mm | 3.468 mm |
| Simultaneous two-pad samples | 160 | 172 |
| Simultaneous span | 0.437 s | 0.450 s |
| Contact ended before completion | 1.754 s | 1.763 s |
| Retention | false | false |
| `physics_grasp_verified` | false | false |

The treatment's +0.076 mm peak-lift and +13 ms contact-span deltas are small;
both cubes returned to table height. This bounded result does not support
friction as the current bottleneck and does not prove the DART solver applied
the coefficient exactly as configured.

The audit also preserves three non-evidence attempts. Two startup graphs were
rejected before motion because peer nodes observed `/clock` in different
startup callback order. The final launch starts the fail-closed clock consumer
before delaying the synthetic target publisher by 3 s; it does not relax the
future-target rule. One later treatment attempt failed the observer's 200 ms
baseline freshness gate while a separately launched diagnostic Gazebo process
was still consuming CPU. That exact process group was identified and stopped,
then both paired rows above ran from clean graphs. Every final exact-domain
cleanup left zero matching processes and no `/clock` topic.

Full values, log hashes, pre-motion failures, and claim boundaries are in
`docs/observations/2026-08-28-candidate012-pad-friction-runtime.json`.

### Candidate013 rejection and Candidate014 moving-pad runtime

Candidate013 changed only the target/tool-frame Y alignment by +5 mm and then
+2 mm. Both isolated plan-only graphs rejected approach-to-descend with
`NO_IK_SOLUTION`; neither graph contained adapter, trajectory-gate,
ExecuteTrajectory, FJT, or execution activity. The rejected profile/API was not
kept in the Windows project.

Candidate014 kept Candidate011's reachable arm poses, 0.40 rad close command,
scene, planner/scaling, and explicit `mu=mu2=1.0`, while extending only the
EdgeGrasp-owned moving distal-pad box by 12 mm along negative local Y. This is a
simulation attachment, not pinned-upstream or hardware-validated geometry.

The first apparent plan rejection was superseded after its logs proved a
harness race: the ACM service explicitly returned `success=False` before the
base PlanningScene was confirmed, and the old script ignored that response.
Both harnesses now wait for base-scene confirmation, require `SetBool
success=True`, and require a subsequent confirmed status with
`allow_target_pad_contacts=true`.

The authoritative plan-only command used `ROS_DOMAIN_ID=85` and artifact
directory
`/home/edgegrasp/ros2_ws/test_results/candidate014_moving_pad_plan_fixed_20260828_1710`.
All three chained Pilz segments passed in 3/3 attempts with stable per-segment
digests. Trajectory publication, ExecuteTrajectory goals, FJT goals, and
execution remained zero.

The one controlled typed runtime used `ROS_DOMAIN_ID=86` and artifact directory
`/home/edgegrasp/ros2_ws/test_results/candidate014_moving_pad_runtime_20260828_1710`.
It ran from `2026-08-28T17:04:22.241431667+08:00` to
`2026-08-28T17:06:17.022788781+08:00`. APPROACH, DESCEND, CLOSE_GRIPPER, and
LIFT completed with correlated command ID
`candidate014-moving-pad-runtime|lift|3`, but the physics observer correctly
kept `physics_grasp_verified=false`:

- peak lift: 3.680 mm versus the required 20 mm;
- simultaneous two-pad contact: 172 samples over 0.462 s;
- bilateral contact ended 1.744 s before sequence completion;
- final cube height returned to table height; retention false;
- three controllers remained active, four EdgeGrasp actions were present, the
  robot stayed at the world origin, and the scoped Gazebo log contained zero
  tracked DART construction diagnostics;
- exact-domain cleanup left zero matching processes and no `/clock` topic.

Compared with Candidate011, peak lift decreased by 0.031 mm and simultaneous
contact shortened by 1 ms. The extension therefore gives no meaningful evidence
that contact-patch length is the current bottleneck. Exact commands, hashes,
generated dimensions, and cleanup evidence are in
`docs/observations/2026-08-28-candidate014-moving-pad-runtime.json`.

### Latest 20:14-20:59 candidate006-008 contact-quality and timing runtime

Both clean successful-observation graphs kept the candidate005 routing,
selective pad-only ACM, 0.60 rad close target, proxy MoveGroup, typed actions,
and independent read-only physics observer. Candidate006 added Gazebo/DART
contact-depth/wrench diagnostics and exact gripper effort. Candidate007 added
source timestamps for each pad, same-sample bilateral contact, sequence
completion, and effort-at-completion.

Candidate006 used `ROS_DOMAIN_ID=48`, task
`candidate006-contact-quality-retry2-001`, and artifact directory
`/home/edgegrasp/ros2_ws/test_results/candidate006_contact_quality_retry2_20260827_2050`:

- Sequence completed through proxy MoveGroup, typed gate, arm/gripper FJT; client
  exit 11 correctly meant sequence success plus independent physics failure.
- Fixed/moving token counts were 920/170, with 36 same-sample messages. Their
  maximum depths were 8.55/3.43 micrometres and maximum absolute
  normal-projected solver forces were 54.42/16.01 N.
- Gripper effort peaked at 0.765 simulated units but was near zero at observation
  end. Peak cube lift was 0.333 mm versus 20 mm required; retention was false.
- These solver values are diagnostics, not calibrated force or force closure.

Candidate007 used a fresh `ROS_DOMAIN_ID=51`, task
`candidate007-contact-timing-retry2-001`, and artifact directory
`/home/edgegrasp/ros2_ws/test_results/candidate007_contact_timing_retry2_20260827_2140`.
The graph ran from `2026-08-27T20:31:15.827338837+08:00` through capture at
`20:33:23.194073204+08:00`; exact-domain cleanup ended at
`20:33:34.905515021+08:00`.

- Both tokens appeared in 39 same-sample messages from simulation time
  `37.504 s` through `37.603 s`, a 0.099 s interval.
- The correlated sequence terminal was at `39.350 s`. Bilateral contact had
  therefore ended 1.747 s earlier; the later moving-pad contact ended at
  `37.950 s`, still 1.400 s before completion.
- Simulated gripper effort was `-0.0001491425` at sequence completion after a
  peak absolute value of `0.6107977`.
- Cube baseline/peak/final Z were `0.20499999990200096`,
  `0.20535939004689518`, and `0.20499998446971884` m: only
  `0.000359390144894218` m peak lift, no retention, and
  `physics_grasp_verified=false`.
- Robot pose remained world zero, all three controllers stayed active, all four
  EdgeGrasp actions and both FJT actions were present, tracked DART/geometry
  diagnostics were zero, and exactly one `/clock` publisher (`ros_gz_bridge`)
  existed during capture.
- Exact-domain cleanup reported zero matching processes and then an unknown
  `/clock` topic. The trial, cleanup, and Gazebo-server hashes are fixed in the
  observation JSON.

The defensible interpretation is transient bilateral contact followed by loss
of bilateral loading before lift completion. Slip, unload, and geometry loss
remain possible mechanisms; the run does not distinguish them. The complete
records are
`docs/observations/2026-08-27-candidate006-contact-quality-runtime.json` and
`docs/observations/2026-08-27-candidate007-contact-timing-runtime.json`.

Candidate008 then changed exactly one coupled geometry/command variable: the
moving-pad OBB/AABB was reproducibly regenerated from the fixed-SHA kinematic
contract at 0.50 rad, and the runtime close command matched 0.50 rad. It used
fresh `ROS_DOMAIN_ID=52`, task `candidate008-q0p50-001`, and artifact directory
`/home/edgegrasp/ros2_ws/test_results/candidate008_q0p50_20260827_2105` from
`2026-08-27T20:56:50.285840973+08:00` through capture at
`20:58:57.710491697+08:00`; cleanup ended at
`20:59:08.856480762+08:00`.

- The generated profile preflight passed and reported the exact profile
  basename plus `gripper_contact_position_rad=0.5`.
- The correlated sequence completed; exit 11 again meant sequence success plus
  independent physics failure.
- Same-sample two-pad contact increased from candidate007's 39 samples/0.099 s
  to 98 samples/0.252 s. Peak simulated gripper effort increased from 0.611 to
  1.439, and peak lift increased from 0.359 mm to 2.667 mm.
- Bilateral contact still ended 1.788 s before sequence completion; the moving
  pad's later contact ended 1.522 s before completion. The cube returned to
  table height and retention remained false.
- The final six-decimal Gazebo CLI pose implies approximately 16.57 mm XY drift
  and 0.438 rad yaw. This supports a push/slip hypothesis but does not uniquely
  identify the mechanism.
- Robot pose remained world zero, three controllers and both FJT actions stayed
  present, tracked DART/geometry diagnostics were zero, and one `/clock`
  publisher existed during capture. Exact-domain cleanup left zero processes;
  `/clock` was then unknown.

This is a stronger negative/control comparison, not a simulated-grasp result.
The next bounded experiment should address pose/contact symmetry or friction,
not blindly decrease the close angle again. The complete record is
`docs/observations/2026-08-27-candidate008-q0p50-runtime.json`.

Four earlier setup graphs remain explicitly outside the motion evidence:
domains 46/47 exposed target-ID and target-position mismatches; domain 49's
typed preparation was rejected before downstream dispatch because safety was
false; domain 50 stopped before MoveIt/motion when a harness log-selector
pipeline propagated SIGPIPE. Each defect was corrected without weakening a
safety gate, and every exact-domain cleanup ended with zero matching processes.

### Earlier 19:18-19:31 candidate005 proxy-MoveGroup runtime

Two clean-graph attempts used Gazebo 8.11/DART, the pinned SO-101 SHA, and
scene digest
`8fb303a83b4d8c22ef6e491d5a33ea73b39371a17adc8511db47a5f1c89b6f35`.
Both launched `edgegrasp_proxy_move_group.launch.py`, not pinned upstream
MoveGroup directly.

- `ROS_DOMAIN_ID=44`, task `candidate005-simultaneous-proxy-001`: while the
  approach was active, ROS simulation time moved from `115406000000` ns back
  to `115405000000` ns. With exactly one `/clock` publisher
  (`ros_gz_bridge`), EdgeGrasp canceled, returned wrapper status 6, latched
  `SAFE_STOP`, sent no later stage, and reported zero grasp contacts. Three
  controllers remained active and robot/cube poses remained safe. No reset or
  retry occurred in that graph.
- Exact-PID cleanup removed the domain-44 graph and returned `/clock` publisher
  count to zero. One bounded retry used a fresh graph and
  `ROS_DOMAIN_ID=45`; rollback detection was not weakened.
- The retry confirmed motion permission, interface readiness, all three active
  controllers, both FJT endpoints, all four EdgeGrasp actions, one
  PlanningScene loader, retained table/cube, and the selective ACM. The current
  default periodic-revalidation implementation accepted the policy transition
  while a confirmed read-only query was pending, so the earlier timing fix now
  has live runtime evidence.
- MoveGroup's runtime `robot_description` contained both dedicated pad links.
  Three `ValidateSolution` instances loaded, and no MoveIt direct-execution
  action appeared. Gripper preparation went through typed ExecuteTrajectory
  and completed with wrapper status 4/FJT error 0.
- Task `candidate005-simultaneous-proxy-retry-001` completed APPROACH,
  DESCEND, CLOSE_GRIPPER, and LIFT with exact typed terminal correlation.
  Sequence wrapper status was 4, client exit 11 correctly represented sequence
  success plus independent physics failure, and the final trajectory digest
  was `88f16a929ba73754e9c84c274beddce09f94883d157afe2d1ec802e1b97911334`.
- The observer counted 1,125 fixed-pad-token contacts and 188 moving-pad-token
  contacts. Both configured pad tokens occurred in the same contact sample 35
  times. This is simultaneous-contact evidence, not force-closure evidence.
- Cube baseline/peak/final Z were `0.20499999990200096`,
  `0.20533242954819875`, and `0.204999979183065` m: only
  `0.0003324296461977849` m peak lift versus the required `0.02` m, with no
  retention. The defensible result is `sequence_completed=true` and
  `physics_grasp_verified=false`.
- The robot remained at world pose zero, all controllers remained active,
  tracked DART mesh-construction and geometry-creation counts were zero, and
  two contact topics existed. Exact-PID cleanup ended at
  `2026-08-27T19:31:53.182432665+08:00` with zero matching processes and zero
  `/clock` publishers.

The older candidate005 histogram run accidentally launched pinned upstream
MoveGroup. Its Gazebo contact counts remain valid physical observations, but
its proxy-planning claim is retracted. The current machine-readable record,
commands, hashes, failed attempt, retry, and claim boundaries are in
`docs/observations/2026-08-27-candidate005-simultaneous-proxy-runtime.json`.

### Latest 16:24 candidate003 collision and physics gate

Runtime graph: `2026-08-27T16:24:42.842897+08:00` through cleanup at
`2026-08-27T16:32:42.7748803+08:00`, with `ROS_DOMAIN_ID=230`, Gazebo 8.11/DART,
the pinned SO-101 SHA, and scene digest
`8fb303a83b4d8c22ef6e491d5a33ea73b39371a17adc8511db47a5f1c89b6f35`.

- Candidate002's negative tool-axis approach failed closed. Pilz generated 91
  points but `ValidateSolution` rejected indices 57–68 for target-cube versus
  `gripper_link`; OMPL independently returned `GOAL_STATE_INVALID`. Neither
  probe published or executed a trajectory.
- Candidate003's far approach passed collision-aware IK and Pilz plan-only with
  table and cube present (84 source points, 83 gate-ready points). Typed
  preparation opened the gripper. The approach then executed through
  `PlanTarget -> ExecuteTrajectory -> arm FJT`, with wrapper/downstream status
  4, FJT error 0, digest
  `96e1c6d49619c6b0a3d20a626f4dfc62d1af1230c3b298c027513ba55283147b`,
  and `moveit_direct_execution_used=false`. The cube pose was unchanged.
- The explicit scene transition removed only the optional MoveIt cube and
  confirmed the required table; Gazebo retained the cube. Read-only
  approach-to-descend and descend-to-lift checks passed with 51/50 and 22/21
  source/gate-ready points before motion.
- Task `candidate003-physics-042` completed approach, descend, separate gripper
  close, and lift. The sequence wrapper status was 4 and every corresponding
  controller log reported `Goal reached, success`.
- The observer now requires exact fixed/moving distal-pad proxy tokens. It saw
  zero qualifying contacts, only `0.00243373808118089 m` peak lift versus the
  required `0.02 m`, no retention, and `0.0522983480763947 m` lateral
  displacement. The final raw contact pair was cube-to-table. Physics wrapper
  status was 6 with `observation_wall_timeout`; grasp remained false.
- Gazebo logged zero tracked DART mesh-construction and geometry-creation
  diagnostics; the anchored robot stayed at world pose zero. Cleanup left no
  matching ROS/Gazebo process and `/clock` was unknown. MoveGroup still
  segfaulted on SIGINT, PlanningScene raised an rclpy conversion exception,
  Gazebo returned `-2`, and the trajectory gate repeated its `Destroyable`
  warning.

Exact commands, digests, poses, counts, logs and claim boundaries are in
`docs/observations/2026-08-27-candidate002-003-plan-only.json` and
`docs/observations/2026-08-27-candidate003-runtime.json`. The defensible result
is **collision-gated sequence success, zero distal-pad contact, physics grasp
false**.

### Latest 17:10 candidate004 selective-ACM plan-only gate

Runtime graph: `2026-08-27T17:10:39.540791+08:00` through exact-PID cleanup at
`2026-08-27T17:18:40.094517463+08:00`, with `ROS_DOMAIN_ID=231`, Gazebo
8.11/DART, the pinned SO-101 SHA, and the same scene digest.

- All three controllers became active, six joint states were observed, and
  both pinned FJT endpoints were present. MoveGroup exposed
  `/get_planning_scene` and `/plan_kinematic_path` with their expected types.
- MoveIt retained `edgegrasp_table` and `edgegrasp_target_cube`.
  `/edgegrasp/set_target_pad_contacts` returned success; the queried ACM
  allowed cube contact only for `edgegrasp_fixed_finger_pad_link` and
  `edgegrasp_moving_finger_pad_link`. `gripper_link` and
  `moving_jaw_so101_v1_link` were explicitly false.
- Pilz produced 51-point approach-to-descend candidates with gripper start
  states 1.5 and 0.60, but `ValidateSolution` rejected states 27-42 both times
  for `edgegrasp_target_cube` versus `gripper_link`. Both probes exited 2 and
  attempted no execution.
- The gripper=0.60 descend-to-lift control returned MoveIt code 1 and a valid
  22-point trajectory; it was also plan-only and attempted no execution.
- The persistent MoveGroup log contains two invalid-motion-plan responses and
  32 corresponding contact lines. Its SHA-256 is
  `f671671dffa8d80e554bf7232dcce0310cef9cf449815e18d4774042f84655a4`.
  The Gazebo server log contains zero tracked DART mesh-construction or
  geometry-creation diagnostics.
- All launch parents exited 0. The Gazebo server child outlived its launch
  parent and required an exact `SIGINT` to PID 508; afterward matching process
  count was zero, `/clock` publisher count was zero, and the ROS daemon was
  stopped. No broad kill was used.

The runtime also exposed a diagnostic-only inconsistency during periodic scene
revalidation: `ready=true` accompanied
`reason=awaiting_planning_scene_confirmation`. The implementation now reports
`confirmed_revalidation_pending` while preserving the already-confirmed ready
state; this wording fix is covered by the final package tests below, not by the
earlier runtime graph.

Exact commands, timings, plan counts, log paths, hashes, ACM permissions and
claim boundaries are in
`docs/observations/2026-08-27-candidate004-selective-acm-plan-only.json`.
The defensible result is **selective retained-cube policy verified, unsafe
descend rejected, lift plan-only accepted, candidate004 execution and physics
grasp not run**.

### Final live task006

Runtime graph start: `2026-08-27T07:48:02+08:00`; accepted trial client start:
`2026-08-27T07:51:30+08:00`; cleanup completed by `07:53:50+08:00`.

- Three controllers were active; six joint names and both FJT actions were
  observed. MoveGroup, PlanningScene readiness, PlanTarget, ExecuteTrajectory,
  GraspSequence, and GraspPhysicsEvidence were present. `/clock` had exactly one
  publisher.
- Typed preparation opened the gripper and moved the arm to
  `[-0.05, 0.8, -1.4, -1.4, 0.0]`; both wrapper statuses were 4, both downstream
  terminals were observed, and both FJT error codes were 0.
- The first physics request asked for 120 seconds, exceeded the configured
  30-second maximum, and was rejected before sequence or trial motion started.
- The accepted task006 completed APPROACH, DESCEND, CLOSE_GRIPPER, and LIFT.
  Its sequence wrapper status was 4, final command was
  `grasp-physics-trial-20260827-006|lift|3`, and every stage had action
  `STATUS_SUCCEEDED` plus FJT error code 0. The task005 premature lift cancel did
  not recur with the bounded 15-second adapter wall guard.
- No `clock_rollback` occurred. The observer collected 2781 cube poses, zero
  gripper contacts, and 11408 table contacts. Baseline, peak, and final cube Z
  were all `0.424999999902001 m`; physics grasp remained false. Because Gazebo
  ran below 0.5 real time, the 30 simulated-second window hit its 60-second wall
  guard and returned `observation_wall_timeout` fail-closed.
- SO-101 remained at world pose zero. The final Gazebo server log contains zero
  tracked DART mesh-construction/geometry diagnostics. After SIGINT cleanup,
  matching runtime process count and `/clock` publisher count were both zero.
  MoveGroup again exited `-11` during SIGINT teardown.

Exact full digests, poses, counts, log paths, and task004-task006 progression are
in
`docs/observations/2026-08-27-grasp-trial-final-runtime.json`. The defensible
result is **sequence success, physics grasp false**.

### Latest 15:14 Pilz collision and task039 contact gate

Runtime graph: `2026-08-27T15:14:39+08:00` to cleanup before
`2026-08-27T15:40:14+08:00`. The graph used `ROS_DOMAIN_ID=230`, the pinned
SO-101 SHA, the EdgeGrasp primitive-proxy Gazebo launch, the unique
`edgegrasp_proxy_move_group.launch.py`, and the shared scene digest
`8fb303a83b4d8c22ef6e491d5a33ea73b39371a17adc8511db47a5f1c89b6f35`.

- MoveGroup loaded Pilz `default_planning_response_adapters/ValidateSolution`.
  Direct MoveGroup execution stayed disabled through
  `allow_trajectory_execution=false` and disabled execute capabilities.
- The mid approach at
  `[0.2662075259898732, 0.1649137000605404, 0.2567987744381357]` generated an
  84-state candidate, but states 51–71 collided between
  `edgegrasp_target_cube` and `gripper_link`. `ValidateSolution` returned
  `INVALID_MOTION_PLAN`; the probe reported error `99999`,
  `plan_accepted=false`, and `execution_attempted=false`.
- The farther approach at
  `[0.27859814485128837, 0.1733906932169842, 0.29693801242225025]` passed the
  same validator. Task036 was canceled fail-closed on one transient
  `safety_signal_stale`; the downstream CANCELED terminal was observed and the
  cube did not move. Sim-clock measurements immediately afterward were about
  50 Hz permission, 20 Hz target, and 100 Hz joint states. A single bounded
  retry, task037, reached wrapper/FJT status 4 and FJT error 0 without cube
  displacement.
- The loader then confirmed table-only mode: expected `edgegrasp_table`,
  forbidden `edgegrasp_target_cube`, `ready=true`, `reason=confirmed`. This was
  an explicit target-contact policy; it did not remove the table. Read-only
  Pilz probes accepted the planned descend and lift paths.
- A 40-second physics observation request was rejected before motion because
  the configured maximum is 30 seconds. With a new task ID and the valid
  timeout, task039 completed approach, descend, gripper close, and lift. All
  four correlated downstream FJT terminals succeeded.
- The observer collected 2614 pose samples, 1860 gripper contacts, and 10834
  table contacts. The matched contact was
  `target_cube::target_cube_link::collision` versus
  `so101::gripper_link::gripper_link_collision`. Peak lift was
  `0.0022676879996189336 m`, below the required `0.02 m`; the cube returned to
  table height and ended about `0.059987 m` from its initial XY position.
  Retention and physics grasp were false.
- Three controllers remained active and the robot base stayed at world pose
  zero. The Gazebo log contained zero tracked DART construction diagnostics.
  Cleanup left no matching project process and no `/clock` publisher.
  MoveGroup still exited `-11` on SIGINT; Gazebo returned `-2`, and the
  interface probe emitted a teardown-time rclpy conversion exception.

Exact commands, full digests, poses, log paths, and the negative physics result
are in
`docs/observations/2026-08-27-pilz-collision-contact-runtime.json`. The current
defensible result is **collision rejection verified, sequence success, contact
observed, physics grasp false**.

## 1. Windows core/structural gates

### Recorded 18:20 baseline

Run window: `2026-08-26T18:20:52.7848022+08:00` to
`2026-08-26T18:21:11.1955041+08:00`.

Environment:

- Windows 11 Pro 10.0.26200, x64;
- CPython 3.10.11 from the project `.venv`;
- pytest 9.1.1, pluggy 1.6.0.

Exact command:

```powershell
powershell -NoProfile -NonInteractive -ExecutionPolicy Bypass \
  -File scripts\check.ps1 -ReplayRuns 100 \
  -PythonExecutable .\.venv\Scripts\python.exe
```

The script ran, in order:

1. `python -m compileall -q src tests ros_ws\src`;
2. `python -m pytest`;
3. `python scripts\validate_project.py`;
4. `python -m edgegrasp replay --runs 100`.

Observed result:

```text
collected: 124
passed:    124
failed:    0
skipped:   0
pytest wall time: 15.98 s
project structure: PASS
process exit: 0
```

The 124 tests cover the timestamped/frame-scoped `Target3D`, constant-velocity
prediction, strict `PlanResult`, required post-planning `execute_now_ns`, clock
rollback/epoch recovery, the 200 ms source and receive-watchdog boundaries,
confidence and frame gates, reset/stop/backend failures, the synchronous core
execution contract, endpoint-only workspace rejection, packaging, pinned
controller/action contracts, ROS source/config structure, upstream manifest,
MCAP scripts, and deterministic replay. ROS runtime tests are not hidden inside
this Windows count.

### Recorded 22:36 supplemental gate

After the typed late-result recovery barrier and dependency-free four-stage
sequence were added, the same CPython 3.10.11 project environment ran:

```powershell
.\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider
```

Run window: `2026-08-26T22:36:03.9485086+08:00` to
`2026-08-26T22:36:24.3928964+08:00`.

```text
collected: 171
passed:    171
failed:    0
skipped:   0
pytest wall time: 19.71 s
```

The 47 additional tests since the recorded baseline include typed
ExecuteTrajectory correlation/failure paths and 30 grasp-sequence tests. The
sequence suite verifies approach, descend, close-gripper and lift ordering;
exact task/target/command/clock correlation; every required gate/FJT success
field; four safety inputs; timeout/rollback/cancel uncertainty; explicit reset;
late terminal evidence; zero next-stage dispatch after failure; and identical
effect/history records across 100 runs. `COMPLETE` always reports
`physics_grasp_verified=false`.

### Recorded 23:07 sequence-hardening and typed-target gate

After the sequence event guard, epoch recovery, task-reuse barrier, malformed
input handling, serialized callbacks, and gripper completion-time checks were
hardened, the same CPython 3.10.11 environment ran:

```powershell
.\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider
ruff check src tests scripts ros_ws/src
.\.venv\Scripts\python.exe scripts\validate_project.py
```

Run window: `2026-08-26T23:06:43.0503813+08:00` to
`2026-08-26T23:07:04.1846436+08:00`.

```text
collected: 187
passed:    187
failed:    0
skipped:   0
pytest wall time: 20.38 s
ruff: all checks passed
project structure: PASS
```

The focused dependency-free sequence suite collected and passed 45 tests.
In addition to the earlier hardening, commands now bind to the most recent
fresh target source timestamp at every stage. The original task timestamp is
still matched at admission, source timestamps may not roll back, and the gate
result must echo the exact per-command source timestamp. This keeps the 200 ms
boundary intact for sequences that last longer than 200 ms.
The exact same sequence source and test files were checksum-synchronized to
the isolated Ubuntu 24.04/Jazzy workspace and passed 45/45 under Python 3.12.3.

### Recorded 2026-08-27 shared-scene gate

After adding the shared PlanningScene loader, adapter readiness gate,
service-echo verification, runtime observation schema, and periodic
revalidation false-pulse regression, the final one-key Windows gate ran:

```powershell
powershell -NoProfile -NonInteractive -ExecutionPolicy Bypass \
  -File scripts\check.ps1 -ReplayRuns 100 \
  -PythonExecutable .\.venv\Scripts\python.exe
```

Run window: `2026-08-27T01:52:28.9279831+08:00` to
`2026-08-27T01:52:54.0066919+08:00`.

```text
interpreter: CPython 3.10.11
collected:   190
passed:      190
failed:      0
skipped:     0
pytest time: 22.18 s
structure:   PASS
replay:      0/20/40 mm/s, 100 identical runs each
```

On the same source snapshot, `ruff check src tests scripts ros_ws/src` had
already returned `All checks passed!` at `01:48 +08:00`; subsequent changes
before the one-key gate were Markdown-only.

The loader tests parse the single scene contract, publish the required table
and optional cube as MoveIt boxes, query names plus geometry, reject
missing/drifted results, fail closed on unavailable service/clock rollback,
and assert that periodic confirmation never emits false after the first true.
The observation-schema test preserves zero-dispatch boundaries for B/C/D/E
and leaves optional cube, clearance, full proxy coverage, Gazebo sequence, and
physics grasp explicitly false.

### Recorded 02:45 correlated grasp-sequence gate

After adding the ROS `GraspSequence.action` wrapper and hardening result-slot
ownership, exact gripper digest matching, legal terminal-status evidence,
cancel-versus-success ordering, normalized ROS timestamps, and audit IDs, the
one-key Windows gate ran:

```powershell
powershell -NoProfile -NonInteractive -ExecutionPolicy Bypass \
  -File scripts\check.ps1 -ReplayRuns 100 \
  -PythonExecutable .\.venv\Scripts\python.exe
```

Run window: `2026-08-27T02:49:29.8451937+08:00` to
`2026-08-27T02:49:50.6553923+08:00`.

```text
interpreter: CPython 3.10.11
pytest:      9.1.1
collected:   206
passed:      206
failed:      0
skipped:     0
pytest time: 17.81 s
structure:   PASS
replay:      0/20/40 mm/s, 100 identical runs each
ruff:        all checks passed
```

The focused dependency-free sequence suite is 56/56. On the checksum-synced
Ubuntu 24.04/Jazzy workspace, the five EdgeGrasp packages rebuilt successfully
and the package-scoped test run used:

```bash
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash
cd /home/edgegrasp/ros2_ws
colcon test --packages-select edgegrasp_core edgegrasp_interfaces \
  edgegrasp_ros edgegrasp_moveit_adapter edgegrasp_grasp_sequence \
  --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/grasp_sequence_20260827_0246 \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/grasp_sequence_20260827_0246 \
  --all --verbose
```

Run window: `2026-08-27T02:46:05.144959882+08:00` to
`2026-08-27T02:46:14.714042396+08:00`, Python 3.12.3 and pytest 7.4.4.

| Package | Collected | Passed | Failed | Skipped |
| --- | ---: | ---: | ---: | ---: |
| `edgegrasp_core` | 1 | 1 | 0 | 0 |
| `edgegrasp_interfaces` | 0 | 0 | 0 | 0 |
| `edgegrasp_ros` | 32 | 32 | 0 | 0 |
| `edgegrasp_moveit_adapter` | 19 | 19 | 0 | 0 |
| `edgegrasp_grasp_sequence` | 14 | 14 | 0 | 0 |
| **Total** | **66** | **66** | **0** | **0** |

The 14 sequence tests instantiate ROS action clients/servers and prove the
four-stage typed protocol, failure gating, exact IDs and digest, retained
stop-uncertain slots, reset refusal, target drift cancellation, downstream
unavailability, concurrent-goal rejection, and a cancel/late-success race with
zero next-stage dispatch. They use fake PlanTarget/ExecuteTrajectory servers;
they do not prove a four-stage Gazebo run or object grasp physics.

### Recorded 03:58 core and static gate

After the immutable sequence-orientation contract, real runtime observation,
planning-scene shutdown guard, and ROS action-test teardown fix were present in
the same Windows tree, the one-key project gate ran from
`2026-08-27T03:58:07.8239928+08:00` to
`2026-08-27T03:58:29.2337297+08:00`:

```powershell
.\scripts\check.ps1 -ReplayRuns 100 \
  -PythonExecutable .\.venv\Scripts\python.exe
ruff check src tests scripts ros_ws\src
```

```text
interpreter: CPython 3.10.11
pytest:      9.1.1
collected:   206
passed:      206
failed:      0
skipped:     0
pytest time: 18.93 s
structure:   PASS
replay:      0/20/40 mm/s, 100 identical runs each
ruff:        all checks passed
JSON parse:  observation and upstream manifest PASS
```

The Windows suite intentionally collects only `tests/`; ROS-runtime tests are
reported separately from their Jazzy result XML. The earlier failed attempts
to invoke this gate from the wrong working directory and to run Ruff from a
venv without Ruff did not execute the relevant checks and are not counted as
test failures or passes.

### Current 05:37 final Windows gate

After the repeat harness switched to direct installed entry points, rclpy
shutdown handling was guarded, and the repeatability evidence was added, the
same one-key gate ran against one stable Windows tree:

```powershell
powershell -NoProfile -NonInteractive -ExecutionPolicy Bypass \
  -File scripts\check.ps1 -ReplayRuns 100 \
  -PythonExecutable .\.venv\Scripts\python.exe
```

Run window: `2026-08-27T05:37:25.0187758+08:00` to
`2026-08-27T05:37:48.1975325+08:00`.

```text
interpreter: CPython 3.10.11
pytest:      9.1.1
collected:   210
passed:      210
failed:      0
skipped:     0
pytest time: 20.29 s
compileall:  PASS
structure:   PASS
replay:      0/20/40 mm/s, 100 identical runs each
process exit: 0
```

The deterministic replay digests remained unchanged from the table below.
This is the current Windows fact used by the README and simulation plan; the
older 206-test and lower-count sections remain only as timestamped history.

### Deterministic replay

Each scenario has 20 samples and was replayed 100 times:

| Scenario | Identical runs | Accepted/run | Rejected | Digest |
| --- | ---: | ---: | ---: | --- |
| 0 mm/s | 100 | 20/20 | 0 | `0a6ba6f7025798cda009bf979ba303a36677cab9940900cfe1acc307944735d7` |
| 20 mm/s | 100 | 20/20 | 0 | `854cf2da9c384fe0d115a4b241a6f05c72eec21713fe8194e9cafec8ae846aac` |
| 40 mm/s | 100 | 20/20 | 0 | `da7fcd929295d9651327b7319b151de5bdef41537a7cc68892a269728b7aa3e0` |

This is core replay determinism, not ROS scheduling, Gazebo dynamics,
controller, contact, or grasp determinism.

### Synthetic predictor benchmark

Run window: `2026-08-26T18:23:33.8657831+08:00` to
`2026-08-26T18:23:34.6472739+08:00`, CPython 3.10.11 on this host.

```powershell
.\.venv\Scripts\python.exe scripts\benchmark_replay.py \
  --runs 100 --horizon-ms 100
```

| Speed | Max error after warm-up | Compute p50 | Compute p95 | Replay wall time |
| ---: | ---: | ---: | ---: | ---: |
| 0 mm/s | 0 mm | 7.5 us | 11.1 us | 150.8178 ms |
| 20 mm/s | 5.5511e-14 mm | 6.6 us | 7.1 us | 147.5190 ms |
| 40 mm/s | 5.5511e-14 mm | 6.5 us | 7.0 us | 148.2025 ms |

The moving residual is floating-point roundoff in noiseless data generated by
the same linear model. These compute timings are a timestamped local snapshot,
not stable performance or sensor-to-actuator latency.

## 2. Ubuntu 24.04 / Jazzy build and tests

Guest environment:

- Ubuntu 24.04.4 LTS Noble under WSL2;
- Python 3.12.3, pytest 7.4.4;
- ROS 2 Jazzy and Gazebo Harmonic (`gz sim` 8.11.0);
- dedicated workspace `/home/edgegrasp/ros2_ws`;
- 12 packages discovered: five EdgeGrasp packages and seven pinned
  `so101_ros2` packages.

The complete project layout was copied to
`src/edgegrasp-sim`; only copying `edgegrasp_core` is intentionally unsupported.
The pinned upstream copy came from the clean detached Windows checkout at
`0305e03ab54e64aae9263fcbf339622e654012f3`.

### Recorded whole-workspace build

Latest build log started at `2026-08-26T18:19:02+08:00`.

```bash
source /opt/ros/jazzy/setup.bash
cd /home/edgegrasp/ros2_ws
colcon build --symlink-install --event-handlers console_direct+
```

Observed: **11 packages finished, zero build failures**. Direct xacro output to
`/tmp/edgegrasp_so101_final.urdf` and `check_urdf` also passed with the camera
link tree present. The hard-coded upstream state-publisher launch was not used
for this parse check.

### EdgeGrasp package tests

Result files were isolated at
`/home/edgegrasp/ros2_ws/test_results/final_edgegrasp_20260826_1822`.
They were written between `18:21:54` and `18:21:58 +08:00`.

```bash
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash
cd /home/edgegrasp/ros2_ws
colcon test --packages-select edgegrasp_core edgegrasp_interfaces \
  edgegrasp_ros edgegrasp_moveit_adapter \
  --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/final_edgegrasp_20260826_1822 \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/final_edgegrasp_20260826_1822 \
  --all --verbose
```

| Package | Collected | Passed | Failed | Skipped |
| --- | ---: | ---: | ---: | ---: |
| `edgegrasp_core` | 1 | 1 | 0 | 0 |
| `edgegrasp_interfaces` | 0 | 0 | 0 | 0 |
| `edgegrasp_ros` | 11 | 11 | 0 | 0 |
| `edgegrasp_moveit_adapter` | 13 | 13 | 0 | 0 |
| **Total** | **25** | **25** | **0** | **0** |

The ROS tests genuinely instantiate nodes and fake action/service servers; this
section is injected/fake contract evidence and not real MoveGroup health.
They cover invalid/out-of-order targets, rollback/reset, downstream readiness,
send/result failures, cancel accept/reject/exception/timeout, accepted-cancel
without terminal result, bounded three-attempt escalation, exact arm/gripper
FJT forwarding, and a simulated-clock target dropout. The fake-clock test
allows exactly 200 ms and observes cancellation by the next 50 Hz tick at
220 ms; executor scheduling adds ordinary runtime jitter.

### Recorded 22:36 supplemental package tests

The current source snapshot was rebuilt and retested into the isolated result
base `/home/edgegrasp/ros2_ws/test_results/final_edgegrasp_20260826_2236`.
Result files were written from `22:36:37` through `22:36:43 +08:00`:

| Package | Collected | Passed | Failed | Skipped |
| --- | ---: | ---: | ---: | ---: |
| `edgegrasp_core` | 1 | 1 | 0 | 0 |
| `edgegrasp_interfaces` | 0 | 0 | 0 | 0 |
| `edgegrasp_ros` | 27 | 27 | 0 | 0 |
| `edgegrasp_moveit_adapter` | 16 | 16 | 0 | 0 |
| **Total** | **44** | **44** | **0** | **0** |

The gate additions specifically cover same-dispatch FJT results arriving after
a stop-unconfirmed tombstone. A late SUCCEEDED, CANCELED, or ABORTED result may
confirm downstream termination, but it cannot rewrite the already-returned
wrapper result, release controller ownership, or resume motion. Only an
explicit reset clears the confirmed recovery barrier. `get_result_async()` and
result-future exceptions now complete the typed failure immediately while
retaining stop-unconfirmed state.

### Recorded 23:07 supplemental package tests

After adding the per-command source-timestamp echo and atomic
`TrackedTarget.msg`, the four EdgeGrasp packages were rebuilt and retested into
`/home/edgegrasp/ros2_ws/test_results/final_edgegrasp_20260826_2307`.
Result files were written from `23:07:15` through `23:07:19 +08:00`:

| Package | Collected | Passed | Failed | Skipped |
| --- | ---: | ---: | ---: | ---: |
| `edgegrasp_core` | 1 | 1 | 0 | 0 |
| `edgegrasp_interfaces` | 0 | 0 | 0 | 0 |
| `edgegrasp_ros` | 28 | 28 | 0 | 0 |
| `edgegrasp_moveit_adapter` | 17 | 17 | 0 | 0 |
| **Total** | **46** | **46** | **0** | **0** |

This run regenerated the ROS message/action bindings, verified that the gate
returns the request's exact source timestamp, verified that the MoveIt adapter
fails closed on a mismatched timestamp, and observed matching mock
`PointStamped`/`TrackedTarget` points and stamps with exact target ID/domain/
epoch. It did not start Gazebo, controllers, MoveIt, or hardware.

### Recorded 2026-08-27 shared-scene package gate

The complete project layout was checksum-resynchronized with the full exclude
set, then the four EdgeGrasp packages were rebuilt and tested in an isolated
result base:

```bash
colcon build --packages-select \
  edgegrasp_core edgegrasp_interfaces edgegrasp_ros \
  edgegrasp_moveit_adapter --symlink-install
colcon test --packages-select \
  edgegrasp_core edgegrasp_interfaces edgegrasp_ros \
  edgegrasp_moveit_adapter \
  --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/planning_scene_20260827_final2 \
  --return-code-on-test-failure
colcon test-result --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/planning_scene_20260827_final2 \
  --all --verbose
```

Run window: `2026-08-27T01:49:20.598021907+08:00` to
`2026-08-27T01:49:40.812743111+08:00`, Ubuntu Python 3.12.3.

| Package | Collected | Passed | Failed | Skipped |
| --- | ---: | ---: | ---: | ---: |
| `edgegrasp_core` | 1 | 1 | 0 | 0 |
| `edgegrasp_interfaces` | 0 | 0 | 0 | 0 |
| `edgegrasp_ros` | 32 | 32 | 0 | 0 |
| `edgegrasp_moveit_adapter` | 19 | 19 | 0 | 0 |
| **Total** | **52** | **52** | **0** | **0** |

All four selected packages built successfully. This is package-scoped
EdgeGrasp evidence; it does not reuse contaminated whole-workspace historical
lint results from the pinned upstream tree.

### Current 03:57 five-package gate

The five EdgeGrasp packages rebuilt successfully in the checksum-synchronized
Ubuntu workspace; `colcon` reported five packages finished in `7.00 s`. The
package-scoped tests then ran from
`2026-08-27T03:57:18.375488132+08:00` to
`2026-08-27T03:57:27.646995593+08:00` into the fresh result base
`/home/edgegrasp/ros2_ws/test_results/final_20260827_0358`:

```bash
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash
cd /home/edgegrasp/ros2_ws
colcon test --packages-select \
  edgegrasp_core edgegrasp_interfaces edgegrasp_ros \
  edgegrasp_moveit_adapter edgegrasp_grasp_sequence \
  --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/final_20260827_0358 \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/final_20260827_0358 \
  --all --verbose
```

| Package | Collected | Passed | Failed | Skipped |
| --- | ---: | ---: | ---: | ---: |
| `edgegrasp_core` | 1 | 1 | 0 | 0 |
| `edgegrasp_interfaces` | 0 | 0 | 0 | 0 |
| `edgegrasp_ros` | 32 | 32 | 0 | 0 |
| `edgegrasp_moveit_adapter` | 19 | 19 | 0 | 0 |
| `edgegrasp_grasp_sequence` | 22 | 22 | 0 | 0 |
| **Total** | **74** | **74** | **0** | **0** |

The 17 sequence integration cases genuinely instantiate the GraspSequence,
PlanTarget and ExecuteTrajectory action clients/servers. The test harness now
waits until the Jazzy executor has no pending action coroutine before node
destruction; the final 22-test package run emitted no prior unfetched
`InvalidHandle / Destroyable` teardown warning. These are fake-server runtime
tests, separate from the real Gazebo sequence observation below.

### Current 05:35 synchronized five-package gate

After the final Windows tree was synchronized with the complete exclude set
(`.git`, both local virtual environments, `build/install/log`, `__pycache__`,
and `*.pyc`), all five EdgeGrasp packages rebuilt in `7.51 s`. The exact
package-scoped command then wrote a fresh result base:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/edgegrasp/ros2_ws
colcon build --packages-select \
  edgegrasp_core edgegrasp_interfaces edgegrasp_ros \
  edgegrasp_moveit_adapter edgegrasp_grasp_sequence \
  --symlink-install --event-handlers console_direct+
source /home/edgegrasp/ros2_ws/install/setup.bash
colcon test --packages-select \
  edgegrasp_core edgegrasp_interfaces edgegrasp_ros \
  edgegrasp_moveit_adapter edgegrasp_grasp_sequence \
  --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/final_20260827_0536 \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --test-result-base \
  /home/edgegrasp/ros2_ws/test_results/final_20260827_0536 \
  --all --verbose
```

Result XML files were written from
`2026-08-27T05:35:14.858297295+08:00` through
`2026-08-27T05:35:22.454683855+08:00`; colcon reported `9.63 s` total test
time.

| Package | Collected | Passed | Failed | Errors | Skipped |
| --- | ---: | ---: | ---: | ---: | ---: |
| `edgegrasp_core` | 1 | 1 | 0 | 0 | 0 |
| `edgegrasp_interfaces` | 0 | 0 | 0 | 0 | 0 |
| `edgegrasp_ros` | 32 | 32 | 0 | 0 | 0 |
| `edgegrasp_moveit_adapter` | 19 | 19 | 0 | 0 | 0 |
| `edgegrasp_grasp_sequence` | 22 | 22 | 0 | 0 | 0 |
| **Total** | **74** | **74** | **0** | **0** | **0** |

The console completed without the earlier unfetched Destroyable/context
teardown warning. This result base contains only the selected EdgeGrasp
packages and is not contaminated by historical pinned-upstream lint XML.

### Pinned-upstream test distinction

A separate whole-workspace test used an isolated result base. EdgeGrasp still
passed 25/25. The aggregate was 48 tests, 9 failures, and 1 skip because the
pinned upstream checkout has lint failures in five packages:

- `so101_bringup`: one flake8 line-length failure;
- `so101_description`: one CMake whitespace lint failure;
- `so101_gazebo`: flake8/pep257 docstring plus CMake whitespace failures;
- `so101_moveit_config`: unused launch variables/docstring plus CMake lint;
- `so101_teleop_bridge`: import-order lint; copyright test skipped.

`so101_ros2` and the TODO-shell `so101_system_tests` lint suites passed. These
upstream lint results do not negate the successful 11-package build or scoped
runtime, and the clean third-party checkout was not modified to hide them.

## 3. Scoped SO-101 runtime observations

ROS log timestamps place these runs approximately between 16:29 and 17:46
`+08:00`. Exact reproduction commands are in the Ubuntu runbook.

### Gazebo controllers and safety gate

Using `empty.world`, camera/RViz disabled, and simulated time:

- `joint_state_broadcaster`, `arm_controller`, and `gripper_controller` were
  active;
- `/joint_states` contained the five arm joints plus `gripper`;
- exact actions existed as
  `/arm_controller/follow_joint_trajectory` and
  `/gripper_controller/follow_joint_trajectory`, both
  `control_msgs/action/FollowJointTrajectory`;
- conservative direct diagnostic goals completed, but are labeled safety
  bypasses;
- conservative requests through both `/edgegrasp/*_trajectory_request` paths
  were accepted and returned status 4;
- live target dropout caused the gate to request arm cancellation; the fake
  controller/live simulator cancellation path returned status 5 in the
  observed run.

A cancel request or status code is not proof that physical hardware stopped.
In dependency-injected fake-action tests, the gate state machine treats
rejected/exception/timeout cancels and an accepted cancel without terminal
result as faults, retries no more than three times, then publishes explicit
controller-stop escalation status.

### MoveIt planning and target adapter

The fixed-SHA tree contains a complete-looking static MoveIt configuration.
Runtime confirmed move_group loading KDL plus OMPL/Pilz/STOMP and the two FJT
controller mappings. `/plan_kinematic_path` existed with
`moveit_msgs/srv/GetMotionPlan`.

A manual five-joint plan-only request returned `error_code.val == 1`, planning
time `0.016825348 s`, and a nonempty six-point trajectory in exact arm order;
final `time_from_start` was `0.466441682 s` and fixed-base multi-DOF points were
empty. This is planning evidence, not execution.

The EdgeGrasp adapter implements the separate path:

```text
PlanTarget immutable pose
  -> timestamped tf2 to base_link
  -> bounded /compute_ik with fresh /joint_states seed
  -> moveit_msgs/action/MoveGroup, plan_only=true
  -> independent RobotTrajectory validation
  -> /edgegrasp/execute_trajectory (typed arm command)
  -> trajectory_gate
  -> /arm_controller/follow_joint_trajectory
```

A 2 mm Cartesian offset was correctly rejected with
`NO_IK_SOLUTION (-31)` and zero trajectory publication. An exact current pose
passed. A reachable pose derived from the joint target
`[0.05, -0.05, 0.05, -0.05, 0]` also returned MoveIt success, published exactly
once through the gate, and completed at controller status 4. Observed joint
feedback was approximately pan `0.0490`, lift `-0.0509`, elbow `0.0521`,
wrist-flex `-0.0492`, wrist-roll `0.0228`; the redundant wrist result is not an
endpoint-accuracy benchmark.

The adapter never invokes MoveIt execution, never publishes a combined
`arm_with_gripper` path, never maps a ROS trajectory into the core Cartesian
`MotionPlan`, and keeps gripper sequencing separate.

### Shared PlanningScene table and A-E cases

Run window: `2026-08-27T01:23:51+08:00` to
`2026-08-27T01:41:00+08:00`. A single Gazebo 8.11/DART graph supplied
`/clock`, the pinned move_group supplied `/get_planning_scene`, and the
EdgeGrasp loader used the same `scene.json` whose generated SDF launched the
table/cube world.

The service echoed `edgegrasp_table` in `base_link` as a box with dimensions
`[0.6, 0.8, 0.04] m` at `[0.35, 0, 0.38] m`. The loader digest was
`adbc39d4a582744159a71c18abf05cdcc1ce673fc3e238577a211d6f09e00665`.
Ten observed periodic readiness samples after confirmation were all true.

The first A run found a real false-pulse bug during periodic service requery.
MoveIt planned and the downstream controller reported goal success, but the
adapter saw readiness false, requested cancel, and the gate correctly latched
stop-unconfirmed rather than reporting success. After the loader preserved a
prior confirmation during in-flight revalidation, a clean restart produced:

| Case | MoveIt / safety result | Trajectory dispatch | Controller result |
| --- | --- | ---: | --- |
| A current reachable pose | `SUCCESS (1)` | 1 | wrapper/FJT status 4, FJT error 0 |
| B inside table | collision-aware IK `-31` | 0 | not sent |
| C FK-reachable table-crossing/detour case | `ValidateSolution: INVALID_MOTION_PLAN`, `99999` | 0 | not sent |
| D far unreachable | IK `-31` | 0 | not sent |
| E wrong epoch | `clock_epoch_mismatch` | 0 | not sent |
| E planning beyond freshness | `stale_target` | 0 | not sent |

For C, `/plan_kinematic_path` returned a nonempty candidate with planning time
`0.080471859 s` but error code `99999`. EdgeGrasp required SUCCESS as well as
valid structure and therefore published nothing. No direct MoveIt execution
was used. The exact JSON results and log paths are in
`docs/observations/2026-08-27-planning-scene-runtime.json`.

This upgrades only the required table and these scoped cases. The optional
cube was not loaded into MoveIt; minimum clearance, every proxy link, ROS grasp
orchestration, cube lift/retention, and physical grasp remain unverified.

### Correlated four-stage sequence runtime

Run window: `2026-08-27T03:26:59+08:00` to
`2026-08-27T03:40:24+08:00`. The isolated graph contained the anchored proxy
Gazebo model, all three active controllers, pinned MoveGroup, the table-only
PlanningScene, EdgeGrasp adapter/gate, and GraspSequence wrapper. Gazebo was
the sole `/clock` authority.

The action contract was corrected to carry one finite, normalized
`arm_orientation` quaternion in the immutable task snapshot. Approach,
descend, and lift copy exactly that quaternion; the client default is the zero
quaternion and therefore fails closed until the caller supplies a valid
orientation.

After a typed, gated pre-position completed, task
`grasp-sequence-runtime-0331` produced the exact ordered terminal chain:

```text
APPROACH_PLAN -> APPROACH_EXEC
DESCEND_PLAN  -> DESCEND_EXEC
CLOSE_GRIPPER_EXEC
LIFT_PLAN     -> LIFT_EXEC
COMPLETE
```

The controller log contained exactly three successful arm FJT goals and one
successful gripper FJT goal. The wrapper returned status `4`,
`sequence_completed=true`, terminal phase `COMPLETE`, and last correlated
command `grasp-sequence-runtime-0331|lift|3`. Every arm trajectory came from
MoveIt plan-only and then traversed the typed EdgeGrasp gate; MoveIt execution
was not used directly.

This was deliberately a below-table wiring/ordering smoke run. It did not
contact the cube, the optional cube was not loaded into MoveIt, and no cube
pose/contact/retention observer was present. The result correctly remained
`physics_grasp_verified=false`.

A fresh-wrapper follow-up task, `grasp-sequence-runtime-0333`, completed
approach and descend, accepted the gripper goal, then detected
`joint_state_stale`. It requested gripper cancellation, entered `SAFE_STOP`,
and dispatched no lift. This is retained fail-closed evidence from the first
runtime phase; the corrected repeatability result is recorded separately
below. Full machine-readable IDs, digests, poses, log paths and claim
boundaries are in
`docs/observations/2026-08-27-grasp-sequence-runtime.json`.

### Corrected ten-run sequence repeatability

The first strict repeat batch at 04:39:59-04:43:39 completed 4/10 and failed
closed six times: three `target_source_timestamp_future` faults and three
`trajectory_gate_cancel_rejected` faults. A subsequent batch is deliberately
classified **INVALID_MEASUREMENT** because its harness backgrounded
`ros2 run`, tracked the CLI wrapper PID, and could report a clean shutdown
while child action servers remained alive. Its apparent 6/7 completion is not
an acceptance result.

The harness now resolves the installed package prefix, starts the actual
`grasp_sequence` and `grasp_sequence_client` entry points, gives every run a
unique action endpoint, owns the action-server PID, and marks traceback or a
lingering node as shutdown failure. The first direct-entrypoint attempt passed
1/2, then exposed `task_source_stale` plus an rclpy invalid-context wait-set
exception. After the guarded shutdown fix, a 2/2 smoke passed.

The accepted current-source batch ran from artifact time
`2026-08-27T05:20:07.547091973+08:00` through
`2026-08-27T05:22:58.702799320+08:00`:

```bash
bash scripts/run_grasp_sequence_repetitions.sh 10 \
  /home/edgegrasp/ros2_ws/test_results/grasp_repeat_direct_final_20260827_0520 \
  grasp-direct-accept \
  '[0.391231968, -0.001571668, 0.256520737]' \
  '[0.391231968, -0.001571668, 0.251520737]' \
  '[0.391231968, -0.001571668, 0.261520737]' \
  '[0.017007859, 0.706463960, 0.013976791, 0.707406570]' \
  0.2
```

```text
runs:                       10
client exit 0:              10
sequence_completed=true:    10
per-run sequence shutdown:  10 clean
wrapper status:              10 x 4
physics_grasp_verified:      0
distinct final digests:      10
```

The summary is
`/home/edgegrasp/ros2_ws/test_results/grasp_repeat_direct_final_20260827_0520/summary.tsv`,
SHA-256
`8658ff43a9974edec6f6c69934f37052e530ebff6fb1891a32c899268d38c4da`.
This is a 10/10 typed protocol-outcome repeatability result. The ten different
trajectory digests explicitly prevent a byte-identical planner-output claim;
there was no cube contact/lift/retention evidence and no physics-grasp success.
All retained batches and claim flags are in
`docs/observations/2026-08-27-grasp-sequence-repeatability.json`.

### Camera/world boundary

The pinned pick-and-place world published color, depth, camera-info, and point
cloud topics. Under WSL load, image topics were roughly 4 Hz and camera info
reported 424x240. First-use Gazebo Fuel resources downloaded successfully.

The original DART run emitted mesh-construction diagnostics. A later generated
URDF replaced exactly 13 robot collision meshes with conservative primitive
boxes while preserving visuals and anchored `world -> base_link`. In the final
proxy run, those earlier robot-mesh diagnostics were absent; table-cube contact
and one specifically named base-proxy contact were observed. This is narrow
behavioral evidence for one proxy and the table/cube pair—not full 13-proxy
fidelity, optional-cube planning, cube retention, or grasp success. Required
table agreement with MoveIt is covered separately above.
Exact commands and log boundaries are in
`docs/observations/2026-08-26-dart-proxy-final.md`.

### MCAP single-clock evidence

The ros_sim recording profile waited for `/clock` and used
`ros2 bag record -s mcap --use-sim-time`. The evidence bag is
`/home/edgegrasp/edgegrasp_mcap_evidence_20260826_1740`:

```text
storage:  mcap
size:     186.3 KiB
duration: 1.387 s
messages: 1759
/clock: 1389; /joint_states: 138; /edgegrasp/target_3d: 27
motion_allowed: 68; safety_status: 69
interface_ready: 34; interface_status: 34
```

Camera topics were absent because this particular bag came from the
controller-only run. Command/gate topics were registered but had zero samples.
The time-domain validator found 27/27 targets with receive-minus-header deltas
from -1 ms to +1 ms and zero violations at its 1000 ms guard.

After all Gazebo/MoveIt/controller processes were stopped and `/clock` was
unknown, `edgegrasp_replay.launch.py` plus the raw-input allowlist playback ran
to exit 0. Initial `no_fresh_target`, a brief `future_target_pending` caused by
callback ordering, and subsequent `allowed` were observed. A second rewind
without wrapper reset latched `clock_rollback`, as designed. The clock
preflight refused playback when a synthetic external `/clock` publisher was
present.

Derived/status/command topics are recorded intentionally for comparison but
never appear in default playback. This verifies `SAFETY_REPLAY` wrapper/gate
behavior only; there was no planner, backend, controller, or physics replay,
and repeat-run MCAP determinism is still unmeasured.

## 4. Static, audit, and source-pin checks

Current static check window: `2026-08-28T13:24:33.1027318+08:00` to
`2026-08-28T13:24:33.3889212+08:00`.

```powershell
ruff check .
& 'C:\Program Files\Git\bin\bash.exe' -n <each scripts\*.sh>
# Parse each scripts\*.ps1 with the PowerShell language parser.
# Parse each JSON file below docs/, src/, and ros_ws/ with ConvertFrom-Json.
```

Observed: Ruff PASS; project structure PASS; Git Bash syntax PASS for 11 shell
scripts; PowerShell parser PASS for four scripts; JSON parser PASS for 38
current contract/manifest/config/observation files. These checks are static
only: Git Bash syntax is not Ubuntu execution, and JSON syntax is not runtime
truth.

The final Windows `compileall` regenerated 82 ignored `.pyc` files in 14
`__pycache__` directories strictly below the project `ros_ws` tree. Every
absolute path was enumerated and `OUTSIDE_ROOT_COUNT=0`. The host command
policy rejected both the batch removal and a single explicit-directory
`Remove-Item` before execution; all 82 files remain generated caches, not
source or runtime evidence. No broader directory or upstream checkout was
touched.

Windows static check window:
`2026-08-26T18:21:28.9264939+08:00` to
`2026-08-26T18:21:29.9440356+08:00`.

```powershell
& 'C:\Program Files\Git\bin\bash.exe' -n <all four scripts\*.sh>
# Parse every scripts\*.ps1 with the PowerShell language parser.
ruff check src tests scripts ros_ws\src
.\.venv\Scripts\python.exe -m json.tool docs\upstream-manifest.json
.\.venv\Scripts\python.exe -m json.tool \
  src\edgegrasp\contracts\so101_ros2_0305e03.json
.\.venv\Scripts\python.exe scripts\validate_so101_mujoco.py --static-only
```

Results: Git Bash syntax PASS for four files; PowerShell parser PASS; Ruff
0.15.16 PASS; both JSON parsers PASS; original pinned SO-ARM100 MJCF static
check PASS with six expected joints/actuators, 13 meshes, and documented base
collision/gripper limitations. Git Bash syntax is not Ubuntu script execution.

The Linux audit script did run in Ubuntu and found Python, CMake, colcon,
`ros2`, `gz`, and RViz2. The PowerShell audit at
`2026-08-26T18:03:41.8340358+08:00` safely selected `Ubuntu-24.04` and passed
both host metadata and guest OS/kernel probes. See
[environment-audit.md](environment-audit.md).

In an earlier snapshot, 35 generated `.pyc` files and 10 now-empty
`__pycache__` directories were enumerated as absolute paths, verified to be
strictly below the project `ros_ws` root, and removed individually. A final
scan found zero remaining ROS-workspace cache files/directories. No source,
third-party checkout, or broader workspace path was deleted.

Pinned Windows checkout status at final audit:

- `adoodevv/so101_ros2`: clean detached
  `0305e03ab54e64aae9263fcbf339622e654012f3`;
- `TheRobotStudio/SO-ARM100`: clean detached
  `7629d2ad9853d10fb903093a33ef6114099d97e5`;
- legalaspro: no local checkout; remote pinned evidence only.

The earlier claim that adoodevv lacked MoveIt files is retracted. Local git
tree and fixed-SHA raw files prove the SRDF, move_group launch, controller and
planning YAML are tracked. Its README remains stale about MoveIt completeness
and the gripper controller. EdgeGrasp follows the actual pinned YAML FJT
mapping and does not copy or fabricate a MoveIt config overlay.

## 5. Resolved regression history

Historical failures are recorded to prevent old snapshots from becoming
current claims:

1. A transient tree lacked `src/edgegrasp/controller.py`, causing five import
   errors after nine collected items. The intended safety implementation was
   restored before further work.
2. The planner rename initially left tests importing
   `AxisAlignedWorkspacePlanner`; collection failed. The project now exposes
   only `EndpointWorkspaceGate / endpoint_blocked`, with no misleading alias.
3. Old call sites omitted required `execute_now_ns`; all callers now pass a
   distinct planning-boundary time and slow-planning expiry is tested.
4. The first current colcon run found an empty `edgegrasp_core` test package
   and one SafetyMonitor clock test based on an invalid zero-clock assumption.
   A package import smoke test and live-clock-relative rollback test fixed both.
5. Parallel whole-workspace testing exposed a three-second fake-controller
   teardown race. Bounded readiness waits, independent executor capacity, and
   shutdown ordering now pass cleanly in the final 25-test run.
6. Cancel acceptance without a terminal result could previously suppress a
   retry. It now has a separate completion deadline, at most three attempts,
   latched error/escalation, and a regression test.
7. The first live PlanningScene retry published a false permission pulse while
   the previously confirmed geometry was merely being requeried. The gate
   canceled and latched stop-unconfirmed. Revalidation now preserves a prior
   confirmation until timeout, service loss, mismatch, or clock fault, and a
   Jazzy regression test forbids false after first true.
8. Counts such as 26, 30, 108, 121, 124, 171, 181, 184, 187, 190, 206, 210, 25,
   44, 45, 46, 52, 74, 97, 98, 259, 260, 262, and 269 belong to recorded or
   superseded snapshots. The current Windows CPython 3.14.5 checkout collected
   332 project tests: 330 passed and two optional pinned-local-checkout tests
   skipped because that checkout was absent. The latest package-scoped
   EdgeGrasp colcon run passed 129/129 selected tests.
9. The first 22-test sequence package rerun passed but emitted unfetched
   Jazzy `InvalidHandle / Destroyable` teardown exceptions. The test harness
   had treated an empty sequence slot as executor quiescence even though the
   corresponding ActionServer coroutine could still be returning. Teardown
   now waits for pending action work, joins the executor worker pool, and
   fails on any retained task exception. The latest isolated package run is
   sequence 45/45 and `edgegrasp_ros` 49/49; XML results are clean. The
   intermittent `Destroyable because destruction was requested` warning did
   not recur in the final captured run, but its earlier evidence remains.

## 6. Remaining limitations

- No broad collision-safe claim. The core planner remains endpoint-only.
  MoveIt now has scoped required-table planning/rejection evidence, while only
  one named robot proxy and the physical table-cube pair have Gazebo contact
  evidence; optional-cube planning, minimum clearance, and the other proxies
  remain unverified.
- The four-stage controller and ROS wrapper complete correlated
  approach/descend/close/lift tasks against the actual
  Gazebo/MoveIt/gate/controllers graph. Candidate024 is the strongest scoped
  result: 10 independently correlated simulation-physics successes, 28.858-
  29.128 mm retained lift, and 9/9 post-fix repeatability. This does not
  establish diverse-target coverage, byte-identical planning, simulator-time
  determinism, force closure, or real-robot grasp success.
- The wrapper correctly avoids reusing the task's original target stamp for all
  commands: each stage binds to a fresh `TrackedTarget` observation, rejects
  target-position drift, and preserves exact command/digest correlation. Epoch
  reset coordination across the safety monitor, adapter, gate, and sequence
  wrapper is still locally managed rather than one atomic distributed reset.
- No physical-stop guarantee. ROS cancel/timeout/escalation contracts are
  tested with dependency-injected fake action servers; no healthy real
  MoveGroup cancel claim, motor power cut, torque stop, or real-controller stop
  was measured.
- Candidate024 reuses one fixed target pose. Its per-stage P50/P95 timing is
  measured, but no ten-distinct-target endpoint-error or grasp study exists yet.
- MCAP record/raw replay ran once per fresh epoch; deterministic repeated bag
  output, time mapping with live Gazebo, and physics replay remain unverified.
- Legacy safety/gate nodes still consume `PointStamped` and assign local
  epochs. `TrackedTarget` exists and is runtime-tested at the mock publisher,
  but cross-node consumption and reset/epoch propagation are not wired yet.
- `use_camera=false` produced SRDF torso/camera warnings. One move_group
  Ctrl-C shutdown ended with signal/exit `-11`; startup, planning, and the
  separately observed trajectory still succeeded, but shutdown robustness
  needs investigation.
- SO-ARM100 MuJoCo remains Windows-runtime blocked; static asset checks are not
  dynamics evidence.
- No real SO-101, calibration, EEPROM/udev, thermal, camera extrinsics, force
  calibration/closure, or hardware pick/place evidence exists.
- The Windows meta-repository still has no first commit, has zero tracked files,
  and presents the project tree as untracked; it must not be called clean. The
  WSL `edgegrasp-sim` source directory has no `.git` metadata and likewise has
  no clean/dirty Git claim. Only the two pinned upstream checkouts are assessed
  separately for clean state. No GitHub Actions workflow is used.

At the final candidate008 shutdown, exact `ROS_DOMAIN_ID=52` cleanup reported
zero matching simulator/controller/MoveIt/EdgeGrasp processes at
`2026-08-27T20:59:08.856480762+08:00`; `/clock` was then unknown. Reparented
children were selected from exact `/proc/.../environ` domain evidence rather
than a broad process-name kill. This is scoped process/clock cleanup evidence,
not a universal clean-shutdown claim.

Candidate024 r01-r11 independently repeated exact-domain cleanup in domains
139-149. All 11 artifacts report zero remaining matching processes and an
unknown `/clock` after cleanup. The post-fix repeatability subset is r03-r11;
it includes one live, correlated observer-baseline cancel/restart in r04.
