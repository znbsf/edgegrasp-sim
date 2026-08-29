# Four-stage sequence repeatability observation

Observed on 2026-08-27 in the isolated Ubuntu 24.04 WSL2 / ROS 2 Jazzy /
Gazebo 8.11 DART workspace. The accepted artifact window is
05:20:07.547 through 05:22:58.703 Asia/Shanghai.

## Accepted result

The current-source harness launched the installed GraspSequence server and
client entry points directly, gave every run a unique action endpoint and task
ID, and owned the actual server PID through shutdown. All ten runs recorded:

```text
client_exit=0
sequence_completed=true
shutdown_clean=true
wrapper_status=4
physics_grasp_verified=false
```

The accepted summary is:

```text
/home/edgegrasp/ros2_ws/test_results/grasp_repeat_direct_final_20260827_0520/summary.tsv
SHA-256 8658ff43a9974edec6f6c69934f37052e530ebff6fb1891a32c899268d38c4da
RESULT runs=10 successes=10 failures=0
```

The ten results contain ten distinct final trajectory digests. This proves
10/10 repeatability of the typed protocol outcome, not byte-identical MoveIt
planning, ROS scheduling determinism, physics determinism, or grasp success.

## Reproduction

With the documented Gazebo, controllers, PlanningScene, MoveGroup, safety
monitor, interface probe, trajectory gate, and MoveIt adapter graph already
healthy:

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

These below-table smoke poses verify wiring and ordering only. They do not
interact with the target cube and must not be reused from a different start
state without a fresh pose/orientation check.

## Failed and invalid evidence retained

- The earlier strict batch at `grasp_repeat_final_20260827_0450` completed
  4/10 and failed closed six times: three future-source-timestamp faults and
  three gate cancel rejections.
- `grasp_repeat_current_20260827_0510` is **invalid measurement evidence**.
  Its old harness backgrounded `ros2 run`, tracked the CLI wrapper instead of
  the child action server, and could report a clean shutdown while nodes
  lingered. Its nominal 6/7 completions are not an acceptance result.
- The first direct-entrypoint batch at `grasp_repeat_direct_20260827_0514`
  completed 1/2; run 2 failed on `task_source_stale`, and node shutdown exposed
  an rclpy invalid-context wait-set exception.
- After the guarded shutdown fix, `grasp_repeat_direct_smoke_20260827_0518`
  passed 2/2 before the accepted 10-run batch.

Keeping these artifacts makes the measurement correction and fail-closed
history auditable; the 10/10 result does not erase them.

## Boundary and cleanup

The sequence result can become successful only after each correlated
PlanTarget / ExecuteTrajectory / FJT terminal satisfies the typed contract.
No failed run was converted into a success. However, no cube contact, lift, or
retention signal was observed, so every result correctly preserved
`physics_grasp_verified=false`.

After the accepted batch, no GraspSequence process or action server remained
and `/clock` had zero publishers. Full-graph teardown is not clean evidence:
MoveGroup still exited `-11` on SIGINT and Gazebo exited `-2` after SIGINT.

Machine-readable details are in
`2026-08-27-grasp-sequence-repeatability.json`.
