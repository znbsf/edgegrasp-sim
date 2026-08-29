# Shared PlanningScene runtime observation

Run window: `2026-08-27T01:23:51+08:00` to
`2026-08-27T01:41:00+08:00`, Ubuntu 24.04 WSL2, ROS 2 Jazzy,
Gazebo Sim 8.11.0 with DART. Gazebo was the only `/clock` publisher.

The EdgeGrasp loader published the shared table contract to MoveIt and did not
declare ready until `/get_planning_scene` returned the expected geometry:

```text
id: edgegrasp_table
frame: base_link
BOX dimensions: [0.6, 0.8, 0.04] m
position: [0.35, 0.0, 0.38] m
scene digest: adbc39d4a582744159a71c18abf05cdcc1ce673fc3e238577a211d6f09e00665
components mask: WORLD_OBJECT_NAMES | WORLD_OBJECT_GEOMETRY = 24
```

The first A run exposed a real readiness race: periodic revalidation emitted a
false Bool after the scene had already been confirmed. MoveIt planned, the
controller reached its goal, but the adapter correctly canceled when that
false pulse arrived; cancellation was not terminally confirmed, so the gate
latched a tombstone. The loader now preserves the last confirmed readiness
during an in-flight requery and revokes it only on timeout, service loss,
geometry mismatch, or clock failure. A Jazzy test observes several requery
cycles and asserts that no false value follows the first true.

After a clean graph restart, the scoped cases were:

| Case | MoveIt/result | Gate/controller | Outcome |
| --- | --- | --- | --- |
| A current reachable pose | MoveIt `1` | exactly one typed arm command; wrapper/FJT status `4`; FJT error `0` | PASS |
| B point inside table | collision-aware IK `-31` | zero trajectory dispatch | rejected as required |
| C known-FK goal requiring a table crossing/detour | `ValidateSolution` returned `INVALID_MOTION_PLAN` / `99999` | zero trajectory dispatch | rejected as required |
| D far unreachable point | IK `-31` | zero trajectory dispatch | rejected as required |
| E epoch mismatch | `clock_epoch_mismatch` | zero trajectory dispatch | rejected as required |
| E slow planning | target exceeded 200 ms | zero trajectory dispatch | rejected as required |

The C plan-only service diagnostic returned a nonempty candidate trajectory
but error code `99999`; EdgeGrasp did not treat nonempty data as success and
did not execute it. This is the intended fail-closed distinction between a
candidate and an accepted collision-valid plan.

Exact structured results and WSL artifact paths are in
[`2026-08-27-planning-scene-runtime.json`](2026-08-27-planning-scene-runtime.json).
Only the table was loaded into MoveIt in this run. Optional-cube loading,
minimum clearance, every robot collision proxy, a ROS grasp orchestrator, cube
lift/retention, and physical grasp remain unverified. Shutdown inspection found
zero ROS nodes, no `/clock` topic, and no matching Gazebo/MoveIt/EdgeGrasp
runtime process.
