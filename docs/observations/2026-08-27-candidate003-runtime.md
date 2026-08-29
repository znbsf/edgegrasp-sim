# Candidate 003 collision and physics runtime

Observation window: `2026-08-27T16:24:42.842897+08:00` through `2026-08-27T16:32:42.7748803+08:00`.

Candidate 003 improved the command path but did not achieve a physical grasp:

- Five EdgeGrasp ROS packages built. Package-scoped Jazzy tests passed 98/98 (`core=1`, `interfaces=0`, `ROS=46`, `MoveIt adapter=21`, `sequence=30`). The known ROS `Destroyable` teardown warning remains.
- With table and cube both present in MoveIt, the far approach passed and executed exclusively through `PlanTarget -> ExecuteTrajectory -> arm FJT`. The cube pose was unchanged and direct MoveGroup/FJT bypass was false.
- The explicit target-contact policy then removed only the optional cube from MoveIt; the required table stayed active and the Gazebo cube remained. Approach-to-descend and descend-to-lift plan-only checks passed before motion.
- All four correlated sequence stages reached successful FJT terminals.
- The tightened physics observer accepted only exact distal-pad proxy collision tokens. It observed zero qualifying contacts, only 2.43 mm peak cube lift, no retention, and about 52.3 mm lateral displacement. The final raw contact was cube-to-table.

Therefore `sequence_completed=true` but `physics_grasp_verified=false`. This is reproducible negative grasp evidence, not a simulated pick success. Full machine-readable commands, digests, poses, logs, and teardown defects are in `2026-08-27-candidate003-runtime.json`.
