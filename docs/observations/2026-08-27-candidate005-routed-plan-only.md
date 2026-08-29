# Candidate005 routed plan-only observation

This observation found a reproducible collision-planning route without
executing the robot. The target cube stayed in the MoveIt PlanningScene. Only
the two EdgeGrasp distal-pad links were allowed to contact it; gripper_link
and moving_jaw_so101_v1_link remained forbidden.

The original straight Pilz LIN approach still failed closed on
edgegrasp_target_cube versus gripper_link. An OMPL RRTConnect comparison
proved that a route exists, but only 6 of 10 responses passed both MoveIt and
the EdgeGrasp trajectory validator, so it is not the selected reproducible
path.

One accepted OMPL path supplied a collision-valid routing state. Its FK pose
became candidate005's approach via. Pose-constrained Pilz PTP was then run
ten times for each arm segment:

- home to approach via: 10/10, 83 source points and 82 gate-ready points;
- approach via to descend: 10/10, 32 source points and 31 gate-ready points;
- closed descend to lift: 10/10, 22 source points and 21 gate-ready points.

Every probe called only /plan_kinematic_path; execution_attempted was false.
The graph deliberately launched no EdgeGrasp motion nodes. This proves a
repeatable plan-only route, not action execution, contact, cube lift,
retention, or simulated grasp success. Full machine-readable inputs, timings,
hashes, and cleanup evidence are in the adjacent JSON file.
