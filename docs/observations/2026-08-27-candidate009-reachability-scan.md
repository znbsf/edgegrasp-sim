# Candidate009 reachability-boundary scan

The static symmetry optimum was not executable. With all arm orientation,
scene, ACM, planner, tolerance, and 0.50 rad gripper parameters fixed, the
zero-offset Candidate008 control passed all three chained Pilz PTP segments in
10 of 10 attempts. Each segment also produced one stable trajectory digest.

The first negative tool-X grid step, -0.1 mm, failed the
approach-to-descend segment with `NO_IK_SOLUTION`. The sampled -0.2, -0.3,
-0.5, -0.7, -0.8, and -0.9 mm cases failed the same way. Every graph omitted
the EdgeGrasp motion adapter, gate, and sequence; trajectory-publication,
ExecuteTrajectory-goal, FJT-goal, and execution-attempt counters stayed zero.
Each domain was cleaned and `/clock` was absent afterward.

The correct conclusion is not that the static geometry calculation was wrong;
it is that OBB symmetry alone ignored the current kinematic boundary. The
translation-only search is stopped. The next controlled variable keeps the
known reachable arm poses and reduces the close angle from 0.50 to 0.45 rad,
again requiring static and plan-only gates before any physics run.

The adjacent JSON contains every sampled offset, artifact path, result hash,
repeatability statistic, and evidence boundary.
