# Candidate010 q=0.45 runtime observation

Candidate010 completed the full correlated sequence through PlanTarget,
ExecuteTrajectory, and both FollowJointTrajectory controllers, but it did not
pass the independent physics gate. The client returned the defined code 11:
sequence complete, physics unverified/failed.

The 0.45 rad close angle was a measurable improvement over Candidate008 at
0.50 rad. Simultaneous two-pad samples increased from 98 to 144, their span
from 0.252 to 0.368 simulated seconds, peak lift from 2.667 to 3.323 mm, and
approximate final XY drift fell from 16.57 to 14.12 mm. Simulated peak gripper
effort rose from 1.44 to 2.31.

It is still not a grasp success. Bilateral contact ended 1.744 simulated
seconds before sequence completion, the cube returned to table height, peak
lift was far below the required 20 mm, and retention was false. The physics
observer therefore ended in `FAULT/observation_wall_timeout` with
`physics_grasp_verified=false`.

The run used one `/clock` authority, retained all three active controllers,
kept the robot anchored, recorded zero DART mesh-construction diagnostics, and
cleaned ROS domain 62 completely. The adjacent JSON contains exact action
correlation, contact/effort/timing data, comparisons, hashes, and evidence
boundaries.
