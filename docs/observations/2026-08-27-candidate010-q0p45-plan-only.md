# Candidate010 q=0.45 plan-only observation

Candidate010 keeps the known-reachable Candidate008 arm poses and changes one
variable: the gripper contact position from 0.50 to 0.45 rad. The generated
proxy profile reduces the predicted inner gap from about 37.71 mm to 35.44 mm
and passes the same static exact-OBB and body-clearance gates.

In ROS domain 61, all three chained Pilz PTP segments passed 10 of 10 plan-only
attempts. Source/gate-ready point counts were stable at 83/82, 32/31, and
22/21, and each segment had one unique trajectory digest across all ten runs.
The graph contained no EdgeGrasp adapter, trajectory gate, or sequence node;
zero trajectories and zero action goals were sent.

This authorizes one controlled physics attempt, not a success claim. The next
run must still pass the typed PlanTarget/ExecuteTrajectory chain and separately
measure bilateral contact, cube lift of at least 20 mm, and retention. The
adjacent JSON records exact timings, hashes, graph boundaries, poses, and
cleanup evidence.
