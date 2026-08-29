# Candidate005 proxy-MoveIt simultaneous-contact runtime

This observation contains two clean-graph attempts and preserves both outcomes.
The first (`ROS_DOMAIN_ID=44`) observed a one-millisecond ROS simulation-clock
rollback while the approach was active. EdgeGrasp canceled the command and
latched `SAFE_STOP`; no later stage ran. The graph had exactly one `/clock`
publisher (`ros_gz_bridge`). The safety policy was not relaxed or reset in that
graph.

After exact-PID cleanup, a single bounded retry used a new graph and
`ROS_DOMAIN_ID=45`. It launched
`edgegrasp_proxy_move_group.launch.py`, confirmed both EdgeGrasp distal-pad
links in MoveGroup's `robot_description`, loaded `ValidateSolution` for all
three planning pipelines, retained table and cube, and confirmed an ACM that
allowed only those two pad child links to contact the cube. There was no MoveIt
execution action; motion flowed through PlanTarget, typed ExecuteTrajectory,
and the arm/gripper FollowJointTrajectory controllers.

The retry completed APPROACH, DESCEND, CLOSE_GRIPPER, and LIFT with matching
terminal correlation. It recorded 1,125 fixed-pad-token and 188
moving-pad-token contacts. More importantly than the earlier histogram, both
configured tokens appeared in the same contact sample 35 times. This closes
the observation-timing ambiguity but does not establish force closure.

The cube rose only 0.332 mm versus the required 20 mm, returned to table
height, and never entered the retention window. The result is therefore
`sequence_completed=true`, `physics_grasp_verified=false`. The robot remained
anchored at world pose zero, all controllers stayed active, the Gazebo log had
zero tracked DART mesh/geometry diagnostics, cleanup left zero matching
processes, and `/clock` publisher count returned to zero.

The preceding contact-histogram observation used the pinned upstream MoveGroup
launch by operator mistake. Its Gazebo contact measurements remain useful, but
it is not evidence that the EdgeGrasp proxy planning model accepted the path.
This observation supersedes it only for planning-model/runtime-chain scope.
Exact commands, IDs, digests, counts, timestamps, hashes, and boundaries are in
the adjacent JSON file.
