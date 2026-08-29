# Candidate009 plan-only runtime observation

Candidate009 did not pass the execution gate. In an isolated ROS domain, the
new service-only probe planned the unchanged home-to-approach segment, then
MoveIt rejected the shifted approach-to-descend target. The service returned
error code `99999`; the fixed run log identifies `NO_IK_SOLUTION` and says it
could not compute inverse kinematics for `gripper_frame_link`. The lift segment
was therefore not attempted.

The graph deliberately contained Gazebo, the three controllers, pinned
MoveIt, and the shared PlanningScene loader, but no EdgeGrasp adapter,
trajectory gate, or grasp-sequence node. The probe created no publisher or
action client. Its counters remained zero for trajectory publication,
`ExecuteTrajectory` goals, FJT goals, and execution attempts. Cleanup found no
remaining process in ROS domain 53 and `/clock` was absent afterward.

This preserves the useful negative result: a static OBB optimum is not enough
to establish kinematic feasibility. Candidate009 remains a static symmetry
candidate only and is not authorized for execution. The next bounded step is a
plan-only search back toward candidate008's reachable pose, keeping the 0.50
rad close angle and all other variables fixed.

The adjacent JSON records the exact poses, error, hashes, paths, controller and
graph boundary, and cleanup evidence.
