# Candidate005 typed runtime observation

Candidate005 completed the entire correlated command sequence in Gazebo:
APPROACH, DESCEND, CLOSE_GRIPPER, and LIFT all advanced through the typed
PlanTarget/ExecuteTrajectory path. The final sequence wrapper status was 4,
the terminal phase was COMPLETE, and the last correlated command was
`candidate005-physics-001|lift|3`.

This run improved the physical evidence but did not produce a grasp. The
independent observer recorded 1,283 qualifying cube-to-gripper-proxy contacts.
Its reported collision pair contained the fixed-finger pad proxy. Cube height
rose only 0.316 mm against the required 20 mm, returned to table height, and
never satisfied the retention window. The correct result is therefore
`sequence_completed=true` and `physics_grasp_verified=false`.

The initial typed gripper setup also exposed a real clock-domain defect: the
tool accepted only `ros_sim` targets while its node clock defaulted to system
time. Two commands failed before dispatch. With `use_sim_time=true`, the third
command completed through the EdgeGrasp gate with wrapper status 4 and FJT
error code 0. The script now defaults to simulation time and fails closed if
that invariant is not active.

The robot remained anchored at world pose zero, all three controllers remained
active, and the Gazebo server log contained zero tracked DART mesh-construction
or geometry-creation diagnostics. Cleanup used exact PIDs; no matching process
or `/clock` topic remained. Some launch children required their normal
SIGINT-to-SIGTERM escalation, which is retained as teardown evidence.

The machine-readable adjacent JSON contains exact poses, command IDs, digests,
log hashes, times, and claim boundaries. It does not claim bilateral contact,
20 mm lift, retention, full collision fidelity, or simulated grasp success.
