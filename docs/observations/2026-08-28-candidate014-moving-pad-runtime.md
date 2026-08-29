# Candidate014 moving-pad attachment observation

Candidate014 changed one simulation variable from Candidate011: the moving
distal-pad box was extended by 12 mm along negative moving-jaw local Y while
its proximal edge, arm poses, 0.40 rad close command, scene, planner, scaling,
and explicit `mu=mu2=1.0` stayed fixed. The resulting 49.985 mm contact length
is an EdgeGrasp simulation attachment, not upstream or verified hardware
geometry.

The corrected isolated plan-only graph passed all three chained Pilz segments
in 3/3 attempts. Every returned trajectory was validated and discarded;
trajectory publication, ExecuteTrajectory goals, FJT goals, and execution all
remained zero. A prior apparent rejection was traced to the harness calling the
selective ACM service before the base PlanningScene was confirmed and ignoring
its explicit `success=False` response. The corrected order requires base scene
confirmation, `SetBool success=True`, and a later status with
`allow_target_pad_contacts=true`.

The single typed Gazebo run completed APPROACH, DESCEND, CLOSE_GRIPPER, and
LIFT, but failed the unchanged physics gate. Peak cube lift was 3.680 mm versus
the required 20 mm, bilateral contact lasted 0.462 s and ended 1.744 s before
sequence completion, the cube returned to table height, and retention was
false. Compared with Candidate011, peak lift decreased by 0.031 mm and the
bilateral span decreased by 1 ms. The extension therefore did not identify
contact-patch length as the current bottleneck and is not a simulated grasp.

The machine-readable record, exact commands, hashes, rejected alignment
preflights, harness correction, and cleanup evidence are in
`2026-08-28-candidate014-moving-pad-runtime.json`.
