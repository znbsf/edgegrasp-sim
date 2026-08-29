# Candidate006 contact-quality runtime observation

Status: `SEQUENCE_COMPLETE_CONTACT_QUALITY_MEASURED_PHYSICS_GRASP_FAILED`.

The clean `ROS_DOMAIN_ID=48` run used the EdgeGrasp proxy MoveGroup, shared
table/cube scene, typed `ExecuteTrajectory` gate, correlated four-stage
sequence, and read-only physics observer. The sequence completed, but the
observer correctly returned `physics_grasp_verified=false`: peak cube lift was
only **0.333 mm**, versus the required **20 mm**, and no retention window began.

This run closes a narrower diagnostic gap. It measured 920 fixed-pad-token and
170 moving-pad-token contacts, including 36 messages containing both tokens.
The fixed/moving maximum penetration depths were about 8.55/3.43 micrometres;
their maximum absolute normal-projected solver forces were 54.42/16.01 N. The
same-message weaker-side maxima were 3.02 micrometres and 16.01 N. Gripper
effort peaked at 0.765 simulated units but was almost zero at observation end.

These are Gazebo/DART solver diagnostics, not calibrated hardware force and not
proof of sustained bilateral loading or force closure. The supported inference
is only that candidate005 made transient/asymmetric two-pad contact rather than
making no contact. Before changing the 0.60 rad close target, the observer must
correlate first/last pad contacts with sequence phase. A new close target also
requires a regenerated OBB profile; bypassing the current geometry lock is not
allowed.

Two earlier graph attempts were rejected before the sequence: domain 46 exposed
a target-ID mismatch, and domain 47 exposed a target-position/scene mismatch.
Both fail-closed checks were preserved. The final harness now passes the shared
scene target ID and coordinates explicitly and cleans reparented processes by
exact `ROS_DOMAIN_ID`; domain 48 ended with zero matching processes and no
`/clock` topic.

Machine-readable evidence, exact paths, hashes, and claim boundaries are in
[`2026-08-27-candidate006-contact-quality-runtime.json`](2026-08-27-candidate006-contact-quality-runtime.json).
