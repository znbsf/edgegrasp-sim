# Candidate007 contact-timing runtime observation

Status: `SEQUENCE_COMPLETE_TRANSIENT_BILATERAL_CONTACT_PHYSICS_GRASP_FAILED`.

The clean `ROS_DOMAIN_ID=51` run kept the candidate005 geometry and motion
parameters, then added source-time correlation between contact evidence and the
typed sequence terminal. The sequence again completed through the EdgeGrasp
plan-only adapter, typed trajectory gate, and downstream FJT actions. Independent
physics evidence still failed: peak cube lift was **0.359 mm** versus the required
**20 mm**, and no retention window began.

The timing result is more diagnostic than the earlier contact maxima. Both pad
tokens appeared in the same 39 contact messages, but only over **0.099 s** of
simulation time. Their last simultaneous sample was at 37.603 s; the correlated
sequence terminal was at 39.350 s, a gap of **1.747 s**. The later moving-pad
contact ended 1.400 s before sequence completion. Simulated gripper effort was
about `-1.49e-4` at sequence completion after an earlier 0.611 absolute peak.

The observed fact is therefore transient bilateral contact followed by absent
bilateral contact before the lift terminal. Slip, unload, and geometry loss are
candidate mechanisms, not separately proven causes. The contact solver values
remain Gazebo/DART diagnostics—not calibrated force, force closure, or real
hardware evidence.

Two prior graphs did not become motion trials. Domain 49 sent a typed gripper
preparation goal, which the gate rejected before downstream dispatch because
safety permission was false. Domain 50 stopped before MoveIt or any motion when
the harness propagated `sort`'s SIGPIPE under `pipefail`; the log selector was
then corrected. Both domains were cleaned by exact `ROS_DOMAIN_ID`, with zero
matching processes and no `/clock` topic afterward.

Machine-readable evidence, artifact paths, source timestamps, hashes, and claim
boundaries are in
[`2026-08-27-candidate007-contact-timing-runtime.json`](2026-08-27-candidate007-contact-timing-runtime.json).
