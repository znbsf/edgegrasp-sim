# Candidate005 contact-histogram revalidation

> Planning-model correction: this run accidentally launched pinned upstream
> `so101_moveit_config move_group.launch.py`, not
> `edgegrasp_proxy_move_group.launch.py`. Its Gazebo contact histogram and
> controller outcomes remain valid, but it does not prove that the EdgeGrasp
> pad-link proxy planning model accepted the path. The later
> `2026-08-27-candidate005-simultaneous-proxy-runtime.json` run supplies that
> scope and also records same-sample two-pad contact.

The unchanged candidate005 command sequence completed again through
PlanTarget, typed ExecuteTrajectory, and the two FollowJointTrajectory
controllers. The sequence wrapper returned `STATUS_SUCCEEDED`, every expected
phase reached its correlated terminal result, and the client returned 11 only
because the independent physics criterion did not pass.

This run closes the earlier per-pad observation gap. The fixed-finger proxy
token accumulated 1,159 qualifying cube contacts and the moving-finger token
accumulated 195, for 1,354 total. This proves that both configured pad tokens
were observed during the same trial. It does **not** prove that both pads held
the cube simultaneously or achieved force closure.

The cube rose only 0.331 mm versus the required 20 mm, returned to table
height, and never entered a retention window. The defensible result remains
`sequence_completed=true` and `physics_grasp_verified=false`. The evidence now
points away from a simple one-sided-contact absence and toward closure-force,
friction, contact-balance, or proxy/model limitations.

Before motion, the graph reported motion permission and interface readiness,
all three controllers, both FJT endpoints, all four EdgeGrasp actions, the
retained cube, and an ACM that allowed only the two distal pad links. The two
parent gripper links remained forbidden. Typed gripper preparation succeeded
without an explicit `use_sim_time` override, verifying the script's corrected
simulation-clock default.

Startup also reproduced a planning-scene service timing issue. An already
confirmed node could have its ACM transition refused whenever a periodic
read-only query happened to be in flight. This particular runtime used an
explicit bounded 5 s retry/query configuration. The source was subsequently
narrowed so a confirmed periodic read can be superseded by the new policy,
while an initial unconfirmed query still fails closed; that source change
requires its own post-change Jazzy test and is not attributed to this run.

The Gazebo server log contains zero tracked DART mesh-construction or geometry
creation diagnostics. The robot remained anchored at world pose zero, all
controllers remained active after the trial, and cleanup left zero matching
processes and no `/clock` publisher. Exact values, command IDs, hashes, WSL
paths, timings, and claim boundaries are in the adjacent JSON file.
