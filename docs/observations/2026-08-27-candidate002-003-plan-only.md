# Candidate 002/003 plan-only observation

Recorded at `2026-08-27T16:18:07.2821027+08:00` on WSL2 Ubuntu 24.04, ROS 2 Jazzy, Gazebo 8.11/DART, `ROS_DOMAIN_ID=230`.

- The shared PlanningScene was confirmed ready with `edgegrasp_table` and `edgegrasp_target_cube`, digest `8fb303a83b4d8c22ef6e491d5a33ea73b39371a17adc8511db47a5f1c89b6f35`.
- Candidate 002's negative tool-axis approach failed closed. Pilz generated 91 points, but `ValidateSolution` rejected indices 57 through 68 for `edgegrasp_target_cube` versus `gripper_link`; OMPL separately reported `GOAL_STATE_INVALID` and returned no trajectory.
- Candidate 003 keeps the corrected descend-side palm clearance and moves the approach to 30 mm along tool `+Z` plus 80 mm along planning-frame `+Z`. Collision-aware IK succeeded, and Pilz PTP returned `MoveItErrorCodes.SUCCESS` with 84 source points and 83 validated gate-ready points while the cube remained in the scene.
- Every command in this observation was read-only planning. `execution_attempted=false`; no trajectory was published or sent to a controller.

This is approach plan-only evidence, not descend/contact, sequence, physics-grasp, or hardware evidence. Machine-readable details are in `2026-08-27-candidate002-003-plan-only.json`.
