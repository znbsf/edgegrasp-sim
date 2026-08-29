# Candidate004 selective-contact plan-only runtime

Observation window: `2026-08-27T17:10:39.540791+08:00` through
`2026-08-27T17:18:40.094517463+08:00`.

This run closed the PlanningScene policy gap without executing candidate004:

- Gazebo/DART started with all three controllers active, six joint states and
  both pinned FJT actions. The tracked DART mesh-construction and geometry
  creation diagnostics remained at zero.
- MoveIt confirmed the table and target cube from the shared scene contract.
  `/edgegrasp/set_target_pad_contacts` retained the cube and enabled target
  contact only for `edgegrasp_fixed_finger_pad_link` and
  `edgegrasp_moving_finger_pad_link`. `gripper_link` and
  `moving_jaw_so101_v1_link` remained forbidden in the queried ACM.
- The current approach-to-descend path was rejected twice by Pilz
  `ValidateSolution`, with both gripper=1.5 and gripper=0.60. States 27–42 of
  51 collided between `edgegrasp_target_cube` and `gripper_link`.
- The closed-gripper descend-to-lift control returned MoveIt SUCCESS with a
  valid 22-point trajectory. The probe is plan-only and did not publish or
  execute either trajectory.

Candidate004 therefore remains execution-blocked. The evidence proves that the
selective ACM is active and fail-closed; it does not prove a collision-free
descent, distal-pad contact, cube lift/retention, or simulated grasp success.
The exact logs, hashes, counts and cleanup state are in the adjacent JSON file.
