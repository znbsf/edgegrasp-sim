# Candidate012 finger-pad friction observation

Candidate012 ran a paired Gazebo/DART experiment on the same Candidate011
geometry, scene, target, planning pipeline, scaling, and typed execution path.
The generated pad collisions used explicit isotropic `mu=mu2=1.0` for the
control and `1.5` for the treatment. All three material rows first passed
10/10 plan-only repetitions with zero execution, and the two explicit runtime
rows then completed APPROACH/DESCEND/CLOSE/LIFT.

The treatment did not solve the grasp. Control peak lift was 3.392 mm and
treatment peak lift was 3.468 mm, a difference of only 0.076 mm. Simultaneous
two-pad contact increased from 160 samples / 0.437 s to 172 samples / 0.450 s,
but in both runs contact ended about 1.75 s before sequence completion, the
cube returned to table height, and retention was false. Both results therefore
remain `physics_grasp_verified=false`.

The static URDF-to-SDF mapping proves only that the requested `mu` and `mu2`
values reached the two generated pad collision surfaces. It does not by itself
prove the DART solver applied friction as intended. The paired runtime result
is best interpreted as a negative sensitivity experiment: within the bounded
1.0-to-1.5 range, friction was not the dominant missing ingredient. The next
useful change is distal-pad/contact geometry or grasp kinematics, not a wider
unbounded friction sweep. Exact paths, hashes, failed pre-motion attempts, and
cleanup evidence are in the adjacent JSON.
