# Candidate011 q=0.40 runtime observation

Candidate011 completed the correlated PlanTarget/ExecuteTrajectory sequence,
but the independent physics observer again rejected grasp success. The 0.40
rad close angle produced the strongest transient result in the controlled
0.50/0.45/0.40 series: 171 simultaneous two-pad samples over 0.463 simulated
seconds and a 3.711 mm peak lift.

It still failed the actual contract. Bilateral contact ended 1.737 simulated
seconds before sequence completion, the cube returned to table height, peak
lift was below 20 mm, and retention was false. The result is therefore
`physics_grasp_verified=false`, not a simulated grasp.

The close-angle search now stops. Further closing would increasingly tune
proxy penetration while the evidence points to insufficient contact holding.
The shortest honest next experiment is a bounded finger-pad friction/contact
material study or a distal-pad proxy-geometry revision, preserving the same
success gate. Exact measurements, hashes, and cleanup evidence are in the
adjacent JSON.
