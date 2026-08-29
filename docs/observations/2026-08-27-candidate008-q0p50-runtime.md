# Candidate008 q=0.50 runtime observation

Recorded on 2026-08-27 (Asia/Shanghai) in a clean `ROS_DOMAIN_ID=52`
Gazebo Harmonic / DART graph. The run used the generated
`so101_grasp_geometry_candidate008_q0p50.json` profile and commanded the
gripper to `0.50 rad`; the earlier candidate007 control used `0.60 rad`.

The correlated sequence completed through the EdgeGrasp plan-only adapter,
typed trajectory gate, arm FJT, and gripper FJT. The independent physics
observer nevertheless returned exit 11 and `physics_grasp_verified=false`.
The success rule was not relaxed: it still requires contact, at least 20 mm of
cube lift, table clearance, and a 0.5 s retention window.

| Measurement | Candidate007 | Candidate008 q=0.50 |
|---|---:|---:|
| Simultaneous two-pad samples | 39 | 98 |
| Simultaneous span | 0.099 s | 0.252 s |
| Peak simulated gripper effort magnitude | 0.611 | 1.439 |
| Peak cube lift | 0.359 mm | 2.667 mm |
| Physics grasp verified | No | No |

Candidate008 is useful negative/control evidence: tighter closure produced
2.51x as many simultaneous samples, a 2.55x longer simultaneous span, and a
7.42x higher transient lift. It still did not hold the cube. Bilateral contact
ended 1.788 simulated seconds before sequence completion, the cube returned to
table height, and retention was false. The final Gazebo CLI pose implies about
16.57 mm of XY drift and 0.438 rad yaw; that drift is approximate because the
CLI prints six decimal places.

The contact wrench and effort figures are Gazebo/DART diagnostics, not
calibrated hardware force or force-closure proof. The next controlled study
should address pose/contact symmetry or friction/contact parameters rather than
blindly closing the gripper further.

Reproduction command:

```bash
bash scripts/run_candidate005_contact_quality.sh 52 \
  /home/edgegrasp/ros2_ws/test_results/candidate008_q0p50_20260827_2105 \
  candidate008-q0p50-001 \
  so101_grasp_geometry_candidate008_q0p50.json \
  0.50
```

The complete machine-readable record, including timestamps, contact timing,
quality diagnostics, trajectory/evidence digests, controller/action inventory,
cleanup result, and log hashes, is in
[`2026-08-27-candidate008-q0p50-runtime.json`](2026-08-27-candidate008-q0p50-runtime.json).
