# Fixed-scene RGB-D grasp development

Status: all three declared positions have end-to-end RGB-D and independent
simulation physics success. Retained lift: control 28.85 mm, X-minus 28.91 mm,
X-plus 28.68 mm. Original failures and temporal-alignment limitations remain
recorded. See [the concise result and reproduction guide](rgbd-static-grasp-result.md)
and `observations/2026-09-06-rgbd-final-validation.json`.

## Scope and evidence

Use the existing 50 mm red cube, fixed table, camera extrinsics and declared
Candidate024 yaw (0.6500077341171558 rad). The first position is the existing
Candidate024 control. The X +/-1 mm positions are now declared in
`ros_ws/src/edgegrasp_ros/config/rgbd_static_scope.json`; both have now individually
cleared perception, isolated planning and independent simulation physics. The
initial cube yaw is declared; post-contact rotation is measured within a declared
40-degree bound. The scope does not cover arbitrary object poses.

The estimator takes registered RGB, metric depth, intrinsics and an optical-to-
planning transform. It does not read scene configuration, model pose, contact
truth, target coordinates or grasp profiles. A separate capture callback writes
truth to an evaluation-only file. The evaluator alone reads that file.

Red pixels are eroded at the silhouette, unprojected and fitted to the three
visible planes of the declared cube. Invalid depth, missing faces, clipped
objects and inconsistent surfaces reject. Residuals measure fit quality; they
do not independently guarantee pose accuracy or prove identity under occlusion.

The existing center tolerance is 3/6/4 mm in the grasp frame; the narrower
fixed-pad preclose clearance interval is 0.5–1.5 mm, with baseline 1.002 mm.
For spatial evaluation we require Euclidean center error <=0.25 mm, reserving
half the narrowest clearance margin. Existing planning/collision and physical
criteria remain mandatory.

The sections below preserve historical development checkpoints, including their
then-current limitations and failed experiments. The status above and the linked
concise result guide describe the final implementation.

## Camera findings

The pinned camera publishes 424x240 registered RGB-D at 5 Hz, with CameraInfo and
both images stamped `camera_head_link`. Optical coordinates require the
colocated `camera_head_depth_optical_frame` transform. The separate color optical
frame includes a 15 mm translation and must not be substituted.

Gazebo's default rendering projection samples pixel centers at `(u+0.5,v+0.5)`
relative to the published principal point. For one synchronized captured cloud,
integer unprojection disagreed with the camera's own XYZ by up to 1.167 mm;
half-pixel unprojection agreed within 2.4e-8 m. This was a sensor-to-sensor check,
not a correction fitted to target truth. The installed Ogre2 depth shaders also
reconstruct view-space positions from interpolated camera rays. The half-pixel
adapter is deliberately limited to this pinned simulated camera.

At 5 Hz, source ages repeatedly exceeded the 100 ms admission budget. An optional
sensor-only 20 Hz overlay leaves upstream files, collision geometry and time
thresholds unchanged. The first 20 Hz capture retained one initial no-target
rejection and nine accepted observations, each with approximately 50 ms source
age. This short observation run is not a latency soak or motion-time guarantee.

## Reproduce the initial checkpoint (historical)

In Ubuntu-24.04, with this worktree available at
`/mnt/c/Users/huang/.codex/worktrees/c313/edgegrasp-sim-private`:

```bash
cd /mnt/c/Users/huang/.codex/worktrees/c313/edgegrasp-sim-private
bash scripts/check_rgbd.sh
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash
# Select a fresh external output path and verify that domain 166 is unused.
python3 scripts/run_rgbd_capture.py NEW_EXTERNAL_CAPTURE_DIR --domain 166
```

Capture launches no MoveGroup, target publisher or grasp node. It requests no
motion, records at most ten synchronized samples within 40 seconds, and cleans
only this run's processes. Gazebo may require explicit cleanup of its child
after the Ruby launcher terminates; cleanup results are retained.

The ROS `rgbd_target_publisher` entry point publishes the existing `TrackedTarget`
and `PointStamped` streams. It requires a declared yaw and simulation clock,
checks exact RGB/depth/info timestamps and TF at source time, preserves source
timestamps and epoch, checks the 100 ms budget again after processing, refuses
duplicates and latches clock rollback. Its ROS-message regression uses no
MoveGroup, controller or simulator. It has not yet been qualified as a live
execution input.

The existing plan-only runner accepts `EDGEGRASP_RGBD_OBSERVATION` pointing to a
recorded estimate JSON. With that option it enables the camera and derives
descend/lift from the recorded observed center; scene geometry remains the
independent planning/collision environment. Recorded observations are spatial
planning evidence only, never fresh execution admission.

```bash
export EDGEGRASP_RGBD_OBSERVATION=/home/edgegrasp/ros2_ws/test_results/rgbd_20hz_20260906T0110/first_accepted_estimate.json
bash scripts/run_grasp_candidate_plan_only.sh 168 NEW_EXTERNAL_PLAN_DIR \
  rgbd-first-observation so101_grasp_geometry_candidate024_face_aligned_q0p40.json \
  1 0.0 implicit_default 0.0 so101_side_grasp_candidate024_face_aligned.json \
  scene_candidate024_face_aligned.json table_cube_candidate024_face_aligned.sdf
```

Choose the first accepted observation, without selecting by truth error. Retain
all rejection rows and record the selection. Raw NPZ, JSONL and logs live outside
Git; the observation index records paths and key hashes.

## Remaining work before a physical-success claim

1. Extend the live publisher/safety qualification to manipulation occlusion and
   the faster sensor cadence needed for the independent observer baseline.
   The 20 Hz live test published 39 source-correlated targets; permission became
   false after input loss and no true permission was recorded past source expiry.
2. Resolve the existing sequence's whole-cycle 5 mm drift-from-initial-center
   invariant: a real observation of a commanded >=20 mm lift necessarily violates
   it. An opt-in `rigid_lift` mode now binds the observed cube's position in the
   gripper frame at lift dispatch, after the existing correlated close terminal.
   During LIFT_EXEC it compares the visual center against that rigid transform
   at the exact image source timestamp. The 5 mm residual, freshness gates and
   independent physics criteria remain unchanged. Missing time-correlated TF,
   task/epoch reuse and slip reject. Zero-command ROS health checks pass; motion-
   time qualification remains open. The static mode retains its original check.
3. Declare the remaining small position set, pass fresh per-scene plan-only, then
   run bounded Gazebo grasp trials and retain every failure. Require independent
   same-sample dual-pad contact, >=20 mm lift and >=0.5 s retention.

`accepted_goal_result_timeout_verified=false` and
`strong_move_group_request_id_correlation=false` remain unchanged. Normal
plan-only results do not establish either claim. No hardware, remote system,
public push, merge or release is in this checkpoint.

## Subsequent bounded trials

- `rgbd_live_safety_20260906a`: 20 Hz production perception plus the existing
  SafetyMonitor, no motion gate/MoveGroup or motion requests. 39 tracked targets,
  six recorded validation facets pass. Input loss used SIGINT only on this run's
  perception process while the simulator clock remained live.
- `rgbd_control_trial_20260906a`: rejected before simulator startup because the
  material validator treated an empty converter-generated camera friction
  placeholder as an assigned material. A same-input implicit-material conversion
  proved the camera collision XML identical. The validator now permits exactly
  empty `friction/ode`; real non-pad coefficients/attributes remain rejected.
- `rgbd_control_trial_20260906b`: typed gripper preparation/release completed,
  but all three observer-baseline attempts left the bound frame beyond the
  unchanged 100 ms admission budget (recorded latter attempts: 112/114 ms).
  Correlated observer cancellation was confirmed before retries. The trial
  returned 6; no grasp-sequence arm command was sent. All MCAP/logs are retained.

The runtime client also now derives descend/lift from the exact bound visual
target rather than the configured scene center. A regression poisons the scene
center while shifting the observation and verifies that command geometry follows
only the latter. Independent scene geometry still validates the resulting route.

50 Hz qualified live safety in `rgbd_live_safety_50hz_20260906b` (47 targets, all
six facets pass). The preceding `...50hz_20260906a` retained one missing recorded
CameraInfo sample and did not pass complete source association. The capture
recorder now buffers raw assets for writing after observation and uses a deeper
receive queue; production perception freshness checks remain unchanged.

`rgbd_control_trial_20260906c` then passed baseline admission, started approach
and ended SAFE_STOP with `target_source_stale` (trial return 12). No pad contact,
lift or retention occurred. Its offline RGB-D replay found 241 spatially accepted
frames, 3 incomplete extents and 361 missing-visible-face refusals. The transition
image at 19.640 s shows the arm occluding the cube; the last recorded target was
at 19.620 s. Missing final recorded triads (16) remain disclosed. Offline replay
does not establish live freshness. The run's imported module paths/hashes and
all MCAP/logs are retained; exact-domain cleanup reports zero remaining processes.

Next: reposition the fixed simulated camera outside the arm occlusion path,
retain strict visibility refusal, and repeat camera qualification plus isolated
plan-only before another bounded trial. No timeout, source-age threshold,
baseline sample count, retry count or physical-success criterion has increased.

## Fixed opposite-table camera checkpoint

`opposite_table_edge` moves the existing fixed camera assembly to center
(0.5, 0.5, 0.5) m with pitch 33 degrees and yaw -125 degrees. Camera and pole
collision shapes are preserved; the base remains beyond the table Y edge.
The same project-owned overlay configures Gazebo, MoveGroup and the generated
material-preflight URDF. Arm joints, pads, scene and grasp profile are unchanged.
Use `EDGEGRASP_RGBD_CAMERA_VIEW=opposite_table_edge` for both planning and runtime;
recorded observations identify their view and runtime rejects a different plan view.

`rgbd_opposite_capture_20260906a` used 50 Hz and no motion requests. All six live
safety facets passed with 37 tracked targets. Nine spatial samples passed the
0.25 mm budget; the startup frame was refused and retained. The first accepted
observation was selected without consulting truth error.

`rgbd_opposite_plan_20260906a` used that observation and passed all three chained
segments. Trajectory publication, ExecuteTrajectory and FJT goal counts were zero.
The scoped cleanup reported zero remaining processes and Gazebo reported no
geometry creation failures. This is planning evidence only.

The camera overlay has two focused regressions covering unchanged arm/collision
geometry, the declared fixed pose, and rejection of unknown or incomplete inputs.
Windows checks: 340 passed, 3 skipped; ROS observation/health/geometry checks: 13
passed. Physical manipulation with this viewpoint is being qualified separately.

`rgbd_opposite_trial_20260906a` admitted its first observer baseline (93 ms target
age) and recorded successful arm-controller terminals for approach and descend.
The client's unchanged 90 s result wait expired before close dispatch; it requested
cancel, observed the sequence terminal, and completed typed zero-effort release.
The trial returned 9. Independent physics recorded no pad contact and zero lift.
Offline replay accepted all 885 complete image triads, with 2 unmatched final
triads; 883 target publications were recorded. This supports visibility during
these two motion stages, not successful manipulation or complete live freshness.
Exact-domain cleanup reported zero remaining processes.

Reproduction uses the same plan/runtime commands above with
`EDGEGRASP_RGBD_CAMERA_VIEW=opposite_table_edge`, the observation in
`rgbd_opposite_capture_20260906a/first_accepted_estimate.json`, and
`EDGEGRASP_RGBD_PLAN_RESULT` pointing to
`rgbd_opposite_plan_20260906a/plan_only_result.json`. Use fresh external directories.
The new camera capture command adds `--camera-view opposite_table_edge --camera-hz 50`.

Current next step: inspect simulator throughput and the client's wall-time wait.
The successful approach and descend span 10.76 simulated seconds; this run does
not justify increasing safety thresholds or claiming the remaining close/lift
stages. See `observations/2026-09-06-rgbd-opposite-view-runtime.json` for hashes and
correlated terminals. The two other predeclared positions remain untested.

## Rendering throughput and 30 Hz qualification

The current Ogre log identifies Mesa llvmpipe software rendering. Source/receiver
clock observations from bounded camera captures measured simulated/wall time
ratios of 0.361 at 20 Hz, 0.166 at the original-view 50 Hz qualification and 0.136
at the opposite-view 50 Hz qualification. These are observed capture intervals,
not a controlled hardware benchmark. The trial client's result wait uses
`time.monotonic`; controller trajectories use ROS simulation time.

`rgbd_opposite_30hz_capture_20260906a` measured 0.208 but retained missing RGB/depth/
CameraInfo recorder entries, so its source-association facet failed. JSONL rows
are now buffered during this bounded capture and written after observation, like
the already-buffered raw arrays. No production-publisher behavior changed.
`rgbd_opposite_30hz_capture_20260906b` passed all six facets with 56 targets.
Runtime now accepts `EDGEGRASP_RGBD_CAMERA_HZ=20|30|50` (default 50) and records the
chosen rate in its scope log. Camera geometry, trajectory scaling, 90 s client
wait, 100 ms source admission, baseline sample count and physical criteria remain
unchanged. The existing same-view plan-only qualifies the unchanged geometry.


The 30 Hz trial `rgbd_opposite_30hz_trial_20260906a` returned 12. It completed
close and entered lift, then refused an observation at 30.823 s because TF was
available only through 30.821 s. The physics observer reported its wall timeout
and no grasp success; the separate whole-bag timeline measured only 0.014 mm
peak cube lift. Typed release succeeded and scoped cleanup left zero processes.

The opt-in lift health check now selects the newest complete image/TF pair
inside the existing 200 ms sequence-source budget and after the lift binding source.
The publisher separately requires its estimate to be no older than 100 ms.
It preserves that pair's timestamps and exact-time TF. It never selects based on
residual quality: a newer complete pair showing 6 mm slip still rejects, even
when an older pair would pass. Wrong identity and no fresh matching TF refuse.
The default static motion policy is unchanged. ROS checks pass (13), as do
Windows checks (340 passed, 3 skipped) and replay checks. Runtime qualification
of this correction remains open.

A process-only `LP_NUM_THREADS=2` camera capture passed live safety but measured
0.211 simulated/wall ratio versus 0.209 for the default-thread 30 Hz capture.
This is not meaningful evidence of a speedup, so the runtime retains its default
rendering thread selection. No system setting changed. The parameter is documented
by [Mesa](https://docs.mesa3d.org/envvars.html#envvar-LP_NUM_THREADS).

## Bounded timing qualification

Correction: the observer's own wall bound is twice its 30 s observation window,
therefore 60 s. The client's 100 s physics-result wait is a separate limit and
never extends the observer. Neither limit has changed.

`rgbd_320_capture_20260906a` retained the same field of view and camera pose with
320x180 images. All six live checks and nine valid spatial samples passed, but
its observed simulated/wall ratio was only 0.221 versus 0.209 at 424x240. This
modest difference is insufficient to qualify a full cycle; runtime keeps the
upstream resolution. The optional resolution overlay and its geometry-preserving
regression remain available for reproducing this bounded investigation.

A velocity scaling of 0.15 was declared before execution; acceleration remains
0.1. This is a distinct motion qualification, not a repeat of the original 0.1
baseline. `rgbd_v015_plan_20260906a` passed the unchanged whole-trajectory collision
and 1 rad/s joint-velocity validation for all three segments, with zero motion
goals or publication. Segment durations were 5.0844, 2.7250 and 1.7483 simulated
seconds. Scoped cleanup reported zero processes. Runtime rejects a plan whose
recorded scaling differs from `EDGEGRASP_RGBD_VELOCITY_SCALING` (0.1 default;
0.15 is the only additional declared value). Freshness, gripper profile and
independent physics criteria are unchanged.


`rgbd_v015_trial_20260906a` used upstream image resolution, opposite-table view,
30 Hz, velocity scaling 0.15 and acceleration scaling 0.1. It passed baseline,
approach, descend and close, then entered healthy LIFT_EXEC without the former
TF extrapolation refusal. It stopped with `target_source_stale` (trial return 12).
The physics observer remained live and reported `sequence_not_completed`, with
386 simultaneous dual-pad samples and 1.97 mm peak lift before its terminal.
The whole-bag evaluator measured 2.15 mm peak lift. Neither meets the 20 mm rule.
Typed release was denied by the latched gate; the existing scoped fallback
confirmed gripper-controller deactivation and unclaimed effort before cleanup.
Zero processes remained in the selected domain.

Offline replay retained 564 accepted image triads, 162 surface-inconsistency
refusals and two incomplete final triads. The first refusal was at 29.734 s.
The cube remained visibly exposed. Separate truth-only evaluation at 29.730 and
29.740 s showed 1.31 and 1.43 degrees of tilt, supporting a mismatch with the
fixed-orientation model. These poses are diagnostic evidence and are not fed to
perception. Next: estimate bounded small orientation changes from the image/depth
planes, test synthetic rotations and replay this entire bag against independent
truth. Preserve the 0.5 mm surface-residual and 100 ms freshness limits.


## Bounded RGB-D orientation fitting (not yet motion-qualified)

The opt-in `max_rotation_deg=5.0` mode estimates three face normals from adjacent
measured depth points, fits their planes and orthogonalizes the resulting axes.
It rejects rotation beyond five degrees around the declared initial orientation.
The final 0.5 mm surface residual, visible-face counts, extent, depth and 100 ms
source gates remain. Default mode remains fixed orientation (0.0). Synthetic
-3/+2 degree ray-box cases preserve the 0.25 mm center budget; a 10 degree case
rejects. ROS/geometry checks: 15 pass.

Replay `rgbd_v015_trial_20260906a/offline_rgbd_rotation5` accepted all 726 complete
triads; two final incomplete triads remain disclosed. Independent nearest-truth
comparison passed 725/726: the outlier at 35.047 s was compared to truth at
35.050 s during post-release fall and had 0.462 mm discrepancy. Interpolated
bracketing truth yields at most 0.0533 mm error for covered samples; one boundary
sample lacks interpolation. Interpolation is supporting evidence, not a direct
same-time truth measurement. Both reports are retained. Reproduce with
`scripts/analyze_rgbd_mcap.py ... --max-rotation-deg 5` and
`scripts/evaluate_rgbd_replay.py RUN_DIR ESTIMATE_JSONL NEW_REPORT`.

The no-motion production test `rgbd_rotation5_capture_20260906a` passed source
association, identity and observed freshness, but its permission-expiry receive
check failed: true arrived at last target source +206 ms, then false at +207 ms.
The unstamped Bool does not distinguish issuance time from receiver delay. Do not
silently increase the threshold or declare this full live qualification passed.
Next: add issuance-time diagnostic evidence and repeat the loss qualification
before a bounded trial with `EDGEGRASP_RGBD_ROTATION_BOUND_DEG=5.0`. No motion used
this mode yet. See `observations/2026-09-06-rgbd-bounded-rotation-qualification.json`.


## Permission issuance and five-degree runtime boundary

`/edgegrasp/permission_evidence` now records decision time, publication interval,
source target time and a contiguous sequence number. The Bool permission and its
safety decision are unchanged; diagnostics do not authorize motion. MCAP and the
read-only capture record this topic. `validate_permission_timing.py CAPTURE_DIR`
keeps issuance checks separate from reception checks.

`rgbd_rotation5_permission_20260906a` passed all six live checks and all four
issuance checks (261 diagnostic rows, 65 true). Eight SafetyMonitor ROS tests pass.
Cross-process ROS clock samples differ, so the receiver-minus-publisher values
are not a direct network-latency measurement. These diagnostics cannot reconstruct
the previous run's unstamped true-at-206-ms message; that failed result is retained.

`rgbd_rotation5_trial_20260906a` used the matched 0.15/0.1 plan and the opt-in
five-degree estimator. The observer recorded 457 simultaneous pad samples and
4.63 mm peak lift before SAFE_STOP; whole-bag peak was 4.89 mm. First offline
orientation refusal was at 34.849 s. Separate truth at 34.840/34.850 s measured
5.30/5.58 degrees of tilt. Replay retained 623 accepted triads, four orientation-
bound refusals, 169 missing-orientation-face refusals, and seven incomplete final
triads. The latter face refusal also constrains normal association near the
initial orientation; it must not automatically be called visual occlusion.
Controller deactivation and unclaimed effort were confirmed by the existing
release fallback, followed by zero scoped processes.

Next: qualify a larger depth-derived orientation range with independent synthetic
and recorded evaluation before choosing it for runtime. Surface residual, source
freshness, target consistency and 20 mm/0.5 s physical criteria remain unchanged.

## Twenty-degree orientation qualification

The 20-degree option first groups local depth normals by nominal face, then
retains the dominant direction within two degrees before plane fitting. This
removes mixed crease normals without increasing the final surface-residual limit.
The first broad-association replay remains in `offline_rgbd_rotation20`: it
accepted 700 triads and rejected 96, including previously usable stationary frames.
The corrected `offline_rgbd_rotation20b` accepted all 796 complete triads, with
seven incomplete final triads retained. Nearest-truth qualification passed
791/796; bracketing interpolation covered all but seven and had at most 0.087 mm
error. The nearest-sample failures and missing truth coverage are not hidden.
Synthetic -12/+15 degree cases pass the 0.25 mm center budget and a 30-degree case
rejects. Combined ROS/geometry tests: 17 pass.

`rgbd_rotation20_capture_20260906a` passed the six live and four permission-issuance
checks, with 36 tracked targets and 257 continuous permission diagnostic rows.
`EDGEGRASP_RGBD_ROTATION_BOUND_DEG=20.0` is opt-in; the default remains 0.0. Camera,
source age, residual, contact criteria and matched 0.15/0.1 motion settings are
unchanged. This is a bounded extension for the cube's motion during pickup, not
an arbitrary-yaw or general-object perception claim.


`rgbd_rotation20_trial_20260906a` retained 561 simultaneous pad samples and
5.99 mm peak lift before the sequence terminal; whole-bag peak was 6.14 mm.
The sequence stopped on `target_geometry_drift:0.0052997063615872145`, not missing
observations. Recorded same-time TF and image centers independently reproduce
that value at 34.750 s, relative to the binding sample at 33.991 s. The cube's
horizontal displacement was 1.24 mm and vertical rise 5.20 mm at that source time.
The first bilateral contact at 34.026 s was after the binding image but before
the correlated close terminal/lift dispatch at 34.057 s. These facts do not prove
rigid attachment, and do not by themselves distinguish rotation about a contact
from sliding. Typed release succeeded; scoped cleanup reported zero processes.

Next: evaluate a task-appropriate observed-object motion constraint against the
explicit planned lift region, preserving spatial margins and independent physics
success. Do not silently increase the existing rigid-residual threshold or infer
grasp success from a point staying inside a permitted region.


## Planned lift region qualification

The rigid-attachment residual is stricter than a bounded object-workspace
check. The rotation20 trial reproduced a 5.30 mm rigid residual while its image
center moved only 1.24 mm horizontally and 5.20 mm upward. This does not prove
whether the object slid or rolled between the pads.

An additional explicit `planned_lift_region` mode is limited to `ros_sim` and
`LIFT_EXEC`. It measures Euclidean distance to the segment from the immutable
initial visual center to that center plus the requested lift displacement.
Only a vertical displacement greater than zero and at most 40 mm is accepted.
The existing 5 mm limit, task/target/epoch binding, source freshness, pre-lift
static guard and independent physics criteria remain in force. It uses the
latest source observation, with no restamping or selection by geometry.

This check permits axial lag, a stationary cube, or a drop that remains in the
region. It is solely a workspace constraint: contact, 20 mm lift and 0.5 s
retention must still be proven independently. It neither detects every loss of
attachment nor turns a completed command sequence into a successful grasp.
Defaults remain `static` in the node and `rigid_lift` in the opt-in RGB-D trial
harness. Select this additional trial with
`EDGEGRASP_RGBD_MOTION_MODE=planned_lift_region`; the harness records the mode
in `start.txt` and still requires its matching plan-only result.

Qualification before its first motion trial: Windows `scripts/check.ps1`
passed 341 tests (3 explicit skips), structural validation and three scenarios
with 100 deterministic replays each. `bash scripts/check_rgbd.sh` passed 19
checks including source identity, pre-binding observations, 6 mm lateral drift,
6 mm below-start and beyond-end rejection. Ruff and shell syntax passed.


Its first control execution, `rgbd_region_trial_20260906a`, retained 413
same-sample bilateral contacts in the independent observer window. That
observer hit its 60 s wall timeout at 3.19 mm peak lift; its window must not be
conflated with the later whole-bag maximum of 10.61 mm. The sequence stopped on
`target_source_stale`, not a region-distance rejection. At source 30.658 s the
estimator first refused rotation outside its 20-degree declaration; nearby
independent truth measured 20.37 degrees at 30.650 s and 21.09 at 30.660 s.
Normal association then refused missing orientation faces from 30.724 s.

Offline replay retained 490 accepted estimates, two orientation-bound refusals,
253 missing-orientation-face refusals and two final incomplete triads. All 490
accepted estimates passed nearest-truth evaluation (maximum 0.106 mm, within
the unchanged 0.25 mm budget). Supplementary interpolation had one unavailable
boundary and maximum 0.010 mm error among matched samples.

Typed release returned 1; the existing fallback confirmed controller deactivation
and unclaimed effort with exit 0. Scoped cleanup recorded `REMAINING_COUNT=0`.
This is a retained failure. The next investigation must address the actual
post-contact orientation/visibility and observer timing; it must not claim that
increasing a rotation threshold alone qualifies perception or physical success.


## Rotation40 and fixed-pad geometry investigation

`observations/2026-09-06-rgbd-rotation40-and-pad-geometry.json` records the next
qualification, with no new motion execution. A fresh read of the historical
Candidate024 success MCAP showed rotation 35.80 degrees and horizontal motion
5.96 mm at its first 20 mm lift. These are historical physics facts, not RGB-D
qualification. They explain why the previous 20-degree sensor constraint and
5 mm fixed vertical segment cannot cover that known physical motion.

The additional 40-degree estimator mode retains three depth-supported faces,
0.5 mm surface residual and full-extent checks. It recovered all 745 complete
RGB-D triads from the region trial; all passed independent nearest-truth spatial
evaluation, maximum 0.106 mm. Two incomplete final triads remain unavailable.
The new 30 Hz no-motion capture passed all six live safety facets and all four
permission issuance facets (34 observed targets). The mode is available for
capture and offline replay; the execution harness has not enabled it yet.

`CubeEstimate.orientation_rows` now exposes the measured proper rotation matrix
for offline geometric inspection. The synthetic +/-30 degree cases verify
center accuracy and orientation error below 0.001 rad. An initial elementwise
1e-5 assertion was too strict for the retained plane-fit method; the +30 degree
case differed by about 0.0002 per matrix element. The test now states an angular
accuracy bound rather than implying exact synthetic reconstruction.

`evaluate_rgbd_pad_consistency.py` compares this measured cuboid to the fixed
pad primitive using the exact image-source TF and normalized SAT axes. Across
the evaluated close/lift interval there were no missing TF pairs, and the
largest separating-axis gap was 0.00761 mm. This uses no object truth in its
geometry calculation. It is not force/contact proof, does not model the moving
pad at its actual joint position, and has not replaced any live safety guard.
A future live guard needs atomic source/identity pairing of the measured cube
orientation and center, exact-source gripper TF, and rejection tests before
another motion trial. Independent physics criteria remain unchanged.

Example offline command (source the existing Jazzy overlay and current source
PYTHONPATH as in `check_rgbd.sh`):

```bash
python3 scripts/evaluate_rgbd_pad_consistency.py \
  /home/edgegrasp/ros2_ws/test_results/rgbd_region_trial_20260906a/mcap \
  /home/edgegrasp/ros2_ws/test_results/rgbd_region_trial_20260906a/offline_rgbd_rotation40_pose/offline_estimates.jsonl \
  ros_ws/src/edgegrasp_ros/config/so101_grasp_geometry_candidate024_face_aligned_q0p40.json \
  /home/edgegrasp/ros2_ws/test_results/NEW_GEOMETRY_REPORT.json \
  --from-ns 29834000000 --to-ns 30850000000
```


## Live measured-pad mode and retained lock-order failure

The explicit `measured_pad` mode consumes `/edgegrasp/measured_cube` as one
JSON message containing schema version, target/frame identity, source stamp,
clock domain/epoch, center and measured orientation matrix. It rejects malformed
or reflected/nonorthogonal rotations and conflicting content at the same source.
The orientation and center are not joined across independent topic callbacks.

During `LIFT_EXEC` it chooses the newest fresh observation with TF at that exact
source time and compares the measured cube OBB with the profile's fixed pad.
The permitted normalized SAT separation is 5 mm. This replaces the rigid-center
or vertical-segment assumption only in this explicit mode. It is geometric
proximity, not force, bilateral contact or grasp success. Pre-lift static checks,
source expiry, identity checks and independent physics criteria remain active.
The profile is supplied through `observed_cube_geometry_profile`; defaults remain
static and the other opt-in models remain available for reproduction.

For slow RGB-D rendering, the harness explicitly sets the read-only observer's
`observation_wall_factor=6.0`: its unchanged 30 s simulation window is bounded by
180 s wall time instead of the default 60 s. A live parameter query confirmed
6.0. The action wait remains 90 s and freshness/lift/retention thresholds remain
unchanged. Three observer regressions covered successful retention, a single pad
and table-only contact without relaxing any success condition.

`rgbd_measured_pad_trial_20260906a` is a retained failure: the sequence client hit
its 90 s result limit during approach, cube peak lift was zero, and scoped
cleanup recorded zero remaining processes. The newly added callback acquired
state then core-event locks, conflicting with the existing event-to-state order.
The corrected callback follows event then state. A no-motion lock-order checker
fails with `lock order inversion` when the old order is reintroduced in memory
and passes with the corrected implementation. No real process-interruption
experiment was used. The full failed bag and `old_lock_order_regression.log`
remain in the external artifact directory.

The corrected source passed 23 RGB-D/ROS checks. Windows validation passed 342
tests (3 skips), project structure and three sets of 100 deterministic replays.

The two declared translation inputs were generated by
`prepare_rgbd_static_cases.py` into
`/home/edgegrasp/ros2_ws/test_results/rgbd_declared_positions_20260906a`.
Its manifest and local install receipt hash all six new files; no existing
configuration was overwritten. Capture selects them with `--position x_minus_1mm`
or `--position x_plus_1mm`. Generated inputs are not planning or execution evidence.


## First measured-pad control success

After the lock-order fix, `rgbd_measured_pad_trial_20260906b` completed all four
commands. The independent observer returned `contact_lift_retention_verified`,
with 1612 same-sample bilateral contact observations and retained height
28.8468 mm. This upgrades only the control position's local simulation physics
claim. The hardware and real MoveGroup timeout/correlation flags remain false.

The recorded source audit covers 387 observations from the bound target source
through the observer's final simultaneous contact: all have exact recorded RGB,
depth and calibration timestamps, with matching tracked centers. The full bag
contains 641 atomic observations. Nearest-truth spatial evaluation passed 639;
the failures at 34.585 s (close) and 39.634 s (after sequence completion/release)
measured 0.308 and 0.654 mm. All supplementary interpolated comparisons were
available, with maximum 0.0554 mm error. These interpolated results must not be
reported as exact-same-time direct truth or used to erase the nearest failures.

Typed release returned 1. The existing fallback confirmed controller deactivation
and unclaimed effort; scoped cleanup recorded zero remaining processes. The
wrapper's final nonzero return must not erase the separately recorded trial
success (trial exit 0) or be described as a clean typed-release success.

Reproduce a fresh control trial with the matching recorded-target plan:

```bash
export EDGEGRASP_RGBD_TRIAL=1
export EDGEGRASP_RGBD_ROTATION_BOUND_DEG=40.0
export EDGEGRASP_RGBD_CAMERA_HZ=30
export EDGEGRASP_RGBD_CAMERA_VIEW=opposite_table_edge
export EDGEGRASP_RGBD_VELOCITY_SCALING=0.15
export EDGEGRASP_RGBD_MOTION_MODE=measured_pad
export EDGEGRASP_RGBD_PLAN_RESULT=/home/edgegrasp/ros2_ws/test_results/rgbd_v015_plan_20260906a/plan_only_result.json
bash scripts/run_candidate005_contact_quality.sh 170 \
  /home/edgegrasp/ros2_ws/test_results/NEW_CONTROL_RUN NEW_CONTROL_TASK \
  so101_grasp_geometry_candidate024_face_aligned_q0p40.json 0.40 true \
  candidate012_control_mu1p0 0.0 effort_pid_preload -0.02 \
  so101_side_grasp_candidate024_face_aligned.json \
  scene_candidate024_face_aligned.json table_cube_candidate024_face_aligned.sdf
```

The measured cube parser additionally refuses source timestamps and clock epochs
outside their ROS message integer ranges. This prevents malformed atomic input
from reaching ROS field assignment with an out-of-range value.


## Translated-position qualification and yaw derivation

The declared X-minus and X-plus 1 mm captures each passed all six live safety
facets and all four issuance facets. Each retained ten static samples: nine
qualified live samples passed the 0.25 mm spatial budget, with the startup
rejection retained. Their original fixed-grasp-orientation plans both reached
`NO_IK_SOLUTION` on approach-to-descend, after a validated approach. Neither
published trajectories nor sent ExecuteTrajectory/FJT goals. Both domains were
cleaned. These are fixed-pose IK failures, not proof that every orientation at
those target positions is unreachable.

`derive_rgbd_yaw_candidate.py` constructs an additional, explicitly bounded
candidate at each same target position. The generated URDF locates shoulder pan
at (0.0388353, -8.97657e-9, 0.0624) m. The qualified baseline descend IK places
the pitch-axis plane normal at approximately (-0.60519, 0.79608, 0). Keeping the
arm's lateral coordinate requires
`n dot Rz(-delta) (observed_center - shoulder_pivot)` to equal its baseline
value. The script chooses the analytic root nearest zero, refuses more than
0.5 degree, and left-multiplies the grasp orientation by that world-Z rotation.
The approach pose, cube position/yaw and grasp geometry profile remain fixed.
This is a kinematic hypothesis; collision and complete plan checks still decide
whether execution is allowed. It is not an IK-success or physics claim by itself.

The measured centers produce +0.145323 degree for X-minus and -0.144329 degree
for X-plus. No failed position was replaced and no orientation sweep was run.
The original failed candidates and new derived candidates are stored separately.


## Translated execution and service discovery

The derived X-plus candidate completed its sequence and independently verified
contact/lift/retention, retaining approximately 28.68 mm lift. Its 567 recorded
atomic estimates all passed nearest-truth spatial evaluation (maximum 0.1733 mm);
all 387 active-interval sources matched RGB, depth, calibration and tracked centers.

The first X-minus execution (`rgbd_x_minus_trial_20260906a`) reached descend but
refused gripper dispatch with `execute_trajectory_unavailable`. It had no finger
contact or cube lift. No service-process exit was found, and the endpoint was
listed in the final ROS graph; the exact cause of the transient readiness failure
is not proven. This failed run and its scoped cleanup are retained.

The wrapper now permits up to a 50 ms discovery wait, querying at intervals no
longer than 5 ms. This sends no action goal and does not retry a dispatched goal.
After a wait it rechecks the immutable command's source age and epoch before
sending. The local rclpy generic wait uses 250 ms sleeps, so it is deliberately
not used for this short budget. Tests verify absent-service timeout, prompt
availability, and refusal when the source expires during the wait. Normal OS
scheduling is not a hard real-time guarantee; actual waits are logged.
