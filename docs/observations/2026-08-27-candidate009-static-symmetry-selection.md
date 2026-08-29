# Candidate009 static symmetry selection

Candidate008 improved transient contact and lift, but the fixed/moving pad
contact counts were 966/195 and the cube finished about 16.57 mm from its
initial XY location. That is enough evidence to avoid another blind close-angle
change.

The dependency-free selector scanned 61 tool-frame X offsets from -3 mm to
+3 mm in 0.1 mm steps. Its objective was to minimize the difference between
the cube's two exact, normalized SAT overlap margins while keeping both above
the existing 1 mm static gate. It selected `-0.9 mm`:

| Static quantity | Candidate008 | Candidate009 selection |
|---|---:|---:|
| Fixed-pad minimum overlap | 11.083 mm | 10.184 mm |
| Moving-pad minimum overlap | 9.412 mm | 10.201 mm |
| Absolute difference | 1.671 mm | 0.0166 mm |
| Close command | 0.50 rad | 0.50 rad |

The resulting world-space descend/lift shift has norm 0.9 mm. The routed
approach, orientation, planner, scaling, scene, selective ACM, and close command
remain unchanged.

This is static primitive geometry selection only. It has not yet passed live
MoveIt planning, Gazebo contact symmetry, lift, retention, or hardware gates.
The machine-readable selection and exact next gates are in
[`2026-08-27-candidate009-static-symmetry-selection.json`](2026-08-27-candidate009-static-symmetry-selection.json).
