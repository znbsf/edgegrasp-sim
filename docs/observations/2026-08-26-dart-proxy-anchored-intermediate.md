# DART proxy anchored intermediate observations

- Observation time: 2026-08-26 21:23–21:31 Asia/Shanghai
- Gazebo log: `/home/edgegrasp/.gz/sim/log/2026-08-26T21:23:51.983589943/server_console.log`
- Engine: Gazebo Sim 8.11.0 / gz-physics 7.6.0 / DART 6.13.2
- Status: **ANCHOR_PASS; SPECIFIC_BASE_PROXY_UNRESOLVED in this snapshot**

The generated robot description had one `world` root and a fixed
`world_joint` from `world` to `base_link`. Runtime observations showed:

- SO-101 pose remained exactly XYZ `[0, 0, 0]`, RPY `[0, 0, 0]` across a
  three-second interval.
- `/tf_static` had one publisher, `robot_state_publisher`; `world -> base_link`
  was the identity transform.
- All three controllers were active, six expected joint names were published,
  and both expected FJT actions existed.
- `/edgegrasp/table_contacts` published a contact pair between
  `table::table_top::collision` and `target_cube::cube_link::collision`.
- The DART mesh-construction diagnostic count and associated
  `geometry ... couldn't be created` count were both zero in the new log.

An intermediate probe at `x=0.0263353 m` was retained by robot geometry rather
than reaching the ground, but settled above the intended base surface (separate
observations produced `z ~= 0.1084 m` and `z ~= 0.123399 m`). Static zero-joint
AABB analysis showed that this XY overlaps a conservative shoulder proxy whose
lower X bound is about `0.02613 m`. Therefore this is classified only as:

```text
ROBOT_PROXY_CONTACT_OBSERVED
SPECIFIC_BASE_PROXY_UNRESOLVED
```

It is not a base-proxy pass. The replacement probe uses `x=0`, which remains
inside the broad base proxy and outside the shoulder proxy, parses settled and
retention Z with explicit tolerances, and requires the contact message to name
`so101::base_link::edgegrasp_proxy_base_link_collision_1`. That revised test
requires a fresh world because dynamically spawned contact-sensor discovery
also depends on `UserCommands` running before the `Contact` system.
