# Environment audit

Current audit snapshot: 2026-08-26T18:03:41.8340358+08:00, Asia/Shanghai.
Inventory probes are read-only and omit credential values. Host repair and
Ubuntu installation happened earlier in this task only after user authorization;
the audit script itself never repairs, installs, restarts, or elevates.

## Windows host

| Area | Observed state |
| --- | --- |
| OS | Windows 11 Pro, 10.0.26200, x64 |
| CPU/virtualization | AMD Ryzen 5 PRO 4650G; firmware virtualization and Hyper-V/VBS observed |
| Graphics | AMD Radeon Graphics 31.0.21921.1000 reports OK; Virtual Desktop Monitor reports Error |
| Python used by project | CPython 3.10.11 project `.venv`, pytest 9.1.1 |
| Native commands | Python and Docker CLI present; Windows-side CMake, colcon, ROS 2, Gazebo/`gz`, and RViz2 absent |
| Free space at snapshot | C: 123,923,324,928 bytes; D: 2,821,170,094,080 bytes; E: 86,995,509,248 bytes; F: 370,266,152,960 bytes; H: 9,162,356,146,176 bytes |

Docker CLI presence was not upgraded to daemon/runtime evidence. Windows-native
robot tools remain absent; all ROS runtime evidence below belongs to the
dedicated WSL guest.

## WSL host metadata and guest metadata

Exact command:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass \
  -File scripts\audit_environment.ps1 -IncludeWsl \
  -WslDistribution Ubuntu-24.04 -WslTimeoutSeconds 30
```

Observed host metadata:

- `wsl.exe --status`: PASS;
- `wsl.exe --version`: PASS — WSL 2.7.12.0, kernel 6.18.33.2-2,
  WSLg 1.0.73.2, Windows 10.0.26200.8457;
- `wsl.exe --list --verbose` and `--list --quiet`: PASS;
- registered distributions: `Ubuntu-22.04`, `Ubuntu-24.04`,
  `docker-desktop`, and `docker-desktop-data`;
- default distribution remains `Ubuntu-22.04`; the audit safely selected the
  exact requested `Ubuntu-24.04` instead of changing the default.

The bounded guest probe
`wsl.exe -d Ubuntu-24.04 -- sh -lc 'cat /etc/os-release; uname -a'` exited 0
and reported Ubuntu 24.04.4 LTS Noble, x86_64, WSL2 kernel
6.18.33.2-microsoft-standard-WSL2. WSL emitted a localhost/NAT warning on
startup; it did not prevent the probe, build, tests, or scoped simulation runs.

This separates two evidence classes:

- host WSL package/version/registration metadata: PASS;
- selected guest release/kernel/architecture metadata: PASS.

WSLg package presence is observed, and Gazebo processes ran, but no separate
visual desktop/OpenGL conformance test was recorded. Do not infer production
GPU or GUI performance from WSLg version alone.

## Authorized repair history

The first WSL start entered Windows Installer repair and failed while writing
the exact shell-registration key
`HKLM\SOFTWARE\Classes\Directory\Background\shell\WSL`. The existing
`Ubuntu-22.04` distribution was never unregistered, deleted, or reset.

After authorization, the repair was narrowly scoped: an ACL backup was saved
under
`C:\Users\huang\AppData\Local\Temp\edgegrasp-wsl-repair-20260826-155446`,
and `SYSTEM` FullControl was added only to that exact WSL key and its `command`
child. No BIOS change, SFC/DISM, GPU-driver change, or broad registry rewrite
was performed. A separate Ubuntu 24.04 distribution/user (`edgegrasp`) was then
installed for this project; Ubuntu 22.04 remains registered and default.

Future audit runs must not repeat or widen this repair automatically. Official
host operations such as `wsl --update`, `wsl --shutdown`, distro installation,
optional Windows features, SFC/DISM, driver work, and reboot remain explicit
user/admin boundaries.

## Ubuntu 24.04 / ROS inventory

The dedicated guest now has ROS 2 Jazzy, Gazebo Harmonic, ros-gz,
ros2_control/controllers, `gz_ros2_control`, MoveIt 2, MCAP storage, rosdep,
colcon, pytest, venv support, `check_urdf`, and project dependencies. Key
observations include:

- guest Python 3.12.3 and pytest 7.4.4;
- `gz sim --versions`: 8.11.0;
- `ros2 pkg prefix` succeeded for `ros_gz_sim`, `ros_gz_bridge`,
  `controller_manager`, `gz_ros2_control`, `moveit_ros_move_group`, and
  `moveit_setup_assistant`;
- `/home/edgegrasp/ros2_ws` discovers 11 packages and the complete build passes.

Package inventory is not SO-101 runtime proof. The separate validation report
records controller, topic, action, planning, gated-execution, camera, and MCAP
observations.

Clean rosdep reproduced unresolved metadata keys `ament_python` and
`moveit_planners_pilz_industrial_motion_planner`. After verifying the relevant
installed packages, the disposable workspace used the explicitly bounded
`--skip-keys` documented in the runbook. This remains an upstream metadata
risk, not evidence that the runtime packages are absent.

## Audit-script behavior

Without `-IncludeWsl`, `scripts/audit_environment.ps1` checks only the Windows
host. With it, the script runs bounded exact-argument calls for WSL status,
version, verbose/quiet distro lists, and an optional exact distro guest probe.
`-WslDistribution` is matched against parsed registered names and passed as a
separate argument; it is never concatenated into a command string. Failure,
timeout, or an unregistered requested name is recorded as `UNVERIFIED` with
the command/exit evidence.

The Linux audit was also run inside the selected guest after sourcing Jazzy:

```bash
bash scripts/audit_environment.sh
```

It reported Python, CMake, colcon, `ros2`, `gz`, and RViz2 on PATH; the legacy
`gazebo` executable was absent, as expected for the Harmonic `gz` path. Guest
disk availability was about 948 GiB, `DISPLAY=:0`, and
`WAYLAND_DISPLAY=wayland-0`. These variables prove environment wiring, not a
visual rendering benchmark. Separately, Git Bash syntax validation remains
only a Windows static check. The full Ubuntu build/run procedure is in
[ubuntu-jazzy-runbook.md](ubuntu-jazzy-runbook.md).
