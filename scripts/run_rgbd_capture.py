#!/usr/bin/env python3
"""Run one bounded local Gazebo camera-only observation session under ROS Jazzy.

Source the existing Jazzy/EdgeGrasp overlay before invocation. No motion node,
observer, target publisher or MoveGroup is launched. Raw evidence stays external.
"""

import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def scoped_pids(domain, partition):
    matches = []
    for proc in Path("/proc").glob("[0-9]*"):
        try:
            environment = (proc / "environ").read_bytes().split(b"\0")
            if (f"ROS_DOMAIN_ID={domain}".encode() in environment
                    and f"GZ_PARTITION={partition}".encode() in environment):
                matches.append(int(proc.name))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return matches


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--domain", type=int, default=166)
    parser.add_argument("--camera-hz", type=int, choices=(20, 30, 50), default=20)
    parser.add_argument("--camera-view", choices=("upstream", "opposite_table_edge"), default="upstream")
    parser.add_argument("--camera-resolution", choices=("upstream", "320x180"), default="upstream")
    parser.add_argument("--live-safety-test", action="store_true",
                        help="publish observations and test input loss; no motion gate or requests")
    parser.add_argument("--max-rotation-deg", type=float, choices=(0.0, 5.0, 20.0, 40.0), default=0.0)
    parser.add_argument("--position", choices=("control", "x_minus_1mm", "x_plus_1mm"), default="control")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    if output.is_relative_to(root) or not 1 <= args.domain <= 232:
        parser.error("use external artifacts and a dedicated domain in [1, 232]")
    output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1",
               GZ_PARTITION="edgegrasp_rgbd_" + output.name,
               PYTHONPATH=os.pathsep.join((str(root / "src"),
                   str(root / "ros_ws/src/edgegrasp_ros"), env.get("PYTHONPATH", ""))))
    scene_name = ("scene_candidate024_face_aligned.json" if args.position == "control"
                  else f"scene_rgbd_{args.position}.json")
    world_name = ("table_cube_candidate024_face_aligned.sdf" if args.position == "control"
                  else f"table_cube_rgbd_{args.position}.sdf")
    command = [
        "ros2", "launch", str(root / "ros_ws/src/edgegrasp_ros/launch/edgegrasp_proxy_gazebo.launch.py"),
        "use_camera:=true", "launch_edgegrasp_nodes:=false",
        f"camera_update_rate_hz:={args.camera_hz}.0",
        f"camera_view:={args.camera_view}",
        f"camera_resolution:={args.camera_resolution}",
        "launch_physics_observer:=false",
        f"scene_config_filename:={scene_name}",
        f"world_filename:={world_name}",
    ]
    with ExitStack() as stack:
        log = stack.enter_context((output / "gazebo.log").open("w"))
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        helpers = []
        (output / "scope.json").write_text(json.dumps({
            "launch_pid": process.pid, "ros_domain_id": args.domain,
            "gz_partition": env["GZ_PARTITION"], "command": command,
            "perception_max_rotation_deg": args.max_rotation_deg, "position_id": args.position,
            "render_process_environment": {"LP_NUM_THREADS": env.get("LP_NUM_THREADS")},
            "claim": "read_only_rgbd_observation_no_motion_requests",
        }, indent=2) + "\n")
        try:
            if args.live_safety_test:
                for module, params in [
                    ("rgbd_target_publisher", ["-p", "yaw_rad:=0.6500077341171558", "-p", f"max_rotation_deg:={args.max_rotation_deg}"]),
                    ("safety_monitor", ["-p", "clock_domain:=ros_sim"]),
                ]:
                    helper_log = stack.enter_context((output / (module + ".log")).open("w"))
                    helpers.append(subprocess.Popen([
                        "python3", "-m", "edgegrasp_ros." + module,
                        "--ros-args", "-p", "use_sim_time:=true", *params,
                    ], env=env, stdout=helper_log, stderr=subprocess.STDOUT))
            capture_command = [
                "python3", str(root / "scripts/probe_rgbd_observation.py"),
                str(output / "camera"), "--yaw-rad", "0.6500077341171558",
            ]
            if args.live_safety_test:
                capture_command += ["--observe-seconds", "30"]
                capture_process = subprocess.Popen(capture_command, env=env)
                # Stop only this run's perception process. Gazebo clock and the
                # safety consumer stay live so target-loss refusal is observable.
                time.sleep(18)
                (output / "input_loss.json").write_text(json.dumps({
                    "wall_ns": time.time_ns(), "stopped_perception_pid": helpers[0].pid,
                    "method": "SIGINT_to_own_publisher_no_motion_active",
                }) + "\n")
                helpers[0].send_signal(signal.SIGINT)
                helpers[0].wait(timeout=5)
                capture_process.wait(timeout=25)
                if capture_process.returncode:
                    raise RuntimeError("camera capture failed")
            else:
                subprocess.run(capture_command, env=env, check=True, timeout=55)
        finally:
            for helper in helpers:
                if helper.poll() is None:
                    helper.send_signal(signal.SIGINT)
                    helper.wait(timeout=5)
            process.send_signal(signal.SIGINT)
            # ROS launch propagates shutdown to exactly the children it started.
            process.wait(timeout=25)
            # gz's Ruby launcher can exit before its server child. Only match
            # this fresh run's domain AND unique partition, and log exact PIDs.
            remaining = scoped_pids(args.domain, env["GZ_PARTITION"])
            for pid in remaining:
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            time.sleep(1)
            survivors = scoped_pids(args.domain, env["GZ_PARTITION"])
            (output / "cleanup.json").write_text(json.dumps({
                "term_pids": remaining, "remaining_pids": survivors,
            }, indent=2) + "\n")
            if survivors:
                raise RuntimeError(f"scoped processes remain: {survivors}")


if __name__ == "__main__":
    main()
