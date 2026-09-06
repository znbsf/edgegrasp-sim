#!/usr/bin/env bash
# Zero-motion ROS message and offline geometry tests using the existing overlay.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/edgegrasp/ros2_ws/install/setup.bash
rgbd_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$rgbd_root/src:$rgbd_root/ros_ws/src/edgegrasp_ros:$rgbd_root/ros_ws/src/edgegrasp_grasp_sequence:${PYTHONPATH:-}"
cd "$rgbd_root"
python3 -m pytest tests/test_action_readiness.py tests/test_rgbd.py tests/test_target_motion.py \
  ros_ws/src/edgegrasp_ros/test/test_rgbd_target_publisher.py \
  ros_ws/src/edgegrasp_grasp_sequence/test/test_observed_lift.py -q
python3 -m pytest ros_ws/src/edgegrasp_grasp_sequence/test/test_grasp_sequence_client.py \
  -k derives_candidate009 -q
