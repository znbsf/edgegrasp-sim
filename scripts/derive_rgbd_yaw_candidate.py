#!/usr/bin/env python3
"""Derive a bounded yaw adjustment from the pinned planar arm geometry; no motion."""

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def rpy(values):
    r, p, y = map(float, values.split())
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    return rz(y) @ ry @ rx


def derive_delta(normal_xy, reference_xy, observed_xy):
    normal = np.asarray(normal_xy) / np.linalg.norm(normal_xy)
    reference, observed = np.asarray(reference_xy), np.asarray(observed_xy)
    ratio = np.dot(normal, reference) / np.linalg.norm(observed)
    if abs(ratio) > 1:
        raise ValueError("no yaw solution")
    angle = math.atan2(observed[1], observed[0]) - math.atan2(normal[1], normal[0])
    choices = [(angle + sign * math.acos(ratio) + math.pi) % (2*math.pi) - math.pi
               for sign in (-1, 1)]
    delta = min(choices, key=abs)
    if abs(delta) > math.radians(.5):
        raise ValueError("derived yaw exceeds the declared half-degree bound")
    return delta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("baseline_plan", "urdf", "candidate", "observation", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    plan = json.loads(args.baseline_plan.read_text())
    if not plan["all_segments_accepted"] or plan["execution_attempted"]:
        raise ValueError("baseline must be qualified plan-only")
    baseline = plan["config"]["recorded_rgbd_observation"]["observation"]["center_m"]
    observed = json.loads(args.observation.read_text())["center_m"]
    if math.dist(baseline, observed) > .00101:
        raise ValueError("target outside the declared translation neighborhood")
    joint_row = next(row for row in plan["segments"] if row["segment"] == "approach_to_descend")
    pan = joint_row["terminal_joint_positions_rad"][0]
    robot = ET.parse(args.urdf).getroot()
    shoulder = robot.find("joint[@name='shoulder_pan']")
    lift = robot.find("joint[@name='shoulder_lift']")
    if shoulder.find("parent").attrib["link"] != "base_link":
        raise ValueError("unsupported shoulder pivot frame")
    if shoulder.find("axis").attrib["xyz"] != "0 0 1":
        raise ValueError("unsupported shoulder axis")
    pivot = np.array(list(map(float, shoulder.find("origin").attrib["xyz"].split())))
    normal = rpy(shoulder.find("origin").attrib["rpy"]) @ rz(pan) @ rpy(lift.find("origin").attrib["rpy"]) @ [0, 0, 1]
    if abs(normal[2]) > 1e-4:
        raise ValueError("pitch axis is not horizontal")
    delta = derive_delta(normal[:2], np.asarray(baseline)[:2]-pivot[:2], np.asarray(observed)[:2]-pivot[:2])
    candidate = json.loads(args.candidate.read_text())
    x, y, z, w = candidate["orientations_xyzw"]["grasp"]
    c, s = math.cos(delta/2), math.sin(delta/2)
    candidate["orientations_xyzw"]["grasp"] = [c*x-s*y, c*y+s*x, c*z+s*w, c*w-s*z]
    candidate["name"] += "_derived_yaw"
    candidate["changed_variable"] = dict(name="derived_base_yaw_for_same_declared_position",
        delta_rad=delta, delta_deg=math.degrees(delta), maximum_abs_deg=.5,
        shoulder_pivot_m=pivot.tolist(), plane_normal=normal.tolist(),
        recorded_rgbd_observation=str(args.observation), baseline_plan=str(args.baseline_plan),
        claim="kinematic candidate only; collision validation and isolated plan required")
    with args.output.open("x") as stream:
        json.dump(candidate, stream, indent=2)
    print(json.dumps(candidate["changed_variable"]))


if __name__ == "__main__":
    main()
