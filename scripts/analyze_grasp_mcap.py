#!/usr/bin/env python3
"""Build one source-time grasp timeline from an EdgeGrasp ros_sim MCAP.

This is an offline evidence tool.  It never starts a node, publishes a topic,
or sends a motion command.  Run it only after sourcing the Jazzy workspace so
the custom EdgeGrasp message types can be deserialized.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = (
    "/joint_states",
    "/gripper_controller/controller_state",
    "/edgegrasp/target_cube_pose",
    "/edgegrasp/target_cube_contacts",
    "/edgegrasp/grasp_timeline",
    "/edgegrasp/grasp_sequence_terminal",
)
PAD_TOKENS = (
    "fixed_finger_pad",
    "moving_finger_pad",
)
TARGET_TOKEN = "target_cube"


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--task-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--csv", type=Path)
    return parser.parse_args()


def _stamp_ns(stamp) -> int:
    sec = int(stamp.sec)
    nanosec = int(stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError("message contains an invalid source timestamp")
    return sec * 1_000_000_000 + nanosec


def _finite_or_none(values, index: int) -> float | None:
    if index >= len(values):
        return None
    value = float(values[index])
    return value if math.isfinite(value) else None


def _force_magnitude(vector) -> float:
    values = (float(vector.x), float(vector.y), float(vector.z))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("contact force is non-finite")
    return math.sqrt(sum(value * value for value in values))


def _contact_event(message, receive_ns: int) -> dict[str, object]:
    tokens: set[str] = set()
    collisions: list[tuple[str, str]] = []
    max_force_by_token = {token: 0.0 for token in PAD_TOKENS}
    for row in message.contacts:
        first = str(row.collision1.name)
        second = str(row.collision2.name)
        collisions.append((first, second))
        joined = f"{first}|{second}"
        if TARGET_TOKEN not in joined:
            continue
        row_force = 0.0
        for wrench in row.wrenches:
            row_force = max(
                row_force,
                _force_magnitude(wrench.body_1_wrench.force),
                _force_magnitude(wrench.body_2_wrench.force),
            )
        for token in PAD_TOKENS:
            if token in joined:
                tokens.add(token)
                max_force_by_token[token] = max(max_force_by_token[token], row_force)
    return {
        "time_ns": _stamp_ns(message.header.stamp),
        "receive_ns": receive_ns,
        "stream": "contact",
        "fixed_contact": PAD_TOKENS[0] in tokens,
        "moving_contact": PAD_TOKENS[1] in tokens,
        "fixed_max_force_n": max_force_by_token[PAD_TOKENS[0]],
        "moving_max_force_n": max_force_by_token[PAD_TOKENS[1]],
        "collision_pairs": collisions,
    }


def _reader(bag: Path):
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise ValueError(f"MCAP bag metadata is missing: {bag}")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    topic_types = {
        row.name: row.type for row in reader.get_all_topics_and_types()
    }
    missing = [topic for topic in TOPICS if topic not in topic_types]
    if missing:
        raise ValueError(f"required timeline topics are missing: {missing}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    return reader, topic_types


def _nearest(events: list[dict[str, object]], time_ns: int) -> dict[str, object] | None:
    if not events:
        return None
    return min(events, key=lambda item: abs(int(item["time_ns"]) - time_ns))


def _first_timeline(
    timeline: list[dict[str, object]], event_type: str, stage: str | None = None
) -> dict[str, object] | None:
    for event in timeline:
        if event.get("event_type") == event_type and (
            stage is None or event.get("stage") == stage
        ):
            return event
    return None


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    fields = (
        "time_ns",
        "receive_ns",
        "stream",
        "event_type",
        "phase",
        "stage",
        "command_id",
        "position_rad",
        "velocity_rad_s",
        "effort_nm",
        "controller_reference_position_rad",
        "controller_feedback_position_rad",
        "controller_error_position_rad",
        "controller_reference_effort_nm",
        "controller_output_effort_nm",
        "cube_x_m",
        "cube_y_m",
        "cube_z_m",
        "fixed_contact",
        "moving_contact",
        "fixed_max_force_n",
        "moving_max_force_n",
        "reason",
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(sorted(rows, key=lambda item: (int(item["time_ns"]), str(item["stream"]))))


def main() -> int:
    args = _arguments()
    reader, topic_types = _reader(args.bag)
    message_types = {topic: get_message(name) for topic, name in topic_types.items()}
    rows: list[dict[str, object]] = []
    timeline: list[dict[str, object]] = []
    joints: list[dict[str, object]] = []
    poses: list[dict[str, object]] = []
    contacts: list[dict[str, object]] = []
    controller_outputs: list[dict[str, object]] = []
    counts = {topic: 0 for topic in TOPICS}

    while reader.has_next():
        topic, data, receive_ns = reader.read_next()
        message = deserialize_message(data, message_types[topic])
        counts[topic] += 1
        if topic == "/edgegrasp/grasp_timeline":
            event = json.loads(message.data)
            if event.get("task_id") != args.task_id:
                continue
            event["time_ns"] = int(event["event_stamp_ns"])
            event["receive_ns"] = int(receive_ns)
            event["stream"] = "sequence"
            timeline.append(event)
            rows.append(event)
        elif topic == "/joint_states":
            names = list(message.name)
            if names.count("gripper") != 1:
                continue
            index = names.index("gripper")
            event = {
                "time_ns": _stamp_ns(message.header.stamp),
                "receive_ns": int(receive_ns),
                "stream": "gripper_joint",
                "position_rad": _finite_or_none(message.position, index),
                "velocity_rad_s": _finite_or_none(message.velocity, index),
                "effort_nm": _finite_or_none(message.effort, index),
            }
            joints.append(event)
            rows.append(event)
        elif topic == "/gripper_controller/controller_state":
            names = list(message.joint_names)
            if names.count("gripper") != 1:
                continue
            index = names.index("gripper")
            event = {
                "time_ns": _stamp_ns(message.header.stamp),
                "receive_ns": int(receive_ns),
                "stream": "gripper_controller",
                "controller_reference_position_rad": _finite_or_none(
                    message.reference.positions, index
                ),
                "controller_feedback_position_rad": _finite_or_none(
                    message.feedback.positions, index
                ),
                "controller_error_position_rad": _finite_or_none(
                    message.error.positions, index
                ),
                "controller_reference_effort_nm": _finite_or_none(
                    message.reference.effort, index
                ),
                "controller_output_effort_nm": _finite_or_none(
                    message.output.effort, index
                ),
            }
            controller_outputs.append(event)
            rows.append(event)
        elif topic == "/edgegrasp/target_cube_pose":
            position = message.pose.position
            event = {
                "time_ns": _stamp_ns(message.header.stamp),
                "receive_ns": int(receive_ns),
                "stream": "cube_pose",
                "cube_x_m": float(position.x),
                "cube_y_m": float(position.y),
                "cube_z_m": float(position.z),
            }
            if all(math.isfinite(float(event[field])) for field in ("cube_x_m", "cube_y_m", "cube_z_m")):
                poses.append(event)
                rows.append(event)
        elif topic == "/edgegrasp/target_cube_contacts":
            event = _contact_event(message, int(receive_ns))
            contacts.append(event)
            rows.append(event)

    timeline.sort(key=lambda item: (int(item["time_ns"]), int(item["timeline_seq"])))
    joints.sort(key=lambda item: int(item["time_ns"]))
    poses.sort(key=lambda item: int(item["time_ns"]))
    contacts.sort(key=lambda item: int(item["time_ns"]))
    controller_outputs.sort(key=lambda item: int(item["time_ns"]))
    if not timeline or not joints or not poses or not contacts or not controller_outputs:
        raise ValueError("timeline capture is incomplete")

    close_dispatch = _first_timeline(timeline, "gripper_command_dispatched", "close_gripper")
    close_terminal = _first_timeline(timeline, "gripper_command_terminal", "close_gripper")
    lift_dispatch = _first_timeline(timeline, "arm_command_dispatched", "lift")
    lift_terminal = _first_timeline(timeline, "arm_command_terminal", "lift")
    command_events = {
        "close_dispatch": close_dispatch,
        "close_terminal": close_terminal,
        "lift_dispatch": lift_dispatch,
        "lift_terminal": lift_terminal,
    }
    missing_command_events = [
        name for name, event in command_events.items() if event is None
    ]

    contact_times = {
        token: [
            int(item["time_ns"])
            for item in contacts
            if bool(item["fixed_contact" if token == PAD_TOKENS[0] else "moving_contact"])
        ]
        for token in PAD_TOKENS
    }
    simultaneous = [
        int(item["time_ns"])
        for item in contacts
        if item["fixed_contact"] and item["moving_contact"]
    ]
    close_start_ns = (
        int(close_dispatch["time_ns"]) if close_dispatch is not None else None
    )
    close_terminal_ns = (
        int(close_terminal["time_ns"]) if close_terminal is not None else None
    )
    lift_start_ns = (
        int(lift_dispatch["time_ns"]) if lift_dispatch is not None else None
    )
    lift_end_ns = int(lift_terminal["time_ns"]) if lift_terminal is not None else None
    bilateral_window = (
        (simultaneous[0], simultaneous[-1]) if simultaneous else None
    )
    bilateral_efforts = [
        float(item["effort_nm"])
        for item in joints
        if item["effort_nm"] is not None
        and bilateral_window is not None
        and bilateral_window[0] <= int(item["time_ns"]) <= bilateral_window[1]
    ]
    lift_efforts = [
        float(item["effort_nm"])
        for item in joints
        if item["effort_nm"] is not None
        and lift_start_ns is not None
        and lift_end_ns is not None
        and lift_start_ns <= int(item["time_ns"]) <= lift_end_ns
    ]
    lift_controller_efforts = [
        float(item["controller_output_effort_nm"])
        for item in controller_outputs
        if item["controller_output_effort_nm"] is not None
        and lift_start_ns is not None
        and lift_end_ns is not None
        and lift_start_ns <= int(item["time_ns"]) <= lift_end_ns
    ]
    baseline_pose = poses[0]
    final_pose = poses[-1]
    peak_pose = max(poses, key=lambda item: float(item["cube_z_m"]))
    first_fixed = (
        contact_times[PAD_TOKENS[0]][0]
        if contact_times[PAD_TOKENS[0]]
        else None
    )
    first_moving = (
        contact_times[PAD_TOKENS[1]][0]
        if contact_times[PAD_TOKENS[1]]
        else None
    )

    summary = {
        "schema_version": 1,
        "evidence_kind": "single_clock_grasp_timeline",
        "bag": str(args.bag.resolve()),
        "task_id": args.task_id,
        "time_contract": {
            "clock_domain": "ros_sim",
            "bag_recording_requires_use_sim_time": True,
            "message_alignment": "source_header_or_local_ros_event_stamp",
            "receive_timestamp_retained_separately": True,
        },
        "message_counts": counts,
        "sequence_events": timeline,
        "command_timeline_complete": not missing_command_events,
        "missing_command_events": missing_command_events,
        "contact": {
            "first_fixed_source_timestamp_ns": first_fixed,
            "first_moving_source_timestamp_ns": first_moving,
            "first_contact_delta_ns": (
                None
                if first_fixed is None or first_moving is None
                else abs(first_fixed - first_moving)
            ),
            "first_contact_order": (
                "unilateral_or_none"
                if first_fixed is None or first_moving is None
                else (
                    "fixed_then_moving"
                    if first_fixed < first_moving
                    else "moving_then_fixed"
                )
            ),
            "first_simultaneous_source_timestamp_ns": (
                simultaneous[0] if simultaneous else None
            ),
            "last_simultaneous_source_timestamp_ns": (
                simultaneous[-1] if simultaneous else None
            ),
            "simultaneous_span_ns": (
                simultaneous[-1] - simultaneous[0] if simultaneous else 0
            ),
            "simultaneous_during_lift_start": bool(
                simultaneous
                and lift_start_ns is not None
                and simultaneous[0] <= lift_start_ns <= simultaneous[-1]
            ),
        },
        "timing": {
            "close_dispatch_ns": close_start_ns,
            "close_terminal_ns": close_terminal_ns,
            "lift_dispatch_ns": lift_start_ns,
            "lift_terminal_ns": lift_end_ns,
            "close_terminal_to_lift_dispatch_ns": (
                lift_start_ns - close_terminal_ns
                if lift_start_ns is not None and close_terminal_ns is not None
                else None
            ),
        },
        "gripper": {
            "state_at_close_dispatch": (
                _nearest(joints, close_start_ns)
                if close_start_ns is not None
                else None
            ),
            "state_at_first_fixed_contact": (
                None if first_fixed is None else _nearest(joints, first_fixed)
            ),
            "state_at_first_moving_contact": (
                None if first_moving is None else _nearest(joints, first_moving)
            ),
            "state_at_lift_dispatch": (
                _nearest(joints, lift_start_ns)
                if lift_start_ns is not None
                else None
            ),
            "state_at_lift_terminal": (
                _nearest(joints, lift_end_ns) if lift_end_ns is not None else None
            ),
            "median_effort_during_bilateral_contact_nm": (
                statistics.median(bilateral_efforts) if bilateral_efforts else None
            ),
            "median_effort_during_lift_nm": (
                statistics.median(lift_efforts) if lift_efforts else None
            ),
            "peak_abs_effort_nm": max(
                abs(float(item["effort_nm"]))
                for item in joints
                if item["effort_nm"] is not None
            ),
            "controller_state_at_lift_dispatch": _nearest(
                controller_outputs, lift_start_ns
            )
            if lift_start_ns is not None
            else None,
            "median_controller_output_effort_during_lift_nm": (
                statistics.median(lift_controller_efforts)
                if lift_controller_efforts
                else None
            ),
            "peak_abs_controller_output_effort_nm": (
                max(
                    abs(float(item["controller_output_effort_nm"]))
                    for item in controller_outputs
                    if item["controller_output_effort_nm"] is not None
                )
                if any(
                    item["controller_output_effort_nm"] is not None
                    for item in controller_outputs
                )
                else None
            ),
        },
        "cube": {
            "baseline_pose": baseline_pose,
            "peak_pose": peak_pose,
            "final_pose": final_pose,
            "peak_lift_m": float(peak_pose["cube_z_m"]) - float(baseline_pose["cube_z_m"]),
            "final_xy_drift_m": math.hypot(
                float(final_pose["cube_x_m"]) - float(baseline_pose["cube_x_m"]),
                float(final_pose["cube_y_m"]) - float(baseline_pose["cube_y_m"]),
            ),
        },
        "claim_boundary": {
            "sequence_events_are_derived_evidence": True,
            "physics_grasp_verified": False,
            "hardware_verified": False,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    if args.csv is not None:
        _write_csv(args.csv, rows)
    print(json.dumps(summary, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
