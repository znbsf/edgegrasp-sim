from __future__ import annotations

import pytest

from edgegrasp.trajectory_identity import (
    make_trajectory_command_id,
    trajectory_digest,
    validate_trajectory_command_id,
)


POINTS = (
    {
        "positions": (0.0, 0.1),
        "velocities": (0.0, 0.0),
        "accelerations": (),
        "effort": (),
        "time_from_start_ns": 1_000_000_000,
    },
    {
        "positions": (0.2, 0.3),
        "velocities": (),
        "accelerations": (),
        "effort": (),
        "time_from_start_ns": 2_000_000_000,
    },
)


def test_command_id_is_unambiguously_bound_to_task_stage_and_sequence() -> None:
    command_id = make_trajectory_command_id("pick-17", "approach", 3)
    assert command_id == "pick-17|approach|3"
    validate_trajectory_command_id(command_id, "pick-17", "approach", 3)
    with pytest.raises(ValueError, match="command_id mismatch"):
        validate_trajectory_command_id(command_id, "pick-17", "descend", 3)


@pytest.mark.parametrize(
    ("task_id", "stage", "sequence_no"),
    [
        ("contains|separator", "approach", 0),
        ("task", "contains space", 0),
        ("task", "approach", -1),
        ("task", "approach", True),
    ],
)
def test_invalid_command_identity_is_rejected(
    task_id: str, stage: str, sequence_no: int
) -> None:
    with pytest.raises(ValueError):
        make_trajectory_command_id(task_id, stage, sequence_no)


def test_trajectory_digest_is_stable_and_covers_complete_command() -> None:
    first = trajectory_digest("arm_controller", ("j1", "j2"), POINTS)
    second = trajectory_digest("arm_controller", ["j1", "j2"], list(POINTS))
    assert first == second
    assert len(first) == 64

    mutations = (
        ("gripper_controller", ("j1", "j2"), POINTS),
        ("arm_controller", ("j2", "j1"), POINTS),
        (
            "arm_controller",
            ("j1", "j2"),
            (POINTS[0], {**POINTS[1], "positions": (0.2, 0.31)}),
        ),
        (
            "arm_controller",
            ("j1", "j2"),
            (POINTS[0], {**POINTS[1], "time_from_start_ns": 2_000_000_001}),
        ),
    )
    assert all(trajectory_digest(*mutation) != first for mutation in mutations)


def test_trajectory_digest_rejects_nonfinite_values() -> None:
    points = ({**POINTS[0], "positions": (0.0, float("nan"))},)
    with pytest.raises(ValueError, match="non-finite"):
        trajectory_digest("arm_controller", ("j1", "j2"), points)
