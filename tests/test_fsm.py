import pytest

from edgegrasp.fsm import PipelineState, StateMachine


def test_minimum_happy_path_states_are_explicit() -> None:
    machine = StateMachine()
    machine.transition(PipelineState.TRACKING, "target", 1)
    machine.transition(PipelineState.PLANNING, "predict", 2)
    machine.transition(PipelineState.EXECUTING, "plan", 3)
    machine.transition(PipelineState.TRACKING, "done", 4)

    assert [transition.current for transition in machine.history] == [
        PipelineState.TRACKING,
        PipelineState.PLANNING,
        PipelineState.EXECUTING,
        PipelineState.TRACKING,
    ]


def test_safe_stop_requires_reset_before_tracking() -> None:
    machine = StateMachine()
    machine.transition(PipelineState.SAFE_STOP, "stale", 1)

    with pytest.raises(ValueError, match="invalid transition"):
        machine.transition(PipelineState.TRACKING, "unsafe restart", 2)

    machine.reset(3)
    machine.transition(PipelineState.TRACKING, "fresh target", 4)
    assert machine.state is PipelineState.TRACKING
