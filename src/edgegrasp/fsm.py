"""Explicit lifecycle state machine for the grasp pipeline."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class PipelineState(str, Enum):
    IDLE = "IDLE"
    TRACKING = "TRACKING"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    SAFE_STOP = "SAFE_STOP"
    ERROR = "ERROR"


@dataclass(frozen=True, slots=True)
class StateTransition:
    previous: PipelineState
    current: PipelineState
    reason: str
    timestamp_ns: int


_ALLOWED_TRANSITIONS: dict[PipelineState, set[PipelineState]] = {
    PipelineState.IDLE: {PipelineState.TRACKING, PipelineState.SAFE_STOP, PipelineState.ERROR},
    PipelineState.TRACKING: {
        PipelineState.TRACKING,
        PipelineState.PLANNING,
        PipelineState.IDLE,
        PipelineState.SAFE_STOP,
        PipelineState.ERROR,
    },
    PipelineState.PLANNING: {
        PipelineState.EXECUTING,
        PipelineState.SAFE_STOP,
        PipelineState.ERROR,
    },
    PipelineState.EXECUTING: {
        PipelineState.TRACKING,
        PipelineState.IDLE,
        PipelineState.SAFE_STOP,
        PipelineState.ERROR,
    },
    PipelineState.SAFE_STOP: {PipelineState.IDLE},
    PipelineState.ERROR: {PipelineState.IDLE},
}


class StateMachine:
    def __init__(self) -> None:
        self._state = PipelineState.IDLE
        self._history: list[StateTransition] = []

    @property
    def state(self) -> PipelineState:
        return self._state

    @property
    def history(self) -> tuple[StateTransition, ...]:
        return tuple(self._history)

    def transition(self, new_state: PipelineState, reason: str, timestamp_ns: int) -> None:
        if new_state not in _ALLOWED_TRANSITIONS[self._state]:
            raise ValueError(f"invalid transition: {self._state.value} -> {new_state.value}")
        previous = self._state
        self._state = new_state
        self._history.append(StateTransition(previous, new_state, reason, timestamp_ns))

    def reset(self, timestamp_ns: int, reason: str = "manual_reset") -> None:
        if self._state == PipelineState.IDLE:
            return
        if PipelineState.IDLE not in _ALLOWED_TRANSITIONS[self._state]:
            raise ValueError(f"state {self._state.value} cannot be reset directly")
        self.transition(PipelineState.IDLE, reason, timestamp_ns)
