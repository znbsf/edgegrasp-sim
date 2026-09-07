"""Fail-closed ROS FollowJointTrajectory command-boundary adapter.

Only arm and gripper trajectories published to the two EdgeGrasp request
topics are part of the safety path. Direct goals to either downstream action
bypass this node and are unsafe diagnostic-only commands.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from functools import partial
import math
from threading import Event, RLock
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from edgegrasp.config import (
    DEFAULT_TARGET_FRAME,
    ROS_SYSTEM_CLOCK_DOMAIN,
    validate_ros_clock_domain,
)
from edgegrasp.permission import MotionPermissionGate
from edgegrasp.so101_contract import (
    SO101_ARM_ACTION,
    SO101_ARM_JOINTS,
    SO101_GRIPPER_ACTION,
    SO101_GRIPPER_JOINTS,
    validate_trajectory_contract,
)
from edgegrasp.trajectory_identity import (
    trajectory_digest,
    validate_trajectory_command_id,
)
from edgegrasp_interfaces.action import ExecuteTrajectory
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory


CONTROLLERS = ("arm_controller", "gripper_controller")
FJT_ERROR_CODE_UNAVAILABLE = -(2**31)
TYPED_STAGE_CONTROLLERS = {
    "approach": "arm_controller",
    "descend": "arm_controller",
    "close_gripper": "gripper_controller",
    "lift": "arm_controller",
    "place": "arm_controller",
    "retreat": "arm_controller",
    "diagnostic": None,
}


@dataclass(slots=True)
class _TypedExecution:
    """One immutable command correlated through the downstream FJT result."""

    goal_handle: object
    task_id: str
    command_id: str
    target_id: str
    stage: str
    sequence_no: int
    controller: str
    source_timestamp_ns: int
    trajectory_digest: str
    accepted: bool = False
    terminal: bool = False
    downstream_terminal_observed: bool = False
    action_goal_status: int = GoalStatus.STATUS_UNKNOWN
    fjt_error_code: int = FJT_ERROR_CODE_UNAVAILABLE
    fjt_error_string: str = ""
    success: bool = False
    reason: str = "pending"
    completed_timestamp_ns: int = 0
    cancel_reason: str | None = None
    event: Event = field(default_factory=Event)


@dataclass(slots=True)
class _TypedTombstone:
    """Recovery barrier retained after a wrapper terminal with uncertain stop.

    A late downstream result may add terminal evidence, but it can never
    rewrite the already-returned ExecuteTrajectory result or release the
    controller.  Only the explicit reset service clears a confirmed tombstone.
    """

    command_id: str
    terminal_confirmed: bool = False
    action_goal_status: int = GoalStatus.STATUS_UNKNOWN
    evidence: str = "stop_unconfirmed"


class TrajectoryGate(Node):
    """Forward only permitted trajectories matching the pinned SO-101 contract."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__("edgegrasp_trajectory_gate", **node_kwargs)
        self.declare_parameter(
            "arm_trajectory_request_topic", "/edgegrasp/arm_joint_trajectory_request"
        )
        self.declare_parameter(
            "gripper_trajectory_request_topic",
            "/edgegrasp/gripper_joint_trajectory_request",
        )
        self.declare_parameter("motion_allowed_topic", "/edgegrasp/motion_allowed")
        self.declare_parameter("interface_ready_topic", "/edgegrasp/interface_ready")
        self.declare_parameter("target_topic", "/edgegrasp/target_3d")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter(
            "execute_trajectory_action", "/edgegrasp/execute_trajectory"
        )
        self.declare_parameter("arm_downstream_action", SO101_ARM_ACTION)
        self.declare_parameter("gripper_downstream_action", SO101_GRIPPER_ACTION)
        self.declare_parameter("permission_timeout_ms", 100.0)
        self.declare_parameter("interface_timeout_ms", 500.0)
        self.declare_parameter("target_watchdog_timeout_ms", 200.0)
        self.declare_parameter("joint_state_timeout_ms", 250.0)
        self.declare_parameter("action_response_timeout_ms", 1000.0)
        self.declare_parameter("cancel_response_timeout_ms", 500.0)
        self.declare_parameter("cancel_completion_timeout_ms", 1000.0)
        self.declare_parameter("result_timeout_margin_ms", 2000.0)
        # This is a wall-clock guard for a wedged action pipeline.  Downstream
        # trajectory deadlines are enforced separately in the active ROS
        # clock domain.  A simulated 8 s trajectory can take much longer than
        # 8 s of wall time when headless Gazebo runs below real time.
        self.declare_parameter("typed_terminal_wall_guard_ms", 90000.0)
        self.declare_parameter("max_cancel_attempts", 3)
        self.declare_parameter("cancel_retry_ms", 100.0)
        self.declare_parameter("check_rate_hz", 50.0)
        self.declare_parameter("target_frame", DEFAULT_TARGET_FRAME)
        self.declare_parameter("clock_domain", ROS_SYSTEM_CLOCK_DOMAIN)
        self.declare_parameter("clock_epoch", 0)

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._clock_domain = str(self.get_parameter("clock_domain").value)
        self._clock_epoch = int(self.get_parameter("clock_epoch").value)
        use_sim_time = bool(self.get_parameter("use_sim_time").value)
        validate_ros_clock_domain(use_sim_time, self._clock_domain)
        if not self._target_frame or self._clock_epoch < 0:
            raise ValueError("target_frame and clock_epoch must be valid")

        configured_actions = {
            "arm_controller": str(self.get_parameter("arm_downstream_action").value),
            "gripper_controller": str(
                self.get_parameter("gripper_downstream_action").value
            ),
        }
        expected_actions = {
            "arm_controller": SO101_ARM_ACTION,
            "gripper_controller": SO101_GRIPPER_ACTION,
        }
        if configured_actions != expected_actions:
            raise ValueError(
                "downstream actions must exactly match the pinned SO-101 contract"
            )

        self._permission = MotionPermissionGate(
            float(self.get_parameter("permission_timeout_ms").value)
        )
        self._interface_permission = MotionPermissionGate(
            float(self.get_parameter("interface_timeout_ms").value)
        )
        self._target_watchdog_timeout_ns = int(
            float(self.get_parameter("target_watchdog_timeout_ms").value) * 1_000_000
        )
        self._joint_state_timeout_ns = int(
            float(self.get_parameter("joint_state_timeout_ms").value) * 1_000_000
        )
        self._action_response_timeout_ns = int(
            float(self.get_parameter("action_response_timeout_ms").value) * 1_000_000
        )
        self._cancel_response_timeout_ns = int(
            float(self.get_parameter("cancel_response_timeout_ms").value) * 1_000_000
        )
        self._cancel_completion_timeout_ns = int(
            float(self.get_parameter("cancel_completion_timeout_ms").value)
            * 1_000_000
        )
        self._result_timeout_margin_ns = int(
            float(self.get_parameter("result_timeout_margin_ms").value) * 1_000_000
        )
        self._typed_terminal_wall_guard_s = (
            float(self.get_parameter("typed_terminal_wall_guard_ms").value) / 1000.0
        )
        self._max_cancel_attempts = int(self.get_parameter("max_cancel_attempts").value)
        self._cancel_retry_ns = int(
            float(self.get_parameter("cancel_retry_ms").value) * 1_000_000
        )
        if (
            self._max_cancel_attempts < 1
            or not math.isfinite(self._typed_terminal_wall_guard_s)
            or self._typed_terminal_wall_guard_s <= 0.0
        ):
            raise ValueError("cancel attempts and typed wall guard must be positive")
        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = RLock()
        self._goal_clients = {
            controller: ActionClient(
                self,
                FollowJointTrajectory,
                action,
                callback_group=self._callback_group,
            )
            for controller, action in configured_actions.items()
        }
        self._goal_handles = {controller: None for controller in CONTROLLERS}
        self._send_goal_futures = {controller: None for controller in CONTROLLERS}
        self._send_goal_started_ns = {controller: None for controller in CONTROLLERS}
        self._result_futures = {controller: None for controller in CONTROLLERS}
        self._result_deadline_ns = {controller: None for controller in CONTROLLERS}
        self._goal_duration_ns = {controller: 0 for controller in CONTROLLERS}
        self._cancel_on_accept = {controller: False for controller in CONTROLLERS}
        self._cancel_requested = {controller: False for controller in CONTROLLERS}
        self._cancel_futures = {controller: None for controller in CONTROLLERS}
        self._cancel_started_ns = {controller: None for controller in CONTROLLERS}
        self._cancel_attempts = {controller: 0 for controller in CONTROLLERS}
        self._cancel_retry_due_ns = {controller: None for controller in CONTROLLERS}
        self._cancel_exhausted = {controller: False for controller in CONTROLLERS}
        self._typed_commands: dict[str, _TypedExecution | None] = {
            controller: None for controller in CONTROLLERS
        }
        self._typed_reservations: dict[str, str] = {}
        self._seen_command_ids: set[str] = set()
        self._typed_tombstones: dict[str, _TypedTombstone] = {}
        self._controller_owners: dict[str, str | None] = {
            controller: None for controller in CONTROLLERS
        }
        self._legacy_sequence = 0
        self._last_target_receive_ns: int | None = None
        self._last_joint_state_ns: int | None = None
        self._joint_positions: dict[str, float] = {}
        self._fault_latched: str | None = None

        self._status = self.create_publisher(
            String, "/edgegrasp/trajectory_gate_status", 10
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("motion_allowed_topic").value),
            self._on_permission,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("interface_ready_topic").value),
            self._on_interface_ready,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("target_topic").value),
            self._on_target,
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            10,
        )
        self.create_subscription(
            JointTrajectory,
            str(self.get_parameter("arm_trajectory_request_topic").value),
            partial(self._on_trajectory, "arm_controller"),
            10,
        )
        self.create_subscription(
            JointTrajectory,
            str(self.get_parameter("gripper_trajectory_request_topic").value),
            partial(self._on_trajectory, "gripper_controller"),
            10,
        )
        self.create_service(
            Trigger, "/edgegrasp/reset_trajectory_gate_epoch", self._on_reset
        )
        self._execute_server = ActionServer(
            self,
            ExecuteTrajectory,
            str(self.get_parameter("execute_trajectory_action").value),
            execute_callback=self._execute_typed,
            goal_callback=self._on_typed_goal,
            cancel_callback=self._on_typed_cancel,
            callback_group=self._callback_group,
        )
        rate_hz = float(self.get_parameter("check_rate_hz").value)
        if rate_hz <= 0.0:
            raise ValueError("check_rate_hz must be positive")
        self._timer = self.create_timer(
            1.0 / rate_hz,
            self._watchdog,
            callback_group=self._callback_group,
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _on_permission(self, message: Bool) -> None:
        # Keep the ROS clock read and the stateful permission update under one
        # lock.  This node runs in a MultiThreadedExecutor: sampling before the
        # lock lets an older callback commit after a newer callback and falsely
        # latch a clock rollback even though /clock itself is monotonic.
        with self._state_lock:
            decision = self._permission.update(bool(message.data), self._now_ns())
        if not decision.allowed:
            self._cancel_all(decision.reason)

    def _on_interface_ready(self, message: Bool) -> None:
        with self._state_lock:
            decision = self._interface_permission.update(
                bool(message.data), self._now_ns()
            )
        if not decision.allowed:
            self._cancel_all(f"interface:{decision.reason}")

    def _on_target(self, message: PointStamped) -> None:
        frame_id = message.header.frame_id or "<empty>"
        if frame_id != self._target_frame:
            self._set_fault(f"target_frame_mismatch:{frame_id}!={self._target_frame}")
            self._cancel_all(self._fault_latched or "target_frame_mismatch")
            return
        with self._state_lock:
            now_ns = self._now_ns()
            if (
                self._last_target_receive_ns is not None
                and now_ns < self._last_target_receive_ns
            ):
                rollback = (
                    f"target_clock_rollback:{now_ns}<"
                    f"{self._last_target_receive_ns}"
                )
            else:
                rollback = None
                self._last_target_receive_ns = now_ns
        if rollback is not None:
            self._set_fault(rollback)
            self._cancel_all(self._fault_latched or "target_clock_rollback")

    def _on_joint_state(self, message: JointState) -> None:
        if (
            len(message.name) != len(message.position)
            or len(set(message.name)) != len(message.name)
            or any(not math.isfinite(float(value)) for value in message.position)
        ):
            self._set_fault("invalid_joint_states")
            self._cancel_all(self._fault_latched or "invalid_joint_states")
            return
        positions = dict(zip(message.name, message.position, strict=True))
        expected = set(SO101_ARM_JOINTS + SO101_GRIPPER_JOINTS)
        if not expected.issubset(positions):
            self._set_fault("joint_states_missing_pinned_joints")
            self._cancel_all(self._fault_latched or "joint_states_missing_pinned_joints")
            return
        with self._state_lock:
            now_ns = self._now_ns()
            self._joint_positions = {
                joint: float(positions[joint]) for joint in expected
            }
            self._last_joint_state_ns = now_ns

    def _admission_state(
        self, controller: str, now_ns: int
    ) -> tuple[str | None, tuple[float, ...] | None]:
        if self._fault_latched is not None:
            return f"fault_latched:{self._fault_latched}", None
        interface = self._interface_permission.evaluate(now_ns)
        if not interface.allowed:
            return f"interface:{interface.reason}", None
        if self._last_target_receive_ns is None:
            return "target_liveness_unknown", None
        target_age_ns = now_ns - self._last_target_receive_ns
        if target_age_ns < 0:
            self._set_fault("target_clock_rollback")
            return "target_clock_rollback", None
        if target_age_ns > self._target_watchdog_timeout_ns:
            return "target_stream_timeout", None
        if self._last_joint_state_ns is None:
            return "joint_states_unknown", None
        joint_state_age_ns = now_ns - self._last_joint_state_ns
        if joint_state_age_ns < 0:
            self._set_fault("joint_state_clock_rollback")
            return "joint_state_clock_rollback", None
        if joint_state_age_ns > self._joint_state_timeout_ns:
            return "joint_states_stale", None
        joints = (
            SO101_ARM_JOINTS
            if controller == "arm_controller"
            else SO101_GRIPPER_JOINTS
        )
        try:
            start_positions = tuple(self._joint_positions[joint] for joint in joints)
        except KeyError:
            return "joint_states_missing_pinned_joints", None
        return None, start_positions

    def _set_fault(self, reason: str) -> None:
        if self._fault_latched is None:
            self._fault_latched = reason
            self._publish(f"ERROR:{reason}")

    @staticmethod
    def _point_payload(point) -> dict[str, object]:
        return {
            "positions": point.positions,
            "velocities": point.velocities,
            "accelerations": point.accelerations,
            "effort": point.effort,
            "time_from_start_ns": (
                point.time_from_start.sec * 1_000_000_000
                + point.time_from_start.nanosec
            ),
        }

    def _typed_metadata_reason(
        self,
        request: ExecuteTrajectory.Goal,
        *,
        now_ns: int | None = None,
    ) -> str | None:
        if request.controller not in CONTROLLERS:
            return f"unknown_controller:{request.controller}"
        expected_controller = TYPED_STAGE_CONTROLLERS.get(request.stage, "unknown")
        if expected_controller == "unknown":
            return f"unknown_stage:{request.stage}"
        if expected_controller is not None and request.controller != expected_controller:
            return (
                f"stage_controller_mismatch:{request.stage}:"
                f"{request.controller}!={expected_controller}"
            )
        try:
            validate_trajectory_command_id(
                request.command_id,
                request.task_id,
                request.stage,
                int(request.sequence_no),
            )
        except ValueError as error:
            return f"invalid_command_identity:{error}"
        if (
            not request.target_id
            or request.target_id.strip() != request.target_id
            or len(request.target_id) > 128
        ):
            return "invalid_target_id"
        if request.clock_domain != self._clock_domain:
            return (
                f"clock_domain_mismatch:{request.clock_domain}!="
                f"{self._clock_domain}"
            )
        if int(request.clock_epoch) != self._clock_epoch:
            return f"clock_epoch_mismatch:{request.clock_epoch}!={self._clock_epoch}"
        frame_id = request.trajectory.header.frame_id or "<empty>"
        if frame_id != self._target_frame:
            return f"trajectory_frame_mismatch:{frame_id}!={self._target_frame}"
        source_ns = int(request.source_timestamp_ns)
        if source_ns < 0:
            return "invalid_source_timestamp"
        if now_ns is None:
            now_ns = self._now_ns()
        age_ns = now_ns - source_ns
        if age_ns < 0:
            return f"source_timestamp_in_future:{source_ns}>{now_ns}"
        if age_ns > self._target_watchdog_timeout_ns:
            return f"source_timestamp_stale:{age_ns}"
        return None

    def _on_typed_goal(self, request: ExecuteTrajectory.Goal) -> GoalResponse:
        """Reserve one immutable command; duplicate/concurrent goals are rejected."""

        if request.controller not in CONTROLLERS:
            self._publish("typed_goal_rejected:unknown_controller")
            return GoalResponse.REJECT
        if request.stage not in TYPED_STAGE_CONTROLLERS:
            self._publish(
                "typed_goal_rejected:unknown_stage",
                request.controller,
            )
            return GoalResponse.REJECT
        expected_controller = TYPED_STAGE_CONTROLLERS[request.stage]
        if expected_controller is not None and request.controller != expected_controller:
            self._publish(
                "typed_goal_rejected:stage_controller_mismatch",
                request.controller,
            )
            return GoalResponse.REJECT
        try:
            validate_trajectory_command_id(
                request.command_id,
                request.task_id,
                request.stage,
                int(request.sequence_no),
            )
        except ValueError:
            self._publish(
                "typed_goal_rejected:invalid_command_identity",
                request.controller,
            )
            return GoalResponse.REJECT
        with self._state_lock:
            controller_reserved = request.controller in self._typed_reservations.values()
            downstream_busy = (
                self._goal_handles[request.controller] is not None
                or self._send_goal_futures[request.controller] is not None
            )
            if (
                self._fault_latched is not None
                or bool(self._typed_tombstones)
                or request.command_id in self._seen_command_ids
                or controller_reserved
                or self._controller_owners[request.controller] is not None
                or downstream_busy
            ):
                self._publish(
                    "typed_goal_rejected:duplicate_or_controller_busy",
                    request.controller,
                )
                return GoalResponse.REJECT
            self._seen_command_ids.add(request.command_id)
            self._typed_reservations[request.command_id] = request.controller
            self._controller_owners[request.controller] = request.command_id
        return GoalResponse.ACCEPT

    def _latch_typed_tombstone(
        self,
        controller: str,
        command_id: str | None,
        evidence: str = "stop_unconfirmed",
    ) -> None:
        if command_id is None:
            return
        with self._state_lock:
            current = self._typed_tombstones.get(controller)
            if current is None:
                self._typed_tombstones[controller] = _TypedTombstone(
                    command_id=command_id,
                    evidence=evidence,
                )
            elif current.command_id != command_id:
                self._fault_latched = (
                    f"tombstone_identity_conflict:{controller}:"
                    f"{current.command_id}!={command_id}"
                )

    def _typed_tombstone(
        self, controller: str, command_id: str | None
    ) -> _TypedTombstone | None:
        if command_id is None:
            return None
        with self._state_lock:
            tombstone = self._typed_tombstones.get(controller)
            if tombstone is None or tombstone.command_id != command_id:
                return None
            return tombstone

    def _mark_tombstone_terminal(
        self,
        controller: str,
        command_id: str,
        *,
        action_goal_status: int,
        evidence: str,
    ) -> None:
        if action_goal_status not in (
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        ):
            return
        self._latch_typed_tombstone(controller, command_id, evidence)
        with self._state_lock:
            tombstone = self._typed_tombstones.get(controller)
            if tombstone is None or tombstone.command_id != command_id:
                return
            tombstone.terminal_confirmed = True
            tombstone.action_goal_status = int(action_goal_status)
            tombstone.evidence = evidence

    def _mark_tombstone_no_downstream_goal(
        self, controller: str, command_id: str, evidence: str
    ) -> None:
        self._latch_typed_tombstone(controller, command_id, evidence)
        with self._state_lock:
            tombstone = self._typed_tombstones.get(controller)
            if tombstone is None or tombstone.command_id != command_id:
                return
            tombstone.terminal_confirmed = True
            tombstone.action_goal_status = GoalStatus.STATUS_UNKNOWN
            tombstone.evidence = evidence

    def _clear_low_level_after_terminal(self, controller: str) -> None:
        """Clear only FJT transport state after a proven downstream terminal."""

        with self._state_lock:
            self._goal_handles[controller] = None
            self._send_goal_futures[controller] = None
            self._send_goal_started_ns[controller] = None
            self._result_futures[controller] = None
            self._result_deadline_ns[controller] = None
            self._cancel_requested[controller] = False
            self._cancel_futures[controller] = None
            self._cancel_started_ns[controller] = None
            self._cancel_on_accept[controller] = False
            self._cancel_retry_due_ns[controller] = None
            self._cancel_exhausted[controller] = False

    def _release_controller_owner(
        self, controller: str, dispatch_id: str
    ) -> None:
        with self._state_lock:
            tombstone = self._typed_tombstones.get(controller)
            if tombstone is not None and tombstone.command_id == dispatch_id:
                return
            if self._controller_owners.get(controller) == dispatch_id:
                self._controller_owners[controller] = None

    def _dispatch_is_current(self, controller: str, dispatch_id: str) -> bool:
        with self._state_lock:
            return self._controller_owners.get(controller) == dispatch_id

    def _on_typed_cancel(self, goal_handle) -> CancelResponse:
        request = goal_handle.request
        with self._state_lock:
            context = self._typed_commands.get(request.controller)
            reserved = self._typed_reservations.get(request.command_id)
            if context is not None and context.command_id == request.command_id:
                if context.terminal:
                    return CancelResponse.REJECT
                context.cancel_reason = "client_cancel_requested"
            elif reserved != request.controller:
                return CancelResponse.REJECT
        self._cancel_controller(
            request.controller, f"typed_client_cancel:{request.command_id}"
        )
        return CancelResponse.ACCEPT

    def _typed_context(
        self, controller: str, command_id: str | None
    ) -> _TypedExecution | None:
        if command_id is None:
            return None
        with self._state_lock:
            context = self._typed_commands.get(controller)
            if context is None or context.command_id != command_id:
                return None
            return context

    def _complete_typed(
        self,
        context: _TypedExecution | None,
        *,
        accepted: bool | None = None,
        action_goal_status: int = GoalStatus.STATUS_UNKNOWN,
        fjt_error_code: int = FJT_ERROR_CODE_UNAVAILABLE,
        fjt_error_string: str = "",
        downstream_terminal_observed: bool = False,
        success: bool,
        reason: str,
    ) -> None:
        if context is None:
            return
        with self._state_lock:
            if context.terminal:
                return
            if accepted is not None:
                context.accepted = accepted
            context.action_goal_status = int(action_goal_status)
            context.fjt_error_code = int(fjt_error_code)
            context.fjt_error_string = str(fjt_error_string)
            context.downstream_terminal_observed = bool(
                downstream_terminal_observed
            )
            context.success = bool(success)
            context.reason = reason
            context.completed_timestamp_ns = self._now_ns()
            context.terminal = True
            context.event.set()

    def _typed_result(self, context: _TypedExecution) -> ExecuteTrajectory.Result:
        result = ExecuteTrajectory.Result()
        result.task_id = context.task_id
        result.command_id = context.command_id
        result.target_id = context.target_id
        result.stage = context.stage
        result.sequence_no = context.sequence_no
        result.controller = context.controller
        result.trajectory_digest = context.trajectory_digest
        result.accepted = context.accepted
        result.terminal = context.terminal
        result.downstream_terminal_observed = context.downstream_terminal_observed
        result.cancel_requested = context.cancel_reason is not None
        result.action_goal_status = context.action_goal_status
        result.fjt_error_code = context.fjt_error_code
        result.fjt_error_string = context.fjt_error_string
        result.success = context.success
        result.reason = context.reason
        result.source_timestamp_ns = context.source_timestamp_ns
        result.completed_timestamp_ns = context.completed_timestamp_ns
        result.clock_domain = self._clock_domain
        result.clock_epoch = self._clock_epoch
        return result

    def _typed_feedback(self, context: _TypedExecution, state: str) -> None:
        feedback = ExecuteTrajectory.Feedback()
        feedback.task_id = context.task_id
        feedback.command_id = context.command_id
        feedback.stage = context.stage
        feedback.state = state
        try:
            context.goal_handle.publish_feedback(feedback)
        except Exception as error:
            self._set_fault(f"typed_feedback_failed:{type(error).__name__}")

    def _execute_typed(self, goal_handle) -> ExecuteTrajectory.Result:
        request = goal_handle.request
        try:
            digest = trajectory_digest(
                request.controller,
                request.trajectory.joint_names,
                [self._point_payload(point) for point in request.trajectory.points],
            )
        except ValueError:
            digest = ""
        context = _TypedExecution(
            goal_handle=goal_handle,
            task_id=request.task_id,
            command_id=request.command_id,
            target_id=request.target_id,
            stage=request.stage,
            sequence_no=int(request.sequence_no),
            controller=request.controller,
            source_timestamp_ns=int(request.source_timestamp_ns),
            trajectory_digest=digest,
        )
        with self._state_lock:
            if (
                self._typed_reservations.get(request.command_id)
                != request.controller
                or self._controller_owners.get(request.controller)
                != request.command_id
                or self._typed_commands.get(request.controller) is not None
            ):
                self._complete_typed(
                    context,
                    success=False,
                    reason="command_reservation_lost",
                )
            else:
                self._typed_commands[request.controller] = context
        try:
            if not context.terminal:
                self._typed_feedback(context, "admission")
                reason = self._typed_metadata_reason(request)
                if reason is not None:
                    self._publish(f"rejected:{reason}", request.controller)
                    self._complete_typed(
                        context,
                        accepted=False,
                        success=False,
                        reason=reason,
                    )
                elif not digest:
                    self._publish(
                        "rejected:invalid_trajectory_digest_input",
                        request.controller,
                    )
                    self._complete_typed(
                        context,
                        accepted=False,
                        success=False,
                        reason="invalid_trajectory_digest_input",
                    )
                elif goal_handle.is_cancel_requested:
                    context.cancel_reason = "client_cancel_requested_before_dispatch"
                    self._complete_typed(
                        context,
                        accepted=False,
                        action_goal_status=GoalStatus.STATUS_CANCELED,
                        success=False,
                        reason=context.cancel_reason,
                    )
                else:
                    self._dispatch_trajectory(
                        request.controller,
                        request.trajectory,
                        dispatch_id=request.command_id,
                        command_id=request.command_id,
                    )

            # Never reinterpret ROS/sim trajectory duration as wall time.  The
            # watchdog above owns the ROS-time deadline; this independent
            # finite bound exists only for a wedged executor or stalled clock.
            deadline = time.monotonic() + self._typed_terminal_wall_guard_s
            while not context.event.wait(timeout=0.02):
                if goal_handle.is_cancel_requested and context.cancel_reason is None:
                    context.cancel_reason = "client_cancel_requested"
                    self._cancel_controller(
                        request.controller,
                        f"typed_client_cancel:{request.command_id}",
                    )
                if time.monotonic() >= deadline:
                    self._set_fault(f"typed_terminal_timeout:{request.controller}")
                    self._latch_typed_tombstone(
                        request.controller, request.command_id
                    )
                    self._cancel_controller(
                        request.controller, "typed_terminal_timeout"
                    )
                    self._complete_typed(
                        context,
                        accepted=context.accepted,
                        success=False,
                        reason="typed_terminal_timeout_stop_unconfirmed",
                    )
                    break

            if context.success:
                goal_handle.succeed()
            elif (
                goal_handle.is_cancel_requested
                and context.action_goal_status == GoalStatus.STATUS_CANCELED
            ):
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return self._typed_result(context)
        except Exception as error:
            detail = type(error).__name__
            self._set_fault(f"typed_execute_exception:{detail}")
            downstream_may_be_active = any(
                state is not None
                for state in (
                    self._send_goal_futures.get(request.controller),
                    self._goal_handles.get(request.controller),
                    self._result_futures.get(request.controller),
                    self._cancel_futures.get(request.controller),
                )
            )
            if downstream_may_be_active:
                with self._state_lock:
                    if context.cancel_reason is None:
                        context.cancel_reason = "typed_execute_exception"
                self._latch_typed_tombstone(
                    request.controller, request.command_id
                )
                self._cancel_controller(
                    request.controller, "typed_execute_exception"
                )
            with self._state_lock:
                if context.terminal:
                    context.success = False
                    context.reason = f"typed_execute_exception:{detail}"
                else:
                    self._complete_typed(
                        context,
                        accepted=context.accepted,
                        success=False,
                        reason=f"typed_execute_exception:{detail}",
                    )
            try:
                goal_handle.abort()
            except Exception:
                pass
            return self._typed_result(context)
        finally:
            with self._state_lock:
                if self._typed_commands.get(request.controller) is context:
                    self._typed_commands[request.controller] = None
                self._typed_reservations.pop(request.command_id, None)
            self._release_controller_owner(
                request.controller, request.command_id
            )

    def _on_trajectory(self, controller: str, message: JointTrajectory) -> None:
        with self._state_lock:
            if (
                self._fault_latched is not None
                or bool(self._typed_tombstones)
                or self._controller_owners[controller] is not None
            ):
                self._publish("rejected:controller_reserved_or_faulted", controller)
                return
            self._legacy_sequence += 1
            dispatch_id = f"legacy:{controller}:{self._legacy_sequence}"
            self._controller_owners[controller] = dispatch_id
        self._dispatch_trajectory(
            controller,
            message,
            dispatch_id=dispatch_id,
            command_id=None,
        )

    def _dispatch_trajectory(
        self,
        controller: str,
        message: JointTrajectory,
        *,
        dispatch_id: str,
        command_id: str | None,
    ) -> None:
        context = self._typed_context(controller, command_id)
        if not self._dispatch_is_current(controller, dispatch_id):
            self._complete_typed(
                context,
                accepted=False,
                success=False,
                reason="dispatch_owner_mismatch",
            )
            return
        frame_id = message.header.frame_id or "<empty>"
        if frame_id != self._target_frame:
            reason = f"trajectory_frame_mismatch:{frame_id}!={self._target_frame}"
            self._publish(f"rejected:{reason}", controller)
            self._complete_typed(
                context,
                accepted=False,
                success=False,
                reason=reason,
            )
            self._release_controller_owner(controller, dispatch_id)
            return
        with self._state_lock:
            now_ns = self._now_ns()
            permission = self._permission.evaluate(now_ns)
            admission_reason, start_positions = self._admission_state(
                controller, now_ns
            )
        if not permission.allowed:
            self._publish(f"rejected:{permission.reason}", controller)
            self._complete_typed(
                context,
                accepted=False,
                success=False,
                reason=f"permission:{permission.reason}",
            )
            self._release_controller_owner(controller, dispatch_id)
            return
        if admission_reason is not None or start_positions is None:
            self._publish(f"rejected:{admission_reason}", controller)
            self._complete_typed(
                context,
                accepted=False,
                success=False,
                reason=str(admission_reason),
            )
            self._release_controller_owner(controller, dispatch_id)
            return
        contract = validate_trajectory_contract(
            controller,
            message.joint_names,
            [self._point_payload(point) for point in message.points],
            start_positions=start_positions,
        )
        if not contract.accepted:
            self._publish(f"rejected:{contract.reason.value}:{contract.detail}", controller)
            self._complete_typed(
                context,
                accepted=False,
                success=False,
                reason=f"trajectory:{contract.reason.value}:{contract.detail}",
            )
            self._release_controller_owner(controller, dispatch_id)
            return
        pending = self._send_goal_futures[controller]
        if self._goal_handles[controller] is not None or (
            pending is not None and not pending.done()
        ):
            self._publish("rejected:goal_in_progress", controller)
            self._complete_typed(
                context,
                accepted=False,
                success=False,
                reason="goal_in_progress",
            )
            self._release_controller_owner(controller, dispatch_id)
            return
        client = self._goal_clients[controller]
        if not client.server_is_ready():
            self._publish("rejected:downstream_action_unavailable", controller)
            self._complete_typed(
                context,
                accepted=False,
                success=False,
                reason="downstream_action_unavailable",
            )
            self._release_controller_owner(controller, dispatch_id)
            return

        goal = FollowJointTrajectory.Goal()
        # ``JointTrajectory.header.stamp`` is an absolute controller start
        # time.  Target acquisition time is already carried independently by
        # ExecuteTrajectory.source_timestamp_ns; forwarding that older stamp
        # makes short/no-op plans expire before the JTC receives them.  The
        # safety boundary therefore preserves the validated frame/points but
        # explicitly requests immediate downstream execution with ROS's zero
        # timestamp convention.
        downstream_trajectory = deepcopy(message)
        downstream_trajectory.header.stamp.sec = 0
        downstream_trajectory.header.stamp.nanosec = 0
        goal.trajectory = downstream_trajectory
        self._goal_duration_ns[controller] = (
            message.points[-1].time_from_start.sec * 1_000_000_000
            + message.points[-1].time_from_start.nanosec
        )
        self._cancel_on_accept[controller] = False
        self._cancel_requested[controller] = False
        self._cancel_attempts[controller] = 0
        self._cancel_retry_due_ns[controller] = None
        self._cancel_exhausted[controller] = False
        try:
            future = client.send_goal_async(goal)
        except Exception as error:
            self._set_fault(f"send_goal_failed:{type(error).__name__}")
            self._publish(
                f"failed:send_goal:{type(error).__name__}", controller
            )
            self._complete_typed(
                context,
                accepted=False,
                success=False,
                reason=f"send_goal_failed:{type(error).__name__}",
            )
            self._release_controller_owner(controller, dispatch_id)
            return
        self._send_goal_futures[controller] = future
        self._send_goal_started_ns[controller] = now_ns
        future.add_done_callback(
            partial(
                self._on_goal_response,
                controller,
                future,
                dispatch_id,
                command_id,
            )
        )
        self._publish("forwarding:awaiting_goal_response", controller)

    def _on_goal_response(
        self,
        controller: str,
        expected_future,
        dispatch_id: str,
        command_id: str | None,
        future,
    ) -> None:
        if (
            self._send_goal_futures[controller] is not expected_future
            or not self._dispatch_is_current(controller, dispatch_id)
        ):
            self._publish("ignored:late_goal_response", controller)
            return
        context = self._typed_context(controller, command_id)
        tombstone = self._typed_tombstone(controller, dispatch_id)
        try:
            goal_handle = future.result()
        except Exception as error:
            self._send_goal_futures[controller] = None
            self._send_goal_started_ns[controller] = None
            self._set_fault(f"goal_response_failed:{type(error).__name__}")
            self._publish(
                f"failed:goal_response:{type(error).__name__}", controller
            )
            self._complete_typed(
                context,
                accepted=False,
                success=False,
                reason=(
                    f"goal_response_failed_stop_unconfirmed:{type(error).__name__}"
                    if tombstone is not None
                    else f"goal_response_failed:{type(error).__name__}"
                ),
            )
            if tombstone is None:
                self._release_controller_owner(controller, dispatch_id)
            return
        self._send_goal_futures[controller] = None
        self._send_goal_started_ns[controller] = None
        if goal_handle is None or not goal_handle.accepted:
            self._publish("rejected:downstream_goal_rejected", controller)
            if tombstone is not None:
                self._mark_tombstone_no_downstream_goal(
                    controller,
                    dispatch_id,
                    "late_downstream_goal_rejected",
                )
                self._publish(
                    "late_terminal_confirmed:downstream_goal_rejected:"
                    "explicit_reset_required",
                    controller,
                )
                return
            self._complete_typed(
                context,
                accepted=False,
                success=False,
                reason="downstream_goal_rejected",
            )
            self._release_controller_owner(controller, dispatch_id)
            return

        self._goal_handles[controller] = goal_handle
        if context is not None:
            with self._state_lock:
                context.accepted = True
        with self._state_lock:
            boundary_now_ns = self._now_ns()
            permission = self._permission.evaluate(boundary_now_ns)
            admission_reason, _ = self._admission_state(
                controller, boundary_now_ns
            )
            metadata_reason = (
                self._typed_metadata_reason(
                    context.goal_handle.request,
                    now_ns=boundary_now_ns,
                )
                if context is not None
                else None
            )
        cancel_reason: str | None = None
        if not permission.allowed:
            cancel_reason = permission.reason
        if cancel_reason is None and admission_reason is not None:
            cancel_reason = admission_reason
        if cancel_reason is None:
            cancel_reason = metadata_reason
        try:
            result_future = goal_handle.get_result_async()
        except Exception as error:
            detail = type(error).__name__
            self._set_fault(f"result_request_failed:{detail}")
            self._publish(
                f"failed:result_request:{detail}:stop_unconfirmed", controller
            )
            with self._state_lock:
                if context is not None and context.cancel_reason is None:
                    context.cancel_reason = "result_request_failed"
            self._latch_typed_tombstone(
                controller,
                dispatch_id,
                f"result_request_failed:{detail}",
            )
            self._complete_typed(
                context,
                accepted=True,
                success=False,
                reason=f"result_request_failed_stop_unconfirmed:{detail}",
            )
            self._cancel_controller(controller, "result_request_failed")
            return
        self._result_futures[controller] = result_future
        self._result_deadline_ns[controller] = (
            self._now_ns()
            + self._goal_duration_ns[controller]
            + self._result_timeout_margin_ns
        )
        result_future.add_done_callback(
            partial(
                self._on_result,
                controller,
                result_future,
                dispatch_id,
                command_id,
            )
        )
        self._publish("active:goal_accepted", controller)
        if self._cancel_on_accept[controller]:
            self._cancel_controller(controller, "cancel_on_accept")
        elif cancel_reason is not None:
            if context is not None and context.cancel_reason is None:
                context.cancel_reason = cancel_reason
            self._cancel_controller(controller, cancel_reason)

    def _on_result(
        self,
        controller: str,
        expected_future,
        dispatch_id: str,
        command_id: str | None,
        future,
    ) -> None:
        if (
            self._result_futures[controller] is not expected_future
            or not self._dispatch_is_current(controller, dispatch_id)
        ):
            self._publish("ignored:late_result", controller)
            return
        context = self._typed_context(controller, command_id)
        try:
            wrapped_result = future.result()
            status = int(
                getattr(wrapped_result, "status", GoalStatus.STATUS_UNKNOWN)
            )
            fjt_result = getattr(wrapped_result, "result", None)
            fjt_error_code = int(
                getattr(fjt_result, "error_code", FJT_ERROR_CODE_UNAVAILABLE)
            )
            fjt_error_string = str(getattr(fjt_result, "error_string", ""))
        except Exception as error:
            detail = type(error).__name__
            self._set_fault(f"result_failed:{detail}")
            self._publish(f"failed:result:{detail}:stop_unconfirmed", controller)
            self._result_futures[controller] = None
            self._result_deadline_ns[controller] = None
            with self._state_lock:
                if context is not None and context.cancel_reason is None:
                    context.cancel_reason = "result_failed"
            self._latch_typed_tombstone(
                controller,
                dispatch_id,
                f"result_failed:{detail}",
            )
            self._complete_typed(
                context,
                accepted=(self._goal_handles[controller] is not None),
                success=False,
                reason=f"result_failed_stop_unconfirmed:{detail}",
            )
            self._cancel_controller(controller, "result_failed")
            return
        tombstone = self._typed_tombstone(controller, dispatch_id)
        if tombstone is not None:
            downstream_terminal = status in {
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED,
            }
            if downstream_terminal:
                self._mark_tombstone_terminal(
                    controller,
                    dispatch_id,
                    action_goal_status=status,
                    evidence=(
                        f"late_fjt_terminal:status={status}:"
                        f"error_code={fjt_error_code}"
                    ),
                )
                self._clear_low_level_after_terminal(controller)
                self._publish(
                    f"late_terminal_confirmed:status={status}:"
                    f"fjt_error_code={fjt_error_code}:explicit_reset_required",
                    controller,
                )
            else:
                self._set_fault(
                    f"late_result_not_terminal:{controller}:{status}"
                )
                self._publish(
                    f"late_terminal_unconfirmed:status={status}:"
                    "explicit_recovery_required",
                    controller,
                )
            return
        wrapper_and_fjt_success = (
            status == GoalStatus.STATUS_SUCCEEDED
            and fjt_error_code == FollowJointTrajectory.Result.SUCCESSFUL
        )
        with self._state_lock:
            cancel_reason = context.cancel_reason if context is not None else None
        success = wrapper_and_fjt_success and cancel_reason is None
        if wrapper_and_fjt_success and cancel_reason is not None:
            reason = f"cancel_race_downstream_succeeded:{cancel_reason}"
            self._set_fault(f"cancel_race_downstream_succeeded:{controller}")
        elif success:
            reason = "succeeded"
        elif status == GoalStatus.STATUS_CANCELED:
            if cancel_reason is None:
                reason = "unexpected_downstream_canceled"
                self._set_fault(f"unexpected_downstream_canceled:{controller}")
            else:
                reason = f"canceled:{cancel_reason}"
        elif status != GoalStatus.STATUS_SUCCEEDED:
            reason = f"downstream_action_status:{status}"
            self._set_fault(f"downstream_action_failed:{controller}:{status}")
        else:
            reason = f"fjt_error:{fjt_error_code}"
            self._set_fault(
                f"fjt_execution_failed:{controller}:{fjt_error_code}"
            )
        sanitized_error = fjt_error_string.replace(";", ",")
        self._publish(
            f"completed:status={status}:fjt_error_code={fjt_error_code}:"
            f"success={str(success).lower()}:error={sanitized_error}",
            controller,
        )
        self._complete_typed(
            context,
            accepted=True,
            action_goal_status=status,
            fjt_error_code=fjt_error_code,
            fjt_error_string=fjt_error_string,
            downstream_terminal_observed=True,
            success=success,
            reason=reason,
        )
        self._release_controller_owner(controller, dispatch_id)
        self._clear_low_level_after_terminal(controller)

    def _watchdog(self) -> None:
        with self._state_lock:
            now_ns = self._now_ns()
            permission = self._permission.evaluate(now_ns)
            admission_reasons = {
                controller: self._admission_state(controller, now_ns)[0]
                for controller in CONTROLLERS
            }
        if not permission.allowed:
            self._cancel_all(permission.reason)
        for controller in CONTROLLERS:
            admission_reason = admission_reasons[controller]
            if admission_reason is not None:
                self._cancel_controller(controller, admission_reason)

            send_future = self._send_goal_futures[controller]
            send_started_ns = self._send_goal_started_ns[controller]
            if (
                send_future is not None
                and not send_future.done()
                and send_started_ns is not None
                and now_ns - send_started_ns > self._action_response_timeout_ns
            ):
                self._cancel_on_accept[controller] = True
                self._set_fault(f"goal_response_timeout:{controller}")
                command = self._typed_commands.get(controller)
                self._latch_typed_tombstone(
                    controller,
                    command.command_id
                    if command is not None
                    else self._controller_owners.get(controller),
                )
                self._complete_typed(
                    command,
                    accepted=False,
                    success=False,
                    reason="goal_response_timeout_stop_unconfirmed",
                )

            result_deadline_ns = self._result_deadline_ns[controller]
            if (
                self._goal_handles[controller] is not None
                and result_deadline_ns is not None
                and now_ns > result_deadline_ns
            ):
                self._result_deadline_ns[controller] = None
                if self._cancel_requested[controller]:
                    self._handle_cancel_failure(
                        controller, "accepted_but_goal_still_active"
                    )
                else:
                    self._set_fault(f"result_timeout:{controller}")
                    self._cancel_controller(controller, "result_timeout")

            cancel_future = self._cancel_futures[controller]
            cancel_started_ns = self._cancel_started_ns[controller]
            if (
                cancel_future is not None
                and not cancel_future.done()
                and cancel_started_ns is not None
                and now_ns - cancel_started_ns > self._cancel_response_timeout_ns
            ):
                self._handle_cancel_failure(controller, "timeout")

            retry_due_ns = self._cancel_retry_due_ns[controller]
            if retry_due_ns is not None and now_ns >= retry_due_ns:
                self._cancel_retry_due_ns[controller] = None
                self._cancel_controller(controller, "cancel_retry")

    def _cancel_all(self, reason: str) -> None:
        for controller in CONTROLLERS:
            self._cancel_controller(controller, reason)

    def _cancel_controller(self, controller: str, reason: str) -> None:
        context = self._typed_commands.get(controller)
        if context is not None:
            with self._state_lock:
                if not context.terminal and context.cancel_reason is None:
                    context.cancel_reason = reason
        pending = self._send_goal_futures[controller]
        if pending is not None and self._goal_handles[controller] is None:
            self._cancel_on_accept[controller] = True
        goal_handle = self._goal_handles[controller]
        if goal_handle is None:
            self._publish(f"blocked:{reason}", controller)
            return
        if self._cancel_exhausted[controller]:
            return
        if (
            self._cancel_retry_due_ns[controller] is not None
            and reason != "cancel_retry"
        ):
            return
        if self._cancel_requested[controller]:
            return
        self._cancel_requested[controller] = True
        self._cancel_attempts[controller] += 1
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as error:
            self._handle_cancel_failure(controller, type(error).__name__)
            return
        self._cancel_futures[controller] = cancel_future
        self._cancel_started_ns[controller] = self._now_ns()
        cancel_future.add_done_callback(
            partial(self._on_cancel_response, controller, cancel_future)
        )
        self._publish(f"cancelling:{reason}", controller)

    def _on_cancel_response(self, controller: str, expected_future, future) -> None:
        if self._cancel_futures[controller] is not expected_future:
            return
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", ()))
            if goals_canceling <= 0:
                self._handle_cancel_failure(controller, "rejected")
                return
            self._cancel_futures[controller] = None
            self._cancel_started_ns[controller] = None
            self._cancel_retry_due_ns[controller] = None
            # An accepted cancel response is only an acknowledgement.  Require
            # the action result to confirm a terminal state within a separate
            # bounded interval; otherwise retry and finally latch escalation.
            self._result_deadline_ns[controller] = (
                self._now_ns() + self._cancel_completion_timeout_ns
            )
            self._publish(f"cancel_accepted:goals={goals_canceling}", controller)
        except Exception as error:
            self._handle_cancel_failure(controller, type(error).__name__)

    def _handle_cancel_failure(self, controller: str, detail: str) -> None:
        self._cancel_requested[controller] = False
        self._cancel_futures[controller] = None
        self._cancel_started_ns[controller] = None
        self._result_deadline_ns[controller] = None
        self._set_fault(f"cancel_failed:{controller}:{detail}")
        if (
            self._goal_handles[controller] is not None
            and self._cancel_attempts[controller] < self._max_cancel_attempts
        ):
            self._cancel_retry_due_ns[controller] = self._now_ns() + self._cancel_retry_ns
            self._publish(
                f"ERROR:cancel_retry_scheduled:{detail}:"
                f"attempt={self._cancel_attempts[controller]}",
                controller,
            )
            return
        self._cancel_exhausted[controller] = True
        self._cancel_retry_due_ns[controller] = None
        self._publish(
            f"ERROR:cancel_failed_stop_escalation_required:{detail}", controller
        )
        command = self._typed_commands.get(controller)
        self._latch_typed_tombstone(
            controller,
            command.command_id
            if command is not None
            else self._controller_owners.get(controller),
        )
        self._complete_typed(
            command,
            accepted=(self._goal_handles[controller] is not None),
            success=False,
            reason=f"cancel_failed_stop_unconfirmed:{detail}",
        )

    def _on_reset(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        with self._state_lock:
            unconfirmed_tombstone = any(
                not tombstone.terminal_confirmed
                for tombstone in self._typed_tombstones.values()
            )
            non_tombstone_owner = any(
                owner is not None
                and (
                    controller not in self._typed_tombstones
                    or self._typed_tombstones[controller].command_id != owner
                )
                for controller, owner in self._controller_owners.items()
            )
            active = (
                bool(self._typed_reservations)
                or unconfirmed_tombstone
                or non_tombstone_owner
                or any(
                    self._goal_handles[controller] is not None
                    or self._send_goal_futures[controller] is not None
                    or self._result_futures[controller] is not None
                    or self._cancel_futures[controller] is not None
                    or self._cancel_requested[controller]
                    or self._cancel_retry_due_ns[controller] is not None
                    or self._typed_commands[controller] is not None
                    for controller in CONTROLLERS
                )
            )
        if active:
            self._cancel_all("reset_requested")
            response.success = False
            response.message = "reset refused while a downstream goal may be active"
            return response
        with self._state_lock:
            self._clock_epoch += 1
            now_ns = self._now_ns()
            self._permission.reset_epoch(now_ns)
            self._interface_permission.reset_epoch(now_ns)
            self._last_target_receive_ns = None
            self._last_joint_state_ns = None
            self._joint_positions = {}
            self._fault_latched = None
            self._seen_command_ids.clear()
            self._typed_tombstones.clear()
            for controller in CONTROLLERS:
                self._controller_owners[controller] = None
                self._cancel_attempts[controller] = 0
                self._cancel_retry_due_ns[controller] = None
                self._cancel_exhausted[controller] = False
        self._publish("blocked:epoch_reset_waiting_for_permission")
        response.success = True
        response.message = f"trajectory gate reset to local clock epoch {self._clock_epoch}"
        return response

    def destroy_node(self) -> bool:
        self._execute_server.destroy()
        for client in self._goal_clients.values():
            destroy = getattr(client, "destroy", None)
            if callable(destroy):
                destroy()
        return super().destroy_node()

    def _publish(self, status: str, controller: str = "all") -> None:
        message = String()
        message.data = (
            f"{status};controller={controller};target_frame={self._target_frame};"
            f"clock_domain={self._clock_domain};clock_epoch={self._clock_epoch}"
        )
        self._status.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrajectoryGate()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown(timeout_sec=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
