"""Fail-closed MoveGroup plan-only adapter for an immutable target request."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, replace
from functools import partial
import json
import math
from threading import Event, Lock, Timer
import time
import uuid

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from edgegrasp.config import (
    DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS,
    DEFAULT_TARGET_FRAME,
    ROS_SYSTEM_CLOCK_DOMAIN,
    validate_ros_clock_domain,
)
from edgegrasp.permission import MotionPermissionGate
from edgegrasp.so101_contract import SO101_ARM_JOINTS, SO101_PLANNING_PIPELINES
from edgegrasp.trajectory_identity import (
    make_trajectory_command_id,
    trajectory_digest,
    validate_trajectory_command_id,
)
from edgegrasp_interfaces.action import ExecuteTrajectory, PlanTarget
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory

from .trajectory_validation import validate_and_convert_robot_trajectory


PINNED_PLANNING_PIPELINES = frozenset(SO101_PLANNING_PIPELINES)
ARM_EXECUTION_STAGES = {"approach", "descend", "lift", "place", "retreat", "diagnostic"}
_STATUS_ATTEMPT_UNSET = object()


@dataclass(frozen=True, slots=True)
class _GateOutcome:
    dispatched: bool = False
    trajectory_digest: str = ""
    accepted: bool = False
    terminal: bool = False
    downstream_terminal_observed: bool = False
    cancel_requested: bool = False
    action_goal_status: int = GoalStatus.STATUS_UNKNOWN
    fjt_error_code: int = -(2**31)
    fjt_error_string: str = ""
    success: bool = False
    reason: str = "not_dispatched"


@dataclass(frozen=True, slots=True)
class _PlanningSceneStatus:
    ready: bool
    allow_target_pad_contacts: bool
    reason: str
    scene_digest: str
    observed_at_ns: int


class MoveItPlanOnlyAdapter(Node):
    """Plan one arm target at a time and publish only to the EdgeGrasp gate."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__("edgegrasp_moveit_plan_only_adapter", **node_kwargs)
        # Late-result watchdogs run on daemon Timer threads.  Serialize the
        # final publisher access with shutdown so a watchdog can either
        # publish before teardown or observe the destroying flag, but can
        # never race a destroyed rclpy publisher handle.
        self._destroying = Event()
        self._status_publish_lock = Lock()
        self._late_terminal_watchdog_lock = Lock()
        self._late_terminal_watchdogs: set[Timer] = set()
        self.declare_parameter("plan_target_action", "/edgegrasp/plan_target")
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter(
            "execute_trajectory_action", "/edgegrasp/execute_trajectory"
        )
        self.declare_parameter("motion_allowed_topic", "/edgegrasp/motion_allowed")
        self.declare_parameter("interface_ready_topic", "/edgegrasp/interface_ready")
        self.declare_parameter(
            "planning_scene_ready_topic", "/edgegrasp/planning_scene_ready"
        )
        self.declare_parameter(
            "planning_scene_status_topic", "/edgegrasp/planning_scene_status"
        )
        self.declare_parameter(
            "target_pad_contact_service", "/edgegrasp/set_target_pad_contacts"
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("planning_frame", DEFAULT_TARGET_FRAME)
        self.declare_parameter("end_effector_link", "gripper_frame_link")
        self.declare_parameter("clock_domain", ROS_SYSTEM_CLOCK_DOMAIN)
        self.declare_parameter("clock_epoch", 0)
        self.declare_parameter("target_freshness_timeout_ms", 200.0)
        self.declare_parameter(
            "future_skew_tolerance_ms", DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS
        )
        self.declare_parameter("permission_timeout_ms", 100.0)
        self.declare_parameter("interface_timeout_ms", 500.0)
        self.declare_parameter("planning_scene_timeout_ms", 1500.0)
        self.declare_parameter("contact_policy_timeout_ms", 5000.0)
        self.declare_parameter("joint_state_timeout_ms", 250.0)
        self.declare_parameter("tf_lookup_timeout_ms", 50.0)
        self.declare_parameter("compute_ik_service", "/compute_ik")
        self.declare_parameter("ik_service_discovery_timeout_ms", 1000.0)
        # This is a wall-clock transport guard, not the target-freshness
        # policy.  Gazebo can run well below real time, so a 50 ms simulated
        # KDL solve may legitimately take more than 150 ms of wall time.  The
        # polling loop still checks the independent 200 ms ROS-clock target
        # freshness boundary on every iteration and fails closed there.
        self.declare_parameter("ik_response_timeout_ms", 5000.0)
        self.declare_parameter("ik_solver_timeout_ms", 50.0)
        self.declare_parameter("ik_avoid_collisions", True)
        self.declare_parameter("move_group_discovery_timeout_ms", 1000.0)
        self.declare_parameter("goal_response_timeout_ms", 1000.0)
        self.declare_parameter("cancel_response_timeout_ms", 500.0)
        self.declare_parameter("result_timeout_margin_ms", 500.0)
        # A MoveGroup cancellation response only acknowledges the cancel
        # request.  Keep a separate, bounded wall-clock wait for the same
        # accepted goal's result future so a late success cannot reach the
        # trajectory gate without terminal evidence.
        self.declare_parameter("move_group_terminal_timeout_ms", 500.0)
        self.declare_parameter("gate_discovery_timeout_ms", 1000.0)
        self.declare_parameter("gate_result_timeout_margin_ms", 3000.0)
        # The typed gate owns ROS-time execution liveness and cancellation.
        # This separate wall guard only bounds a crashed gate process.  A slow
        # headless WSL Gazebo can require >15 s of wall time for an 8 s
        # simulated trajectory, so keep a larger but still finite outer bound.
        self.declare_parameter("gate_terminal_wall_guard_ms", 120000.0)
        self.declare_parameter("max_planning_timeout_s", 2.0)
        self.declare_parameter("max_planning_attempts", 3)
        self.declare_parameter("max_request_scaling", 0.2)
        self.declare_parameter("joint_goal_tolerance_rad", 0.005)
        self.declare_parameter("trajectory_start_tolerance_rad", 0.02)

        self._planning_frame = str(self.get_parameter("planning_frame").value)
        self._end_effector_link = str(
            self.get_parameter("end_effector_link").value
        )
        self._clock_domain = str(self.get_parameter("clock_domain").value)
        self._clock_epoch = int(self.get_parameter("clock_epoch").value)
        validate_ros_clock_domain(
            bool(self.get_parameter("use_sim_time").value), self._clock_domain
        )
        if (
            not self._planning_frame
            or not self._end_effector_link
            or self._clock_epoch < 0
        ):
            raise ValueError("planning frame/end-effector/clock epoch must be valid")

        self._target_timeout_ns = self._milliseconds("target_freshness_timeout_ms")
        self._future_skew_tolerance_ns = self._milliseconds(
            "future_skew_tolerance_ms"
        )
        self._joint_timeout_ns = self._milliseconds("joint_state_timeout_ms")
        self._contact_policy_timeout_s = self._seconds(
            "contact_policy_timeout_ms"
        )
        self._tf_timeout_s = self._seconds("tf_lookup_timeout_ms")
        self._ik_service_discovery_s = self._seconds(
            "ik_service_discovery_timeout_ms"
        )
        self._ik_response_s = self._seconds("ik_response_timeout_ms")
        self._ik_solver_timeout_s = self._seconds("ik_solver_timeout_ms")
        self._ik_avoid_collisions = bool(
            self.get_parameter("ik_avoid_collisions").value
        )
        self._move_group_discovery_s = self._seconds(
            "move_group_discovery_timeout_ms"
        )
        self._goal_response_s = self._seconds("goal_response_timeout_ms")
        self._cancel_response_s = self._seconds("cancel_response_timeout_ms")
        self._result_margin_s = self._seconds("result_timeout_margin_ms")
        self._move_group_terminal_s = self._seconds(
            "move_group_terminal_timeout_ms"
        )
        self._gate_discovery_s = self._seconds("gate_discovery_timeout_ms")
        self._gate_result_margin_s = self._seconds(
            "gate_result_timeout_margin_ms"
        )
        self._gate_terminal_wall_guard_s = self._seconds(
            "gate_terminal_wall_guard_ms"
        )
        self._max_planning_timeout_s = float(
            self.get_parameter("max_planning_timeout_s").value
        )
        self._max_planning_attempts = int(
            self.get_parameter("max_planning_attempts").value
        )
        self._max_request_scaling = float(
            self.get_parameter("max_request_scaling").value
        )
        self._joint_goal_tolerance_rad = float(
            self.get_parameter("joint_goal_tolerance_rad").value
        )
        self._start_tolerance_rad = float(
            self.get_parameter("trajectory_start_tolerance_rad").value
        )
        if any(
            value <= 0.0
            for value in (
                self._max_planning_timeout_s,
                self._max_request_scaling,
                self._joint_goal_tolerance_rad,
                self._start_tolerance_rad,
            )
        ) or self._max_planning_attempts < 1:
            raise ValueError("adapter safety limits must be positive")

        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = Lock()
        self._permission = MotionPermissionGate(
            float(self.get_parameter("permission_timeout_ms").value)
        )
        self._interface_permission = MotionPermissionGate(
            float(self.get_parameter("interface_timeout_ms").value)
        )
        self._planning_scene_permission = MotionPermissionGate(
            float(self.get_parameter("planning_scene_timeout_ms").value)
        )
        self._last_clock_ns: int | None = None
        self._joint_positions: dict[str, float] = {}
        self._last_joint_receive_ns: int | None = None
        self._fault_latched: str | None = None
        self._reserved_request_id: str | None = None
        self._active_request_id: str | None = None
        self._attempt_generation_counter = 0
        self._active_attempt_generation: int | None = None
        self._active_moveit_goal = None
        self._active_moveit_goal_id: str | None = None
        self._active_moveit_result_future = None
        self._dispatched_command_ids: set[str] = set()
        self._planning_scene_status: _PlanningSceneStatus | None = None
        self._planning_scene_status_generation = 0
        self._planning_scene_status_event = Event()

        self._status = self.create_publisher(
            String, "/edgegrasp/moveit_adapter_status", 10
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("motion_allowed_topic").value),
            self._on_permission,
            10,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("interface_ready_topic").value),
            self._on_interface_ready,
            10,
            callback_group=self._callback_group,
        )
        scene_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("planning_scene_status_topic").value),
            self._on_planning_scene_status,
            scene_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("planning_scene_ready_topic").value),
            self._on_planning_scene_ready,
            scene_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(
            Trigger,
            "/edgegrasp/reset_moveit_adapter_epoch",
            self._on_reset,
            callback_group=self._callback_group,
        )
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self._compute_ik = self.create_client(
            GetPositionIK,
            str(self.get_parameter("compute_ik_service").value),
            callback_group=self._callback_group,
        )
        self._target_pad_contacts = self.create_client(
            SetBool,
            str(self.get_parameter("target_pad_contact_service").value),
            callback_group=self._callback_group,
        )
        self._move_group = ActionClient(
            self,
            MoveGroup,
            str(self.get_parameter("move_group_action").value),
            callback_group=self._callback_group,
        )
        self._trajectory_gate = ActionClient(
            self,
            ExecuteTrajectory,
            str(self.get_parameter("execute_trajectory_action").value),
            callback_group=self._callback_group,
        )
        self._plan_server = ActionServer(
            self,
            PlanTarget,
            str(self.get_parameter("plan_target_action").value),
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._callback_group,
        )

    def _milliseconds(self, parameter_name: str) -> int:
        value = float(self.get_parameter(parameter_name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{parameter_name} must be positive")
        return int(value * 1_000_000)

    def _seconds(self, millisecond_parameter: str) -> float:
        return self._milliseconds(millisecond_parameter) / 1_000_000_000.0

    @staticmethod
    def _request_id(request: PlanTarget.Goal) -> str:
        return make_trajectory_command_id(
            request.task_id,
            request.stage,
            int(request.sequence_no),
        )

    @staticmethod
    def _goal_id_hex(goal_handle) -> str | None:
        """Return a ROS action goal UUID as stable lowercase hexadecimal.

        ``rclpy`` exposes ``ClientGoalHandle.goal_id`` as a
        ``unique_identifier_msgs/UUID`` message, while the small fake
        dependencies used by the injected runner may expose ``uuid.UUID`` or
        raw bytes.  Normalize all of those representations at the adapter
        boundary so status records can join a request with the exact action
        goal that was accepted.
        """

        goal_id = getattr(goal_handle, "goal_id", None)
        if goal_id is None:
            return None
        raw = getattr(goal_id, "uuid", goal_id)
        try:
            if isinstance(raw, uuid.UUID):
                return raw.hex
            if isinstance(raw, str):
                return uuid.UUID(raw).hex
            if hasattr(raw, "bytes") and not isinstance(raw, (bytes, bytearray)):
                raw = raw.bytes
            value = bytes(raw)
        except (TypeError, ValueError, AttributeError):
            return None
        if len(value) != 16:
            return None
        return value.hex()

    def _active_matches(
        self, request_id: str, attempt_generation: int
    ) -> bool:
        with self._state_lock:
            return (
                self._active_request_id == request_id
                and self._active_attempt_generation == attempt_generation
            )

    def _new_attempt(self, request_id: str) -> int:
        """Reserve a monotonically increasing execution generation."""

        with self._state_lock:
            self._attempt_generation_counter += 1
            attempt_generation = self._attempt_generation_counter
            self._active_request_id = request_id
            self._active_attempt_generation = attempt_generation
            self._active_moveit_goal = None
            self._active_moveit_goal_id = None
            self._active_moveit_result_future = None
        return attempt_generation

    def _latch_active_attempt_fault(
        self,
        reason: str,
        request_id: str | None,
        attempt_generation: int | None,
    ) -> None:
        """Latch a callback fault only while its captured attempt is active."""

        if request_id is None or attempt_generation is None:
            return
        with self._state_lock:
            if (
                self._active_request_id != request_id
                or self._active_attempt_generation != attempt_generation
            ):
                return
            if self._fault_latched is None:
                self._fault_latched = reason
        # The identity check and fault mutation above are one atomic state
        # transition.  Publish only the callback's captured identity after
        # releasing the lock; a newer attempt cannot inherit this fault.
        self._publish_status(
            "ERROR",
            reason,
            request_id,
            attempt_generation=attempt_generation,
        )

    def _observe_now(self) -> tuple[int, str | None]:
        with self._state_lock:
            # Read and commit the observation under one lock.  Concurrent
            # callbacks must not read T1, pause, then overwrite a newer T2 as
            # an apparent clock rollback when they resume out of order.
            now_ns = self.get_clock().now().nanoseconds
            if self._last_clock_ns is not None and now_ns < self._last_clock_ns:
                self._fault_latched = (
                    f"clock_rollback:{now_ns}<{self._last_clock_ns}"
                )
            self._last_clock_ns = now_ns
            return now_ns, self._fault_latched

    def _latch_fault(self, reason: str) -> None:
        with self._state_lock:
            if self._fault_latched is None:
                self._fault_latched = reason
        self._publish_status("ERROR", reason)

    def _publish_status(
        self,
        stage: str,
        reason: str,
        request_id: str = "<none>",
        *,
        attempt_generation: int | None | object = _STATUS_ATTEMPT_UNSET,
        move_group_goal_id: str | None = None,
        result_future_pending_at_timeout: bool | None = None,
        goal_response_future_pending_at_timeout: bool | None = None,
        gate_goal_response_future_pending_at_timeout: bool | None = None,
        move_group_cancel_requested: bool | None = None,
        move_group_terminal_observed: bool | None = None,
        move_group_terminal_status: int | None = None,
        gate_goal_id: str | None = None,
        gate_cancel_requested: bool | None = None,
        gate_terminal_observed: bool | None = None,
        gate_terminal_status: int | None = None,
        action_goal_status: int | None = None,
        cancel_response_accepted: bool | None = None,
    ) -> None:
        # Request-specific callbacks pass their captured generation and goal
        # id explicitly.  For ordinary in-flight status, infer the current
        # values under the same lock used by the execute state machine.
        if attempt_generation is _STATUS_ATTEMPT_UNSET:
            with self._state_lock:
                attempt_generation = (
                    self._active_attempt_generation
                    if request_id == self._active_request_id
                    else None
                )
        message = String()
        message.data = json.dumps(
            {
                "stage": stage,
                "reason": reason,
                "request_id": request_id,
                "attempt_generation": attempt_generation,
                "move_group_goal_id": move_group_goal_id,
                "result_future_pending_at_timeout": (
                    result_future_pending_at_timeout
                ),
                "goal_response_future_pending_at_timeout": (
                    goal_response_future_pending_at_timeout
                ),
                "gate_goal_response_future_pending_at_timeout": (
                    gate_goal_response_future_pending_at_timeout
                ),
                "move_group_cancel_requested": move_group_cancel_requested,
                "move_group_terminal_observed": move_group_terminal_observed,
                "move_group_terminal_status": move_group_terminal_status,
                "gate_goal_id": gate_goal_id,
                "gate_cancel_requested": gate_cancel_requested,
                "gate_terminal_observed": gate_terminal_observed,
                "gate_terminal_status": gate_terminal_status,
                "action_goal_status": action_goal_status,
                "cancel_response_accepted": cancel_response_accepted,
                "clock_domain": self._clock_domain,
                "clock_epoch": self._clock_epoch,
                "plan_only": True,
            },
            sort_keys=True,
        )
        with self._status_publish_lock:
            if self._destroying.is_set():
                return
            self._status.publish(message)

    def _feedback(self, goal_handle, stage: str, request_id: str) -> None:
        feedback = PlanTarget.Feedback()
        feedback.stage = stage
        goal_handle.publish_feedback(feedback)
        self._publish_status(stage, "in_progress", request_id)

    def _on_permission(self, message: Bool) -> None:
        now_ns, fault = self._observe_now()
        if fault is None:
            self._permission.update(bool(message.data), now_ns)

    def _on_interface_ready(self, message: Bool) -> None:
        now_ns, fault = self._observe_now()
        if fault is None:
            self._interface_permission.update(bool(message.data), now_ns)

    def _on_planning_scene_ready(self, message: Bool) -> None:
        now_ns, fault = self._observe_now()
        if fault is None:
            self._planning_scene_permission.update(bool(message.data), now_ns)

    def _on_planning_scene_status(self, message: String) -> None:
        now_ns, fault = self._observe_now()
        if fault is not None:
            return
        try:
            payload = json.loads(message.data)
            if (
                not isinstance(payload, dict)
                or not isinstance(payload.get("ready"), bool)
                or not isinstance(payload.get("allow_target_pad_contacts"), bool)
                or not isinstance(payload.get("reason"), str)
                or not isinstance(payload.get("scene_digest"), str)
                or not payload["scene_digest"]
            ):
                raise ValueError("invalid planning-scene status fields")
            if payload.get("target_frame") != self._planning_frame:
                raise ValueError("planning-scene target frame mismatch")
            if payload.get("clock_domain") != self._clock_domain:
                raise ValueError("planning-scene clock domain mismatch")
            if payload.get("clock_epoch") != self._clock_epoch:
                raise ValueError("planning-scene clock epoch mismatch")
            sample = _PlanningSceneStatus(
                ready=payload["ready"],
                allow_target_pad_contacts=payload[
                    "allow_target_pad_contacts"
                ],
                reason=payload["reason"],
                scene_digest=payload["scene_digest"],
                observed_at_ns=now_ns,
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self._planning_scene_permission.update(False, now_ns)
            self._latch_fault(
                f"invalid_planning_scene_status:{type(error).__name__}"
            )
            return
        with self._state_lock:
            self._planning_scene_status = sample
            self._planning_scene_status_generation += 1
        self._planning_scene_status_event.set()

    def _set_target_pad_contact_policy(
        self, goal_handle, request: PlanTarget.Goal
    ) -> str | None:
        # The first physics-safe sequence descends with an open gripper and no
        # target-pad ACM exemption.  Contacts are enabled only after the
        # correlated descend terminal, before close; lift then observes the
        # already-confirmed allowed policy.
        allow = request.stage in {"lift", "place", "retreat"}
        now_ns, fault = self._observe_now()
        if fault is not None:
            return f"fault_latched:{fault}"
        with self._state_lock:
            current_status = self._planning_scene_status
            status_generation_before_call = self._planning_scene_status_generation
        if (
            current_status is not None
            and current_status.ready
            and current_status.reason == "confirmed"
            and current_status.allow_target_pad_contacts == allow
            and self._planning_scene_permission.evaluate(now_ns).allowed
        ):
            self._publish_status(
                "contact_policy_confirmed",
                "target_pad_contacts_allowed"
                if allow
                else "target_pad_contacts_disallowed",
                self._request_id(request),
            )
            return None
        if not self._target_pad_contacts.wait_for_service(
            timeout_sec=self._contact_policy_timeout_s
        ):
            return "target_pad_contact_policy_service_unavailable"
        policy_request = SetBool.Request()
        policy_request.data = allow
        try:
            future = self._target_pad_contacts.call_async(policy_request)
        except Exception as error:
            return f"target_pad_contact_policy_call_exception:{type(error).__name__}"
        deadline = time.monotonic() + self._contact_policy_timeout_s
        while not future.done() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                future.cancel()
                return "cancel_requested_during_contact_policy"
            time.sleep(0.005)
        if not future.done():
            future.cancel()
            return "target_pad_contact_policy_response_timeout"
        try:
            response = future.result()
        except Exception as error:
            return (
                "target_pad_contact_policy_response_exception:"
                f"{type(error).__name__}"
            )
        if response is None or not bool(response.success):
            detail = "no_response" if response is None else response.message
            return f"target_pad_contact_policy_rejected:{detail}"

        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return "cancel_requested_during_contact_policy_confirmation"
            with self._state_lock:
                status = self._planning_scene_status
                status_generation = self._planning_scene_status_generation
            now_ns, fault = self._observe_now()
            if fault is not None:
                return f"fault_latched:{fault}"
            if (
                status is not None
                and status_generation > status_generation_before_call
                and status.ready
                and status.reason == "confirmed"
                and status.allow_target_pad_contacts == allow
                and self._planning_scene_permission.evaluate(now_ns).allowed
            ):
                self._publish_status(
                    "contact_policy_confirmed",
                    "target_pad_contacts_allowed"
                    if allow
                    else "target_pad_contacts_disallowed",
                    self._request_id(request),
                )
                return None
            self._planning_scene_status_event.wait(0.01)
            self._planning_scene_status_event.clear()
        return "target_pad_contact_policy_confirmation_timeout"

    def _on_joint_state(self, message: JointState) -> None:
        now_ns, fault = self._observe_now()
        if fault is not None:
            return
        if (
            len(message.name) != len(message.position)
            or len(set(message.name)) != len(message.name)
            or any(not math.isfinite(float(value)) for value in message.position)
        ):
            self._latch_fault("invalid_joint_states")
            return
        positions = dict(zip(message.name, message.position, strict=True))
        if not set(SO101_ARM_JOINTS).issubset(positions):
            self._latch_fault("joint_states_missing_arm_joints")
            return
        with self._state_lock:
            self._joint_positions = {
                joint: float(positions[joint]) for joint in SO101_ARM_JOINTS
            }
            self._last_joint_receive_ns = now_ns

    def _on_goal(self, request: PlanTarget.Goal) -> GoalResponse:
        try:
            request_id = self._request_id(request)
        except ValueError:
            self._publish_status("rejected", "invalid_command_identity")
            return GoalResponse.REJECT
        with self._state_lock:
            concurrent_request = self._reserved_request_id is not None
            if not concurrent_request:
                self._reserved_request_id = request_id
        if concurrent_request:
            self._publish_status("rejected", "concurrent_request", request_id)
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def _validate_request(self, request: PlanTarget.Goal) -> str | None:
        try:
            validate_trajectory_command_id(
                self._request_id(request),
                request.task_id,
                request.stage,
                int(request.sequence_no),
            )
        except ValueError:
            return "invalid_command_identity"
        if request.stage not in ARM_EXECUTION_STAGES:
            return "stage_must_be_arm_phase"
        if not request.target_id:
            return "empty_target_id"
        if not request.plan_only:
            return "plan_only_required"
        if request.planning_group != "arm":
            return "planning_group_must_be_arm"
        if request.pipeline_id not in PINNED_PLANNING_PIPELINES:
            return "unsupported_planning_pipeline"
        if request.clock_domain != self._clock_domain:
            return "clock_domain_mismatch"
        if int(request.clock_epoch) != self._clock_epoch:
            return "clock_epoch_mismatch"
        stamp_ns = (
            request.target_pose.header.stamp.sec * 1_000_000_000
            + request.target_pose.header.stamp.nanosec
        )
        if request.source_timestamp_ns < 0 or stamp_ns != request.source_timestamp_ns:
            return "source_timestamp_mismatch"
        if not request.target_pose.header.frame_id:
            return "empty_target_frame"
        finite_pose = (
            request.target_pose.pose.position.x,
            request.target_pose.pose.position.y,
            request.target_pose.pose.position.z,
            request.target_pose.pose.orientation.x,
            request.target_pose.pose.orientation.y,
            request.target_pose.pose.orientation.z,
            request.target_pose.pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in finite_pose):
            return "nonfinite_target_pose"
        if (
            not math.isfinite(request.planning_timeout_s)
            or not 0.0 < request.planning_timeout_s <= self._max_planning_timeout_s
        ):
            return "planning_timeout_out_of_range"
        for name, value in (
            ("velocity_scaling", request.velocity_scaling),
            ("acceleration_scaling", request.acceleration_scaling),
        ):
            if (
                not math.isfinite(value)
                or not 0.0 < value <= self._max_request_scaling
            ):
                return f"{name}_out_of_range"
        return None

    def _fresh_start_positions(self, now_ns: int) -> tuple[float, ...] | None:
        with self._state_lock:
            # A joint callback may have advanced receive time since the caller
            # sampled now_ns. Compare one locked snapshot against the actual
            # current clock; do not clamp negative ages or extend the timeout.
            current_ns = self.get_clock().now().nanoseconds
            if current_ns < now_ns or (
                self._last_clock_ns is not None
                and current_ns < self._last_clock_ns
            ):
                self._fault_latched = f"clock_rollback:{current_ns}"
            self._last_clock_ns = current_ns
            if self._fault_latched is not None:
                return None
            receive_ns = self._last_joint_receive_ns
            positions = dict(self._joint_positions)
        if receive_ns is None or current_ns - receive_ns < 0:
            return None
        if current_ns - receive_ns > self._joint_timeout_ns:
            return None
        try:
            return tuple(positions[joint] for joint in SO101_ARM_JOINTS)
        except KeyError:
            return None

    def _safety_reason(
        self,
        request: PlanTarget.Goal,
        *,
        require_source_freshness: bool = True,
    ) -> str | None:
        now_ns, fault = self._observe_now()
        if fault is not None:
            return f"fault_latched:{fault}"
        permission = self._permission.evaluate(now_ns)
        if not permission.allowed:
            return f"permission:{permission.reason}"
        interface = self._interface_permission.evaluate(now_ns)
        if not interface.allowed:
            return f"interface:{interface.reason}"
        planning_scene = self._planning_scene_permission.evaluate(now_ns)
        if not planning_scene.allowed:
            return f"planning_scene:{planning_scene.reason}"
        if require_source_freshness:
            target_age_ns = now_ns - request.source_timestamp_ns
            if target_age_ns < 0:
                return "future_target"
            if target_age_ns > self._target_timeout_ns:
                return "stale_target"
        if request.clock_domain != self._clock_domain:
            return "clock_domain_mismatch"
        if int(request.clock_epoch) != self._clock_epoch:
            return "clock_epoch_mismatch"
        if self._fresh_start_positions(now_ns) is None:
            return "joint_states_missing_or_stale"
        if not self._trajectory_gate.server_is_ready():
            return "trajectory_gate_unavailable"
        return None

    def _wait_for_source_clock(self, request: PlanTarget.Goal) -> str | None:
        now_ns, fault = self._observe_now()
        if fault is not None:
            return f"fault_latched:{fault}"
        future_ns = request.source_timestamp_ns - now_ns
        if future_ns <= 0:
            return None
        if future_ns > self._future_skew_tolerance_ns:
            return "future_target_exceeds_clock_delivery_window"
        # No planning or motion is allowed here.  This bounded wall-clock wait
        # only lets this node observe the same ROS /clock sample as the sender.
        deadline = time.monotonic() + max(
            1.0, self._future_skew_tolerance_ns / 1_000_000_000.0 * 2.0
        )
        while self.context.ok() and time.monotonic() < deadline:
            now_ns, fault = self._observe_now()
            if fault is not None:
                return f"fault_latched:{fault}"
            if now_ns >= request.source_timestamp_ns:
                return None
            time.sleep(0.01)
        return "future_target_clock_catchup_timeout"

    def _transform_target(
        self, request: PlanTarget.Goal
    ) -> tuple[Pose | None, str | None]:
        orientation = request.target_pose.pose.orientation
        norm = math.sqrt(
            orientation.x**2
            + orientation.y**2
            + orientation.z**2
            + orientation.w**2
        )
        orientation_defined = norm > 1e-9
        if orientation_defined and abs(norm - 1.0) > 1e-3:
            return None, "invalid_target_orientation"
        pose = deepcopy(request.target_pose.pose)
        if not orientation_defined:
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
        try:
            transform = self._tf_buffer.lookup_transform(
                self._planning_frame,
                request.target_pose.header.frame_id,
                Time.from_msg(request.target_pose.header.stamp),
                timeout=Duration(seconds=self._tf_timeout_s),
            )
            transformed = do_transform_pose(pose, transform)
            if not orientation_defined:
                current_end_effector = self._tf_buffer.lookup_transform(
                    self._planning_frame,
                    self._end_effector_link,
                    Time.from_msg(request.target_pose.header.stamp),
                    timeout=Duration(seconds=self._tf_timeout_s),
                )
                transformed.orientation = current_end_effector.transform.rotation
        except TransformException as error:
            return None, f"tf2_transform_failed:{type(error).__name__}"
        return transformed, None

    @staticmethod
    def _extract_ik_arm_positions(response) -> tuple[tuple[float, ...] | None, str | None]:
        names = tuple(response.solution.joint_state.name)
        values = tuple(response.solution.joint_state.position)
        if len(names) != len(values) or len(set(names)) != len(names):
            return None, "ik_solution_malformed"
        positions_by_name = dict(zip(names, values, strict=True))
        if not set(SO101_ARM_JOINTS).issubset(positions_by_name):
            return None, "ik_solution_missing_arm_joints"
        arm_positions = tuple(
            float(positions_by_name[joint]) for joint in SO101_ARM_JOINTS
        )
        if not all(math.isfinite(value) for value in arm_positions):
            return None, "ik_solution_nonfinite"
        from edgegrasp.so101_contract import SO101_CONTRACT

        bounds = SO101_CONTRACT["edgegrasp_admission_profile"][
            "joint_position_bounds_rad"
        ]
        for joint, value in zip(SO101_ARM_JOINTS, arm_positions, strict=True):
            lower, upper = bounds[joint]
            if not float(lower) <= value <= float(upper):
                return None, f"ik_solution_position_limit:{joint}:{value}"
        return arm_positions, None

    def _solve_ik(
        self,
        goal_handle,
        request: PlanTarget.Goal,
        transformed_pose: Pose,
        start_positions: tuple[float, ...],
    ) -> tuple[tuple[float, ...] | None, str | None, int]:
        if not self._compute_ik.wait_for_service(
            timeout_sec=self._ik_service_discovery_s
        ):
            return None, "compute_ik_unavailable", 0
        service_request = GetPositionIK.Request()
        service_request.ik_request.group_name = "arm"
        service_request.ik_request.ik_link_name = self._end_effector_link
        service_request.ik_request.robot_state.is_diff = True
        seed = service_request.ik_request.robot_state.joint_state
        seed.header.stamp = request.target_pose.header.stamp
        seed.name = list(SO101_ARM_JOINTS)
        seed.position = list(start_positions)
        service_request.ik_request.avoid_collisions = self._ik_avoid_collisions
        pose = PoseStamped()
        pose.header.frame_id = self._planning_frame
        pose.header.stamp = request.target_pose.header.stamp
        pose.pose = transformed_pose
        service_request.ik_request.pose_stamped = pose
        service_request.ik_request.timeout = Duration(
            seconds=self._ik_solver_timeout_s
        ).to_msg()
        try:
            future = self._compute_ik.call_async(service_request)
        except Exception as error:
            self._latch_fault(f"compute_ik_call_exception:{type(error).__name__}")
            return None, "compute_ik_call_exception", 0
        reason = self._poll_future(
            future,
            time.monotonic() + self._ik_response_s,
            goal_handle,
            request,
            timeout_reason="compute_ik_timeout",
        )
        if reason is not None:
            return None, reason, 0
        try:
            response = future.result()
        except Exception as error:
            self._latch_fault(f"compute_ik_response_exception:{type(error).__name__}")
            return None, "compute_ik_response_exception", 0
        if response is None:
            return None, "compute_ik_empty_response", 0
        error_code = int(response.error_code.val)
        if error_code != MoveItErrorCodes.SUCCESS:
            detail = ":".join(
                part
                for part in (
                    str(getattr(response.error_code, "message", "")),
                    str(getattr(response.error_code, "source", "")),
                )
                if part
            )
            return None, f"compute_ik_error:{error_code}:{detail}", error_code
        positions, reason = self._extract_ik_arm_positions(response)
        return positions, reason, error_code

    def _make_move_group_goal(
        self,
        request: PlanTarget.Goal,
        goal_positions: tuple[float, ...],
        start_positions: tuple[float, ...],
    ) -> MoveGroup.Goal:
        constraint = Constraints()
        constraint.name = f"edgegrasp:{self._request_id(request)}"
        constraint.joint_constraints = []
        for joint, position in zip(
            SO101_ARM_JOINTS, goal_positions, strict=True
        ):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint
            joint_constraint.position = position
            joint_constraint.tolerance_above = self._joint_goal_tolerance_rad
            joint_constraint.tolerance_below = self._joint_goal_tolerance_rad
            joint_constraint.weight = 1.0
            constraint.joint_constraints.append(joint_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.pipeline_id = request.pipeline_id
        goal.request.planner_id = request.planner_id
        goal.request.num_planning_attempts = 1
        goal.request.allowed_planning_time = request.planning_timeout_s
        goal.request.max_velocity_scaling_factor = request.velocity_scaling
        goal.request.max_acceleration_scaling_factor = request.acceleration_scaling
        goal.request.start_state.is_diff = True
        goal.request.start_state.joint_state.header.stamp = (
            request.target_pose.header.stamp
        )
        goal.request.start_state.joint_state.name = list(SO101_ARM_JOINTS)
        goal.request.start_state.joint_state.position = list(start_positions)
        goal.request.goal_constraints = [constraint]
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        return goal

    @staticmethod
    def _future_done(future) -> bool:
        return future is not None and future.done()

    @staticmethod
    def _get_move_group_result_future(goal_handle):
        """Request the accepted MoveGroup result through an injectable seam."""

        return goal_handle.get_result_async()

    @staticmethod
    def _get_gate_result_future(goal_handle):
        """Request the accepted typed-gate result through an injectable seam."""

        return goal_handle.get_result_async()

    @staticmethod
    def _is_terminal_status(status: int) -> bool:
        return int(status) in {
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        }

    @classmethod
    def _result_future_terminal_observed(cls, result_future) -> bool:
        if result_future is None or not result_future.done():
            return False
        try:
            wrapped = result_future.result()
            return cls._is_terminal_status(
                int(getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN))
            )
        except Exception:
            return False

    @staticmethod
    def _result_future_status(result_future) -> int | None:
        if result_future is None or not result_future.done():
            return None
        try:
            wrapped = result_future.result()
            return int(getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN))
        except Exception:
            return None

    def _poll_future(
        self,
        future,
        deadline: float,
        goal_handle,
        request: PlanTarget.Goal,
        *,
        timeout_reason: str = "move_group_timeout",
        require_source_freshness: bool = True,
    ) -> str | None:
        while self.context.ok() and time.monotonic() < deadline:
            if self._future_done(future):
                return None
            if goal_handle.is_cancel_requested:
                return "plan_target_cancel_requested"
            reason = self._safety_reason(
                request,
                require_source_freshness=require_source_freshness,
            )
            if reason is not None:
                return reason
            time.sleep(0.01)
        return timeout_reason

    def _wait_for_move_group_terminal(
        self,
        result_future,
        request_id: str,
        attempt_generation: int | None,
        move_group_goal_id: str | None,
        *,
        pending_at_timeout: bool | None,
    ) -> bool:
        """Wait for the exact accepted MoveGroup goal to become terminal.

        A cancel response is not terminal evidence.  This wait deliberately
        uses the result future obtained from the same ``ClientGoalHandle``
        that was canceled, and it is fail-closed when that future remains
        pending.  In particular, a later success is observed as a canceled
        attempt and is never allowed to continue into gate dispatch.
        """

        if result_future is None:
            self._latch_active_attempt_fault(
                "move_group_terminal_unconfirmed",
                request_id,
                attempt_generation,
            )
            self._publish_status(
                "move_group_terminal_unconfirmed",
                "result_future_unavailable",
                request_id,
                attempt_generation=attempt_generation,
                move_group_goal_id=move_group_goal_id,
                result_future_pending_at_timeout=pending_at_timeout,
                move_group_cancel_requested=True,
                move_group_terminal_observed=False,
            )
            return False
        deadline = time.monotonic() + self._move_group_terminal_s
        while (
            self.context.ok()
            and time.monotonic() < deadline
            and not result_future.done()
        ):
            time.sleep(0.01)
        if not result_future.done():
            self._latch_active_attempt_fault(
                "move_group_terminal_unconfirmed",
                request_id,
                attempt_generation,
            )
            self._publish_status(
                "move_group_terminal_unconfirmed",
                "result_future_timeout",
                request_id,
                attempt_generation=attempt_generation,
                move_group_goal_id=move_group_goal_id,
                result_future_pending_at_timeout=pending_at_timeout,
                move_group_cancel_requested=True,
                move_group_terminal_observed=False,
            )
            return False
        try:
            wrapped = result_future.result()
            status = int(getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN))
        except Exception as error:
            self._latch_active_attempt_fault(
                f"move_group_terminal_result_exception:{type(error).__name__}",
                request_id,
                attempt_generation,
            )
            self._publish_status(
                "move_group_terminal_unconfirmed",
                f"result_exception:{type(error).__name__}",
                request_id,
                attempt_generation=attempt_generation,
                move_group_goal_id=move_group_goal_id,
                result_future_pending_at_timeout=pending_at_timeout,
                move_group_cancel_requested=True,
                move_group_terminal_observed=False,
            )
            return False
        terminal = self._is_terminal_status(status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            # A success arriving after cancellation is a race, not evidence
            # that the canceled attempt may continue.  Keep it observable and
            # fail closed for the remainder of this attempt.
            self._latch_active_attempt_fault(
                "move_group_cancel_success_race",
                request_id,
                attempt_generation,
            )
        self._publish_status(
            "move_group_terminal_observed" if terminal else "move_group_terminal_unconfirmed",
            "success_after_cancel"
            if status == GoalStatus.STATUS_SUCCEEDED
            else "observed_after_cancel"
            if terminal
            else "nonterminal_status",
            request_id,
            attempt_generation=attempt_generation,
            move_group_goal_id=move_group_goal_id,
            result_future_pending_at_timeout=pending_at_timeout,
            move_group_cancel_requested=True,
            move_group_terminal_observed=terminal,
            move_group_terminal_status=status,
            action_goal_status=status,
        )
        if not terminal:
            self._latch_active_attempt_fault(
                "move_group_terminal_unconfirmed",
                request_id,
                attempt_generation,
            )
        return terminal and status != GoalStatus.STATUS_SUCCEEDED

    def _cancel_move_group(
        self,
        request_id: str,
        reason: str,
        *,
        attempt_generation: int | None = None,
        moveit_goal=None,
        result_future=None,
    ) -> bool:
        explicit_moveit_goal = moveit_goal is not None
        with self._state_lock:
            active_matches = (
                attempt_generation is None
                or (
                    self._active_request_id == request_id
                    and self._active_attempt_generation == attempt_generation
                )
            )
            if moveit_goal is None and active_matches:
                moveit_goal = self._active_moveit_goal
            if (
                result_future is None
                and active_matches
                and not explicit_moveit_goal
            ):
                result_future = self._active_moveit_result_future
            move_group_goal_id = self._goal_id_hex(moveit_goal)
            if (
                move_group_goal_id is None
                and active_matches
                and not explicit_moveit_goal
            ):
                move_group_goal_id = self._active_moveit_goal_id
        pending_at_timeout = (
            None if result_future is None else not result_future.done()
        )
        if moveit_goal is None:
            if result_future is None:
                return self._wait_for_move_group_terminal(
                    None,
                    request_id,
                    attempt_generation,
                move_group_goal_id,
                pending_at_timeout=None,
                )
            return self._wait_for_move_group_terminal(
                result_future,
                request_id,
                attempt_generation,
                move_group_goal_id,
                pending_at_timeout=pending_at_timeout,
            )
        cancel_requested = False
        cancel_accepted = False
        try:
            future = moveit_goal.cancel_goal_async()
            cancel_requested = True
        except Exception as error:
            self._latch_active_attempt_fault(
                f"move_group_cancel_exception:{type(error).__name__}",
                request_id,
                attempt_generation,
            )
        else:
            deadline = time.monotonic() + self._cancel_response_s
            while (
                self.context.ok()
                and time.monotonic() < deadline
                and not future.done()
            ):
                time.sleep(0.01)
            if not future.done():
                self._latch_active_attempt_fault(
                    "move_group_cancel_timeout",
                    request_id,
                    attempt_generation,
                )
            else:
                try:
                    response = future.result()
                    cancel_accepted = bool(
                        getattr(response, "goals_canceling", ())
                    )
                    if not cancel_accepted:
                        self._latch_active_attempt_fault(
                            "move_group_cancel_rejected",
                            request_id,
                            attempt_generation,
                        )
                except Exception as error:
                    self._latch_active_attempt_fault(
                        f"move_group_cancel_failed:{type(error).__name__}",
                        request_id,
                        attempt_generation,
                    )
        self._publish_status(
            "cancelled_move_group",
            reason,
            request_id,
            attempt_generation=attempt_generation,
            move_group_goal_id=move_group_goal_id,
            result_future_pending_at_timeout=pending_at_timeout,
            move_group_cancel_requested=cancel_requested,
            cancel_response_accepted=cancel_accepted,
        )
        terminal_observed = False
        if result_future is None:
            # The accepted goal exists, but there is no result future that can
            # prove its terminal state.  A cancel acknowledgement alone is
            # never sufficient to proceed.
            terminal_observed = self._wait_for_move_group_terminal(
                None,
                request_id,
                attempt_generation,
                move_group_goal_id,
                pending_at_timeout=pending_at_timeout,
            )
        else:
            terminal_observed = self._wait_for_move_group_terminal(
                result_future,
                request_id,
                attempt_generation,
                move_group_goal_id,
                pending_at_timeout=pending_at_timeout,
            )
        return cancel_requested and cancel_accepted and terminal_observed

    def _late_cancel_response_callback(
        self,
        future,
        *,
        request_id: str | None,
        attempt_generation: int | None,
        goal_id: str | None,
        kind: str,
    ) -> None:
        prefix = "move_group" if kind == "move_group" else "trajectory_gate"
        try:
            response = future.result()
            accepted = bool(getattr(response, "goals_canceling", ()))
        except Exception as error:
            self._latch_active_attempt_fault(
                f"late_{prefix}_goal_cancel_failed:{type(error).__name__}",
                request_id,
                attempt_generation,
            )
            self._publish_status(
                f"late_{kind}_goal_cancel_response",
                f"exception:{type(error).__name__}",
                request_id or "<none>",
                attempt_generation=attempt_generation,
                move_group_goal_id=goal_id if kind == "move_group" else None,
                gate_goal_id=goal_id if kind == "gate" else None,
                gate_cancel_requested=True if kind == "gate" else None,
                move_group_cancel_requested=True
                if kind == "move_group"
                else None,
                cancel_response_accepted=False,
            )
            return
        if not accepted:
            self._latch_active_attempt_fault(
                f"late_{prefix}_goal_cancel_rejected",
                request_id,
                attempt_generation,
            )
        self._publish_status(
            f"late_{kind}_goal_cancel_response",
            "accepted" if accepted else "rejected",
            request_id or "<none>",
            attempt_generation=attempt_generation,
            move_group_goal_id=goal_id if kind == "move_group" else None,
            gate_goal_id=goal_id if kind == "gate" else None,
            gate_cancel_requested=True if kind == "gate" else None,
            move_group_cancel_requested=True if kind == "move_group" else None,
            cancel_response_accepted=accepted,
        )

    def _late_goal_terminal_callback(
        self,
        future,
        *,
        request_id: str | None,
        attempt_generation: int | None,
        goal_id: str | None,
        kind: str,
    ) -> None:
        try:
            wrapped = future.result()
            status = int(getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN))
        except Exception as error:
            self._latch_active_attempt_fault(
                f"late_{kind}_goal_terminal_failed:{type(error).__name__}",
                request_id,
                attempt_generation,
            )
            self._publish_status(
                f"late_{kind}_goal_terminal",
                f"exception:{type(error).__name__}",
                request_id or "<none>",
                attempt_generation=attempt_generation,
                move_group_goal_id=goal_id if kind == "move_group" else None,
                gate_goal_id=goal_id if kind == "gate" else None,
                move_group_cancel_requested=True if kind == "move_group" else None,
                gate_cancel_requested=True if kind == "gate" else None,
                move_group_terminal_observed=False
                if kind == "move_group"
                else None,
                gate_terminal_observed=False if kind == "gate" else None,
                gate_terminal_status=None,
            )
            return
        terminal = self._is_terminal_status(status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._latch_active_attempt_fault(
                f"late_{kind}_goal_success_after_cancel",
                request_id,
                attempt_generation,
            )
        self._publish_status(
            f"late_{kind}_goal_terminal",
            "observed" if terminal else "nonterminal",
            request_id or "<none>",
            attempt_generation=attempt_generation,
            move_group_goal_id=goal_id if kind == "move_group" else None,
            gate_goal_id=goal_id if kind == "gate" else None,
            move_group_cancel_requested=True if kind == "move_group" else None,
            gate_cancel_requested=True if kind == "gate" else None,
            move_group_terminal_observed=terminal if kind == "move_group" else None,
            gate_terminal_observed=terminal if kind == "gate" else None,
            gate_terminal_status=status if kind == "gate" else None,
            move_group_terminal_status=status if kind == "move_group" else None,
            action_goal_status=status,
        )
        if not terminal:
            self._latch_active_attempt_fault(
                f"late_{kind}_goal_terminal_unconfirmed",
                request_id,
                attempt_generation,
            )

    def _late_goal_terminal_watchdog(
        self,
        result_future,
        *,
        request_id: str | None,
        attempt_generation: int | None,
        goal_id: str | None,
        kind: str,
    ) -> None:
        """Record a bounded late-goal terminal miss without stopping a controller."""

        if self._destroying.is_set() or result_future.done():
            return
        self._latch_active_attempt_fault(
            f"late_{kind}_goal_terminal_unconfirmed",
            request_id,
            attempt_generation,
        )
        self._publish_status(
            f"late_{kind}_goal_terminal_unconfirmed",
            "terminal_timeout",
            request_id or "<none>",
            attempt_generation=attempt_generation,
            move_group_goal_id=goal_id if kind == "move_group" else None,
            gate_goal_id=goal_id if kind == "gate" else None,
            move_group_cancel_requested=True if kind == "move_group" else None,
            gate_cancel_requested=True if kind == "gate" else None,
            move_group_terminal_observed=False
            if kind == "move_group"
            else None,
            gate_terminal_observed=False if kind == "gate" else None,
        )

    def _schedule_late_terminal_watchdog(
        self,
        result_future,
        *,
        request_id: str | None,
        attempt_generation: int | None,
        goal_id: str | None,
        kind: str,
    ) -> None:
        if (
            result_future is None
            or result_future.done()
            or self._destroying.is_set()
        ):
            return

        watchdog: Timer

        def run_watchdog() -> None:
            try:
                self._late_goal_terminal_watchdog(
                    result_future,
                    request_id=request_id,
                    attempt_generation=attempt_generation,
                    goal_id=goal_id,
                    kind=kind,
                )
            finally:
                with self._late_terminal_watchdog_lock:
                    self._late_terminal_watchdogs.discard(watchdog)

        watchdog = Timer(
            self._move_group_terminal_s,
            run_watchdog,
        )
        watchdog.daemon = True
        with self._late_terminal_watchdog_lock:
            if self._destroying.is_set():
                return
            self._late_terminal_watchdogs.add(watchdog)
            watchdog.start()

        def cancel_watchdog(_future) -> None:
            watchdog.cancel()
            with self._late_terminal_watchdog_lock:
                self._late_terminal_watchdogs.discard(watchdog)

        result_future.add_done_callback(cancel_watchdog)

    def _publish_late_result_future_unavailable(
        self,
        *,
        request_id: str | None,
        attempt_generation: int | None,
        goal_id: str | None,
        kind: str,
        reason: str,
    ) -> None:
        """Expose a late accepted goal whose exact terminal future is absent."""

        self._publish_status(
            f"late_{kind}_goal_terminal_unconfirmed",
            reason,
            request_id or "<none>",
            attempt_generation=attempt_generation,
            move_group_goal_id=goal_id if kind == "move_group" else None,
            gate_goal_id=goal_id if kind == "gate" else None,
            goal_response_future_pending_at_timeout=True
            if kind == "move_group"
            else None,
            gate_goal_response_future_pending_at_timeout=True
            if kind == "gate"
            else None,
            result_future_pending_at_timeout=None,
            move_group_cancel_requested=True if kind == "move_group" else None,
            gate_cancel_requested=True if kind == "gate" else None,
            move_group_terminal_observed=False
            if kind == "move_group"
            else None,
            gate_terminal_observed=False if kind == "gate" else None,
        )

    def _observe_unconfirmed_move_group_terminal(
        self,
        result_future,
        *,
        request_id: str,
        attempt_generation: int,
        goal_id: str,
    ) -> None:
        """Record a terminal that arrives after the bounded cancel wait.

        The callback carries the original attempt identity.  Its fault path is
        generation-aware, so observing an old terminal cannot mutate a newer
        retry, while the old goal lifecycle remains directly observable.
        """

        if result_future is None or result_future.done():
            return
        result_future.add_done_callback(
            partial(
                self._late_goal_terminal_callback,
                request_id=request_id,
                attempt_generation=attempt_generation,
                goal_id=goal_id,
                kind="move_group",
            )
        )

    def _late_cancel_callback(
        self,
        future,
        request_id: str | None = None,
        attempt_generation: int | None = None,
    ) -> None:
        # This callback is installed only after the send future has already
        # timed out or the wrapper request has failed closed.  The future can
        # become ready before _execute() reaches its finally block, so active
        # request identity is not evidence that normal goal handling continues.
        # Always cancel the accepted planning goal returned by this future.
        # Do not inspect active state here: a newer attempt must not make this
        # older accepted goal safe to leave running.
        try:
            goal = future.result()
            if goal is None or not goal.accepted:
                self._publish_status(
                    "late_move_group_goal_response",
                    "rejected",
                    request_id or "<none>",
                    attempt_generation=attempt_generation,
                )
                return
            goal_id = self._goal_id_hex(goal)
            self._publish_status(
                "late_move_group_goal_response",
                "accepted_cancel_requested",
                request_id or "<none>",
                attempt_generation=attempt_generation,
                move_group_goal_id=goal_id,
                goal_response_future_pending_at_timeout=True,
                move_group_cancel_requested=True,
            )
            try:
                cancel_future = goal.cancel_goal_async()
            except Exception as error:
                self._latch_active_attempt_fault(
                    f"late_move_group_goal_cancel_failed:{type(error).__name__}",
                    request_id,
                    attempt_generation,
                )
            else:
                add_done_callback = getattr(cancel_future, "add_done_callback", None)
                if callable(add_done_callback):
                    add_done_callback(
                        partial(
                            self._late_cancel_response_callback,
                            request_id=request_id,
                            attempt_generation=attempt_generation,
                            goal_id=goal_id,
                            kind="move_group",
                        )
                    )
            try:
                result_future = goal.get_result_async()
            except Exception as error:
                self._latch_active_attempt_fault(
                    f"late_move_group_goal_result_request_failed:{type(error).__name__}",
                    request_id,
                    attempt_generation,
                )
                self._publish_late_result_future_unavailable(
                    request_id=request_id,
                    attempt_generation=attempt_generation,
                    goal_id=goal_id,
                    kind="move_group",
                    reason=f"result_exception:{type(error).__name__}",
                )
            else:
                if result_future is None:
                    self._latch_active_attempt_fault(
                        "late_move_group_goal_result_future_unavailable",
                        request_id,
                        attempt_generation,
                    )
                    self._publish_late_result_future_unavailable(
                        request_id=request_id,
                        attempt_generation=attempt_generation,
                        goal_id=goal_id,
                        kind="move_group",
                        reason="result_future_unavailable",
                    )
                else:
                    add_done_callback = getattr(
                        result_future, "add_done_callback", None
                    )
                    if callable(add_done_callback):
                        add_done_callback(
                            partial(
                                self._late_goal_terminal_callback,
                                request_id=request_id,
                                attempt_generation=attempt_generation,
                                goal_id=goal_id,
                                kind="move_group",
                            )
                        )
                    self._schedule_late_terminal_watchdog(
                        result_future,
                        request_id=request_id,
                        attempt_generation=attempt_generation,
                        goal_id=goal_id,
                        kind="move_group",
                    )
        except Exception as error:
            self._latch_active_attempt_fault(
                f"late_goal_cancel_failed:{type(error).__name__}",
                request_id,
                attempt_generation,
            )

    @staticmethod
    def _trajectory_point_payload(point) -> dict[str, object]:
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

    def _make_gate_goal(
        self,
        request: PlanTarget.Goal,
        trajectory: JointTrajectory,
    ) -> tuple[ExecuteTrajectory.Goal, str]:
        gated_trajectory = deepcopy(trajectory)
        gated_trajectory.header.frame_id = self._planning_frame
        gated_trajectory.header.stamp = request.target_pose.header.stamp
        digest = trajectory_digest(
            "arm_controller",
            gated_trajectory.joint_names,
            [
                self._trajectory_point_payload(point)
                for point in gated_trajectory.points
            ],
        )
        goal = ExecuteTrajectory.Goal()
        goal.task_id = request.task_id
        goal.command_id = self._request_id(request)
        goal.target_id = request.target_id
        goal.stage = request.stage
        goal.sequence_no = request.sequence_no
        goal.controller = "arm_controller"
        goal.trajectory = gated_trajectory
        goal.source_timestamp_ns = request.source_timestamp_ns
        goal.clock_domain = request.clock_domain
        goal.clock_epoch = request.clock_epoch
        return goal, digest

    def _wait_for_gate_terminal(
        self,
        result_future,
        request_id: str,
        attempt_generation: int | None,
        gate_goal_id: str | None,
    ) -> bool:
        """Observe the exact accepted gate goal after a cancel request."""

        if result_future is None:
            self._latch_active_attempt_fault(
                "trajectory_gate_terminal_unconfirmed",
                request_id,
                attempt_generation,
            )
            self._publish_status(
                "gate_terminal_unconfirmed",
                "result_future_unavailable",
                request_id,
                attempt_generation=attempt_generation,
                gate_goal_id=gate_goal_id,
                gate_cancel_requested=True,
                gate_terminal_observed=False,
            )
            return False
        deadline = time.monotonic() + self._move_group_terminal_s
        while (
            self.context.ok()
            and time.monotonic() < deadline
            and not result_future.done()
        ):
            time.sleep(0.01)
        if not result_future.done():
            self._latch_active_attempt_fault(
                "trajectory_gate_terminal_unconfirmed",
                request_id,
                attempt_generation,
            )
            self._publish_status(
                "gate_terminal_unconfirmed",
                "result_future_timeout",
                request_id,
                attempt_generation=attempt_generation,
                gate_goal_id=gate_goal_id,
                gate_cancel_requested=True,
                gate_terminal_observed=False,
            )
            return False
        try:
            wrapped = result_future.result()
            status = int(getattr(wrapped, "status", GoalStatus.STATUS_UNKNOWN))
        except Exception as error:
            self._latch_active_attempt_fault(
                f"trajectory_gate_terminal_result_exception:{type(error).__name__}",
                request_id,
                attempt_generation,
            )
            self._publish_status(
                "gate_terminal_unconfirmed",
                f"result_exception:{type(error).__name__}",
                request_id,
                attempt_generation=attempt_generation,
                gate_goal_id=gate_goal_id,
                gate_cancel_requested=True,
                gate_terminal_observed=False,
            )
            return False
        terminal = self._is_terminal_status(status)
        self._publish_status(
            "gate_terminal_observed" if terminal else "gate_terminal_unconfirmed",
            "success_after_cancel"
            if status == GoalStatus.STATUS_SUCCEEDED
            else "observed_after_cancel"
            if terminal
            else "nonterminal_status",
            request_id,
            attempt_generation=attempt_generation,
            gate_goal_id=gate_goal_id,
            gate_cancel_requested=True,
            gate_terminal_observed=terminal,
            gate_terminal_status=status,
            action_goal_status=status,
        )
        if not terminal:
            self._latch_active_attempt_fault(
                "trajectory_gate_terminal_unconfirmed",
                request_id,
                attempt_generation,
            )
        elif status == GoalStatus.STATUS_SUCCEEDED:
            self._latch_active_attempt_fault(
                "trajectory_gate_cancel_success_race",
                request_id,
                attempt_generation,
            )
        return terminal and status != GoalStatus.STATUS_SUCCEEDED

    def _cancel_gate_goal(
        self,
        goal_handle,
        request_id: str,
        *,
        attempt_generation: int | None = None,
        result_future=None,
    ) -> bool:
        gate_goal_id = self._goal_id_hex(goal_handle)
        cancel_requested = False
        cancel_accepted = False
        try:
            future = goal_handle.cancel_goal_async()
            cancel_requested = True
            if (
                future is None
                or not callable(getattr(future, "done", None))
                or not callable(getattr(future, "result", None))
            ):
                raise TypeError("cancel_goal_async returned no usable future")
        except Exception as error:
            self._latch_active_attempt_fault(
                f"trajectory_gate_cancel_exception:{type(error).__name__}",
                request_id,
                attempt_generation,
            )
        else:
            try:
                deadline = time.monotonic() + self._cancel_response_s
                while (
                    self.context.ok()
                    and time.monotonic() < deadline
                    and not future.done()
                ):
                    time.sleep(0.01)
                if not future.done():
                    self._latch_active_attempt_fault(
                        "trajectory_gate_cancel_timeout",
                        request_id,
                        attempt_generation,
                    )
                else:
                    response = future.result()
                    cancel_accepted = bool(
                        getattr(response, "goals_canceling", ())
                    )
                    if not cancel_accepted:
                        self._latch_active_attempt_fault(
                            "trajectory_gate_cancel_rejected",
                            request_id,
                            attempt_generation,
                        )
            except Exception as error:
                self._latch_active_attempt_fault(
                    f"trajectory_gate_cancel_failed:{type(error).__name__}",
                    request_id,
                    attempt_generation,
                )
        self._publish_status(
            "cancelled_gate_command",
            "accepted" if cancel_accepted else "unconfirmed",
            request_id,
            attempt_generation=attempt_generation,
            gate_goal_id=gate_goal_id,
            gate_cancel_requested=cancel_requested,
            cancel_response_accepted=cancel_accepted,
        )
        terminal_observed = self._wait_for_gate_terminal(
            result_future,
            request_id,
            attempt_generation,
            gate_goal_id,
        )
        return cancel_requested and cancel_accepted and terminal_observed

    def _late_gate_cancel_callback(
        self,
        future,
        request_id: str | None = None,
        attempt_generation: int | None = None,
    ) -> None:
        # A late accepted gate goal belongs to an already failed send path,
        # even if _finish() has not yet cleared the request identity.  As with
        # MoveGroup, always cancel the goal returned by this exact future and
        # retain the captured request/generation for lifecycle evidence.
        try:
            goal = future.result()
            if goal is None or not goal.accepted:
                self._publish_status(
                    "late_gate_goal_response",
                    "rejected",
                    request_id or "<none>",
                    attempt_generation=attempt_generation,
                )
                return
            goal_id = self._goal_id_hex(goal)
            self._publish_status(
                "late_gate_goal_response",
                "accepted_cancel_requested",
                request_id or "<none>",
                attempt_generation=attempt_generation,
                gate_goal_id=goal_id,
                gate_goal_response_future_pending_at_timeout=True,
                gate_cancel_requested=True,
            )
            try:
                cancel_future = goal.cancel_goal_async()
            except Exception as error:
                self._latch_active_attempt_fault(
                    f"late_gate_goal_cancel_failed:{type(error).__name__}",
                    request_id,
                    attempt_generation,
                )
            else:
                add_done_callback = getattr(cancel_future, "add_done_callback", None)
                if callable(add_done_callback):
                    add_done_callback(
                        partial(
                            self._late_cancel_response_callback,
                            request_id=request_id,
                            attempt_generation=attempt_generation,
                            goal_id=goal_id,
                            kind="gate",
                        )
                    )
            try:
                result_future = goal.get_result_async()
            except Exception as error:
                self._latch_active_attempt_fault(
                    f"late_gate_goal_result_request_failed:{type(error).__name__}",
                    request_id,
                    attempt_generation,
                )
                self._publish_late_result_future_unavailable(
                    request_id=request_id,
                    attempt_generation=attempt_generation,
                    goal_id=goal_id,
                    kind="gate",
                    reason=f"result_exception:{type(error).__name__}",
                )
            else:
                if result_future is None:
                    self._latch_active_attempt_fault(
                        "late_gate_goal_result_future_unavailable",
                        request_id,
                        attempt_generation,
                    )
                    self._publish_late_result_future_unavailable(
                        request_id=request_id,
                        attempt_generation=attempt_generation,
                        goal_id=goal_id,
                        kind="gate",
                        reason="result_future_unavailable",
                    )
                else:
                    add_done_callback = getattr(
                        result_future, "add_done_callback", None
                    )
                    if callable(add_done_callback):
                        add_done_callback(
                            partial(
                                self._late_goal_terminal_callback,
                                request_id=request_id,
                                attempt_generation=attempt_generation,
                                goal_id=goal_id,
                                kind="gate",
                            )
                        )
                    self._schedule_late_terminal_watchdog(
                        result_future,
                        request_id=request_id,
                        attempt_generation=attempt_generation,
                        goal_id=goal_id,
                        kind="gate",
                    )
        except Exception as error:
            self._latch_active_attempt_fault(
                f"late_gate_goal_cancel_failed:{type(error).__name__}",
                request_id,
                attempt_generation,
            )

    def _gate_outcome(
        self,
        request: PlanTarget.Goal,
        expected_digest: str,
        wrapped,
    ) -> _GateOutcome:
        result = wrapped.result
        expected = (
            request.task_id,
            self._request_id(request),
            request.target_id,
            request.stage,
            int(request.sequence_no),
            "arm_controller",
            expected_digest,
            int(request.source_timestamp_ns),
            request.clock_domain,
            int(request.clock_epoch),
        )
        observed = (
            result.task_id,
            result.command_id,
            result.target_id,
            result.stage,
            int(result.sequence_no),
            result.controller,
            result.trajectory_digest,
            int(result.source_timestamp_ns),
            result.clock_domain,
            int(result.clock_epoch),
        )
        if observed != expected:
            self._latch_fault("trajectory_gate_result_correlation_mismatch")
            return _GateOutcome(
                dispatched=True,
                trajectory_digest=expected_digest,
                reason="trajectory_gate_result_correlation_mismatch",
            )
        success = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and bool(result.success)
            and bool(result.terminal)
            and bool(result.downstream_terminal_observed)
            and int(result.action_goal_status) == GoalStatus.STATUS_SUCCEEDED
            and int(result.fjt_error_code)
            == FollowJointTrajectory.Result.SUCCESSFUL
        )
        return _GateOutcome(
            dispatched=True,
            trajectory_digest=expected_digest,
            accepted=bool(result.accepted),
            terminal=bool(result.terminal),
            downstream_terminal_observed=bool(
                result.downstream_terminal_observed
            ),
            cancel_requested=bool(result.cancel_requested),
            action_goal_status=int(result.action_goal_status),
            fjt_error_code=int(result.fjt_error_code),
            fjt_error_string=str(result.fjt_error_string),
            success=success,
            reason="succeeded" if success else f"trajectory_gate:{result.reason}",
        )

    def _dispatch_to_gate(
        self,
        goal_handle,
        request: PlanTarget.Goal,
        trajectory: JointTrajectory,
        *,
        attempt_generation: int | None = None,
    ) -> _GateOutcome:
        request_id = self._request_id(request)
        if goal_handle.is_cancel_requested:
            return _GateOutcome(reason="plan_target_cancel_requested")
        if (
            attempt_generation is not None
            and not self._active_matches(request_id, attempt_generation)
        ):
            return _GateOutcome(reason="late_result_ignored")
        try:
            gate_goal, digest = self._make_gate_goal(request, trajectory)
        except ValueError as error:
            return _GateOutcome(reason=f"trajectory_digest_failed:{error}")
        if not self._trajectory_gate.wait_for_server(
            timeout_sec=self._gate_discovery_s
        ):
            return _GateOutcome(
                trajectory_digest=digest,
                reason="trajectory_gate_unavailable",
            )
        # Re-run the admission checks after service discovery.  Discovery can
        # consume enough wall time for cancellation or a newer attempt to win
        # the boundary race.
        if goal_handle.is_cancel_requested:
            return _GateOutcome(
                trajectory_digest=digest,
                reason="plan_target_cancel_requested",
            )
        if (
            attempt_generation is not None
            and not self._active_matches(request_id, attempt_generation)
        ):
            return _GateOutcome(
                trajectory_digest=digest,
                reason="late_result_ignored",
            )
        with self._state_lock:
            if (
                attempt_generation is not None
                and (
                    self._active_request_id != request_id
                    or self._active_attempt_generation != attempt_generation
                )
            ):
                return _GateOutcome(
                    trajectory_digest=digest,
                    reason="late_result_ignored",
                )
            if goal_handle.is_cancel_requested:
                return _GateOutcome(
                    trajectory_digest=digest,
                    reason="plan_target_cancel_requested",
                )
            if request_id in self._dispatched_command_ids:
                self._fault_latched = "duplicate_dispatch_prevented"
                return _GateOutcome(
                    trajectory_digest=digest,
                    reason="duplicate_dispatch_prevented",
                )
            # This is the final atomic admission point.  Keep the command id
            # reserved once the send path passes its last check; release it
            # if a cancellation/identity race wins before the call exists or
            # if send_goal_async itself fails.
            self._dispatched_command_ids.add(request_id)
        if goal_handle.is_cancel_requested or (
            attempt_generation is not None
            and not self._active_matches(request_id, attempt_generation)
        ):
            with self._state_lock:
                self._dispatched_command_ids.discard(request_id)
            return _GateOutcome(
                trajectory_digest=digest,
                reason=(
                    "plan_target_cancel_requested"
                    if goal_handle.is_cancel_requested
                    else "late_result_ignored"
                ),
            )
        try:
            send_future = self._trajectory_gate.send_goal_async(gate_goal)
        except Exception as error:
            with self._state_lock:
                self._dispatched_command_ids.discard(request_id)
            self._latch_active_attempt_fault(
                f"trajectory_gate_send_exception:{type(error).__name__}",
                request_id,
                attempt_generation,
            )
            return _GateOutcome(
                trajectory_digest=digest,
                reason="trajectory_gate_send_exception",
            )
        self._publish_status(
            "trajectory_gate_goal_sent",
            "goal_response_pending",
            request_id,
            attempt_generation=attempt_generation,
            gate_goal_response_future_pending_at_timeout=True,
        )
        send_reason = self._poll_future(
            send_future,
            time.monotonic() + self._goal_response_s,
            goal_handle,
            request,
            timeout_reason="trajectory_gate_goal_response_timeout",
        )
        if send_reason is not None:
            self._publish_status(
                "trajectory_gate_goal_response_timeout"
                if send_reason == "trajectory_gate_goal_response_timeout"
                else "trajectory_gate_goal_response_failed",
                send_reason,
                request_id,
                attempt_generation=attempt_generation,
                gate_goal_response_future_pending_at_timeout=True,
            )
            send_future.add_done_callback(
                partial(
                    self._late_gate_cancel_callback,
                    request_id=request_id,
                    attempt_generation=attempt_generation,
                )
            )
            return _GateOutcome(
                trajectory_digest=digest,
                reason=send_reason,
            )
        try:
            gate_handle = send_future.result()
        except Exception as error:
            self._latch_active_attempt_fault(
                f"trajectory_gate_goal_response_exception:{type(error).__name__}",
                request_id,
                attempt_generation,
            )
            return _GateOutcome(
                trajectory_digest=digest,
                reason="trajectory_gate_goal_response_exception",
            )
        if gate_handle is None or not gate_handle.accepted:
            return _GateOutcome(
                trajectory_digest=digest,
                reason="trajectory_gate_goal_rejected",
            )
        gate_goal_id = self._goal_id_hex(gate_handle)
        if gate_goal_id is None:
            self._latch_active_attempt_fault(
                "trajectory_gate_goal_id_unavailable",
                request_id,
                attempt_generation,
            )
            self._publish_status(
                "trajectory_gate_goal_id_unavailable",
                "accepted_goal_not_correlatable",
                request_id,
                attempt_generation=attempt_generation,
                gate_cancel_requested=True,
            )
            try:
                uncorrelatable_result_future = self._get_gate_result_future(
                    gate_handle
                )
            except Exception as error:
                self._latch_active_attempt_fault(
                    f"trajectory_gate_result_request_exception:{type(error).__name__}",
                    request_id,
                    attempt_generation,
                )
                self._cancel_gate_goal(
                    gate_handle,
                    request_id,
                    attempt_generation=attempt_generation,
                )
                return _GateOutcome(
                    dispatched=True,
                    trajectory_digest=digest,
                    accepted=True,
                    cancel_requested=True,
                    reason="trajectory_gate_goal_id_unavailable:gate_terminal_unconfirmed",
                )
            terminal_confirmed = self._cancel_gate_goal(
                gate_handle,
                request_id,
                attempt_generation=attempt_generation,
                result_future=uncorrelatable_result_future,
            )
            reason = "trajectory_gate_goal_id_unavailable"
            if not terminal_confirmed:
                reason += ":gate_terminal_unconfirmed"
            return _GateOutcome(
                dispatched=True,
                trajectory_digest=digest,
                accepted=True,
                terminal=terminal_confirmed,
                downstream_terminal_observed=terminal_confirmed,
                cancel_requested=True,
                action_goal_status=(
                    GoalStatus.STATUS_CANCELED
                    if terminal_confirmed
                    else GoalStatus.STATUS_UNKNOWN
                ),
                reason=reason,
            )
        self._publish_status(
            "gate_goal_accepted",
            "result_pending",
            request_id,
            attempt_generation=attempt_generation,
            gate_goal_id=gate_goal_id,
        )
        try:
            result_future = self._get_gate_result_future(gate_handle)
        except Exception as error:
            self._latch_active_attempt_fault(
                f"trajectory_gate_result_request_exception:{type(error).__name__}",
                request_id,
                attempt_generation,
            )
            self._cancel_gate_goal(
                gate_handle,
                request_id,
                attempt_generation=attempt_generation,
            )
            return _GateOutcome(
                dispatched=True,
                trajectory_digest=digest,
                accepted=True,
                cancel_requested=True,
                reason="trajectory_gate_result_request_exception:gate_terminal_unconfirmed",
            )
        if result_future is None:
            self._latch_active_attempt_fault(
                "trajectory_gate_result_future_unavailable",
                request_id,
                attempt_generation,
            )
            self._cancel_gate_goal(
                gate_handle,
                request_id,
                attempt_generation=attempt_generation,
                result_future=None,
            )
            return _GateOutcome(
                dispatched=True,
                trajectory_digest=digest,
                accepted=True,
                cancel_requested=True,
                reason=(
                    "trajectory_gate_result_future_unavailable:"
                    "gate_terminal_unconfirmed"
                ),
            )
        duration_ns = (
            trajectory.points[-1].time_from_start.sec * 1_000_000_000
            + trajectory.points[-1].time_from_start.nanosec
        )
        result_reason = self._poll_future(
            result_future,
            time.monotonic()
            + max(
                # The typed gate owns execution-time safety in the configured
                # ROS clock domain.  A slow Gazebo real-time factor must not
                # make this adapter's wall clock cancel an otherwise healthy
                # simulated trajectory before the gate reaches its terminal.
                # The wall guard remains bounded for a crashed gate process.
                duration_ns / 1e9 + self._gate_result_margin_s,
                self._gate_terminal_wall_guard_s,
            ),
            goal_handle,
            request,
            timeout_reason="trajectory_gate_result_timeout",
            # The immutable target stamp is a planning/dispatch boundary, not
            # an execution-liveness clock.  Once the typed gate accepts the
            # trajectory, its local target-stream watchdog and the sequence
            # wrapper's identity/drift monitor own active-motion freshness.
            require_source_freshness=False,
        )
        gate_cancel_already_requested = False
        gate_cancel_terminal_confirmed = False
        if result_reason is not None:
            gate_cancel_terminal_confirmed = self._cancel_gate_goal(
                gate_handle,
                request_id,
                attempt_generation=attempt_generation,
                result_future=result_future,
            )
            gate_cancel_already_requested = True
            if not result_future.done():
                return _GateOutcome(
                    dispatched=True,
                    trajectory_digest=digest,
                    accepted=True,
                    cancel_requested=True,
                    reason=f"{result_reason}:gate_terminal_unconfirmed",
                )
        # The gate result and a PlanTarget cancel can complete concurrently.
        # Even a successful gate result is unusable once this wrapper has
        # observed cancellation; report the terminal gate state but never
        # promote it to a successful PlanTarget result.
        if goal_handle.is_cancel_requested:
            if not gate_cancel_already_requested:
                self._cancel_gate_goal(
                    gate_handle,
                    request_id,
                    attempt_generation=attempt_generation,
                    result_future=result_future,
                )
            if result_future.done():
                try:
                    wrapped = result_future.result()
                    outcome = self._gate_outcome(request, digest, wrapped)
                    if outcome.success:
                        self._latch_active_attempt_fault(
                            "trajectory_gate_cancel_success_race",
                            request_id,
                            attempt_generation,
                        )
                    return replace(
                        outcome,
                        success=False,
                        cancel_requested=True,
                        reason="plan_target_cancel_requested:gate_result_after_cancel",
                    )
                except Exception:
                    pass
            return _GateOutcome(
                dispatched=True,
                trajectory_digest=digest,
                cancel_requested=True,
                reason="plan_target_cancel_requested:gate_terminal_unconfirmed",
            )
        try:
            wrapped = result_future.result()
        except Exception as error:
            self._latch_active_attempt_fault(
                f"trajectory_gate_result_exception:{type(error).__name__}",
                request_id,
                attempt_generation,
            )
            terminal_confirmed = gate_cancel_terminal_confirmed
            if not gate_cancel_already_requested:
                terminal_confirmed = self._cancel_gate_goal(
                    gate_handle,
                    request_id,
                    attempt_generation=attempt_generation,
                    result_future=result_future,
                )
            reason = "trajectory_gate_result_exception"
            if not terminal_confirmed:
                reason += ":gate_terminal_unconfirmed"
            return _GateOutcome(
                dispatched=True,
                trajectory_digest=digest,
                accepted=True,
                terminal=terminal_confirmed,
                downstream_terminal_observed=terminal_confirmed,
                cancel_requested=True,
                action_goal_status=(
                    GoalStatus.STATUS_CANCELED
                    if terminal_confirmed
                    else GoalStatus.STATUS_UNKNOWN
                ),
                reason=reason,
            )
        outcome = self._gate_outcome(request, digest, wrapped)
        self._publish_status(
            "gate_terminal",
            "observed" if outcome.terminal else "unconfirmed",
            request_id,
            attempt_generation=attempt_generation,
            gate_goal_id=gate_goal_id,
            gate_cancel_requested=outcome.cancel_requested,
            gate_terminal_observed=outcome.terminal,
            gate_terminal_status=outcome.action_goal_status,
        )
        if result_reason is not None and outcome.success:
            self._latch_active_attempt_fault(
                "trajectory_gate_cancel_success_race",
                request_id,
                attempt_generation,
            )
            return _GateOutcome(
                dispatched=True,
                trajectory_digest=digest,
                accepted=outcome.accepted,
                terminal=outcome.terminal,
                downstream_terminal_observed=outcome.downstream_terminal_observed,
                cancel_requested=True,
                action_goal_status=outcome.action_goal_status,
                fjt_error_code=outcome.fjt_error_code,
                fjt_error_string=outcome.fjt_error_string,
                success=False,
                reason=f"{result_reason}:gate_reported_success_after_cancel",
            )
        if result_reason is not None:
            return replace(
                outcome,
                success=False,
                cancel_requested=True,
                reason=f"{result_reason}:{outcome.reason}",
            )
        return outcome

    def _result(
        self,
        request: PlanTarget.Goal,
        requested_at,
        *,
        success: bool,
        reason: str,
        moveit_error_code: int = 0,
        gate: _GateOutcome | None = None,
    ) -> PlanTarget.Result:
        gate = gate or _GateOutcome()
        result = PlanTarget.Result()
        result.task_id = request.task_id
        try:
            result.command_id = self._request_id(request)
        except ValueError:
            result.command_id = ""
        result.target_id = request.target_id
        result.stage = request.stage
        result.sequence_no = request.sequence_no
        result.accepted = True
        result.success = success
        result.reason = reason
        result.moveit_error_code = moveit_error_code
        result.requested_at = requested_at
        result.source_timestamp_ns = request.source_timestamp_ns
        result.clock_domain = request.clock_domain
        result.clock_epoch = request.clock_epoch
        result.trajectory_dispatched = gate.dispatched
        result.trajectory_digest = gate.trajectory_digest
        result.gate_accepted = gate.accepted
        result.gate_terminal = gate.terminal
        result.downstream_terminal_observed = gate.downstream_terminal_observed
        result.cancel_requested = gate.cancel_requested
        result.action_goal_status = gate.action_goal_status
        result.fjt_error_code = gate.fjt_error_code
        result.fjt_error_string = gate.fjt_error_string
        return result

    def _finish(self, request_id: str, attempt_generation: int) -> None:
        with self._state_lock:
            if (
                self._active_request_id == request_id
                and self._active_attempt_generation == attempt_generation
            ):
                self._active_request_id = None
                self._active_attempt_generation = None
                self._active_moveit_goal = None
                self._active_moveit_goal_id = None
                self._active_moveit_result_future = None
                if self._reserved_request_id == request_id:
                    self._reserved_request_id = None

    def _execute(self, goal_handle) -> PlanTarget.Result:
        request = goal_handle.request
        request_id = self._request_id(request)
        attempt_generation = self._new_attempt(request_id)
        requested_at = self.get_clock().now().to_msg()
        try:
            self._feedback(goal_handle, "validating_request", request_id)
            reason = self._validate_request(request)
            if reason is not None:
                goal_handle.abort()
                return self._result(request, requested_at, success=False, reason=reason)
            reason = self._wait_for_source_clock(request)
            if reason is not None:
                goal_handle.abort()
                return self._result(request, requested_at, success=False, reason=reason)
            reason = self._safety_reason(request)
            if reason is not None:
                goal_handle.abort()
                return self._result(request, requested_at, success=False, reason=reason)
            self._feedback(goal_handle, "configuring_contact_policy", request_id)
            reason = self._set_target_pad_contact_policy(goal_handle, request)
            if reason is not None:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=reason,
                )
            # The policy transition may have taken long enough for any of the
            # independent safety inputs to expire.  Re-evaluate them before IK
            # and planning; a confirmed ACM update never authorizes motion by
            # itself.
            reason = self._safety_reason(request)
            if reason is not None:
                goal_handle.abort()
                return self._result(request, requested_at, success=False, reason=reason)
            now_ns, _ = self._observe_now()
            start_positions = self._fresh_start_positions(now_ns)
            if start_positions is None:
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="joint_states_missing_or_stale",
                )

            self._feedback(goal_handle, "transforming_target", request_id)
            transformed, reason = self._transform_target(request)
            if reason is not None or transformed is None:
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=reason or "tf2_transform_failed",
                )
            self._feedback(goal_handle, "solving_ik", request_id)
            goal_positions, reason, ik_error_code = self._solve_ik(
                goal_handle, request, transformed, start_positions
            )
            if reason is not None or goal_positions is None:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=reason or "compute_ik_failed",
                    moveit_error_code=ik_error_code,
                )
            reason = self._safety_reason(request)
            if reason is not None:
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=reason,
                    moveit_error_code=ik_error_code,
                )
            if not self._move_group.wait_for_server(
                timeout_sec=self._move_group_discovery_s
            ):
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="move_group_unavailable",
                )

            move_group_goal = self._make_move_group_goal(
                request, goal_positions, start_positions
            )
            self._feedback(goal_handle, "planning", request_id)
            try:
                send_future = self._move_group.send_goal_async(move_group_goal)
            except Exception as error:
                self._latch_active_attempt_fault(
                    f"move_group_send_exception:{type(error).__name__}",
                    request_id,
                    attempt_generation,
                )
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="move_group_send_exception",
                )
            self._publish_status(
                "move_group_goal_sent",
                "goal_response_pending",
                request_id,
                attempt_generation=attempt_generation,
                goal_response_future_pending_at_timeout=True,
            )
            send_reason = self._poll_future(
                send_future,
                time.monotonic() + self._goal_response_s,
                goal_handle,
                request,
            )
            if send_reason is not None:
                self._publish_status(
                    "move_group_goal_response_timeout"
                    if send_reason == "move_group_timeout"
                    else "move_group_goal_response_failed",
                    send_reason,
                    request_id,
                    attempt_generation=attempt_generation,
                    goal_response_future_pending_at_timeout=True,
                )
                send_future.add_done_callback(
                    partial(
                        self._late_cancel_callback,
                        request_id=request_id,
                        attempt_generation=attempt_generation,
                    )
                )
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(
                    request, requested_at, success=False, reason=send_reason
                )
            try:
                moveit_goal_handle = send_future.result()
            except Exception as error:
                self._latch_active_attempt_fault(
                    f"move_group_goal_response_exception:{type(error).__name__}",
                    request_id,
                    attempt_generation,
                )
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="move_group_goal_response_exception",
                )
            if moveit_goal_handle is None or not moveit_goal_handle.accepted:
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="move_group_goal_rejected",
                )
            move_group_goal_id = self._goal_id_hex(moveit_goal_handle)
            with self._state_lock:
                if (
                    self._active_request_id != request_id
                    or self._active_attempt_generation != attempt_generation
                ):
                    goal_handle.abort()
                    return self._result(
                        request,
                        requested_at,
                        success=False,
                        reason="late_result_ignored",
                    )
                self._active_moveit_goal = moveit_goal_handle
                self._active_moveit_goal_id = move_group_goal_id
            if move_group_goal_id is None:
                self._latch_active_attempt_fault(
                    "move_group_goal_id_unavailable",
                    request_id,
                    attempt_generation,
                )
                self._publish_status(
                    "move_group_goal_id_unavailable",
                    "accepted_goal_not_correlatable",
                    request_id,
                    attempt_generation=attempt_generation,
                )
            else:
                self._publish_status(
                    "move_group_goal_accepted",
                    "result_pending",
                    request_id,
                    attempt_generation=attempt_generation,
                    move_group_goal_id=move_group_goal_id,
                )

            try:
                result_future = self._get_move_group_result_future(
                    moveit_goal_handle
                )
            except Exception as error:
                self._latch_active_attempt_fault(
                    f"move_group_result_request_exception:{type(error).__name__}",
                    request_id,
                    attempt_generation,
                )
                self._cancel_move_group(
                    request_id,
                    "result_request_exception",
                    attempt_generation=attempt_generation,
                    moveit_goal=moveit_goal_handle,
                )
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="move_group_result_request_exception",
                )
            if result_future is None:
                self._latch_active_attempt_fault(
                    "move_group_result_future_unavailable",
                    request_id,
                    attempt_generation,
                )
                self._publish_status(
                    "move_group_result_future_unavailable",
                    "accepted_goal_has_no_result_future",
                    request_id,
                    attempt_generation=attempt_generation,
                    move_group_goal_id=move_group_goal_id,
                    result_future_pending_at_timeout=None,
                    move_group_cancel_requested=True,
                    move_group_terminal_observed=False,
                )
                self._cancel_move_group(
                    request_id,
                    "result_future_unavailable",
                    attempt_generation=attempt_generation,
                    moveit_goal=moveit_goal_handle,
                    result_future=None,
                )
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="move_group_result_future_unavailable",
                )
            with self._state_lock:
                if (
                    self._active_request_id == request_id
                    and self._active_attempt_generation == attempt_generation
                ):
                    self._active_moveit_result_future = result_future
            if move_group_goal_id is None:
                terminal_confirmed = self._cancel_move_group(
                    request_id,
                    "move_group_goal_id_unavailable",
                    attempt_generation=attempt_generation,
                    moveit_goal=moveit_goal_handle,
                    result_future=result_future,
                )
                result_reason = "move_group_goal_id_unavailable"
                if not self._result_future_terminal_observed(result_future):
                    result_reason += ":move_group_terminal_unconfirmed"
                elif not terminal_confirmed:
                    result_reason += ":move_group_cancel_unconfirmed"
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=result_reason,
                )
            result_reason = self._poll_future(
                result_future,
                time.monotonic()
                + request.planning_timeout_s
                + self._result_margin_s,
                goal_handle,
                request,
                timeout_reason="move_group_result_timeout",
            )
            if result_reason is not None:
                pending_at_timeout = not result_future.done()
                self._publish_status(
                    "move_group_result_timeout"
                    if result_reason == "move_group_result_timeout"
                    else "move_group_cancel_requested",
                    result_reason,
                    request_id,
                    attempt_generation=attempt_generation,
                    move_group_goal_id=move_group_goal_id,
                    result_future_pending_at_timeout=pending_at_timeout,
                    move_group_cancel_requested=True,
                )
                terminal_confirmed = self._cancel_move_group(
                    request_id,
                    result_reason,
                    attempt_generation=attempt_generation,
                    moveit_goal=moveit_goal_handle,
                    result_future=result_future,
                )
                terminal_observed = self._result_future_terminal_observed(
                    result_future
                )
                if not terminal_observed:
                    self._observe_unconfirmed_move_group_terminal(
                        result_future,
                        request_id=request_id,
                        attempt_generation=attempt_generation,
                        goal_id=move_group_goal_id,
                    )
                    result_reason = (
                        f"{result_reason}:move_group_terminal_unconfirmed"
                    )
                elif (
                    self._result_future_status(result_future)
                    == GoalStatus.STATUS_SUCCEEDED
                ):
                    result_reason = f"{result_reason}:move_group_success_after_cancel"
                elif not terminal_confirmed:
                    result_reason = f"{result_reason}:move_group_cancel_unconfirmed"
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(
                    request, requested_at, success=False, reason=result_reason
                )
            # A done result and a client cancel can race between the poll loop
            # and result parsing.  Admit the result only after this second
            # cancellation check; a canceled wrapper must never reach gate.
            if goal_handle.is_cancel_requested:
                terminal_confirmed = self._cancel_move_group(
                    request_id,
                    "plan_target_cancel_requested",
                    attempt_generation=attempt_generation,
                    moveit_goal=moveit_goal_handle,
                    result_future=result_future,
                )
                reason = "plan_target_cancel_requested"
                if not self._result_future_terminal_observed(result_future):
                    reason += ":move_group_terminal_unconfirmed"
                elif (
                    self._result_future_status(result_future)
                    == GoalStatus.STATUS_SUCCEEDED
                ):
                    reason += ":move_group_success_after_cancel"
                elif not terminal_confirmed:
                    reason += ":move_group_cancel_unconfirmed"
                goal_handle.canceled()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=reason,
                )
            try:
                wrapped = result_future.result()
                moveit_result = wrapped.result
            except Exception as error:
                self._latch_active_attempt_fault(
                    f"move_group_result_exception:{type(error).__name__}",
                    request_id,
                    attempt_generation,
                )
                terminal_confirmed = self._cancel_move_group(
                    request_id,
                    "result_exception",
                    attempt_generation=attempt_generation,
                    moveit_goal=moveit_goal_handle,
                    result_future=result_future,
                )
                result_reason = "move_group_result_exception"
                if not self._result_future_terminal_observed(result_future):
                    result_reason += ":move_group_terminal_unconfirmed"
                elif (
                    self._result_future_status(result_future)
                    == GoalStatus.STATUS_SUCCEEDED
                ):
                    result_reason += ":move_group_success_after_cancel"
                elif not terminal_confirmed:
                    result_reason += ":move_group_cancel_unconfirmed"
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=result_reason,
                )
            # Parse-time cancellation can race with the result callback.  The
            # accepted result belongs to this generation, but cancellation is
            # still authoritative and no gate dispatch is permitted.
            if goal_handle.is_cancel_requested:
                terminal_confirmed = self._cancel_move_group(
                    request_id,
                    "plan_target_cancel_requested",
                    attempt_generation=attempt_generation,
                    moveit_goal=moveit_goal_handle,
                    result_future=result_future,
                )
                reason = "plan_target_cancel_requested"
                if not self._result_future_terminal_observed(result_future):
                    reason += ":move_group_terminal_unconfirmed"
                elif (
                    self._result_future_status(result_future)
                    == GoalStatus.STATUS_SUCCEEDED
                ):
                    reason += ":move_group_success_after_cancel"
                elif not terminal_confirmed:
                    reason += ":move_group_cancel_unconfirmed"
                goal_handle.canceled()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=reason,
                )
            self._publish_status(
                "move_group_terminal",
                "observed" if self._is_terminal_status(int(wrapped.status)) else "unconfirmed",
                request_id,
                attempt_generation=attempt_generation,
                move_group_goal_id=move_group_goal_id,
                move_group_terminal_observed=self._is_terminal_status(
                    int(wrapped.status)
                ),
                move_group_terminal_status=int(wrapped.status),
                action_goal_status=int(wrapped.status),
            )
            error_code = int(moveit_result.error_code.val)
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=f"move_group_action_status:{wrapped.status}",
                    moveit_error_code=error_code,
                )
            if error_code != MoveItErrorCodes.SUCCESS:
                detail = ":".join(
                    part
                    for part in (
                        str(getattr(moveit_result.error_code, "message", "")),
                        str(getattr(moveit_result.error_code, "source", "")),
                    )
                    if part
                )
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=f"moveit_error:{error_code}:{detail}",
                    moveit_error_code=error_code,
                )

            self._feedback(goal_handle, "validating_trajectory", request_id)
            reason = self._safety_reason(request)
            if reason is not None:
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=reason,
                    moveit_error_code=error_code,
                )
            boundary_now_ns, _ = self._observe_now()
            boundary_start = self._fresh_start_positions(boundary_now_ns)
            if boundary_start is None:
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="joint_states_missing_or_stale_after_planning",
                    moveit_error_code=error_code,
                )
            decision = validate_and_convert_robot_trajectory(
                moveit_result.planned_trajectory,
                fresh_start_positions=boundary_start,
                start_tolerance_rad=self._start_tolerance_rad,
            )
            if not decision.accepted or decision.joint_trajectory is None:
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=f"trajectory:{decision.reason.value}:{decision.detail}",
                    moveit_error_code=error_code,
                )
            reason = self._safety_reason(request)
            if reason is not None:
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=reason,
                    moveit_error_code=error_code,
                )
            with self._state_lock:
                if (
                    self._active_request_id != request_id
                    or self._active_attempt_generation != attempt_generation
                ):
                    goal_handle.abort()
                    return self._result(
                        request,
                        requested_at,
                        success=False,
                        reason="late_result_ignored",
                        moveit_error_code=error_code,
                    )
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._result(
                        request,
                        requested_at,
                        success=False,
                        reason="plan_target_cancel_requested",
                        moveit_error_code=error_code,
                    )
            # Last admission check immediately before sending the typed gate
            # goal.  This closes the done/cancel race after all trajectory
            # validation and request identity checks.
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="plan_target_cancel_requested",
                    moveit_error_code=error_code,
                )
            self._feedback(goal_handle, "executing_through_gate", request_id)
            gate = self._dispatch_to_gate(
                goal_handle,
                request,
                decision.joint_trajectory,
                attempt_generation=attempt_generation,
            )
            if not gate.success:
                if goal_handle.is_cancel_requested or gate.reason.startswith(
                    "plan_target_cancel_requested"
                ):
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=gate.reason,
                    moveit_error_code=error_code,
                    gate=gate,
                )
            with self._state_lock:
                active_attempt = (
                    self._active_request_id == request_id
                    and self._active_attempt_generation == attempt_generation
                )
            if goal_handle.is_cancel_requested or not active_attempt:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    reason = "plan_target_cancel_requested"
                else:
                    goal_handle.abort()
                    reason = "late_result_ignored"
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason=reason,
                    moveit_error_code=error_code,
                    gate=gate,
                )
            goal_handle.succeed()
            return self._result(
                request,
                requested_at,
                success=True,
                reason="trajectory_executed_through_gate",
                moveit_error_code=error_code,
                gate=gate,
            )
        finally:
            self._finish(request_id, attempt_generation)

    def _on_reset(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        with self._state_lock:
            active = self._reserved_request_id is not None
        if active:
            response.success = False
            response.message = "reset refused while a plan request is active"
            return response
        now_ns = self.get_clock().now().nanoseconds
        with self._state_lock:
            self._clock_epoch += 1
            self._last_clock_ns = now_ns
            self._joint_positions = {}
            self._last_joint_receive_ns = None
            self._fault_latched = None
            self._dispatched_command_ids.clear()
            self._planning_scene_status = None
            self._planning_scene_status_generation = 0
            self._planning_scene_status_event.clear()
        self._permission.reset_epoch(now_ns)
        self._interface_permission.reset_epoch(now_ns)
        self._planning_scene_permission.reset_epoch(now_ns)
        response.success = True
        response.message = f"MoveIt adapter reset to clock epoch {self._clock_epoch}"
        self._publish_status("reset", "waiting_for_fresh_inputs")
        return response

    def destroy_node(self) -> bool:
        # Wait for any publisher already inside _publish_status, then prevent
        # every late callback/watchdog from entering the publisher before the
        # rclpy entities are destroyed.
        with self._status_publish_lock:
            self._destroying.set()
        with self._late_terminal_watchdog_lock:
            watchdogs = tuple(self._late_terminal_watchdogs)
            self._late_terminal_watchdogs.clear()
        for watchdog in watchdogs:
            watchdog.cancel()
        watchdog_deadline = time.monotonic() + 0.25
        for watchdog in watchdogs:
            remaining = watchdog_deadline - time.monotonic()
            if remaining <= 0.0:
                break
            watchdog.join(timeout=remaining)
        self._plan_server.destroy()
        self._move_group.destroy()
        self._trajectory_gate.destroy()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MoveItPlanOnlyAdapter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # See the sequence wrapper: Jazzy may report an invalid wait set after
        # SIGINT has already shut down the context.  Do not hide exceptions
        # while the context is still live.
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        executor._executor.shutdown(wait=True, cancel_futures=False)
        with executor._tasks_lock:
            completed_tasks = tuple(executor._pending_tasks)
        for task in completed_tasks:
            if not task.done() or task.cancelled():
                continue
            error = task.exception()
            if error is not None and rclpy.ok():
                node.get_logger().error(
                    f"executor callback failed before shutdown: {error}"
                )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
