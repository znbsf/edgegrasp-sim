"""ROS action orchestration for the dependency-free grasp sequence core.

The dependency-free core owns stage positions; this wrapper additionally
freezes normalized approach and grasp orientations for the immutable task.
Descend and lift reuse the exact grasp orientation.
Arm stages call PlanTarget (which plans only and then waits for the typed
trajectory gate), while gripper close calls ExecuteTrajectory directly. A
sequence advances only after the correlated wrapper action and downstream FJT
terminal contracts both report success.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from functools import partial
import json
import math
from pathlib import Path
from threading import Event, RLock
import time
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from edgegrasp.config import (
    DEFAULT_TARGET_FRAME,
    DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS,
    ROS_SYSTEM_CLOCK_DOMAIN,
    validate_ros_clock_domain,
)
from edgegrasp.grasp_sequence import (
    ACTION_STATUS_ABORTED,
    ArmPlanCommand,
    ArmStageTerminal,
    CancelDisposition,
    CancelOutcome,
    DispatchOutcome,
    GraspPhase,
    GraspSequenceController,
    GraspTaskSnapshot,
    GripperExecutionCommand,
    GripperStageTerminal,
    SequenceDecision,
    SequenceHealth,
    SignalSnapshot,
    StopAllOutcome,
    _arm_terminal_reason,
)
from edgegrasp.models import Vector3
from edgegrasp.action_readiness import wait_for_readiness
from edgegrasp.target_motion import (
    LiftObservationBinding, PlannedLiftRegion, MeasuredCubeObservation, MeasuredPadBinding,
)
from edgegrasp.grasp_geometry import load_grasp_geometry_profile
from edgegrasp.so101_contract import (
    SO101_ARM_JOINTS,
    SO101_CONTRACT,
    SO101_GRIPPER_JOINTS,
    SO101_PLANNING_PIPELINES,
)
from edgegrasp.sim_control import GRIPPER_PRELOAD_EFFORT_LIMIT_NM
from edgegrasp.trajectory_identity import (
    make_trajectory_command_id,
    trajectory_digest,
)
from edgegrasp_interfaces.action import ExecuteTrajectory, GraspSequence, PlanTarget
from edgegrasp_interfaces.msg import GraspSequenceTerminal, TrackedTarget
from geometry_msgs.msg import Point
from rclpy.action import (
    ActionClient,
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener, TransformException


_UNKNOWN_STATUS = GoalStatus.STATUS_UNKNOWN
_UNAVAILABLE_FJT_ERROR = -(2**31)
_MAX_TARGET_CACHE = 64
_ACTION_TERMINAL_STATUSES = frozenset(
    (
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_CANCELED,
        GoalStatus.STATUS_ABORTED,
    )
)


@dataclass(frozen=True, slots=True)
class _BoolSample:
    ready: bool
    observed_at_ns: int
    reason: str


@dataclass(frozen=True, slots=True)
class _TargetSample:
    target_id: str
    position: tuple[float, float, float]
    frame_id: str
    source_timestamp_ns: int
    clock_domain: str
    clock_epoch: int
    observed_at_ns: int

    @property
    def key(self) -> tuple[str, int, str, int]:
        return (
            self.target_id,
            self.source_timestamp_ns,
            self.clock_domain,
            self.clock_epoch,
        )


@dataclass(frozen=True, slots=True)
class _JointSample:
    positions: dict[str, float]
    observed_at_ns: int


@dataclass(frozen=True, slots=True)
class _PlanningSceneStatus:
    ready: bool
    allow_target_pad_contacts: bool
    reason: str
    scene_digest: str
    observed_at_ns: int
    carried_state: str = "world"


@dataclass(slots=True)
class _ActionSlot:
    command: ArmPlanCommand | GripperExecutionCommand
    generation: int
    send_future: Any
    goal_handle: Any = None
    result_future: Any = None
    cancel_future: Any = None
    cancel_on_accept: bool = False
    execution_feedback_sent: bool = False
    terminal: bool = False
    expected_digest: str | None = None


class _ArmPort:
    def __init__(self, node: "GraspSequenceNode") -> None:
        self._node = node

    def submit(self, command: ArmPlanCommand) -> DispatchOutcome:
        return self._node._submit_arm(command)

    def cancel(self, command_id: str, reason: str) -> CancelOutcome:
        return self._node._cancel_slot("arm", command_id, reason)


class _GripperPort:
    def __init__(self, node: "GraspSequenceNode") -> None:
        self._node = node

    def submit(self, command: GripperExecutionCommand) -> DispatchOutcome:
        return self._node._submit_gripper(command)

    def cancel(self, command_id: str, reason: str) -> CancelOutcome:
        return self._node._cancel_slot("gripper", command_id, reason)


class _StopPort:
    def __init__(self, node: "GraspSequenceNode") -> None:
        self._node = node

    def stop_all(self, task_id: str, reason: str) -> StopAllOutcome:
        return self._node._stop_all(task_id, reason)


class GraspSequenceNode(Node):
    """One-in-flight fail-closed action wrapper around GraspSequenceController."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__("edgegrasp_grasp_sequence", **node_kwargs)
        self.declare_parameter("sequence_action", "/edgegrasp/grasp_sequence")
        self.declare_parameter(
            "sequence_terminal_topic", "/edgegrasp/grasp_sequence_terminal"
        )
        self.declare_parameter("plan_target_action", "/edgegrasp/plan_target")
        self.declare_parameter(
            "execute_trajectory_action", "/edgegrasp/execute_trajectory"
        )
        self.declare_parameter("tracked_target_topic", "/edgegrasp/tracked_target")
        self.declare_parameter("motion_allowed_topic", "/edgegrasp/motion_allowed")
        self.declare_parameter("interface_ready_topic", "/edgegrasp/interface_ready")
        self.declare_parameter(
            "planning_scene_status_topic", "/edgegrasp/planning_scene_status"
        )
        self.declare_parameter(
            "target_pad_contact_service", "/edgegrasp/set_target_pad_contacts"
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("target_frame", DEFAULT_TARGET_FRAME)
        self.declare_parameter("clock_domain", ROS_SYSTEM_CLOCK_DOMAIN)
        self.declare_parameter("clock_epoch", 0)
        self.declare_parameter("source_timeout_ms", 200.0)
        self.declare_parameter("health_timeout_ms", 200.0)
        self.declare_parameter("permission_timeout_ms", 100.0)
        self.declare_parameter("interface_timeout_ms", 500.0)
        self.declare_parameter("target_receive_timeout_ms", 200.0)
        self.declare_parameter("joint_state_timeout_ms", 250.0)
        self.declare_parameter("planning_scene_timeout_ms", 1500.0)
        self.declare_parameter("contact_policy_timeout_ms", 5000.0)
        self.declare_parameter(
            "future_skew_tolerance_ms", DEFAULT_ROS_FUTURE_SKEW_TOLERANCE_MS
        )
        self.declare_parameter("command_timeout_ms", 15_000.0)
        self.declare_parameter("target_drift_tolerance_m", 0.005)
        self.declare_parameter("observed_target_motion", "static")
        self.declare_parameter("observed_cube_geometry_profile", "")
        self.declare_parameter("max_planning_timeout_s", 2.0)
        self.declare_parameter("max_request_scaling", 0.2)
        self.declare_parameter("gripper_duration_s", 2.0)
        self.declare_parameter("gripper_preload_ramp_s", 0.5)
        self.declare_parameter("gripper_feedforward_effort_nm", 0.0)
        self.declare_parameter("loop_rate_hz", 50.0)

        self._target_frame = str(self.get_parameter("target_frame").value)
        self._clock_domain = str(self.get_parameter("clock_domain").value)
        self._clock_epoch = int(self.get_parameter("clock_epoch").value)
        self._use_sim_time = bool(self.get_parameter("use_sim_time").value)
        validate_ros_clock_domain(self._use_sim_time, self._clock_domain)
        if not self._target_frame or self._clock_epoch < 0:
            raise ValueError("target_frame and clock_epoch must be valid")
        self._source_timeout_ms = self._positive_parameter("source_timeout_ms")
        self._health_timeout_ms = self._positive_parameter("health_timeout_ms")
        self._permission_timeout_ms = self._positive_parameter(
            "permission_timeout_ms"
        )
        self._interface_timeout_ms = self._positive_parameter(
            "interface_timeout_ms"
        )
        self._target_receive_timeout_ms = self._positive_parameter(
            "target_receive_timeout_ms"
        )
        self._joint_state_timeout_ms = self._positive_parameter(
            "joint_state_timeout_ms"
        )
        self._planning_scene_timeout_ns = int(
            self._positive_parameter("planning_scene_timeout_ms") * 1_000_000
        )
        self._contact_policy_timeout_s = (
            self._positive_parameter("contact_policy_timeout_ms") / 1000.0
        )
        self._future_skew_tolerance_ns = int(
            self._positive_parameter("future_skew_tolerance_ms") * 1_000_000
        )
        self._command_timeout_ms = self._positive_parameter("command_timeout_ms")
        self._target_drift_tolerance_m = self._positive_parameter(
            "target_drift_tolerance_m"
        )
        self._observed_target_motion = str(self.get_parameter("observed_target_motion").value)
        if self._observed_target_motion not in ("static", "rigid_lift", "planned_lift_region", "measured_pad"):
            raise ValueError("observed_target_motion must be static, rigid_lift, planned_lift_region or measured_pad")
        if self._observed_target_motion != "static" and self._clock_domain != "ros_sim":
            raise ValueError("rigid lift observation is scoped to simulation")
        self._lift_observation_binding = None
        self._measured_cube_cache = OrderedDict()
        self._observed_cube_profile = None
        if self._observed_target_motion == "measured_pad":
            self._observed_cube_profile = load_grasp_geometry_profile(
                Path(str(self.get_parameter("observed_cube_geometry_profile").value)))
        self._observation_tf = Buffer() if self._observed_target_motion in ("rigid_lift", "measured_pad") else None
        self._observation_tf_listener = (
            TransformListener(self._observation_tf, self) if self._observation_tf else None
        )
        self._max_planning_timeout_s = self._positive_parameter(
            "max_planning_timeout_s"
        )
        self._max_request_scaling = self._positive_parameter(
            "max_request_scaling"
        )
        self._gripper_duration_s = self._positive_parameter("gripper_duration_s")
        self._gripper_preload_ramp_s = self._positive_parameter(
            "gripper_preload_ramp_s"
        )
        self._gripper_feedforward_effort_nm = float(
            self.get_parameter("gripper_feedforward_effort_nm").value
        )
        if (
            not math.isfinite(self._gripper_feedforward_effort_nm)
            or abs(self._gripper_feedforward_effort_nm)
            > GRIPPER_PRELOAD_EFFORT_LIMIT_NM
        ):
            raise ValueError(
                "gripper_feedforward_effort_nm must be finite and within "
                f"+/-{GRIPPER_PRELOAD_EFFORT_LIMIT_NM} N m"
            )
        loop_rate_hz = self._positive_parameter("loop_rate_hz")
        self._loop_period_s = 1.0 / loop_rate_hz

        self._callback_group = ReentrantCallbackGroup()
        # Preserve per-topic ordering without forcing unrelated high-rate
        # target, permission, interface and joint callbacks through one queue.
        # A single shared mutually-exclusive group produced real joint-state
        # liveness false positives while MoveIt action callbacks were active.
        self._target_callback_group = MutuallyExclusiveCallbackGroup()
        self._permission_callback_group = MutuallyExclusiveCallbackGroup()
        self._interface_callback_group = MutuallyExclusiveCallbackGroup()
        self._joint_state_callback_group = MutuallyExclusiveCallbackGroup()
        self._planning_scene_callback_group = MutuallyExclusiveCallbackGroup()
        self._state_lock = RLock()
        self._core_event_lock = RLock()
        self._wake = Event()
        self._last_clock_ns: int | None = None
        self._input_fault: str | None = None
        self._permission: _BoolSample | None = None
        self._interface: _BoolSample | None = None
        self._joint_state: _JointSample | None = None
        self._latest_target: _TargetSample | None = None
        self._planning_scene_status: _PlanningSceneStatus | None = None
        self._planning_scene_status_generation = 0
        self._planning_scene_status_event = Event()
        self._target_cache: OrderedDict[
            tuple[str, int, str, int], _TargetSample
        ] = OrderedDict()
        self._active_anchor: tuple[float, float, float] | None = None
        self._active_request: GraspSequence.Goal | None = None
        self._active_goal_handle: Any = None
        self._reserved_task_id: str | None = None
        self._client_cancel_requested = False
        self._arm_slot: _ActionSlot | None = None
        self._gripper_slot: _ActionSlot | None = None
        self._generation = 0
        self._uncertain_commands: set[str] = set()
        self._last_feedback: tuple[str, str | None, str] | None = None
        self._timeline_seq = 0
        self._last_unhandled_exception: str | None = None
        self._last_terminal_command_id = ""
        self._last_trajectory_digest = ""

        self._arm_client = ActionClient(
            self,
            PlanTarget,
            str(self.get_parameter("plan_target_action").value),
            callback_group=self._callback_group,
        )
        self._gripper_client = ActionClient(
            self,
            ExecuteTrajectory,
            str(self.get_parameter("execute_trajectory_action").value),
            callback_group=self._callback_group,
        )
        self._target_pad_contacts = self.create_client(
            SetBool,
            str(self.get_parameter("target_pad_contact_service").value),
            callback_group=self._callback_group,
        )
        self._status = self.create_publisher(
            String, "/edgegrasp/grasp_sequence_status", 10
        )
        # Derived, read-only evidence. This stream never participates in
        # admission or state transitions and is intentionally excluded from
        # the default raw-input MCAP replay allowlist.
        self._timeline = self.create_publisher(
            String, "/edgegrasp/grasp_timeline", 100
        )
        self._terminal = self.create_publisher(
            GraspSequenceTerminal,
            str(self.get_parameter("sequence_terminal_topic").value),
            10,
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
            callback_group=self._planning_scene_callback_group,
        )
        self.create_subscription(
            String if self._observed_target_motion == "measured_pad" else TrackedTarget,
            "/edgegrasp/measured_cube" if self._observed_target_motion == "measured_pad"
                else str(self.get_parameter("tracked_target_topic").value),
            self._on_measured_cube if self._observed_target_motion == "measured_pad" else self._on_target,
            10,
            callback_group=self._target_callback_group,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("motion_allowed_topic").value),
            self._on_permission,
            10,
            callback_group=self._permission_callback_group,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("interface_ready_topic").value),
            self._on_interface,
            10,
            callback_group=self._interface_callback_group,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            10,
            callback_group=self._joint_state_callback_group,
        )

        self._core = GraspSequenceController(
            _ArmPort(self),
            _GripperPort(self),
            _StopPort(self),
            target_frame=self._target_frame,
            clock_domain=self._clock_domain,
            clock_epoch=self._clock_epoch,
            source_timeout_ms=self._source_timeout_ms,
            health_timeout_ms=self._health_timeout_ms,
            permission_timeout_ms=self._permission_timeout_ms,
            interface_timeout_ms=self._interface_timeout_ms,
            target_receive_timeout_ms=self._target_receive_timeout_ms,
            joint_state_timeout_ms=self._joint_state_timeout_ms,
            command_timeout_ms=self._command_timeout_ms,
        )
        self.create_service(
            Trigger,
            "/edgegrasp/reset_grasp_sequence_epoch",
            self._on_reset,
            callback_group=self._callback_group,
        )
        self.create_service(Trigger, "/edgegrasp/finish_completed_grasp_cycle",
                            self._on_finish_completed_cycle, callback_group=self._callback_group)
        self._sequence_server = ActionServer(
            self,
            GraspSequence,
            str(self.get_parameter("sequence_action").value),
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            execute_callback=self._execute,
            callback_group=self._callback_group,
        )

    def _positive_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return value

    def _now_ns(self) -> int:
        with self._state_lock:
            # Read the ROS clock while holding the same lock that records the
            # previous value. Concurrent callbacks must not acquire a newer
            # timestamp and commit it before an older pre-lock read.
            now_ns = self.get_clock().now().nanoseconds
            if isinstance(now_ns, bool) or not isinstance(now_ns, int) or now_ns < 0:
                self._input_fault = "invalid_ros_clock"
                return -1
            if self._last_clock_ns is not None and now_ns < self._last_clock_ns:
                self._input_fault = (
                    f"clock_rollback:{now_ns}<{self._last_clock_ns}"
                )
            self._last_clock_ns = now_ns
        return now_ns

    def _wait_for_source_clock(
        self, source_ns: int, initial_now_ns: int
    ) -> tuple[int, str | None]:
        """Wait without accepting future data while distributed ROS time catches up."""

        now_ns = initial_now_ns
        if self._use_sim_time and now_ns == 0 and source_ns > 0:
            return now_ns, "ros_clock_uninitialized"

        future_ns = source_ns - now_ns
        if future_ns <= 0:
            return now_ns, None
        if future_ns > self._future_skew_tolerance_ns:
            return now_ns, "target_source_timestamp_future_exceeds_delivery_window"

        deadline = time.monotonic() + max(
            1.0,
            self._future_skew_tolerance_ns / 1_000_000_000.0 * 2.0,
        )
        while self.context.ok() and time.monotonic() < deadline:
            time.sleep(0.001)
            now_ns = self._now_ns()
            with self._state_lock:
                clock_fault = self._input_fault
            if clock_fault is not None and clock_fault.startswith(
                "clock_rollback:"
            ):
                return now_ns, clock_fault
            if now_ns >= source_ns:
                return now_ns, None
        return now_ns, "target_source_clock_catchup_timeout"

    @staticmethod
    def _stamp_ns(stamp) -> int:
        seconds = getattr(stamp, "sec", None)
        nanoseconds = getattr(stamp, "nanosec", None)
        # Do not coerce malformed ROS time fields.  In particular, int(1.5)
        # silently changes the clock value and can make an invalid source
        # timestamp appear fresh.  Generated ROS fields are integers, while
        # fake/static callers must obey the same contract.
        if (
            isinstance(seconds, bool)
            or not isinstance(seconds, int)
            or isinstance(nanoseconds, bool)
            or not isinstance(nanoseconds, int)
        ):
            raise ValueError("ROS timestamp must use integer sec/nanosec fields")
        if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
            raise ValueError("ROS timestamp must be normalized and non-negative")
        return seconds * 1_000_000_000 + nanoseconds

    @staticmethod
    def _assign_stamp(stamp, timestamp_ns: int) -> None:
        stamp.sec = int(timestamp_ns // 1_000_000_000)
        stamp.nanosec = int(timestamp_ns % 1_000_000_000)

    @staticmethod
    def _point_payload(point: JointTrajectoryPoint) -> dict[str, object]:
        return {
            "positions": point.positions,
            "velocities": point.velocities,
            "accelerations": point.accelerations,
            "effort": point.effort,
            "time_from_start_ns": GraspSequenceNode._stamp_ns(
                point.time_from_start
            ),
        }

    @staticmethod
    def _finite_point(point: Point) -> bool:
        return all(
            math.isfinite(float(value)) for value in (point.x, point.y, point.z)
        )

    def _on_permission(self, message: Bool) -> None:
        with self._core_event_lock:
            now_ns = self._now_ns()
            with self._state_lock:
                self._permission = _BoolSample(
                    bool(message.data),
                    now_ns,
                    "allowed" if message.data else "motion_denied",
                )
        self._wake.set()

    def _on_interface(self, message: Bool) -> None:
        with self._core_event_lock:
            now_ns = self._now_ns()
            with self._state_lock:
                self._interface = _BoolSample(
                    bool(message.data),
                    now_ns,
                    "ready" if message.data else "interface_not_ready",
                )
        self._wake.set()

    def _on_planning_scene_status(self, message: String) -> None:
        now_ns = self._now_ns()
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
            if payload.get("target_frame") != self._target_frame:
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
                carried_state=str(payload.get("carried_state", "world")),
            )
        except (KeyError, TypeError, ValueError) as error:
            with self._state_lock:
                self._planning_scene_status = None
                self._input_fault = (
                    "invalid_planning_scene_status:"
                    f"{type(error).__name__}"
                )
            self._planning_scene_status_event.set()
            self._wake.set()
            return
        with self._state_lock:
            self._planning_scene_status = sample
            self._planning_scene_status_generation += 1
        self._planning_scene_status_event.set()
        self._wake.set()

    def _planning_scene_policy_reason(
        self, *, allow: bool, now_ns: int
    ) -> str | None:
        with self._state_lock:
            status = self._planning_scene_status
        if status is None:
            return "planning_scene_status_unknown"
        if status.observed_at_ns > now_ns:
            return "planning_scene_status_in_future"
        if now_ns - status.observed_at_ns > self._planning_scene_timeout_ns:
            return "planning_scene_status_stale"
        # The loader keeps ``ready=true`` while revalidating an already
        # confirmed scene.  Treat that fresh heartbeat as the same retained
        # evidence; any failed/expired query publishes ready=false and is
        # still rejected below.  Requiring only the literal ``confirmed``
        # created a narrow, nondeterministic admission failure whenever a
        # sequence goal arrived during the periodic service call.
        confirmed_reasons = {"confirmed", "confirmed_revalidation_pending"}
        if not status.ready or status.reason not in confirmed_reasons:
            return f"planning_scene_not_confirmed:{status.reason}"
        if status.allow_target_pad_contacts != allow:
            return (
                "target_pad_contacts_policy_mismatch:"
                f"observed={status.allow_target_pad_contacts}:required={allow}"
            )
        return None

    def _enable_target_pad_contacts_after_descend(self) -> str | None:
        """Confirm the grasp-stage ACM, then require a newer target sample.

        This runs after the correlated descend FJT terminal and before the
        core is allowed to construct the immutable close command.  Approach
        and descend therefore remain collision checked with target-pad
        contacts disabled.  It is intentionally outside ``_core_event_lock``
        so target and status callbacks can continue to advance while the
        service is pending.
        """

        now_ns = self._now_ns()
        if self._planning_scene_policy_reason(allow=True, now_ns=now_ns) is None:
            return None
        if not self._target_pad_contacts.wait_for_service(
            timeout_sec=self._contact_policy_timeout_s
        ):
            return "target_pad_contact_policy_service_unavailable"
        with self._state_lock:
            generation_before_call = self._planning_scene_status_generation
        request = SetBool.Request()
        request.data = True
        try:
            future = self._target_pad_contacts.call_async(request)
        except Exception as error:
            return f"target_pad_contact_policy_call_exception:{type(error).__name__}"
        deadline = time.monotonic() + self._contact_policy_timeout_s
        while not future.done() and time.monotonic() < deadline:
            if self._cancel_is_requested():
                future.cancel()
                return "cancel_requested_during_contact_policy"
            self._planning_scene_status_event.wait(0.005)
            self._planning_scene_status_event.clear()
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

        confirmed_at_ns: int | None = None
        while time.monotonic() < deadline:
            if self._cancel_is_requested():
                return "cancel_requested_during_contact_policy_confirmation"
            now_ns = self._now_ns()
            with self._state_lock:
                generation = self._planning_scene_status_generation
                status = self._planning_scene_status
                input_fault = self._input_fault
            if input_fault is not None:
                return f"input_fault:{input_fault}"
            if (
                generation > generation_before_call
                and status is not None
                and self._planning_scene_policy_reason(
                    allow=True, now_ns=now_ns
                )
                is None
            ):
                confirmed_at_ns = status.observed_at_ns
                break
            self._planning_scene_status_event.wait(0.01)
            self._planning_scene_status_event.clear()
        if confirmed_at_ns is None:
            return "target_pad_contact_policy_confirmation_timeout"

        source_timeout_ns = int(self._source_timeout_ms * 1_000_000)
        while time.monotonic() < deadline:
            if self._cancel_is_requested():
                return "cancel_requested_waiting_for_post_policy_target"
            now_ns = self._now_ns()
            with self._state_lock:
                target = self._latest_target
                input_fault = self._input_fault
            if input_fault is not None:
                return f"input_fault:{input_fault}"
            if (
                target is not None
                and target.observed_at_ns >= confirmed_at_ns
                and target.source_timestamp_ns <= now_ns
                and now_ns - target.source_timestamp_ns <= source_timeout_ns
            ):
                self._publish_status(
                    "contact_policy_confirmed",
                    "target_pad_contacts_allowed_with_post_policy_target",
                    self._active_request.task_id
                    if self._active_request is not None
                    else "<none>",
                )
                return None
            self._wake.wait(0.01)
            self._wake.clear()
        return "post_policy_target_timeout"

    def _on_joint_state(self, message: JointState) -> None:
        with self._core_event_lock:
            now_ns = self._now_ns()
            names = tuple(message.name)
            expected = set(SO101_ARM_JOINTS + SO101_GRIPPER_JOINTS)
            if (
                len(names) != len(message.position)
                or len(set(names)) != len(names)
                or not expected.issubset(names)
                or any(not math.isfinite(float(value)) for value in message.position)
            ):
                with self._state_lock:
                    self._input_fault = "invalid_joint_states"
                    self._joint_state = None
                self._wake.set()
                return
            positions = dict(zip(names, message.position, strict=True))
            with self._state_lock:
                self._joint_state = _JointSample(
                    {joint: float(positions[joint]) for joint in expected}, now_ns
                )
        self._wake.set()

    def _on_measured_cube(self, message: String) -> None:
        try:
            observation = MeasuredCubeObservation.parse(json.loads(message.data))
            target = TrackedTarget()
            target.target_id = observation.target_id
            target.clock_domain, target.clock_epoch = observation.clock_domain, observation.clock_epoch
            target.observation.header.frame_id = observation.frame_id
            self._assign_stamp(target.observation.header.stamp, observation.source_ns)
            point = target.observation.point
            point.x, point.y, point.z = observation.center_m
            # Match the existing event -> state lock order. Holding state
            # while entering _on_target can deadlock a concurrent core event.
            with self._core_event_lock:
                with self._state_lock:
                    previous = self._measured_cube_cache.get(observation.key)
                    if previous is not None and previous != observation:
                        raise ValueError("measured cube source conflict")
                    self._measured_cube_cache[observation.key] = observation
                    self._measured_cube_cache.move_to_end(observation.key)
                    while len(self._measured_cube_cache) > _MAX_TARGET_CACHE:
                        self._measured_cube_cache.popitem(last=False)
                self._on_target(target)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            with self._state_lock:
                self._input_fault = f"invalid_measured_cube:{exc}"

    def _on_target(self, message: TrackedTarget) -> None:
        with self._core_event_lock:
            now_ns = self._now_ns()
            frame_id = message.observation.header.frame_id or "<empty>"
            try:
                source_ns = self._stamp_ns(message.observation.header.stamp)
            except (TypeError, ValueError, OverflowError):
                with self._state_lock:
                    self._input_fault = "invalid_target_timestamp"
                self._wake.set()
                return
            try:
                position = (
                    float(message.observation.point.x),
                    float(message.observation.point.y),
                    float(message.observation.point.z),
                )
            except (TypeError, ValueError, OverflowError):
                with self._state_lock:
                    self._input_fault = "invalid_target_observation"
                self._wake.set()
                return
            reason: str | None = None
            if source_ns > now_ns:
                now_ns, reason = self._wait_for_source_clock(source_ns, now_ns)
                if reason == "ros_clock_uninitialized":
                    # A newly created use_sim_time node has not received its
                    # first /clock sample yet. Do not accept or cache this
                    # target, but also do not permanently poison the epoch;
                    # the next stamped sample is evaluated after clock init.
                    self._wake.set()
                    return
            if reason is None and (
                not message.target_id
                or message.target_id.strip() != message.target_id
                or len(message.target_id) > 128
            ):
                reason = "invalid_target_id"
            elif reason is None and frame_id != self._target_frame:
                reason = f"target_frame_mismatch:{frame_id}!={self._target_frame}"
            elif reason is None and message.clock_domain != self._clock_domain:
                reason = "target_clock_domain_mismatch"
            elif reason is None and int(message.clock_epoch) != self._clock_epoch:
                reason = "target_clock_epoch_mismatch"
            elif reason is None and (source_ns < 0 or any(
                not math.isfinite(value) for value in position
            )):
                reason = "invalid_target_observation"
            sample = _TargetSample(
                target_id=message.target_id,
                position=position,
                frame_id=frame_id,
                source_timestamp_ns=source_ns,
                clock_domain=message.clock_domain,
                clock_epoch=int(message.clock_epoch),
                observed_at_ns=now_ns,
            )
            with self._state_lock:
                previous = self._latest_target
                if (
                    reason is None
                    and previous is not None
                    and previous.target_id == sample.target_id
                    and previous.clock_domain == sample.clock_domain
                    and previous.clock_epoch == sample.clock_epoch
                    and sample.source_timestamp_ns <= previous.source_timestamp_ns
                ):
                    reason = "target_source_timestamp_not_monotonic"
                if reason is not None:
                    self._input_fault = reason
                else:
                    self._latest_target = sample
                    self._target_cache[sample.key] = sample
                    self._target_cache.move_to_end(sample.key)
                    while len(self._target_cache) > _MAX_TARGET_CACHE:
                        self._target_cache.popitem(last=False)
        self._wake.set()

    def _request_target_sample(
        self, request: GraspSequence.Goal
    ) -> _TargetSample | None:
        key = (
            request.target.target_id,
            self._stamp_ns(request.target.observation.header.stamp),
            request.target.clock_domain,
            int(request.target.clock_epoch),
        )
        with self._state_lock:
            return self._target_cache.get(key)

    def _goal_reason(
        self,
        request: GraspSequence.Goal,
        *,
        require_locally_observed_target: bool,
    ) -> str | None:
        try:
            make_trajectory_command_id(request.task_id, "approach", 0)
        except (TypeError, ValueError):
            return "invalid_task_id"
        target = request.target
        if (
            not target.target_id
            or target.target_id.strip() != target.target_id
            or len(target.target_id) > 128
        ):
            return "invalid_target_id"
        if target.observation.header.frame_id != self._target_frame:
            return "target_frame_mismatch"
        if target.clock_domain != self._clock_domain:
            return "target_clock_domain_mismatch"
        if int(target.clock_epoch) != self._clock_epoch:
            return "target_clock_epoch_mismatch"
        try:
            self._stamp_ns(target.observation.header.stamp)
        except (TypeError, ValueError, OverflowError):
            return "invalid_target_timestamp"
        if not self._finite_point(target.observation.point):
            return "invalid_target_point"
        if not all(
            self._finite_point(point)
            for point in (
                request.approach_position,
                request.descend_position,
                request.lift_position,
            )
        ):
            return "invalid_stage_position"
        for label, orientation in (
            ("approach", request.approach_orientation),
            ("grasp", request.grasp_orientation),
        ):
            orientation_values = (
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            )
            if any(not math.isfinite(value) for value in orientation_values):
                return f"invalid_{label}_orientation"
            orientation_norm = math.sqrt(
                sum(value * value for value in orientation_values)
            )
            if abs(orientation_norm - 1.0) > 1e-3:
                return f"{label}_orientation_not_normalized"
        if request.pipeline_id not in SO101_PLANNING_PIPELINES:
            return "unsupported_planning_pipeline"
        if (
            not math.isfinite(request.planning_timeout_s)
            or not 0.0 < request.planning_timeout_s <= self._max_planning_timeout_s
        ):
            return "invalid_planning_timeout"
        for name, value in (
            ("velocity_scaling", request.velocity_scaling),
            ("acceleration_scaling", request.acceleration_scaling),
        ):
            if (
                not math.isfinite(value)
                or not 0.0 < value <= self._max_request_scaling
            ):
                return f"invalid_{name}"
        lower, upper = SO101_CONTRACT["edgegrasp_admission_profile"][
            "joint_position_bounds_rad"
        ]["gripper"]
        if (
            not math.isfinite(request.gripper_closed_position_rad)
            or not float(lower)
            <= request.gripper_closed_position_rad
            <= float(upper)
        ):
            return "gripper_position_out_of_bounds"
        sample = self._request_target_sample(request)
        if sample is None:
            return (
                "target_snapshot_not_observed_locally"
                if require_locally_observed_target
                else None
            )
        requested_position = (
            float(target.observation.point.x),
            float(target.observation.point.y),
            float(target.observation.point.z),
        )
        if sample.position != requested_position:
            return "target_snapshot_payload_mismatch"
        return None

    def _on_goal(self, request: GraspSequence.Goal) -> GoalResponse:
        try:
            reason = self._goal_reason(
                request, require_locally_observed_target=False
            )
        except Exception as error:
            reason = f"invalid_goal:{type(error).__name__}"
        if reason is not None:
            self._publish_status("goal_rejected", reason, request.task_id)
            return GoalResponse.REJECT
        with self._core_event_lock:
            phase = self._core.phase
            with self._state_lock:
                if (
                    self._reserved_task_id is not None
                    or phase is not GraspPhase.IDLE
                ):
                    self._publish_status(
                        "goal_rejected",
                        "sequence_busy_or_reset_required",
                        request.task_id,
                    )
                    return GoalResponse.REJECT
                self._reserved_task_id = request.task_id
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        # Serialize cancellation with every terminal callback.  Merely setting
        # a flag left a window where a concurrent success result could issue
        # the next stage before the execute loop observed the flag.
        with self._core_event_lock:
            with self._state_lock:
                if goal_handle is not self._active_goal_handle:
                    return CancelResponse.REJECT
                if self._core.phase in (
                    GraspPhase.COMPLETE,
                    GraspPhase.SAFE_STOP,
                    GraspPhase.IDLE,
                ):
                    return CancelResponse.REJECT
                self._client_cancel_requested = True
            now_ns = self._now_ns()
            decision = self._core.request_stop("client_cancel", now_ns)
            self._handle_decision(decision)
        return CancelResponse.ACCEPT

    def _cancel_is_requested(self) -> bool:
        """Return the latched/client-visible cancel state.

        The action handle can observe a cancel request before the cancel
        callback has acquired ``_core_event_lock``.  Checking both sources at
        every dispatch/terminal boundary closes that small race: a successful
        result must not issue the next sequence stage after cancellation was
        already requested.
        """

        with self._state_lock:
            requested = self._client_cancel_requested
            goal_handle = self._active_goal_handle
        if requested or goal_handle is None:
            return requested
        try:
            return bool(goal_handle.is_cancel_requested)
        except Exception:
            # A destroyed/invalid action handle is not evidence that motion is
            # safe; retain the conservative latched flag.
            return requested

    def _signal(
        self, sample: _BoolSample | None, missing_reason: str
    ) -> SignalSnapshot:
        if sample is None:
            return SignalSnapshot(False, 0, missing_reason)
        return SignalSnapshot(sample.ready, sample.observed_at_ns, sample.reason)

    def _coherent_lift_target(self, latest, now_ns):
        """Choose the newest fresh image with TF at that exact image time.

        Image and joint-state callbacks arrive independently. A newer image
        awaiting TF does not invalidate an already complete, still-fresh pair.
        Never extrapolate TF, restamp a sample, or select by residual quality.
        """
        with self._state_lock:
            candidates = list(self._target_cache.values())
        candidates = [latest, *(item for item in candidates
            if item.key != latest.key
            and item.target_id == latest.target_id
            and item.frame_id == latest.frame_id
            and item.clock_domain == latest.clock_domain
            and item.clock_epoch == latest.clock_epoch
            and item.source_timestamp_ns < latest.source_timestamp_ns)]
        candidates.sort(key=lambda item: item.source_timestamp_ns, reverse=True)
        for sample in candidates:
            if not 0 <= now_ns - sample.source_timestamp_ns <= self._source_timeout_ms * 1_000_000:
                continue
            if (self._lift_observation_binding is None
                    or sample.source_timestamp_ns < self._lift_observation_binding.bound_source_ns):
                continue
            try:
                self._observed_gripper_pose(sample.source_timestamp_ns)
                return sample
            except TransformException:
                continue
        return latest  # Existing health checks refuse missing TF or stale input.

    def _health(
        self, now_ns: int, *, initial_request: GraspSequence.Goal | None = None
    ) -> SequenceHealth:
        with self._state_lock:
            request = initial_request or self._active_request
            permission = self._permission
            interface = self._interface
            joint_state = self._joint_state
            input_fault = self._input_fault
            if request is None:
                raise RuntimeError("sequence request is unavailable")
            target = (
                self._target_cache.get(
                    (
                        request.target.target_id,
                        self._stamp_ns(request.target.observation.header.stamp),
                        request.target.clock_domain,
                        int(request.target.clock_epoch),
                    )
                )
                if initial_request is not None
                else self._latest_target
            )
            anchor = self._active_anchor
        expected_target_id = request.target.target_id
        if (initial_request is None and input_fault is None and target is not None
                and target.target_id == expected_target_id
                and target.frame_id == self._target_frame
                and target.clock_domain == self._clock_domain
                and target.clock_epoch == self._clock_epoch
                and self._observed_target_motion in ("rigid_lift", "measured_pad")
                # Planning already advances the core source watermark. Use
                # the same causal pair selection before execution starts.
                and self._core.phase in (GraspPhase.LIFT_PLAN, GraspPhase.LIFT_EXEC)):
            target = self._coherent_lift_target(target, now_ns)
        target_ready = input_fault is None and target is not None
        target_reason = input_fault or "target_unknown"
        target_id = expected_target_id
        source_ns = self._stamp_ns(request.target.observation.header.stamp)
        observed_ns = 0
        if target is not None:
            target_id = target.target_id
            source_ns = target.source_timestamp_ns
            observed_ns = target.observed_at_ns
            if input_fault is not None:
                target_ready = False
                target_reason = input_fault
            elif target.target_id != expected_target_id:
                target_ready = False
                target_reason = "target_identity_changed"
            elif (
                target.frame_id != self._target_frame
                or target.clock_domain != self._clock_domain
                or target.clock_epoch != self._clock_epoch
            ):
                target_ready = False
                target_reason = "target_clock_or_frame_mismatch"
            elif anchor is not None:
                drift = math.sqrt(
                    sum(
                        (actual - original) ** 2
                        for actual, original in zip(
                            target.position, anchor, strict=True
                        )
                    )
                )
                if (self._observed_target_motion == "rigid_lift"
                        and self._core.phase is GraspPhase.LIFT_EXEC):
                    try:
                        if self._lift_observation_binding is None:
                            raise ValueError("lift_binding_missing")
                        position, orientation = self._observed_gripper_pose(target.source_timestamp_ns)
                        drift = self._lift_observation_binding.residual_m(
                            task_id=request.task_id, target_id=target.target_id,
                            clock_domain=target.clock_domain, clock_epoch=target.clock_epoch,
                            source_ns=target.source_timestamp_ns, observed_center_m=target.position,
                            gripper_position_m=position, gripper_orientation_xyzw=orientation)
                    except (TransformException, ValueError) as exc:
                        drift = math.inf
                        target_reason = f"lift_observation_unavailable:{exc}"
                if (self._observed_target_motion == "planned_lift_region"
                        and self._core.phase is GraspPhase.LIFT_EXEC):
                    try:
                        if self._lift_observation_binding is None:
                            raise ValueError("lift_binding_missing")
                        drift = self._lift_observation_binding.residual_m(
                            task_id=request.task_id, target_id=target.target_id,
                            clock_domain=target.clock_domain, clock_epoch=target.clock_epoch,
                            source_ns=target.source_timestamp_ns, observed_center_m=target.position)
                    except ValueError as exc:
                        drift = math.inf
                        target_reason = f"lift_observation_unavailable:{exc}"
                if (self._observed_target_motion == "measured_pad"
                        and self._core.phase is GraspPhase.LIFT_EXEC):
                    try:
                        with self._state_lock:
                            observation = self._measured_cube_cache.get(target.key)
                        if self._lift_observation_binding is None or observation is None:
                            raise ValueError("measured_cube_binding_missing")
                        if observation.center_m != target.position:
                            raise ValueError("measured_cube_center_mismatch")
                        position, orientation = self._observed_gripper_pose(target.source_timestamp_ns)
                        drift = self._lift_observation_binding.residual_m(
                            task_id=request.task_id, observation=observation,
                            gripper_position_m=position, gripper_orientation_xyzw=orientation)
                    except (TransformException, ValueError) as exc:
                        drift = math.inf
                        target_reason = f"lift_observation_unavailable:{exc}"
                if drift > self._target_drift_tolerance_m:
                    target_ready = False
                    if math.isfinite(drift):
                        target_reason = f"target_geometry_drift:{drift}"
                else:
                    target_reason = "target_fresh"
            else:
                target_reason = "target_fresh"
        joint_signal = (
            SignalSnapshot(False, 0, "joint_states_unknown")
            if joint_state is None
            else SignalSnapshot(True, joint_state.observed_at_ns, "joint_states_fresh")
        )
        return SequenceHealth(
            target_id=target_id,
            target_source_timestamp_ns=source_ns,
            permission=self._signal(permission, "permission_unknown"),
            interface=self._signal(interface, "interface_unknown"),
            target=SignalSnapshot(target_ready, observed_ns, target_reason),
            joint_state=joint_signal,
            clock_domain=self._clock_domain,
            clock_epoch=self._clock_epoch,
        )

    def _task_snapshot(self, request: GraspSequence.Goal) -> GraspTaskSnapshot:
        return GraspTaskSnapshot(
            task_id=request.task_id,
            target_id=request.target.target_id,
            approach_position_m=Vector3(
                request.approach_position.x,
                request.approach_position.y,
                request.approach_position.z,
            ),
            descend_position_m=Vector3(
                request.descend_position.x,
                request.descend_position.y,
                request.descend_position.z,
            ),
            lift_position_m=Vector3(
                request.lift_position.x,
                request.lift_position.y,
                request.lift_position.z,
            ),
            approach_orientation_xyzw=(
                request.approach_orientation.x,
                request.approach_orientation.y,
                request.approach_orientation.z,
                request.approach_orientation.w,
            ),
            grasp_orientation_xyzw=(
                request.grasp_orientation.x,
                request.grasp_orientation.y,
                request.grasp_orientation.z,
                request.grasp_orientation.w,
            ),
            gripper_closed_position_rad=request.gripper_closed_position_rad,
            source_timestamp_ns=self._stamp_ns(
                request.target.observation.header.stamp
            ),
            frame_id=request.target.observation.header.frame_id,
            clock_domain=request.target.clock_domain,
            clock_epoch=int(request.target.clock_epoch),
        )

    def _publish_timeline(self, event_type: str, **fields: object) -> None:
        """Publish one stable JSON event on the local ROS clock.

        This is diagnostic evidence only.  A telemetry failure must never
        alter the command state machine, while the missing event remains
        visible in the captured artifact.
        """

        try:
            now_ns = self._now_ns()
            with self._state_lock:
                self._timeline_seq += 1
                timeline_seq = self._timeline_seq
                request = self._active_request
            task_id = "<none>" if request is None else request.task_id
            target_id = (
                "<none>" if request is None else request.target.target_id
            )
            source_timestamp_ns = (
                0
                if request is None
                else self._stamp_ns(request.target.observation.header.stamp)
            )
            payload: dict[str, object] = {
                "schema_version": 1,
                "timeline_seq": timeline_seq,
                "event_type": event_type,
                "event_stamp_ns": now_ns,
                "received_at_ns": now_ns,
                "source_timestamp_ns": source_timestamp_ns,
                "clock_domain": self._clock_domain,
                "clock_epoch": self._clock_epoch,
                "task_id": task_id,
                "target_id": target_id,
            }
            payload.update(fields)
            message = String()
            message.data = json.dumps(
                payload,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            self._timeline.publish(message)
        except BaseException as error:
            self.get_logger().error(
                "failed to publish grasp timeline evidence: "
                f"{type(error).__name__}:{error}"
            )

    def _publish_feedback(self, reason: str) -> None:
        with self._state_lock:
            goal_handle = self._active_goal_handle
            request = self._active_request
            command_id = self._core.active_command_id or ""
            current = (self._core.phase.value, command_id or None, reason)
            if goal_handle is None or request is None or current == self._last_feedback:
                return
            self._last_feedback = current
        feedback = GraspSequence.Feedback()
        feedback.task_id = request.task_id
        feedback.phase = self._core.phase.value
        feedback.command_id = command_id
        feedback.reason = reason
        try:
            goal_handle.publish_feedback(feedback)
        except Exception as error:
            self._publish_status(
                "feedback_error", type(error).__name__, request.task_id
            )
        self._publish_status(self._core.phase.value, reason, request.task_id)
        self._publish_timeline(
            "sequence_phase",
            phase=self._core.phase.value,
            stage=self._core.phase.value.lower(),
            command_id=command_id,
            reason=reason,
        )

    def _handle_decision(self, decision: SequenceDecision) -> None:
        self._publish_feedback(decision.reason)
        self._wake.set()

    def _result(self, accepted: bool) -> GraspSequence.Result:
        result = GraspSequence.Result()
        with self._state_lock:
            request = self._active_request
        outcome = self._core.outcome
        result.task_id = "" if request is None else request.task_id
        result.accepted = accepted
        result.sequence_completed = bool(
            outcome is not None and outcome.sequence_completed
        )
        result.physics_grasp_verified = False
        result.terminal_phase = self._core.phase.value
        result.reason = (
            outcome.reason
            if outcome is not None
            else "sequence_terminated_without_outcome"
        )
        result.active_command_id = self._core.active_command_id or ""
        with self._state_lock:
            result.last_terminal_command_id = self._last_terminal_command_id
            result.last_trajectory_digest = self._last_trajectory_digest
        result.clock_epoch = self._clock_epoch
        return result

    def _execute(self, goal_handle) -> GraspSequence.Result:
        """Never let a wrapper exception escape as an empty ROS action result."""

        try:
            result = self._execute_impl(goal_handle)
            self._publish_terminal(result, goal_handle)
            return result
        except BaseException as error:
            detail = f"{type(error).__name__}:{error}"
            self._last_unhandled_exception = detail
            self.get_logger().error(f"grasp sequence wrapper escaped: {detail}")
            try:
                with self._core_event_lock:
                    self._core.request_stop(
                        f"wrapper_unhandled_exception:{type(error).__name__}",
                        self._now_ns(),
                    )
            except BaseException:
                pass
            try:
                goal_handle.abort()
            except BaseException:
                pass
            result = GraspSequence.Result()
            result.task_id = getattr(goal_handle.request, "task_id", "")
            result.accepted = False
            result.sequence_completed = False
            result.physics_grasp_verified = False
            result.terminal_phase = GraspPhase.SAFE_STOP.value
            result.reason = f"wrapper_unhandled_exception:{detail}"
            result.active_command_id = self._core.active_command_id or ""
            with self._state_lock:
                result.last_terminal_command_id = self._last_terminal_command_id
                result.last_trajectory_digest = self._last_trajectory_digest
            result.clock_epoch = self._clock_epoch
            with self._state_lock:
                self._active_goal_handle = None
                self._reserved_task_id = None
                self._client_cancel_requested = False
            self._publish_terminal(result, goal_handle)
            return result

    def _publish_terminal(
        self, result: GraspSequence.Result, goal_handle
    ) -> None:
        """Publish one correlated protocol terminal; never alter action outcome."""

        try:
            request = goal_handle.request
            message = GraspSequenceTerminal()
            self._assign_stamp(message.terminal_stamp, self._now_ns())
            message.task_id = result.task_id
            message.target_id = request.target.target_id
            message.lift_command_id = result.last_terminal_command_id
            message.accepted = bool(result.accepted)
            message.sequence_completed = bool(result.sequence_completed)
            message.terminal_phase = result.terminal_phase
            message.reason = result.reason
            message.trajectory_digest = result.last_trajectory_digest
            message.action_goal_status = int(
                getattr(goal_handle, "status", GoalStatus.STATUS_UNKNOWN)
            )
            message.source_timestamp_ns = self._stamp_ns(
                request.target.observation.header.stamp
            )
            message.clock_domain = request.target.clock_domain
            message.clock_epoch = int(result.clock_epoch)
            self._terminal.publish(message)
            self._publish_timeline(
                "sequence_terminal",
                phase=result.terminal_phase,
                stage="terminal",
                command_id=result.last_terminal_command_id,
                accepted=result.accepted,
                sequence_completed=result.sequence_completed,
                physics_grasp_verified=False,
                action_goal_status=message.action_goal_status,
                trajectory_digest=result.last_trajectory_digest,
                reason=result.reason,
            )
        except BaseException as error:
            self.get_logger().error(
                "failed to publish grasp sequence terminal evidence: "
                f"{type(error).__name__}:{error}"
            )

    def _execute_impl(self, goal_handle) -> GraspSequence.Result:
        request = goal_handle.request
        with self._state_lock:
            if self._reserved_task_id != request.task_id:
                goal_handle.abort()
                return self._result(False)
            self._active_goal_handle = goal_handle
            self._active_request = request
            self._active_anchor = (
                float(request.target.observation.point.x),
                float(request.target.observation.point.y),
                float(request.target.observation.point.z),
            )
            self._client_cancel_requested = False
            self._last_feedback = None
            self._last_terminal_command_id = ""
            self._last_trajectory_digest = ""
        accepted = False
        try:
            target_deadline = (
                time.monotonic() + self._health_timeout_ms / 1000.0
            )
            while (
                self._request_target_sample(request) is None
                and time.monotonic() < target_deadline
            ):
                self._wake.wait(min(self._loop_period_s, 0.02))
                self._wake.clear()
            now_ns = self._now_ns()
            scene_reason = self._planning_scene_policy_reason(
                allow=False, now_ns=now_ns
            )
            if scene_reason is not None:
                self._publish_status(
                    "goal_rejected",
                    f"approach_contact_policy:{scene_reason}",
                    request.task_id,
                )
                goal_handle.abort()
                return self._result(False)
            with self._core_event_lock:
                now_ns = self._now_ns()
                try:
                    decision = self._core.start(
                        self._task_snapshot(request),
                        self._health(now_ns, initial_request=request),
                        now_ns,
                    )
                except Exception as error:
                    self._publish_status(
                        "wrapper_start_exception",
                        type(error).__name__,
                        request.task_id,
                    )
                    goal_handle.abort()
                    return self._result(False)
                accepted = decision.accepted
                self._handle_decision(decision)
            while True:
                with self._core_event_lock:
                    if self._core.phase in (
                        GraspPhase.COMPLETE,
                        GraspPhase.SAFE_STOP,
                    ):
                        break
                with self._state_lock:
                    cancel_requested = self._client_cancel_requested
                with self._core_event_lock:
                    now_ns = self._now_ns()
                    if cancel_requested:
                        decision = self._core.request_stop("client_cancel", now_ns)
                    else:
                        try:
                            decision = self._core.tick(self._health(now_ns), now_ns)
                        except Exception as error:
                            decision = self._core.request_stop(
                                f"wrapper_health_exception:{type(error).__name__}",
                                now_ns,
                            )
                    self._handle_decision(decision)
                self._wake.wait(self._loop_period_s)
                self._wake.clear()
            result = self._result(accepted)
            with self._state_lock:
                cancel_requested = self._client_cancel_requested
            try:
                cancel_requested = cancel_requested or bool(
                    goal_handle.is_cancel_requested
                )
            except Exception:
                # An invalid action handle is not evidence of a successful
                # completion; keep the conservative internal flag.
                pass
            if result.sequence_completed and not cancel_requested:
                goal_handle.succeed()
            elif cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except Exception as error:
            with self._core_event_lock:
                now_ns = self._now_ns()
                self._core.request_stop(
                    f"wrapper_execute_exception:{type(error).__name__}", now_ns
                )
            try:
                goal_handle.abort()
            except Exception:
                pass
            return self._result(accepted)
        finally:
            with self._state_lock:
                self._active_goal_handle = None
                self._reserved_task_id = None
                self._client_cancel_requested = False

    def _next_generation(self) -> int:
        with self._state_lock:
            self._generation += 1
            return self._generation

    def _observed_gripper_pose(self, source_ns):
        transform = self._observation_tf.lookup_transform(
            self._target_frame, "gripper_frame_link",
            Time(nanoseconds=source_ns, clock_type=self.get_clock().clock_type))
        if self._stamp_ns(transform.header.stamp) != source_ns:
            raise ValueError("gripper_tf_not_at_observation_source_time")
        p, q = transform.transform.translation, transform.transform.rotation
        return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)

    def _submit_arm(self, command: ArmPlanCommand) -> DispatchOutcome:
        with self._state_lock:
            request = self._active_request
            if self._client_cancel_requested:
                return DispatchOutcome(False, "client_cancel_pending")
            if self._arm_slot is not None:
                return DispatchOutcome(False, "arm_sequence_client_busy")
        if request is None:
            return DispatchOutcome(False, "sequence_request_unavailable")
        if not self._arm_client.server_is_ready():
            return DispatchOutcome(False, "plan_target_unavailable")
        if self._observed_target_motion == "rigid_lift" and command.stage == "lift":
            try:
                with self._state_lock:
                    target = self._target_cache.get((command.target_id, command.source_timestamp_ns,
                                                     command.clock_domain, command.clock_epoch))
                if target is None:
                    raise ValueError("bound lift target missing")
                position, orientation = self._observed_gripper_pose(target.source_timestamp_ns)
                self._lift_observation_binding = LiftObservationBinding.bind(
                    task_id=command.task_id, target_id=target.target_id,
                    clock_domain=target.clock_domain, clock_epoch=target.clock_epoch,
                    source_ns=target.source_timestamp_ns, observed_center_m=target.position,
                    gripper_position_m=position, gripper_orientation_xyzw=orientation)
            except (TransformException, ValueError) as exc:
                return DispatchOutcome(False, f"lift_binding_rejected:{exc}")
        if self._observed_target_motion == "planned_lift_region" and command.stage == "lift":
            try:
                if self._active_anchor is None:
                    raise ValueError("initial target anchor missing")
                self._lift_observation_binding = PlannedLiftRegion.bind(
                    task_id=command.task_id, target_id=command.target_id,
                    clock_domain=command.clock_domain, clock_epoch=command.clock_epoch,
                    source_ns=command.source_timestamp_ns, initial_center_m=self._active_anchor,
                    descend_position_m=(request.descend_position.x, request.descend_position.y,
                                        request.descend_position.z),
                    lift_position_m=(request.lift_position.x, request.lift_position.y,
                                     request.lift_position.z))
            except ValueError as exc:
                return DispatchOutcome(False, f"lift_binding_rejected:{exc}")
        if self._observed_target_motion == "measured_pad" and command.stage == "lift":
            try:
                with self._state_lock:
                    observation = self._measured_cube_cache.get((command.target_id,
                        command.source_timestamp_ns, command.clock_domain, command.clock_epoch))
                if observation is None:
                    raise ValueError("measured cube missing at lift binding")
                self._observed_gripper_pose(observation.source_ns)
                self._lift_observation_binding = MeasuredPadBinding.bind(
                    task_id=command.task_id, observation=observation, profile=self._observed_cube_profile)
            except (TransformException, ValueError) as exc:
                return DispatchOutcome(False, f"lift_binding_rejected:{exc}")
        goal = PlanTarget.Goal()
        goal.task_id = command.task_id
        goal.stage = command.stage
        goal.sequence_no = command.sequence_no
        goal.target_pose.header.frame_id = command.frame_id
        self._assign_stamp(goal.target_pose.header.stamp, command.source_timestamp_ns)
        goal.target_pose.pose.position.x = command.target_position_m.x
        goal.target_pose.pose.position.y = command.target_position_m.y
        goal.target_pose.pose.position.z = command.target_position_m.z
        # The core binds approach to its routing quaternion and descend/lift to
        # the immutable grasp quaternion. No stage re-snapshots live TF/pose.
        goal.target_pose.pose.orientation.x = command.target_orientation_xyzw[0]
        goal.target_pose.pose.orientation.y = command.target_orientation_xyzw[1]
        goal.target_pose.pose.orientation.z = command.target_orientation_xyzw[2]
        goal.target_pose.pose.orientation.w = command.target_orientation_xyzw[3]
        goal.target_id = command.target_id
        goal.source_timestamp_ns = command.source_timestamp_ns
        goal.clock_domain = command.clock_domain
        goal.clock_epoch = command.clock_epoch
        goal.planning_group = "arm"
        goal.pipeline_id = request.pipeline_id
        goal.planner_id = request.planner_id
        goal.planning_timeout_s = request.planning_timeout_s
        goal.velocity_scaling = request.velocity_scaling
        goal.acceleration_scaling = request.acceleration_scaling
        goal.plan_only = True
        generation = self._next_generation()
        try:
            send_future = self._arm_client.send_goal_async(
                goal,
                feedback_callback=partial(
                    self._on_arm_feedback, command.command_id, generation
                ),
            )
        except Exception as error:
            return DispatchOutcome(
                False, f"plan_target_send_exception:{type(error).__name__}"
            )
        slot = _ActionSlot(command, generation, send_future)
        with self._state_lock:
            if self._arm_slot is not None:
                return DispatchOutcome(False, "arm_sequence_client_race")
            self._arm_slot = slot
        try:
            send_future.add_done_callback(
                partial(self._on_arm_goal_response, command.command_id, generation)
            )
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            return DispatchOutcome(
                False, f"plan_target_goal_callback_exception:{type(error).__name__}"
            )
        self._publish_timeline(
            "arm_command_dispatched",
            phase=self._core.phase.value,
            stage=command.stage,
            command_id=command.command_id,
            sequence_no=command.sequence_no,
            controller="arm_controller",
            source_timestamp_ns=command.source_timestamp_ns,
            reason="plan_target_dispatched",
        )
        return DispatchOutcome(True, "plan_target_dispatched")

    def _submit_gripper(
        self, command: GripperExecutionCommand
    ) -> DispatchOutcome:
        with self._state_lock:
            joint_state = self._joint_state
            if self._client_cancel_requested:
                return DispatchOutcome(False, "client_cancel_pending")
            if self._gripper_slot is not None:
                return DispatchOutcome(False, "gripper_sequence_client_busy")
        if joint_state is None or "gripper" not in joint_state.positions:
            return DispatchOutcome(False, "gripper_joint_state_unavailable")
        if not self._gripper_client.server_is_ready():
            started = time.monotonic()
            ready = wait_for_readiness(self._gripper_client.server_is_ready)
            self.get_logger().info(json.dumps(dict(type="gripper_readiness_wait", ready=ready,
                elapsed_wall_ms=(time.monotonic() - started) * 1000)))
            if not ready:
                return DispatchOutcome(False, "execute_trajectory_unavailable")
            age_ns = self._now_ns() - command.source_timestamp_ns
            if (command.clock_epoch != self._clock_epoch or command.clock_domain != self._clock_domain
                    or not 0 <= age_ns <= self._source_timeout_ms * 1_000_000):
                return DispatchOutcome(False, "target_expired_during_readiness_wait")
        trajectory = JointTrajectory()
        trajectory.header.frame_id = self._target_frame
        trajectory.joint_names = list(SO101_GRIPPER_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [command.position_rad]
        duration_ns = int(self._gripper_duration_s * 1_000_000_000)
        point.time_from_start.sec = duration_ns // 1_000_000_000
        point.time_from_start.nanosec = duration_ns % 1_000_000_000
        trajectory.points = [point]
        if self._gripper_feedforward_effort_nm != 0.0:
            point.effort = [0.0]
            preload = JointTrajectoryPoint()
            preload.positions = [command.position_rad]
            preload.effort = [self._gripper_feedforward_effort_nm]
            preload_ns = duration_ns + int(
                self._gripper_preload_ramp_s * 1_000_000_000
            )
            preload.time_from_start.sec = preload_ns // 1_000_000_000
            preload.time_from_start.nanosec = preload_ns % 1_000_000_000
            trajectory.points.append(preload)
        goal = ExecuteTrajectory.Goal()
        goal.task_id = command.task_id
        goal.command_id = command.command_id
        goal.target_id = command.target_id
        goal.stage = command.stage
        goal.sequence_no = command.sequence_no
        goal.controller = "gripper_controller"
        goal.trajectory = trajectory
        goal.source_timestamp_ns = command.source_timestamp_ns
        goal.clock_domain = command.clock_domain
        goal.clock_epoch = command.clock_epoch
        expected_digest = trajectory_digest(
            goal.controller,
            trajectory.joint_names,
            [self._point_payload(item) for item in trajectory.points],
        )
        generation = self._next_generation()
        try:
            send_future = self._gripper_client.send_goal_async(goal)
        except Exception as error:
            return DispatchOutcome(
                False, f"execute_trajectory_send_exception:{type(error).__name__}"
            )
        slot = _ActionSlot(
            command,
            generation,
            send_future,
            expected_digest=expected_digest,
        )
        with self._state_lock:
            if self._gripper_slot is not None:
                return DispatchOutcome(False, "gripper_sequence_client_race")
            self._gripper_slot = slot
        try:
            send_future.add_done_callback(
                partial(
                    self._on_gripper_goal_response,
                    command.command_id,
                    generation,
                )
            )
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            return DispatchOutcome(
                False,
                f"execute_trajectory_goal_callback_exception:{type(error).__name__}",
            )
        self._publish_timeline(
            "gripper_command_dispatched",
            phase=self._core.phase.value,
            stage=command.stage,
            command_id=command.command_id,
            sequence_no=command.sequence_no,
            controller="gripper_controller",
            source_timestamp_ns=command.source_timestamp_ns,
            commanded_position_rad=command.position_rad,
            commanded_effort_nm=self._gripper_feedforward_effort_nm,
            preload_ramp_s=self._gripper_preload_ramp_s,
            trajectory_digest=expected_digest,
            reason="gripper_execution_dispatched",
        )
        return DispatchOutcome(True, "gripper_execution_dispatched")

    def _slot(
        self, kind: str, command_id: str, generation: int
    ) -> _ActionSlot | None:
        with self._state_lock:
            slot = self._arm_slot if kind == "arm" else self._gripper_slot
            if (
                slot is None
                or slot.command.command_id != command_id
                or slot.generation != generation
            ):
                return None
            return slot

    def _clear_slot(self, kind: str, slot: _ActionSlot) -> None:
        with self._state_lock:
            if kind == "arm" and self._arm_slot is slot:
                self._arm_slot = None
            if kind == "gripper" and self._gripper_slot is slot:
                self._gripper_slot = None

    @staticmethod
    def _is_action_terminal(status: int) -> bool:
        return int(status) in _ACTION_TERMINAL_STATUSES

    @staticmethod
    def _valid_trajectory_digest(value: object) -> bool:
        return (
            isinstance(value, str)
            and len(value) == 64
            and all(character in "0123456789abcdef" for character in value)
        )

    def _arm_slot_terminal_reason(self, slot: _ActionSlot, wrapped, result) -> str | None:
        command = slot.command
        assert isinstance(command, ArmPlanCommand)
        expected = (
            command.task_id,
            command.command_id,
            command.target_id,
            command.stage,
            command.sequence_no,
            command.source_timestamp_ns,
            command.clock_domain,
            command.clock_epoch,
        )
        observed = (
            result.task_id,
            result.command_id,
            result.target_id,
            result.stage,
            int(result.sequence_no),
            int(result.source_timestamp_ns),
            result.clock_domain,
            int(result.clock_epoch),
        )
        if observed != expected:
            return "arm_result_identity_mismatch"
        if not self._is_action_terminal(int(wrapped.status)):
            return "plan_target_wrapper_terminal_unconfirmed"

        dispatched = bool(result.trajectory_dispatched)
        gate_accepted = bool(result.gate_accepted)
        gate_terminal = bool(result.gate_terminal)
        downstream_terminal = bool(result.downstream_terminal_observed)
        if dispatched and not self._valid_trajectory_digest(
            result.trajectory_digest
        ):
            return "arm_result_digest_invalid"
        if not dispatched:
            if gate_accepted or gate_terminal or downstream_terminal:
                return "arm_result_no_dispatch_contradiction"
            return None
        if not gate_accepted:
            # A trajectory cannot have been dispatched if the gate did not
            # accept it.  Treat this as a protocol contradiction even when
            # the terminal flags happen to be false.
            return "arm_result_dispatch_without_gate_acceptance"
        if not gate_terminal or not downstream_terminal:
            return "arm_downstream_terminal_unconfirmed"
        if not self._is_action_terminal(int(result.action_goal_status)):
            return "arm_downstream_action_status_unknown"
        return None

    def _gripper_slot_terminal_reason(
        self, slot: _ActionSlot, wrapped, result
    ) -> str | None:
        command = slot.command
        assert isinstance(command, GripperExecutionCommand)
        expected = (
            command.task_id,
            command.command_id,
            command.target_id,
            command.stage,
            command.sequence_no,
            "gripper_controller",
            command.source_timestamp_ns,
            command.clock_domain,
            command.clock_epoch,
        )
        observed = (
            result.task_id,
            result.command_id,
            result.target_id,
            result.stage,
            int(result.sequence_no),
            result.controller,
            int(result.source_timestamp_ns),
            result.clock_domain,
            int(result.clock_epoch),
        )
        if observed != expected:
            return "gripper_result_identity_mismatch"
        if (
            slot.expected_digest is None
            or result.trajectory_digest != slot.expected_digest
        ):
            return "gripper_result_digest_mismatch"
        if not self._is_action_terminal(int(wrapped.status)):
            return "execute_wrapper_terminal_unconfirmed"
        if not bool(result.terminal):
            return "execute_wrapper_result_not_terminal"
        if not bool(result.accepted):
            if bool(result.downstream_terminal_observed):
                return "gripper_unaccepted_downstream_contradiction"
            return None
        if not bool(result.downstream_terminal_observed):
            return "gripper_downstream_terminal_unconfirmed"
        if not self._is_action_terminal(int(result.action_goal_status)):
            return "gripper_downstream_action_status_unknown"
        return None

    def _retain_uncertain_slot(
        self, kind: str, slot: _ActionSlot, reason: str
    ) -> None:
        with self._state_lock:
            self._uncertain_commands.add(slot.command.command_id)
        self._publish_status(
            "terminal_evidence_untrusted",
            f"{kind}:{reason}",
            slot.command.task_id,
        )
        with self._core_event_lock:
            now_ns = self._now_ns()
            decision = self._core.request_stop(
                f"{kind}_terminal_evidence_untrusted:{reason}", now_ns
            )
            self._handle_decision(decision)

    def _on_arm_feedback(
        self, command_id: str, generation: int, feedback_message
    ) -> None:
        slot = self._slot("arm", command_id, generation)
        if slot is None or slot.execution_feedback_sent:
            return
        stage = str(feedback_message.feedback.stage)
        if stage != "executing_through_gate":
            return
        slot.execution_feedback_sent = True
        command = slot.command
        assert isinstance(command, ArmPlanCommand)
        with self._core_event_lock:
            now_ns = self._now_ns()
            try:
                if self._cancel_is_requested():
                    decision = self._core.request_stop("client_cancel", now_ns)
                else:
                    decision = self._core.arm_execution_started(
                        task_id=command.task_id,
                        command_id=command.command_id,
                        stage=command.stage,
                        sequence_no=command.sequence_no,
                        health=self._health(now_ns),
                        now_ns=now_ns,
                    )
            except Exception as error:
                decision = self._core.request_stop(
                    f"arm_feedback_exception:{type(error).__name__}", now_ns
                )
            self._handle_decision(decision)

    def _on_arm_goal_response(
        self, command_id: str, generation: int, future
    ) -> None:
        slot = self._slot("arm", command_id, generation)
        if slot is None or slot.send_future is not future:
            return
        command = slot.command
        assert isinstance(command, ArmPlanCommand)
        try:
            goal_handle = future.result()
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            self._deliver_arm_failure(
                command,
                f"plan_target_goal_response_exception:{type(error).__name__}",
            )
            return
        try:
            accepted = goal_handle is not None and bool(goal_handle.accepted)
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            self._deliver_arm_failure(
                command,
                f"plan_target_goal_acceptance_exception:{type(error).__name__}",
            )
            return
        if not accepted:
            self._clear_slot("arm", slot)
            self._deliver_arm_failure(command, "plan_target_goal_rejected")
            return
        with self._state_lock:
            slot.goal_handle = goal_handle
            cancel_on_accept = slot.cancel_on_accept
        if cancel_on_accept:
            self._request_cancel("arm", slot, "cancel_on_accept")
        try:
            result_future = goal_handle.get_result_async()
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            self._deliver_arm_failure(
                command,
                f"plan_target_result_request_exception:{type(error).__name__}",
            )
            return
        try:
            with self._state_lock:
                slot.result_future = result_future
            result_future.add_done_callback(
                partial(self._on_arm_result, command_id, generation)
            )
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            self._deliver_arm_failure(
                command,
                f"plan_target_result_callback_exception:{type(error).__name__}",
            )
            return

    def _on_arm_result(self, command_id: str, generation: int, future) -> None:
        slot = self._slot("arm", command_id, generation)
        if slot is None or slot.result_future is not future:
            return
        command = slot.command
        assert isinstance(command, ArmPlanCommand)
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            self._deliver_arm_failure(
                command, f"plan_target_result_exception:{type(error).__name__}"
            )
            return
        try:
            terminal_reason = self._arm_slot_terminal_reason(slot, wrapped, result)
        except Exception as error:
            terminal_reason = f"arm_result_contract_exception:{type(error).__name__}"
        if terminal_reason is not None:
            self._retain_uncertain_slot("arm", slot, terminal_reason)
            return
        # Construct the complete immutable projection before releasing the
        # slot.  A malformed field must leave ownership/stop evidence latched;
        # otherwise the next stage could be dispatched without a correlated
        # terminal result.
        try:
            terminal = ArmStageTerminal(
                task_id=result.task_id,
                command_id=result.command_id,
                target_id=result.target_id,
                stage=result.stage,
                sequence_no=int(result.sequence_no),
                accepted=bool(result.accepted),
                success=bool(result.success),
                reason=result.reason,
                moveit_error_code=int(result.moveit_error_code),
                source_timestamp_ns=int(result.source_timestamp_ns),
                clock_domain=result.clock_domain,
                clock_epoch=int(result.clock_epoch),
                trajectory_dispatched=bool(result.trajectory_dispatched),
                trajectory_digest=result.trajectory_digest,
                gate_accepted=bool(result.gate_accepted),
                gate_terminal=bool(result.gate_terminal),
                downstream_terminal_observed=bool(
                    result.downstream_terminal_observed
                ),
                cancel_requested=bool(result.cancel_requested),
                plan_action_goal_status=int(wrapped.status),
                action_goal_status=int(result.action_goal_status),
                fjt_error_code=int(result.fjt_error_code),
                fjt_error_string=result.fjt_error_string,
            )
        except Exception as error:
            self._retain_uncertain_slot(
                "arm", slot, f"arm_result_projection_exception:{type(error).__name__}"
            )
            return
        with self._state_lock:
            slot.terminal = True
            downstream_terminal = terminal.downstream_terminal_observed
        if downstream_terminal:
            with self._state_lock:
                self._last_terminal_command_id = terminal.command_id
                self._last_trajectory_digest = terminal.trajectory_digest
        self._clear_slot("arm", slot)
        self._deliver_arm_terminal(terminal)

    def _deliver_arm_failure(self, command: ArmPlanCommand, reason: str) -> None:
        terminal = ArmStageTerminal(
            task_id=command.task_id,
            command_id=command.command_id,
            target_id=command.target_id,
            stage=command.stage,
            sequence_no=command.sequence_no,
            accepted=False,
            success=False,
            reason=reason,
            moveit_error_code=0,
            source_timestamp_ns=command.source_timestamp_ns,
            clock_domain=command.clock_domain,
            clock_epoch=command.clock_epoch,
            trajectory_dispatched=False,
            trajectory_digest="",
            gate_accepted=False,
            gate_terminal=False,
            downstream_terminal_observed=False,
            cancel_requested=False,
            plan_action_goal_status=ACTION_STATUS_ABORTED,
            action_goal_status=_UNKNOWN_STATUS,
            fjt_error_code=_UNAVAILABLE_FJT_ERROR,
            fjt_error_string="",
        )
        self._deliver_arm_terminal(terminal)

    def _deliver_arm_terminal(self, terminal: ArmStageTerminal) -> None:
        self._publish_timeline(
            "arm_command_terminal",
            phase=self._core.phase.value,
            stage=terminal.stage,
            command_id=terminal.command_id,
            sequence_no=terminal.sequence_no,
            controller="arm_controller",
            source_timestamp_ns=terminal.source_timestamp_ns,
            accepted=terminal.accepted,
            success=terminal.success,
            moveit_error_code=terminal.moveit_error_code,
            plan_action_goal_status=terminal.plan_action_goal_status,
            gate_accepted=terminal.gate_accepted,
            gate_terminal=terminal.gate_terminal,
            downstream_terminal_observed=terminal.downstream_terminal_observed,
            action_goal_status=terminal.action_goal_status,
            fjt_error_code=terminal.fjt_error_code,
            fjt_error_string=terminal.fjt_error_string,
            trajectory_digest=terminal.trajectory_digest,
            reason=terminal.reason,
        )
        if (
            terminal.stage == "descend"
            and _arm_terminal_reason(terminal) is None
            and not self._cancel_is_requested()
        ):
            policy_reason = self._enable_target_pad_contacts_after_descend()
            if policy_reason is not None:
                with self._core_event_lock:
                    now_ns = self._now_ns()
                    decision = self._core.request_stop(
                        f"contact_policy_transition_failed:{policy_reason}",
                        now_ns,
                    )
                    self._handle_decision(decision)
                return
        with self._core_event_lock:
            now_ns = self._now_ns()
            try:
                # Resolve cancellation under the same core-event lock as the
                # terminal callback.  A correlated success received after the
                # client has requested cancellation must latch SAFE_STOP and
                # never issue descend/close/lift.
                if self._cancel_is_requested():
                    decision = self._core.request_stop("client_cancel", now_ns)
                else:
                    decision = self._core.on_arm_terminal(
                        terminal, self._health(now_ns), now_ns
                    )
            except Exception as error:
                decision = self._core.request_stop(
                    f"arm_terminal_mapping_exception:{type(error).__name__}",
                    now_ns,
                )
            self._handle_decision(decision)

    def _on_gripper_goal_response(
        self, command_id: str, generation: int, future
    ) -> None:
        slot = self._slot("gripper", command_id, generation)
        if slot is None or slot.send_future is not future:
            return
        command = slot.command
        assert isinstance(command, GripperExecutionCommand)
        try:
            goal_handle = future.result()
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            self._deliver_gripper_failure(
                command,
                f"execute_goal_response_exception:{type(error).__name__}",
            )
            return
        try:
            accepted = goal_handle is not None and bool(goal_handle.accepted)
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            self._deliver_gripper_failure(
                command,
                f"execute_goal_acceptance_exception:{type(error).__name__}",
            )
            return
        if not accepted:
            self._clear_slot("gripper", slot)
            self._deliver_gripper_failure(command, "execute_goal_rejected")
            return
        with self._state_lock:
            slot.goal_handle = goal_handle
            cancel_on_accept = slot.cancel_on_accept
        if cancel_on_accept:
            self._request_cancel("gripper", slot, "cancel_on_accept")
        try:
            result_future = goal_handle.get_result_async()
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            self._deliver_gripper_failure(
                command,
                f"execute_result_request_exception:{type(error).__name__}",
            )
            return
        try:
            with self._state_lock:
                slot.result_future = result_future
            result_future.add_done_callback(
                partial(self._on_gripper_result, command_id, generation)
            )
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            self._deliver_gripper_failure(
                command,
                f"execute_result_callback_exception:{type(error).__name__}",
            )
            return

    def _on_gripper_result(
        self, command_id: str, generation: int, future
    ) -> None:
        slot = self._slot("gripper", command_id, generation)
        if slot is None or slot.result_future is not future:
            return
        command = slot.command
        assert isinstance(command, GripperExecutionCommand)
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(command.command_id)
            self._deliver_gripper_failure(
                command, f"execute_result_exception:{type(error).__name__}"
            )
            return
        try:
            terminal_reason = self._gripper_slot_terminal_reason(
                slot, wrapped, result
            )
        except Exception as error:
            terminal_reason = (
                f"gripper_result_contract_exception:{type(error).__name__}"
            )
        if terminal_reason is not None:
            self._retain_uncertain_slot("gripper", slot, terminal_reason)
            return
        try:
            terminal = GripperStageTerminal(
                task_id=result.task_id,
                command_id=result.command_id,
                target_id=result.target_id,
                stage=result.stage,
                sequence_no=int(result.sequence_no),
                controller=result.controller,
                trajectory_digest=result.trajectory_digest,
                accepted=bool(result.accepted),
                terminal=bool(result.terminal),
                downstream_terminal_observed=bool(
                    result.downstream_terminal_observed
                ),
                cancel_requested=bool(result.cancel_requested),
                gate_action_goal_status=int(wrapped.status),
                action_goal_status=int(result.action_goal_status),
                fjt_error_code=int(result.fjt_error_code),
                fjt_error_string=result.fjt_error_string,
                success=bool(result.success),
                reason=result.reason,
                source_timestamp_ns=int(result.source_timestamp_ns),
                completed_timestamp_ns=int(result.completed_timestamp_ns),
                clock_domain=result.clock_domain,
                clock_epoch=int(result.clock_epoch),
            )
        except Exception as error:
            self._retain_uncertain_slot(
                "gripper",
                slot,
                f"gripper_result_projection_exception:{type(error).__name__}",
            )
            return
        with self._state_lock:
            slot.terminal = True
            downstream_terminal = terminal.downstream_terminal_observed
        if downstream_terminal:
            with self._state_lock:
                self._last_terminal_command_id = terminal.command_id
                self._last_trajectory_digest = terminal.trajectory_digest
        self._clear_slot("gripper", slot)
        self._deliver_gripper_terminal(terminal)

    def _deliver_gripper_failure(
        self, command: GripperExecutionCommand, reason: str
    ) -> None:
        terminal = GripperStageTerminal(
            task_id=command.task_id,
            command_id=command.command_id,
            target_id=command.target_id,
            stage=command.stage,
            sequence_no=command.sequence_no,
            controller="gripper_controller",
            trajectory_digest="",
            accepted=False,
            terminal=False,
            downstream_terminal_observed=False,
            cancel_requested=False,
            gate_action_goal_status=ACTION_STATUS_ABORTED,
            action_goal_status=_UNKNOWN_STATUS,
            fjt_error_code=_UNAVAILABLE_FJT_ERROR,
            fjt_error_string="",
            success=False,
            reason=reason,
            source_timestamp_ns=command.source_timestamp_ns,
            completed_timestamp_ns=self._now_ns(),
            clock_domain=command.clock_domain,
            clock_epoch=command.clock_epoch,
        )
        self._deliver_gripper_terminal(terminal)

    def _deliver_gripper_terminal(self, terminal: GripperStageTerminal) -> None:
        self._publish_timeline(
            "gripper_command_terminal",
            phase=self._core.phase.value,
            stage=terminal.stage,
            command_id=terminal.command_id,
            sequence_no=terminal.sequence_no,
            controller=terminal.controller,
            source_timestamp_ns=terminal.source_timestamp_ns,
            accepted=terminal.accepted,
            success=terminal.success,
            gate_terminal=terminal.terminal,
            downstream_terminal_observed=terminal.downstream_terminal_observed,
            gate_action_goal_status=terminal.gate_action_goal_status,
            action_goal_status=terminal.action_goal_status,
            fjt_error_code=terminal.fjt_error_code,
            fjt_error_string=terminal.fjt_error_string,
            trajectory_digest=terminal.trajectory_digest,
            reason=terminal.reason,
        )
        with self._core_event_lock:
            now_ns = self._now_ns()
            try:
                if self._cancel_is_requested():
                    decision = self._core.request_stop("client_cancel", now_ns)
                else:
                    decision = self._core.on_gripper_terminal(
                        terminal, self._health(now_ns), now_ns
                    )
            except Exception as error:
                decision = self._core.request_stop(
                    f"gripper_terminal_mapping_exception:{type(error).__name__}",
                    now_ns,
                )
            self._handle_decision(decision)

    def _request_cancel(self, kind: str, slot: _ActionSlot, reason: str) -> None:
        with self._state_lock:
            current = self._arm_slot if kind == "arm" else self._gripper_slot
            if (
                current is not slot
                or slot.goal_handle is None
                or slot.terminal
                or slot.cancel_future is not None
            ):
                return
            goal_handle = slot.goal_handle
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(slot.command.command_id)
            self._publish_status(
                "cancel_error",
                f"{kind}:{reason}:{type(error).__name__}",
                slot.command.task_id,
            )
            return
        with self._state_lock:
            current = self._arm_slot if kind == "arm" else self._gripper_slot
            if current is not slot or slot.terminal:
                return
            slot.cancel_future = cancel_future
        try:
            cancel_future.add_done_callback(
                partial(
                    self._on_cancel_response,
                    kind,
                    slot.command.command_id,
                    slot.generation,
                )
            )
        except Exception as error:
            with self._state_lock:
                self._uncertain_commands.add(slot.command.command_id)
            self._publish_status(
                "cancel_error",
                f"{kind}:{reason}:callback:{type(error).__name__}",
                slot.command.task_id,
            )

    def _on_cancel_response(
        self, kind: str, command_id: str, generation: int, future
    ) -> None:
        slot = self._slot(kind, command_id, generation)
        if slot is None or slot.cancel_future is not future:
            return
        try:
            response = future.result()
            accepted = bool(response.goals_canceling)
        except Exception as error:
            accepted = False
            self._publish_status(
                "cancel_error",
                f"{kind}:response:{type(error).__name__}",
                slot.command.task_id,
            )
        if not accepted:
            with self._state_lock:
                self._uncertain_commands.add(command_id)
            self._publish_status(
                "cancel_rejected",
                f"{kind}:{command_id}",
                slot.command.task_id,
            )
        self._wake.set()

    def _cancel_slot(
        self, kind: str, command_id: str, reason: str
    ) -> CancelOutcome:
        with self._state_lock:
            slot = self._arm_slot if kind == "arm" else self._gripper_slot
            uncertain = command_id in self._uncertain_commands
            if slot is None:
                if uncertain:
                    return CancelOutcome(
                        CancelDisposition.REJECTED,
                        "downstream_terminal_unconfirmed",
                    )
                return CancelOutcome(
                    CancelDisposition.TERMINAL_CONFIRMED, "no_downstream_goal"
                )
            if slot.command.command_id != command_id:
                return CancelOutcome(
                    CancelDisposition.REJECTED, "command_identity_mismatch"
                )
            if slot.terminal:
                return CancelOutcome(
                    CancelDisposition.TERMINAL_CONFIRMED, "downstream_terminal"
                )
            if slot.goal_handle is None:
                slot.cancel_on_accept = True
                return CancelOutcome(
                    CancelDisposition.ACCEPTED_PENDING,
                    "cancel_waiting_for_goal_response",
                )
            cancel_pending = slot.cancel_future is None
        if cancel_pending:
            self._request_cancel(kind, slot, reason)
        return CancelOutcome(
            CancelDisposition.ACCEPTED_PENDING, "cancel_requested_stop_unconfirmed"
        )

    def _stop_kind(self, kind: str, reason: str) -> bool:
        with self._state_lock:
            slot = self._arm_slot if kind == "arm" else self._gripper_slot
            has_uncertain = bool(self._uncertain_commands)
        if slot is None:
            return not has_uncertain
        outcome = self._cancel_slot(kind, slot.command.command_id, reason)
        return outcome.terminal_confirmed

    def _stop_all(self, task_id: str, reason: str) -> StopAllOutcome:
        del task_id
        arm_stopped = self._stop_kind("arm", reason)
        gripper_stopped = self._stop_kind("gripper", reason)
        return StopAllOutcome(
            arm_stopped,
            gripper_stopped,
            "terminal_confirmed"
            if arm_stopped and gripper_stopped
            else "stop_requested_terminal_unconfirmed",
        )

    def _on_finish_completed_cycle(self, request, response):
        del request
        with self._core_event_lock:
            with self._state_lock:
                busy = (self._active_goal_handle is not None or self._reserved_task_id is not None
                        or self._arm_slot is not None or self._gripper_slot is not None)
                target, joints, scene = self._latest_target, self._joint_state, self._planning_scene_status
                active = self._active_request
                uncertain = bool(self._uncertain_commands)
            try:
                if busy or uncertain or self._input_fault:
                    raise ValueError("handoff_busy_or_faulted")
                now = self._now_ns()
                reason = self._planning_scene_policy_reason(allow=False, now_ns=now)
                if reason or scene is None or scene.carried_state != 'placed':
                    raise ValueError(reason or "handoff_requires_placed_scene")
                if target is None or joints is None or active is None:
                    raise ValueError("handoff_inputs_missing")
                if abs(joints.positions.get('gripper', float('inf'))-1.5) > .08:
                    raise ValueError("handoff_gripper_not_open")
                p = active.target.observation.point
                if any(abs(a-b) > .001 for a, b in zip(target.position, (p.x, p.y, p.z))):
                    raise ValueError("handoff_target_outside_original_scene")
                end, _ = self._observed_gripper_pose(target.source_timestamp_ns)
                if math.dist(end, target.position) < .08:
                    raise ValueError("handoff_gripper_not_clear")
                result = self._core.finish_completed_cycle(self._health(now), now)
                response.success = result.accepted
                response.message = result.reason
                if result.accepted:
                    with self._state_lock:
                        self._active_request = None
                        self._active_anchor = None
                        self._lift_observation_binding = None
                    self._publish_status('IDLE', 'completed_cycle_handoff')
            except Exception as error:
                response.success = False
                response.message = str(error)
        return response

    def _on_reset(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        # Serialize the busy/uncertain check with terminal and dispatch
        # callbacks.  Checking state first and acquiring the core lock later
        # left a reset window in which a new command could be reserved after
        # the check but before ``core.reset``.
        with self._core_event_lock:
            with self._state_lock:
                busy = (
                    self._active_goal_handle is not None
                    or self._reserved_task_id is not None
                    or self._arm_slot is not None
                    or self._gripper_slot is not None
                )
                uncertain = tuple(sorted(self._uncertain_commands))
            if busy:
                response.success = False
                response.message = (
                    "reset refused while sequence/downstream action is active"
                )
                return response
            if uncertain:
                response.success = False
                response.message = (
                    "reset refused: downstream terminal unconfirmed for "
                    + ",".join(uncertain)
                )
                return response
            now_ns = self._now_ns()
            new_epoch = self._clock_epoch + 1
            result = self._core.reset(
                now_ns,
                new_clock_epoch=new_epoch,
                external_stop_confirmed=True,
            )
            if not result.success:
                response.success = False
                response.message = result.reason
                return response
            # Keep the wrapper's epoch and cleared input snapshot atomic with
            # the core reset.  A goal callback must not reserve a task between
            # these two updates and accidentally carry the old epoch forward.
            with self._state_lock:
                self._clock_epoch = new_epoch
                self._last_clock_ns = now_ns
                self._input_fault = None
                self._permission = None
                self._interface = None
                self._joint_state = None
                self._latest_target = None
                self._target_cache.clear()
                self._measured_cube_cache.clear()
                self._planning_scene_status = None
                self._planning_scene_status_generation = 0
                self._planning_scene_status_event.clear()
                self._active_anchor = None
                self._lift_observation_binding = None
                self._active_request = None
                self._last_feedback = None
                self._last_terminal_command_id = ""
                self._last_trajectory_digest = ""
        response.success = True
        response.message = (
            f"grasp sequence reset to local clock epoch {self._clock_epoch}; "
            "reset peer nodes to the same epoch before sending a new task"
        )
        self._publish_status("reset", "waiting_for_fresh_inputs")
        return response

    def _publish_status(
        self, phase: str, reason: str, task_id: str = "<none>"
    ) -> None:
        message = String()
        message.data = (
            f"phase={phase};reason={reason};task_id={task_id};"
            f"clock_domain={self._clock_domain};clock_epoch={self._clock_epoch};"
            "sequence_completed_is_not_physics_grasp_verified"
        )
        self._status.publish(message)

    def destroy_node(self) -> bool:
        self._sequence_server.destroy()
        self._arm_client.destroy()
        self._gripper_client.destroy()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GraspSequenceNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Jazzy can surface an RCLError instead of ExternalShutdownException
        # when SIGINT invalidates the context between two wait-set cycles.
        # Suppress only that already-shutdown race; live-context failures must
        # still reach the caller and fail the process.
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        # Jazzy's MultiThreadedExecutor.shutdown() stops spinning but does not
        # join its private ThreadPoolExecutor.  Join already-submitted action
        # callbacks before destroying their ROS handles, then retrieve any
        # completed task exception so Future.__del__ cannot emit a misleading
        # "exception was never retrieved" warning during context shutdown.
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
