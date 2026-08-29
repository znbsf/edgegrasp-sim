"""Fail-closed MoveGroup plan-only adapter for an immutable target request."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, replace
import json
import math
from threading import Event, Lock
import time

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
ARM_EXECUTION_STAGES = {"approach", "descend", "lift", "diagnostic"}


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
        self._active_moveit_goal = None
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
        self, stage: str, reason: str, request_id: str = "<none>"
    ) -> None:
        message = String()
        message.data = json.dumps(
            {
                "stage": stage,
                "reason": reason,
                "request_id": request_id,
                "clock_domain": self._clock_domain,
                "clock_epoch": self._clock_epoch,
                "plan_only": True,
            },
            sort_keys=True,
        )
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
        allow = request.stage == "lift"
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
            if self._reserved_request_id is not None:
                self._publish_status(
                    "rejected", "concurrent_request", request_id
                )
                return GoalResponse.REJECT
            self._reserved_request_id = request_id
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
            receive_ns = self._last_joint_receive_ns
            positions = dict(self._joint_positions)
        if receive_ns is None or now_ns - receive_ns < 0:
            return None
        if now_ns - receive_ns > self._joint_timeout_ns:
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

    def _cancel_move_group(self, request_id: str, reason: str) -> bool:
        with self._state_lock:
            moveit_goal = self._active_moveit_goal
        if moveit_goal is None:
            return True
        try:
            future = moveit_goal.cancel_goal_async()
        except Exception as error:
            self._latch_fault(f"move_group_cancel_exception:{type(error).__name__}")
            return False
        deadline = time.monotonic() + self._cancel_response_s
        while self.context.ok() and time.monotonic() < deadline and not future.done():
            time.sleep(0.01)
        if not future.done():
            self._latch_fault("move_group_cancel_timeout")
            return False
        try:
            response = future.result()
            if len(getattr(response, "goals_canceling", ())) < 1:
                self._latch_fault("move_group_cancel_rejected")
                return False
        except Exception as error:
            self._latch_fault(f"move_group_cancel_failed:{type(error).__name__}")
            return False
        self._publish_status("cancelled_move_group", reason, request_id)
        return True

    def _late_cancel_callback(self, request_id: str, future) -> None:
        with self._state_lock:
            still_active = self._active_request_id == request_id
        if still_active:
            return
        try:
            goal = future.result()
            if goal is not None and goal.accepted:
                goal.cancel_goal_async()
        except Exception as error:
            self._latch_fault(f"late_goal_cancel_failed:{type(error).__name__}")

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

    def _cancel_gate_goal(self, goal_handle, request_id: str) -> bool:
        try:
            future = goal_handle.cancel_goal_async()
        except Exception as error:
            self._latch_fault(f"trajectory_gate_cancel_exception:{type(error).__name__}")
            return False
        deadline = time.monotonic() + self._cancel_response_s
        while self.context.ok() and time.monotonic() < deadline and not future.done():
            time.sleep(0.01)
        if not future.done():
            self._latch_fault("trajectory_gate_cancel_timeout")
            return False
        try:
            response = future.result()
            if len(getattr(response, "goals_canceling", ())) < 1:
                self._latch_fault("trajectory_gate_cancel_rejected")
                return False
        except Exception as error:
            self._latch_fault(f"trajectory_gate_cancel_failed:{type(error).__name__}")
            return False
        self._publish_status("cancelled_gate_command", "accepted", request_id)
        return True

    def _late_gate_cancel_callback(self, request_id: str, future) -> None:
        with self._state_lock:
            still_active = self._active_request_id == request_id
        if still_active:
            return
        try:
            goal = future.result()
            if goal is not None and goal.accepted:
                goal.cancel_goal_async()
        except Exception as error:
            self._latch_fault(f"late_gate_goal_cancel_failed:{type(error).__name__}")

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
    ) -> _GateOutcome:
        request_id = self._request_id(request)
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
        try:
            send_future = self._trajectory_gate.send_goal_async(gate_goal)
        except Exception as error:
            self._latch_fault(f"trajectory_gate_send_exception:{type(error).__name__}")
            return _GateOutcome(
                trajectory_digest=digest,
                reason="trajectory_gate_send_exception",
            )
        send_reason = self._poll_future(
            send_future,
            time.monotonic() + self._goal_response_s,
            goal_handle,
            request,
            timeout_reason="trajectory_gate_goal_response_timeout",
        )
        if send_reason is not None:
            send_future.add_done_callback(
                lambda future: self._late_gate_cancel_callback(request_id, future)
            )
            return _GateOutcome(
                trajectory_digest=digest,
                reason=send_reason,
            )
        try:
            gate_handle = send_future.result()
        except Exception as error:
            self._latch_fault(
                f"trajectory_gate_goal_response_exception:{type(error).__name__}"
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
        try:
            result_future = gate_handle.get_result_async()
        except Exception as error:
            self._latch_fault(
                f"trajectory_gate_result_request_exception:{type(error).__name__}"
            )
            self._cancel_gate_goal(gate_handle, request_id)
            return _GateOutcome(
                dispatched=True,
                trajectory_digest=digest,
                reason="trajectory_gate_result_request_exception",
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
        if result_reason is not None:
            self._cancel_gate_goal(gate_handle, request_id)
            terminal_deadline = time.monotonic() + self._gate_result_margin_s
            while (
                self.context.ok()
                and time.monotonic() < terminal_deadline
                and not result_future.done()
            ):
                time.sleep(0.01)
            if not result_future.done():
                self._latch_fault("trajectory_gate_terminal_unconfirmed")
                return _GateOutcome(
                    dispatched=True,
                    trajectory_digest=digest,
                    cancel_requested=True,
                    reason=f"{result_reason}:gate_terminal_unconfirmed",
                )
        try:
            wrapped = result_future.result()
        except Exception as error:
            self._latch_fault(
                f"trajectory_gate_result_exception:{type(error).__name__}"
            )
            return _GateOutcome(
                dispatched=True,
                trajectory_digest=digest,
                reason="trajectory_gate_result_exception",
            )
        outcome = self._gate_outcome(request, digest, wrapped)
        if result_reason is not None and outcome.success:
            self._latch_fault("trajectory_gate_cancel_success_race")
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

    def _finish(self, request_id: str) -> None:
        with self._state_lock:
            if self._active_request_id == request_id:
                self._active_request_id = None
                self._active_moveit_goal = None
            if self._reserved_request_id == request_id:
                self._reserved_request_id = None

    def _execute(self, goal_handle) -> PlanTarget.Result:
        request = goal_handle.request
        request_id = self._request_id(request)
        requested_at = self.get_clock().now().to_msg()
        with self._state_lock:
            self._active_request_id = request_id
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
                self._latch_fault(f"move_group_send_exception:{type(error).__name__}")
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="move_group_send_exception",
                )
            send_reason = self._poll_future(
                send_future,
                time.monotonic() + self._goal_response_s,
                goal_handle,
                request,
            )
            if send_reason is not None:
                send_future.add_done_callback(
                    lambda future: self._late_cancel_callback(request_id, future)
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
                self._latch_fault(
                    f"move_group_goal_response_exception:{type(error).__name__}"
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
            with self._state_lock:
                self._active_moveit_goal = moveit_goal_handle

            try:
                result_future = moveit_goal_handle.get_result_async()
            except Exception as error:
                self._latch_fault(
                    f"move_group_result_request_exception:{type(error).__name__}"
                )
                self._cancel_move_group(request_id, "result_request_exception")
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="move_group_result_request_exception",
                )
            result_reason = self._poll_future(
                result_future,
                time.monotonic()
                + request.planning_timeout_s
                + self._result_margin_s,
                goal_handle,
                request,
            )
            if result_reason is not None:
                self._cancel_move_group(request_id, result_reason)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(
                    request, requested_at, success=False, reason=result_reason
                )
            try:
                wrapped = result_future.result()
                moveit_result = wrapped.result
            except Exception as error:
                self._latch_fault(
                    f"move_group_result_exception:{type(error).__name__}"
                )
                goal_handle.abort()
                return self._result(
                    request,
                    requested_at,
                    success=False,
                    reason="move_group_result_exception",
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
                if self._active_request_id != request_id:
                    goal_handle.abort()
                    return self._result(
                        request,
                        requested_at,
                        success=False,
                        reason="late_result_ignored",
                        moveit_error_code=error_code,
                    )
                if request_id in self._dispatched_command_ids:
                    self._fault_latched = "duplicate_dispatch_prevented"
                    goal_handle.abort()
                    return self._result(
                        request,
                        requested_at,
                        success=False,
                        reason="duplicate_dispatch_prevented",
                        moveit_error_code=error_code,
                    )
                self._dispatched_command_ids.add(request_id)
            self._feedback(goal_handle, "executing_through_gate", request_id)
            gate = self._dispatch_to_gate(
                goal_handle,
                request,
                decision.joint_trajectory,
            )
            if not gate.success:
                if goal_handle.is_cancel_requested and gate.terminal:
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
            self._publish_status("gate_terminal", "succeeded", request_id)
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
            self._finish(request_id)

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
