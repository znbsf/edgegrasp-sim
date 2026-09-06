"""Read-only ROS wrapper for contact, lift and retention evidence.

The node owns no publisher or client capable of commanding motion.  It binds a
single immutable observation request to dedicated target-cube pose/contact
topics and to a correlated GraspSequence terminal event.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math
from pathlib import Path
from threading import Event, RLock
import time

from ament_index_python.packages import get_package_share_directory
from edgegrasp.config import ROS_SIM_CLOCK_DOMAIN, validate_ros_clock_domain
from edgegrasp.grasp_evidence import (
    CollisionPair,
    ContactSample,
    CubePoseSample,
    EvidencePhase,
    GraspPhysicsEvidenceController,
    GripperEffortSample,
    PhysicsEvidenceResult,
    PhysicsEvidenceTask,
    SequenceCompletion,
)
from edgegrasp.scene import load_scene_contract
from edgegrasp.trajectory_identity import make_trajectory_command_id
from edgegrasp_interfaces.action import GraspPhysicsEvidence
from edgegrasp_interfaces.msg import GraspSequenceTerminal
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass(frozen=True, order=True)
class _PendingObservation:
    """Same-domain evidence waiting for the local ROS clock to catch up."""

    source_timestamp_ns: int
    sequence_no: int
    kind: str = field(compare=False)
    value: CubePoseSample | ContactSample | GripperEffortSample | SequenceCompletion = field(
        compare=False
    )


class GraspPhysicsObserver(Node):
    """One-in-flight, fail-closed physical-evidence observer."""

    def __init__(self, **node_kwargs) -> None:
        super().__init__("edgegrasp_grasp_physics_observer", **node_kwargs)
        default_scene = str(
            Path(get_package_share_directory("edgegrasp_ros"))
            / "config"
            / "scene.json"
        )
        self.declare_parameter(
            "evidence_action", "/edgegrasp/grasp_physics_evidence"
        )
        self.declare_parameter("cube_pose_topic", "/edgegrasp/target_cube_pose")
        self.declare_parameter(
            "cube_contact_topic", "/edgegrasp/target_cube_contacts"
        )
        self.declare_parameter(
            "sequence_terminal_topic", "/edgegrasp/grasp_sequence_terminal"
        )
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("gripper_joint_name", "gripper")
        self.declare_parameter("scene_config", default_scene)
        self.declare_parameter("clock_domain", ROS_SIM_CLOCK_DOMAIN)
        self.declare_parameter("clock_epoch", 0)
        self.declare_parameter(
            "gripper_collision_tokens",
            [
                "fixed_finger_pad",
                "moving_finger_pad",
            ],
        )
        self.declare_parameter("minimum_baseline_samples", 5)
        self.declare_parameter("minimum_lift_m", 0.02)
        self.declare_parameter("minimum_clearance_m", 0.01)
        self.declare_parameter("minimum_retention_s", 0.5)
        self.declare_parameter("maximum_xy_drift_m", 0.01)
        self.declare_parameter("maximum_freshness_ms", 200.0)
        self.declare_parameter("maximum_observation_s", 30.0)
        self.declare_parameter("observation_wall_factor", 2.0)
        self._observation_wall_factor = float(self.get_parameter("observation_wall_factor").value)
        if self._observation_wall_factor not in (2.0, 6.0):
            raise ValueError("observation_wall_factor must be 2 or 6")
        self.declare_parameter("maximum_pending_observations", 512)
        self.declare_parameter("loop_rate_hz", 100.0)

        self._clock_domain = str(self.get_parameter("clock_domain").value)
        self._clock_epoch = int(self.get_parameter("clock_epoch").value)
        self._use_sim_time = bool(self.get_parameter("use_sim_time").value)
        validate_ros_clock_domain(self._use_sim_time, self._clock_domain)
        if self._clock_epoch < 0:
            raise ValueError("clock_epoch must be non-negative")

        scene_path = Path(str(self.get_parameter("scene_config").value))
        self._scene = load_scene_contract(scene_path)
        if self._scene.clock_domain != self._clock_domain:
            raise ValueError("scene and observer clock domains differ")
        self._cube = next(
            item for item in self._scene.objects if item.object_id == "target_cube"
        )
        self._table = next(
            item for item in self._scene.objects if item.object_id == "table"
        )
        if self._cube.pose_topic is None or self._cube.contact_topic is None:
            raise ValueError("target_cube pose/contact evidence topics are required")
        self._pose_frame = self._scene.name
        self._gripper_tokens = tuple(
            str(value)
            for value in self.get_parameter("gripper_collision_tokens").value
            if str(value).strip()
        )
        if not self._gripper_tokens:
            raise ValueError("gripper_collision_tokens must not be empty")
        self._gripper_joint_name = str(
            self.get_parameter("gripper_joint_name").value
        ).strip()
        if not self._gripper_joint_name:
            raise ValueError("gripper_joint_name must not be empty")

        self._minimum_baseline_samples = self._positive_int_parameter(
            "minimum_baseline_samples"
        )
        self._minimum_lift_m = self._positive_parameter("minimum_lift_m")
        self._minimum_clearance_m = self._nonnegative_parameter(
            "minimum_clearance_m"
        )
        self._minimum_retention_s = self._positive_parameter(
            "minimum_retention_s"
        )
        self._maximum_xy_drift_m = self._nonnegative_parameter(
            "maximum_xy_drift_m"
        )
        self._maximum_freshness_ms = self._positive_parameter(
            "maximum_freshness_ms"
        )
        self._maximum_observation_s = self._positive_parameter(
            "maximum_observation_s"
        )
        self._maximum_pending_observations = self._positive_int_parameter(
            "maximum_pending_observations"
        )
        self._loop_period_s = 1.0 / self._positive_parameter("loop_rate_hz")

        self._lock = RLock()
        self._event = Event()
        self._callback_group = ReentrantCallbackGroup()
        self._pose_group = MutuallyExclusiveCallbackGroup()
        self._contact_group = MutuallyExclusiveCallbackGroup()
        self._effort_group = MutuallyExclusiveCallbackGroup()
        self._terminal_group = MutuallyExclusiveCallbackGroup()
        self._core: GraspPhysicsEvidenceController | None = None
        self._active_goal_handle = None
        self._reserved_task_id: str | None = None
        self._clock_fault: str | None = None
        self._last_clock_ns: int | None = None
        self._last_feedback: tuple[str, str, int, int] | None = None
        self._active_freshness_ns: int | None = None
        self._pending_sequence_no = 0
        self._pending_observations: list[_PendingObservation] = []

        self._status = self.create_publisher(
            String, "/edgegrasp/grasp_physics_status", 10
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("cube_pose_topic").value),
            self._on_pose,
            10,
            callback_group=self._pose_group,
        )
        self.create_subscription(
            Contacts,
            str(self.get_parameter("cube_contact_topic").value),
            self._on_contacts,
            10,
            callback_group=self._contact_group,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_states_topic").value),
            self._on_joint_state,
            10,
            callback_group=self._effort_group,
        )
        self.create_subscription(
            GraspSequenceTerminal,
            str(self.get_parameter("sequence_terminal_topic").value),
            self._on_sequence_terminal,
            10,
            callback_group=self._terminal_group,
        )
        self.create_service(
            Trigger,
            "/edgegrasp/reset_grasp_physics_epoch",
            self._on_reset,
            callback_group=self._callback_group,
        )
        self._server = ActionServer(
            self,
            GraspPhysicsEvidence,
            str(self.get_parameter("evidence_action").value),
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

    def _nonnegative_parameter(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
        return value

    def _positive_int_parameter(self, name: str) -> int:
        value = self.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f"{name} must be a positive integer")
        return value

    @staticmethod
    def _stamp_ns(stamp) -> int:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
        if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
            raise ValueError("invalid ROS timestamp")
        return sec * 1_000_000_000 + nanosec

    @staticmethod
    def _duration_ns(duration) -> int:
        sec = int(duration.sec)
        nanosec = int(duration.nanosec)
        if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
            raise ValueError("invalid ROS duration")
        value = sec * 1_000_000_000 + nanosec
        if value <= 0:
            raise ValueError("ROS duration must be positive")
        return value

    @staticmethod
    def _assign_stamp(stamp, value_ns: int | None) -> None:
        value = 0 if value_ns is None else int(value_ns)
        stamp.sec = value // 1_000_000_000
        stamp.nanosec = value % 1_000_000_000

    def _now_ns(self) -> int:
        with self._lock:
            # Read and compare the ROS clock under the same lock.  This node uses
            # a MultiThreadedExecutor; reading before acquiring the lock lets an
            # older sample from one callback be committed after a newer sample
            # from another callback and falsely looks like a /clock rollback.
            now_ns = self.get_clock().now().nanoseconds
            if self._last_clock_ns is not None and now_ns < self._last_clock_ns:
                self._clock_fault = (
                    f"ros_clock_rollback:{now_ns}<{self._last_clock_ns}"
                )
                if self._core is not None:
                    self._core.fail_closed("ros_clock_rollback", now_ns)
            self._last_clock_ns = now_ns
        return now_ns

    def _goal_reason(self, request: GraspPhysicsEvidence.Goal) -> str | None:
        with self._lock:
            if self._reserved_task_id is not None or self._active_goal_handle is not None:
                return "observer_busy"
            if self._clock_fault is not None:
                return self._clock_fault
        if not request.task_id.strip() or not request.target_id.strip():
            return "task_or_target_id_empty"
        if request.expected_lift_command_id != make_trajectory_command_id(
            request.task_id, "lift", 3
        ):
            return "expected_lift_command_id_mismatch"
        if request.cube_model_name != self._cube.gazebo_name:
            return "cube_model_name_mismatch"
        if request.pose_frame_id != self._pose_frame:
            return "pose_frame_id_mismatch"
        if request.scene_digest != self._scene.digest:
            return "scene_digest_mismatch"
        if request.clock_domain != self._clock_domain:
            return "clock_domain_mismatch"
        if int(request.clock_epoch) != self._clock_epoch:
            return "clock_epoch_mismatch"
        try:
            started_ns = self._stamp_ns(request.started_at)
            retention_ns = self._duration_ns(request.retention_duration)
            freshness_ns = self._duration_ns(request.freshness_timeout)
            observation_ns = self._duration_ns(request.observation_timeout)
        except ValueError as error:
            return str(error).replace(" ", "_")
        now_ns = self._now_ns()
        if started_ns > now_ns:
            return "started_at_in_future"
        if now_ns - started_ns > freshness_ns:
            return "started_at_stale"
        target_source_ns = int(request.target_source_timestamp_ns)
        if target_source_ns > started_ns:
            return "target_source_after_started_at"
        if started_ns - target_source_ns > freshness_ns:
            return "target_source_stale"
        if int(request.baseline_sample_count) < self._minimum_baseline_samples:
            return "baseline_sample_count_below_policy"
        values = (
            request.min_lift_m,
            request.min_clearance_m,
            request.max_xy_drift_m,
        )
        if any(not math.isfinite(value) for value in values):
            return "nonfinite_threshold"
        if request.min_lift_m < self._minimum_lift_m:
            return "min_lift_below_policy"
        if request.min_clearance_m < self._minimum_clearance_m:
            return "min_clearance_below_policy"
        if retention_ns < int(self._minimum_retention_s * 1_000_000_000):
            return "retention_below_policy"
        if request.max_xy_drift_m > self._maximum_xy_drift_m:
            return "xy_drift_above_policy"
        if freshness_ns > int(self._maximum_freshness_ms * 1_000_000):
            return "freshness_above_policy"
        if observation_ns > int(self._maximum_observation_s * 1_000_000_000):
            return "observation_timeout_above_policy"
        if observation_ns <= retention_ns:
            return "observation_timeout_must_exceed_retention"
        return None

    def _on_goal(self, request: GraspPhysicsEvidence.Goal) -> GoalResponse:
        try:
            reason = self._goal_reason(request)
        except Exception as error:
            reason = f"goal_validation_exception:{type(error).__name__}"
        if reason is not None:
            self._publish_status("REJECTED", reason, request.task_id)
            return GoalResponse.REJECT
        with self._lock:
            if self._reserved_task_id is not None:
                return GoalResponse.REJECT
            self._reserved_task_id = request.task_id
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        with self._lock:
            if goal_handle is not self._active_goal_handle:
                return CancelResponse.REJECT
        self._event.set()
        return CancelResponse.ACCEPT

    def _task(self, request: GraspPhysicsEvidence.Goal) -> PhysicsEvidenceTask:
        return PhysicsEvidenceTask(
            task_id=request.task_id,
            target_id=request.target_id,
            target_source_timestamp_ns=int(request.target_source_timestamp_ns),
            expected_lift_command_id=request.expected_lift_command_id,
            scene_digest=request.scene_digest,
            cube_model_name=self._cube.gazebo_name,
            cube_collision_token=(
                f"{self._cube.gazebo_name}::{self._cube.object_id}_link::collision"
            ),
            gripper_collision_tokens=self._gripper_tokens,
            table_collision_tokens=(
                f"{self._table.gazebo_name}::{self._table.object_id}_link::collision",
            ),
            started_at_ns=self._stamp_ns(request.started_at),
            world_frame=self._pose_frame,
            gripper_joint_name=self._gripper_joint_name,
            clock_domain=request.clock_domain,
            clock_epoch=int(request.clock_epoch),
            table_top_z_m=(
                self._table.pose_world.position_m[2] + self._table.size_m[2] / 2.0
            ),
            cube_height_m=self._cube.size_m[2],
            baseline_sample_count=int(request.baseline_sample_count),
            min_lift_m=float(request.min_lift_m),
            min_clearance_m=float(request.min_clearance_m),
            retention_ns=self._duration_ns(request.retention_duration),
            max_xy_drift_m=float(request.max_xy_drift_m),
            freshness_timeout_ns=self._duration_ns(request.freshness_timeout),
        )

    def _execute(self, goal_handle) -> GraspPhysicsEvidence.Result:
        request = goal_handle.request
        accepted = False
        try:
            with self._lock:
                if self._reserved_task_id != request.task_id:
                    goal_handle.abort()
                    return self._result(False)
                self._active_goal_handle = goal_handle
                self._core = GraspPhysicsEvidenceController()
                self._last_feedback = None
                self._active_freshness_ns = self._duration_ns(
                    request.freshness_timeout
                )
                self._pending_sequence_no = 0
                self._pending_observations.clear()
            with self._lock:
                now_ns = self._now_ns()
                decision = self._core.start(self._task(request), now_ns)
            accepted = decision.phase is not EvidencePhase.FAULT
            self._publish_feedback(decision.reason)
            observation_ns = self._duration_ns(request.observation_timeout)
            sim_deadline_ns = now_ns + observation_ns
            wall_deadline = time.monotonic() + max(
                5.0, observation_ns / 1_000_000_000.0 * self._observation_wall_factor
            )
            while True:
                with self._lock:
                    assert self._core is not None
                    phase = self._core.phase
                if phase in (EvidencePhase.VERIFIED, EvidencePhase.FAULT):
                    break
                self._drain_pending_observations()
                with self._lock:
                    # Re-sample under the same lock used for the core call.
                    # A callback can otherwise advance the core clock after a
                    # local value is read but before this thread calls tick().
                    now_ns = self._now_ns()
                    if goal_handle.is_cancel_requested:
                        decision = self._core.fail_closed(
                            "observer_cancelled", now_ns
                        )
                    elif time.monotonic() >= wall_deadline:
                        decision = self._core.fail_closed(
                            "observation_wall_timeout", now_ns
                        )
                    elif now_ns >= sim_deadline_ns:
                        decision = self._core.fail_closed(
                            "observation_sim_timeout", now_ns
                        )
                    else:
                        decision = self._core.tick(now_ns)
                self._publish_feedback(decision.reason)
                self._event.wait(self._loop_period_s)
                self._event.clear()

            result = self._result(accepted)
            if result.physics_grasp_verified:
                goal_handle.succeed()
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except BaseException as error:
            self.get_logger().error(
                f"physics observer wrapper escaped: {type(error).__name__}:{error}"
            )
            try:
                with self._lock:
                    if self._core is not None:
                        self._core.fail_closed(
                            f"observer_exception:{type(error).__name__}",
                            self._now_ns(),
                        )
                goal_handle.abort()
            except BaseException:
                pass
            return self._result(accepted)
        finally:
            with self._lock:
                self._active_goal_handle = None
                self._reserved_task_id = None
                self._active_freshness_ns = None
                self._pending_observations.clear()

    def _on_pose(self, message: PoseStamped) -> None:
        with self._lock:
            core = self._core
            active = self._active_goal_handle is not None
        if core is None or not active:
            return
        try:
            now_ns = self._now_ns()
            sample = CubePoseSample(
                model_name=self._cube.gazebo_name,
                frame_id=message.header.frame_id,
                x_m=float(message.pose.position.x),
                y_m=float(message.pose.position.y),
                z_m=float(message.pose.position.z),
                source_timestamp_ns=self._stamp_ns(message.header.stamp),
                received_at_ns=now_ns,
                clock_domain=self._clock_domain,
                clock_epoch=self._clock_epoch,
            )
            self._queue_observation("pose", sample)
        except Exception as error:
            self._input_fault(f"pose_input_invalid:{type(error).__name__}")

    def _on_contacts(self, message: Contacts) -> None:
        with self._lock:
            core = self._core
            active = self._active_goal_handle is not None
        if core is None or not active:
            return
        try:
            now_ns = self._now_ns()
            pairs = tuple(self._collision_pair(row) for row in message.contacts)
            sample = ContactSample(
                pairs=pairs,
                source_timestamp_ns=self._stamp_ns(message.header.stamp),
                received_at_ns=now_ns,
                clock_domain=self._clock_domain,
                clock_epoch=self._clock_epoch,
            )
            self._queue_observation("contact", sample)
        except Exception as error:
            self._input_fault(f"contact_input_invalid:{type(error).__name__}")

    @staticmethod
    def _finite_vector(vector, field: str) -> tuple[float, float, float]:
        values = (float(vector.x), float(vector.y), float(vector.z))
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{field} must be finite")
        return values

    @classmethod
    def _collision_pair(cls, row) -> CollisionPair:
        depths = tuple(float(value) for value in row.depths)
        if any(not math.isfinite(value) or value < 0.0 for value in depths):
            raise ValueError("contact depths must be finite and non-negative")

        normals = tuple(
            cls._finite_vector(value, "contact normal") for value in row.normals
        )
        body_forces: list[tuple[tuple[float, float, float], tuple[float, float, float]]] = []
        force_magnitudes: list[float] = []
        for wrench in row.wrenches:
            first = cls._finite_vector(
                wrench.body_1_wrench.force, "body_1 contact force"
            )
            second = cls._finite_vector(
                wrench.body_2_wrench.force, "body_2 contact force"
            )
            body_forces.append((first, second))
            force_magnitudes.extend(
                math.sqrt(sum(component * component for component in force))
                for force in (first, second)
            )

        normal_forces: list[float] = []
        for normal, (first, second) in zip(normals, body_forces):
            norm = math.sqrt(sum(component * component for component in normal))
            if norm <= 0.0:
                raise ValueError("contact normal must have non-zero magnitude")
            unit = tuple(component / norm for component in normal)
            normal_forces.extend(
                abs(sum(force[index] * unit[index] for index in range(3)))
                for force in (first, second)
            )

        return CollisionPair(
            collision1=str(row.collision1.name),
            collision2=str(row.collision2.name),
            max_penetration_depth_m=max(depths) if depths else None,
            max_force_magnitude_n=max(force_magnitudes) if force_magnitudes else None,
            max_abs_normal_force_n=max(normal_forces) if normal_forces else None,
        )

    def _on_joint_state(self, message: JointState) -> None:
        with self._lock:
            core = self._core
            active = self._active_goal_handle is not None
        if core is None or not active:
            return
        try:
            names = list(message.name)
            occurrences = names.count(self._gripper_joint_name)
            if occurrences == 0:
                return
            if occurrences != 1:
                raise ValueError("gripper joint name must appear exactly once")
            index = names.index(self._gripper_joint_name)
            if index >= len(message.effort):
                # Effort is optional in JointState. Its absence is explicitly
                # represented as unavailable, never synthesized as zero.
                return
            now_ns = self._now_ns()
            sample = GripperEffortSample(
                joint_name=self._gripper_joint_name,
                effort=float(message.effort[index]),
                source_timestamp_ns=self._stamp_ns(message.header.stamp),
                received_at_ns=now_ns,
                clock_domain=self._clock_domain,
                clock_epoch=self._clock_epoch,
            )
            self._queue_observation("effort", sample)
        except Exception as error:
            self._input_fault(f"gripper_effort_input_invalid:{type(error).__name__}")

    def _on_sequence_terminal(self, message: GraspSequenceTerminal) -> None:
        with self._lock:
            core = self._core
            active = self._active_goal_handle is not None
        if core is None or not active:
            return
        try:
            now_ns = self._now_ns()
            completion = SequenceCompletion(
                task_id=message.task_id,
                target_id=message.target_id,
                target_source_timestamp_ns=int(message.source_timestamp_ns),
                lift_command_id=message.lift_command_id,
                sequence_completed=bool(message.sequence_completed),
                source_timestamp_ns=self._stamp_ns(message.terminal_stamp),
                received_at_ns=now_ns,
                clock_domain=message.clock_domain,
                clock_epoch=int(message.clock_epoch),
            )
            self._queue_observation("sequence", completion)
        except Exception as error:
            self._input_fault(f"sequence_terminal_invalid:{type(error).__name__}")

    def _queue_observation(
        self,
        kind: str,
        value: CubePoseSample | ContactSample | GripperEffortSample | SequenceCompletion,
    ) -> None:
        """Queue evidence until its same-domain ROS timestamp is observable.

        Gazebo publishes pose/contact and ``/clock`` on separate transports, so
        a valid evidence message can reach this process a few milliseconds
        before the corresponding clock update.  The core remains strict:
        samples are not submitted with a future timestamp.  This wrapper only
        buffers them within the configured freshness bound and fails closed on
        excessive skew or queue growth.
        """

        decision = None
        source_ns = value.source_timestamp_ns
        with self._lock:
            # Callback-local timestamps can become stale while another callback
            # advances the strict core.  Re-sample only after owning the core
            # lock so receive-time ordering matches evaluation ordering.
            now_ns = self._now_ns()
            core = self._core
            if core is None or self._active_goal_handle is None:
                return
            freshness_ns = self._active_freshness_ns
            if freshness_ns is None:
                decision = core.fail_closed("observer_freshness_policy_missing", now_ns)
            elif source_ns > now_ns and source_ns - now_ns > freshness_ns:
                decision = core.fail_closed("source_timestamp_in_future", now_ns)
            elif len(self._pending_observations) >= self._maximum_pending_observations:
                decision = core.fail_closed("pending_observation_overflow", now_ns)
            else:
                self._pending_sequence_no += 1
                self._pending_observations.append(
                    _PendingObservation(
                        source_timestamp_ns=source_ns,
                        sequence_no=self._pending_sequence_no,
                        kind=kind,
                        value=value,
                    )
                )
                self._pending_observations.sort()
        if decision is not None:
            self._publish_feedback(decision.reason)
        else:
            self._drain_pending_observations()
        self._event.set()

    def _drain_pending_observations(self) -> None:
        """Submit timestamp-ready observations to the strict core in order."""

        while True:
            with self._lock:
                now_ns = self._now_ns()
                core = self._core
                if (
                    core is None
                    or self._active_goal_handle is None
                    or core.phase in (EvidencePhase.VERIFIED, EvidencePhase.FAULT)
                    or not self._pending_observations
                    or self._pending_observations[0].source_timestamp_ns > now_ns
                ):
                    return
                pending = self._pending_observations.pop(0)
                received = replace(pending.value, received_at_ns=now_ns)
                if pending.kind == "pose":
                    assert isinstance(received, CubePoseSample)
                    decision = core.observe_pose(received)
                elif pending.kind == "contact":
                    assert isinstance(received, ContactSample)
                    decision = core.observe_contact(received)
                elif pending.kind == "effort":
                    assert isinstance(received, GripperEffortSample)
                    decision = core.observe_gripper_effort(received)
                else:
                    assert pending.kind == "sequence"
                    assert isinstance(received, SequenceCompletion)
                    decision = core.observe_sequence_completion(received)
            self._publish_feedback(decision.reason)

    def _input_fault(self, reason: str) -> None:
        try:
            with self._lock:
                now_ns = self._now_ns()
                if self._core is not None:
                    decision = self._core.fail_closed(reason, now_ns)
                else:
                    return
            self._publish_feedback(decision.reason)
        finally:
            self._event.set()

    def _publish_feedback(self, reason: str) -> None:
        with self._lock:
            core = self._core
            goal_handle = self._active_goal_handle
            result = None if core is None else core.result
            if goal_handle is None or result is None:
                return
            current = (
                result.phase.value,
                reason,
                # Pose evidence may arrive at simulator rate.  Feedback is
                # diagnostic, so bucket pose counts to avoid stdout/DDS load
                # starving the motion result path during a live trial.
                result.pose_sample_count // 20,
                # Once contact is observed the boolean transition is the
                # meaningful feedback event.  Publishing every 100 Hz contact
                # sample can flood DDS/stdout and slow the very simulation we
                # are measuring.
                1 if result.gripper_contact_count > 0 else 0,
            )
            if current == self._last_feedback:
                return
            self._last_feedback = current
        feedback = GraspPhysicsEvidence.Feedback()
        feedback.task_id = result.task_id
        feedback.phase = result.phase.value
        feedback.reason = reason
        feedback.sequence_completed = result.sequence_completed
        feedback.gripper_contact_observed = result.gripper_contact_count > 0
        feedback.current_pose_available = result.final_z_m is not None
        feedback.current_z_m = 0.0 if result.final_z_m is None else result.final_z_m
        feedback.peak_z_m = 0.0 if result.peak_z_m is None else result.peak_z_m
        elapsed_ns = 0
        if result.retention_started_at_ns is not None:
            elapsed_ns = max(0, self._now_ns() - result.retention_started_at_ns)
        self._assign_stamp(feedback.retention_elapsed, elapsed_ns)
        try:
            goal_handle.publish_feedback(feedback)
        except Exception as error:
            self.get_logger().error(
                f"physics feedback failed: {type(error).__name__}:{error}"
            )
        self._publish_status(result.phase.value, reason, result.task_id)
        self._event.set()

    def _result(self, accepted: bool) -> GraspPhysicsEvidence.Result:
        message = GraspPhysicsEvidence.Result()
        with self._lock:
            core_result = None if self._core is None else self._core.result
            task_id = self._reserved_task_id or ""
        if core_result is None:
            message.task_id = task_id
            message.accepted = accepted
            message.terminal_phase = EvidencePhase.FAULT.value
            message.reason = "observer_terminated_without_core_result"
            message.clock_domain = self._clock_domain
            message.clock_epoch = self._clock_epoch
            return message
        self._fill_result(message, core_result, accepted)
        return message

    def _fill_result(
        self,
        message: GraspPhysicsEvidence.Result,
        result: PhysicsEvidenceResult,
        accepted: bool,
    ) -> None:
        message.task_id = result.task_id
        message.target_id = result.target_id
        message.target_source_timestamp_ns = result.target_source_timestamp_ns
        message.cube_model_name = result.cube_model_name
        message.pose_frame_id = result.pose_frame_id
        message.expected_lift_command_id = result.expected_lift_command_id
        message.scene_digest = result.scene_digest
        message.accepted = accepted
        message.sequence_completed = result.sequence_completed
        message.gripper_contact_observed = result.gripper_contact_observed
        message.lift_observed = result.lift_observed
        message.retention_observed = result.retention_observed
        message.physics_grasp_verified = result.physics_grasp_verified
        message.terminal_phase = result.phase.value
        message.reason = result.reason
        message.baseline_available = result.baseline_z_m is not None
        message.baseline_z_m = 0.0 if result.baseline_z_m is None else result.baseline_z_m
        message.final_pose_available = result.final_z_m is not None
        message.final_z_m = 0.0 if result.final_z_m is None else result.final_z_m
        message.peak_available = result.peak_z_m is not None
        message.peak_z_m = 0.0 if result.peak_z_m is None else result.peak_z_m
        message.retention_clearance_available = (
            result.retention_min_clearance_m is not None
        )
        message.retention_min_clearance_m = (
            0.0
            if result.retention_min_clearance_m is None
            else result.retention_min_clearance_m
        )
        message.max_xy_drift_m = result.max_xy_drift_m
        message.pose_sample_count = result.pose_sample_count
        message.gripper_contact_count = result.gripper_contact_count
        message.gripper_contact_tokens = list(result.gripper_contact_tokens)
        message.gripper_contact_counts_by_token = list(
            result.gripper_contact_counts_by_token
        )
        message.gripper_first_contact_stamp_available_by_token = [
            value is not None
            for value in result.gripper_first_contact_source_timestamp_ns_by_token
        ]
        message.gripper_first_contact_source_timestamp_ns_by_token = [
            0 if value is None else value
            for value in result.gripper_first_contact_source_timestamp_ns_by_token
        ]
        message.gripper_last_contact_stamp_available_by_token = [
            value is not None
            for value in result.gripper_last_contact_source_timestamp_ns_by_token
        ]
        message.gripper_last_contact_source_timestamp_ns_by_token = [
            0 if value is None else value
            for value in result.gripper_last_contact_source_timestamp_ns_by_token
        ]
        message.gripper_penetration_depth_available_by_token = [
            value is not None
            for value in result.gripper_max_penetration_depth_m_by_token
        ]
        message.gripper_max_penetration_depth_m_by_token = [
            0.0 if value is None else value
            for value in result.gripper_max_penetration_depth_m_by_token
        ]
        message.gripper_force_magnitude_available_by_token = [
            value is not None for value in result.gripper_max_force_magnitude_n_by_token
        ]
        message.gripper_max_force_magnitude_n_by_token = [
            0.0 if value is None else value
            for value in result.gripper_max_force_magnitude_n_by_token
        ]
        message.gripper_normal_force_available_by_token = [
            value is not None
            for value in result.gripper_max_abs_normal_force_n_by_token
        ]
        message.gripper_max_abs_normal_force_n_by_token = [
            0.0 if value is None else value
            for value in result.gripper_max_abs_normal_force_n_by_token
        ]
        message.all_gripper_tokens_observed = result.all_gripper_tokens_observed
        message.simultaneous_gripper_contact_sample_count = (
            result.simultaneous_gripper_contact_sample_count
        )
        message.first_simultaneous_contact_stamp_available = (
            result.first_simultaneous_gripper_contact_source_timestamp_ns is not None
        )
        message.first_simultaneous_contact_source_timestamp_ns = (
            0
            if result.first_simultaneous_gripper_contact_source_timestamp_ns is None
            else result.first_simultaneous_gripper_contact_source_timestamp_ns
        )
        message.last_simultaneous_contact_stamp_available = (
            result.last_simultaneous_gripper_contact_source_timestamp_ns is not None
        )
        message.last_simultaneous_contact_source_timestamp_ns = (
            0
            if result.last_simultaneous_gripper_contact_source_timestamp_ns is None
            else result.last_simultaneous_gripper_contact_source_timestamp_ns
        )
        message.simultaneous_min_penetration_depth_available = (
            result.simultaneous_gripper_contact_max_min_penetration_depth_m
            is not None
        )
        message.simultaneous_max_min_penetration_depth_m = (
            0.0
            if result.simultaneous_gripper_contact_max_min_penetration_depth_m is None
            else result.simultaneous_gripper_contact_max_min_penetration_depth_m
        )
        message.simultaneous_min_force_magnitude_available = (
            result.simultaneous_gripper_contact_max_min_force_magnitude_n is not None
        )
        message.simultaneous_max_min_force_magnitude_n = (
            0.0
            if result.simultaneous_gripper_contact_max_min_force_magnitude_n is None
            else result.simultaneous_gripper_contact_max_min_force_magnitude_n
        )
        message.simultaneous_min_normal_force_available = (
            result.simultaneous_gripper_contact_max_min_abs_normal_force_n is not None
        )
        message.simultaneous_max_min_abs_normal_force_n = (
            0.0
            if result.simultaneous_gripper_contact_max_min_abs_normal_force_n is None
            else result.simultaneous_gripper_contact_max_min_abs_normal_force_n
        )
        message.gripper_joint_name = result.gripper_joint_name
        message.gripper_effort_available = result.latest_gripper_effort is not None
        message.gripper_effort_sample_count = result.gripper_effort_sample_count
        message.latest_gripper_effort = (
            0.0 if result.latest_gripper_effort is None else result.latest_gripper_effort
        )
        message.peak_abs_gripper_effort = (
            0.0
            if result.peak_abs_gripper_effort is None
            else result.peak_abs_gripper_effort
        )
        message.sequence_completion_stamp_available = (
            result.sequence_completed_source_timestamp_ns is not None
        )
        message.sequence_completed_source_timestamp_ns = (
            0
            if result.sequence_completed_source_timestamp_ns is None
            else result.sequence_completed_source_timestamp_ns
        )
        message.simultaneous_gripper_contact_sample_count_at_sequence_completion = (
            result.simultaneous_gripper_contact_sample_count_at_sequence_completion
        )
        message.last_simultaneous_contact_at_sequence_completion_available = (
            result.last_simultaneous_contact_source_timestamp_ns_at_sequence_completion
            is not None
        )
        message.last_simultaneous_contact_source_timestamp_ns_at_sequence_completion = (
            0
            if result.last_simultaneous_contact_source_timestamp_ns_at_sequence_completion
            is None
            else result.last_simultaneous_contact_source_timestamp_ns_at_sequence_completion
        )
        message.gripper_effort_at_sequence_completion_available = (
            result.gripper_effort_at_sequence_completion is not None
        )
        message.gripper_effort_at_sequence_completion = (
            0.0
            if result.gripper_effort_at_sequence_completion is None
            else result.gripper_effort_at_sequence_completion
        )
        message.peak_abs_gripper_effort_at_sequence_completion = (
            0.0
            if result.peak_abs_gripper_effort_at_sequence_completion is None
            else result.peak_abs_gripper_effort_at_sequence_completion
        )
        message.table_contact_count = result.table_contact_count
        message.matched_gripper_collision1 = (
            ""
            if result.matched_gripper_collision1 is None
            else result.matched_gripper_collision1
        )
        message.matched_gripper_collision2 = (
            ""
            if result.matched_gripper_collision2 is None
            else result.matched_gripper_collision2
        )
        self._assign_stamp(message.evidence_started_at, result.evidence_started_at_ns)
        self._assign_stamp(
            message.evidence_completed_at, result.evidence_completed_at_ns
        )
        self._assign_stamp(
            message.first_source_stamp, result.first_source_timestamp_ns
        )
        self._assign_stamp(message.last_source_stamp, result.last_source_timestamp_ns)
        self._assign_stamp(
            message.retention_started_at, result.retention_started_at_ns
        )
        self._assign_stamp(
            message.retention_completed_at, result.retention_completed_at_ns
        )
        message.clock_domain = result.clock_domain
        message.clock_epoch = result.clock_epoch
        message.evidence_digest = result.digest

    def _on_reset(self, request, response):
        del request
        with self._lock:
            if self._active_goal_handle is not None or self._reserved_task_id is not None:
                response.success = False
                response.message = "reset refused while observation is active"
                return response
            self._clock_epoch += 1
            self._clock_fault = None
            self._last_clock_ns = None
            self._core = None
        response.success = True
        response.message = (
            f"physics observer reset to local clock epoch {self._clock_epoch}; "
            "reset all peer nodes to the same epoch"
        )
        return response

    def _publish_status(self, phase: str, reason: str, task_id: str) -> None:
        message = String()
        message.data = (
            f"phase={phase};reason={reason};task_id={task_id};"
            f"scene_digest={self._scene.digest};pose_frame={self._pose_frame};"
            f"clock_domain={self._clock_domain};clock_epoch={self._clock_epoch};"
            "observer_is_read_only"
        )
        self._status.publish(message)

    def destroy_node(self) -> bool:
        self._server.destroy()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GraspPhysicsObserver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        # Jazzy's MultiThreadedExecutor leaves its Python worker pool alive
        # after shutdown().  Join submitted callbacks and retrieve completed
        # task exceptions before destroying the ActionServer handles.  This
        # keeps SIGINT teardown from leaving unobserved Future/Destroyable
        # warnings after an otherwise clean observer run.
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
