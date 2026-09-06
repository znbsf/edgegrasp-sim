"""Zero-command health checks using timestamped TF and visual target messages."""

from dataclasses import replace

from edgegrasp.grasp_sequence import GraspPhase
from edgegrasp.target_motion import LiftObservationBinding, PlannedLiftRegion
from edgegrasp_interfaces.action import GraspSequence
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter

from edgegrasp_grasp_sequence.node import GraspSequenceNode, _TargetSample


def test_lift_health_requires_correlated_tf_and_retains_five_mm_residual():
    context = Context()
    rclpy.init(context=context, domain_id=169)
    node = GraspSequenceNode(context=context, parameter_overrides=[
        Parameter("use_sim_time", value=True), Parameter("clock_domain", value="ros_sim"),
        Parameter("observed_target_motion", value="rigid_lift")])
    try:
        goal = GraspSequence.Goal()
        goal.task_id, goal.target.target_id = "test-lift", "cube"
        node._active_request = goal
        node._active_anchor = (0.2, 0.1, 0.205)
        node._latest_target = _TargetSample("cube", (0.2, 0.1, 0.245), "base_link",
                                             2_000_000_000, "ros_sim", 0, 2_010_000_000)
        node._lift_observation_binding = LiftObservationBinding.bind(
            task_id="test-lift", target_id="cube", clock_domain="ros_sim", clock_epoch=0,
            source_ns=1_000_000_000, observed_center_m=node._active_anchor,
            gripper_position_m=(0.21, 0.1, 0.225), gripper_orientation_xyzw=(0, 0, 0, 1))
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "base_link", "gripper_frame_link"
        tf.header.stamp.sec = 2
        tf.transform.translation.x, tf.transform.translation.y = 0.21, 0.1
        tf.transform.translation.z = 0.265
        tf.transform.rotation.w = 1.
        node._observation_tf.set_transform(tf, "test-joint-state-fk")
        node._core._phase = GraspPhase.DESCEND_EXEC
        assert not node._health(2_010_000_000).target.ready
        node._core._phase = GraspPhase.LIFT_EXEC
        assert node._health(2_010_000_000).target.ready
        node._latest_target = replace(node._latest_target, position=(0.206, 0.1, 0.245))
        assert not node._health(2_010_000_000).target.ready
        # Images may arrive 2 ms ahead of joint-state TF. Select the newest
        # complete pair inside the sequence freshness budget, preserving its source.
        mature = replace(node._latest_target, position=(0.2, 0.1, 0.245))
        node._target_cache[mature.key] = mature
        node._latest_target = replace(node._latest_target, source_timestamp_ns=2_002_000_000)
        paired = node._health(2_010_000_000)
        assert paired.target.ready and paired.target_source_timestamp_ns == 2_000_000_000
        # Once newer TF arrives, its 6 mm slip must reject; never cherry-pick
        # an older good residual over a newer complete observation.
        tf.header.stamp.nanosec = 2_000_000
        node._observation_tf.set_transform(tf, "test-joint-state-fk")
        assert not node._health(2_010_000_000).target.ready
        node._latest_target = replace(node._latest_target, target_id="wrong-cube")
        assert node._health(2_010_000_000).target.reason == "target_identity_changed"
        node._latest_target = replace(node._latest_target, target_id="cube")
        node._latest_target = replace(node._latest_target, position=(0.2, 0.1, 0.245),
                                      source_timestamp_ns=3_000_000_000)
        missing_tf = node._health(3_010_000_000).target
        assert not missing_tf.ready and "lift_observation_unavailable" in missing_tf.reason
        assert node._target_drift_tolerance_m == 0.005
    finally:
        node.destroy_node()
        context.shutdown()


def test_planned_region_health_keeps_identity_source_and_pre_lift_guard():
    context = Context()
    rclpy.init(context=context, domain_id=169)
    node = GraspSequenceNode(context=context, parameter_overrides=[
        Parameter("use_sim_time", value=True), Parameter("clock_domain", value="ros_sim"),
        Parameter("observed_target_motion", value="planned_lift_region")])
    try:
        goal = GraspSequence.Goal()
        goal.task_id, goal.target.target_id = "test-region", "cube"
        node._active_request = goal
        node._active_anchor = (0.2, 0.1, 0.205)
        node._latest_target = _TargetSample("cube", (0.2012, 0.1, 0.2102), "base_link",
                                           2_000_000_000, "ros_sim", 0, 2_010_000_000)
        node._core._phase = GraspPhase.DESCEND_EXEC
        assert not node._health(2_010_000_000).target.ready
        node._core._phase = GraspPhase.LIFT_EXEC
        assert not node._health(2_010_000_000).target.ready  # No binding.
        node._lift_observation_binding = PlannedLiftRegion.bind(
            task_id="test-region", target_id="cube", clock_domain="ros_sim", clock_epoch=0,
            source_ns=1_000_000_000, initial_center_m=node._active_anchor,
            descend_position_m=(.21, .1, .225), lift_position_m=(.21, .1, .265))
        health = node._health(2_010_000_000)
        assert health.target.ready and health.target_source_timestamp_ns == 2_000_000_000
        assert node._observation_tf is None  # No gripper attachment assumption.
        for position in ((.206, .1, .21), (.2, .1, .199), (.2, .1, .251)):
            node._latest_target = replace(node._latest_target, position=position)
            assert not node._health(2_010_000_000).target.ready
        node._latest_target = replace(node._latest_target, position=(.2, .1, .21),
                                      source_timestamp_ns=999_000_000)
        assert not node._health(2_010_000_000).target.ready
        node._latest_target = replace(node._latest_target, source_timestamp_ns=2_000_000_000,
                                      target_id="other")
        assert node._health(2_010_000_000).target.reason == "target_identity_changed"
        assert node._target_drift_tolerance_m == .005
    finally:
        node.destroy_node()
        context.shutdown()


def test_measured_pad_atomic_source_tf_and_conflict(monkeypatch):
    import json
    from pathlib import Path
    from rclpy.time import Time
    from std_msgs.msg import String
    from edgegrasp.target_motion import MeasuredPadBinding
    path = Path(__file__).resolve().parents[2] / "edgegrasp_ros/config/so101_grasp_geometry_candidate024_face_aligned_q0p40.json"
    context = Context()
    rclpy.init(context=context, domain_id=169)
    node = GraspSequenceNode(context=context, parameter_overrides=[
        Parameter("use_sim_time", value=True), Parameter("clock_domain", value="ros_sim"),
        Parameter("observed_target_motion", value="measured_pad"),
        Parameter("observed_cube_geometry_profile", value=str(path))])
    try:
        # Enforce the lock order used by concurrent sequence events. This
        # reproduces the state -> event inversion even without sending motion.
        from threading import RLock, local
        held = local()
        held.ranks = []
        class OrderedLock:
            def __init__(self, rank):
                self.rank, self.lock = rank, RLock()
            def __enter__(self):
                assert not held.ranks or self.rank >= held.ranks[-1], "lock order inversion"
                self.lock.acquire()
                held.ranks.append(self.rank)
            def __exit__(self, *unused):
                held.ranks.pop()
                self.lock.release()
        monkeypatch.setattr(node, "_core_event_lock", OrderedLock(1))
        monkeypatch.setattr(node, "_state_lock", OrderedLock(2))
        node.get_clock().set_ros_time_override(Time(nanoseconds=2_010_000_000))
        pad = node._observed_cube_profile.required_contact_pad_obbs[0]
        value = dict(schema_version=1, target_id="target_cube", frame_id="base_link", source_ns=2_000_000_000,
                     clock_domain="ros_sim", clock_epoch=0, center_m=pad.center_m,
                     orientation_rows=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        node._on_measured_cube(String(data=json.dumps(value)))
        assert node._input_fault is None
        goal = GraspSequence.Goal()
        goal.task_id, goal.target.target_id = "pad-test", "target_cube"
        node._active_request, node._active_anchor = goal, pad.center_m
        node._core._phase = GraspPhase.LIFT_EXEC
        observation = node._measured_cube_cache[node._latest_target.key]
        node._lift_observation_binding = MeasuredPadBinding.bind(
            task_id="pad-test", observation=observation, profile=node._observed_cube_profile)
        assert not node._health(2_010_000_000).target.ready  # Exact-source TF absent.
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "base_link", "gripper_frame_link"
        tf.header.stamp.sec, tf.transform.rotation.w = 2, 1.
        node._observation_tf.set_transform(tf, "test")
        assert node._health(2_010_000_000).target.ready
        newer = value | dict(source_ns=2_002_000_000, center_m=[.15, 0., 0.])
        node._on_measured_cube(String(data=json.dumps(newer)))
        assert node._health(2_010_000_000).target.ready  # Newer TF pending; older complete pair.
        tf.header.stamp.nanosec = 2_000_000
        node._observation_tf.set_transform(tf, "test")
        assert not node._health(2_010_000_000).target.ready  # No older-good cherry-picking.
        node._on_measured_cube(String(data=json.dumps(newer | dict(center_m=[0., 0., 0.]))))
        assert "source conflict" in node._input_fault
    finally:
        node.destroy_node()
        context.shutdown()


def test_readiness_wait_cannot_dispatch_an_expired_observation(monkeypatch):
    from types import SimpleNamespace
    import edgegrasp_grasp_sequence.node as module
    context = Context()
    rclpy.init(context=context, domain_id=169)
    node = GraspSequenceNode(context=context, parameter_overrides=[
        Parameter("use_sim_time", value=True), Parameter("clock_domain", value="ros_sim")])
    try:
        node._joint_state = SimpleNamespace(positions={"gripper": 1.5})
        monkeypatch.setattr(node._gripper_client, "server_is_ready", lambda: False)
        monkeypatch.setattr(module, "wait_for_readiness", lambda ready: True)
        monkeypatch.setattr(node, "_now_ns", lambda: 1_201_000_000)
        outcome = node._submit_gripper(SimpleNamespace(source_timestamp_ns=1_000_000_000,
                                                       clock_epoch=0, clock_domain="ros_sim"))
        assert not outcome.accepted
        assert outcome.reason == "target_expired_during_readiness_wait"
    finally:
        node.destroy_node()
        context.shutdown()
