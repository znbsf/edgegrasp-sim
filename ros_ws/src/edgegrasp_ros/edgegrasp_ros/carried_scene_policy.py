"""Explicit carried-object transitions; readiness remains owned by the loader."""

import hashlib
import json
import os
from pathlib import Path
import time
from collections import deque

from edgegrasp.carried_scene import RigidPose, validate_live_measurement
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def ros_pose(pose):
    message = Pose()
    for name, value in zip(('x', 'y', 'z'), pose.position):
        setattr(message.position, name, float(value))
    for name, value in zip(('x', 'y', 'z', 'w'), pose.orientation):
        setattr(message.orientation, name, float(value))
    return message


class CarriedScenePolicy:
    def __init__(self, loader):
        self.loader = loader
        self.state = 'world'
        self.generation = 0
        self.attached = None
        self.diff = None
        self.table_contact = False
        self.last_source = None
        self.measurement = None
        self.measurements = deque(maxlen=32)
        self.received = 0.
        self.evidence = None
        self.fault = None
        self.tf = Buffer(node=loader)
        self.listener = TransformListener(self.tf, loader)
        loader.create_subscription(String, '/edgegrasp/measured_cube', self.observe, 10)
        # This opt-in is restricted to a graph without motion admission nodes.
        # Its historical values are never accepted by the live measurement path.
        path = os.environ.get('EDGEGRASP_CARRIED_PLAN_EVIDENCE')
        self.recorded = json.loads(Path(path).read_text()) if path else None
        if self.recorded:
            if self.recorded['scope'] != 'recorded_carried_geometry_plan_only':
                raise ValueError('invalid_recorded_carried_scope')
            for source in self.recorded['sources']:
                if hashlib.sha256(Path(source['path']).read_bytes()).hexdigest() != source['sha256']:
                    raise ValueError('recorded_carried_provenance_mismatch')
        for operation in ('attach', 'place_contact', 'detach', 'refresh'):
            loader.create_service(Trigger, '/edgegrasp/carried_' + operation,
                                  lambda req, res, op=operation: self.transition(op, res))

    def observe(self, message):
        try:
            value = json.loads(message.data)
            if self.measurement is not None and value['source_ns'] < self.measurement['source_ns']:
                self.fault = 'measured_source_rollback'
                self.loader._fault_latched = self.fault
                self.loader._publish_ready(False, self.fault)
                return
            self.measurement = value
            self.received = time.monotonic()
            self.measurements.append((value, self.received))
        except (ValueError, KeyError, TypeError):
            self.measurement = None

    def pose_for(self, operation):
        node = self.loader
        if self.recorded:
            forbidden = ('trajectory_gate', 'grasp_sequence', 'moveit_plan_only_adapter',
                         'trial_preparation_client')
            if any(any(word in name for word in forbidden) for name in node.get_node_names()):
                raise ValueError('recorded_geometry_forbidden_in_execution_graph')
            value = self.recorded[operation]
            self.evidence = {'scope': 'recorded_geometry_only', 'sources': self.recorded['sources']}
            return RigidPose(tuple(value['position']), tuple(value['orientation']))
        now = node.get_clock().now().nanoseconds
        matched = None
        for value, received in reversed(self.measurements):
            if (not 0 <= now-value['source_ns'] <= 200_000_000
                    or time.monotonic()-received > .2
                    or (self.last_source is not None and value['source_ns'] <= self.last_source)):
                continue
            try:
                transform = self.tf.lookup_transform('base_link', 'gripper_frame_link',
                                                    Time(nanoseconds=value['source_ns']))
                matched = value, received, transform
                break
            except Exception:
                continue
        if matched is None:
            raise ValueError('fresh_measured_cube_and_causal_tf_missing')
        value, received, transform = matched
        stamp = transform.header.stamp.sec*1_000_000_000+transform.header.stamp.nanosec
        cube = validate_live_measurement(value, now_ns=node.get_clock().now().nanoseconds,
                                         epoch=node._clock_epoch, tf_stamp_ns=stamp,
                                         last_source_ns=self.last_source,
                                         receive_age_s=time.monotonic()-received)
        self.last_source = value['source_ns']
        self.evidence = {'scope': 'live_rgbd_tf', 'measurement': value, 'tf_source_ns': stamp}
        if operation in ('detach', 'refresh'):
            return cube
        t, q = transform.transform.translation, transform.transform.rotation
        end = RigidPose((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        relative = end.inverse().compose(cube)
        if sum(v*v for v in relative.position) > .15**2:
            raise ValueError('carried_object_too_far_from_gripper')
        return relative

    def transition(self, operation, response):
        from edgegrasp_ros.planning_scene_loader import (
            build_collision_object, build_remove_collision_object,
            build_target_pad_collision_matrix,
        )
        node = self.loader
        try:
            clock_fault = node._observe_clock()
            if clock_fault:
                node._fault_latched = clock_fault
                node._ready = False
                node._publish_ready(False, clock_fault)
            if self.fault or node._fault_latched:
                raise ValueError('carried_transition_clock_or_source_fault')
            if not node._ready or node._last_observed_acm is None or not node._include_optional_cube:
                raise ValueError('carried_transition_requires_confirmed_scene')
            allowed = {'attach': ('world', 'placed'), 'place_contact': ('attached',),
                       'detach': ('placing',), 'refresh': ('placed',)}
            if self.state not in allowed[operation]:
                raise ValueError('invalid_carried_transition:' + self.state + ':' + operation)
            if operation == 'attach' and not node._allow_target_pad_contacts:
                raise ValueError('attachment_requires_existing_pad_contact_policy')
            target = next(item for item in node._contract.objects
                          if item.planning_scene_id == node._target_object_id)
            pose = self.pose_for(operation) if operation != 'place_contact' else None
            # Compute before revoking readiness; no partial mutation on bad input.
            matrix = build_target_pad_collision_matrix(
                node._last_observed_acm, target_object_id=node._target_object_id,
                allowed_pad_links=('edgegrasp_table',),
                forbidden_parent_links=node._forbidden_target_contact_links,
                allow=operation != 'attach')
            node._pending = None  # discard the old generation's read-only query
            node._pending_started_wall = None
            node._ready = False
            self.generation += 1
            node._publish_ready(False, 'carried_transition_pending')
            scene = PlanningScene()
            scene.is_diff = True
            scene.robot_state.is_diff = True
            node._desired_acm = matrix
            self.table_contact = operation != 'attach'
            if operation == 'attach':
                attached = AttachedCollisionObject()
                attached.link_name = 'gripper_frame_link'
                attached.touch_links = list(node._target_contact_links)
                attached.object = build_collision_object(target, attached.link_name)
                attached.object.primitive_poses = [ros_pose(pose)]
                scene.world.collision_objects = [build_remove_collision_object(
                    target.planning_scene_id, node._target_frame)]
                scene.robot_state.attached_collision_objects = [attached]
                self.attached = attached
                node._expected = tuple(item for item in node._expected
                                       if item.object_id != target.planning_scene_id)
                node._messages = tuple(item for item in node._messages
                                       if item.id != target.planning_scene_id)
                node._forbidden_ids = (target.planning_scene_id,)
                self.state = 'attached'
            elif operation == 'place_contact':
                scene.robot_state.attached_collision_objects = [self.attached]
                scene.world.collision_objects = [build_remove_collision_object(
                    target.planning_scene_id, node._target_frame)]
                self.state = 'placing'
            else:
                removed = AttachedCollisionObject()
                removed.link_name = 'gripper_frame_link'
                removed.object.id = target.planning_scene_id
                removed.object.operation = CollisionObject.REMOVE
                scene.robot_state.attached_collision_objects = [removed]
                world = build_collision_object(target, node._target_frame)
                world.primitive_poses = [ros_pose(pose)]
                scene.world.collision_objects = [world]
                from edgegrasp_ros.planning_scene_loader import ExpectedCollisionObject
                expected = ExpectedCollisionObject(target.planning_scene_id, node._target_frame,
                                                   target.size_m, pose.position, pose.orientation)
                node._expected = (*tuple(item for item in node._expected
                                         if item.object_id != target.planning_scene_id), expected)
                node._messages = (*tuple(item for item in node._messages
                                         if item.id != target.planning_scene_id), world)
                node._forbidden_ids = ()
                self.attached = None
                self.state = 'placed'
            self.diff = scene
            node._next_attempt_wall = 0.
            node._publish_acm_diff(matrix)
            node._scene_diff_publisher.publish(scene)
            response.success = True
            response.message = json.dumps({'generation': self.generation, 'state': self.state})
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def verify(self, scene):
        from edgegrasp_ros.planning_scene_loader import (
            ExpectedCollisionObject, _effective_collision_permission,
            _quaternion_tuple, verify_planning_scene_objects,
        )
        node = self.loader
        attached = [item for item in scene.robot_state.attached_collision_objects
                    if item.object.id == node._target_object_id]
        if len(attached) != (1 if self.attached else 0):
            return 'carried_object_missing_or_duplicate'
        if self.attached:
            item = attached[0]
            if (item.link_name != self.attached.link_name
                    or set(item.touch_links) != set(self.attached.touch_links)
                    or len(item.touch_links) != len(self.attached.touch_links)):
                return 'carried_link_or_touch_policy_mismatch'
            desired = self.attached.object
            p = desired.primitive_poses[0]
            expected = ExpectedCollisionObject(desired.id, desired.header.frame_id,
                tuple(desired.primitives[0].dimensions),
                (p.position.x, p.position.y, p.position.z), _quaternion_tuple(p.orientation))
            reason = verify_planning_scene_objects([item.object], (expected,))
            if reason:
                return 'attached_' + reason
        if self.generation:
            matrix = scene.allowed_collision_matrix
            if _effective_collision_permission(matrix, node._target_object_id, 'edgegrasp_table') != self.table_contact:
                return 'carried_table_contact_policy_mismatch'
            allowed = set(node._target_contact_links) if node._allow_target_pad_contacts else set()
            if self.table_contact:
                allowed.add('edgegrasp_table')
            for name in set(matrix.entry_names) | set(matrix.default_entry_names):
                if name != node._target_object_id and name not in allowed:
                    if _effective_collision_permission(matrix, node._target_object_id, name):
                        return 'unexpected_carried_contact_allowance:' + name
            for name in (*node._target_contact_links, *node._forbidden_target_contact_links):
                if _effective_collision_permission(matrix, 'edgegrasp_table', name):
                    return 'robot_table_contact_forbidden:' + name
        return None
