#!/usr/bin/env python3
"""One bounded, fixed-scene place/release/retreat after independently verified grasp.

All arm commands use PlanTarget and all gripper commands use ExecuteTrajectory.
Recorded plan poses are predeclared spatial configuration, never target freshness.
Truth/contact observations only score support and release; they never create poses.
"""

import argparse
import json
from pathlib import Path
import time
from math import dist
from carried_scene_client import transition, finish_cycle

from edgegrasp.release_cycle import PlacementSample, PlacementSynchronizer, StablePlacement
from edgegrasp.carried_scene import measured_pose
from edgegrasp.grasp_geometry import derive_grasp_stage_geometry, load_grasp_geometry_profile, validate_routed_grasp_stage_geometry, select_so101_control_grasp
from edgegrasp_interfaces.action import PlanTarget
from geometry_msgs.msg import PoseStamped
from prepare_so101_trial import PreparationClient
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from ros_gz_interfaces.msg import Contacts
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import Buffer, TransformListener


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def validate_regrasp_geometry(plan, measurement):
    pose = measured_pose(measurement)
    profile = load_grasp_geometry_profile(Path(plan['config']['grasp_geometry_path']))
    approach, descend = plan['segments'][:2]
    q = tuple(descend['target_orientation_xyzw'])
    nominal = tuple(plan['config']['cube_center_m'])
    if any(abs(a-b) > .001 for a, b in zip(pose.position, nominal)):
        raise ValueError('ready:target_outside_predeclared_control_scene')
    if plan['config'].get('bounded_control_pan', False):
        seed = plan['config']['bounded_pan_seed']
        _, detail = select_so101_control_grasp(
            nominal_center_m=nominal, cube_center_m=pose.position,
            cube_orientation_xyzw=pose.orientation,
            approach_position_m=tuple(seed['approach_position_m']),
            approach_orientation_xyzw=tuple(seed['approach_orientation_xyzw']),
            grasp_orientation_xyzw=tuple(seed['grasp_orientation_xyzw']), profile=profile)
        return {'source_ns': measurement['source_ns'], 'orientation_xyzw': pose.orientation,
                'preclose_geometry_validated': True, 'bounded_control_pan': detail}
    geometry = derive_grasp_stage_geometry(nominal, q, profile)
    validate_routed_grasp_stage_geometry(
        cube_center_m=pose.position, cube_orientation_xyzw=pose.orientation,
        approach_position_m=tuple(approach['target_position_m']),
        approach_orientation_xyzw=tuple(approach['target_orientation_xyzw']),
        descend_position_m=geometry.descend_position_m,
        lift_position_m=geometry.lift_position_m, grasp_orientation_xyzw=q,
        gripper_position_rad=profile.gripper_contact_position_rad, profile=profile)
    return {'source_ns': measurement['source_ns'], 'orientation_xyzw': pose.orientation,
            'preclose_geometry_validated': True, 'fixed_control_stage_geometry': True}


class CycleClient(PreparationClient):
    def __init__(self, output):
        super().__init__('target_cube')
        self.arm = ActionClient(self, PlanTarget, '/edgegrasp/plan_target')
        self.output = output
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.pose = self.contacts = None
        self.sync = PlacementSynchronizer()
        self.create_subscription(PoseStamped, '/edgegrasp/target_cube_pose',
                                 lambda m: self.sync.push('poses', stamp_ns(m), m), 10)
        self.create_subscription(Contacts, '/edgegrasp/target_cube_contacts',
                                 lambda m: self.sync.push('contacts', stamp_ns(m), m), 10)

    def _on_joint_state(self, message):
        previous = self.latest_joint_observed_at
        super()._on_joint_state(message)
        if self.latest_joint_observed_at != previous:
            self.sync.push('joints', stamp_ns(message), dict(self.latest_joint_positions))

    def record(self, value):
        value = {'observed_now_ns': self.get_clock().now().nanoseconds, **value}
        with self.output.open('a') as stream:
            stream.write(json.dumps(value) + '\n')

    def arm_stage(self, task, stage, number, segment):
        if not self.arm.wait_for_server(timeout_sec=5):
            raise RuntimeError('plan_target unavailable')
        target = self.fresh_target(5)
        goal = PlanTarget.Goal()
        goal.task_id, goal.stage, goal.sequence_no = task, stage, number
        goal.target_id = target.target_id
        goal.source_timestamp_ns = self._source_ns(target)
        goal.clock_domain, goal.clock_epoch = target.clock_domain, target.clock_epoch
        goal.target_pose.header = target.observation.header
        for field, value in zip(('x', 'y', 'z'), segment['target_position_m']):
            setattr(goal.target_pose.pose.position, field, value)
        for field, value in zip(('x', 'y', 'z', 'w'), segment['target_orientation_xyzw']):
            setattr(goal.target_pose.pose.orientation, field, value)
        goal.planning_group = 'arm'
        goal.pipeline_id = 'pilz_industrial_motion_planner'
        goal.planner_id = 'PTP'
        goal.planning_timeout_s = 2.0
        goal.velocity_scaling, goal.acceleration_scaling = .15, .1
        goal.plan_only = True
        handle = self.wait_future(self.arm.send_goal_async(goal), 10, 'arm response timeout')
        if not handle.accepted:
            raise RuntimeError(f'{stage}:goal rejected')
        wrapped = self.wait_future(handle.get_result_async(), 40, 'arm result timeout')
        result = wrapped.result
        self.record({'stage': stage, 'wrapper_status': int(wrapped.status),
                     'result': message_to_ordereddict(result)})
        if not (wrapped.status == 4 and result.success and result.accepted
                and result.command_id == f'{task}|{stage}|{number}'
                and result.trajectory_dispatched and result.gate_accepted
                and result.gate_terminal and result.downstream_terminal_observed
                and result.fjt_error_code == 0):
            raise RuntimeError(f'{stage}:{result.reason}')

    def supported(self, stage, released=False, retreated=False, opened=False):
        scorer = StablePlacement(released=released, opened=opened)
        deadline = time.monotonic() + 20
        self.sync.poses.clear()  # Evidence for this stage must follow its start.
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            now = self.get_clock().now().nanoseconds
            for source, pose, matched, reason in self.sync.ready(now):
                if reason or time.monotonic()-self.latest_joint_observed_at > .2:
                    scorer.first = None
                    self.record({'stage': stage, 'stamp_ns': source,
                                 'input_rejected': reason or 'joint_receive_stale'})
                    continue
                if self.score_support(stage, scorer, source, pose, matched, now, retreated):
                    return
        raise RuntimeError(f'{stage}:continuous_support_evidence_timeout')

    def score_support(self, stage, scorer, source, pose, matched, now, retreated):
        contacts, joints = matched[0][1], matched[1][1]
        # The existing cube sensor includes both table and finger pairs.
        # The world has a table sensor too, but scene_bridge.yaml does not
        # expose that separate Gazebo topic to ROS.
        pairs = [f'{c.collision1.name}|{c.collision2.name}' for c in contacts.contacts]
        table_contact = any('target_cube::' in s and 'table::' in s for s in pairs)
        pad_contact = any('target_cube' in s and 'finger_pad' in s for s in pairs)
        moving_pad_contact = any('target_cube::' in s and 'moving_finger_pad' in s for s in pairs)
        p, q = pose.pose.position, pose.pose.orientation
        sample = PlacementSample(source, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                                 table_contact, pad_contact,
                                     joints.get('gripper', float('nan')), moving_pad_contact)
        passed = scorer.observe(sample, now)
        clearance = None
        if retreated:
            try:
                transform = self.tf.lookup_transform('base_link', 'gripper_frame_link', Time())
                t = transform.transform.translation
                clearance = dist(sample.position, (t.x, t.y, t.z))
                if not (0 <= now-stamp_ns(transform) <= 200_000_000 and clearance >= .08):
                    scorer.first = None
                    passed = False
            except Exception:
                scorer.first = None
                passed = False
        self.record({'stage': stage, 'stamp_ns': source, 'position': sample.position,
                     'contact_source_ns': matched[0][0], 'joint_source_ns': matched[1][0],
                     'orientation': sample.orientation, 'table_contact': table_contact,
                     'moving_pad_contact': moving_pad_contact,
                     'pad_contact': pad_contact, 'gripper': sample.gripper_position,
                     'end_effector_center_clearance_m': clearance, 'passed': passed})
        return passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--task-id', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    if (plan['status'] != 'PLAN_ONLY_PASS' or plan['execution_attempted']
            or not plan['config'].get('release_cycle') or len(plan['segments']) != 5
            or not all(s['validator_accepted'] for s in plan['segments'])):
        raise ValueError('matching five-stage release cycle plan required')
    if not plan['config'].get('carried_scene'):
        raise ValueError('fresh carried-scene five-stage plan required')
    args.output.touch(exist_ok=False)
    rclpy.init()
    node = CycleClient(args.output)
    try:
        for operation in ('attach', 'place_contact'):
            node.record({'stage': 'scene_' + operation,
                         'confirmation': transition(node, operation, scope='live_rgbd_tf')})
        node.arm_stage(args.task_id, 'place', 4, plan['segments'][3])
        node.supported('supported_before_release')
        release = node.execute(task_id=args.task_id, sequence_no=5,
                               controller='gripper_controller', joints=('gripper',),
                               positions=(1.5,), efforts=(0.,), duration_s=2., timeout_s=20.)
        node.record({'stage': 'release', 'result': release})
        if not release['success']:
            raise RuntimeError('release:' + release['reason'])
        # Fixed-pad touching can persist until withdrawal. Opening must remove
        # the moving-pad contact; withdrawal must remove *all* pad contacts.
        node.supported('supported_and_open', opened=True)
        node.record({'stage': 'scene_detach',
                     'confirmation': transition(node, 'detach', scope='live_rgbd_tf')})
        node.arm_stage(args.task_id, 'retreat', 6, plan['segments'][4])
        node.supported('supported_after_retreat', released=True, retreated=True)
        target = node.fresh_target(5.)
        point = target.observation.point
        center = (point.x, point.y, point.z)
        errors = [abs(a-b) for a, b in zip(center, plan['config']['cube_center_m'])]
        # Reuse the existing trial client's 1 mm scene agreement admission.
        # A cube resting elsewhere inside the placement region is not ready
        # for another fixed-scene grasp.
        node.record({'stage': 'ready', 'target_source_ns': node._source_ns(target),
                     'measured_center_m': center, 'scene_errors_m': errors})
        if any(error > .001 for error in errors):
            raise RuntimeError('ready:target_outside_predeclared_control_scene')
        refreshed = transition(node, 'refresh', scope='live_rgbd_tf')
        node.record({'stage': 'scene_refresh', 'confirmation': refreshed})
        node.record({'stage': 'regrasp_geometry', 'result': validate_regrasp_geometry(
            plan, refreshed['carried_evidence']['measurement'])})
        node.record({'stage': 'completed_cycle_handoff', 'result': finish_cycle(node)})
        node.record({'status': 'PLACE_RELEASE_RETREAT_COMPLETE', 'physics_grasp_claim': False})
        return 0
    except Exception as exc:
        node.record({'status': 'STOPPED', 'reason': str(exc), 'retry_count': 0})
        return 2
    finally:
        node.arm.destroy()
        node.gate.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
