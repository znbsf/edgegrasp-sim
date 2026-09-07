"""A transition receipt is not readiness: wait for its exact confirmed generation."""

import json
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool


def transition(node, operation, *, scope, timeout_s=10.):
    seen = []
    measurement_sources = []
    admission_rejections = []
    def observe(message):
        try:
            seen.append(json.loads(message.data))
        except ValueError:
            pass
    subscription = node.create_subscription(String, '/edgegrasp/planning_scene_status',
        observe, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))
    def observe_measurement(message):
        try:
            value = json.loads(message.data)
            if (isinstance(value, dict) and value.get('target_id') == 'target_cube' and value.get('frame_id') == 'base_link'
                    and value.get('clock_domain') == 'ros_sim' and value.get('clock_epoch') == 0
                    and type(value.get('source_ns')) is int):
                measurement_sources.append(value['source_ns'])
        except (ValueError, TypeError):
            pass
    measurement_subscription = node.create_subscription(
        String, '/edgegrasp/measured_cube', observe_measurement, 10)
    client = node.create_client(Trigger, '/edgegrasp/carried_' + operation)
    deadline = time.monotonic()+timeout_s
    try:
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError('carried_service_unavailable:' + operation)
        for attempt in range(3):
            future = client.call_async(Trigger.Request())
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            if not future.done():
                # No terminal receipt: never resend an ambiguous mutation.
                raise RuntimeError('carried_transition_response_timeout')
            response = future.result()
            if response.success:
                break
            if (scope != 'live_rgbd_tf' or attempt == 2
                    or response.message != 'fresh_measured_cube_and_causal_tf_missing'):
                raise RuntimeError('carried_transition_rejected:' + response.message)
            # This exact rejection occurs before any state/generation mutation.
            # Wait for a newer input; all server freshness/TF/fault checks remain.
            prior_source = max(measurement_sources, default=-1)
            rejection = dict(operation=operation, attempt=attempt+1,
                             reason=response.message, prior_source_ns=prior_source)
            admission_rejections.append(rejection)
            if hasattr(node, 'record'):
                node.record({'stage': 'carried_input_wait', **rejection})
            while max(measurement_sources, default=-1) <= prior_source and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
            if time.monotonic() >= deadline:
                raise RuntimeError('carried_fresh_input_wait_timeout')
        receipt = json.loads(response.message)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            for status in reversed(seen):
                if (status.get('scene_generation') == receipt['generation']
                        and status.get('carried_state') == receipt['state']
                        and status.get('ready') is True and status.get('reason') == 'confirmed'
                        and status.get('clock_domain') == 'ros_sim'
                        and status.get('clock_epoch') == 0
                        and (status.get('carried_evidence') or {}).get('scope') == scope):
                    return {**status, 'client_admission_rejections': admission_rejections}
        raise RuntimeError('carried_generation_confirmation_timeout:' + operation)
    finally:
        node.destroy_client(client)
        node.destroy_subscription(subscription)
        node.destroy_subscription(measurement_subscription)


def finish_cycle(node, timeout_s=10.):
    statuses = []
    subscription = node.create_subscription(String, '/edgegrasp/planning_scene_status',
        lambda message: statuses.append(json.loads(message.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=ReliabilityPolicy.RELIABLE))
    pads = node.create_client(SetBool, '/edgegrasp/set_target_pad_contacts')
    handoff = node.create_client(Trigger, '/edgegrasp/finish_completed_grasp_cycle')
    deadline = time.monotonic()+timeout_s
    def wait(future):
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not future.done():
            raise RuntimeError('completed_cycle_handoff_timeout')
        response = future.result()
        if not response.success:
            raise RuntimeError('completed_cycle_handoff_rejected:' + response.message)
        return response
    try:
        if not pads.wait_for_service(timeout_sec=timeout_s) or not handoff.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError('completed_cycle_services_unavailable')
        wait(pads.call_async(SetBool.Request(data=False)))
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if (statuses and statuses[-1].get('ready') is True
                    and statuses[-1].get('allow_target_pad_contacts') is False
                    and statuses[-1].get('carried_state') == 'placed'):
                result = wait(handoff.call_async(Trigger.Request()))
                return {'success': True, 'reason': result.message, 'epoch_changed': False}
        raise RuntimeError('pad_policy_revocation_not_confirmed')
    finally:
        node.destroy_client(pads)
        node.destroy_client(handoff)
        node.destroy_subscription(subscription)
