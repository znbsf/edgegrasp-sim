"""Message-level transition/readback tests; no ROS node or execution server."""

from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

from edgegrasp.carried_scene import RigidPose
from edgegrasp.scene import load_scene_contract
from edgegrasp_ros.carried_scene_policy import CarriedScenePolicy
from edgegrasp_ros.planning_scene_loader import (
    build_collision_object, build_target_pad_collision_matrix,
    expected_collision_objects, verify_planning_scene_objects,
)
from moveit_msgs.msg import AllowedCollisionMatrix, PlanningScene
from std_srvs.srv import Trigger


def fixture():
    root = Path(__file__).resolve().parents[4]
    contract = load_scene_contract(root/'ros_ws/src/edgegrasp_ros/config/scene_candidate024_face_aligned.json')
    pads = ('edgegrasp_fixed_finger_pad_link', 'edgegrasp_moving_finger_pad_link')
    parents = ('gripper_link', 'moving_jaw_so101_v1_link')
    acm = build_target_pad_collision_matrix(AllowedCollisionMatrix(), target_object_id='edgegrasp_target_cube',
        allowed_pad_links=pads, forbidden_parent_links=parents, allow=True)
    published = []
    ready = []
    node = SimpleNamespace(_contract=contract, _fault_latched=None, _ready=True,
        _last_observed_acm=acm, _include_optional_cube=True, _target_object_id='edgegrasp_target_cube',
        _target_frame='base_link', _target_contact_links=pads, _forbidden_target_contact_links=parents,
        _allow_target_pad_contacts=True, _pending=object(), _pending_started_wall=1.,
        _expected=expected_collision_objects(contract, include_optional=True), _forbidden_ids=(),
        _messages=tuple(build_collision_object(o, 'base_link') for o in contract.planning_objects(include_optional=True)),
        _observe_clock=lambda: None, _publish_ready=lambda *args: ready.append(args),
        _publish_acm_diff=lambda m: None,
        _scene_diff_publisher=SimpleNamespace(publish=published.append))
    policy = CarriedScenePolicy.__new__(CarriedScenePolicy)
    policy.loader=node
    policy.state='world'
    policy.generation=0
    policy.attached=None
    policy.fault=None
    policy.table_contact=False
    policy.pose_for=lambda op: RigidPose((.01, .02, .03), (0., 0., 0., 1.))
    return node, policy, ready


def readback(node, policy):
    scene = PlanningScene()
    scene.world.collision_objects=list(node._messages)
    scene.allowed_collision_matrix=node._desired_acm
    scene.robot_state.attached_collision_objects=[policy.attached] if policy.attached else []
    return scene


def test_transition_generation_and_exact_readback():
    node, policy, ready=fixture()
    for generation, operation in enumerate(('attach', 'place_contact', 'detach', 'refresh'), 1):
        response=policy.transition(operation, Trigger.Response())
        assert response.success, response.message
        assert not node._ready and ready[-1][0] is False
        assert node._pending is None and policy.generation == generation
        scene=readback(node, policy)
        assert verify_planning_scene_objects(scene.world.collision_objects, node._expected, node._forbidden_ids) is None
        assert policy.verify(scene) is None
        if operation == 'attach':
            bad=deepcopy(scene)
            bad.robot_state.attached_collision_objects.append(deepcopy(policy.attached))
            assert policy.verify(bad) == 'carried_object_missing_or_duplicate'
            bad=deepcopy(scene)
            bad.robot_state.attached_collision_objects[0].touch_links.append('gripper_link')
            assert policy.verify(bad) == 'carried_link_or_touch_policy_mismatch'
            bad=deepcopy(scene)
            bad.robot_state.attached_collision_objects[0].object.primitive_poses[0].position.x += .01
            assert policy.verify(bad).startswith('attached_position_mismatch')
            bad=deepcopy(scene)
            bad.allowed_collision_matrix=build_target_pad_collision_matrix(scene.allowed_collision_matrix,
                target_object_id='edgegrasp_target_cube', allowed_pad_links=('unrelated_link',),
                forbidden_parent_links=node._forbidden_target_contact_links, allow=True)
            assert policy.verify(bad) == 'unexpected_carried_contact_allowance:unrelated_link'
        node._ready=True
        node._last_observed_acm=node._desired_acm
    assert policy.state == 'placed'
    assert not policy.transition('detach', Trigger.Response()).success


def test_invalid_input_never_mutates_scene():
    node, policy, ready=fixture()
    def reject(op):
        raise ValueError('measurement_not_fresh')
    policy.pose_for=reject
    response=policy.transition('attach', Trigger.Response())
    assert not response.success
    assert policy.generation == 0 and policy.state == 'world' and not ready
    node._observe_clock=lambda: 'clock_rollback'
    assert not policy.transition('attach', Trigger.Response()).success
    assert node._fault_latched == 'clock_rollback' and not node._ready


def test_duplicate_world_object_is_rejected():
    node, _, _=fixture()
    objects=list(node._messages)
    assert verify_planning_scene_objects(objects+[objects[0]], node._expected) == 'duplicate_world_object'


def test_missing_causal_pair_receipt_proves_no_transition_mutation():
    node, policy, ready = fixture()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100))
    policy.recorded = None
    policy.measurements = []
    policy.last_source = 50
    policy.evidence = {'previous': True}
    policy.pose_for = CarriedScenePolicy.pose_for.__get__(policy)
    pending, messages = node._pending, node._messages
    response = policy.transition('attach', Trigger.Response())
    assert not response.success
    assert response.message == 'fresh_measured_cube_and_causal_tf_missing'
    assert policy.state == 'world' and policy.generation == 0
    assert policy.last_source == 50 and policy.evidence == {'previous': True}
    assert node._ready and not ready
    assert node._pending is pending and node._messages is messages
