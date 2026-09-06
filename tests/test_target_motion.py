import pytest

from edgegrasp.target_motion import LiftObservationBinding, PlannedLiftRegion


def test_lift_motion_preserves_residual_and_detects_failed_attachment():
    identity = dict(task_id="trial", target_id="cube", clock_domain="ros_sim", clock_epoch=2)
    binding = LiftObservationBinding.bind(
        **identity, source_ns=100, observed_center_m=(0.2, 0.1, 0.205),
        gripper_position_m=(0.21, 0.1, 0.225), gripper_orientation_xyzw=(0, 0, 0, 1))
    lifted = dict(**identity, source_ns=200, gripper_position_m=(0.21, 0.1, 0.265),
                  gripper_orientation_xyzw=(0, 0, 0, 1))
    assert binding.residual_m(**lifted, observed_center_m=(0.2, 0.1, 0.245)) < 1e-12
    assert binding.residual_m(**lifted, observed_center_m=(0.2, 0.1, 0.205)) > 0.005
    assert binding.residual_m(**lifted, observed_center_m=(0.206, 0.1, 0.245)) > 0.005
    for change in ({"clock_epoch": 3}, {"task_id": "next-trial"}, {"source_ns": 99}):
        with pytest.raises(ValueError, match="binding mismatch"):
            binding.residual_m(**(lifted | change), observed_center_m=(0.2, 0.1, 0.245))


def test_planned_region_bounds_motion_without_asserting_attachment():
    identity = dict(task_id="trial", target_id="cube", clock_domain="ros_sim", clock_epoch=2)
    binding = PlannedLiftRegion.bind(
        **identity, source_ns=100, initial_center_m=(0.2, 0.1, 0.205),
        descend_position_m=(0.21, 0.1, 0.225), lift_position_m=(0.21, 0.1, 0.265))
    # Stationary, lagging and fully lifted observations are spatially valid;
    # only the independent physics observer can distinguish grasp success.
    for z in (0.205, 0.2102, 0.245):
        assert binding.residual_m(**identity, source_ns=200,
                                  observed_center_m=(0.2, 0.1, z)) < 1e-12
    for point in ((0.206, 0.1, 0.21), (0.2, 0.1, 0.199), (0.2, 0.1, 0.251)):
        assert binding.residual_m(**identity, source_ns=200, observed_center_m=point) > .005
    for change in ({"clock_epoch": 3}, {"task_id": "other"}, {"target_id": "other"},
                   {"clock_domain": "wall"}, {"source_ns": 99}):
        with pytest.raises(ValueError, match="binding mismatch"):
            binding.residual_m(**(dict(**identity, source_ns=200) | change),
                               observed_center_m=(0.2, 0.1, 0.21))
    for end in ((0.22, 0.1, 0.265), (0.21, 0.1, 0.225), (0.21, 0.1, 0.266),
                (0.21, 0.1, float("nan"))):
        with pytest.raises(ValueError):
            PlannedLiftRegion.bind(**identity, source_ns=100,
                                   initial_center_m=(0.2, 0.1, 0.205),
                                   descend_position_m=(0.21, 0.1, 0.225), lift_position_m=end)


def test_measured_pad_pose_validation_and_separation():
    from types import SimpleNamespace
    from edgegrasp.grasp_geometry import OrientedBoxEnvelope
    from edgegrasp.target_motion import MeasuredCubeObservation, MeasuredPadBinding
    identity = dict(schema_version=1, target_id="cube", frame_id="base_link",
                    clock_domain="ros_sim", clock_epoch=0, source_ns=100,
                    center_m=[.035, 0, 0], orientation_rows=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    observation = MeasuredCubeObservation.parse(identity)
    profile = SimpleNamespace(target_id="cube", cube_size_m=(.05, .05, .05),
        end_effector_frame="gripper_frame_link",
        preclose_contact_policy=SimpleNamespace(fixed_pad_name="fixed"),
        required_contact_pad_obbs=(OrientedBoxEnvelope("fixed", (0, 0, 0), (.02, .02, .02),
                                   ((1, 0, 0), (0, 1, 0), (0, 0, 1))),))
    binding = MeasuredPadBinding.bind(task_id="task", observation=observation, profile=profile)
    arguments = dict(task_id="task", gripper_position_m=(0, 0, 0), gripper_orientation_xyzw=(0, 0, 0, 1))
    assert binding.residual_m(**arguments, observation=observation) < 1e-9
    outside = MeasuredCubeObservation.parse(identity | dict(source_ns=200, center_m=[.041, 0, 0]))
    assert binding.residual_m(**arguments, observation=outside) > .005
    for change in (dict(source_ns=99), dict(clock_epoch=1), dict(target_id="other"), dict(frame_id="other")):
        with pytest.raises(ValueError, match="binding mismatch"):
            binding.residual_m(**arguments, observation=MeasuredCubeObservation.parse(identity | change))
    for change in (dict(source_ns=True), dict(source_ns=2**63), dict(clock_epoch=2**64), dict(center_m=[float("nan"), 0, 0]),
                   dict(orientation_rows=[[2, 0, 0], [0, 1, 0], [0, 0, 1]]),
                   dict(orientation_rows=[[-1, 0, 0], [0, 1, 0], [0, 0, 1]])):
        with pytest.raises(ValueError):
            MeasuredCubeObservation.parse(identity | change)
