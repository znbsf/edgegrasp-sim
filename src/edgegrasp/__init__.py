"""ROS-independent deterministic core for EdgeGrasp."""

from .backend import BackendKind, MockBackend, create_backend
from .config import (
    DEFAULT_CLOCK_DOMAIN,
    DEFAULT_TARGET_FRAME,
    ROS_SIM_CLOCK_DOMAIN,
    ROS_SYSTEM_CLOCK_DOMAIN,
    ros_clock_domain,
    validate_ros_clock_domain,
)
from .controller import EdgeGraspController, ResultCode
from .fsm import PipelineState, StateMachine
from .grasp_sequence import GraspPhase, GraspSequenceController
from .grasp_evidence import (
    EvidencePhase,
    GraspPhysicsEvidenceController,
    GripperEffortSample,
)
from .models import Target3D, Vector3
from .planner import EndpointWorkspaceGate
from .permission import MotionPermissionGate
from .predictor import ConstantVelocityPredictor
from .safety import StaleTargetGate
from .so101_contract import (
    SO101_ARM_ACTION,
    SO101_ARM_JOINTS,
    SO101_GRIPPER_ACTION,
    SO101_GRIPPER_JOINTS,
    TrajectoryContractReason,
    validate_trajectory_contract,
)

__all__ = [
    "EndpointWorkspaceGate",
    "BackendKind",
    "ConstantVelocityPredictor",
    "DEFAULT_CLOCK_DOMAIN",
    "DEFAULT_TARGET_FRAME",
    "EdgeGraspController",
    "EvidencePhase",
    "GraspPhase",
    "GraspPhysicsEvidenceController",
    "GripperEffortSample",
    "GraspSequenceController",
    "MockBackend",
    "MotionPermissionGate",
    "PipelineState",
    "ROS_SIM_CLOCK_DOMAIN",
    "ROS_SYSTEM_CLOCK_DOMAIN",
    "ResultCode",
    "StaleTargetGate",
    "StateMachine",
    "SO101_ARM_ACTION",
    "SO101_ARM_JOINTS",
    "SO101_GRIPPER_ACTION",
    "SO101_GRIPPER_JOINTS",
    "Target3D",
    "TrajectoryContractReason",
    "Vector3",
    "create_backend",
    "ros_clock_domain",
    "validate_ros_clock_domain",
    "validate_trajectory_contract",
]
