import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OBSERVATION = (
    ROOT
    / "docs"
    / "observations"
    / "2026-08-27-candidate010-q0p45-runtime.json"
)
CANDIDATE = (
    ROOT
    / "ros_ws"
    / "src"
    / "edgegrasp_ros"
    / "config"
    / "so101_side_grasp_candidate.json"
)


def test_candidate010_sequence_complete_but_physics_fails_closed() -> None:
    report = json.loads(OBSERVATION.read_text(encoding="utf-8"))
    assert report["evidence_status"] == (
        "SEQUENCE_COMPLETE_IMPROVED_TRANSIENT_CONTACT_PHYSICS_GRASP_FAILED"
    )
    assert report["sequence_result"]["sequence_completed"] is True
    assert report["sequence_result"]["wrapper_status"] == 4
    assert report["sequence_result"]["client_exit_code"] == 11
    physics = report["physics_result"]
    assert physics["physics_grasp_verified"] is False
    assert physics["phase"] == "FAULT"
    assert physics["reason"] == "observation_wall_timeout"
    assert physics["all_gripper_tokens_observed"] is True
    assert physics["simultaneous_gripper_contact_sample_count"] == 144
    assert physics["peak_lift_m"] < physics["required_lift_m"]
    assert physics["retention_observed"] is False
    assert report["runtime_health"]["cleanup_exact_domain_processes_remaining"] == 0
    assert report["claim_boundary"]["physics_grasp_verified"] is False


def test_candidate010_improves_transients_without_promoting_claim() -> None:
    comparison = json.loads(OBSERVATION.read_text(encoding="utf-8"))[
        "comparison_to_candidate008"
    ]
    assert comparison["simultaneous_sample_count"][
        "candidate010_over_candidate008_ratio"
    ] > 1.0
    assert comparison["simultaneous_span_ns"][
        "candidate010_over_candidate008_ratio"
    ] > 1.0
    assert comparison["peak_lift_m"]["candidate010_over_candidate008_ratio"] > 1.0
    assert comparison["approximate_final_xy_drift_m"][
        "candidate010_over_candidate008_ratio"
    ] < 1.0
    assert comparison["physics_grasp_verified"] == {
        "candidate008": False,
        "candidate010": False,
    }


def test_candidate_contract_links_candidate010_runtime() -> None:
    candidate = json.loads(CANDIDATE.read_text(encoding="utf-8"))
    runtime = candidate["candidate010_q0p45_runtime_observation"]
    assert candidate["schema_version"] == 14
    assert runtime["path"].endswith("candidate010-q0p45-runtime.json")
    assert runtime["sequence_completed"] is True
    assert runtime["physics_grasp_verified"] is False
    assert runtime["peak_lift_m"] < runtime["required_lift_m"]
