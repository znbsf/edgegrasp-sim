"""Offline preparation must reject drift and keep motion opt-in in emitted commands."""

import json
from pathlib import Path

import pytest

from edgegrasp.baseline import (
    contained, inspect_baseline, read_json, render_command, sha256, validate_config,
)
from edgegrasp.baseline_results import read_cycle


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def config():
    return read_json(ROOT / "config/release_cycle_baseline.json")


@pytest.mark.parametrize("change", [
    lambda c: c["environment"].update(EDGEGRASP_BOUNDED_CONTROL_PAN="0"),
    lambda c: c["environment"].update(EDGEGRASP_EXTRA_OVERRIDE="1"),
    lambda c: c["claims"].update(hardware_verified=True),
    lambda c: c["claims"].update(accepted_goal_result_timeout_verified=True),
    lambda c: c["claims"].update(strong_move_group_request_id_correlation=True),
    lambda c: c["cases"][1].update(plan=c["cases"][0]["plan"]),
    lambda c: c["cases"][2].update(ready=True),
    lambda c: c["cases"][0].update(planned_cycles=True),
    lambda c: c["cases"][0].update(planned_cycles=4),
    lambda c: c["cases"][0].update(offset_x_m=.001),
])
def test_bad_config_rejected_before_preparation(config, change):
    change(config)
    with pytest.raises(ValueError):
        validate_config(config)


@pytest.mark.parametrize("path", ["../old", "/tmp/old", "C:/old", "a/../../old", "a\\old", ""])
def test_evidence_paths_cannot_escape_root(tmp_path, path):
    with pytest.raises(ValueError):
        contained(tmp_path, path)


def test_default_inspection_has_no_runtime_or_evidence_access(config):
    lock = read_json(ROOT / "config/release_cycle_baseline.lock.json")
    result = inspect_baseline(ROOT, config, lock)
    assert result["repository_ok"]
    assert result["raw_plan"]["status"] == "not_checked"
    assert result["runtime"]["status"] == "not_checked"
    assert not result["command_preparation_ready"]
    assert not result["motion_started"]


def test_changed_criteria_cannot_reuse_baseline_lock(config):
    config["criteria"]["ready"]["center_per_axis_limit_m"] = .002
    lock = read_json(ROOT / "config/release_cycle_baseline.lock.json")
    with pytest.raises(ValueError, match="configuration differs"):
        inspect_baseline(ROOT, config, lock)


def test_tampered_source_prevents_command_preparation(config, tmp_path):
    source = tmp_path / "runtime.py"
    source.write_text("original")
    lock = {"repository_files": [{"path": "runtime.py", "sha256": sha256(source)}]}
    source.write_text("changed")
    result = inspect_baseline(tmp_path, config, lock)
    assert not result["repository_ok"]
    with pytest.raises(ValueError, match="NOT_READY"):
        render_command(config, result, case_id="control", domain=227,
                       output="/home/edgegrasp/ros2_ws/test_results/new_trial", task_id="new-trial", linux_repo="/repo")


def test_offset_cannot_render_control_command(config):
    with pytest.raises(ValueError, match="offset-specific"):
        render_command(config, {"command_preparation_ready": True}, case_id="x_minus_1mm", domain=227,
                       output="/home/edgegrasp/ros2_ws/test_results/new_trial", task_id="new-trial", linux_repo="/repo")


def test_emitted_command_defaults_to_no_motion_and_rechecks(config):
    script = render_command(config, {"command_preparation_ready": True}, case_id="control", domain=227,
                            output="/home/edgegrasp/ros2_ws/test_results/new_trial", task_id="new-trial", linux_repo="/repo with space")
    assert script.index("exit 0") < script.index("cd '/repo with space'")
    assert script.index("--require-local-runtime") < script.index("run_candidate005_contact_quality.sh")
    assert "--execute-simulation" in script
    assert "EDGEGRASP_RELEASE_CYCLE_COUNT=3" in script
    assert "unset" in script


def write_recorded_cycle(directory, *, geometry=True, wrong_task=False, independent=True):
    directory.mkdir(exist_ok=True)
    result = dict(type="grasp_trial_result", task_id="one", physics_grasp_verified=independent,
                  retention_observed=True, all_gripper_tokens_observed=True,
                  simultaneous_gripper_contact_sample_count=10, baseline_z_m=.205, final_z_m=.234)
    events = []
    for stage in ("place", "release", "retreat"):
        events.append(dict(stage=stage, result=dict(success=True, downstream_terminal_observed=True,
                                                    fjt_error_code=0, command_id=("other" if wrong_task else "one") + "|" + stage)))
    events += [dict(stage=s, passed=True) for s in ("supported_before_release", "supported_and_open", "supported_after_retreat")]
    events += [dict(stage="ready", scene_errors_m=[.0005, .0005, 0]),
               dict(stage="completed_cycle_handoff", result=dict(success=True, epoch_changed=False))]
    if geometry:
        events.append(dict(stage="regrasp_geometry", result=dict(preclose_geometry_validated=True)))
    events.append(dict(status="PLACE_RELEASE_RETREAT_COMPLETE"))
    (directory / "trial.log").write_text(json.dumps(result) + "\n")
    (directory / "release_cycle.jsonl").write_text("\n".join(map(json.dumps, events)) + "\n")
    (directory / "gripper_release_exit_code.txt").write_text("0\n")


def test_complete_label_cannot_replace_independent_grasp(tmp_path):
    write_recorded_cycle(tmp_path, independent=False)
    result = read_cycle(tmp_path)
    assert result["complete_label"]
    assert not result["grasp_success"]
    assert not result["full_cycle_success"]


def test_old_complete_kept_without_promoting_missing_geometry(tmp_path):
    write_recorded_cycle(tmp_path, geometry=False)
    result = read_cycle(tmp_path)
    assert result["grasp_success"]
    assert result["complete_label"]
    assert not result["full_cycle_success"]
    assert result["outcome"] == "historical_complete_missing_current_geometry"


def test_unrelated_terminal_cannot_complete_current_cycle(tmp_path):
    write_recorded_cycle(tmp_path, wrong_task=True)
    result = read_cycle(tmp_path)
    assert result["grasp_success"]
    assert not result["full_cycle_success"]


def test_missing_exit_code_does_not_become_zero(tmp_path):
    write_recorded_cycle(tmp_path)
    assert read_cycle(tmp_path)["full_cycle_success"]
    (tmp_path / "gripper_release_exit_code.txt").unlink()
    assert not read_cycle(tmp_path)["full_cycle_success"]


def test_corrupt_jsonl_is_not_silently_ignored(tmp_path):
    write_recorded_cycle(tmp_path)
    with (tmp_path / "release_cycle.jsonl").open("a") as stream:
        stream.write("{corrupted}\n")
    with pytest.raises(ValueError, match="malformed JSONL"):
        read_cycle(tmp_path)


def test_summary_keeps_failures_unexecuted_cycles_and_retests(tmp_path):
    from edgegrasp.baseline_results import build_report

    evidence = tmp_path / "evidence"
    evidence.mkdir()
    write_recorded_cycle(evidence / "old", geometry=False)
    write_recorded_cycle(evidence / "new", independent=False)
    prefix = "/home/edgegrasp/ros2_ws/test_results/"

    def entry(name):
        return {"source_directory": prefix + name,
                "trial.log_sha256": sha256(evidence / name / "trial.log"),
                "release_cycle.jsonl_sha256": sha256(evidence / name / "release_cycle.jsonl")}

    observations = tmp_path / "repo/docs/observations"
    observations.mkdir(parents=True)
    (evidence / "plan").mkdir()
    (evidence / "plan/plan_only_result.json").write_text('{"status":"PLAN_ONLY_REJECTED"}')
    ledger = dict(executions=[dict(candidate="a", **entry("old"))],
                  groups=[dict(candidate="b", summary=dict(requested=3, attempted=1), cycles=[entry("new")])],
                  planning_attempts=[], bootstrap_failures=[dict(candidate="c", error="bad quote")],
                  kinematic_only_not_executed=["d"])
    data = {"2026-09-06-release-cycle-iteration.json": ledger,
            "2026-09-06-release-cycle-attempt.json": dict(plan=dict(run="plan", status="PLAN_ONLY_REJECTED", sha256=sha256(evidence / "plan/plan_only_result.json"))),
            "2026-09-06-rgbd-final-validation.json": dict(positions=[]),
            "2026-09-06-rgbd-artifact-inventory.json": dict(artifacts=[dict(
                artifact_dir=prefix + "plan", key_evidence={"plan_only_result.json": {
                    "sha256": sha256(evidence / "plan/plan_only_result.json")}})])}
    for name, value in data.items():
        (observations / name).write_text(json.dumps(value))
    report = build_report(tmp_path / "repo", evidence)
    assert report["integrity_ok"]
    assert report["summary"]["release_attempted"] == 2
    assert report["summary"]["release_not_executed"] == 2
    assert report["summary"]["planning_rejected"] == 1
    assert report["summary"]["release_historical_complete_missing_current_geometry"] == 1
    assert report["summary"]["release_full_cycle_successes_current_criteria"] == 0
    assert next(r for r in report["rows"] if r["id"] == "plan")["recorded_plan_status"] == "PLAN_ONLY_REJECTED"
    assert {r.get("attempt_kind") for r in report["rows"]} >= {"initial", "repair_retest"}
    (evidence / "old/trial.log").write_text("tampered")
    changed = build_report(tmp_path / "repo", evidence)
    assert not changed["integrity_ok"]
    assert next(r for r in changed["rows"] if r["id"] == "release-a-1")["grasp_success"] is False
