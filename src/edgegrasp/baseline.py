"""Offline preparation of the recorded AM baseline. No ROS imports or execution."""

import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import shlex
import xml.etree.ElementTree as ET


RAW_ROOT = "/home/edgegrasp/ros2_ws/test_results"
ENVIRONMENT = {
    "EDGEGRASP_RGBD_TRIAL": "1",
    "EDGEGRASP_RELEASE_CYCLE": "1",
    "EDGEGRASP_BOUNDED_CONTROL_PAN": "1",
    "EDGEGRASP_RELEASE_CYCLE_COUNT": "3",
    "EDGEGRASP_RGBD_ROTATION_BOUND_DEG": "40.0",
    "EDGEGRASP_RGBD_CAMERA_HZ": "30",
    "EDGEGRASP_RGBD_CAMERA_VIEW": "opposite_table_edge",
    "EDGEGRASP_RGBD_CAMERA_RESOLUTION": "upstream",
    "EDGEGRASP_RGBD_VELOCITY_SCALING": "0.15",
    "EDGEGRASP_RGBD_MOTION_MODE": "measured_pad",
    "EDGEGRASP_OBSERVATION_TIMEOUT_S": "30.0",
}
SEGMENTS = ["home_to_approach", "approach_to_descend", "descend_to_lift_closed",
            "lift_to_place_closed", "place_to_retreat_open"]


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def contained(root, relative):
    """Reject traversal, Windows drive paths, and symlink escapes on either OS."""
    root = Path(root).resolve()
    rel = PurePosixPath(relative)
    if not relative or rel.is_absolute() or ".." in rel.parts or ":" in relative or "\\" in relative:
        raise ValueError(f"unsafe relative path: {relative}")
    result = (root / relative).resolve()
    if not result.is_relative_to(root) or result == root:
        raise ValueError(f"path escapes root: {relative}")
    return result


def raw_path(root, recorded):
    prefix = RAW_ROOT + "/"
    if not recorded.startswith(prefix):
        raise ValueError(f"unsupported evidence path: {recorded}")
    return contained(root, recorded[len(prefix):])


def validate_config(config):
    if config.get("schema_version") != 1 or config.get("baseline_id") != "candidate024-am-20260906-v1":
        raise ValueError("unsupported baseline version")
    if config.get("environment") != ENVIRONMENT:
        raise ValueError("AM environment drift; a new baseline requires separate review")
    for name in ("accepted_goal_result_timeout_verified", "strong_move_group_request_id_correlation", "hardware_verified"):
        if config.get("claims", {}).get(name) is not False:
            raise ValueError(f"claim must remain false: {name}")
    cases = config.get("cases", [])
    if [c.get("id") for c in cases] != ["control", "x_minus_1mm", "x_plus_1mm"]:
        raise ValueError("expected exactly the three declared positions")
    for case, offset in zip(cases, (0.0, -0.001, 0.001)):
        if type(case.get("planned_cycles")) is not int or case["planned_cycles"] != 3:
            raise ValueError("each case must predeclare exactly three cycles")
        if case.get("offset_x_m") != offset:
            raise ValueError("position offset drift")
        if case["id"] != "control" and (case.get("plan") is not None or case.get("ready") is not False):
            raise ValueError("offset full-cycle configuration is not prepared; control plan reuse forbidden")
    control = cases[0]
    if control.get("ready") is not True or control.get("plan") != "release_cycle_plan_20260906aj/plan_only_result.json":
        raise ValueError("control requires the recorded AJ plan")
    if config.get("criteria_version") != "am-independent-geometry-v1":
        raise ValueError("unsupported independent criteria")


def verify_files(root, entries):
    checks = []
    for entry in entries:
        path = contained(root, entry["path"])
        actual = sha256(path) if path.is_file() else None
        checks.append({"path": entry["path"], "expected_sha256": entry["sha256"],
                       "actual_sha256": actual, "ok": actual == entry["sha256"]})
    return checks


def inspect_baseline(repo, config, lock, evidence_root=None, runtime_root=None):
    validate_config(config)
    if "config_sha256" in lock:
        actual = hashlib.sha256(json.dumps(config, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        if actual != lock["config_sha256"]:
            raise ValueError("configuration differs from the pinned baseline lock")
    checks = verify_files(repo, lock["repository_files"])
    result = {"mode": "offline_check", "motion_started": False,
              "baseline_id": config["baseline_id"], "repository_checks": checks,
              "repository_ok": all(c["ok"] for c in checks),
              "cases": config["cases"], "raw_plan": {"status": "not_checked"},
              "runtime": {"status": "not_checked"}, "claims": config["claims"]}
    if evidence_root is not None:
        path = contained(evidence_root, config["cases"][0]["plan"])
        if not path.is_file():
            result["raw_plan"] = {"status": "missing", "path": str(path)}
        else:
            plan = read_json(path)
            valid = (sha256(path) == lock["plan_sha256"]
                     and plan.get("status") == "PLAN_ONLY_PASS"
                     and plan.get("all_segments_accepted") is True
                     and plan.get("execution_attempted") is False
                     and all(plan.get(k) == 0 for k in ("trajectory_publication_count", "execute_trajectory_goal_count", "fjt_goal_count"))
                     and [s.get("segment") for s in plan.get("segments", [])] == SEGMENTS
                     and all(s.get("plan_accepted") is True for s in plan["segments"])
                     and plan["config"].get("bounded_control_pan") is True
                     and plan["config"].get("diagnostic_target_center_x_offset_m") == 0)
            result["raw_plan"] = {"status": "verified" if valid else "mismatch", "path": str(path), "sha256": sha256(path)}
    if runtime_root is not None:
        assets = verify_files(runtime_root, lock["runtime_files"])
        required = [{"path": p, "exists": contained(runtime_root, p).is_file()}
                    for p in config["runtime_required_paths"]]
        versions = []
        for name, expected in config.get("runtime_package_versions", {}).items():
            path = contained(runtime_root, f"opt/ros/jazzy/share/{name}/package.xml")
            actual = ET.parse(path).getroot().findtext("version") if path.is_file() else None
            versions.append({"package": name, "expected": expected, "actual": actual, "ok": actual == expected})
        result["runtime"] = {"status": "files_present" if all(c["ok"] for c in assets + versions) and all(c["exists"] for c in required) else "missing_or_mismatch",
                             "assets": assets, "required": required,
                             "package_versions": versions,
                             "live_health_verified": False,
                             "note": "File checks only; no ROS graph, controller, package import or motion probe."}
    result["command_preparation_ready"] = (result["repository_ok"]
                                            and result["raw_plan"]["status"] == "verified"
                                            and result["runtime"]["status"] == "files_present")
    return result


def render_command(config, inspection, *, case_id, domain, output, task_id, linux_repo):
    """Emit a guarded script; the caller never invokes it."""
    validate_config(config)
    if case_id != "control":
        raise ValueError("NOT_READY: offset-specific full-cycle plan, scene and release configuration missing")
    if not inspection["command_preparation_ready"]:
        raise ValueError("NOT_READY: matching sources, AJ plan and local dependency files required")
    if type(domain) is not int or not 1 <= domain <= 232:
        raise ValueError("domain must be an explicitly selected integer in [1, 232]")
    if not re.fullmatch(r"[a-zA-Z0-9._-]+", task_id):
        raise ValueError("invalid task ID")
    out = PurePosixPath(output)
    if out.parent != PurePosixPath(RAW_ROOT) or not re.fullmatch(r"[a-zA-Z0-9_-]+", out.name):
        raise ValueError("new experiment must be one named directory directly under the existing test_results root")
    if not linux_repo.startswith("/") or ".." in PurePosixPath(linux_repo).parts or "\n" in linux_repo:
        raise ValueError("linux_repo must be an absolute Linux path")
    quote = shlex.quote
    lines = ["#!/usr/bin/env bash", "# Generated offline. Review and obtain current scoped authorization before execution.",
             "set -eo pipefail", 'if [[ "${1:-}" != --execute-simulation || "$#" != 1 ]]; then',
             '  echo "Dry-run: no processes started. Explicit --execute-simulation required."', "  exit 0", "fi",
             f"cd {quote(linux_repo)}", f"test ! -e {quote(output)} || {{ echo 'Output already exists' >&2; exit 2; }}",
             "python3 scripts/baseline.py check --require-local-runtime --case control --evidence-root " + RAW_ROOT + " --runtime-root /",
             "# Remove inherited experiment overrides before setting this pinned baseline.",
             'for baseline_var in ${!EDGEGRASP_@}; do unset "$baseline_var"; done']
    lines += [f"export {key}={quote(value)}" for key, value in config["environment"].items()]
    lines += ["export EDGEGRASP_RGBD_PLAN_RESULT=" + quote(RAW_ROOT + "/" + config["cases"][0]["plan"])]
    args = ["bash", "scripts/run_candidate005_contact_quality.sh", str(domain), output, task_id,
            *config["runner_arguments"]]
    lines += [" ".join(map(quote, args)), ""]
    return "\n".join(lines)
