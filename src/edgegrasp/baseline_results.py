"""Read-only reduction of existing logs, with separate historical denominators."""

from collections import Counter
import json

from edgegrasp.baseline import contained, raw_path, read_json, sha256


def json_lines(path, *, strict=False):
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            value = json.loads(line)
        except ValueError:
            if strict and line.strip():
                raise ValueError(f"malformed JSONL: {path}") from None
            continue
        if isinstance(value, dict):
            rows.append(value)
    return rows


def last(rows, **fields):
    return next((r for r in reversed(rows) if all(r.get(k) == v for k, v in fields.items())), {})


def read_cycle(directory):
    trial = json_lines(directory / "trial.log")
    events = json_lines(directory / "release_cycle.jsonl", strict=True)
    result = last(trial, type="grasp_trial_result")
    terminal = next((r for r in reversed(events) if "status" in r), {})
    task = result.get("task_id")
    lift = (result["final_z_m"] - result["baseline_z_m"]
            if all(isinstance(result.get(k), (float, int)) for k in ("final_z_m", "baseline_z_m")) else None)
    grasp = (result.get("physics_grasp_verified") is True
             and result.get("retention_observed") is True
             and result.get("all_gripper_tokens_observed") is True
             and result.get("simultaneous_gripper_contact_sample_count", 0) > 0
             and lift is not None and lift >= .02)
    support = {s: any(e.get("stage") == s and e.get("passed") is True for e in events)
               for s in ("supported_before_release", "supported_and_open", "supported_after_retreat")}
    geometry = last(events, stage="regrasp_geometry").get("result", {}).get("preclose_geometry_validated") is True
    handoff = last(events, stage="completed_cycle_handoff").get("result", {})
    typed = {}
    for stage in ("place", "release", "retreat"):
        item = last(events, stage=stage).get("result", {})
        typed[stage] = (bool(task) and item.get("success") is True
                        and item.get("downstream_terminal_observed") is True
                        and item.get("fjt_error_code") == 0
                        and str(item.get("command_id", "")).startswith(task + "|"))
    ready = last(events, stage="ready")
    errors = ready.get("scene_errors_m", [])
    ready_pass = len(errors) == 3 and all(isinstance(x, (int, float)) and 0 <= x <= .001 for x in errors)
    exit_path = directory / "gripper_release_exit_code.txt"
    release_exit = exit_path.read_text().strip() if exit_path.is_file() else None
    complete_label = terminal.get("status") == "PLACE_RELEASE_RETREAT_COMPLETE"
    complete = (grasp and complete_label and all(support.values()) and all(typed.values())
                and geometry and ready_pass and handoff.get("success") is True
                and handoff.get("epoch_changed") is False and release_exit == "0")
    reason = terminal.get("reason")
    feedback = next((r for r in reversed(trial) if r.get("type") == "sequence_feedback"), {})
    if not reason and not complete:
        if complete_label and not geometry:
            reason = "historical_complete_without_regrasp_geometry"
        elif result and not grasp:
            reason = result.get("sequence_reason") or result.get("physics_reason") or "grasp_not_verified"
        elif not result:
            # Keep the exact terminal error when no action result was produced.
            text = (directory / "trial.log").read_text(errors="replace") if (directory / "trial.log").is_file() else ""
            reason = next((line.strip() for line in reversed(text.splitlines())
                           if any(token in line for token in ("Error:", "Exception:", "FAILED", "timeout", "exhausted"))), "no_trial_result")
        else:
            reason = "independent_cycle_evidence_incomplete"
    outcome = ("verified" if complete else "historical_complete_missing_current_geometry"
               if complete_label and not geometry else "not_verified" if complete_label else "failed")
    failure_stage = None if complete else (str(reason).split(":")[0] if terminal.get("reason") else feedback.get("phase", "admission_or_evidence"))
    if complete_label and not geometry:
        failure_stage = "regrasp_geometry_not_recorded"
    return {"task_id": task, "grasp_success": grasp, "complete_label": complete_label,
            "outcome": outcome,
            "full_cycle_success": complete, "failure_reason": reason,
            "failure_stage": failure_stage,
            "regrasp_geometry_verified": geometry, "support_stages": support,
            "typed_terminals": typed, "ready_geometry_center_pass": ready_pass,
            "retention_observed": result.get("retention_observed") is True,
            "lift_m": lift, "simultaneous_contact_samples": result.get("simultaneous_gripper_contact_sample_count", 0),
            "release_exit": release_exit, "source_directory_exists": directory.is_dir(),
            "physics_baseline_renewals": sum(r.get("type") == "physics_baseline_restart" for r in trial),
            "fallback": (directory / "gripper_release_fallback_status.txt").read_text().strip()
            if (directory / "gripper_release_fallback_status.txt").is_file() else None}


def evidence_hashes(directory, expected):
    checks = []
    for filename, digest in expected.items():
        path = contained(directory, filename)
        actual = sha256(path) if path.is_file() else None
        checks.append({"file": str(path), "expected_sha256": digest, "actual_sha256": actual,
                       "ok": actual == digest})
    return checks


def build_report(repo, evidence_root):
    observations = repo / "docs/observations"
    names = ["2026-09-06-release-cycle-iteration.json", "2026-09-06-release-cycle-attempt.json",
             "2026-09-06-rgbd-final-validation.json", "2026-09-06-rgbd-artifact-inventory.json"]
    ledger, first, rgbd, inventory = [read_json(observations / n) for n in names]
    config_path = repo / "config/release_cycle_baseline.json"
    lock_path = repo / "config/release_cycle_baseline.lock.json"
    baseline_reference = ({"baseline_id": read_json(config_path)["baseline_id"],
                           "config_sha256": sha256(config_path), "lock_sha256": sha256(lock_path),
                           "planned_evaluation": read_json(config_path)["cases"]}
                          if config_path.is_file() and lock_path.is_file() else None)
    rows, hashes, groups = [], [], []

    def add_cycle(entry, candidate, cycle, group=None):
        directory = raw_path(evidence_root, entry["source_directory"])
        expected = {k.removesuffix("_sha256"): v for k, v in entry.items() if k.endswith("_sha256")}
        checks = evidence_hashes(directory, expected)
        hashes.extend(checks)
        row = read_cycle(directory)
        disagreements = [key for key, actual in (("physics_grasp_verified", row["grasp_success"]),
                                                   ("regrasp_geometry_verified", row["regrasp_geometry_verified"]))
                         if key in entry and entry[key] != actual]
        row.update(id=f"release-{candidate}-{cycle}", cohort="release_iteration", candidate=candidate,
                   cycle=cycle, group=group, position="control", state="attempted",
                   attempt_kind="initial" if candidate == "a" else "repair_retest",
                   version="AM source lock" if candidate in ("al", "am") else "historical candidate " + candidate,
                   source_directory=entry["source_directory"], ledger_entry=entry,
                   ledger_disagreements=disagreements,
                   evidence_integrity_ok=bool(checks) and all(c["ok"] for c in checks))
        if not row["evidence_integrity_ok"] or disagreements:
            row["full_cycle_success"] = False
            row["grasp_success"] = False
            row["failure_reason"] = "evidence_missing_hash_mismatch_or_ledger_disagreement"
            row["failure_stage"] = "evidence_integrity"
            row["outcome"] = "evidence_unavailable"
        rows.append(row)

    for entry in ledger["executions"]:
        add_cycle(entry, entry["candidate"], 1)
    for group in ledger["groups"]:
        candidate, summary = group["candidate"], group["summary"]
        if len(group["cycles"]) != summary["attempted"] or not 0 <= summary["attempted"] <= summary["requested"]:
            raise ValueError("group attempted count disagrees with cycle entries")
        for i, entry in enumerate(group["cycles"], 1):
            add_cycle(entry, candidate, i, candidate)
        for i in range(summary["attempted"] + 1, summary["requested"] + 1):
            rows.append(dict(id=f"release-{candidate}-{i}", cohort="release_iteration", position="control",
                             candidate=candidate, group=candidate, cycle=i, state="not_executed",
                             attempt_kind="repair_retest", failure_reason="group_stopped_before_this_cycle",
                             grasp_success=False, full_cycle_success=False))
        selected = [r for r in rows if r.get("group") == candidate]
        groups.append({"candidate": candidate, "recorded_summary": summary,
                       "planned": summary["requested"], "attempted": summary["attempted"],
                       "not_executed": summary["requested"] - summary["attempted"],
                       "independent_full_cycle_successes": sum(r["full_cycle_success"] for r in selected),
                       "grasp_successes": sum(r["grasp_success"] for r in selected)})

    plans = list(ledger["planning_attempts"])
    plans.insert(0, {"candidate": "a", "source_directory": "/home/edgegrasp/ros2_ws/test_results/" + first["plan"]["run"],
                     "status": first["plan"]["status"], "result_sha256": first["plan"]["sha256"]})
    for plan in plans:
        directory = raw_path(evidence_root, plan["source_directory"])
        hashes.extend(evidence_hashes(directory, {"plan_only_result.json": plan["result_sha256"]}))
        rows.append(dict(id="plan-" + plan["candidate"], cohort="release_planning", state="plan_only",
                         failure_stage="planning" if plan["status"] != "PLAN_ONLY_PASS" else None,
                         recorded_plan=plan, grasp_success=False, full_cycle_success=False))
    for failure in ledger["bootstrap_failures"]:
        rows.append(dict(id="bootstrap-" + failure["candidate"], cohort="bootstrap", state="not_executed",
                         failure_stage="bootstrap", recorded_failure=failure, grasp_success=False, full_cycle_success=False))
    for candidate in ledger["kinematic_only_not_executed"]:
        rows.append(dict(id="kinematic-" + candidate, cohort="kinematic_only", state="not_executed",
                         grasp_success=False, full_cycle_success=False))

    # Retain every earlier inventory item, including failures before artifact/client startup.
    legacy_positions = {p["artifact_dir"]: p for p in rgbd["positions"]}
    for artifact in inventory["artifacts"]:
        directory = raw_path(evidence_root, artifact["artifact_dir"])
        legacy_checks = evidence_hashes(directory, {k: v["sha256"] for k, v in artifact["key_evidence"].items()})
        hashes.extend(legacy_checks)
        if "_trial_" not in directory.name:
            plan_file = directory / "plan_only_result.json"
            plan = read_json(plan_file) if plan_file.is_file() else None
            rows.append(dict(id=directory.name, cohort="legacy_planning" if plan else "legacy_inventory",
                             state="plan_only" if plan else "inventory_only",
                             source_directory=artifact["artifact_dir"], recorded_plan_status=plan.get("status") if plan else None,
                             failure_stage="planning" if plan and plan.get("status") != "PLAN_ONLY_PASS" else None,
                             grasp_success=False, full_cycle_success=False,
                             evidence_integrity_ok=all(c["ok"] for c in legacy_checks)))
            continue
        row = read_cycle(directory)
        known = legacy_positions.get(artifact["artifact_dir"])
        if known:
            known_checks = evidence_hashes(directory, known["hashes"])
            hashes.extend(known_checks)
            legacy_checks += known_checks
        if not all(c["ok"] for c in legacy_checks):
            row["grasp_success"] = False
            row["outcome"] = "evidence_unavailable"
            row["failure_reason"] = "evidence_hash_mismatch"
        row.update(id=directory.name, cohort="legacy_grasp", state="attempted" if (directory / "trial.log").is_file() else "bootstrap_or_preflight",
                   position=known["position"] if known else ("x_minus_1mm" if "x_minus" in directory.name else "x_plus_1mm" if "x_plus" in directory.name else "historical_control_variant"),
                   source_directory=artifact["artifact_dir"], full_cycle_success=False,
                   full_cycle_assessment="not_in_full_cycle_protocol", attempt_kind="historical_separate_protocol")
        rows.append(row)

    attempted = [r for r in rows if r["cohort"] == "release_iteration" and r["state"] == "attempted"]
    return {"schema_version": 1, "claim": "offline_reduction_of_recorded_simulation_only",
            "baseline_reference": baseline_reference,
            "physics_recomputed": False, "new_experiments": 0,
            "source_ledgers": [{"path": "docs/observations/" + n, "sha256": sha256(observations / n)} for n in names],
            "evidence_root": str(evidence_root), "hash_checks": hashes,
            "integrity_ok": all(c["ok"] for c in hashes) and not any(r.get("ledger_disagreements") for r in rows),
            "rows": rows, "groups": groups, "legacy_inventory": inventory,
            "initial_attempt_corrections": first.get("correction"),
            "summary": {"release_attempted": len(attempted),
                        "release_grasp_successes": sum(r["grasp_success"] for r in attempted),
                        "release_full_cycle_successes_current_criteria": sum(r["full_cycle_success"] for r in attempted),
                        "release_not_successful_current_criteria": sum(not r["full_cycle_success"] for r in attempted),
                        "release_historical_complete_missing_current_geometry": sum(r["outcome"] == "historical_complete_missing_current_geometry" for r in attempted),
                        "release_recorded_failed": sum(r["outcome"] == "failed" for r in attempted),
                        "release_not_executed": sum(r["cohort"] == "release_iteration" and r["state"] == "not_executed" for r in rows),
                        "failure_stages": dict(Counter(r["failure_stage"] for r in attempted if not r["full_cycle_success"])),
                        "planning_attempted": len(plans), "planning_rejected": sum(p["status"] != "PLAN_ONLY_PASS" for p in plans),
                        "legacy_inventory_count": len(inventory["artifacts"])},
            "denominators": {"release_attempted": "All entered cycles, including admission failures before motion; failures retained across repairs. Heterogeneous historical versions, not a baseline success rate.",
                             "not_executed": "Declared later cycles stopped before entry; separate planned denominator, never counted as attempted success/failure.",
                             "planning": "Every recorded plan including rejection; separate from physical attempts.",
                             "legacy": "Earlier grasp-only protocol and preflight attempts kept separately; no claim of current full-cycle evaluation.",
                             "am": "Only the three cycles in one AM group; no pooling with AL or earlier repairs."},
            "claims": {"hardware_verified": False, "accepted_goal_result_timeout_verified": False,
                       "strong_move_group_request_id_correlation": False}}


def markdown_report(report):
    lines = ["# 既有基线证据汇总", "", "只读归约已有日志；未重算物理、未运行新实验。", "",
             f"原始文件哈希检查：{sum(c['ok'] for c in report['hash_checks'])}/{len(report['hash_checks'])}。", "",
             "历史版本不同，以下总数不是 AM 基线成功率；旧失败和未执行轮次全部保留。", "",
             "| 组 | 计划 | 尝试 | 独立抓取通过 | 当前完整判据通过 | 未执行 |",
             "|---|---:|---:|---:|---:|---:|"]
    for g in report["groups"]:
        lines.append(f"| {g['candidate'].upper()} | {g['planned']} | {g['attempted']} | {g['grasp_successes']} | {g['independent_full_cycle_successes']} | {g['not_executed']} |")
    lines += ["", "| 记录 | 状态 | 抓取通过 | 完整循环通过 | 原因 |", "|---|---|---|---|---|"]
    for r in report["rows"]:
        reason = str(r.get("failure_reason") or r.get("failure_stage") or "").replace("|", "/").replace("\n", " ")
        lines.append(f"| {r['id']} | {r['state']} | {r['grasp_success']} | {r['full_cycle_success']} | {reason} |")
    lines += ["", "分母定义、逐项哈希、原账本条目与 41 目录历史索引见 report.json。",
              "X±1 mm 的早期抓取成功单独保留；两位置完整循环未执行、未准备好。", ""]
    return "\n".join(lines)
