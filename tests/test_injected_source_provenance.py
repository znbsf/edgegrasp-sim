"""Exercise source provenance checks without importing ROS or starting nodes."""

import ast
import hashlib
import json
import os
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
INTEGRATION = ROOT / (
    "ros_ws/src/edgegrasp_moveit_adapter/test/test_moveit_adapter_integration.py"
)


@pytest.mark.parametrize("case", ["matching", "different", "missing", "tampered"])
def test_imported_source_must_match_declared_source(tmp_path, monkeypatch, case):
    expected = tmp_path / "expected.py"
    actual = tmp_path / "installed.py"
    expected.write_text("VERSION = 1\n", encoding="utf-8")
    actual.write_text(
        "VERSION = 2\n" if case == "different" else "VERSION = 1\n",
        encoding="utf-8",
    )
    monkeypatch.setenv("EDGEGRASP_INJECTED_EVENT_LOG", str(tmp_path / "scenarios.jsonl"))
    monkeypatch.setenv("EDGEGRASP_INJECTED_ADAPTER_SOURCE", str(expected))

    # Compile the actual fixture in isolation; do not load the ROS test module.
    tree = ast.parse(INTEGRATION.read_text(encoding="utf-8"))
    fixture = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "verify_injected_adapter_source"
    )
    fixture.decorator_list = []
    namespace = {
        "os": os, "Path": Path, "hashlib": hashlib, "json": json,
        "adapter_module": SimpleNamespace(__file__=str(actual)),
    }
    exec(compile(ast.Module(body=[fixture], type_ignores=[]), str(INTEGRATION), "exec"), namespace)
    if case == "different":
        with pytest.raises(AssertionError, match="Imported adapter differs"):
            namespace[fixture.name]()
    elif case != "missing":
        namespace[fixture.name]()

    record = tmp_path / "imported_source.json"
    if case == "tampered":
        data = json.loads(record.read_text(encoding="utf-8"))
        data["actual_sha256"] = "0" * 64
        record.write_text(json.dumps(data), encoding="utf-8")

    # Exercise the runner's independent summary admission check as written.
    runner = (ROOT / "scripts/run_injected_moveit_fail_closed.sh").read_text(encoding="utf-8")
    check = runner.split("# This record is written by pytest", 1)[1]
    check = check[check.index("imported_source_path ="):].split("\nartifact_paths =", 1)[0]
    summary = {
        "artifact": tmp_path, "json": json, "source_ok": True, "errors": [],
        "source": {"adapter": {"sha256": hashlib.sha256(expected.read_bytes()).hexdigest()}},
    }
    exec(compile(check, "runner_source_verification", "exec"), summary)
    assert summary["source_ok"] is (case == "matching")
    assert bool(summary["errors"]) is (case != "matching")
