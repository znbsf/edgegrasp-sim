import json
from pathlib import Path
import re


MANIFEST = Path(__file__).resolve().parents[1] / "docs" / "upstream-manifest.json"
REQUIRED_KEYS = {
    "repository",
    "commit",
    "license",
    "role",
    "source_url",
    "evidence_urls",
    "official_commands",
    "reproduction_steps",
    "known_limitations",
    "local_verification_status",
}


def test_manifest_is_valid_json_with_required_fields() -> None:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))

    assert data["schema_version"] == 1
    assert len(data["upstreams"]) == 3
    for upstream in data["upstreams"]:
        assert REQUIRED_KEYS <= upstream.keys()
        assert re.fullmatch(r"[0-9a-f]{40}", upstream["commit"])
        assert upstream["commit"] in upstream["source_url"]
        assert upstream["official_commands"]
        assert upstream["reproduction_steps"]
        assert all(
            {"cwd", "command"} <= step.keys()
            for step in upstream["reproduction_steps"]
        )
        assert upstream["known_limitations"]


def test_all_github_evidence_urls_are_commit_pinned() -> None:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))

    for upstream in data["upstreams"]:
        commit = upstream["commit"]
        for label, url in upstream["evidence_urls"].items():
            assert commit in url, f"{upstream['repository']}:{label} is not pinned"
            assert "/main/" not in url and "/master/" not in url


def test_expected_pins_and_licenses_are_exact() -> None:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    actual = {
        item["repository"]: (item["commit"], item["license"])
        for item in data["upstreams"]
    }

    assert actual == {
        "adoodevv/so101_ros2": (
            "0305e03ab54e64aae9263fcbf339622e654012f3",
            "BSD-3-Clause",
        ),
        "TheRobotStudio/SO-ARM100": (
            "7629d2ad9853d10fb903093a33ef6114099d97e5",
            "Apache-2.0",
        ),
        "legalaspro/so101-ros-physical-ai": (
            "58318c905a2c61289fa907de85cb8473322fbe68",
            "Apache-2.0",
        ),
    }


def test_feetech_submodule_has_direct_repository_and_commit_evidence() -> None:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    legalaspro = next(
        item
        for item in data["upstreams"]
        if item["repository"] == "legalaspro/so101-ros-physical-ai"
    )
    submodule = legalaspro["submodules"][0]

    assert submodule["repository_url"] == (
        "https://github.com/legalaspro/feetech_ros2_driver"
    )
    assert submodule["commit"] in submodule["commit_url"]


def test_local_and_remote_verification_layers_are_not_conflated() -> None:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    upstreams = {item["repository"]: item for item in data["upstreams"]}

    adoodevv = upstreams["adoodevv/so101_ros2"]["local_verification_status"]
    assert adoodevv["local_checkout"] is True
    assert adoodevv["local_checkout_static_verified"] is True
    assert adoodevv["runtime_unverified"] is False
    assert adoodevv["runtime_full_stack_verified"] is False
    assert adoodevv["ros_build_verified"] is True
    assert adoodevv["moveit_plan_runtime_verified"] is True
    assert adoodevv["edgegrasp_gated_execution_runtime_verified"] is True
    assert adoodevv["collision_and_grasp_runtime_verified"] is False

    legalaspro = upstreams["legalaspro/so101-ros-physical-ai"][
        "local_verification_status"
    ]
    assert legalaspro["remote_pinned_evidence_only"] is True
    assert legalaspro["local_checkout"] is False
    assert legalaspro["build_verified"] is False
    assert legalaspro["runtime_unverified"] is True


def test_retracted_moveit_claim_is_preserved_as_auditable_history() -> None:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    retraction = data["audit_history"][0]

    assert retraction["status"] == "retracted"
    assert retraction["commit"] == "0305e03ab54e64aae9263fcbf339622e654012f3"
    assert "complete-looking static MoveIt" in retraction["corrected_observation"]
    runtime = data["audit_history"][1]
    assert runtime["status"] == "partial_runtime_verified"
    assert "Collision fidelity" in runtime["observation"]
