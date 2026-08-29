"""Colcon smoke test for the monorepo-backed core package."""

from edgegrasp import DEFAULT_TARGET_FRAME, EndpointWorkspaceGate


def test_colcon_install_exposes_the_single_core_source_tree() -> None:
    assert DEFAULT_TARGET_FRAME == "base_link"
    assert EndpointWorkspaceGate.__module__ == "edgegrasp.planner"
