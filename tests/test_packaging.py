from pathlib import Path
import os
import subprocess
import sys
import shutil


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CORE_AMENT_PACKAGE = PROJECT_ROOT / "ros_ws" / "src" / "edgegrasp_core"


def clean_environment() -> dict[str, str]:
    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    return environment


def test_standard_distribution_installs_and_imports_without_source_pythonpath(
    tmp_path: Path,
) -> None:
    target = tmp_path / "installed"
    install = subprocess.run(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "--no-deps",
            "--no-build-isolation",
            "--target",
            str(target),
            str(PROJECT_ROOT),
        ],
        cwd=tmp_path,
        env=clean_environment(),
        capture_output=True,
        text=True,
        timeout=120,
    )
    assert install.returncode == 0, install.stdout + install.stderr

    probe = subprocess.run(
        [
            sys.executable,
            "-I",
            "-c",
            (
                "import sys;"
                f"sys.path.insert(0, {str(target)!r});"
                "import edgegrasp;"
                "print(edgegrasp.DEFAULT_TARGET_FRAME);"
                "print(edgegrasp.SO101_ARM_ACTION)"
            ),
        ],
        cwd=tmp_path,
        env=clean_environment(),
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert probe.returncode == 0, probe.stdout + probe.stderr
    assert probe.stdout.splitlines() == [
        "base_link",
        "/arm_controller/follow_joint_trajectory",
    ]


def test_ament_core_package_resolves_the_single_core_source_tree() -> None:
    probe = subprocess.run(
        [sys.executable, "setup.py", "--name"],
        cwd=CORE_AMENT_PACKAGE,
        env=clean_environment(),
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert probe.returncode == 0, probe.stdout + probe.stderr
    assert probe.stdout.strip() == "edgegrasp_core"


def test_ament_core_monorepo_adapter_rejects_isolated_subpackage_copy(
    tmp_path: Path,
) -> None:
    isolated = tmp_path / "edgegrasp_core"
    shutil.copytree(CORE_AMENT_PACKAGE, isolated)

    probe = subprocess.run(
        [sys.executable, "setup.py", "--name"],
        cwd=isolated,
        env=clean_environment(),
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert probe.returncode != 0
    assert "expects ros_ws/src/edgegrasp_core inside the EdgeGrasp project" in (
        probe.stdout + probe.stderr
    )


def test_ament_core_metadata_resolves_from_a_copied_complete_project_layout(
    tmp_path: Path,
) -> None:
    copied_project = tmp_path / "edgegrasp-sim"
    shutil.copytree(
        PROJECT_ROOT,
        copied_project,
        ignore=shutil.ignore_patterns(
            ".git",
            ".venv",
            ".pytest_cache",
            "__pycache__",
            "*.pyc",
            "build",
            "install",
            "log",
        ),
    )
    copied_adapter = copied_project / "ros_ws" / "src" / "edgegrasp_core"
    probe = subprocess.run(
        [sys.executable, "setup.py", "--name"],
        cwd=copied_adapter,
        env=clean_environment(),
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert probe.returncode == 0, probe.stdout + probe.stderr
    assert probe.stdout.strip() == "edgegrasp_core"
    assert (copied_project / "src" / "edgegrasp" / "controller.py").is_file()
