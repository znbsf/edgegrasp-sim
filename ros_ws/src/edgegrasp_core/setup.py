import os
from pathlib import Path

from setuptools import find_packages, setup
from setuptools.command.develop import develop


PROJECT_ROOT = Path(__file__).resolve().parents[3]
CORE_SRC = PROJECT_ROOT / "src"
if not (CORE_SRC / "edgegrasp" / "__init__.py").is_file():
    raise RuntimeError(
        "edgegrasp_core expects ros_ws/src/edgegrasp_core inside the EdgeGrasp project"
    )

# ``setuptools develop`` (used by ``colcon build --symlink-install``) rejects
# an absolute package_dir outside the setup.py directory.  Keep the monorepo
# source single-copy, but express its path relative to the command working
# directory so both colcon's build directory and direct wheel builds work.
CORE_SRC_FROM_CWD = os.path.relpath(CORE_SRC, Path.cwd())


class MonorepoDevelop(develop):
    """Allow colcon's editable install to point at the root core source tree."""

    @staticmethod
    def _resolve_setup_path(egg_base, install_dir, egg_path):  # noqa: ARG004
        # setuptools assumes package_dir is below setup.py and otherwise rejects
        # the editable install.  Here egg_base deliberately is PROJECT_ROOT/src;
        # the setup.py used by colcon is symlinked into its package build folder.
        # The second .egg-link line must therefore lead from egg_base back to
        # that build folder.
        return os.path.relpath(Path.cwd(), Path(egg_base).resolve()).replace(
            os.sep, "/"
        )

setup(
    name="edgegrasp_core",
    version="0.1.0",
    packages=find_packages(where=CORE_SRC_FROM_CWD),
    package_dir={"": CORE_SRC_FROM_CWD},
    package_data={"edgegrasp.contracts": ["*.json"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/edgegrasp_core"]),
        ("share/edgegrasp_core", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="EdgeGrasp contributors",
    maintainer_email="maintainers@example.invalid",
    description="ROS-independent EdgeGrasp core",
    license="MIT",
    cmdclass={"develop": MonorepoDevelop},
)
