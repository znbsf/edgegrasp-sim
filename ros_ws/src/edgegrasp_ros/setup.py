from glob import glob
from setuptools import find_packages, setup


PACKAGE_NAME = "edgegrasp_ros"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME + "/launch", glob("launch/*.launch.py")),
        (
            "share/" + PACKAGE_NAME + "/config",
            glob("config/*.yaml") + glob("config/*.json"),
        ),
        ("share/" + PACKAGE_NAME + "/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="EdgeGrasp contributors",
    maintainer_email="maintainers@example.invalid",
    description="ROS 2 adapters and probes for EdgeGrasp",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mock_target_publisher = edgegrasp_ros.mock_target_publisher:main",
            "safety_monitor = edgegrasp_ros.safety_monitor:main",
            "interface_probe = edgegrasp_ros.interface_probe:main",
            "planning_scene_loader = edgegrasp_ros.planning_scene_loader:main",
            "trajectory_gate = edgegrasp_ros.trajectory_gate:main",
            "grasp_physics_observer = edgegrasp_ros.grasp_physics_observer:main",
        ]
    },
)
