from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "edgegrasp_moveit_adapter"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        ("share/" + PACKAGE_NAME + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="EdgeGrasp contributors",
    maintainer_email="maintainers@example.invalid",
    description="MoveIt plan-only to EdgeGrasp trajectory-gate adapter",
    license="MIT",
    entry_points={
        "console_scripts": [
            "moveit_adapter = edgegrasp_moveit_adapter.adapter_node:main",
            "plan_target_client = edgegrasp_moveit_adapter.plan_target_client:main",
        ]
    },
)
