from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "edgegrasp_grasp_sequence"

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
    description="Correlated fail-closed SO-101 grasp-sequence action wrapper",
    license="MIT",
    entry_points={
        "console_scripts": [
            "grasp_sequence = edgegrasp_grasp_sequence.node:main",
            "grasp_sequence_client = edgegrasp_grasp_sequence.client:main",
            "grasp_trial_client = edgegrasp_grasp_sequence.trial_client:main",
        ]
    },
)
