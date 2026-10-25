from glob import glob

from setuptools import find_packages, setup

package_name = "restore_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Oguzhan Esen",
    maintainer_email="ogguzhanesen@gmail.com",
    description="The orchestrator: scene diff, ordering, grasp poses and the pick-and-place state machine",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "restore_task = restore_task.node:main",
            "plan_offline = restore_task.plan_offline:main",
        ],
    },
)
