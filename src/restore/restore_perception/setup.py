from glob import glob

from setuptools import find_packages, setup

package_name = "restore_perception"

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
    description="Scene perception: object poses as scene snapshots",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scene_perception_node = restore_perception.node:main",
            "offline_eval = restore_perception.offline_eval:main",
        ],
    },
)
