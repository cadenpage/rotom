from glob import glob
from setuptools import find_packages, setup

package_name = "rotom_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="caden",
    maintainer_email="ckp634@my.utexas.edu",
    description="MediaPipe teleoperation bridge for Rotom.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mediapipe_teleop = rotom_teleop.mediapipe_teleop_node:main",
        ],
    },
)
