from glob import glob
from setuptools import find_packages, setup

package_name = "rotom_vision"

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
    description="Vision-side target generation utilities for Rotom.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "stereo_splitter = rotom_vision.stereo_splitter_node:main",
            "marker_follower = rotom_vision.marker_follower_node:main",
            "aruco_tracker = rotom_vision.aruco_tracker_node:main",
            "image_monitor = rotom_vision.image_monitor_node:main",
        ],
    },
)
