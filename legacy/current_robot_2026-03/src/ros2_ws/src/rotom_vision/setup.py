from glob import glob
from setuptools import setup

package_name = "rotom_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="caden",
    maintainer_email="caden@example.com",
    description="Stereo split, ArUco tracking, and visual-servo target generation for Rotom.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "stereo_splitter = rotom_vision.stereo_splitter_node:main",
            "aruco_tracker = rotom_vision.aruco_tracker_node:main",
            "marker_follower = rotom_vision.marker_follower_node:main",
            "visual_servo = rotom_vision.visual_servo_node:main",
            "drag_target_servo = rotom_vision.drag_target_servo_node:main",
        ],
    },
)
