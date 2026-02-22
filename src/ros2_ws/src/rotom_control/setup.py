from setuptools import setup

package_name = "rotom_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/rotom_control.launch.py"]),
        ("share/" + package_name + "/config", ["config/rotom_control.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="caden",
    maintainer_email="caden@example.com",
    description="Minimal ROS2 bridge for Rotom motor control.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rotom_motor_bridge = rotom_control.ros2_motor_bridge:main",
        ],
    },
)
