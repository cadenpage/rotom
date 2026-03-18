from setuptools import find_packages, setup

package_name = "rotom_servo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/twist_relay.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="caden",
    maintainer_email="ckp634@my.utexas.edu",
    description="Servo command source utilities for Rotom.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "twist_relay = rotom_servo.twist_relay_node:main",
        ],
    },
)
