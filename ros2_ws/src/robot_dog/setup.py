from setuptools import setup
import os
from glob import glob

package_name = "robot_dog"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sharan G S",
    maintainer_email="sharan@example.com",
    description="QuadBot-AI — LLM-powered quadruped robot dog",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sensor_publisher = robot_dog.sensor_publisher:main",
            "motor_controller = robot_dog.motor_controller_node:main",
            "ekf_node = robot_dog.ekf_node:main",
            "brain_node = robot_dog.brain_node:main",
        ],
    },
)
