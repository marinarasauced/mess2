from glob import glob
from os import path
from setuptools import find_packages, setup

package_name = "mess2_gazebo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (path.join('share', package_name, 'launch'), glob(path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="marinarasauced",
    maintainer_email="marinarasauced@outlook.com",
    description="ROS2 custom package for publishing fake TurtleBot3 data for ACE Lab TurtleBot3 unmanned ground vehicles.",
    license="Apache 2.0",
    entry_points={
        "console_scripts": [
            "turtlebot3_fake = src.turtlebot3_fake:main",
        ],
    },
)
