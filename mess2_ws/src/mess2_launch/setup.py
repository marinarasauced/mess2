from glob import glob
import os.path

from setuptools import find_packages, setup

package_name = 'mess2_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marinarasauced',
    maintainer_email='marinarasauced@outlook.com',
    description='ROS2 Jazzy custom launch files for mess2 CSCP research.',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
