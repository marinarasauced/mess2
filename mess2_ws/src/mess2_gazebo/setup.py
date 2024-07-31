from glob import glob
import os.path

from setuptools import find_packages, setup

package_name = 'mess2_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.*'))),
        (os.path.join('share', package_name, 'models/ugv/sdf'), glob(os.path.join('models/ugv/sdf', '*.*'))),
        (os.path.join('share', package_name, 'models/ugv/meshes/bases'), glob(os.path.join('models/ugv/meshes/bases', '*.*'))),
        (os.path.join('share', package_name, 'models/ugv/meshes/sensors'), glob(os.path.join('models/ugv/meshes/sensors', '*.*'))),
        (os.path.join('share', package_name, 'models/ugv/meshes/wheels'), glob(os.path.join('models/ugv/meshes/wheels', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marinarasauced',
    maintainer_email='marinarasauced@outlook.com',
    description='ROS2 Jazzy custom launch files for starting CSCP Gazebo Harmonic simulations.',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'ugv_republisher = src.ugv_republisher:main'
        ],
    },
)
