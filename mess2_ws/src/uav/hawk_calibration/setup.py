from setuptools import find_packages, setup

package_name = 'hawk_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marinarasauced',
    maintainer_email='marinarasauced@outlook.com',
    description='ROS2 package for calibrating ACE Lab Hawks in the VICON environment.',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'server = src.server:main'
        ],
    },
)
