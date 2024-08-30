from setuptools import find_packages, setup

package_name = 'mess2_visualizer_py'

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
    maintainer_email='mjnelson@wpi.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'threat_annotator = src.threat_annotator:main'
        ],
    },
)
