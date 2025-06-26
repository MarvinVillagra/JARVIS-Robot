from setuptools import setup
import os
from glob import glob

package_name = 'jarvis_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marvin',
    maintainer_email='marvinvillagra@gmail.com',
    description='Robot control package for JARVIS using PS5 controller and ROS 2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ps5_publisher = jarvis_control.ps5_publisher:main',
            'motor_controller = jarvis_control.motor_controller:main',
            'camera_publisher = jarvis_control.camera_publisher:main',
        ],
    },
)
