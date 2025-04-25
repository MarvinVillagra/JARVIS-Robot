from setuptools import find_packages, setup

package_name = 'robot_pwm_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flackods3',
    maintainer_email='flackods3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
	    'motor_publisher = robot_pwm_test.motor_publisher:main',
            'motor_subscriber = robot_pwm_test.motor_subscriber:main',
	    'motor_controller = robot_pwm_test.motor_controller:main',
            'ps5_publisher = robot_pwm_test.ps5_publisher:main',

        ],
    },
)
