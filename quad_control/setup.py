from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'quad_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'controllers'), glob('controllers/*')),
        ('share/' + package_name, ['launch/verify_sim.launch.xml']),
        ('share/' + package_name, ['launch/launch_sim.launch.xml']),
        ('share/' + package_name, ['launch/launch_sim_controller.launch.xml']),
        ('share/' + package_name, ['launch/launch_multiple.launch.xml']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ananya Agarwal',
    maintainer_email='ananyaagarwal2024@u.northwestern.edu',
    description='Controllers for quadrotor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_pid = quad_control.controller_node:main',
            'joystick = quad_control.joystick:main',
            'record = quad_control.record:main',
            'drone_control = quad_control.drone_control:main',
            "controller_min = quad_control.controller_min_snap:main",
        ],
    },
)
