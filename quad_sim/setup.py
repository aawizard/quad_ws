from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'quad_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'dynamics'), glob('dynamics/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ananya Agarwal',
    maintainer_email='ananyaagarwal2024@u.northwestern.edu',
    description='Simulation Environment for quadrotor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim = quad_sim.sim:main',
        ],
    },
)
