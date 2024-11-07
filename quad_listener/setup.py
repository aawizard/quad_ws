from setuptools import find_packages, setup

package_name = 'quad_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config/pose_offsets.yaml']),
        ('share/' + package_name, ['launch/listener.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aawizard',
    maintainer_email='agwananyaa2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "quad_listener = quad_listener.listener_node:main",
        ],
    },
)
