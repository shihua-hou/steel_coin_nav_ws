import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'genisom_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linaro',
    maintainer_email='linaro@rk3588s-ubuntu',
    description='ROS2 bridge for AgiBot D1 Edu Ultra (ZSL-1) via mc_sdk HighLevel API',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'genisom_bridge = genisom_bridge.bridge_node:main',
        ],
    },
)
