from setuptools import setup
from glob import glob

package_name = 'nav2_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [
            'nav2_params.yaml',
            'remote_nav2_params.yaml',
        ]),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linaro',
    maintainer_email='linaro@local',
    description='Nav2 helpers for AgiBot D1',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_building_node = nav2_tools.map_building_node:main',
            'pcd2pgm = nav2_tools.pcd2pgm:main',
            'nav_scan_node = nav2_tools.nav_scan_node:main',
        ],
    },
)
