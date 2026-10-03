from setuptools import find_packages, setup

package_name = 'lio_tf_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='linaro',
    maintainer_email='linaro@example.com',
    description='Bridge FAST-LIO2 Odometry to Nav2 odom frame with lidar-to-base extrinsic.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'lio_tf_bridge = lio_tf_bridge.bridge_node:main',
        ],
    },
)
