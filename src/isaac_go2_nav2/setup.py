from glob import glob
from pathlib import Path
from setuptools import setup


package_name = "isaac_go2_nav2"


def collect_files(root):
    paths = []
    for path in Path(root).rglob("*"):
        if path.is_file():
            paths.append(str(path))
    return paths


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/docs", collect_files("docs")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Open-source contributors",
    maintainer_email="maintainers@example.invalid",
    description="Navigation integration for the 'steel coin' AgiBot D1 quadruped (Mid-360S + Nav2). Fork of Go2W-navigate-project isaac_go2_nav2.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cmd_vel_limiter = isaac_go2_nav2.cmd_vel_limiter:main",
            "external_pose_map_corrector = isaac_go2_nav2.external_pose_map_corrector:main",
            "fastlio_nav2_odom_filter = isaac_go2_nav2.fastlio_nav2_odom_filter:main",
            "map_scan_simulator = isaac_go2_nav2.map_scan_simulator:main",
            "odom_to_imu = isaac_go2_nav2.odom_to_imu:main",
            "odom_tf_broadcaster = isaac_go2_nav2.odom_tf_broadcaster:main",
            "send_goal = isaac_go2_nav2.send_goal:main",
            "stage2_pointcloud_to_laserscan = isaac_go2_nav2.stage2_pointcloud_to_laserscan:main",
        ],
    },
)
