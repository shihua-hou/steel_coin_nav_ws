from setuptools import setup

package_name = "oem700_rtk"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/99-oem700-rtk.rules",
            "config/oem700_cors.yaml.example",
        ]),
        ("share/" + package_name + "/launch", ["launch/oem700_rtk.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="steel_coin",
    maintainer_email="maintainers@example.invalid",
    description="OEM700 RTK driver and map-frame fusion for the steel-coin dog.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "oem700_driver = oem700_rtk.driver_node:main",
            "rtk_to_map = oem700_rtk.rtk_to_map:main",
            "global_pose_arbiter = oem700_rtk.global_pose_arbiter:main",
        ],
    },
)
