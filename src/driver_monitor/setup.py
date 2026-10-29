from glob import glob
from setuptools import find_packages, setup


package_name = "driver_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/static", glob("static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rpi-can",
    maintainer_email="rpi-can@example.com",
    description="Driver monitoring, warning, web streaming and SocketCAN nodes.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_node = driver_monitor.camera_node:main",
            "driver_monitor_node = driver_monitor.driver_monitor_node:main",
            "warning_manager_node = driver_monitor.warning_manager_node:main",
            "socketcan_bridge_node = driver_monitor.socketcan_bridge_node:main",
            "web_stream_node = driver_monitor.web_stream_node:main",
        ],
    },
)
