from setuptools import find_packages, setup

package_name = "ida_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="IDA Autonomy Team",
    maintainer_email="team@example.com",
    description="Real perception nodes (YOLO camera + RPLidar) for IDA autonomy.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_camera_node = ida_perception.yolo_camera_node:main",
            "lidar_node = ida_perception.lidar_node:main",
            "sllidar_bridge_node = ida_perception.sllidar_bridge_node:main",
        ],
    },
)
