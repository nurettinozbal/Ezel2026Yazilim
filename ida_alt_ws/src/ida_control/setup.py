from setuptools import find_packages, setup

package_name = "ida_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "mavsdk", "pymavlink", "pyserial"],
    zip_safe=True,
    maintainer="IDA Autonomy Team",
    maintainer_email="team@example.com",
    description="Command limiting and MAVSDK bridge nodes for IDA autonomy.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "command_limiter_node = ida_control.command_limiter_node:main",
            "mavsdk_bridge_node = ida_control.mavsdk_bridge_node:main",
            "tty_mavlink_router = ida_control.tty_mavlink_router:main",
            "gazebo_pose_sync_node = ida_control.gazebo_pose_sync_node:main",
        ],
    },
)
