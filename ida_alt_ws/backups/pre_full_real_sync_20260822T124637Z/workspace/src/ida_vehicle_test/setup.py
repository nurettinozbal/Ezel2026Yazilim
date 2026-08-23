from setuptools import find_packages, setup

package_name = "ida_vehicle_test"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "websockets>=10.4,<14"],
    zip_safe=True,
    maintainer="IDA Autonomy Team",
    maintainer_email="team@example.com",
    description="Passive and actuation-free ROS 2 vehicle test monitor.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vehicle_test_monitor = ida_vehicle_test.monitor_node:main",
            "vehicle_test_producer = ida_vehicle_test.producer_node:main",
        ],
    },
)
