from setuptools import find_packages, setup

package_name = "ida_logging"

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
    description="Data logging nodes (telemetry, video, map) for IDA autonomy.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "telemetry_logger_node = ida_logging.telemetry_logger_node:main",
            "video_logger_node = ida_logging.video_logger_node:main",
            "map_logger_node = ida_logging.map_logger_node:main",
            "events_logger_node = ida_logging.events_logger:main",
            "logging_status_node = ida_logging.status_node:main",
        ],
    },
)
