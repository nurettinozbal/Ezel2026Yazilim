from setuptools import find_packages, setup

package_name = "ida_uav_target"

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
    description="UAV target-color decision node for Parkur 3.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "uav_target_node = ida_uav_target.uav_target_node:main",
        ],
    },
)
