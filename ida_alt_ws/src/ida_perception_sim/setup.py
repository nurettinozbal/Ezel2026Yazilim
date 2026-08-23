from setuptools import find_packages, setup

package_name = "ida_perception_sim"

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
    description="Scenario-based perception simulator for IDA autonomy development.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "perception_sim_node = ida_perception_sim.perception_sim_node:main",
        ],
    },
)
