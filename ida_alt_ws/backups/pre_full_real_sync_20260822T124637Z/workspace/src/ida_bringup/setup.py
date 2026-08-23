import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ida_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "scenarios"), glob("scenarios/*.yaml")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        # Launch dosyaları bu yardımcıları FindPackageShare altından
        # çalıştırır; source tree'de bulunmaları kurulu workspace için
        # yeterli değildir.
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="IDA Autonomy Team",
    maintainer_email="team@example.com",
    description="Launch and configuration files for IDA autonomy.",
    license="MIT",
)
