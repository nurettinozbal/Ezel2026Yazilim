from setuptools import find_packages
from setuptools import setup

setup(
    name='idaws_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('idaws_msgs', 'idaws_msgs.*')),
)
