from setuptools import find_packages
from setuptools import setup

setup(
    name='ublox_msgs',
    version='2.3.0',
    packages=find_packages(
        include=('ublox_msgs', 'ublox_msgs.*')),
)
