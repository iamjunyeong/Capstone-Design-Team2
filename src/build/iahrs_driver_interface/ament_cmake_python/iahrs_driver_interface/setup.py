from setuptools import find_packages
from setuptools import setup

setup(
    name='iahrs_driver_interface',
    version='0.0.0',
    packages=find_packages(
        include=('iahrs_driver_interface', 'iahrs_driver_interface.*')),
)
