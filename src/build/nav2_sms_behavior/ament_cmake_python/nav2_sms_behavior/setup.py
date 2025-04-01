from setuptools import find_packages
from setuptools import setup

setup(
    name='nav2_sms_behavior',
    version='1.0.0',
    packages=find_packages(
        include=('nav2_sms_behavior', 'nav2_sms_behavior.*')),
)
