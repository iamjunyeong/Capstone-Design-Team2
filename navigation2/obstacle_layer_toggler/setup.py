from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'obstacle_layer_toggler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
         (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # If you add launch files later
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='ROS 2 node to toggle obstacle layer observation sources based on obstacle info.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'toggle_obstacle_layer = obstacle_layer_toggler.toggle_obstacle_layer:main'
        ],
    },
)
