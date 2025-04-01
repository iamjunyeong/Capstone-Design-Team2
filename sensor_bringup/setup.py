from setuptools import find_packages, setup

package_name = 'sensor_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', [
            'launch/imu.launch.py',
            'launch/rtk_gps.launch.py',
            'launch/sensor_bringup.launch.py'
            'launch/ublox_gps_node-launch.py',
        ]),
        #('share/' + package_name + '/config', ['config/imu.yaml', 'config/gps.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jun',
    maintainer_email='junyeong4321@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
