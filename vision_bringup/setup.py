from setuptools import find_packages, setup

package_name = 'vision_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='loe',
    maintainer_email='ldm92834334@gmail.com',
    description='This package is about vision module that process vision data',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pcd_process = vision_bringup.pcd_processs:main',
            'yolo_process = vision_bringup.yolo_process:main',
        ],
    },
)
