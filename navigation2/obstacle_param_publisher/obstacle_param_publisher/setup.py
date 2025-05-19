from setuptools import setup, find_packages  # ✅ find_packages 추가

package_name = 'obstacle_param_publisher'

setup(
    name='obstacle_param_publisher',
    version='0.0.0',
    packages=find_packages(),  # ✅ 수정
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'obstacle_param_publisher']),
        ('share/obstacle_param_publisher', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Parameter publisher for obstacle info',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_param_publisher = obstacle_param_publisher.obstacle_param_publisher:main',  # ✅ 유지
            'keyboard_obstacle_publisher = obstacle_param_publisher.keyboard_obstacle_publisher:main', # ✅ 추가
            'lidar_processor = obstacle_param_publisher.lidar_processor:main',  # ✅ 추가
            'obstacle_integration_node = obstacle_param_publisher.obstacle_integration_node:main',  # ✅ 추가
        ],
    },
)
