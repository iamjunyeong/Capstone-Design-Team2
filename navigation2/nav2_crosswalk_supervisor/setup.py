from setuptools import setup
package_name = 'nav2_crosswalk_supervisor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'crosswalk_supervisor = '
            f'{package_name}.crosswalk_supervisor:main',
        ],
    },
)
