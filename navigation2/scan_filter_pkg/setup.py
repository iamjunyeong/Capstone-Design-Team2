from setuptools import setup

package_name = 'scan_filter_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Filter LaserScan in a given angle range',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_filter_node = scan_filter_pkg.scan_filter_node:main',
        ],
    },
)
