from setuptools import setup

package_name = "iahrs_driver"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="varofla",
    maintainer_email="dhksrl0508@naver.com",
    description="ROS 2 Driver for iAHRS RB-SDA-v1",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["driver = iahrs_driver.driver:main"],
    },
)
